#include <wiringPi.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <mosquitto.h>
#include <string.h>

// SETTINGS (YOU WILL PROBABLY NEED TO ADJUST THESE)
#define CIRCULATION_DURATION   360.0  // minutes (how long to circulate the water)
#define TIMER_WAIT_DURATION     15.0  // minutes (duration to wait between timer activation of the fire pump)
#define TIMER_PUMP_DURATION      5.0  // minutes (duration of pumping when the fire pump timer is active)
#define EXPECTED_FULL_DEPTH     48.0  // inches (set this slightly lower than the expected depth when the cistern is full)
#define MINIMUM_WATER_DEPTH     30.0  // inches (system goes out of service when water level is below this amount)
#define LOW_WATER_REFILL_AMOUNT  2.0  // inches (when a low water state is active, this is the amount that must be added to return to service, to prevent short-cycling)
#define PUBLISH_INTERVAL         0.25 // minutes (how often to publish data to MQTT)
#define NORMALLY_CLOSED_VALVE         // comment this line out if using a normally open valve

// GPIO PINOUT
//                             BCM pin            physical pin
#define MANUAL_ON_SWITCH_PIN      5             //     29         Triggers on HIGH (switched positive).
#define TIMER_SWITCH_PIN          6             //     31         Triggers on HIGH (switched positive).
#define WELL_PUMP_ACTIVATION_PIN 26             //     37         Triggers on LOW (switched negative) state to reduce the number of wires required in the control cable.
#define VALVE_PIN                22             //     15
#define PUMP_PIN                 27             //     13

// STUFF FOR I2C INTERFACE TO ADS1115 (PRESSURE SENSOR INTERFACE)
#define I2C_DEV "/dev/i2c-1"
#define ADS1115_ADDR 0x48
#define SAMPLE_COUNT 30

// HELPERS
#ifdef NORMALLY_CLOSED_VALVE
#define VALVE_CLOSED_STATE LOW
#define VALVE_OPEN_STATE HIGH
#else
#define VALVE_CLOSED_STATE HIGH
#define VALVE_OPEN_STATE LOW
#endif

double getWaterVolume(double waterDepth)
{
   // You need to modify this function based on the shape of your cistern.
   return (3.14159 * 35.5 * 35.5 * waterDepth + 44 * 71 * waterDepth) / 231.0; // US gallons. The shape is oval (half cylinder on each end plus rectangular cuboid in the middle).
}

enum pumpstate { IDLE, CIRCULATE_WAITING, CIRCULATE, FIREPUMP_TIMER_WAITING, FIREPUMP_TIMER_PUMPING, FIREPUMP_MANUAL, LOW_WATER };

static inline const char* stringFromPumpState(enum pumpstate ps)
{
   static const char *strings[] = { "IDLE", "CIRCULATE_WAITING", "CIRCULATE", "FIREPUMP_TIMER_WAITING", "FIREPUMP_TIMER_PUMPING", "FIREPUMP_MANUAL", "LOW_WATER" };
   return strings[ps];
}

bool isManualSwitchOn()
{
   return digitalRead(MANUAL_ON_SWITCH_PIN) == HIGH;
}

bool isTimerSwitchOn()
{
   return digitalRead(TIMER_SWITCH_PIN) == HIGH;
}

bool isWellPumpActivationSignal()
{
   return digitalRead(WELL_PUMP_ACTIVATION_PIN) == LOW;
}

void goIdle()
{
   digitalWrite(PUMP_PIN, LOW);
   usleep(500000); // reduce water hammer from the pump and water inertia
   digitalWrite(VALVE_PIN, LOW);
}

void goFirePump()
{
   digitalWrite(VALVE_PIN, VALVE_CLOSED_STATE);
   digitalWrite(PUMP_PIN, HIGH);
}

void goCirculate()
{
   digitalWrite(VALVE_PIN, VALVE_OPEN_STATE);
   digitalWrite(PUMP_PIN, HIGH);
}

void getTime(struct timespec *time)
{
   clock_gettime(CLOCK_REALTIME, time);
}

long int getElapsedTimeSeconds(struct timespec startTime)
{
   struct timespec now;
   getTime(&now);
   return now.tv_sec - startTime.tv_sec;
}

bool durationElapsed(struct timespec startTime, double durationMinutes)
{
   long int elapsedTimeSeconds = getElapsedTimeSeconds(startTime);
   return ((double)elapsedTimeSeconds >= durationMinutes * 60.0);
}

double getWaterDepthRaw()
{
   int file;
   if ((file = open(I2C_DEV, O_RDWR)) < 0)
   {
      perror("Failed to open I2C bus");
      return 0.0;
   }

   if (ioctl(file, I2C_SLAVE, ADS1115_ADDR) < 0)
   {
      perror("Failed to connect to ADS1115");
      return 0.0;
   }

   unsigned char config[3];
   config[0] = 0x01; // config register
   config[1] = 0xC2; // 1100 0010 -> OS:        Operational status/single-shot conversion start = 1 : Begin a single conversion (when in power-down mode)
                     //              MUX[2:0]:  Input multiplexer configuration                 = 100 : AINP = AIN0 and AINN = GND
                     //              PGA[2:0]:  Programmable gain amplifier configuration       = 001 : FS = ±4.096V  (This parameter expresses the full-scale range of the ADC scaling.)
                     //              MODE:      Device operating mode                           = 0 : Continuous conversion mode
   config[2] = 0x83; // 1000 0011 -> DR[2:0]:   Data rate                    = 100 : 128SPS (default)
                     //              COMP_MODE: Comparator mode              = 0   : Traditional comparator with hysteresis (default)
                     //              COMP_POL:  Comparator polarity          = 0   : Active low (default)
                     //              COMP_LAT:  Latching comparator          = 0   : Non-latching comparator (default)
                     //              COMP_QUE:  Comparator queue and disable = 11  : Disable comparator (default)

   if (write(file, config, 3) != 3)
   {
      perror("Failed to write config to ADS1115");
      return 0.0;
   }

   // Wait for analog to digital conversion to complete
   usleep(10000);

   unsigned char reg[1] = { 0x00 }; // conversion register
   if (write(file, reg, 1) != 1)
   {
      perror("Failed to select conversion register");
      return 0.0;
   }

   // Read 2 bytes from conversion register
   unsigned char data[2];
   if (read(file, data, 2) != 2)
   {
      perror("Failed to read conversion");
      return 0.0;
   }

   int16_t rawAdc = (data[0] << 8) | data[1];
   if (rawAdc < 0)
      rawAdc = 0;

   close(file);

   double depthInches = (double)(rawAdc - 3100) / 125.47;
   return depthInches;
}

double getMedian(double a, double b, double c)
{
   if ((b <= a && a <= c) || (c <= a && a <= b))
      return a;

   if ((a <= b && b <= c) || (c <= b && b <= a))
      return b;

   return c;
}

double signal[SAMPLE_COUNT];

double getWaterDepth()
{
   for (int i = 0; i < SAMPLE_COUNT; i++)
   {
      signal[i] = getWaterDepthRaw();
   }

   // apply a median filter to discard spiky outlier values
   double filteredSignalSum = 0.0;
   for (int i = 1; i < SAMPLE_COUNT - 1; i++)
   {
      filteredSignalSum += getMedian(signal[i - 1], signal[i], signal[i + 1]);
   }

   double average = filteredSignalSum / (double)(SAMPLE_COUNT - 2);
   return average;
}

void publishData(enum pumpstate state, double waterDepth, double waterVolume)
{
   mosquitto_lib_init();
   struct mosquitto *mosq = mosquitto_new(NULL, true, NULL);
   if (!mosq)
      perror("Failed to create mosquitto client");

   if (mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS)
      perror("Failed to connecting to MQTT broker");

   const char* stateString = stringFromPumpState(state);
   char waterDepthString[100];
   sprintf(waterDepthString, "%.2f", waterDepth);
   char waterVolumeString[100];
   sprintf(waterVolumeString, "%.0f", waterVolume);

   mosquitto_publish(mosq, NULL, "cistern/state",       strlen(stateString),       stateString,       0, true);
   mosquitto_publish(mosq, NULL, "cistern/waterDepth",  strlen(waterDepthString),  waterDepthString,  0, true);
   mosquitto_publish(mosq, NULL, "cistern/waterVolume", strlen(waterVolumeString), waterVolumeString, 0, true);

   mosquitto_destroy(mosq);
   mosquitto_lib_cleanup();
}


int main(void)
{
   wiringPiSetupGpio();
   pinMode        (MANUAL_ON_SWITCH_PIN, INPUT);    // pin mode ..(INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
   pullUpDnControl(MANUAL_ON_SWITCH_PIN, PUD_DOWN); // pull up/down mode (PUD_OFF, PUD_UP, PUD_DOWN) => down
   pinMode        (TIMER_SWITCH_PIN, INPUT);    // pin mode ..(INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
   pullUpDnControl(TIMER_SWITCH_PIN, PUD_DOWN); // pull up/down mode (PUD_OFF, PUD_UP, PUD_DOWN) => down
   pinMode        (WELL_PUMP_ACTIVATION_PIN, INPUT);
   pullUpDnControl(WELL_PUMP_ACTIVATION_PIN, PUD_UP);
   pinMode        (VALVE_PIN, OUTPUT);
   pinMode        (PUMP_PIN,  OUTPUT);

   enum pumpstate state = IDLE;
   goIdle();
   struct timespec circulationStartTime;
   struct timespec timerPumpingStartTime;
   struct timespec timerWaitingStartTime;
   struct timespec lastPublishTime;
   getTime(&lastPublishTime);

   while(1)
   {
      double waterDepth = getWaterDepth();

      if (waterDepth < (double)MINIMUM_WATER_DEPTH)
      {
         state = LOW_WATER;
         goIdle();
      }

      switch (state)
      {
         case IDLE:
         case CIRCULATE_WAITING:
         case CIRCULATE:
            if (isManualSwitchOn())
            {
               state = FIREPUMP_MANUAL;
               goFirePump();
            }
            else if (isTimerSwitchOn())
            {
               state = FIREPUMP_TIMER_PUMPING;
               goFirePump();
               getTime(&timerPumpingStartTime);
            }
            else if (state == IDLE && waterDepth < EXPECTED_FULL_DEPTH)
            {
               state = CIRCULATE_WAITING;
            }
            else if (state == CIRCULATE_WAITING && waterDepth > EXPECTED_FULL_DEPTH && !isWellPumpActivationSignal())
            {
               state = CIRCULATE;
               goCirculate();
               getTime(&circulationStartTime);
            }
            else if (state == CIRCULATE && (durationElapsed(circulationStartTime, CIRCULATION_DURATION) || (waterDepth < EXPECTED_FULL_DEPTH && isWellPumpActivationSignal())))
            {
               state = IDLE;
               goIdle();
            }
         break;

         case FIREPUMP_MANUAL:
            if (!isManualSwitchOn())
            {
               state = IDLE;
               goIdle();
            }
         break;

         case FIREPUMP_TIMER_WAITING:
            if (!isTimerSwitchOn())
            {
               state = IDLE;
               goIdle();
            }
            else if (durationElapsed(timerWaitingStartTime, TIMER_WAIT_DURATION))
            {
               state = FIREPUMP_TIMER_PUMPING;
               goFirePump();
               getTime(&timerPumpingStartTime);
            }
         break;

         case FIREPUMP_TIMER_PUMPING:
            if (!isTimerSwitchOn())
            {
               state = IDLE;
               goIdle();
            }
            else if (durationElapsed(timerPumpingStartTime, TIMER_PUMP_DURATION))
            {
               state = FIREPUMP_TIMER_WAITING;
               goIdle();
               getTime(&timerWaitingStartTime);
            }
         break;

         case LOW_WATER:
            if (waterDepth > (double)MINIMUM_WATER_DEPTH + LOW_WATER_REFILL_AMOUNT)
            {
               state = IDLE;
            }
         break;
      }

      if (durationElapsed(lastPublishTime, PUBLISH_INTERVAL))
      {
         double waterVolume = getWaterVolume(waterDepth);
         publishData(state, waterDepth, waterVolume);
         getTime(&lastPublishTime);
      }
   }

   return 0;
}
