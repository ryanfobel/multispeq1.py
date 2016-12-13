
// Global defines and some misc functions

#include <Arduino.h>
#include <stdint.h>
#include "utility/Adafruit_BME280.h"      // temp/humidity/pressure sensor

//#define DEBUG 1         // uncomment to add full debug features
const int DEBUGSIMPLE= 0;   // uncomment to add partial debug features
//#define DAC 1           // uncomment for boards which do not use DAC for light intensity control
const int PULSERDEBUG=0;   // uncomment to debug the pulser and detector
//#define NO_ADDON        // uncomment if add-on board isn't present (one missing DAC, etc)
#define CORAL_SPEQ 0

// new in 1.08 -->
// removed the 30 second auto-progress relating to open-close-start (now waits forever)
//

// FIRMWARE VERSION OF THIS FILE (SAVED TO EEPROM ON FIRMWARE FLASH)
#define DEVICE_FIRMWARE "1.11"
#define DEVICE_NAME "MultispeQ"
#define DEVICE_VERSION "1"
     
//////////////////////PIN DEFINITIONS AND TEENSY SETTINGS////////////////////////
//Serial, I2C, SPI...
#define RX       0        // serial port pins
#define TX       1

#define MOSI1    11       // SPI pins
#define MISO1    12

#define MOSI2    7
#define MISO2    8

#define SDA1     18       // I2C
#define SCL1     19

#define SCL2     29
#define SDA2     30

#define SS1      22
#define SCK1     13
#define SS2      9

// hall effect sensor (analog)
#define HALL_OUT 35

// Lights - map LED pin # to MCU pin #cd 
// 1-5 on main board, 6-10 on add-on board
// document colors
#define PULSE1   5
#define PULSE2   20
#define PULSE3   3
#define PULSE4   28      // was 10, now 28
#define PULSE5   4
#define PULSE6   24
#define PULSE7   27
#define PULSE8   26
#define PULSE9   25
#define PULSE10  23

// use this to store values to eeprom 
#define store(location, value)   { typeof(value) f = value;  if (eeprom->location != f) eeprom->location = f;  while (!(FTFL_FCNFG & FTFL_FCNFG_EEERDY)) {} }

// use to get the number of elements in an array
template <typename T, size_t N> 
char (&ArraySizeHelper(T (&array)[N]))[N];
#define arraysize(array) (sizeof(ArraySizeHelper(array)))

#define sleep_cpu()             {asm("wfi");}                 // go to lower power mode

// hash a string to an unsigned int AT COMPILE TIME, so it can be used in a switch statement
constexpr unsigned hash(const char *string)
{
 return *string == 0 ? 17325 : *string + (*string * hash(string+1));
}

const int NUM_LEDS=10;
// map LED (1-10 not 0-9) to MCU pin
const uint8_t LED_to_pin[NUM_LEDS + 1] = {0, PULSE1, PULSE2, PULSE3, PULSE4, PULSE5, PULSE6, PULSE7, PULSE8, PULSE9, PULSE10 }; // NOTE!  We skip the first element in the array so that the array lines up correctly (PULSE1 == 1, PULSE2 == 2 ... )

// bluetooth BLE (shared with IOEXTx)
#define DEBUG_DC 14   // could allow reflashing of BLE module
#define DEBUG_DD 32


// sample and hold (hold + release detector cap)
#define HOLDM    6        // output
#define HOLDADD 21

// peripheral USB 3.0 connector pins
#define DACT     40
#define ADCT     37
#define IOEXT1   14
#define IOEXT2   32

// power/battery management
// note: 5V and analog is used by: LEDs, detectors, external ADC, hall effect and peripheral connector

#define WAKE_DC   2            // output to power up 5V and analog power supplies, active high
#define WAKE_3V3  31           // output to turn on 3.3V, AKA 3.3V_ON, active low
#define BAT_MEAS  33           // output to activate battery voltage sampling, active low
#define BAT_TEST  A10          // input to measure battery voltage
#define USB_MEAS  A12          // input to measure USB voltage

#define REF_VOLTAGE 1.195
#define BAT_MIN_LOADED 3.4    // in volts, below this is too low
#define BAT_MIN 3.5
#define BAT_MAX 4.2

#define WAKE_TILT 10          // interrupt from accel to MK20

// blank pin (used when no other pin is selected - probably should change this later
//#define BLANK    32   // error - same as IOEXT2


//struct to hold tilt information
struct Tilt {
  float angle;
  String angle_direction;
};

// Functions
#include <stdint.h>
void activity(void);
void powerdown(void);
uint16_t median16(uint16_t array[], const int n, const float percentile = .50);
float stdev16(uint16_t array[], const int n);
int check_protocol(char *str);
void deep_sleep(void);


/*
 * Sensor-related variables which are accessed in loop
 * These values are accessible through the expr() function
 */

#ifndef EXTERN
#define EXTERN extern
#endif

EXTERN float light_intensity;
EXTERN float light_intensity_averaged;
EXTERN float light_intensity_raw;
EXTERN float light_intensity_raw_averaged;
EXTERN float r;
EXTERN float r_averaged;
EXTERN float g;
EXTERN float g_averaged;
EXTERN float b;
EXTERN float b_averaged;

EXTERN float thickness;
EXTERN float thickness_averaged;
EXTERN int thickness_raw;
EXTERN float thickness_raw_averaged;

EXTERN float contactless_temp;
EXTERN float contactless_temp_averaged;

EXTERN float compass;
EXTERN float compass_averaged;
EXTERN int x_compass_raw, y_compass_raw, z_compass_raw;
EXTERN float x_compass_raw_averaged, y_compass_raw_averaged, z_compass_raw_averaged;

EXTERN double angle;
EXTERN double angle_averaged;
EXTERN String angle_direction;
EXTERN float roll, pitch;
EXTERN float roll_averaged, pitch_averaged;
EXTERN int x_tilt, y_tilt, z_tilt;
EXTERN float x_tilt_averaged, y_tilt_averaged, z_tilt_averaged;

EXTERN float temperature, humidity, pressure;
EXTERN float temperature_averaged, humidity_averaged, pressure_averaged;

EXTERN float temperature2, humidity2, pressure2;
EXTERN float temperature2_averaged, humidity2_averaged, pressure2_averaged;

EXTERN float co2, co2_averaged;

EXTERN float detector_read1, detector_read2, detector_read3;
EXTERN float detector_read1_averaged, detector_read2_averaged,detector_read3_averaged;

EXTERN float analog_read, digital_read, adc_read, adc_read2, adc_read3;
EXTERN float analog_read_averaged, digital_read_averaged, adc_read_averaged, adc_read2_averaged, adc_read3_averaged;

// pressure/temp/humidity sensors
EXTERN Adafruit_BME280 bme1;        // I2C sensor
EXTERN Adafruit_BME280 bme2;

// Coral SpeQ
//////////////////////PIN DEFINITIONS FOR CORALSPEQ////////////////////////
#define SPEC_GAIN      28
//#define SPEC_EOS       NA
#define SPEC_ST        26
#define SPEC_CLK       25
#define SPEC_VIDEO     A10
//#define LED530         15
//#define LED2200k       16
//#define LED470         20
//#define LED2200K       2
#if CORAL_SPEQ
#define SPEC_CHANNELS    256
#else
#define SPEC_CHANNELS 1                  // save memory
#endif
EXTERN uint16_t spec_data[SPEC_CHANNELS];
EXTERN unsigned long spec_data_average[SPEC_CHANNELS];            // saves the averages of each spec measurement
EXTERN int idx;

#undef EXTERN
