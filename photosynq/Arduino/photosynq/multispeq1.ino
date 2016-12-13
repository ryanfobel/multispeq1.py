
// Firmware for MultispeQ 1.0 hardware.   Part of the PhotosynQ project.

// setup() - initize things on startup
// main loop is in loop.cpp

/*

prepare for bluetooth classic
changed baud rate to 115200 from 57600
changed packet_mode to 0
added bluetooth_configure command to 1009+ or "bluetooth_configure"
removed device_manufacture
set #define BLE_DELAY  to 0 (no delay between packets)

  add watchdog - need to know max delays
  // update DAC and get lights working in [{}]
  // once lights work, comparison test old and new adc routines, with timing
  //
  change 0,1 to before/after for environmentals
  consider adding _raw option as "raw" option in json

  RTC doesn't work
  X test get_device_info - WORKS!  But set to a string (currently it's a long)
  test sensors in "environmental" make sure averages works as it should - WORKS!
  test par_to_dac an light_intensity_raw_to_par
  so the rule is whatever the last value you took (could have been this protocol, or two protocols ago...) that's the one you get when you call an expression variable (light_intensity for example)... values do not get deleted between protocols.
  they only get reset between measurements.
  maybe consolidate all sensor measurements into PAR.cpp and call it sensors (?) - they are all pretty similar in structure.  Also, probably should make those a structure so Jon can reference them as global variables in the expression
  finish IR baseline calibrations
  test + document 1030 = 1045
  make sure that the IR LEDs are calibrated to yint = 0 and slope = 1

  + test do we need to calibrate offsets (like we did with the betas?)

  Create API for read_userdef, save_userdev, and reset_eeprom, delete all other eeprom commands, clean up all 1000… calls.
  X Using bluetooth ID as ID -
  X Firmware needs api call to write code to eeprom (unique ID).
  Get rid of user_enter… and replace with new user enter, then delete main function
  Pull out the alert/confirm/prompt into a subroutine.
  If averages == 1, then print data in real time
  Reimplement Actinic background light… make sure to update (currently set to 13)
  reimplement print_offset and get_offset

  x Make the “environmental” a separate subroutine and pass the before or after 0,1 to it.

  I would like to set the micro-einstein level for the lights in my measurements rather than a raw (unitless) 12 bit value.

  Hardware
  Noise in detector - big caps help, but not completely.  ANy better ideas?
  Low DAC values (eg, LED5 read from main) cause much higher stdev
  Detector has a DC offset (which causes ADC to read zero)
  Pulse width vs detector output is non-linear - caused by the DC filter?

  x Switch to combined ISR for LED pulses (no glitches)

  Convert all possible into an array to make designing protocols more user friendly
  x turn pulse_distance and pulse_size → into an array

  x Greg - find suitable small magnet to embed, talk with Geoff

  Android to check for empty ID (if all 0s, or all 1s, then set api call to make unique ID == BLE mac address.
  Check protocol routines (produce error codes if fail):
  X Battery check: Calculate battery output based on flashing the 4 IR LEDs at 250 mA each for 10uS.  This should run just before any new protocol - if it’s too low, report to the user
   (greg) Overheat LED check: adds up time + intensity + pulsesize and length of pulses, and calculates any overages.  Report overages - do not proceed if over.  Also needs a shutoff which is available through a 1000 call.
   Syntax check: make sure that the structure of the JSON is correct, report corrupted
   x Jon - CRC check: so we expect CRC on end of protocol JSON, and check to make sure it’s valid.  Report corrupted
   LED intensity range check?  Ie, certain LEDs can only go up to a certain intensity

  Define and then code 1000+ calls for all of the sensors (for chrome app to call)
  Implement application of magnetometer/compass calibrations
  Sebastian - can we change the sensor read commands 1000 over to normal protocols - then you output as per normal?)
  Check with sebastian about adding comments to protocols (even the inventor of json thinks there is a place for them)
  Clean up the protocols - light intensity (make into a single 1000+ call, see old code to bring it in)
  Check to make sure “averages” works in the protocols
  Clean up the pulsesize = 0 and all that stuff… meas_intensity…
  Attach par sensor to USB-C breakout, make calibration routine.
  Look up and see why iOS doesn’t connect to BLE.
  Test the power down feature from the bluetooth.  Determine how long it takes when powered down to come back to life (from bluetooth).  Include auto power off in the main while loop - auto-off after 10 seconds.
  x Test BLE packet mode
  also do packet mode from Android to MS?
  Troubleshoot issues with bluetooth between protocols.

  Start documenting the commands + parameters to pass…
  And the eeprom commands

   Next to do:
   Add calibration commands back in
   consolidate commands to end up with:
   get rid of droop in dac values at <10.
   fix detector mv offset (prevents reading low light levels)

   First thing - go through eeprom.h and set all of the variables in top and bottom of file...
   expose only a portion to users via 1000+ commands

   read_userdef - option to get a single userdef or all userdefs.  include the long arrays (like calibration_slope) as well as the offset and other variables which are not currently saved as userdefs.  All saved values are saveable in get_userdef.  This should have a print option when someone wants to also print it, and a get all option to print all userdefs (as JSON follow existing structure).
   replaces get_calibration, get_calibration_userdef, call_print_calibration, print_sensor_calibration, print_offset, set_device_info

   save_userdef - saves a value to EEPROM.  The size of the saved array here is defined by the userdef number.  So maybe number 1- 20 are user definable (some doubles, some triples, some long arrays), then 21 - 60 are 'reserved' for us developers (ie we don't expose them). is device info, 2 - 45 are 2 arrays, 45 - 50 are 3 array, and 50 - 60 are 25 array (or something).  Should include a standard print option to show success (as JSON).
   replaces add_calibration, add_userdef, save_calibration_slope, save_calibration_yint, calibrate_offset

   reset_eeprom - option to reset all, or just reset the exposed user values (1 - 20)
   replaces reset_all

   Note: code for each function should have some intial comments describing what it does

   X Note: eeprom writing needs refactoring - eliminate all EEPROMxAnything()

   Here's the remaining commands in this file which should be moved.  I've organized them by function.  Actually this will be nice as some of them need to be renamed and cleaned up anyway.

  // Calibration commands
  add_calibration             // this adds a single element to the calibrations stored as an array (containing all of the LED pins)
  get_calibration             // this gets and prints to serial a saved calibration value
  print_cal                   // print a single calibration value
  add_userdef                 // this adds a set of userdefs (may be 2, 3, or 4 objects in length)
  get_calibration_userdef     // this gets and prints to serial a saved userdef value
  print_get_userdef           // this prints all of the saved userdef values
  call_print_calibration      // call or call + print all calibration values
  save_calibration_slope      // get rid of these - integrate into rest of normal save system
  save_calibration_yint       // get rid of these - integrate into rest of normal save system
  reset_all                   // reset all calibration values, or only all except the device info.

  // Calculation of PAR from PAR sensor, and from MultispeQ LEDs, and calibration of light sensor
  Light_Intensity
  calculate_intensity
  calculate_intensity_background
  lux_to_uE
  uE_to_intensity
  calibrate_light_sensor

  // Calculate frequency of the timers
  reset_freq

  // CoralspeQ related functions
  readSpectrometer
  print_data

  // Commands related to the detector offset
  calibrate_offset
  calculate_offset
  print_sensor_calibration
  print_offset

  // Device info command
  set_device_info

  NEXT STEPS:

  - CHANGE THE WAY WE UPTAKE THE ACTNINIC AND MEASURING LIGHTS - ACT_LIGHTS, MEAS_LIGHTS, DETECTORS 1/2/3/4, SEE HOW IT'S ITERPRETED IN THE CODE.

  a_lights : [[3,4,5],[3],[3]]
  a_intensities : [[254,450,900],[254],[254]]
  meas_lights : [[2,3],[2,3],[3]]
  meas_intensities[[500,100],[500,100],[100]]
  detectors : [[2, 1],[2, 1],[1]]
  pulses : [100,100,200]
  pulse_distance : [10000,10000,1000]
  pulse_size :  [10,100,10]
  message : [["alert":"alert message"],["prompt":"prompt message"],["0","0"]]


  A flat structure is also possible... where each pulse set is a separate object.  In that case I'd have to interpret each pulse set separately using the JSON interpret tool.  This takes time, and it's preferable to completely
  interpret the JSON prior to starting the measurement.  Interpreting it between pulses would likely cause delays and increase the shortest possible measurement pulse distance.
  The other option is to interpret each JSON and rebuild the arrays at the beginning of the measurement - this would be quite tedious and hard to do.

*/

/*
  ////////////////////// HARDWARE NOTES //////////////////////////

  The detector operates with an high pass filter, so only pulsing light (<100us pulses) passes through the filter.  Permanent light sources (like the sun or any other constant light) is completely
  filtered out.  So in order to measure absorbance or fluorescence a pulsing light must be used to be detectedb by the detector.  Below are notes on the speed of the measuring lights
  and actinic lights used in the MultispeQ, and the noise level of the detector:

  Optical:
  RISE TIME, Measuring: 0.4us
  FALL TIME, Measuring: 0.2us
  RISE TIME Actinic (bright): 2us
  FALL TIME Actinic (bright): 2us
  RISE TIME Actinic (dim): 1.5us
  FALL TIME Actinic (dim): 1.5us

  Electrical:
  RISE TIME, Measuring: .4us
  FALL TIME, Measuring: .2us
  RISE TIME Actinic: 1.5us
  FALL TIME Actinic: 2us
  NOISE LEVEL, detector 1: ~300ppm or ~25 detector units from peak to peak
  OVERALL: Excellent results - definitely good enough for the spectroscopic measurements we plant to make (absorbance (ECS), transmittance, fluorescence (PMF), etc.)

  //////////////////////  I2C Addresses //////////////////////
  TCS34725 0x29

*/

// includes
#include "defines.h"
#include <i2c_t3.h>
#include <Time.h>                   // enable real time clock library
#define EXTERN
#include "eeprom.h"
#include "serial.h"
#include <SPI.h>              
#include "util.h"
#include <TimeLib.h>

void setup_pins(void);          // initialize pins

// This routine is called first



void setup() 
{
  // turn on power and initialize ICs
  
  // Set up I2C bus - CAUTION: any subsequent calls to Wire.begin() will mess this up
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_800);  // using alternative wire library

  turn_on_3V3();                 // note: these routines already have delay() in them
  
//  delay(100);                   // let battery voltage stabilize - if there are no other delays ahead of it then this can be deleted.
  
  if (eeprom->sleep == 1) {     // sleep forever if requested
     store(sleep,0);            // but don't sleep after next reboot
     deep_sleep();
  }

  // set up serial ports (Serial and Serial1)
  Serial_Set(4);                // auto switch between USB and BLE
  Serial_Begin(115200);

  turn_on_5V();                  // LEAVE THIS HERE!  Lots of hard to troubleshoot problems emerge if this is removed.

  // set up MCU pins
  setup_pins();

  // initialize SPI bus
  SPI.begin ();

  eeprom_initialize();      // eeprom
  assert(sizeof(eeprom_class) < 2048);      // check that we haven't exceeded eeprom space

#if CORALSPEQ == 1
  // Set pinmodes for the coralspeq
  //pinMode(SPEC_EOS, INPUT);
  pinMode(SPEC_GAIN, OUTPUT);
  pinMode(SPEC_ST, OUTPUT);
  pinMode(SPEC_CLK, OUTPUT);

  digitalWrite(SPEC_GAIN, LOW);
  digitalWrite(SPEC_ST, HIGH);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_GAIN, HIGH); //High Gain
  //digitalWrite(SPEC_GAIN, LOW); //LOW Gain
#endif

  // ADC config
  analogReference(EXTERNAL);
  analogReadResolution(16);
  analogReadAveraging(4);
  { // check voltage level
    //uint32_t x = analogRead(39) >> 4;  // formula needs 12 bits, not 16
    //uint32_t mv = (178 * x * x + 2688757565 - 1184375 * x) / 372346; // milli-volts input to MCU, clips at ~3500
    //assert(mv > 3400);      // voltage is too low for proper operation
  }
  analogReference(INTERNAL);   // 1.20V

  setTime(Teensy3Clock.get());              // set time from RTC

  Serial_Print(DEVICE_NAME);                // note: this may not display because Serial isn't ready
  Serial_Print_Line(" Ready");

}  // setup() - now execute loop()


void setup_pins()
{
  // set up LED on/off pins
  for (unsigned i = 1; i < NUM_LEDS + 1; ++i)
    pinMode(LED_to_pin[i], OUTPUT);

  // pins used to turn on/off detector integration/discharge
  pinMode(HOLDM, OUTPUT);
  digitalWriteFast(HOLDM, HIGH);                  // discharge cap
  pinMode(HOLDADD, OUTPUT);
  digitalWriteFast(HOLDADD, HIGH);                // discharge cap
}


void unset_pins()     // save power, set pins to high impedance
{
  // turn off almost every pin
  for (unsigned i = 0; i < 33; ++i)
//     if (i != 18 && i != 19 && i != WAKE_DC && i != WAKE_3V3 && i != 0 && i != 1)  // leave I2C and power control on
        pinMode(i, INPUT);  
}


#if CORAL_SPEQ == 1

void readSpectrometer(int intTime, int delay_time, int read_time, int accumulateMode)
{
  /*
    //int delay_time = 35;     // delay per half clock (in microseconds).  This ultimately conrols the integration time.
    int delay_time = 1;     // delay per half clock (in microseconds).  This ultimately conrols the integration time.
    int idx = 0;
    int read_time = 35;      // Amount of time that the analogRead() procedure takes (in microseconds)
    int intTime = 100;
    int accumulateMode = false;
  */

  // Step 1: start leading clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 2: Send start pulse to signal start of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 3: Integration time -- sample for a period of time determined by the intTime parameter
  int blockTime = delay_time * 8;
  int numIntegrationBlocks = (intTime * 1000) / blockTime;
  for (int i = 0; i < numIntegrationBlocks; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }


  // Step 4: Send start pulse to signal end of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 5: Read Data 2 (this is the actual read, since the spectrometer has now sampled data)
  idx = 0;
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);

    // Analog value is valid on low transition
    if (accumulateMode == false) {
      spec_data[idx] = analogRead(SPEC_VIDEO);
      spec_data_average[idx] += spec_data[idx];
    } else {
      spec_data[idx] += analogRead(SPEC_VIDEO);
    }
    idx += 1;
    if (delay_time > read_time) delayMicroseconds(delay_time - read_time);   // Read takes about 135uSec

    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    // Second block of 2 clocks -- idle
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 6: trailing clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }
}

void print_data()
{
  Serial_Print("\"data_raw\":[");
  for (int i = 0; i < SPEC_CHANNELS; i++)
  {
    Serial_Print((int)spec_data[i]);
    if (i != SPEC_CHANNELS - 1) {               // if it's the last one in printed array, don't print comma
      Serial_Print(",");
    }
  }
  Serial_Print("]");
}
#endif
