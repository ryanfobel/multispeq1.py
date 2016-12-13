#include "Node.h"
#include "defines.h"            // various globals  
#include "json/JsonParser.h"
#include "DAC.h"
#include "utility/AD7689.h"     // external ADC
#include "eeprom.h"
#include "serial.h"
#include "flasher.h"
#include "utility/crc32.h"
#include <TimeLib.h>
#include "util.h"
#include "malloc.h"
#include <i2c_t3.h>
#include "SPI.h"

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

void Node::print_data()
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

namespace multispeq1 {

void Node::begin() {
#if !defined(DISABLE_SERIAL)
  // Start Serial after loading config to set baud rate.
  Serial.begin(115200);
#endif  // #ifndef DISABLE_SERIAL

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
}

}  // namespace multispeq1
