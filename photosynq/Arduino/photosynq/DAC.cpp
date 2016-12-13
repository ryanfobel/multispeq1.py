//  This is used to set LED intensity
//  Do not write to the DACs or ldac lines anywhere else.
//  Any calls from outside this file assume that pins are numbered 1-x (vs 0-x).
//  *** Important: assume that the DAC ICs have i2C addresses of 1,2,3 (can be set with a routine below)
//  Jon Zeeff, March, 2016

#include "utility/mcp4728.h"              // DAC
#include "DAC.h"
#include "serial.h"
#include "defines.h"

static mcp4728 *dac[NUM_DACS];    // pointer only - initialize under more controlled circumstances so debugging is possible

// map LED number (here numbered 0-9) to which DAC IC that pin is on
static const short LED_to_dac[NUM_LEDS] = {0, 0, 0, 2, 0, 1, 2, 1, 1, 1};  // must be 0,1,2

// map pin number (eg, 0-9) to which DAC channel that pin is on
static const short LED_to_channel[NUM_LEDS] = {2, 3, 0, 0, 1, 1, 1, 3, 2, 0};   // must be 0,1,2,3

static unsigned short prev_value[NUM_LEDS];  // store prev value so we don't waste time

// initialize the DACs

int DAC_init(void)
{
  // set up ldac pins
  pinMode(LDAC1, OUTPUT);
  digitalWriteFast(LDAC1, HIGH);
  pinMode(LDAC2, OUTPUT);
  digitalWriteFast(LDAC2, HIGH);
  pinMode(LDAC3, OUTPUT);
  digitalWriteFast(LDAC3, HIGH);

  // initialize DACs
  for (int i = 0; i < NUM_DACS; ++i) {
    if (dac[i] == 0)
      dac[i] = new mcp4728(i + 1);            // create it - avoid using address 0 (the default chip address)
    dac[i]->setVref(1, 1, 1, 1);
    dac[i]->setGain(0, 0, 0, 0);
    for (int j = 0; j < 4; ++j)               // set all initial values to zero
      dac[i]->analogWrite(j, 0);
  } // for

  DAC_change();  // cause changes to take effect

  // reset previous values
  for (int i = 0; i < NUM_LEDS; ++i)
    prev_value[i] = 6666;            // impossible value
  return 0;
  
}  // DAC_init()


// shutdown the DAC to save power
void DAC_shutdown()
{
 // set up ldac pins
  pinMode(LDAC1, INPUT);
  pinMode(LDAC2, INPUT);
  pinMode(LDAC3, INPUT);   
}

// set the DAC value for a particular LED (eg, 1-10)
// doesn't take effect until DAC_change()
// value is 0-4095 (12 bits)

void DAC_set(unsigned int led, unsigned int value)
{ 
  if (led ==  0)                                            // if you get a zero, quietly skip it
    return;

  assert(led > 0 && led <= NUM_LEDS);                       // any other wrong value is a fatal error

  led -= 1;   // convert to 0-x numbering

  assert(value <= 4095);

  // for readability, break these out
  int dac_number = LED_to_dac[led];
  int dac_channel = LED_to_channel[led];

  //    Serial.print(dac_number);
  //    Serial.print(",");
  //    Serial.println(dac_channel);

  if ((unsigned short) value != prev_value[led])                           // save time if already set
    dac[dac_number]->analogWrite(dac_channel, value);    // set value on DAC

  prev_value[led] = (unsigned short) value;

}  // DAC_set()

// Cause DAC changes to take effect
// This allows many changes to happen at the same time

void DAC_change(void)
{
  
  // toggle all ldac lines for 1 usecond
  digitalWriteFast(LDAC1, LOW);
  digitalWriteFast(LDAC2, LOW);
  digitalWriteFast(LDAC3, LOW);
  delayMicroseconds(1);
  digitalWriteFast(LDAC1, HIGH);
  digitalWriteFast(LDAC2, HIGH);
  digitalWriteFast(LDAC3, HIGH);
} // DAC_change()


// re-address a dac IC

int DAC_set_address(int ldac_pin, unsigned oldAddress, unsigned newAddress)           // ldac pin and address to assign (0-7)
{
  int ret = 0;

  if (newAddress < 0 || newAddress > 7) return -1;   // invalid new address
  if (oldAddress < 0 || oldAddress > 7) return -1;   // invalid old address

  Serial.printf("\n*** readdress dac using ldac %d from %d to %d\n", ldac_pin, oldAddress, newAddress);

  // set up LDAC pin
  pinMode(ldac_pin, OUTPUT);
  delay(10);
  digitalWriteFast(ldac_pin, HIGH);   // normally high

#define SET_ADDRESS
#ifdef SET_ADDRESS

  Wire.resetBus();
  Wire.setRate(I2C_RATE_100);   // slower makes timing possible
  delay(500);

  // This command is valid only when the LDAC pin makes
  // a transition from “High” to “Low” at the low time of the
  // last bit (8th clock) of the second byte, and stays “Low”
  // until the end of the third byte.

  // bytes to send (they just go into a buffer)
  Wire.beginTransmission((unsigned char)(0B01100000 | (oldAddress << 0)));            // 7 bit address  0xC0
  Wire.send(             (unsigned char)(0B01100001 | (oldAddress << 2)));            // LDAC should go low at the end of this byte, 0x61
  Wire.send(             (unsigned char)(0B01100010 | (newAddress << 2)));      // 0x66 for addr = 1
  Wire.send(             (unsigned char)(0B01100011 | (newAddress << 2)));      // 0x67 for addr = 1


  Serial.printf("sending %x %x %x %x\n",
                0B11000000 | (oldAddress << 0),
                0B01100001 | (oldAddress << 2),
                0B01100010 | (newAddress << 2),
                0B01100011 | (newAddress << 2));

  // send it
  noInterrupts();                   // timing is critical
  Wire.sendTransmission();
  //Wire.endTransmission();

  // set LDAC low at exactly the right time
  // use a scope or logic analzer to verify against clock and data lines
  // could also poll the clock pin for 17 falling edges

#define USE_DELAY
#ifdef USE_DELAY
  // this one is tested and works
  delayMicroseconds(-1 + 10 * 18);      // 10 usec per bit @ 100 khz i2c bus speed
#else
  // not sure if this will work
  for (int i = 0; i < 1 + 17; ++i) {               // number of falling edges
    while (digitalReadFast(SCL_PIN) != 1) {}     // wait for high
    while (digitalReadFast(SCL_PIN) != 0) {}     // wait for low (falling edge)
  } // for
  delayMicroseconds(1);                 // wait for falling edge to stabilize
#endif

  digitalWriteFast(ldac_pin, LOW);           // set LDAC low

  // evidently LDAC can stay low for the remainder - doesn't matter when it rises
  //delayMicroseconds(0 + 10 * 9);      // 10 usec per bit @ 100 khz i2c bus speed
  //digitalWriteFast(ldac_pin,  HIGH);

  // finish up
  interrupts();

  ret = Wire.finish();
  if (ret != 1)
    Serial.println("re-address failed");
  else
    Serial.println("re-address looked ok");


  delay(1000);  // wait for eeprom write
  digitalWriteFast(ldac_pin,  HIGH); // back to normal state
#endif

#define TEST_CHANGE2
#ifdef TEST_CHANGE2

  // just send a reset command to see if it is there

  Wire.beginTransmission((unsigned char)(0B01100000 | (newAddress << 0)));            // 7 bit address  0xC0
  Wire.send((unsigned char)0B00000110);          // reset DAC command
  int ret2 = Wire.endTransmission();

  if (ret2 == 0)
    Serial.printf("Success, a device is on new address %d\n", newAddress);

#endif

  Wire.setRate(I2C_RATE_400);     // back to normal speed

  return ret;

}  // DAC_set_address()

