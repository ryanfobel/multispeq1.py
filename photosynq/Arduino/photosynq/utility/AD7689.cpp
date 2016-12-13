
// routines to support reading from the ADC7689 16 bit SPI ADC
// Jon Zeeff 2016

#define FILTER 0                // Change this to 0 for no filtering or 1 for filtering - note, must also reduce sampling speed
#define AD7689_PIN 22		// CNV and chip select pin to use for SPI (10 is standard, 22 for photosynq 1.0)

#include <SPI.h>		// include the new SPI library:
#include "AD7689.h"

// test code
#if 0
#include "serial.h"

void
setup ()
{
  delay(1000);
  // initialize SPI:
  SPI.begin ();
  delay(1000);
}

void
loop ()
{

  //  delayMicroseconds(200);
  Serial.println(AD7689_read(1));            // read value with precise capture time
  Serial.print("temp = ");
  Serial.println(AD7689_read_temp());
  delay(2000);

} // loop()
#endif


//*************************************************

// AD7689 16 bit SPI A/D converter interface
// Supports accurate sample time
// Note: sampling/aquisition time is 1.8 usec
// Note: reference can drift +/- 10 ppm per degree C (16 counts from 25C to 0C)

// for debugging

#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)

// Serial.println(analogValue, BIN);  // print as an ASCII-encoded binary

// set up the speed, mode and endianness of each device
// MODE0: SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
SPISettings AD7689_settings (20000000, MSBFIRST, SPI_MODE0);

// Note: use CPHA = CPOL = 0
// Note: two dummy conversions are required on startup

static uint16_t ad7689_config;

// do conversion and return result - slow and imprecise

uint16_t
AD7689_read(int chan)
{
//  assert(chan >= 0 && chan < 9);

  // set up
  AD7689_set (chan);

  // do conversion
  AD7689_sample();

  delayMicroseconds(3);

  // read conversion result
  return AD7689_read_sample();
}


// use set, sample and read_sample for control over timing
// future enhancement - use the optional config parameter
// channels are numbered 0-7

void
AD7689_set (int chan, uint16_t config)
{
//  assert(chan >= 0 && chan < 9);

if (chan < 0 || chan > 8) {
	chan = 0;
}

  // bit shifts needed for config register values
#define CFG 13
#define INCC 10
#define INx 7
#define BW  6
#define REF 3
#define SEQ 1
#define RB 0

  // select channel and other config
  ad7689_config = 0;

  ad7689_config |= 1 << CFG;		// update config on chip

  if (chan == 8)
    ad7689_config |= 0B011 << INCC;	// temperature
  else {
    ad7689_config |= 0B111 << INCC;	// unipolar referenced to ground
    ad7689_config |= chan << INx;	  // channel
  }

  ad7689_config |= FILTER << BW;		    // 1 adds more filtering
  ad7689_config |= 0B000 << REF;	// use internal 2.5V reference
  //ad7689_config |= 0B011 << REF;	// use external reference (maybe ~3.3V)
  ad7689_config |= 0 << SEQ;		  // don't auto sequence
  ad7689_config |= 0 << RB;		    // don't read back config value

  ad7689_config = ad7689_config << 2;   // convert 14 bits to 16 bits

  //Serial.printf("ADC config: %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",BYTETOBINARY(config>>8), BYTETOBINARY(config));  // debug only

  pinMode (AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  SPI.beginTransaction (AD7689_settings);

  // send config (RAC mode)
  digitalWrite (AD7689_PIN, LOW);
  SPI.transfer (ad7689_config >> 8);	// high byte
  SPI.transfer (ad7689_config & 0xFF);	// low byte, 2 bits ignored
  digitalWrite (AD7689_PIN, HIGH);
  delayMicroseconds(6);

  // dummy
  digitalWrite (AD7689_PIN, LOW);
  SPI.transfer (ad7689_config >> 8);	// high byte
  SPI.transfer (ad7689_config & 0xFF);	// low byte, 2 bits ignored
  digitalWrite (AD7689_PIN, HIGH);
  delayMicroseconds(6);

  SPI.endTransaction ();

}

// save power
void
AD7689_shutdown ()
{
  pinMode (AD7689_PIN, INPUT);      // set to high impedance
}


// start the conversion - very time accurate, takes 1.8 usec

//inline void AD7689_sample() __attribute__((always_inline));

void
AD7689_sample()
{
  // do conversion
  digitalWriteFast(AD7689_PIN, LOW);         // chip select
  digitalWriteFast(AD7689_PIN, HIGH);        // chip deselect  (starts conversion)
}

// read the value - not very fast

uint16_t
AD7689_read_sample()
{
  delayMicroseconds(5);                                     // wait till conversion from AD7689_sample() is complete
  SPI.beginTransaction (AD7689_settings);
  digitalWriteFast (AD7689_PIN, LOW);                       // chip select
  uint16_t val = SPI.transfer (ad7689_config  >> 8) << 8;   // high byte
  val |= SPI.transfer (ad7689_config);	                    // low byte
  digitalWriteFast (AD7689_PIN, HIGH);                      // chip select
  delayMicroseconds(5);                                     // wait for unintended conversion to complete
  SPI.endTransaction ();

  return val;
}

// chip has an internal temp sensor - might be useful

uint16_t
AD7689_read_temp()
{
  AD7689_set (8);   // set to read temp
  AD7689_sample();
  // note: automatically switches back to reading inputs after this
  // need to divide this value by 250 to get close to an accurate temp (in degrees C)
  return AD7689_read_sample();
}



// read n samples as fast as possible into an array
// use AD7689_set() before calling this
// should be almost 250K samples/sec
// currently 222K

void AD7689_read_array(uint16_t array[], int num_samples)
{
//  assert(num_samples > 0 && num_samples < 4096);

  SPI.beginTransaction (AD7689_settings);    // set up SPI bus speed, etc

  // use Read/Write during conversion mode

  for (int i = -2; i < num_samples; ++i) {    // data is behind, so skip intial

    digitalWriteFast (AD7689_PIN, LOW);       // low so we can bring it high
    digitalWriteFast (AD7689_PIN, HIGH);      // chip deselect/CNV (starts conversion)
    digitalWriteFast (AD7689_PIN, LOW);       // chip select (so we can read/write to it)

    // note: read must complete before time tdata (1.2 usec from CNV high)
    // requires about a 16 mhz bus speed, check with scope

    register uint16_t val = SPI.transfer16 (ad7689_config);

    digitalWriteFast (AD7689_PIN, HIGH);      // CNV must be high at EOC

    if (i >= 0)
      array[i] = val;                         // account for skipped sample

    // wait for conversion to finish
    // chip is rated at 250K samples/sec (4 usec)
    // assume that the above time exceeded 1 usec
    delayMicroseconds(3 - 1);    // should be 4-1, lower values are EXPERIMENTAL

  } // for

  SPI.endTransaction ();

}  // AD7689_read_array()



// read n samples as fast as possible from two channels into two arrays
// chan parameter is numbered 0-7

void AD7689_read_arrays(int chan1, uint16_t array1[], int chan2, uint16_t array2[], int num_samples)
{
  // calc config values

  uint16_t ad7689_config1 = 0;
  ad7689_config1 |= 1 << CFG;           // update config on chip
  ad7689_config1 |= FILTER << BW;	// 1 adds more filtering
  ad7689_config1 |= 0B111 << INCC;      // unipolar referenced to ground
  ad7689_config1 |= chan1 << INx;       // channel
  ad7689_config1 = ad7689_config1 << 2; // convert 14 bits to 16 bits

  uint16_t ad7689_config2 = 0;
  ad7689_config2 |= 1 << CFG;           // update config on chip
  ad7689_config1 |= FILTER << BW;	// 1 adds more filtering
  ad7689_config2 |= 0B111 << INCC;      // unipolar referenced to ground
  ad7689_config2 |= chan2 << INx;       // channel
  ad7689_config2 = ad7689_config2 << 2; // convert 14 bits to 16 bits

  SPI.beginTransaction (AD7689_settings);     // set up SPI bus speed, etc

  for (int i = -2; i < num_samples; ++i) {    // data is behind, so skip intial
    uint16_t val;

    digitalWriteFast (AD7689_PIN, LOW);       // low so we can bring it high
    digitalWriteFast (AD7689_PIN, HIGH);      // chip deselect/CNV (starts conversion)
    digitalWriteFast (AD7689_PIN, LOW);       // chip select (so we can read/write to it)

    val = SPI.transfer16 (ad7689_config1);
    if (i >= 0)
      array1[i] = val;                        // account for skipped sample
    digitalWriteFast (AD7689_PIN, HIGH);      // must be high at EOC
    delayMicroseconds(4 - 1);                 // wait for conversion to complete

    digitalWriteFast (AD7689_PIN, LOW);       // low so we can bring it high
    digitalWriteFast (AD7689_PIN, HIGH);      // chip deselect/CNV (starts conversion)
    digitalWriteFast (AD7689_PIN, LOW);       // chip select (so we can read/write to it)

    val = SPI.transfer16 (ad7689_config2);
    if (i >= 0)
      array2[i] = val;                        // account for skipped sample
    digitalWriteFast (AD7689_PIN, HIGH);      // must be high at EOC
    delayMicroseconds(4 - 1);                 // wait for conversion to complete

  } // for

  SPI.endTransaction ();

}  // AD7689_read_array()


