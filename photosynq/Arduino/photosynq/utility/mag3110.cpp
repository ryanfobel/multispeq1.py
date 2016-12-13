/*
  MAG3110 (i2c compass) read code
  
  JZ, May 2015
  note: returns units of tenths of a micro-tesla but is uncalibrated for offsets (ie, wildly off)
        value will always range +/- 20,000
        Earth's magnetic field is 25 to 65 micro-teslas or a range of 250-650 tenths.
*/

#include <i2c_t3.h>

#define ledPin 13

#define TEST 0
#if TEST

void
setup()
{
  Serial.begin (9600);		// start serial for output
  
  delay(1000);
  
  pinMode(ledPin, OUTPUT);
  
  Wire.begin();		// join i2c bus (address optional for master)
//  Wire.pinConfigure(I2C_PINS_18_19, I2C_PULLUP_INT);  // no need for external pullup resistors
  delay(100);

  if (MAG3110_init() != 0) 		// turn the MAG3110 on
      Serial.println ("MAG3110 not detected");
  else
      Serial.println ("MAG3110 detected");
   
  Serial.printf("Relative temp = %d\n",(signed char)MAG3110_temp());
  
  // teensy 3.1 pin 18 is SDA0
  // pin 19 is SCL
}

void
loop()
{
  delay (100);
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(100);
  digitalWrite(ledPin, LOW);   // set the LED on
  
  delay(1000); 
  MAG3110_print_values();
}

#endif

#define MAG_ADDR  0x0E    // 7-bit address for the MAG3110, doesn't change

int
MAG3110_who_am_i (void)
{
  Wire.beginTransmission (MAG_ADDR);  // transmit to device 0x0E
  Wire.write (0x07);            // x MSB reg
  Wire.endTransmission(I2C_STOP, 1000); // stop transmitting

  delayMicroseconds (2);          // needs 1.3us free time between start and stop

  Wire.requestFrom (MAG_ADDR, 1); // request 1 byte
  delay(2);
  return (unsigned char)Wire.read();    // read the byte
}


// note: normally a compass chip is calibrated with offsets learned from (max-min)/2 as you rotate it
// this chip can apply these offsets


int
MAG3110_init (void)
{
  delay(1);

  Wire.beginTransmission (MAG_ADDR);	// transmit to device 0x0E
  Wire.write (0x10);		        // cntrl register1
  Wire.write (0B00101001);              // some oversampling, active mode
  Wire.write (0B10100000);		// reg 2 - enable auto resets and raw
  Wire.endTransmission(I2C_STOP, 1000);

  delay(1);

  if (MAG3110_who_am_i() != 0XC4)
     return -1;                         // error
  else
     return 0;                          // chip detected
}

// return temp in degrees C.  Note, uncalibrated offset is way off, so use only as a relative reading

int
MAG3110_temp(void)
{
  Wire.beginTransmission (MAG_ADDR);	// transmit to device 0x0E
  Wire.write (0x0f);		        // register for temp 
  Wire.endTransmission(I2C_STOP, 1000);	// stop transmitting

  delayMicroseconds (2);	        // needs 1.3us free time between start and stop

  Wire.requestFrom (MAG_ADDR, 1);	// request 1 byte
  delay(2);
  return (signed char)Wire.read();	// read the byte
}


// read all three axis

void
MAG3110_read (int *x,int *y,int *z)
{
  Wire.beginTransmission (MAG_ADDR);    // transmit to device 0x0E
  Wire.write (1);                       // starting register is 0x01
  Wire.endTransmission(I2C_STOP, 1000); // stop transmitting

  delayMicroseconds (2);                // needs 1.3us free time between start and stop

  Wire.requestFrom (MAG_ADDR, 6);       // request 6 bytes (all 3 values)

  unsigned char high_byte=0, low_byte=0;    // MSB and LSB

  *x = *y = *z = 0;

  if (Wire.available())
     high_byte = Wire.read();           // read the byte
  if (Wire.available())
     low_byte = Wire.read();            // read the byte
  *x = (int16_t) (low_byte | (high_byte << 8));    // concatenate the MSB and LSB

  if (Wire.available())
     high_byte = Wire.read();           // read the byte
  if (Wire.available())
     low_byte = Wire.read();            // read the byte
  *y = (int16_t) (low_byte | (high_byte << 8));    // concatenate the MSB and LSB

  if (Wire.available())
     high_byte = Wire.read();           // read the byte
  if (Wire.available())
     low_byte = Wire.read();            // read the byte
  *z = (int16_t) (low_byte | (high_byte << 8));    // concatenate the MSB and LSB

} // MAG3110_read()

void
MAG3110_print_values (void)
{
  int x,y,z;

  MAG3110_read (&x,&y,&z);
  Serial.printf (F("compass: x,y,z=%d %d %d\n"),x,y,z);
}

