
#include <i2c_t3.h>


float MLX90615_Read(int TaTo);
uint16_t MLX90615_emissivity(double em);
double MLX90615_getEmissivity(void); 
static uint16_t MLX90615_getRawData(int TaTo);
int MLX90615_setAddress(uint8_t address);   // 0x5D is typical
uint16_t MLX90615_getAddress(void);


int MLX90615_address = 0x5B;  // 0x5B is default, be sure to use it initially to re-program

#define TEST 0
#if TEST

void setup() {
  // put your setup code here, to run once:

  Wire.begin ();                // start of the i2c protocol

  Wire.pinConfigure(I2C_PINS_18_19, I2C_PULLUP_INT);  // no need for external pullup resistors
  Wire.setRate(I2C_RATE_100);

  // teensy 3.1 pin 18 is SDA0
  // pin 19 is SCL

  Serial.begin (9600);          // start serial for output

  delay(1000);
  Serial.println("start MLX90615");
  delay(500);

  MLX90615_address = 0x5d;
  
  MLX90615_emissivity(0.0);  // erase first
  delay(500);
  MLX90615_emissivity(.97);  // set
  delay(500);

  Serial.print("emiss is:");
  Serial.println(MLX90615_getEmissivity());
  
  delay(500);
  //MLX90615_setAddress(0x0);  // erase
  delay(500);
  //int i = MLX90615_setAddress(0x5d);  // set

  delay(500);
  Serial.printf("addr: 0x%x\n",MLX90615_getAddress());

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(MLX90615_Read(0));
  delay(1000);

}

#endif

float MLX90615_Read(int TaTo) {
  int rawData = MLX90615_getRawData(TaTo);
  double tempData = (rawData * 0.02);
  tempData -= 273.15;

  return tempData;
}

static uint16_t MLX90615_getRawData(int TaTo) {
  // Store the two relevant bytes of data for temperature
  byte dataLow = 0x00;
  byte dataHigh = 0x00;
  
  Wire.beginTransmission(MLX90615_address);
  if (TaTo)
    Wire.send((uint8_t)0x26); //measure ambient temp
  else
    Wire.send((uint8_t)0x27); // measure objec temp
  Wire.endTransmission(I2C_NOSTOP);

  Wire.requestFrom(MLX90615_address, 2);
  dataLow = Wire.read();
  dataHigh = Wire.read();
  Wire.endTransmission();
  int tempData = (((dataHigh & 0x007F) << 8) | dataLow);
  return tempData;
}

// CRC-8 with polynomial X8+X2+X1+1.

// Written by Nick Gammon
// 22 April 2011

#define byte uint8_t

static byte crc8(byte *addr, byte len)
{
  byte crc = 0;
  while (len--) {
    byte inbyte = *addr++;
    int i;
    for (i = 8; i; i--)
    {
      byte carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}  // end of crc8


// set emissivity
// JZ

uint16_t MLX90615_emissivity(double em)   // .965 is a typical value
{
  // setup
  const unsigned int value = (em * 16384) + .49;
  uint8_t values[4];
  values[0] = MLX90615_address << 1;
  values[1] = 0x13;
  values[2] = value & 0xff;   // LSB
  values[3] = value >> 8;     // MSB

  // write
  Wire.beginTransmission(MLX90615_address);
  Wire.send((uint8_t)0x13);             //set the eeprom register
  Wire.send((uint8_t)(value & 0xff));     //set the LSB
  Wire.send((uint8_t)(value >> 8));       //set the MSB
  Wire.send((uint8_t)crc8(values, 4));  //do write
  int i =  Wire.endTransmission();
  
  // Serial.printf("set emiss: ret %d, crc = %x\n",i,crc8(values, 4));
  
  return i;
}


double MLX90615_getEmissivity(void) {
  uint8_t dataLow = 0x00;
  uint8_t dataHigh = 0x00;
  
  Wire.beginTransmission(MLX90615_address);
  Wire.send((uint8_t)0x13);                   // emissivity register
  Wire.endTransmission(I2C_NOSTOP);

  Wire.requestFrom(MLX90615_address, 2);
  dataLow = Wire.read();
  dataHigh = Wire.read();
  Wire.endTransmission();

  //Serial.printf("raw emiss = %x %x\n",dataHigh, dataLow);
  
  uint16_t tempData = (((dataHigh & 0xFF) << 8) | dataLow);

  //Serial.printf("raw emiss = %x\n",tempData);
  
  return tempData / 16384.0;
}


// Change a MLX90615 from the default address of 0x5B to a user specified value (like 0x5C)
// This change is written to eeprom, so is permanent

int MLX90615_setAddress(uint8_t address)   // 0x5C is typical
{
 // setup
  uint8_t values[4];
  values[0] = 0; // MLX90615_address << 1;    //  7 bit default address + R/W bit
  values[1] = 0x10;                     // register 0 in eeprom
  values[2] = address;   // LSB
  values[3] = 0;     // MSB

  // write
  Wire.beginTransmission(0); // MLX90615_address);
  Wire.send((uint8_t)0x10);             //set the eeprom register
  Wire.send((uint8_t)address); //set the LSB
  Wire.send((uint8_t)0);       //set the MSB
  Wire.send((uint8_t)crc8(values, 4));  //do write
  
  int i = Wire.endTransmission();
  // Serial.printf("set addr: ret = %d, pec = %x\n",i,(uint8_t)crc8(values, 4));
  return i;
}  
  
uint16_t MLX90615_getAddress(void) {
  uint8_t dataLow = 0x00;
  uint8_t dataHigh = 0x00;
  
  Wire.beginTransmission(MLX90615_address);
  Wire.send((uint8_t)0x10);                   // address register
  Wire.endTransmission(I2C_NOSTOP);

  Wire.requestFrom(MLX90615_address, 2);
  dataLow = Wire.read();
  dataHigh = Wire.read();
  Wire.endTransmission();
  return (dataHigh << 8) | dataLow;
}    // MLX90615_setAddress()

