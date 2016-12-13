
// read 3 axis values (in milli-Gs) from a MMA8653FC accelerometer chip
// based on code from hoelig
// modifications by Jon Zeeff, 2015

// Note: normally the device is spun in 3D circles to get a max and min for each axis and then each axis 
// is scaled -100 to +100.   Or a jig can be used to get calibration values.  Or trust the factory calibration.

#include <i2c_t3.h>

#define ledPin 13

#define TEST 0
#if TEST

void
setup ()
{
  Wire.begin ();		// start of the i2c protocol

  Wire.pinConfigure(I2C_PINS_18_19, I2C_PULLUP_INT);  // no need for external pullup resistors

  // teensy 3.1 pin 18 is SDA0
  // pin 19 is SCL

  Serial.begin (9600);		// start serial for output

  delay(1000);
  Serial.println("start");
  delay(2000);

  if (MMA8653FC_init ())	        // initialize the accelerometer by the i2c bus.
    Serial.println("not detected");
}

//------------------------------------------------------------------

void
loop ()
{
  int x,y,z;

  MMA8653FC_read (&axeXnow, &axeYnow, &axeZnow);
  Serial.printf("%d,%d,%d,",x,y,z);
  MAG3110_read (&x,&y,&z);
  Serial.printf("%d,%d,%d\n",x,y,z);
  
  delay (1000);
}

#endif

// I2C BUS:  already defined in "wire" librairy
// SDA
// SCL
// Accelerometer connected to +3.3V of arduino DO NOT CONNECT TO 5V (this will destroy the accelerometer!)
// all GND Pin of accelerometer connected to gnd of arduino

/********************ACCELEROMETER DATAS************/

// adresss of accelerometer
#define ADDRESS 0X1D		// MMA8653FC and MMA8652FC

// address of registers for MMA8653FC
#define ctrl_reg1 0x2A
#define ctrl_reg2 0x2B
#define ctrl_reg3 0x2C
#define ctrl_reg4 0x2D
#define ctrl_reg5 0x2E
#define whoami_reg 0x0D
#define whoami_val 0x5A
#define int_source 0x0C
#define status_ 0x00
#define f_setup 0x09
#define out_x_msb 0x01
#define out_y_msb 0x03
#define out_z_msb 0x05
#define sysmod 0x0B
#define xyz_data_cfg 0x0E
#define pl_status 0x10
#define ff_mt_cfg 0x15
#define ff_mt_src 0x16
#define ff_mt_ths 0x17
#define ff_mt_count 0x18


//******PROGRAM DATAS**********/
#define ctrl_reg_address 0     // to understand why 0x00 and not 0x01, look at the data-sheet p.19 or on the comments of the sub. This is valid only becaus we use auto-increment   

//------------------------------------------------------------------

static void
I2C_SEND (unsigned char REG_ADDRESS, unsigned char DATA)  //SEND data
{
  Wire.beginTransmission (ADDRESS);
  Wire.write (REG_ADDRESS);
  Wire.write (DATA);
  Wire.endTransmission (I2C_NOSTOP, 1000);
  delayMicroseconds (2);
}

// READ number data from i2c slave ctrl-reg register and return the result

int
I2C_READ_REG (int reg_address)
{
  Wire.beginTransmission (ADDRESS);	  //=ST + (Device Adress+W(0)) + wait for ACK
  Wire.write (reg_address);	          // register to read
  Wire.endTransmission(I2C_NOSTOP, 1000); // stop transmitting
  delayMicroseconds (2);	          //
  Wire.requestFrom (ADDRESS, 1);	  // read one byte
  return Wire.read();
}


// read the chip whoami value

int
MMA8653_who_am_i (void)
{
  Wire.beginTransmission (ADDRESS);    // transmit to device 0x0E
  Wire.write (whoami_reg);      // x MSB reg
  Wire.endTransmission(I2C_NOSTOP, 1000); // stop transmitting
  delayMicroseconds (2);            //
  Wire.requestFrom (ADDRESS, 1);    // request 1 byte
  return (unsigned char)Wire.read();      // read the byte
}

//------------------------------------------------------------------

int
MMA8653FC_init ()
{
  I2C_SEND (ctrl_reg1, 0X00);	        // standby to be able to configure

  I2C_SEND (xyz_data_cfg, B00000000);	// 2G full range mode
  I2C_SEND (ctrl_reg1, B00100001);	// Output data rate at 50Hz, no auto wake, no auto scale adjust, no fast read mode, active

  if (MMA8653_who_am_i() == whoami_val)
    return 0;                          // OK
  else
    return -1;                         // error

}

// reduce power draw

void MMA8653FC_low_power()
{
  I2C_SEND (ctrl_reg1,  0X00);	        // standby to be able to configure
  I2C_SEND (ff_mt_cfg,  B01111000);	//
  I2C_SEND (int_source, B10010100);	// interrupt on wake, orientation change or motion
  I2C_SEND (ctrl_reg4,  B11111111);	// Interrupt enable on anything
  I2C_SEND (ctrl_reg5,  B00000000);	// Use INT2

  // clear previous interrupts
  I2C_READ_REG(sysmod);
  I2C_READ_REG(pl_status);
  I2C_READ_REG(ff_mt_src);
  
  I2C_SEND (ctrl_reg1,  B01101001);	// Output data rate at 12Hz, active
}

void MMA8653FC_standby()
{
  I2C_SEND (ctrl_reg1,  0X00);	        // standby mode for lowest power
}


//------------------------------------------------------------------

// read current values from accelerometer in milli-Gs

void
MMA8653FC_read(int *axeXnow, int *axeYnow, int *axeZnow)
{

  Wire.beginTransmission (ADDRESS);	  //= ST + (Device Adress+W(0)) + wait for ACK
  Wire.write (ctrl_reg_address);	  // store the register to read in the buffer of the wire library
  Wire.endTransmission (I2C_NOSTOP,1000); // actually send the data on the bus -note: returns 0 if transmission OK-
  delayMicroseconds (2);	          //
  Wire.requestFrom (ADDRESS, 7);	  // read a number of byte and store them in wire.read (note: by nature, this is called an "auto-increment register adress")
  
  Wire.read();   // discard first byte (status)

  // read six bytes
  // note:  2G is 32768, -2G is -32768

  *axeXnow = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first
  *axeYnow = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first
  *axeZnow = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first

}  // MMA8653FC_read()

//------------------------------------------------------------------


