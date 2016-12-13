
// code related to the PAR/color sensor

#include <Arduino.h>
#include "utility/TCS3471.h"              // color sensor
#include "serial.h"
#include "defines.h"
#include "eeprom.h"

// external function declarations
void i2cWrite(byte address, byte count, byte* buffer);
void i2cRead(byte address, byte count, byte* buffer);

// global variables 
extern float light_intensity;
extern float light_intensity_averaged;
extern float light_intensity_raw;
extern float light_intensity_raw_averaged;
extern float r;
extern float r_averaged;
extern float g;
extern float g_averaged;
extern float b;
extern float b_averaged;

static TCS3471 *par_sensor=0;


// initialize the PAR/color sensor

void PAR_init()
{
  // color sensor init

  if (par_sensor == 0)
     par_sensor = new TCS3471(i2cWrite, i2cRead);

  par_sensor->setWaitTime(200.0);
  par_sensor->setIntegrationTime(700.0);
  par_sensor->setGain(TCS3471_GAIN_1X);
  par_sensor->enable();

}  // PAR_init()

uint16_t par_to_dac (float _par, uint16_t _pin) {                                             // convert dac value to par, in form y = mx2+ rx + b where y is the dac value  
//  int dac_value = _par * _par * eeprom->par_to_dac_slope1[_pin] + _par * eeprom->par_to_dac_slope2[_pin] + eeprom->par_to_dac_yint[_pin];     
  double a = _par * _par * _par * _par * eeprom->par_to_dac_slope1[_pin]/1000000000; 
  double b = _par * _par * _par * eeprom->par_to_dac_slope2[_pin]/1000000000; 
  double c = _par * _par * eeprom->par_to_dac_slope3[_pin]/1000000000; 
  double d = _par * eeprom->par_to_dac_slope4[_pin]; 
  double e = eeprom->par_to_dac_yint[_pin]; 
  int dac_value = a + b + c + d + e;   
  if (_par == 0) {                                                                           // regardless of the calibration, force a PAR of zero to lights off
    dac_value = 0;
  } 
  dac_value = constrain(dac_value,0,4095);
/*
  Serial_Print_Line("");
  Serial_Print_Line(eeprom->par_to_dac_slope1[_pin]/1000000000,15);
  Serial_Print_Line(eeprom->par_to_dac_slope2[_pin]/1000000000,11);
  Serial_Print_Line(eeprom->par_to_dac_slope3[_pin]/1000000000,9);
  Serial_Print_Line(eeprom->par_to_dac_slope4[_pin],7);
  Serial_Print_Line(eeprom->par_to_dac_yint[_pin],4);

  Serial_Print_Line("");
  Serial_Print_Line(a,15);
  Serial_Print_Line(b,11);
  Serial_Print_Line(c,9);
  Serial_Print_Line(d,7);
  Serial_Print_Line(e,4);
*/
  return dac_value;
}

float light_intensity_raw_to_par (float _light_intensity_raw, float _r, float _g, float _b) {
  int par_value = eeprom->light_slope_all * _light_intensity_raw + _r * eeprom->light_slope_r + _g * eeprom->light_slope_g + _b * eeprom->light_slope_b + eeprom->light_yint;
 
  if (par_value < 0)                                                                             // there may be cases when the output could be less than zero.  In those cases, set to zero and mark a flag 
    par_value = 0; 

/*
  Serial_Print_Line("light_intenisty_raw_to_par");
  Serial_Print_Line(par_value);
  Serial_Print_Line(_light_intensity_raw);
  Serial_Print_Line(_r);
  Serial_Print_Line(_g);
  Serial_Print_Line(_b);
  Serial_Print_Line("saved values");
  Serial_Print_Line(eeprom->light_slope_all,6);
  Serial_Print_Line(eeprom->light_slope_r,6);
  Serial_Print_Line(eeprom->light_slope_g,6);
  Serial_Print_Line(eeprom->light_slope_b,6);
  Serial_Print_Line(eeprom->light_yint,6);
  */
  return par_value;
}

int get_light_intensity(int _averages) {

  r = par_sensor->readRData();
  g = par_sensor->readGData();
  b = par_sensor->readBData();
  light_intensity_raw = par_sensor->readCData();
  light_intensity = light_intensity_raw_to_par(light_intensity_raw, r, g, b);

  r_averaged += r / _averages;
  g_averaged += g / _averages;
  b_averaged += b / _averages;
/*
  Serial_Print_Line("get_light_intensity");
  Serial_Print_Line(light_intensity_raw,6);
  Serial_Print_Line(light_intensity,6);
  Serial_Print_Line(r_averaged,6);
  Serial_Print_Line(g_averaged,6);
  Serial_Print_Line(b_averaged,6);
*/

  light_intensity_raw_averaged += light_intensity_raw / _averages;
  light_intensity_averaged += light_intensity / _averages;
  return light_intensity;
} // get_light_intensity()

