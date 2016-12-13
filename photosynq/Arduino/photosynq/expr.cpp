/*
 * These are the strings which can be interpreted by the expr() function
 * They are accessible through the following API calls:
 * 
 * save
 * recall
 * pulse_length
 * pulse_distance
 * pulsed_lights_brightness
 * nonpulsed_lights_brightness
 * 
 * Example use:
 * 
 * 
 * See the PhotosynQ firmware wiki for details
 * 
 * NOTE! If this file is updated, the wiki should also be updated!
 * 
 */


#include <stdio.h>
#include <string.h>
#include "defines.h"
#include "eeprom.h"
#include "serial.h"

// return value of a variable by name

float variable(const char *name)
{
  //Serial_Printf("\nvar = |%s|\n",name);

  // look up the variables by name

  float value = NAN;

  // Userdef commands
  if (strncmp( name, "userdef[", 8) == 0) {  // handles all userdef[x] references
    unsigned index = atoi(name + 8);
    if (index < NUM_USERDEFS)
      value = eeprom->userdef[index];

  } else if (strcmp( name, "light_intensity" ) == 0 ) {
    value = light_intensity;

  } else if (strcmp( name, "light_intensity_averaged" ) == 0 ) {
    value = light_intensity_averaged;

  } else if (strcmp( name, "light_intensity_raw" ) == 0 ) {
    value = light_intensity_raw;

  } else if (strcmp( name, "light_intensity_raw_averaged" ) == 0 ) {
    value = light_intensity_raw_averaged;

  } else if (strcmp( name, "r" ) == 0 ) {
    value = r;

  } else if (strcmp( name, "r_averaged" ) == 0 ) {
    value = r_averaged;

  } else if (strcmp( name, "g" ) == 0 ) {
    value = g;

  } else if (strcmp( name, "g_averaged" ) == 0 ) {
    value = g_averaged;

  } else if (strcmp( name, "b" ) == 0 ) {
    value = b;

  } else if (strcmp( name, "b_averaged" ) == 0 ) {
    value = b_averaged;

  } else if (strcmp( name, "detector_read1" ) == 0 ) {
    value = detector_read1;
    
  } else if (strcmp( name, "detector_read1_averaged" ) == 0 ) {
    value = detector_read1_averaged;

  } else if (strcmp( name, "detector_read2" ) == 0 ) {
    value = detector_read2;
    
  } else if (strcmp( name, "detector_read2_averaged" ) == 0 ) {
    value = detector_read2_averaged;

  } else if (strcmp( name, "detector_read3" ) == 0 ) {
    value = detector_read3;
    
  } else if (strcmp( name, "detector_read3_averaged" ) == 0 ) {
    value = detector_read3_averaged;

  } else if (strcmp( name, "thickness" ) == 0 ) {
    value = thickness;

  } else if (strcmp( name, "thickness_averaged" ) == 0 ) {
    value = thickness_averaged;

  } else if (strcmp( name, "thickness_raw" ) == 0 ) {
    value = thickness_raw;

  } else if (strcmp( name, "thickness_raw_averaged" ) == 0 ) {
    value = thickness_raw_averaged;

  } else if (strcmp( name, "contactless_temp" ) == 0 ) {
    value = contactless_temp;

  } else if (strcmp( name, "contactless_temp_averaged" ) == 0 ) {
    value = contactless_temp_averaged;

  } else if (strcmp( name, "compass" ) == 0 ) {
    value = compass;

  } else if (strcmp( name, "compass_averaged" ) == 0 ) {
    value = compass_averaged;

  } else if (strcmp( name, "x_compass_raw" ) == 0 ) {
    value = x_compass_raw;

  } else if (strcmp( name, "y_compass_raw" ) == 0 ) {
    value = y_compass_raw;

  } else if (strcmp( name, "z_compass_raw" ) == 0 ) {
    value = z_compass_raw;

  } else if (strcmp( name, "x_compass_raw_averaged" ) == 0 ) {
    value = x_compass_raw_averaged;

  } else if (strcmp( name, "y_compass_raw_averaged" ) == 0 ) {
    value = y_compass_raw_averaged;

  } else if (strcmp( name, "z_compass_raw_averaged" ) == 0 ) {
    value = z_compass_raw_averaged;

  } else if (strcmp( name, "angle" ) == 0 ) {
    value = angle;

  } else if (strcmp( name, "angle_averaged" ) == 0 ) {
    value = angle_averaged;

  } else if (strcmp( name, "pitch" ) == 0 ) {
    value = pitch;

  } else if (strcmp( name, "pitch_averaged" ) == 0 ) {
    value = pitch_averaged;

  } else if (strcmp( name, "roll" ) == 0 ) {
    value = roll;

  } else if (strcmp( name, "roll_averaged" ) == 0 ) {
    value = roll_averaged;

  } else if (strcmp( name, "x_tilt" ) == 0 ) {
    value = x_tilt;

  } else if (strcmp( name, "y_tilt" ) == 0 ) {
    value = y_tilt;

  } else if (strcmp( name, "z_tilt" ) == 0 ) {
    value = z_tilt;

  } else if (strcmp( name, "x_tilt_averaged" ) == 0 ) {
    value = x_tilt_averaged;

  } else if (strcmp( name, "y_tilt_averaged" ) == 0 ) {
    value = y_tilt_averaged;

  } else if (strcmp( name, "z_tilt_averaged" ) == 0 ) {
    value = z_tilt_averaged;

  } else if (strcmp( name, "temperature" ) == 0 ) {
    value = temperature;

  } else if (strcmp( name, "humidity" ) == 0 ) {
    value = humidity;

  } else if (strcmp( name, "pressure" ) == 0 ) {
    value = pressure;

  } else if (strcmp( name, "temperature_averaged" ) == 0 ) {
    value = temperature_averaged;

  } else if (strcmp( name, "humidity_averaged" ) == 0 ) {
    value = humidity_averaged;

  } else if (strcmp( name, "pressure_averaged" ) == 0 ) {
    value = pressure_averaged;

  } else if (strcmp( name, "temperature2" ) == 0 ) {
    value = temperature2;

  } else if (strcmp( name, "humidity2" ) == 0 ) {
    value = humidity2;

  } else if (strcmp( name, "pressure2" ) == 0 ) {
    value = pressure2;

  } else if (strcmp( name, "temperature2_averaged" ) == 0 ) {
    value = temperature2_averaged;

  } else if (strcmp( name, "humidity2_averaged" ) == 0 ) {
    value = humidity2_averaged;

  } else if (strcmp( name, "pressure2_averaged" ) == 0 ) {
    value = pressure2_averaged;

  } else if (strcmp( name, "co2" ) == 0 ) {
    value = co2;

  } else if (strcmp( name, "co2_averaged" ) == 0 ) {
    value = co2_averaged;

  } else if (strcmp( name, "light_yint" ) == 0 ) {
    value = eeprom->light_yint;

  } else if (strcmp( name, "light_slope_all" ) == 0 ) {
    value = eeprom->light_slope_all;

  } else if (strcmp( name, "light_slope_r" ) == 0 ) {
    value = eeprom->light_slope_r;

  } else if (strcmp( name, "light_slope_g" ) == 0 ) {
    value = eeprom->light_slope_g;

  } else if (strcmp( name, "light_slope_b" ) == 0 ) {
    value = eeprom->light_slope_b;

  } else if (strcmp( name, "thickness_a" ) == 0 ) {
    value = eeprom->thickness_a;

  } else if (strcmp( name, "thickness_b" ) == 0 ) {
    value = eeprom->thickness_b;

  } else if (strcmp( name, "thickness_c" ) == 0 ) {
    value = eeprom->thickness_c;

  } else if (strcmp( name, "analog_read" ) == 0 ) {
    value = analog_read;

  } else if (strcmp( name, "analog_read_averaged" ) == 0 ) {
    value = analog_read_averaged;

  } else if (strcmp( name, "digital_read" ) == 0 ) {
    value = digital_read;

  } else if (strcmp( name, "digital_read_averaged" ) == 0 ) {
    value = digital_read_averaged;

  } else if (strcmp( name, "adc_read" ) == 0 ) {
    value = adc_read;

  } else if (strcmp( name, "adc_read_averaged" ) == 0 ) {
    value = adc_read_averaged;

  } else if (strcmp( name, "adc_read2" ) == 0 ) {
    value = adc_read2;

  } else if (strcmp( name, "adc_read2_averaged" ) == 0 ) {
    value = adc_read2_averaged;

  } else if (strcmp( name, "adc_read3" ) == 0 ) {
    value = adc_read3;

  } else if (strcmp( name, "adc_read3_averaged" ) == 0 ) {
    value = adc_read3_averaged;

/* Values saved to EEPROM
 * 
 */

  } else  if (strncmp( name, "detector_offset_slope[", 22) == 0 ) {
    unsigned index = atoi(name + 22);
    if (index < 4)
      value = eeprom->detector_offset_slope[index];

  } else  if (strncmp( name, "detector_offset_yint[", 21) == 0 ) {
    unsigned index = atoi(name + 21);
    if (index < 4)
      value = eeprom->detector_offset_yint[index];

  } else  if (strncmp( name, "mag_bias[", 9) == 0 ) {
    unsigned index = atoi(name + 9);
    if (index < 3)
      value = eeprom->mag_bias[index];

  } else  if (strncmp( name, "accel_bias[", 11) == 0 ) {
    unsigned index = atoi(name + 11);
    if (index < 3)
      value = eeprom->accel_bias[index];

  } else  if (strncmp( name, "par_to_dac_slope1[", 18) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      value = eeprom->par_to_dac_slope1[index];

  } else  if (strncmp( name, "par_to_dac_slope2[", 18) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      value = eeprom->par_to_dac_slope2[index];

  } else  if (strncmp( name, "par_to_dac_slope3[", 18) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      value = eeprom->par_to_dac_slope3[index];

  } else  if (strncmp( name, "par_to_dac_slope4[", 18) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      value = eeprom->par_to_dac_slope4[index];

  } else  if (strncmp( name, "par_to_dac_yint[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      value = eeprom->par_to_dac_yint[index];

  } else  if (strncmp( name, "ir_baseline_slope[", 18) == 0 ) {
    unsigned index = atoi(name + 18);
    if (index < NUM_LEDS + 1)
      value = eeprom->ir_baseline_slope[index];

  } else  if (strncmp( name, "ir_baseline_yint[", 17) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      value = eeprom->ir_baseline_yint[index];

  } else  if (strncmp( name, "colorcal_intensity1_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity1_slope[index];

  } else  if (strncmp( name, "colorcal_intensity2_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity2_slope[index];

  } else  if (strncmp( name, "colorcal_intensity3_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity3_slope[index];

  } else  if (strncmp( name, "colorcal_intensity1_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity1_yint[index];

  } else  if (strncmp( name, "colorcal_intensity2_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity2_yint[index];

  } else  if (strncmp( name, "colorcal_intensity3_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_intensity3_yint[index];

  } else  if (strncmp( name, "colorcal_blank1[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_blank1[index];

  } else  if (strncmp( name, "colorcal_blank2[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_blank2[index];

  } else  if (strncmp( name, "colorcal_blank3[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      value = eeprom->colorcal_blank3[index];

  }

  //Serial_Printf("variable %s=%g\n",name,value);
  return value;
}

// ====================================

// simple +-/*() expression evaluator

// original version by Jerry Coffin
// variables, float, C, non-stdin modifications by Jon Zeeff

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

static const char *ptr = 0;

static float expression(void);

static float factor() {
  float val;
  char ch = *(ptr++);

  if (ch == '(') {
    val = expression();
    ch = *(ptr++);
    if (ch != ')') {
      val = NAN;     // error - missing )
    }
  }
  else if (isdigit(ch) || ch == '.' || ch == '+' || ch == '-') {  // beware of leading + or -
    val = strtod(ptr - 1, (char **)&ptr);
  } else if (isalpha(ch) || ch == '_') {   // variable such as "userdef[1]"
    const char *p = ptr;
    --ptr;
    while (isalnum(*p) || *p == '[' || *p == ']' || *p == '_')     // move past variable (only first has to be alpha)
      ++p;
    char string[50];
    strncpy(string, ptr, p - ptr);   // just to get the null
    string[p - ptr] = 0;             // put in null
    val = variable(string);          // evaluate it
    ptr = p;
  } else
    val = NAN;

  return val;
}

static float term() {
  char ch;
  float val = factor();
  ch = *ptr++;
  if (ch == '*' || ch == '/') {
    float b = term();
    if (ch == '*')
      val *= b;
    else
      val /= b;
  }
  else --ptr;

  return val;
}

static float expression() {
  float val = term();
  char ch = *(ptr++);

  // operators
  if (ch == '-' || ch == '+') {
    float b = expression();
    if (ch == '+')
      val += b;
    else
      val -= b;
  }
  else --ptr;

  return val;
}

// given a string, evaluate it.   Example "2*(userdef[49]+2)"
float expr(const char *s) { 
  ptr = s;
  return expression();
}
