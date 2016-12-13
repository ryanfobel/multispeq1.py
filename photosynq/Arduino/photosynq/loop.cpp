// main loop and some support routines

#define EXTERN
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

// function declarations

inline static void startTimers(unsigned _pulsedistance);
inline static void stopTimers(void);
void reset_freq(void);
void upgrade_firmware(void);            // for over-the-air firmware updates
void boot_check(void);                  // for over-the-air firmware updates
int get_light_intensity(int x);
static void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom);
void get_set_device_info(const int _set);
void temp_get_set_device_info();
int abort_cmd(void);
static void environmentals(JsonArray a, const int _averages, const int x, int oneOrArray);
void readSpectrometer(int intTime, int delay_time, int read_time, int accumulateMode);
void MAG3110_read (int *x, int *y, int *z);
void MMA8653FC_read(int *axeXnow, int *axeYnow, int *axeZnow);
void MMA8653FC_standby(void);
float MLX90615_Read(int TaTo);
uint16_t par_to_dac (float _par, uint16_t _pin);
float light_intensity_raw_to_par (float _light_intensity_raw, float _r, float _g, float _b);
unsigned long requestCo2(int timeout);
static void print_all (void);
float expr(const char str[]);
void do_protocol(void);
void do_command(void);
void print_calibrations(void);
void start_on_open_close(void);
void start_on_pin_high(int pin);
void get_compass_and_angle (int notRaw, int _averages);
float get_thickness (int notRaw, int _averages);
float get_contactless_temp (int _averages);
void get_detector_value (int _averages, int this_light, int this_intensity, int this_detector, int this_pulsesize, int detector_read1or2or3);
void get_temperature_humidity_pressure (int _averages);
void get_temperature_humidity_pressure2 (int _averages);
void init_chips(void);
void configure_bluetooth(void);
void reboot(void);
void turn_on_3V3(void);
void turn_on_5(void);


struct theReadings {                                            // use to return which readings are associated with the environmental_array calls
  const char* reading1;
  const char* reading2;
  const char* reading3;
  const char* reading4;
  const char* reading5;
  int numberReadings;
};
theReadings getReadings (const char* _thisSensor);                        // get the actual sensor readings associated with each environmental call (so compass and angle when you call compass_and_angle, etc.)

//////////////////////// MAIN LOOP /////////////////////////

// process ascii serial input commands of two forms:
// 1010+<parameter1>+<parameter2>+...  (a command)
// [...] (a json protocol to be executed)

void loop() {

  // save power
  turn_off_5V();         // save battery - turn off a few things

  // read until we get a character - primary idle loop
  int c;

  activity();               // record fact that we have seen activity (used with powerdown()), make sure it's recorded after the latest protocol or command is completed.

  for (;;) {
    c = Serial_Peek();

    if (c != -1)            // received something
      break;

    powerdown();            // power down if no activity for x seconds

    sleep_cpu();         // save power - low impact since cpu stays on - this causes an issue an intermittent problem with serial communcation, leave off for now.

  } // for

  crc32_init();

  if (c == '[')
    do_protocol();          // start of json
  else
    do_command();           // received a non '[' char - processs command

  Serial_Flush_Output();

} // loop()

// =========================================

// globals - try to avoid
static uint8_t _meas_light;         // measuring light to be used during the interrupt
static uint16_t _pulsesize;     // pulse width in usec
static volatile int pulse_done = 0; // set by ISR

// process a numeric + command

void do_command()
{
  char choose[50];
  Serial_Input_Chars(choose, "+", 500, sizeof(choose) - 1);

  for (unsigned i = 0; i < strlen(choose); ++i) {   // remove ctrl characters
    if (!isprint(choose[i]))
      choose[i] = 0;
  }

  if (strlen(choose) < 3) {         // short or null command, quietly ignore it
    return;
  }

  if (!isalnum(choose[0])) {
    Serial_Printf("{\"error\":\" bad command\"}\n");
    return;                         // go read another command
  }

  unsigned val;                     // we accept int or alpha commands
  if (isdigit(choose[0]))
    val = atoi(choose);
  else
    val = hash(choose);             // convert alpha command to an int

  // process single commands
  switch (val) {

    case hash("hello"):
    case 1000:                                                                    // print "Ready" to USB and/or Bluetooth
      Serial_Print(DEVICE_NAME);
      Serial_Print_Line(" Ready");
      break;

    case hash("set_dac"):
      Serial_Print("on 5v, ");
      turn_on_5V();                     // is normally off, but many of the below commands need it
      // standard startup routine for new device
      DAC_set_address(LDAC1, 0, 1);                                                 // Set DAC addresses to 1,2,3 assuming addresses are unset and all are factory (0,0,0)
      Serial_Print("dac 1, ");
      DAC_set_address(LDAC2, 0, 2);
      Serial_Print("dac 2, ");
      DAC_set_address(LDAC3, 0, 3);
      Serial_Print_Line("dac 3");
      get_set_device_info(1);                                                           //  input device info and write to eeprom
      break;

    case hash("cycle5v"):
      Serial_Print_Line("turning off 5v in 3 seconds...");
      delay(3000);
      turn_off_5V();
      Serial_Print_Line("turning off 3.3v in 3 seconds...");
      delay(3000);
      turn_off_3V3();
      Serial_Print_Line("turning on 3.3v in 3 seconds...");
      delay(3000);
      turn_on_3V3();
      Serial_Print_Line("turning on 5v in 3 seconds...");
      delay(3000);
      turn_on_5V();
      delay(3000);
      Serial_Print_Line("reboot before rerunning as states may be weird now");
      break;

    case hash("all_sensors"):                                                                          // continuously output until user enter -1+
      {
        int Xcomp, Ycomp, Zcomp;
        int Xval, Yval, Zval;
        int leave = 0;
        const int samples = 1000;
        while (leave != -1) {
          long sum = 0;
          leave = Serial_Input_Long("+", 1000);
          float par = get_light_intensity(1);
          float contactless_temp = (MLX90615_Read(0) + MLX90615_Read(0) + MLX90615_Read(0)) / 3.0;
          MMA8653FC_read(&Xval, &Yval, &Zval);
          MAG3110_read(&Xcomp, &Ycomp, &Zcomp);
          delay(200);
          for (int i = 0; i < samples; ++i) {
            sum += analogRead(HALL_OUT);
          }
          int hall = (sum / samples);
          delay(1);
          float temperature1 = bme1.readTemperature();
          float temperature2 = bme2.readTemperature();
          float relative_humidity1 = bme1.readHumidity();
          float relative_humidity2 = bme2.readHumidity();
          float pressure1 = bme1.readPressure() / 100;
          float pressure2 = bme2.readPressure() / 100;
          Serial_Printf("{\"par\":%f,\"temperature\":%f,\"relative_humidity\":%f,\"pressure\":%f,\"temperature2\":%f,\"relative_humidity2\":%f,\"pressure2\":%f,\"contactless_temp\":%f,\"hall\":%d,\"accelerometer\":[%d,%d,%d],\"magnetometer\":[%d,%d,%d]}", par, temperature1, relative_humidity1, pressure1, temperature2, relative_humidity2, pressure2, contactless_temp, hall, Xval, Yval, Zval, Xcomp, Ycomp, Zcomp);
          //          Serial_Printf("{\"par_raw\":%d,\"contactless_temp\":%f,\"hall\":%d,\"accelerometer\":[%d,%d,%d],\"magnetometer\":[%d,%d,%d]}", par_raw, contactless_temp, hall, Xval, Yval, Zval, Xcomp, Ycomp, Zcomp);
          Serial_Print_CRC();
        }
      }
      break;

    case hash("any_light"):
      {
        turn_on_5V();                  // turn on 5V to turn on the lights
        Serial_Print_Line("\"message\": \"Enter led # setting followed by +: \"}");
        int led =  Serial_Input_Double("+", 0);
        Serial_Print_Line("\"message\": \"Enter dac setting followed by +:  \"}");
        int setting =  Serial_Input_Double("+", 0);
        DAC_set(led, setting);
        DAC_change();
        digitalWriteFast(LED_to_pin[led], HIGH);
        delay(5000);
        digitalWriteFast(LED_to_pin[led], LOW);
        DAC_set(led, 0);
        DAC_change();
      }
      break;

    case hash("print_memory"):
      print_calibrations();
      break;

    case hash("device_info"):
    case 1007:
      get_set_device_info(0);
      break;

    case hash("set_device_info"):  // set the device name and
    case 1008:
      get_set_device_info(1);
      break;

    case hash("configure_bluetooth"):  // set the bluetooth name and baud rate
      configure_bluetooth();
      break;

    case hash("light1"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE1");
      DAC_set(1, 300);
      DAC_change();
      digitalWriteFast(PULSE1, HIGH);
      delay(1000);
      digitalWriteFast(PULSE1, LOW);
      DAC_set(1, 0);
      DAC_change();
      break;
    case hash("light2"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE2");
      DAC_set(2, 300);
      DAC_change();
      digitalWriteFast(PULSE2, HIGH);
      delay(1000);
      digitalWriteFast(PULSE2, LOW);
      DAC_set(2, 0);
      DAC_change();
      break;
    case hash("light3"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE3");
      DAC_set(3, 300);
      DAC_change();
      digitalWriteFast(PULSE3, HIGH);
      delay(1000);
      digitalWriteFast(PULSE3, LOW);
      DAC_set(3, 0);
      DAC_change();
      break;
    case hash("light4"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE4");
      DAC_set(4, 300);
      DAC_change();
      digitalWriteFast(PULSE4, HIGH);
      delay(1000);
      digitalWriteFast(PULSE4, LOW);
      DAC_set(4, 0);
      DAC_change();
      break;
    case hash("light5"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE5");
      DAC_set(5, 300);
      DAC_change();
      digitalWriteFast(PULSE5, HIGH);
      delay(1000);
      digitalWriteFast(PULSE5, LOW);
      DAC_set(5, 0);
      DAC_change();
      break;
    case hash("light6"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE6");
      DAC_set(6, 300);
      DAC_change();
      digitalWriteFast(PULSE6, HIGH);
      delay(1000);
      digitalWriteFast(PULSE6, LOW);
      DAC_set(6, 0);
      DAC_change();
      break;
    case hash("light7"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE7");
      DAC_set(7, 300);
      DAC_change();
      digitalWriteFast(PULSE7, HIGH);
      delay(1000);
      digitalWriteFast(PULSE7, LOW);
      DAC_set(7, 0);
      DAC_change();
      break;
    case hash("light8"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE8");
      DAC_set(8, 300);
      DAC_change();
      digitalWriteFast(PULSE8, HIGH);
      delay(1000);
      digitalWriteFast(PULSE8, LOW);
      DAC_set(8, 0);
      DAC_change();
      break;
    case hash("light9"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE9");
      DAC_set(9, 300);
      DAC_change();
      digitalWriteFast(PULSE9, HIGH);
      delay(1000);
      digitalWriteFast(PULSE9, LOW);
      DAC_set(9, 0);
      DAC_change();
      break;
    case hash("light10"):
      turn_on_5V();                  // turn on 5V to turn on the lights
      Serial_Print_Line("PULSE10");
      DAC_set(10, 300);
      DAC_change();
      digitalWriteFast(PULSE10, HIGH);
      delay(1000);
      digitalWriteFast(PULSE10, LOW);
      DAC_set(10, 0);
      DAC_change();
      break;

    case hash("set_serial"):
      {

        Serial_Print("Enter 1/2/3/4+\n");
        long setserial = Serial_Input_Long();
        Serial_Printf("set serial to %d\n", (int)setserial);
        Serial_Set((int) setserial);
        Serial_Print_Line("test print");
      }
      break;

    case hash("reset"):
    case 1027:                                                                                // restart teensy (keep here!)
      _reboot_Teensyduino_();
      break;

    case hash("reboot"):
      reboot();
      break;

    case hash("print_all"):
    case 1029:
      print_all();                                                                            // print everything in the eeprom (all values defined in eeprom.h)
      break;

    case hash("calibrate_leds"):
      {
        turn_on_5V();                  // turn on 5V to turn on the lights
        // create arrays for the lights we're calibrating so we can loop through them effectively
        int lights_number [5] = {1, 2, 3, 4, 7};
        int lights [5] = {PULSE1, PULSE2, PULSE3, PULSE4, PULSE7};
        while (1) {
          start_on_pin_high(14);          // assumes you're starting when pin 14 goes high (DEBUG_DC)
//          delay(1200);
          for (int i = 0; i < sizeof(lights_number) / sizeof(lights_number[0]); i++) {  //  do every 10 dac values from 20 - 200.
            for (int j = 20; j < 201; j = j + 10) {
              DAC_set(lights_number[i], j);
              DAC_change();
              digitalWriteFast(lights[i], HIGH);
              delay(300);
              digitalWriteFast(lights[i], LOW);
            }
            DAC_set(lights_number[i], 250);   // now do dac values 250, 500, and 800 to get the high range
            DAC_change();
            digitalWriteFast(lights[i], HIGH);
            delay(300);
            digitalWriteFast(lights[i], LOW);
            DAC_set(lights_number[i], 500);
            DAC_change();
            digitalWriteFast(lights[i], HIGH);
            delay(300);
            digitalWriteFast(lights[i], LOW);
            DAC_set(lights_number[i], 800);
            DAC_change();
            digitalWriteFast(lights[i], HIGH);
            delay(300);
            digitalWriteFast(lights[i], LOW);
            DAC_set(lights_number[i], 0);    // now make sure it's shut off.
            DAC_change();
          }
        }
      }
      break;

    case hash("calibrate_leds_manual"):
      {
        turn_on_5V();                  // turn on 5V to turn on the lights
        // create arrays for the lights we're calibrating so we can loop through them effectively
        int lights_number [5] = {1, 2, 3, 4, 7};
        int lights [5] = {PULSE1, PULSE2, PULSE3, PULSE4, PULSE7};
        char responseTemp[20];
        String response;
        delay(1300);
        for (int i = 0; i < sizeof(lights_number) / sizeof(lights_number[0]); i++) {  //  do every 10 dac values from 20 - 200.
          for (int j = 20; j < 201; j = j + 10) {
            DAC_set(lights_number[i], j);
            DAC_change();
            digitalWriteFast(lights[i], HIGH);
            response.append(Serial_Input_Chars(responseTemp,",", 0, 500));       // input the protocol
            response.append(",");       // input the protocol
            digitalWriteFast(lights[i], LOW);
          }
          DAC_set(lights_number[i], 250);   // now do dac values 250, 500, and 800 to get the high range
          DAC_change();
          digitalWriteFast(lights[i], HIGH);
          response.append(Serial_Input_Chars(responseTemp, ",", 0, 500));       // input the protocol
          response.append(",");       // input the protocol
          digitalWriteFast(lights[i], LOW);
          DAC_set(lights_number[i], 500);
          DAC_change();
          digitalWriteFast(lights[i], HIGH);
          response.append(Serial_Input_Chars(responseTemp, ",", 0, 500));       // input the protocol
          response.append(",");       // input the protocol
          digitalWriteFast(lights[i], LOW);
          DAC_set(lights_number[i], 800);
          DAC_change();
          digitalWriteFast(lights[i], HIGH);
          response.append(Serial_Input_Chars(responseTemp, ",", 0, 500));       // input the protocol
          response.append(",");       // input the protocol
          digitalWriteFast(lights[i], LOW);
          DAC_set(lights_number[i], 0);    // now make sure it's shut off.
          DAC_change();
          response.append("\r\n");       // input the protocol
        }
        Serial_Print(response);
      }
      break;

    case hash("set_magnetometer_bias"):
    case 1030: // 3 magnetometer bias values
      store(mag_bias[0], Serial_Input_Double("+", 0));
      store(mag_bias[1], Serial_Input_Double("+", 0));
      store(mag_bias[2], Serial_Input_Double("+", 0));
      break;
    case hash("set_magnetometer"):
    case 1031: // 9 magnetometer calibration values
      for (uint16_t i = 0; i < 3; i++) {
        for (uint16_t j = 0; j < 3; j++) {
          store(mag_cal[j][i], Serial_Input_Double("+", 0));
        }
      }
      break;
    case hash("set_accelerometer_bias"):
    case 1032: // 3 accelerometer bias values
      store(accel_bias[0], Serial_Input_Double("+", 0));
      store(accel_bias[1], Serial_Input_Double("+", 0));
      store(accel_bias[2], Serial_Input_Double("+", 0));
      break;
    case hash("set_accelerometer"):
    case 1033: // 9 accelerometer calibration values
      for (uint16_t i = 0; i < 3; i++) {
        for (uint16_t j = 0; j < 3; j++) {
          store(accel_cal[j][i], Serial_Input_Double("+", 0));
        }
      }
      break;
    case hash("set_par"):
      store(light_slope_all, Serial_Input_Double("+", 0));
      store(light_slope_r, Serial_Input_Double("+", 0));
      store(light_slope_g, Serial_Input_Double("+", 0));
      store(light_slope_b, Serial_Input_Double("+", 0));
      store(light_yint, Serial_Input_Double("+", 0));
      break;
    case hash("set_thickness"):
      store(thickness_a, Serial_Input_Double("+", 0));
      store(thickness_b, Serial_Input_Double("+", 0));
      store(thickness_c, Serial_Input_Double("+", 0));
      store(thickness_min, Serial_Input_Double("+", 0));
      store(thickness_max, Serial_Input_Double("+", 0));
      break;
    case hash("set_thickness_quick"):
      store(thickness_min, Serial_Input_Double("+", 0));
      store(thickness_max, Serial_Input_Double("+", 0));
      break;
    case hash("set_detector1_offset"):
      store(detector_offset_slope[0], Serial_Input_Double("+", 0));
      store(detector_offset_yint[0], Serial_Input_Double("+", 0));
      break;
    case hash("set_detector2_offset"):
      store(detector_offset_slope[1], Serial_Input_Double("+", 0));
      store(detector_offset_yint[1], Serial_Input_Double("+", 0));
      break;
    case hash("set_detector3_offset"):
      store(detector_offset_slope[2], Serial_Input_Double("+", 0));
      store(detector_offset_yint[2], Serial_Input_Double("+", 0));
      break;
    case hash("set_detector4_offset"):
      store(detector_offset_slope[3], Serial_Input_Double("+", 0));
      store(detector_offset_yint[3], Serial_Input_Double("+", 0));
      break;
    case hash("set_led_par"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(par_to_dac_slope1[led], Serial_Input_Double("+", 0));
            store(par_to_dac_slope2[led], Serial_Input_Double("+", 0));
            store(par_to_dac_slope3[led], Serial_Input_Double("+", 0));
            store(par_to_dac_slope4[led], Serial_Input_Double("+", 0));
            store(par_to_dac_yint[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("ir_baseline"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(ir_baseline_slope[led], Serial_Input_Double("+", 0));
            store(ir_baseline_yint[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("set_colorcal1"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(colorcal_intensity1_slope[led], Serial_Input_Double("+", 0));
            store(colorcal_intensity1_yint[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("set_colorcal2"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(colorcal_intensity2_slope[led], Serial_Input_Double("+", 0));
            store(colorcal_intensity2_yint[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("set_colorcal3"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(colorcal_intensity3_slope[led], Serial_Input_Double("+", 0));
            store(colorcal_intensity3_yint[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("set_colorcal_blanks"):
      {
        for (;;) {
          int led = Serial_Input_Double("+", 0);
          if (led == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (led > 0 || led < NUM_LEDS + 1) {
            store(colorcal_blank1[led], Serial_Input_Double("+", 0));
            store(colorcal_blank2[led], Serial_Input_Double("+", 0));
            store(colorcal_blank3[led], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
          }
        }
      }
      break;
    case hash("set_user_defined"):
      {
        for (;;) {
          int userdefID = Serial_Input_Double("+", 0);
          if (userdefID == -1) {                                    // user can bail with -1+ setting as LED
            break;
          }
          else if (userdefID > 0 || userdefID < 50) {
            store(userdef[userdefID], Serial_Input_Double("+", 0));
          }
          else {
            Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", (sizeof(eeprom->userdef) / sizeof(float)));
          }
        }
      }
      break;

    case hash("read_pin"): {
        int thisPin = Serial_Input_Double("+", 0);
        pinMode(thisPin, INPUT);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
        Serial_Print_Line(analogRead(thisPin));
        delay(200);
      }
      break;

    case hash("digital_read_pin"): {
        int thisPin = Serial_Input_Double("+", 0);
        pinMode(thisPin, INPUT);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
        Serial_Print_Line(digitalRead(thisPin));
        delay(200);
      }
      break;


    case hash("print_magnetometer_bias"):
    case 1051:
      Serial_Printf("Magnetometer Bias: %f, %f, %f", eeprom->mag_bias[0], eeprom->mag_bias[1], eeprom->mag_bias[2]);
      break;

    case hash("print_magnetometer"):
    case 1052:
      Serial_Print_Line("Magnetometer Rotation: ");
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          Serial_Print_Line(eeprom->mag_cal[i][j], 7);
        }
      }
      break;

    case 1053:
      turn_on_5V();                  // turn on 5V to turn on the lights
      {
        int leave = 0;
        while (leave != -1) {
          leave = Serial_Input_Long("+", 1000);
          int magX, magY, magZ, accX, accY, accZ;
          MAG3110_read(&magX, &magY, &magZ);
          MMA8653FC_read(&accX, &accY, &accZ);

          float mag_coords[3] = {(float) magX, (float) magY, (float) magZ};
          applyMagCal(mag_coords);

          int acc_coords[3] = {accX, accY, accZ};
          applyAccCal(acc_coords);

          float roll = getRoll(acc_coords[1], acc_coords[2]);
          float pitch = getPitch(acc_coords[0], acc_coords[1], acc_coords[2], roll);
          float yaw = getCompass(mag_coords[0], mag_coords[1], mag_coords[2], pitch, roll);

          Tilt deviceTilt = calculateTilt(roll, pitch, yaw);

          rad_to_deg(&roll, &pitch, &yaw);


          Serial_Printf("Roll: %f, Pitch: %f, Compass: %f, Compass Direction: ", roll, pitch, yaw);
          Serial_Printf("%s, ", getDirection(compass_segment(yaw)));

          Serial_Printf("Tilt angle: %f, Tilt direction: ", deviceTilt.angle);
          Serial_Print_Line(deviceTilt.angle_direction);
        }

      }
      break;

    case 1054:
      { int leave = 0;
        while (leave != -1) {
          leave = Serial_Input_Long("+", 1000);
          get_compass_and_angle(1, 1);
        }
      }

#if 0
    case hash("collect"):
      Serial_Print("Disconnect the cable");
      delay(5000);
      for (int i = 0; i < 100; i++)
      {
        int x, y, z;
        MAG3110_read(&x, &y, &z);
        dataArray[0][i] = x;
        dataArray[1][i] = y;
        dataArray[2][i] = z;
        delay(100);
      }
      break;

    case hash("dump"):
      for (int i = 0; i < 100; i++) {
        Serial_Printf("%d, %d, %d \n", dataArray[0][i], dataArray[1][i], dataArray[2][i]);
      }
      break;
#endif

    case hash("calibrate_compass"):
      delay(5000);
      Serial_Printf("{\"calibration_values\":");
      Serial_Printf("[");
      for (int i = 0; i < 100; i++) {
        int x, y, z;
        MAG3110_read(&x, &y, &z);
        if (i != 0) {
          Serial_Printf(", ");
        }
        Serial_Printf("%d, %d, %d", x, y, z);
        delay(100);
      }
      Serial_Printf("]");
      Serial_Printf("}");
      Serial_Print_CRC();
      break;

    case hash("upgrade"):
    case 1078:                                                                   // over the air update of firmware.   DO NOT MOVE THIS!
      upgrade_firmware();
      break;

    case 1100:
      break;

    case hash("tcs_length"):
      {
        int before = micros();
        float theValue = get_light_intensity(1);                                                                            // print everything in the eeprom (all values defined in eeprom.h)
        int after = micros();
        Serial_Print_Line("");
        Serial_Print_Line(after - before);
        Serial_Print("tcs_output ");
        Serial_Print_Line(theValue);
      }
      break;

#include "loop-switch-jz.h"

    default:
      Serial_Printf("{\"error\":\"bad command\"}\n");
      break;

  }  // switch()

} // do_command()


// read in and execute a protocol
// example: [{"pulses": [150],"a_lights": [[3]],"a_intensities": [[50]],"pulsedistance": 1000,"m_intensities": [[125]],"pulsesize": 2,"detectors": [[3]],"meas_lights": [[1]],"protocols": 1}]<newline>

void do_protocol()
{

  const int serial_buffer_size = 7000;                                        // max size of the incoming jsons
  const int max_jsons = 15;                                                   // max number of protocols per measurement
  const int MAX_JSON_ELEMENTS = 800;      //

  int averages = 1;                        // ??
  uint8_t spec_on = 0;                    // flag to indicate that spec is being used during this measurement
  float data = 0;
  float data_ref = 0;
  int act_background_light = 0;
  //static float freqtimer0;
  //static float freqtimer1;
  //static float freqtimer2;
  int measurements = 1;                                     // the number of times to repeat the entire measurement (all protocols)
  unsigned long measurements_delay_ms = 0;                  // number of milliseconds to wait between measurements
  unsigned long meas_number = 0;                            // counter to cycle through measurement lights 1 - 4 during the run
  //unsigned long end1;
  //unsigned long start1 = millis();

  // these variables could be pulled from the JSON at the time of use... however, because pulling from JSON is slow, it's better to create a int to save them into at the beginning of a protocol run and use the int instead of the raw hashTable.getLong type call
  int _a_lights [NUM_LEDS] = {};
  int _a_intensities [NUM_LEDS] = {};
  int _a_lights_prev [NUM_LEDS] = {};
  int act_background_light_prev = 0;
  int cycle = 0;                                                                // current cycle number (start counting at 0!)
  int pulse = 0;                                                                // current pulse number
  //int total_cycles;                                                           // Total number of cycles - note first cycle is cycle 0
  int meas_array_size = 0;                                                      // measures the number of measurement lights in the current cycle (example: for meas_lights = [[15,15,16],[15],[16,16,20]], the meas_array_size's are [3,1,3].
  //int end_flag = 0;

  int number_of_protocols = 0;

  // make sure that DEBUG_DC and SS2 are set as an input pin and pulled low.  These are used for factory calibration.

  /*
    pinMode(DEBUG_DC, INPUT);
    digitalWrite(DEBUG_DC, LOW);
    pinMode(SS2, INPUT);
    digitalWrite(SS2, LOW);
  */
  String json2 [max_jsons];     // TODO - don't use String   // will contain each json
  for (int i = 0; i < max_jsons; i++) {
    json2[i] = "";                                                              // reset all json2 char's to zero (ie reset all protocols)
  }

  { // create limited scope for serial_buffer
    char serial_buffer[serial_buffer_size + 1];     // large buffer for reading in a json protocol from serial port

    Serial_Input_Chars(serial_buffer, "\r\n", 500, serial_buffer_size);       // input the protocol

    if (!check_protocol(serial_buffer)) {         // sanity check

      // as `received` is not valid json, this trows out the json parser on the other end
      // would be nice if the json payload was escaped

      //Serial_Print("{\"error\":\"bad json protocol (braces or CRC), received\"");
      //Serial_Print(serial_buffer);
      //Serial_Print("\"}");

      //      Serial_Print("{\"error\":\"bad json protocol (braces or CRC)\"}");
      Serial_Print("{\"error\":\"bad json protocol (braces or CRC)\"              ");
      Serial_Print(serial_buffer);
      Serial_Print("      \"}");
      Serial_Print_CRC();
      Serial_Flush_Output();
      return;
    }

    // break up the protocol into individual jsons

    // TODO improve this - use in place, stretch it in place or copy to new C strings?
    // make json2 an array of char pointers to each protocol
    for (unsigned i = 1; i < strlen(serial_buffer); i++) {         // increments through each char in incoming transmission - if it's open curly, it saves all chars until closed curly.  Any other char is ignored.
      if (serial_buffer[i] == '{') {                               // wait until you see a open curly bracket
        while (serial_buffer[i] != '}') {                          // once you see it, save incoming data to json2 until you see closed curly bracket
          json2[number_of_protocols] += serial_buffer[i];          // add single char to json
          i++;
        }
        json2[number_of_protocols] += serial_buffer[i];           // catch the last closed curly
        number_of_protocols++;
      }
    }  // for

  } // no more need for the serial input buffer

  turn_on_5V();                             // turn on the +5V and analog circuits

  // check battery with load before proceeding
  if (battery_low(1)) {
    Serial_Print("{\"error\":\"battery is too low\"}");
    Serial_Print_CRC();
    Serial_Flush_Output();
    return;
  }

#ifdef DEBUGSIMPLE
  Serial_Printf("got %d protocols\n", number_of_protocols);

  // print each json
  for (int i = 0; i < number_of_protocols; i++) {
    Serial_Printf("Incoming JSON %d as received by Teensy : %s\n", i, json2[i].c_str());
  } // for
#endif

  int v = battery_level(0);   // test without load

  const float min_level = BAT_MIN * 1000;   // consider this min charge level on a lithium battery when loaded
  const float max_level = BAT_MAX * 1000;   // consider this fully charged

  v =  ((v - min_level) / (max_level - min_level)) * 100;     // express as %

  Serial_Printf("{\"device_version\":\"%s\",\"device_id\":\"d4:f5:%2.2x:%2.2x:%2.2x:%2.2x\",\"device_battery\":%d,\"device_firmware\":\"%s\",\"firmware_version\":\"%s\"", DEVICE_VERSION,    // I did this so it would work with chrome app
                (unsigned)eeprom->device_id >> 24,
                ((unsigned)eeprom->device_id & 0xff0000) >> 16,
                ((unsigned)eeprom->device_id & 0xff00) >> 8,
                (unsigned)eeprom->device_id & 0xff, v,
                DEVICE_FIRMWARE, DEVICE_FIRMWARE);

  if (year() >= 2016)
    Serial_Printf(",\"device_time\":%u", now());
  Serial_Print(",\"sample\":[");

  // discharge sample and hold in case the cap is currently charged (on add on and main board)
  digitalWriteFast(HOLDM, HIGH);
  digitalWriteFast(HOLDADD, HIGH);
  delay(10);

  // loop through the all measurements to create a measurement group
  for (int y = 0; y < measurements; y++) {                   // measurements is initially 1, but gets updated after the json is parsed

    Serial_Print("[");                                                                        // print brackets to define single measurement

    for (int q = 0; q < number_of_protocols; q++) {                                           // loop through all of the protocols to create a measurement

      JsonHashTable hashTable;
      JsonParser<MAX_JSON_ELEMENTS> root;
      char json[json2[q].length() + 1];                                     // we need a writeable C string
      strncpy(json, json2[q].c_str(), json2[q].length());
      json[json2[q].length()] = '\0';                                       // Add closing character to char*
      json2[q] = "";                                                        // attempt to release the String memory
      hashTable = root.parseHashTable(json);                                // parse it

      if (!hashTable.success()) {                                           // NOTE: if the incomign JSON is too long (>~5000 bytes) this tends to be where you see failure (no response from device)
        Serial_Print("{\"error\":\"JSON failure with:\"}");
        Serial_Print(json);
        goto abort;
      }

      int protocols = 1;                                                                       // starts as 1 but gets updated when the json is parsed
      int quit = 0;

      for (int u = 0; u < protocols; u++) {                                                    // the number of times to repeat the current protocol
        //uint16_t open_close_start = hashTable.getLong("open_close_start");            // if open_close_start == 1, then the user must open and close the clamp in order to proceed with the measurement (as measured by hall sensor)
        JsonArray save_eeprom    = hashTable.getArray("save");                                  // save values to the eeprom.
        JsonArray recall_eeprom  = hashTable.getArray("recall");                                // userdef values to recall
        JsonArray number_samples = hashTable.getArray("number_samples");                       // number of samples on the cap during sample + hold phase (default is 40);
        JsonArray reference =     hashTable.getArray("reference");                              // subtract reference value if set to 1 (also causes 1/2 the sample rate per detector) due to time constraints.  Default is no reference (== 0)
        uint16_t dac_lights =     hashTable.getLong("dac_lights");                                // when inputting light intensity values, use DAC instead of calibrated outputs (do not use expr() function on coming API data)
        uint16_t adc_show =       hashTable.getLong("adc_show");                                // this tells the MultispeQ to print the ADC values only instead of the normal data_raw (for signal quality debugging)
        uint16_t adc_only[150];                                                   // this and first_adc_ref are used to save the first set of ADC averages produced, so it can be optionally displayed instead of data_raw (for signal quality debugging).  USE ONLY WITH REFERENCE == 0 (ie reference is OFF!)
        JsonArray pulses =        hashTable.getArray("pulses");                                // the number of measuring pulses, as an array.  For example [50,10,50] means 50 pulses, followed by 10 pulses, follwed by 50 pulses.
        String protocol_id =      hashTable.getString("protocol_id");                          // used to determine what macro to apply
        int analog_averages =     hashTable.getLong("analog_averages");                          // DEPRECIATED IN NEWEST HARDWARE 10/14 # of measurements per measurement pulse to be internally averaged (min 1 measurement per 6us pulselengthon) - LEAVE THIS AT 1 for now
        if (analog_averages == 0) {                                                              // if averages don't exist, set it to 1 automatically.
          analog_averages = 1;
        }
        averages =                hashTable.getLong("averages");                               // The number of times to average this protocol.  The spectroscopic and environmental data is averaged in the device and appears as a single measurement.
        if (averages == 0) {                                                                   // if averages don't exist, set it to 1 automatically.
          averages = 1;
        }
        int averages_delay_ms =   hashTable.getLong("averages_delay");                       // same as above but in ms
        measurements =            hashTable.getLong("measurements");                            // number of times to repeat a measurement, which is a set of protocols
        measurements_delay_ms =      hashTable.getLong("measurements_delay");                      // delay between measurements in milliseconds
        protocols =               hashTable.getLong("protocols");                               // delay between protocols within a measurement
        if (protocols == 0) {                                                                   // if averages don't exist, set it to 1 automatically.
          protocols = 1;
        }
        int protocols_delay_ms =     hashTable.getLong("protocols_delay");                         // delay between protocols within a measurement in milliseconds
        if (hashTable.getLong("act_background_light") == 0) {                                    // The Teensy pin # to associate with the background actinic light.  This light continues to be turned on EVEN BETWEEN PROTOCOLS AND MEASUREMENTS.  It is always Teensy pin 13 by default.
          act_background_light =  0;                                                            // change to new background actinic light
        }
        else {
          act_background_light =  hashTable.getLong("act_background_light");                    // DEPRECIATED as of 6/29/216
        }
        //averaging0 - 1 - 30
        //averaging1 - 1 - 30
        //resolution0 - 2 - 16
        //resolution1 - 2 - 16
        //conversion_speed - 0 - 5
        //sampling_speed - 0 - 5
        /*
                int averaging0 =          hashTable.getLong("averaging");                               // # of ADC internal averages
                if (averaging0 == 0) {                                                                   // if averaging0 don't exist, set it to 10 automatically.
                  averaging0 = 10;
                }
                //int averaging1 = averaging0;
                int resolution0 =         hashTable.getLong("resolution");                               // adc resolution (# of bits)
                if (resolution0 == 0) {                                                                   // if resolution0 don't exist, set it to 16 automatically.
                  resolution0 = 16;
                }
                //int resolution1 = resolution0;
                int conversion_speed =    hashTable.getLong("conversion_speed");                               // ADC speed to convert analog to digital signal (5 fast, 0 slow)
                if (conversion_speed == 0) {                                                                   // if conversion_speed don't exist, set it to 3 automatically.
                  conversion_speed = 3;
                }
                int sampling_speed =      hashTable.getLong("sampling_speed");                               // ADC speed of sampling (5 fast, 0 slow)
                if (sampling_speed == 0) {                                                                   // if sampling_speed don't exist, set it to 3 automatically.
                  sampling_speed = 3;
                }
        */
        //        int tcs_to_act =            hashTable.getLong("tcs_to_act");                               // sets the % of response from the tcs light sensor to act as actinic during the run (values 1 - 100).  If tcs_to_act is not defined (ie == 0), then the act_background_light intensity is set to actintensity1.
        //int offset_off =          hashTable.getLong("offset_off");                               // turn off detector offsets (default == 0 which is on, set == 1 to turn offsets off)

        ///*
        JsonArray pulsedistance =   hashTable.getArray("pulse_distance");                            // distance between measuring pulses in us.  Minimum 1000 us.
        JsonArray pulsesize =       hashTable.getArray("pulse_length");                            // pulse width in us.

        JsonArray a_lights =        hashTable.getArray("nonpulsed_lights");
        JsonArray a_intensities =   hashTable.getArray("nonpulsed_lights_brightness");
        JsonArray m_intensities =   hashTable.getArray("pulsed_lights_brightness");

        //        int get_offset =          hashTable.getLong("get_offset");                               // include detector offset information in the output
        // NOTE: it takes about 50us to set a DAC channel via I2C at 2.4Mz.

        JsonArray detectors =     hashTable.getArray("detectors");                               // the Teensy pin # of the detectors used during those pulses, as an array of array.  For example, if pulses = [5,2] and detectors = [[34,35],[34,35]] .
        JsonArray meas_lights =   hashTable.getArray("pulsed_lights");
        JsonArray message =       hashTable.getArray("message");                                // sends the user a message to which they must reply <answer>+ to continue
        //*/
        JsonArray environmental = hashTable.getArray("environmental");
        JsonArray environmental_array = hashTable.getArray("environmental_array");

        // ********************INPUT DATA FOR CORALSPEQ*******************
        JsonArray spec =          hashTable.getArray("spec");                                // defines whether the spec will be called during each array.  note for each single plus, the spec will call and add 256 values to data_raw!
        JsonArray delay_time =    hashTable.getArray("delay_time");                          // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray read_time =     hashTable.getArray("read_time");                           // Amount of time that the analogRead() procedure takes (in microseconds)
        JsonArray intTime =       hashTable.getArray("intTime");                             // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray accumulateMode = hashTable.getArray("accumulateMode");
        //        JsonArray env = hashTable.getArray("env");                    // used to define any environmental sensor readings to be performed once per pulse set "_environmental_array_averages":["light_intensity","temperature"]

        long size_of_data_raw = 0;
        long total_pulses = 0;

        for (int i = 0; i < pulses.getLength(); i++) {                                      // count the number of non zero lights and total pulses
          total_pulses += pulses.getLong(i) * meas_lights.getArray(i).getLength();          // count the total number of pulses
          int non_zero_lights = 0;
          for (int j = 0; j < meas_lights.getArray(i).getLength(); j++) {                   // count the total number of non zero pulses
            if (meas_lights.getArray(i).getLong(j) > 0) {
              non_zero_lights++;
            }
          }

          // redefine the size of data raw to account for the 256 spec measurements per 1 pulse if spec is used (for coralspeq)
          if (spec.getLong(i) == 1) {
            size_of_data_raw += pulses.getLong(i) * non_zero_lights * 256;
          }
          else {
            size_of_data_raw += pulses.getLong(i) * non_zero_lights;
          }

        } // for each pulse

        Serial_Print("{");

        Serial_Print("\"protocol_id\":\"");
        Serial_Print(protocol_id.c_str());
        Serial_Print("\",");

        unsigned long data_raw_average[size_of_data_raw];                                          // buffer for ADC output data
        for (int i = 0; i < size_of_data_raw; ++i)                                                 // zero it
          data_raw_average[i] = 0;

        uint16_t env_counter = 0;
        for (uint16_t i = 0; i < environmental_array.getLength(); i++) {                                                  // identify how many arrays to save for each requested environmental variable, based on the number of outputs per call... so compass yields two arrays ("compass" and "angle")... etc.
          theReadings thisSensor = getReadings(environmental_array.getArray(i).getString(0));
          env_counter = env_counter + thisSensor.numberReadings;                                                     // sum the total number of readings, so we create the right number of arrays.
        }
        //        Serial_Printf("env_counter: %d, size_of_data_raw: %d", env_counter, size_of_data_raw);
        float environmental_array_averages[env_counter][size_of_data_raw];                                         // buffer for each of the environmentals as arrays in environmental_array_averages
        if (env_counter > 0) {                                                                                // if there are environmentals during pulse sets, then generate an array to store the outputs in
          for (uint16_t i = 0; i < env_counter; i++) {
            for (uint16_t j = 0; j < size_of_data_raw; j++) {                                               // initialize the array as zeros
              environmental_array_averages[i][j] = 0;
            }
          }
        }

#ifdef DEBUGSIMPLE
        Serial_Print_Line("");
        Serial_Print("size of data raw:  ");
        Serial_Print_Line(size_of_data_raw);

        Serial_Print_Line("");
        Serial_Print("total number of pulses:  ");
        Serial_Print_Line(total_pulses);

        Serial_Print_Line("");
        Serial_Print("all data in data_raw_average:  ");
        for (int i = 0; i < size_of_data_raw; i++) {
          Serial_Print((unsigned)data_raw_average[i]);
        }

        Serial_Print_Line("");
        Serial_Print("number of pulses:  ");
        Serial_Print_Line(pulses.getLength());

        Serial_Print_Line("");
        Serial_Print("arrays in meas_lights:  ");
        Serial_Print_Line(meas_lights.getLength());

        Serial_Print_Line("");
        Serial_Print("length of meas_lights arrays:  ");
        for (int i = 0; i < meas_lights.getLength(); i++) {
          Serial_Print(meas_lights.getArray(i).getLength());
          Serial_Print(", ");
        }
        Serial_Print_Line("");
#endif
        /*
                if (get_offset == 1) {
                  print_offset(1);
                }
        */

        if (averages > 1) {
          Serial_Print("\"averages\":");
          Serial_Print(averages);
          Serial_Print(",");
        }

        //        print_sensor_calibration(1);                                               // print sensor calibration data

        // clear variables (many not needed)

        light_intensity = light_intensity_averaged = 0;
        light_intensity_raw = light_intensity_raw_averaged = 0;
        r = r_averaged =  g = g_averaged = b = b_averaged = 0;

        thickness = thickness_averaged = 0;
        thickness_raw = thickness_raw_averaged = 0;

        contactless_temp = contactless_temp_averaged = 0;

        compass = compass_averaged = 0;
        x_compass_raw = 0, y_compass_raw = 0, z_compass_raw = 0;
        x_compass_raw_averaged = 0, y_compass_raw_averaged = 0, z_compass_raw_averaged = 0;

        angle = 0;
        angle_averaged = 0;
        angle_direction = "";
        roll = roll_averaged = 0;
        pitch = pitch_averaged = 0;
        x_tilt = 0, y_tilt = 0, z_tilt = 0;
        x_tilt_averaged = 0, y_tilt_averaged = 0, z_tilt_averaged = 0;

        temperature = humidity = pressure = 0;
        temperature_averaged = humidity_averaged = pressure_averaged = 0;

        temperature2 = humidity2 = pressure2 = 0;
        temperature2_averaged = humidity2_averaged = pressure2_averaged = 0;

        co2 = co2_averaged = 0;

        detector_read1 = detector_read1_averaged = 0;
        detector_read2 = detector_read2_averaged = 0;
        detector_read3 = detector_read3_averaged = 0;

        analog_read = digital_read = adc_read = adc_read2 = adc_read3 = 0;
        analog_read_averaged = digital_read_averaged = adc_read_averaged = adc_read2_averaged = adc_read3_averaged = 0;

        if (hashTable.getLong("open_close_start") == 1) {                                     // wait for device to open (read hall sensor), then close before proceeding with protocol
          start_on_open_close();
        }

        if (hashTable.getLong("pin_high_start") != 0) {                                     // wait for device to open (read hall sensor), then close before proceeding with protocol
          start_on_pin_high(hashTable.getLong("pin_high_start"));
        }

        // perform the protocol averages times
        for (int x = 0; x < averages; x++) {                                                 // Repeat the protocol this many times

          if (Serial_Available() && Serial_Input_Long("+", 1) == -1) {        // test for abort command
            q = number_of_protocols - 1;
            y = measurements - 1;
            u = protocols;
            x = averages;
          }

          int background_on = 0;
          long data_count = 0;
          int message_flag = 0;                                                              // flags to indicate if an alert, prompt, or confirm have been called at least once (to print the object name to data JSON)
          unsigned _pulsedistance = 0;                                                  // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
          unsigned _pulsedistance_prev = 0;
          uint16_t _reference_flag = 0;                                                           // used to note if this is the first measurement
          float _reference_start = 0;                                                            // reference value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          float _main_start = 0;                                                               // main detector (sample) value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          uint16_t _number_samples = 0;                                                               // create the adc sampling rate number

          environmentals(environmental, averages, x, 0);

          for (int z = 0; z < total_pulses; z++) {                                      // cycle through all of the pulses from all cycles
            int first_flag = 0;                                                           // flag to note the first pulse of a cycle
            int _spec = 0;                                                              // create the spec flag for the coralspeq

            int _intTime = 0;                                                           // create the _intTime flag for the coralspeq
            int _delay_time = 0;                                                        // create the _delay_time flag for the coralspeq
            int _read_time = 0;                                                         // create the _read_time flag for the coralspeq
            int _accumulateMode = 0;                                                    // create the _accumulateMode flag for the coralspeq

            if (pulse == 0) {                                                                                     // if it's the first pulse of a cycle, we need to set up the new set of lights and intensities...
              meas_array_size = meas_lights.getArray(cycle).getLength();                                          // get the number of measurement/detector subsets in the new cycle

              if (PULSERDEBUG) {
                Serial_Printf("\n _number_samples: %d \n", _number_samples);
              } // PULSERDEBUG

              for (unsigned i = 0; i < NUM_LEDS; i++) {                                  // save the list of act lights in the previous pulse set to turn off later
                _a_lights_prev[i] = _a_lights[i];
                if (PULSERDEBUG) {
                  Serial_Printf("\n all a_lights_prev: %d\n", _a_lights_prev[i]);
                } // PULSERDEBUG
              }

              for (unsigned i = 0; i < NUM_LEDS; i++) {                                   // save the current list of act lights, determine if they should be on, and determine their intensity
                _a_lights[i] = a_lights.getArray(cycle).getLong(i);                        // save which light should be turned on/off
                String intensity_string = a_intensities.getArray(cycle).getString(i);
                _a_intensities[i] = expr(intensity_string.c_str());                       // evaluate as an expression

                if (PULSERDEBUG) {
                  Serial_Printf("\n all a_lights, intensities: %d,%d,|%s|,%f,%f,%f\n", _a_lights[i], _a_intensities[i], intensity_string.c_str(), expr(intensity_string.c_str()), light_intensity, light_intensity_averaged);
                } // PULSERDEBUG
              }

              if (CORAL_SPEQ) {
                _spec = spec.getLong(cycle);                                                      // pull whether the spec will get called in this cycle or not for coralspeq and set parameters.  If they are empty (not defined by the user) set them to the default value
                if (_spec == 1) {
                  _intTime = intTime.getLong(cycle);
                  if (_intTime == 0) {
                    _intTime = 100;
                  }
                  _delay_time = delay_time.getLong(cycle);
                  if (_delay_time == 0) {
                    _delay_time = 35;
                  }
                  _read_time = read_time.getLong(cycle);
                  if (_read_time == 0) {
                    _read_time = 35;
                  }
                  _accumulateMode = accumulateMode.getLong(cycle);
                  if (_accumulateMode == 0) {
                    _accumulateMode = false;
                  }
                }
              } // CORAL_SPEQ

              if (cycle != 0) {
                _pulsedistance_prev = _pulsedistance;
              }
              String distanceString = pulsedistance.getString(cycle);                                                    // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
              _pulsedistance = expr(distanceString.c_str());                                                    // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
              first_flag = 1;                                                                                   // flip flag indicating that it's the 0th pulse and a new cycle
              if (cycle == 0) {                                                                                 // if it's the beginning of a measurement (cycle == 0 and pulse == 0), then...
                digitalWriteFast(act_background_light_prev, LOW);                                               // turn off actinic background light and...
                startTimers(_pulsedistance);                                                                    // Use one ISR to turn on and off the measuring lights.
              }
              else if (cycle != 0 && (_pulsedistance != _pulsedistance_prev)) {    // if it's not the 0th cycle and the last pulsesize or pulsedistance was different than the current one, then stop the old timers and set new ones.   If they were the same, avoid resetting the timers by skipping this part.
                startTimers(_pulsedistance);                                    // restart the measurement light timer
              }

            }  // if pulse == 0


            _number_samples = number_samples.getLong(cycle);                                               // set the _number_samples for this cycle
            //            assert(_number_samples >= 0 && _number_samples < 500);

            _meas_light = meas_lights.getArray(cycle).getLong(meas_number % meas_array_size);             // move to next measurement light
            String intensity_string = m_intensities.getArray(cycle).getString(meas_number % meas_array_size);          // evaluate inputted intensity to see if it was an expression
            uint16_t _m_intensity = expr(intensity_string.c_str());
            //            assert(_m_intensity >= 0 && _m_intensity <= 4095);

            uint16_t detector = detectors.getArray(cycle).getLong(meas_number % meas_array_size);          // move to next detector
            //            assert(detector >= 1 && detector <= 10);

            uint16_t _reference = reference.getArray(cycle).getLong(meas_number % meas_array_size);

            String sizeString = pulsesize.getArray(cycle).getString(meas_number % meas_array_size);     // set the pulse size for the next light
            _pulsesize = expr(sizeString.c_str());

            if (_number_samples == 0) {                                                                    // if _number_samples wasn't set or == 0, set it automatically to 19 (default)
              _number_samples = 19;
            }
            if (_reference != 0) {                                                                      // if using the reference detector, make sure to half the sample rate (1/2 sample from main, 1/2 sample from detector)
              _number_samples = _number_samples / 2;
            }

            if (_reference == 0) {                                                                      // If the reference detector isn't turned on, then we need to set the ADC first
              AD7689_set (detector - 1);        // set ADC channel as specified
            }

            if (PULSERDEBUG) {
              Serial_Printf("measurement light, intensity, detector, reference:  %d, %d, %d, %d\n", _meas_light, _m_intensity, detector, _reference);
              Serial_Printf("pulsedistance = %d, pulsesize = %d, cycle = %d, measurement number = %d, measurement array size = %d,total pulses = %d\n", (int) _pulsedistance, (int) _pulsesize, (int) cycle, (int) meas_number, (int) meas_array_size, (int) total_pulses);
            } // PULSERDEBUG

            if (pulse < meas_array_size) {   // if it's the first pulse of a cycle, then change act 1,2,3,4 values as per array's set at beginning of the file

              if (pulse == 0) {
                String _message_type = message.getArray(cycle).getString(0);                                // get what type of message it is
                if ((_message_type != "" || quit == -1) && x == 0) {                                         // if there are some messages or the user has entered -1 to quit AND it's the first repeat of an average (so it doesn't ask these question on every average), then print object name...
                  if (message_flag == 0) {                                                                 // if this is the first time the message has been printed, then print object name
                    Serial_Print("\"message\":[");
                    message_flag = 1;
                  }
                  Serial_Print("[\"");
                  Serial_Print(_message_type.c_str());                                                               // print message
                  Serial_Print("\",");
                  Serial_Print("\"");
                  Serial_Print(message.getArray(cycle).getString(1));
                  Serial_Print("\",");
                  if (_message_type == "0") {
                    Serial_Print("\"\"]");
                  }
                  else if (_message_type == "alert") {                                                    // wait for user response to alert
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    while (1) {
                      long response = Serial_Input_Long("+", 0);
                      if (response == -1) {
                        Serial_Print("\"ok\"]");
                        break;
                      }
                    }
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  else if (_message_type == "confirm") {                                                  // wait for user's confirmation message.  If enters '1' then skip to end.
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    while (1) {
                      long response = Serial_Input_Long("+", 0);
                      if (response == 1) {
                        Serial_Print("\"cancel\"]]");                                                     // set all loops (protocols, measurements, averages, etc.) to the last loop value so it exits gracefully
                        q = number_of_protocols;
                        y = measurements - 1;
                        u = protocols - 1;
                        x = averages;
                        z = total_pulses;
                        break;
                      }
                      if (response == -1) {
                        Serial_Print("\"ok\"]");
                        break;
                      }
                    }
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  else if (_message_type == "prompt") {                                                    // wait for user to input information, followed by +
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    char response[150];
                    Serial_Input_Chars(response, "+", 0, sizeof(response));
                    Serial_Print("\"");
                    Serial_Print(response);
                    Serial_Print("\"]");
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  if (cycle != pulses.getLength() - 1) {                                                  // if it's not the last cycle, then add comma
                    Serial_Print(",");
                  }
                  else {                                                                                 // if it is the last cycle, then close out the array
                    Serial_Print("],");
                  }
                }
                delayMicroseconds(200);
              }

              // calculate_intensity(_meas_light, tcs_to_act, cycle, _light_intensity);                   // in addition, calculate the intensity of the current measuring light

              //            Serial_Printf("_meas_light = %d, par_to_dac = %d, _m_intensity = %d, dac_lights = %d\n",_meas_light,par_to_dac(_m_intensity, _meas_light),_m_intensity,dac_lights);

              if (!dac_lights)                                                     // evaluate as an expression...
                DAC_set(_meas_light, par_to_dac(_m_intensity, _meas_light));       // set the DAC, make sure to convert PAR intensity to DAC value
              else                                                                 // otherwise evaluate directly as a number to enter into the DAC
                DAC_set(_meas_light, _m_intensity);                                // set the DAC, make sure to convert PAR intensity to DAC value

              for (unsigned i = 0; i < NUM_LEDS; i++) {                         // set the DAC lights for actinic lights in the current pulse set
                if (_a_lights[i] != 0) {                                        // if there's a light there, then change it, otherwise skip
                  if (!dac_lights) {                                                                            // evaluate as an expression...
                    DAC_set(_a_lights[i], par_to_dac(_a_intensities[i], _a_lights[i]));
                  }
                  else {                                                                                      // otherwise evaluate directly as a number to enter into the DAC
                    DAC_set(_a_lights[i], _a_intensities[i]);
                  }
                  if (PULSERDEBUG) {
                    Serial_Printf("actinic pin : %d \nactinic intensity %d \n", _a_lights[i], _a_intensities[i]);
                    Serial_Printf("length of _a_lights : %d \n ", sizeof(_a_lights));
                    Serial_Printf("\n _number_samples, _reference, adc_show: %d %d %d\n", _number_samples, _reference, adc_show);
                  } // PULSERDEBUG
                }
              } // for

              DAC_change();                                                        // send values to DAC

            }  // if (pulse < meas_array_size)

            if (Serial_Available() && Serial_Input_Long("+", 1) == -1) {                                      // exit protocol completely if user enters -1+
              q = number_of_protocols;
              y = measurements - 1;
              u = protocols - 1;
              x = averages;
              z = total_pulses;
            }

            uint16_t sample_adc[_number_samples];                                                             // initialize the variables to hold the main and reference detector data
            uint16_t sample_adc_ref[_number_samples];
            //            uint16_t startTimer;                                                                            // to measure the actual time it takes to perform the ADC reads on the sample (for debugging)
            //            uint16_t endTimer;

            pulse_done = 0;             // clear volatile ISR done flag
            while (!pulse_done) {       // wait for LED pulse complete (in ISR)
              //if (abort_cmd())
              //  goto abort;  // or just reboot?
              //sleep_cpu();     // save power - removed, causes race condition
            }

            if (_reference != 0) {
              AD7689_read_arrays((detector - 1), sample_adc, (_reference - 1), sample_adc_ref, _number_samples); // also reads reference detector - note this function takes detectors 0 - 3, so must subtract detector value by 1
            }
            else {
              AD7689_read_array(sample_adc, _number_samples);                                              // just reads the detector defined by the user
            }

            interrupts();                                             // re-enable interrupts (left off after LED ISR)

            digitalWriteFast(HOLDM, HIGH);                            // discharge integrators
            digitalWriteFast(HOLDADD, HIGH);

            if (env_counter > 0) {                                                                                // check to see if there are any objects in environmental array (this saves some time as calling .getLength on environmental_array can take some time)
              environmentals(environmental_array, averages, x, 1);
              uint16_t counter1 = 0;
              for (uint16_t i = 0; i < environmental_array.getLength(); i++) {
                theReadings thisSensor = getReadings(environmental_array.getArray(i).getString(0));
                environmental_array_averages[counter1][z] = expr(thisSensor.reading1);
                counter1++;
                //                Serial_Printf("counter reading 1:%d, array: %s, value: %f\n", counter1, thisSensor.reading1, expr(thisSensor.reading1));
                if (thisSensor.numberReadings >= 2) {                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                  environmental_array_averages[counter1][z] = expr(thisSensor.reading2);
                  counter1++;
                  //                  Serial_Printf("counter reading 2:%d, array: %s, value: %f\n", counter1, thisSensor.reading1, expr(thisSensor.reading2));
                }
                if (thisSensor.numberReadings >= 3) {                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                  environmental_array_averages[counter1][z] = expr(thisSensor.reading3);
                  counter1++;
                  //                  Serial_Printf("counter reading 3:%d, array: %s, value: %f\n", counter1, thisSensor.reading1, expr(thisSensor.reading3));
                }
                if (thisSensor.numberReadings >= 4) {                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                  environmental_array_averages[counter1][z] = expr(thisSensor.reading4);
                  counter1++;
                  //                  Serial_Printf("counter reading 4:%d, array: %s, value: %f\n", counter1, thisSensor.reading1, expr(thisSensor.reading4));
                }
                if (thisSensor.numberReadings >= 5) {                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                  environmental_array_averages[counter1][z] = expr(thisSensor.reading5);
                  counter1++;
                  //                  Serial_Printf("counter reading 5:%d, array: %s, value: %f\n", counter1, thisSensor.reading1, expr(thisSensor.reading5));
                }
              }
            }

            if (adc_show == 1) {                                                                        // save the individual ADC measurements separately if adc_show is set to 1 (to be printed to the output later instead of data_raw)
              //              Serial_Print(",\"last_adc_ref\":[");
              for (unsigned i = 0; i < sizeof(sample_adc) / sizeof(uint16_t); i++) {                           //
                adc_only[i] = sample_adc[i];
                //                Serial_Print(adc_only[i]);
                //                if (i != sizeof(sample_adc)/sizeof(uint16_t) - 1) {
                //                  Serial_Print(",");
                //                }
              }
              //              Serial_Print("],");
            }

            data = median16(sample_adc, _number_samples);                                  // using median - 25% improvements over using mean to determine this value

            if (_reference != 0) {                                                        // if also using reference, then ...
              data_ref = median16(sample_adc_ref, _number_samples);                       // generate median of reference values
              if (_reference_flag == 0) {                                                 // if this is the first pulse set which uses the reference, then flip the flag noting that a reference measurement has been taken, and collect the main and reference detectors starting measurements for normalization
                _reference_start = data_ref;
                _main_start = data;
                _reference_flag = 1;
              }
            }
            if (_reference != 0) {                                                        // now calculate the outputted data value based on the main and reference values, and the initial main and reference values for normalization
              if (PULSERDEBUG) {
                Serial_Printf("reference_start = %d, reference_now = %d,       _main_start = %d, main_now = %d", (int) _reference_start, (int) data_ref, (int) _main_start, (int) data);
              } // PULSERDEBUG

              data = data - _main_start * (data_ref - _reference_start) / _reference_start; // adjust main value according to the % change in the reference relative to main on the first pulse.  You adjust the main in the opposite direction of the reference (to compensate)

              if (PULSERDEBUG) {
                float changed = (data_ref - _reference_start) / _reference_start;
                Serial_Printf(",      main = %d, ref = %d, data_normalized = %f, percent_change = %f\n", detector, _reference, data, changed);
              } // PULSERDEBUG
            }

            if (PULSERDEBUG) {        // use this to see all of the adc reads which are being averaged
              //Serial_Printf("median + first value :%d,%d", data, sample_adc[5]);
              //Serial_Printf("median + first value reference :%d,%d", data_ref, sample_adc_ref[5]);
            } // PULSERDEBUG

            if (first_flag == 1) {                                                                    // if this is the 0th pulse and a therefore new cycle
              for (unsigned i = 0; i < NUM_LEDS; i++) {                            // Turn off all of the previous actinic lights
                if (_a_lights_prev[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights_prev[i]], LOW);
                  if (PULSERDEBUG) {
                    Serial_Printf("turned off actinic light: %d\n", LED_to_pin[_a_lights_prev[i]]);
                  } // PULSERDEBUG
                }
              }
              DAC_change();                                                                               // initiate actinic lights which were set above

              for (unsigned i = 0; i < NUM_LEDS; i++) {                            // Turn on all the new actinic lights for this pulse set
                if (_a_lights[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights[i]], HIGH);
                  if (PULSERDEBUG) {
                    Serial_Printf("turned on new actinic light: %d\n", LED_to_pin[_a_lights[i]]);
                  } // PULSERDEBUG
                }
              }
              first_flag = 0;                                                              // reset flag
            }

            float _offset = 0;
            /*
                        if (offset_off == 0) {
                          switch (detector) {                                                          // apply offset to whicever detector is being used
                            case 34:
                              _offset = offset_34;
                              break;
                            case 35:
                              _offset = offset_35;
                              break;
                          }
                        }
            */

#ifdef DEBUGSIMPLE
            Serial_Print("data count, size of raw data                                   ");
            Serial_Print((int)data_count);
            Serial_Print(",");
            Serial_Print_Line(size_of_data_raw);
#endif

            if (_spec != 1) {                                                    // if spec_on is not equal to 1, then coralspeq is off and proceed as per normal MultispeQ measurement.
              if (_meas_light  != 0) {                                                      // save the data, so long as the measurement light is not equal to zero.
                data_raw_average[data_count] += data - _offset;
                data_count++;
              }
            }
            else if (_spec == 1) {                                              // if spec_on is 1 for this cycle, then collect data from the coralspeq and save it to data_raw_average.
              readSpectrometer(_intTime, _delay_time, _read_time, _accumulateMode);                                                        // collect a reading from the spec
              for (int i = 0 ; i < SPEC_CHANNELS; i++) {
                data_raw_average[data_count] += spec_data[i];
                data_count++;
              }
            }

            pulse++;                                                                     // progress the pulse counter and measurement number counter

#ifdef DEBUGSIMPLE
            Serial_Print("data point average, current data                               ");
            Serial_Print((int)data_raw_average[meas_number]);
            Serial_Print("!");
            Serial_Print_Line(data);
#endif
            meas_number++;                                                              // progress measurement number counters

            if (pulse == pulses.getLong(cycle)*meas_lights.getArray(cycle).getLength()) { // if it's the last pulse of a cycle...
              pulse = 0;                                                               // reset pulse counter
              cycle++;                                                                 // ...move to next cycle
            }

          }  // for pulses z

          background_on = 0;
          /*
                    background_on = calculate_intensity_background(act_background_light, tcs_to_act, cycle, _light_intensity, act_background_light_intensity); // figure out background light intensity and state
          */

          for (unsigned i = 0; i < NUM_LEDS; i++) {
            if (_a_lights[i] != act_background_light) {                                  // turn off all lights unless they are the actinic background light
              digitalWriteFast(LED_to_pin[_a_lights[i]], LOW);
            }
          }

          if (background_on == 1) {
            DAC_change();                                                                               // initiate actinic lights which were set above
            digitalWriteFast(act_background_light, HIGH);                                // turn on actinic background light in case it was off previously.
          }
          else {
            digitalWriteFast(act_background_light, LOW);                                // turn on actinic background light in case it was off previously.
          }

          stopTimers();
          cycle = 0;                                                                     // ...and reset counters
          pulse = 0;
          meas_number = 0;

          /*
            options for relative humidity, temperature, contactless temperature. light_intensity,co2
            0 - take before spectroscopy measurements
            1 - take after spectroscopy measurements
          */

          if (x + 1 < averages) {                                                             //  to next average, unless it's the end of the very last run
            if (averages_delay_ms > 0) {
              Serial_Input_Long("+", averages_delay_ms);
            }
          }

        }  // for each protocol repeat x for averages

        /*
           Recall and save values to the eeprom
        */

        recall_save(recall_eeprom, save_eeprom);                                                    // Recall and save values to the eeprom.

        if (spec_on == 1) {                                                                    // if the spec is being used, then read it and print data_raw as spec values.  Otherwise, print data_raw as multispeq detector values as per normal
          Serial_Print("\"data_raw\":[");
          for (int i = 0; i < SPEC_CHANNELS; i++) {
            Serial_Print((unsigned)(spec_data_average[i] / averages));
            if (i != SPEC_CHANNELS - 1) {                                                     // if it's the last one in printed array, don't print comma
              Serial_Print(",");
            }
          } // for
          Serial_Print("]}");
        }

        uint16_t counter1 = 0;
        for (uint16_t i = 0; i < environmental_array.getLength(); i++) {                        // print the environmental_array data
          theReadings thisSensor = getReadings(environmental_array.getArray(i).getString(0));
          for (uint16_t g = 0; g < thisSensor.numberReadings; g++) {              // print the appropriate sensor value
            switch (g) {
              case 0:                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                Serial_Printf("\"%s\":[", thisSensor.reading1);
                break;
              case 1:                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                Serial_Printf("\"%s\":[", thisSensor.reading2);
                break;
              case 2:                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                Serial_Printf("\"%s\":[", thisSensor.reading3);
                break;
              case 3:                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                Serial_Printf("\"%s\":[", thisSensor.reading4);
                break;
              case 4:                                                              // so if the number of readings is 1 or fewer, do this, next if statement is for 2 or fewer, etc. etc.
                Serial_Printf("\"%s\":[", thisSensor.reading5);
                break;
            }
            for (uint16_t j = 0; j < size_of_data_raw; j++) {
              if (j != size_of_data_raw - 1) {
                Serial_Printf("\"%f\",", environmental_array_averages[counter1][j]);
              }
              else {
                Serial_Printf("\"%f\"],", environmental_array_averages[counter1][j]);
              }
            }
            counter1++;
          }
        }

        // print the data

        if (spec_on == 0) {
          Serial_Print("\"data_raw\":[");
          if (adc_show == 0) {                                                             // normal condition - show data_raw as per usual
            for (int i = 0; i < size_of_data_raw; i++) {                                     // print data_raw, divided by the number of averages
              Serial_Print((unsigned)(data_raw_average[i] / averages));
              // if average = 1, then it might be better to print data as it is collected
              if (i != size_of_data_raw - 1) {
                Serial_Print(",");
              } // if
            } // for
          } else {                                                                         // if adc_show == 1, show first individual adc's only - do not show normal data_raw (used for signal debugging only)
            for (int i = 0; i < number_samples.getLong(0); i++) {
              Serial_Print(adc_only[i]);
              if (i != number_samples.getLong(0) - 1) {
                Serial_Print(",");
              }
            } // for
          }

          Serial_Print("]}");
        }


#ifdef DEBUGSIMPLE
        Serial_Print("# of protocols repeats, current protocol repeat, number of total protocols, current protocol      ");
        Serial_Print(protocols);
        Serial_Print(",");
        Serial_Print(u);
        Serial_Print(",");
        Serial_Print(number_of_protocols);
        Serial_Print(",");
        Serial_Print_Line(q);
#endif

        if (q < number_of_protocols - 1 || u < protocols - 1) {                           // if it's not the last protocol in the measurement and it's not the last repeat of the current protocol, add a comma
          Serial_Print(",");
          if (protocols_delay_ms > 0) {
            Serial_Input_Long("+", protocols_delay_ms);
          }
        }
        else if (q == number_of_protocols - 1 && u == protocols - 1) {                  // if it is the last protocol, then close out the data json
          Serial_Print("]");
        }

        averages = 1;                                                 // number of times to repeat the entire run
        averages_delay_ms = 0;                                                    // seconds wait time between averages
        analog_averages = 1;                                                             // # of measurements per pulse to be averaged (min 1 measurement per 6us pulselengthon)
        for (unsigned i = 0; i < NUM_LEDS; i++) {
          _a_lights[i] = 0;
        }

        if (CORAL_SPEQ) {
          for (int i = 0; i < SPEC_CHANNELS; i++)
            spec_data_average [i] = 0;
        } // CORAL_SPEQ

        act_background_light_prev = act_background_light;                               // set current background as previous background for next protocol
        spec_on = 0;                                                                    // reset flag that spec is turned on for this measurement

      }  // for each protocol repeat u

    }  // for each protocol q

    // [{      "environmental":[["light_intensity",0]],"pulses": [100,100,100],"a_lights": [[2],[2],[2]],"a_intensities": [["light_intensity_averaged"],[1000],["light_intensity_averaged"]],"pulsedistance": [10000,10000,10000],"m_intensities": [[500],[500],[500]],"pulsesize": [60,60,60],"detectors": [[1],[1],[1]],"meas_lights": [[3],[3],[3]],"averages": 1}]

    Serial_Flush_Input();
    if (y < measurements - 1) {                                 // if not last measurement
      Serial_Print(",");                                        // add commas between measurements
      if (measurements_delay_ms > 0) {
        Serial_Input_Long("+", measurements_delay_ms);
      }
    } // if

  }  // for each measurement y

abort:

  Serial_Print("]}");                // terminate output json
  Serial_Print_CRC();             // TODO put this back in one android app is fixed
  Serial_Flush_Output();

  act_background_light = 0;          // ??

  // turn off all lights (just in case)
  for (unsigned i = 1; i <= NUM_LEDS; i++) {
    digitalWriteFast(LED_to_pin[i], LOW);
  }

  return;

} // do_protocol()

//  routines for LED pulsing

static void pulse3() {                      // ISR to turn on/off LED pulse - also controls integration switch

  if (pulse_done)                           // skip this pulse if not ready yet
    return;

  const unsigned  STABILIZE = 10;                // this delay gives the LED current controller op amp the time needed to stabilize
  register int pin = LED_to_pin[_meas_light];
  register int pulse_size = _pulsesize;

  noInterrupts();
  digitalWriteFast(pin, HIGH);           // turn on measuring light
  delayMicroseconds(STABILIZE);          // this delay gives the LED current controller op amp the time needed to turn
  // the light on completely + stabilize.
  // Very low intensity measuring pulses may require an even longer delay here.
  digitalWriteFast(HOLDADD, LOW);        // turn off sample and hold discharge
  digitalWriteFast(HOLDM, LOW);          // turn off sample and hold discharge
  delayMicroseconds(pulse_size);         // pulse width
  digitalWriteFast(pin, LOW);            // turn off measuring light
  pulse_done = 1;                        // indicate that we are done
  // NOTE:  interrupts are left off and must be re-enabled
}

// schedule the turn on and off of the LED(s) via a single ISR

static IntervalTimer timer0;

inline static void startTimers(unsigned _pulsedistance) {
  timer0.begin(pulse3, _pulsedistance);             // schedule pulses
}

inline static void stopTimers() {
  timer0.end();                         // if it's the last cycle and last pulse, then... stop the timers
}

// write userdef values to eeprom
// example json for save: [{"save":[[1,3.43],[2,5545]]}]  for userdef[1] = 3.43 and userdef[2] = 5545
// read userdef and other values from eeprom
// example json for read: [{"recall":["light_slope_all","userdef[1]"]}]

static void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom) {
  int number_saves = _save_eeprom.getLength();                                // define these explicitly to make it easier to understand the logic
  int number_recalls = _recall_eeprom.getLength();                            // define these explicitly to make it easier to understand the logic

  for (int i = 0; i < number_saves; i++) {                             // do any saves
    long location = _save_eeprom.getArray(i).getLong(0);
    //    double value_to_save = _save_eeprom.getArray(i).getDouble(1);
    String value_to_save = _save_eeprom.getArray(i).getString(1);
    if (location >= 0 && location <= (long)NUM_USERDEFS)
      store(userdef[location], expr(value_to_save.c_str()));                         // save new value in the defined eeprom location
  }

  if (number_recalls > 0) {                  // if the user is recalling any saved eeprom values then...
    Serial_Print("\"recall\":{");                                                       // then print the eeprom location number and the value located there

    for (int i = 0; i < number_recalls; i++) {

      String recall_string = _recall_eeprom.getString(i);

      Serial_Printf("\"%s\":%f", recall_string.c_str(), expr(recall_string.c_str()));

      if (i != number_recalls - 1) {
        Serial_Print(",");
      }
      else {
        Serial_Print("},");
      }
    } // for
  } // if
} // recall_save()

// return true if a Ctrl-A character has been typed
int abort_cmd()
{
  return 0;    // TODO
}

/*
   The structure of the sensor calls follows these general rules:
   1) user enters "environmental":[[...],[...]] structure into API, where ... is the call ("light_intensity" or "thickness" for example), and followed by a 0 or 1 (0 if it's before the main spec measurement, or 1 if it's after)
      An example - "environmental":[["tilt",1],["light_intensity",0]] would call tilt after the measurement, and light intensity before the measurement.
   2) Sensor data gets called in 3 ways: either inside the measurement as part of an expression (maybe "a_intensities":[light_intensity/2]), after a measurement or set of averaged measurements.
      In addition, there are often raw and calibrated versions of sensors, like raw tcs value versus the PAR value, or raw hall sensor versus calibrated thickness.
      As a result, for most sensors there is the base version (like x_tilt) which is available in that measurment, an averaged version (like x_tilt_averaged) which is outputted after averaging, and raw versions of each of those (x_tilt and x_tilt_averaged)
*/

//get_temperature_humidity_pressure

void get_temperature_humidity_pressure (int _averages) {    // read temperature, relative humidity, and pressure BME280 module

  temperature = bme1.readTemperature();                // temperature in C
  humidity = bme1.readHumidity();                      // humidity in %
  pressure = bme1.readPressure() / 100;               // pressure in millibar

  temperature_averaged += temperature / _averages;                // same as above, but averaged if API requests averaging
  humidity_averaged += humidity / _averages;
  pressure_averaged += pressure / _averages;

  //  sensorValues thisSensor = {temperature, humidity, pressure};
}

void get_temperature_humidity_pressure2 (int _averages) {    // read temperature, relative humidity, and pressure BME280 module

  temperature2 = bme2.readTemperature();
  humidity2 = bme2.readHumidity();
  pressure2 = bme2.readPressure() / 100;

  temperature2_averaged += temperature2 / _averages;                // collect temp, rh, and humidity data (averaged if API requests averaging)
  humidity2_averaged += humidity2 / _averages;
  pressure2_averaged += pressure2 / _averages;

}

void get_detector_value (int _averages, int this_light, int this_intensity, int this_detector, int this_pulsesize, int detector_read1or2or3) {    // read reflectance of LED 5 (940nm) with 5 pulses and averages.  Keep # of pulses low as a large number of pulses could impact the sample

  const unsigned  STABILIZE = 10;                                           // this delay gives the LED current controller op amp the time needed to stabilize
  uint16_t thisData[5];
  DAC_set(this_light, par_to_dac(this_intensity, this_light));              // set the DAC, make sure to convert PAR intensity to DAC value
  DAC_change();
  AD7689_set (this_detector - 1);                                           // set ADC channel as specified
  delay(50);                                                                // this is required for AD7689 to settle.  Otherwise, values from the following begin high and fall as it runs through the 5 repeats

  for (uint16_t i = 0; i < 5; i++) {
    delayMicroseconds(1000);                                  // wait until next pulse, pulse distance == 1000
    uint16_t this_sample_adc[19];                                              // initialize the variables to hold the main and reference detector data
    noInterrupts();
    digitalWriteFast(LED_to_pin[this_light], HIGH);            // turn on measuring light
    delayMicroseconds(STABILIZE);           // this delay gives the LED current controller op amp the time needed to turn
    // the light on completely + stabilize.
    // Very low intensity measuring pulses may require an even longer delay here.
    digitalWriteFast(HOLDADD, LOW);        // turn off sample and hold discharge
    digitalWriteFast(HOLDM, LOW);          // turn off sample and hold discharge
    delayMicroseconds(this_pulsesize);         // pulse width
    digitalWriteFast(LED_to_pin[this_light], LOW);            // turn off measuring light
    if (i >= 1 && i <= 4) {               // skip the first and last because I'm paranoid to make 48 total samples :)
      AD7689_read_array(this_sample_adc, 19);                                              // read detector, 19 values and save them in this_sample_adc
      thisData[i - 1] = median16(this_sample_adc, 19);
      /*
            for (int i = 0; i < 19; i++) {
              Serial.print(this_sample_adc[i]);
              Serial.print(",");
            }
      */
    }
    //    Serial_Print_Line("");
    interrupts();                                             // re-enable interrupts (left off after LED ISR)
    digitalWriteFast(HOLDM, HIGH);                            // discharge integrators
    digitalWriteFast(HOLDADD, HIGH);
  }
  if (detector_read1or2or3 == 1) {                                                                 // save in detector_read1 or 2 depending on which is called
    detector_read1 = median16(thisData, 4);                                            // using median - 25% improvements over using mean to determine this value
    detector_read1_averaged += detector_read1 / _averages;
    //    Serial_Printf("read1: %f\n",detector_read1);
  }
  else if (detector_read1or2or3 == 2) {
    detector_read2 = median16(thisData, 4);                                             // using median - 25% improvements over using mean to determine this value
    detector_read2_averaged += detector_read2 / _averages;
    //    Serial_Printf("read2: %f\n",detector_read2);
  }
  else if (detector_read1or2or3 == 3) {
    detector_read3 = median16(thisData, 4);                                             // using median - 25% improvements over using mean to determine this value
    detector_read3_averaged += detector_read3 / _averages;
    //    Serial_Printf("read3: %f\n",detector_read3);
  }
}

float get_contactless_temp (int _averages) {
  contactless_temp = (MLX90615_Read(0) + MLX90615_Read(0) + MLX90615_Read(0)) / 3;
  contactless_temp_averaged += contactless_temp / _averages;

  return contactless_temp;
}

// read accelerometer

void get_compass_and_angle (int notRaw, int _averages) {

  int magX, magY, magZ, accX, accY, accZ;
  MAG3110_read(&magX, &magY, &magZ);
  MMA8653FC_read(&accX, &accY, &accZ);

  float mag_coords[3] = {(float) magX, (float) magY, (float) magZ};
  applyMagCal(mag_coords);

  int acc_coords[3] = {accX, accY, accZ};
  applyAccCal(acc_coords);

  roll = getRoll(acc_coords[1], acc_coords[2]);
  pitch = getPitch(acc_coords[0], acc_coords[1], acc_coords[2], roll);
  compass = getCompass(mag_coords[0], mag_coords[1], mag_coords[2], pitch, roll);

  Tilt deviceTilt = calculateTilt(roll, pitch, compass);

  rad_to_deg(&roll, &pitch, &compass);

  angle = deviceTilt.angle;
  //Serial_Printf("%f \n", angle);
  angle_direction = deviceTilt.angle_direction;

  if (notRaw == 0) {                                              // save the raw values average
    x_tilt_averaged += (float)x_tilt / _averages;
    y_tilt_averaged += (float)y_tilt / _averages;
    z_tilt_averaged += (float)z_tilt / _averages;
    x_compass_raw_averaged += (float)x_compass_raw / _averages;
    y_compass_raw_averaged += (float)y_compass_raw / _averages;
    z_compass_raw_averaged += (float)z_compass_raw / _averages;
  }
  if (notRaw == 1) {                                              // save the calibrated values and average
    roll_averaged += roll / _averages;
    pitch_averaged += pitch / _averages;
    compass_averaged += compass / _averages;
    angle_averaged += angle / _averages;
  }
  // add better routine here to produce clearer tilt values
}

// read the hall sensor to measure thickness of a leaf

float get_thickness (int notRaw, int _averages) {
  int sum = 0;
  for (int i = 0; i < 1000; ++i) {
    sum += analogRead(HALL_OUT);
  }
  thickness_raw = (sum / 1000);
  thickness = (eeprom->thickness_a * thickness_raw * thickness_raw + eeprom->thickness_b * thickness_raw + eeprom->thickness_c) / 1000; // calibration information is saved in uM, so divide by 1000 to convert back to mm.

  if (notRaw == 0) {                                              // save the raw values average
    thickness_raw_averaged += (float)thickness_raw / _averages;
    return thickness_raw;
  }
  else if (notRaw == 1) {                                              // save the calibrated values and average
    thickness_averaged += (float)thickness / _averages;
    return thickness;
  }
  else {
    return 0;
  }
}

float get_analog_read (int pin, int _averages) {
  int sum = 0;
  for (int i = 0; i < 1000; ++i) {
    sum += analogRead(pin);
  }
  analog_read = (sum / 1000);
  analog_read_averaged += (float) analog_read / _averages;
  return analog_read;
}



float get_co2(int _averages) {
  co2 = requestCo2(2000);
  //  co2 = getCo2(response);
  delay(100);
  co2_averaged += (float) co2 / _averages;
#ifdef DEBUGSIMPLE
  Serial_Print("\"co2_content\":");
  Serial_Print(co2_value);
  Serial_Print(",");
#endif
  return co2;
}

float get_digital_read (int pin, int _averages) {
  digital_read = digitalRead(pin);
  digital_read_averaged += (float) digital_read / _averages;
  return analog_read;
}

float get_adc_read (int adc_channel, int _averages) {       // adc channels 1 - 4 available through USB 3.0 port.  Actual channels are 4 - 7, so take user value and add 3
  adc_read = AD7689_read(adc_channel + 3);
  adc_read_averaged += (float) adc_read / _averages;
  return adc_read;
}
float get_adc_read2 (int adc_channel, int _averages) {       // adc channels 1 - 4 available through USB 3.0 port.  Actual channels are 4 - 7, so take user value and add 3
  adc_read2 = AD7689_read(adc_channel + 3);
  adc_read2_averaged += (float) adc_read2 / _averages;
  return adc_read2;
}
float get_adc_read3 (int adc_channel, int _averages) {       // adc channels 1 - 4 available through USB 3.0 port.  Actual channels are 4 - 7, so take user value and add 3
  adc_read3 = AD7689_read(adc_channel + 3);
  adc_read3_averaged += (float) adc_read3 / _averages;
  return adc_read3;
}

// check for commands to read various envirmental sensors
// also output the value on the final call

// TODO ok instead of just adding up the values in the array, we probably need to make another nest in the nested array, otherwise we don't know where to start the next environmentals... so right

static void environmentals(JsonArray environmental, const int _averages, const int count, int oneOrArray)
{

  for (int i = 0; i < environmental.getLength(); i++) {                                       // call environmental measurements after the spectroscopic measurement

    String thisSensor = environmental.getArray(i).getString(0);
    //    Serial_Printf("thisSensor: %s", thisSensor);

    if (thisSensor == "temperature_humidity_pressure") {                   // measure light intensity with par calibration applied
      get_temperature_humidity_pressure(_averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"temperature\":%f,\"humidity\":%f,\"pressure\":%f,", temperature_averaged, humidity_averaged, pressure_averaged);
      }
    }
    else if (thisSensor == "temperature_humidity_pressure2") {                   // measure light intensity with par calibration applied
      get_temperature_humidity_pressure2(_averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"temperature2\":%f,\"humidity2\":%f,\"pressure2\":%f,", temperature2_averaged, humidity2_averaged, pressure2_averaged);
      }
    }

    else if (thisSensor == "light_intensity") {                   // measure light intensity with par calibration applied
      get_light_intensity(_averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"light_intensity\":%.2f,\"r\":%.2f,\"g\":%.2f,\"b\":%.2f,\"light_intensity_raw\":%.2f,", light_intensity_averaged, r_averaged, g_averaged, b_averaged, light_intensity_raw_averaged);
      }
    }

    else if (thisSensor == "contactless_temp") {                 // measure contactless temperature
      get_contactless_temp(_averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"contactless_temp\":%.2f,", contactless_temp_averaged);
      }
    }

    else if (thisSensor == "thickness") {                        // measure thickness via hall sensor, with calibration applied
      get_thickness(1, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"thickness\":%.2f,", thickness_averaged);
      }
    }

    else if (thisSensor == "thickness_raw") {                    // measure thickness via hall sensor, raw con
      get_thickness(0, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"thickness_raw\":%.2f,", thickness_raw_averaged);
      }
    }

    else if (thisSensor == "compass_and_angle") {                             // measure tilt in -180 - 180 degrees
      get_compass_and_angle(1, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"compass_direction\":%s,\"compass\":\"%.2f\",\"angle\":%.2f,\"angle_direction\":%s,\"pitch\":%.2f,\"roll\":%.2f,", getDirection(compass_segment(compass_averaged)), compass_averaged, angle_averaged, angle_direction.c_str(), pitch_averaged, roll_averaged);
      }
    }

    else if (thisSensor == "compass_and_angle_raw") {                         // measure tilt from -1000 - 1000
      get_compass_and_angle(0, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"x_tilt\":%.2f,\"y_tilt\":%.2f,\"z_tilt\":%.2f,\"x_compass_raw\":%.2f,\"y_compass_raw\":%.2f,\"z_compass_raw\":%.2f,", x_tilt_averaged, y_tilt_averaged, z_tilt_averaged, x_compass_raw_averaged, y_compass_raw_averaged, z_compass_raw_averaged);
      }
    }

    else if (thisSensor == "detector_read1") {
      int this_light = environmental.getArray(i).getLong(1);
      int this_intensity = environmental.getArray(i).getLong(2);
      int this_detector = environmental.getArray(i).getLong(3);
      int this_pulsesize = environmental.getArray(i).getLong(4);
      get_detector_value (_averages, this_light, this_intensity, this_detector, this_pulsesize, 1);     // save as "detector_read1" from get_detector_value function
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"detector_read1\":%f,", detector_read1_averaged);
      }
    }

    else if (thisSensor == "detector_read2") {
      int this_light = environmental.getArray(i).getLong(1);
      int this_intensity = environmental.getArray(i).getLong(2);
      int this_detector = environmental.getArray(i).getLong(3);
      int this_pulsesize = environmental.getArray(i).getLong(4);
      get_detector_value (_averages, this_light, this_intensity, this_detector, this_pulsesize, 2);     // save as "detector_read2" from get_detector_value function
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"detector_read2\":%f,", detector_read2_averaged);
      }
    }

    else if (thisSensor == "detector_read3") {
      int this_light = environmental.getArray(i).getLong(1);
      int this_intensity = environmental.getArray(i).getLong(2);
      int this_detector = environmental.getArray(i).getLong(3);
      int this_pulsesize = environmental.getArray(i).getLong(4);
      get_detector_value (_averages, this_light, this_intensity, this_detector, this_pulsesize, 3);     // save as "detector_read3" from get_detector_value function
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"detector_read3\":%f,", detector_read3_averaged);
      }
    }

    else if (thisSensor == "analog_read") {                      // perform analog reads
      int pin = environmental.getArray(i).getLong(1);
      get_analog_read(pin, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"analog_read\":%f,", analog_read_averaged);
      }
    }

    else if (thisSensor == "digital_read") {                      // perform digital reads
      int pin = environmental.getArray(i).getLong(1);
      get_digital_read(pin, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"digital_read\":%f,", digital_read_averaged);
      }
    }

    else if (thisSensor == "co2") {                            // perform digital reads
      get_co2(_averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"co2\":%f,", co2_averaged);
      }
    }

    else if (thisSensor == "adc_read") {                      // perform digital reads
      int adc_channel = environmental.getArray(i).getLong(1);
      get_adc_read(adc_channel, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"adc_read\":%f,", adc_read_averaged);
      }
    }

    else if (thisSensor == "adc_read2") {                      // perform digital reads
      int adc_channel = environmental.getArray(i).getLong(1);
      get_adc_read2(adc_channel, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"adc_read2\":%f,", adc_read2_averaged);
      }
    }

    else if (thisSensor == "adc_read3") {                      // perform digital reads
      int adc_channel = environmental.getArray(i).getLong(1);
      get_adc_read3(adc_channel, _averages);
      if (count == _averages - 1 && oneOrArray == 0) {
        Serial_Printf("\"adc_read3\":%f,", adc_read3_averaged);
      }
    }

    else if (thisSensor == "digital_write") {                      // perform digital write
      int pin = environmental.getArray(i).getLong(1);
      int setting = environmental.getArray(i).getLong(2);
      Serial_Flush_Input();
      pinMode(pin, OUTPUT);
      //      delayMicroseconds(300);
      digitalWriteFast(pin, setting);
    }

    else if (thisSensor == "analog_write") {                      // perform analog write with length of time to apply the pwm
      int pin = environmental.getArray(i).getLong(1);
      int setting = environmental.getArray(i).getLong(2);
      int freq = environmental.getArray(i).getLong(3);
      int wait = environmental.getArray(i).getLong(4);

      // TODO sanity checks

#ifdef DEBUGSIMPLE
      Serial_Print_Line(pin);
      Serial_Print_Line(pin);
      Serial_Print_Line(wait);
      Serial_Print_Line(setting);
      Serial_Print_Line(freq);
#endif

      pinMode(pin, OUTPUT);
      analogWriteFrequency(pin, freq);                                                           // set analog frequency
      analogWrite(pin, setting);
      delay(wait);
      analogWrite(pin, 0);
      //reset_freq();                                                                              // reset analog frequencies
    } // if
  } // for
}  //environmentals()

static void print_all () {
  // print every value saved in eeprom in valid json structure (even the undefined values which are still 0)
  Serial_Printf("light_intensity:%g\n", light_intensity);
}

#if 0
// ??  why
void reset_freq() {
  analogWriteFrequency(5, 187500);                                               // reset timer 0
  analogWriteFrequency(3, 187500);                                               // reset timer 1
  analogWriteFrequency(25, 488.28);                                              // reset timer 2
  /*
    Teensy 3.0              Ideal Freq:
    16      0 - 65535       732 Hz          366 Hz
    15      0 - 32767       1464 Hz         732 Hz
    14      0 - 16383       2929 Hz         1464 Hz
    13      0 - 8191        5859 Hz         2929 Hz
    12      0 - 4095        11718 Hz        5859 Hz
    11      0 - 2047        23437 Hz        11718 Hz
    10      0 - 1023        46875 Hz        23437 Hz
    9       0 - 511         93750 Hz        46875 Hz
    8       0 - 255         187500 Hz       93750 Hz
    7       0 - 127         375000 Hz       187500 Hz
    6       0 - 63          750000 Hz       375000 Hz
    5       0 - 31          1500000 Hz      750000 Hz
    4       0 - 15          3000000 Hz      1500000 Hz
    3       0 - 7           6000000 Hz      3000000 Hz
    2       0 - 3           12000000 Hz     6000000 Hz

  */
}
#endif

void print_calibrations() {
  unsigned i;

  Serial_Printf("{\n\"device_id\":\"d4:f5:%x:%x:%x:%x\",\n",
                (unsigned)eeprom->device_id >> 24,
                ((unsigned)eeprom->device_id & 0xff0000) >> 16,
                ((unsigned)eeprom->device_id & 0xff00) >> 8,
                (unsigned)eeprom->device_id & 0xff
               );
  Serial_Printf("\"mag_bias\": [\"%f\",\"%f\",\"%f\"],\n", eeprom->mag_bias[0], eeprom->mag_bias[1], eeprom->mag_bias[2]);
  Serial_Printf("\"mag_cal\": [[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"]],\n", eeprom->mag_cal[0][0], eeprom->mag_cal[0][1], eeprom->mag_cal[0][2], eeprom->mag_cal[1][0], eeprom->mag_cal[1][1], eeprom->mag_cal[1][2], eeprom->mag_cal[2][0], eeprom->mag_cal[2][1], eeprom->mag_cal[2][2]);
  Serial_Printf("\"accel_bias\": [\"%f\",\"%f\",\"%f\"],\n", eeprom->accel_bias[0], eeprom->accel_bias[1], eeprom->accel_bias[2]);
  Serial_Printf("\"accel_cal\": [[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"]],\n", eeprom->accel_cal[0][0], eeprom->accel_cal[0][1], eeprom->accel_cal[0][2], eeprom->accel_cal[1][0], eeprom->accel_cal[1][1], eeprom->accel_cal[1][2], eeprom->accel_cal[2][0], eeprom->accel_cal[2][1], eeprom->accel_cal[2][2]);
  Serial_Printf("\"light_slope_all\": \"%f\",\n", eeprom->light_slope_all);
  Serial_Printf("\"light_slope_r\": \"%f\",\n", eeprom->light_slope_r);
  Serial_Printf("\"light_slope_g\": \"%f\",\n", eeprom->light_slope_g);
  Serial_Printf("\"light_slope_b\": \"%f\",\n", eeprom->light_slope_b);
  Serial_Printf("\"light_yint\": \"%f\",\n", eeprom->light_yint);
  Serial_Printf("\"detector_offset_slope\": [\"%f\",\"%f\",\"%f\",\"%f\"],\n", eeprom->detector_offset_slope[0], eeprom->detector_offset_slope[1], eeprom->detector_offset_slope[2], eeprom->detector_offset_slope[3]);
  Serial_Printf("\"detector_offset_yint\": [\"%f\",\"%f\",\"%f\",\"%f\"],\n", eeprom->detector_offset_yint[0], eeprom->detector_offset_yint[1], eeprom->detector_offset_yint[2], eeprom->detector_offset_yint[3]);
  Serial_Printf("\"thickness_a\": \"%f\",\n", eeprom->thickness_a);
  Serial_Printf("\"thickness_b\": \"%f\",\n", eeprom->thickness_b);
  Serial_Printf("\"thickness_c\": \"%f\",\n", eeprom->thickness_c);
  Serial_Printf("\"thickness_min\": \"%f\",\n", eeprom->thickness_min);
  Serial_Printf("\"thickness_max\": \"%f\",\n", eeprom->thickness_max);

  Serial_Print("\"par_to_dac_slope1\": [");
  for (i = 0; i < arraysize(eeprom->par_to_dac_slope1) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->par_to_dac_slope1[i]);
  Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_slope1[i]);

  Serial_Print("\"par_to_dac_slope2\": [");
  for (i = 0; i < arraysize(eeprom->par_to_dac_slope2) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->par_to_dac_slope2[i]);
  Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_slope2[i]);

  Serial_Print("\"par_to_dac_slope3\": [");
  for (i = 0; i < arraysize(eeprom->par_to_dac_slope3) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->par_to_dac_slope3[i]);
  Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_slope3[i]);

  Serial_Print("\"par_to_dac_slope4\": [");
  for (i = 0; i < arraysize(eeprom->par_to_dac_slope4) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->par_to_dac_slope4[i]);
  Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_slope4[i]);

  Serial_Print("\"par_to_dac_yint\": [");
  for (i = 0; i < arraysize(eeprom->par_to_dac_yint) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->par_to_dac_yint[i]);
  Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_yint[i]);

  Serial_Print("\"ir_baseline_slope\": [");
  for (i = 0; i < arraysize(eeprom->ir_baseline_slope) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->ir_baseline_slope[i]);
  Serial_Printf("\"%f\"],\n", eeprom->ir_baseline_slope[i]);

  Serial_Print("\"ir_baseline_yint\": [");
  for (i = 0; i < arraysize(eeprom->ir_baseline_yint) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->ir_baseline_yint[i]);
  Serial_Printf("\"%f\"],\n", eeprom->ir_baseline_yint[i]);

  Serial_Print("\"colorcal_intensity1_slope\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity1_slope) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity1_slope[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity1_slope[i]);

  Serial_Print("\"colorcal_intensity1_yint\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity1_yint) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity1_yint[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity1_yint[i]);

  Serial_Print("\"colorcal_intensity2_slope\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity2_slope) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity2_slope[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity2_slope[i]);
  Serial_Print("\"colorcal_intensity2_yint\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity2_yint) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity2_yint[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity2_yint[i]);

  Serial_Print("\"colorcal_intensity3_slope\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity3_slope) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity3_slope[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity3_slope[i]);
  Serial_Print("\"colorcal_intensity3_yint\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_intensity3_yint) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_intensity3_yint[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity3_yint[i]);

  Serial_Print("\"colorcal_blank1\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_blank1) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_blank1[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank1[i]);
  Serial_Print("\"colorcal_blank2\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_blank2) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_blank2[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank2[i]);
  Serial_Print("\"colorcal_blank3\": [");
  for (i = 0; i < arraysize(eeprom->colorcal_blank3) - 1; i++)
    Serial_Printf("\"%f\",", eeprom->colorcal_blank3[i]);
  Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank3[i]);

  for (i = 0; i < NUM_USERDEFS - 1; i++)
    Serial_Printf("\"userdef%d\": \"%f\",\n", i, eeprom->userdef[i]);
  Serial_Printf("\"userdef%d\": \"%f\"\n}", i, eeprom->userdef[i]);

  Serial_Print_CRC();
}

theReadings getReadings (const char* _thisSensor) {                       // get the actual sensor readings associated with each environmental call (so compass and angle when you call compass_and_angle, etc.)
  theReadings _theReadings;
  if (!strcmp(_thisSensor, "temperature_humidity_pressure")) {
    _theReadings.reading1 = "temperature";
    _theReadings.reading2 = "humidity";
    _theReadings.reading3 = "pressure";
    _theReadings.numberReadings = 3;
  }
  else if (!strcmp(_thisSensor, "analog_read")) {
    _theReadings.reading1 = "analog_read";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "adc_read")) {
    _theReadings.reading1 = "adc_read";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "adc_read2")) {
    _theReadings.reading1 = "adc_read2";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "adc_read3")) {
    _theReadings.reading1 = "adc_read3";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "co2")) {
    _theReadings.reading1 = "co2";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "temperature_humidity_pressure2")) {
    _theReadings.reading1 = "temperature2";
    _theReadings.reading2 = "humidity2";
    _theReadings.reading3 = "pressure2";
    _theReadings.numberReadings = 3;
  }
  else if (!strcmp(_thisSensor, "contactless_temp")) {
    _theReadings.reading1 = "contactless_temp";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "light_intensity")) {
    _theReadings.reading1 = "light_intensity";
    _theReadings.reading2 = "light_intensity_raw";
    _theReadings.reading3 = "r";
    _theReadings.reading4 = "g";
    _theReadings.reading5 = "b";
    _theReadings.numberReadings = 5;
  }
  else if (!strcmp(_thisSensor, "thickness")) {
    _theReadings.reading1 = "thickness";
    _theReadings.reading2 = "thickness_raw";
    _theReadings.numberReadings = 2;
  }
  else if (!strcmp(_thisSensor, "compass_and_angle")) {
    _theReadings.reading1 = "compass";
    _theReadings.reading2 = "angle";
    _theReadings.numberReadings = 2;
  }
  else if (!strcmp(_thisSensor, "digital_read")) {
    _theReadings.reading1 = "digital_read";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "detector_read1")) {
    _theReadings.reading1 = "detector_read1";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "detector_read2")) {
    _theReadings.reading1 = "detector_read2";
    _theReadings.numberReadings = 1;
  }
  else if (!strcmp(_thisSensor, "detector_read3")) {
    _theReadings.reading1 = "detector_read3";
    _theReadings.numberReadings = 1;
  }
  else {                                                                          // output invalid entry if the entered sensor call is not on the list
    _theReadings.reading1 = "invalid_entry";
    _theReadings.numberReadings = 1;
  }
  //  Serial_Printf("%s, %s, %s, %s, %s, %d\n", _theReadings.reading1, _theReadings.reading2, _theReadings.reading3, _theReadings.reading4, _theReadings.reading5, _theReadings.numberReadings);
  return _theReadings;
}

//======================================

void get_set_device_info(const int _set) {

  if (_set == 1) {
    long val;

    // please enter new device ID (lower 4 bytes of BLE MAC address as a long int) followed by '+'
    //    Serial_Print_Line("{\"message\": \"Please enter device mac address (long int) followed by +: \"}\n");
    val =  Serial_Input_Long("+", 0);              // save to eeprom
    store(device_id, val);              // save to eeprom

  } // if

  // print

  int v = battery_percent(0);   // measured without load

  /*
    turn_off_5V();
    Serial_Printf("no 5 v");
    Serial_Printf("\n battery percent: %d; battery level: %d \n",battery_percent(0), battery_level(0));
    turn_on_5V();
    Serial_Printf("5v on");
    Serial_Printf("\n battery percent: %d; battery level: %d \n",battery_percent(0), battery_level(0));
  */

  //  Serial_Printf("{\"device_name\":\"%s\",\"device_version\":\"%s\",\"device_id\":\"d4:f5:%2.2x:%2.2x:%2.2x:%2.2x\",\"device_firmware\":\"%s\",\"device_manufacture\":%6.6d}", DEVICE_NAME, DEVICE_VERSION,
  Serial_Printf("{\"device_name\":\"%s\",\"device_version\":\"%s\",\"device_id\":\"d4:f5:%2.2x:%2.2x:%2.2x:%2.2x\",\"device_battery\":%d,\"device_firmware\":\"%s\"}", DEVICE_NAME, DEVICE_VERSION,    // I did this so it would work with chrome app
                (unsigned)eeprom->device_id >> 24,
                ((unsigned)eeprom->device_id & 0xff0000) >> 16,
                ((unsigned)eeprom->device_id & 0xff00) >> 8,
                (unsigned)eeprom->device_id & 0xff, v,
                DEVICE_FIRMWARE);
  Serial_Print_CRC();

  return;

} // get_set_device_info()

// ======================================

