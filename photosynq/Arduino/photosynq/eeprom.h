
// Many variables used have to be stored/retrieved from eeprom.  All such variables are defined in this structure.
// This structure is stored directly in eeprom memory
// Assume that there was a calibration process that initialized the eeprom.

// DANGER:  do not do too many writes to eeprom variables - cycles are limited
// WARNING: max size is 2K
// WARNING: Write only with the store() function.  If not, program may freeze.

// Jon Zeeff 2016

// this structure overlays eeprom ram

#include <stdint.h>

const int NUM_USERDEFS = 115;

class eeprom_class
{
  public:
    // each of these needs a comment
    volatile long device_id;          // lower 4 bytes of bluetooth mac address
    volatile long device_manufacture; // month and year, eg 12016

    volatile float mag_bias[3];       // magnetometer/compass calibration
    volatile float mag_cal[3][3];

    volatile float accel_bias[3];
    volatile float accel_cal[3][3];   // accelerometer calibration

    //   now we calculate par based on rgb components of the incoming light and a yintercept
    //     volatile float light_slope;
    volatile float light_yint;         // y intercept for par calibration from tcs
    volatile float light_slope_all;    // slope for the total intensity value for par calibration from tcs
    volatile float light_slope_r;      // slope for just the red portion of the part calibration from tcs
    volatile float light_slope_g;      // " " just green
    volatile float light_slope_b;      // " " just blue
    volatile float detector_offset_slope[4];      // slope and yint for detector offsets (detector 1 - 4 as array positions 1 - 4)
    volatile float detector_offset_yint[4];
    /*
       calibration for leaf thickness (3 parameters - a, b, and c, plus min and max values)
    */
    volatile float thickness_a;
    volatile float thickness_b;
    volatile float thickness_c;
    volatile float thickness_min;
    volatile float thickness_max;
    /*
       replace calibration_slope and all that just with par_to_dac (and create a simple function for conversion)...- if the relationships is more complicated than just mx+b then add more variables as needed
    */
    volatile float par_to_dac_slope1[NUM_LEDS + 1];
    volatile float par_to_dac_slope2[NUM_LEDS + 1];
    volatile float par_to_dac_slope3[NUM_LEDS + 1];
    volatile float par_to_dac_slope4[NUM_LEDS + 1];
    volatile float par_to_dac_yint[NUM_LEDS + 1];
    /*
       IR baseline estimation based on the reflectivity of the object.  This helps address any residual IR in the measuring light itself (used in chlorophyll fluorescence).
    */
    volatile float ir_baseline_slope[NUM_LEDS + 1];
    volatile float ir_baseline_yint[NUM_LEDS + 1];
    /*
       values for color calibrations (used for SPAD, anthocyanin, etc.) at 3 intensity levels (allows for simpler protocols as thickness doesn't need to be adjusted or on the fly).
    */
    volatile float colorcal_blank1[NUM_LEDS + 1];       // blank used to calculate absorbance for LEDs 1,2,3,4,6,8,9,10 .  Blank here is actual blank.
    volatile float colorcal_blank2[NUM_LEDS + 1];       // same as colorcal_blank1, but this blank is a single piece of white paper (for thicker samples)
    volatile float colorcal_blank3[NUM_LEDS + 1];       // same as colorcal_blank1, but this blank is 3 pieces of white paper stacked (for even thicker samples)

    volatile float colorcal_intensity1_slope[NUM_LEDS + 1]; // slope from calibration to Minolta SPAD at intensity 1 (use colorcal_blank1 as blank)
    volatile float colorcal_intensity1_yint[NUM_LEDS + 1]; // y intercept from calibration to Minolta SPAD at intensity 1 (use colorcal_blank1 as blank)
    volatile float colorcal_intensity2_slope[NUM_LEDS + 1]; // slope from calibration to Minolta SPAD at intensity 2 (use colorcal_blank1 as blank)
    volatile float colorcal_intensity2_yint[NUM_LEDS + 1]; // y intercept from calibration to Minolta SPAD at intensity 2 (use colorcal_blank1 as blank)
    volatile float colorcal_intensity3_slope[NUM_LEDS + 1]; // slope from calibration to Minolta SPAD at intensity 3 (use colorcal_blank1 as blank)
    volatile float colorcal_intensity3_yint[NUM_LEDS + 1]; // y intercept from calibration to Minolta SPAD at intensity 3 (use colorcal_blank1 as blank)

    // user set values
    volatile float userdef[NUM_USERDEFS + 1];

    volatile char sleep;     // TODO set to 1 to sleep on boot
};

// where to store permanent data (teensy 3 specific)
#define FlexRAM ((eeprom_class *)0x14000000)

#ifndef EXTERN
extern class eeprom_class * eeprom;
#else
class eeprom_class * eeprom = FlexRAM;
#endif

#undef EXTERN
