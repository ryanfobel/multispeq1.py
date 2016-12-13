
//  Jon Zeeff, March, 2016

#define NUM_DACS 3

// which MCU pin is attached to each dac
#define LDAC1    15
#define LDAC2    16
#define LDAC3    17

// DAC functions
int DAC_init(void);
void DAC_shutdown(void);
void DAC_change(void);
void DAC_set(unsigned int LED, unsigned int value);
int DAC_set_address(int ldac_pin, unsigned oldAddress, unsigned newAddress); 

