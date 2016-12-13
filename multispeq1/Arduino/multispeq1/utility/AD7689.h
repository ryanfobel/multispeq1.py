
// external ADC routines
uint16_t AD7689_read(int chan);
void AD7689_set (int chan, uint16_t config = 0);
void AD7689_shutdown(void);
void AD7689_sample(void);
uint16_t AD7689_read_sample(void);
uint16_t AD7689_read_temp(void);
void AD7689_read_array(uint16_t array[], int num_samples);
void AD7689_read_arrays(int chan1, uint16_t array1[], int chan2, uint16_t array2[], int num_samples);

