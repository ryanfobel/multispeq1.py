#ifndef ___PAR__H___
#define ___PAR__H___

namespace PAR {

int get_light_intensity(int _averages);
uint16_t par_to_dac (float _par, uint16_t _pin);
void PAR_init(); // initialize PAR and RGB sensor

}

#endif // #ifndef ___PAR__H___
