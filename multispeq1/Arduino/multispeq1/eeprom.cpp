#include "eeprom.h"

// where to store permanent data (teensy 3 specific)
#define FlexRAM ((eeprom_class *)0x14000000)

class eeprom_class * eeprom = FlexRAM;
