
#include <stdint.h>
#include <kinetis.h>
//#include "serial.h"

#if defined(__MK20DX128__)
#define FLASH_SIZE              0x20000
#define FLASH_SECTOR_SIZE       0x400
#define FLASH_ID                "fw_teensy3.0"
#define RESERVE_FLASH (2 * FLASH_SECTOR_SIZE)
#elif defined(__MK20DX256__)
#define FLASH_SIZE              0x40000
#define FLASH_SECTOR_SIZE       0x800
#define FLASH_ID                "fw_teensy3.1"
#define RESERVE_FLASH (1 * FLASH_SECTOR_SIZE)
#else
#error NOT SUPPORTED
#endif

// apparently better - thanks to Frank Boesing
#define RAMFUNC  __attribute__ ((section(".fastrun"), noinline, noclone, optimize("Os") ))

void upgrade_firmware(void);
void boot_check(void);
uint32_t  read_once(unsigned char address);
void program_once(unsigned char address, uint32_t value);

