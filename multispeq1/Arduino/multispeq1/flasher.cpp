
// Simple test program for OTA firmware updates on teensy 3.x using the USB virtual serial link.

// hint:  on Linux, exit the serial console and do "dd if=blink.hex of=/dev/ttyACM0", then restart the Serial console and
// enter the ":flash xxx" command.

#include <Arduino.h>
#include "flasher.h"
#include "serial.h"          // you probably don't want this, comment it out

void flash_erase_upper();
RAMFUNC static int flash_word (uint32_t address, uint32_t word_value);
RAMFUNC static int flash_erase_sector (uint32_t address, int unsafe);
RAMFUNC static void flash_move (uint32_t min_address, uint32_t max_address);
static int flash_hex_line(const char *line);
int parse_hex_line (const char *theline, char *bytes, unsigned int *addr, unsigned int *num, unsigned int *code);
static int flash_block(uint32_t address, uint32_t *bytes, int count);

#if 0

// You probably need these, customized for your serial port (ie, Serial, Serial1, etc)
// For use with other communication links (eg, packet radio, CAN, etc), you need to supply your own versions.
#define Serial_Available()  Serial.available()
#define Serial_Read()       Serial.read()
#define Serial_Printf       Serial.printf

const int ledPin = 13;

void setup ()
{
  // put your setup code here, to run once:
  boot_check();             // check if we need to upgrade firmware before running loop()

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);    // set the LED on
  delay(1000);
  digitalWrite(ledPin, LOW);     // set the LED off
}

void loop ()
{
#if 0
  //simple example of writing to flash
  int ret;
  uint32_t addr = FLASH_SIZE / 2;
  ret = flash_word (addr, 0x123);
  Serial.printf ("%x:%x  %d\n", addr, *(volatile unsigned int *) addr, ret);
  ret = flash_word (addr, 0x456);  // this will fail
  Serial.printf ("%x:%x  %d\n", addr, *(volatile unsigned int *) addr, ret);
  flash_erase_sector (addr, 0);
  ret = flash_word (addr, 0x456);
  Serial.printf ("%x:%x  %d\n", addr, *(volatile unsigned int *) addr, ret);
#endif

  upgrade_firmware();

  for (;;) {}

}  // loop()


#if 0
// Example Serial_Printf() if needed as a reference.  With this, you need to write Serial_Print() (probably easier).
#define SIZE 200

void Serial_Printf(const char * format, ... )
{
  char string[SIZE + 1];        // Warning: fixed buffer size
  va_list v_List;
  va_start( v_List, format );
  vsnprintf( string, SIZE, format, v_List );

  //  assert(strlen(string) < SIZE);

  string[SIZE] = 0;
  Serial_Print(string);
  va_end( v_List );
}
#endif

#endif

// ********************************************************************************************

// Version 1.3B

// code to allow firmware update over any byte stream (serial port, radio link, etc)
// for teensy 3.x.  It is intended that this code always be included in your application.

// Jon Zeeff 2016
// This code is in the public domain.  Please retain my name and
// in distributed copies, and let me know about any bugs

// I, Jon Zeeff, give no warranty, expressed or implied for
// this software and/or documentation provided, including, without
// limitation, warranty of merchantability and fitness for a
// particular purpose.


// load new firmware over some ascii byte stream and flash it
// format is an intel hex file followed by ":flash xx" where xx is the number of lines sent
// Max size is 1/2 of flash
// TODO  Modify to allow any combination of programs that fit in flash - ie, allow a 64K program to load a 188K program

// caution:  it is easy to send too fast.  Writing flash takes awhile - put delays between sending lines.

// porting: while the test routine uses a serial port, the only communication link this program needs is supplied by the user supplied routines:
// Serial_Read() and Serial_Printf() and optionally Serial_Available()
// See serial.cpp for an example of these routines used for a packetized, error corrected bluetooth link.


// TODO - instead of splitting flash in half, use whatever flash is remaining.  This would allow a 64K program to flash
// a 192K program.  This would allow the two step process of 192K, downgrade to 64K, upgrade to new 192K.

// TODO - only copy RAMFUNC functions to ram just before they are used

void upgrade_firmware(void)   // main entry point
{
  Serial_Printf("%s flash size = %dK in %dK sectors\n", FLASH_ID, FLASH_SIZE / 1024, FLASH_SECTOR_SIZE / 1024);

  flash_erase_upper ();   // erase upper half of flash

  if ((uint32_t)flash_word < FLASH_SIZE || (uint32_t)flash_erase_sector < FLASH_SIZE || (uint32_t)flash_move < FLASH_SIZE) {
    Serial_Printf("routines not in ram\n");
    return;
  }

  // what is currently used?
  int32_t addr = FLASH_SIZE / 2;
  while (addr > 0 && *((uint32_t *)addr) == 0xFFFFFFFF)
    addr -= 4;
  Serial_Printf("current firmware 0:%x\n", addr + 4);

  if (addr > FLASH_SIZE / 2 - RESERVE_FLASH) {
    Serial_Printf("firmware is too large\n");
    return;
  }

  Serial_Printf("WARNING: this can ruin your device\n");
  Serial_Printf("waiting for intel hex lines\n");

  char line[200];
  int count = 0;

  // read in hex lines

  for (;;)  {
    int c;

    while (!Serial.available()) {}
    c = Serial.read();

    if (c == '\n' || c == '\r') {
      line[count] = 0;          // terminate string
      flash_hex_line(line);
      count = 0;
    } else
      line[count++] = c;        // add to string

  } // for ever

} // upgrade_firmware()


// So what happens if you write bad software and it locks up?  How do you execute "upgrade_firmware()" to recover?
// 1) Always call this routine early in setup()
// 2) Tie the send and receive lines of the receiver together and it should recover.

// This routine is optional and can be removed.

void boot_check(void)
{
  delay(1000);
  Serial_Printf("@");
  delay(500);
  if (Serial_Available() && Serial_Read() == '@')
    upgrade_firmware();

  return;
}  // boot_check()


// check that the uploaded firmware contains a string that indicates that it will run on this MCU
static int check_compatible(uint32_t min, uint32_t max)
{
  uint32_t i;

  // look for FLASH_ID in the new firmware
  for (i = min; i < max - strlen(FLASH_ID); ++i) {
    if (strncmp((char *)i, FLASH_ID, strlen(FLASH_ID)) == 0)
      return 1;
  }
  return 0;
} // check_compatible()

// *****************************************************************************

// WARNING:  you can destroy your MCU with flash erase or write!
// This code may or may not protect you from that.

// Extensive modifications for OTA updates by Jon Zeeff
// Original by Niels A. Moseley, 2015.
// This code is released into the public domain.

// https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/


static int leave_interrupts_disabled = 0;


// *********************************
// actual flash operation occurs here - must run from ram
// flash a 4 byte word

RAMFUNC static int
flash_word (uint32_t address, uint32_t word_value)
{
  if (address >= FLASH_SIZE || (address & 0B11) != 0) // basic checks
    return 1;

  // correct value in FTFL_FSEC no matter what
  if (address == 0x40C) {
    word_value = 0xFFFFFFFE;
  }

  // check if already done - not an error
  if (*(volatile uint32_t *) address == word_value)
    return 0;

  // check if not erased
  if (*(volatile uint32_t *) address != 0xFFFFFFFF)  // TODO this fails
    return 4;

  __disable_irq ();

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  // clear error flags
  FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0;

  // program long word!
  FTFL_FCCOB0 = 0x06;   // PGM
  FTFL_FCCOB1 = address >> 16;
  FTFL_FCCOB2 = address >> 8;
  FTFL_FCCOB3 = address;
  FTFL_FCCOB4 = word_value >> 24;
  FTFL_FCCOB5 = word_value >> 16;
  FTFL_FCCOB6 = word_value >> 8;
  FTFL_FCCOB7 = word_value;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  FMC_PFB0CR |= 0xF << 20;  // flush cache

  if (!leave_interrupts_disabled)
    __enable_irq ();

  // check if done OK
  if (*(volatile uint32_t *) address != word_value)
    return 8;

  return FTFL_FSTAT & (FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0);
}

int erase_count = 0;

// *************************************

RAMFUNC static int
flash_erase_sector (uint32_t address, int unsafe)
{
  if (address > FLASH_SIZE || (address & (FLASH_SECTOR_SIZE - 1)) != 0) // basic checks
    return 1;

  if (address == (0x40C & ~(FLASH_SECTOR_SIZE - 1)) && unsafe != 54321)    // 0x40C is dangerous, don't erase it without override
    return 2;

  __disable_irq ();

  // wait for flash to be ready!
  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)
  {
  };

  // clear error flags
  FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0;

  // erase sector
  FTFL_FCCOB0 = 0x09;
  FTFL_FCCOB1 = address >> 16;
  FTFL_FCCOB2 = address >> 8;
  FTFL_FCCOB3 = address;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  FMC_PFB0CR = 0xF << 20;  // flush cache

  if (!leave_interrupts_disabled)
    __enable_irq ();

  return FTFL_FSTAT & (FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0);

}  // flash_erase_sector()

// ***********************************************
// move upper half down to lower half
// DANGER: if this is interrupted, the teensy could be permanently destroyed

RAMFUNC static void
flash_move (uint32_t min_address, uint32_t max_address)
{
  leave_interrupts_disabled = 1;

  min_address &= ~(FLASH_SECTOR_SIZE - 1);      // round down

  uint32_t address;
  int error = 0;

  // below here is critical

  // copy upper to lower, always erasing as we go up
  for (address = min_address; address <= max_address; address += 4) {

    if ((address & (FLASH_SECTOR_SIZE - 1)) == 0) {   // new sector?
      error |= flash_erase_sector(address, 54321);

      if (address == (0x40C & ~(FLASH_SECTOR_SIZE - 1)))  // critical sector
        error |= flash_word(0x40C, 0xFFFFFFFE);                  // fix it immediately
    }

    error |= flash_word(address, *(uint32_t *)(address + FLASH_SIZE / 2));

  } // for

  // hint - can use LED here for debugging.  Or disable erase and load the same program as is running.

  if (error) {
    //digitalWrite(ledPin, HIGH);    // set the LED on and halt
    for (;;) {}
  }

  // restart to run new program
#define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C)
#define CPU_RESTART_VAL 0x5FA0004
  *CPU_RESTART_ADDR = CPU_RESTART_VAL;

}  // flash_move()


// ***********************************

// Given an Intel hex format string, write it to upper flash memory (normal location + 128K)
// When finished, move it to lower flash
//
// Note:  hex records must be 32 bit word aligned!
// TODO: use a CRC value instead of line count

static int
flash_hex_line (const char *line)
{
  // hex records info
  unsigned int byte_count;
  static uint32_t address;
  unsigned int code;
  char data[128];   // assume no hex line will have more than this.  Alignment?

  static uint32_t base_address = 0;
  static int line_count = 0;
  static int error = 0;
  static int done = 0;
  static uint32_t max_address = 0;
  static uint32_t min_address = ~0;

  if (line[0] != ':')		// a non hex line is ignored
    return 0;

  if (error) {      // a single error and nothing more happens
    return -1;
  }

  // check for final flash execute command
  int lines;
  if (sscanf(line, ":flash %d", &lines) == 1) {
    if (lines == line_count && done) {
      Serial_Printf("flash %x:%x begins...\n", min_address, max_address);
      delay(100);
      flash_move (min_address, max_address);
      // should never get here
    } else {
      Serial_Printf ("bad line count %d vs %d or not done\n", lines, line_count);
      return -7;
    }
  } // if

  ++line_count;

  //int parse_hex_line(const char *theline, char bytes, unsigned int *addr, unsigned int *num, unsigned int *code);

  // must be a hex data line
  if (! parse_hex_line ((const char *)line, (char *)data, (unsigned int *) &address, (unsigned int*) &byte_count, (unsigned int*) &code))
  {
    Serial_Printf ("bad hex line %s\n", line);
    error = 1;
    return -1;
  }

  // address sanity check
  if (base_address + address + byte_count > FLASH_SIZE / 2 - RESERVE_FLASH) {
    Serial_Printf("address too large\n");
    error = 2;
    return -4;
  }

  // process line
  switch (code)
  {
    case 0:             // normal data
      break;
    case 1:             // EOF
      delay(1000);
      Serial_Printf ("done, %d hex lines, address range %x:%x, waiting for :flash %d\n", line_count, min_address, max_address, line_count);
      if (check_compatible(min_address + FLASH_SIZE / 2, max_address + FLASH_SIZE / 2))
        done = 1;
      else
        Serial_Printf ("new firmware not compatible\n");
      return 0;
    case 2:
      base_address = ((data[0] << 8) | data[1]) << 4;   // extended segment address
      return 0;
    case 4:
      base_address = ((data[0] << 8) | data[1]) << 16;  // extended linear address
      return 0;
    default:
      Serial_Printf ("err code = %d, line = %s\n", code, line);
      error = 3;
      return -3;
  }				// switch

  // write hex line to upper flash - note, cast assumes little endian and alignment
  if (flash_block (base_address + address + (FLASH_SIZE / 2), (uint32_t *)data, byte_count))  // offset to upper 128K
  {
    Serial_Printf ("can't flash %d bytes to %x\n", byte_count, address);
    error = 4;
    return -2;
  }

  // track size of modifications
  if (base_address + address + byte_count > max_address)
    max_address = base_address + address + byte_count;
  if (base_address + address < min_address)
    min_address = base_address + address;

  return 0;
}				// flash_hex_line()

// ****************************
// check if sector is all 0xFF
static int
flash_sector_erased(uint32_t address)
{
  uint32_t *ptr;

  for (ptr = (uint32_t *)address; ptr < (uint32_t *)(address + FLASH_SECTOR_SIZE); ++ptr) {
    if (*ptr != 0xFFFFFFFF)
      return 0;
  }
  return 1;
} // flash_sector_erased()

// ***************************
// erase the entire upper half
// Note: highest sectors of flash are used for other things - don't erase them
void flash_erase_upper()
{
  uint32_t address;
  int ret;

  // erase each block
  for (address = FLASH_SIZE / 2; address < (FLASH_SIZE - RESERVE_FLASH); address += FLASH_SECTOR_SIZE) {
    if (!flash_sector_erased(address)) {
      //Serial_Printf("erase sector %x\n", address);
      if ((ret = flash_erase_sector(address, 0)) != 0)
        Serial_Printf("flash erase error %d\n", ret);
    }
  } // for
} // flash_erase_upper()

// **************************
// take a word aligned array of words and write it to upper memory flash
static int
flash_block (uint32_t address, uint32_t * bytes, int count)
{
  int ret;

  if ((address % 4) != 0 || (count % 4 != 0))  // sanity checks
  {
    Serial_Printf ("flash_block align error\n");
    return -1;
  }

  while (count > 0)
  {
    if ((ret = flash_word(address, *bytes)) != 0)
    {
      Serial_Printf ("flash_block write error %d\n", ret);
      return -2;
    }
    address += 4;
    ++bytes;
    count -= 4;
  }				// while

  return 0;
}				// flash_block()


// these two are optional

#if 1

// read a WORD from the write once flash area
// address is 0 to 0xF

RAMFUNC uint32_t read_once(unsigned char address)
{
  __disable_irq ();
  
  // read long word
  FTFL_FCCOB0 = 0x41;   // RDONCE
  FTFL_FCCOB1 = address;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF) {} // wait for ready

  __enable_irq ();
  return (FTFL_FCCOB4 << 24) | (FTFL_FCCOB5 << 16) | (FTFL_FCCOB6 << 8) | FTFL_FCCOB7;
}

// write a 4 byte WORD to the write once flash area
// address is 0 to 0xF

RAMFUNC void program_once(unsigned char address, uint32_t word_value)
{
  __disable_irq ();
  
  // program long word
  FTFL_FCCOB0 = 0x43;   // PGMONCE
  FTFL_FCCOB1 = address;
  FTFL_FCCOB4 = word_value >> 24;
  FTFL_FCCOB5 = word_value >> 16;
  FTFL_FCCOB6 = word_value >> 8;
  FTFL_FCCOB7 = word_value;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF) {} // wait for ready

  FMC_PFB0CR |= 0xF << 20;  // flush cache
  __enable_irq ();
}

#endif

// **********************************************************

/* Intel Hex records:

  Start code, one character, an ASCII colon ':'.
  Byte count, two hex digits, indicating the number of bytes (hex digit pairs) in the data field.
  Address, four hex digits
  Record type (see record types below), two hex digits, 00 to 05, defining the meaning of the data field.
  Data, a sequence of n bytes of data, represented by 2n hex digits.
  Checksum, two hex digits, a computed value that can be used to verify the record has no errors.

  Example:
  :109D3000711F0000AD38000005390000F546000035
  :049D400001480000D6
  :00000001FF

*/


/* Intel HEX read/write functions, Paul Stoffregen, paul@ece.orst.edu */
/* This code is in the public domain.  Please retain my name and */
/* email address in distributed copies, and let me know about any bugs */

/* I, Paul Stoffregen, give no warranty, expressed or implied for */
/* this software and/or documentation provided, including, without */
/* limitation, warranty of merchantability and fitness for a */
/* particular purpose. */

// type modifications by Jon Zeeff


/* parses a line of intel hex code, stores the data in bytes[] */
/* and the beginning address in addr, and returns a 1 if the */
/* line was valid, or a 0 if an error occured.  The variable */
/* num gets the number of bytes that were stored into bytes[] */

int
parse_hex_line (const char *theline, char *bytes, unsigned int *addr, unsigned int *num, unsigned int *code)
{
  unsigned sum, len, cksum;
  const char *ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':')
    return 0;
  if (strlen (theline) < 11)
    return 0;
  ptr = theline + 1;
  if (!sscanf (ptr, "%02x", &len))
    return 0;
  ptr += 2;
  if (strlen (theline) < (11 + (len * 2)))
    return 0;
  if (!sscanf (ptr, "%04x", (unsigned int *)addr))
    return 0;
  ptr += 4;
  /* Serial_Printf("Line: length=%d Addr=%d\n", len, *addr); */
  if (!sscanf (ptr, "%02x", code))
    return 0;
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf (ptr, "%02x", &temp))
      return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256)
      return 0;
  }
  if (!sscanf (ptr, "%02x", &cksum))
    return 0;

  if (((sum & 255) + (cksum & 255)) & 255)
    return 0;			/* checksum error */
  return 1;
}

