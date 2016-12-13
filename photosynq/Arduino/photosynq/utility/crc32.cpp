
// routines to calculate crc values on fixed or on-the-fly data

#include "crc32.h"

char *
int32_to_hex (uint32_t i)
{
  static const char nybble_chars[] = "0123456789ABCDEF";
  static char result[9];

  // process 8 nibbles

  result[0] = nybble_chars[(i >> 28) & 0xf];
  result[1] = nybble_chars[(i >> 24) & 0xf];
  result[2] = nybble_chars[(i >> 20) & 0xf];
  result[3] = nybble_chars[(i >> 16) & 0xf];
  result[4] = nybble_chars[(i >> 12) & 0xf];
  result[5] = nybble_chars[(i >> 8) & 0xf];
  result[6] = nybble_chars[(i >> 4) & 0xf];
  result[7] = nybble_chars[(i >> 0) & 0xf];
  return result;
}

// CRC32 support routines

static uint32_t crc = 0 ^ ~0U;

// must be called before using below routines
void
crc32_init ()
{
  crc = 0 ^ ~0U;		// aka 0xffffffff
}


uint32_t
crc32_value (void)
{
  return ~crc;
}

// single byte version (slow)
void 
crc32_byte (const uint32_t byte)
{
  int j;
  uint32_t mask;

  crc = crc ^ byte;
  for (j = 7; j >= 0; j--)
    {				// Do eight times.
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
}

// buffer version
void
crc32_buf (const char * message, int size)
{
  int i, j;
  unsigned int mask;

  i = 0;
  while (i < size)
    {
      unsigned byte = message[i];	// Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--)
	{			// Do eight times.
	  mask = -(crc & 1);
	  crc = (crc >> 1) ^ (0xEDB88320 & mask);
	}
      i = i + 1;
    }
}


// string version
void
crc32_string (const char *message)
{
  int i, j;
  unsigned int mask;

  i = 0;
  while (message[i] != 0)
    {
      unsigned int byte = message[i];	// Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--)
	{			// Do eight times.
	  mask = -(crc & 1);
	  crc = (crc >> 1) ^ (0xEDB88320 & mask);
	}
      i = i + 1;
    }
}

// test routine

#if 0
int
main ()
{
  crc32_init ();
  crc32_buf ("abcd", 4);
  crc32_byte ('e');
  crc32_buf ("fghij", 5);
  printf ("crc =%x\n", crc32_value ());

  crc32_init ();
  crc32_string ("abcdefghij");
  printf ("crc =%x\n", crc32_value ());
  Serial_Print_CRC ();
  Serial_Print ("abcdefghij");
  Serial_Print_CRC ();
}
#endif

