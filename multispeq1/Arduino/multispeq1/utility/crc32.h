
// replace with FrankBoesing/FastCRC ?

#include <sys/param.h>
#include <stdint.h>

char * int32_to_hex (uint32_t i);
void crc32_init ();
uint32_t crc32_value (void);
void crc32_byte (const uint32_t byte);
void crc32_buf (const char * message, int size);
void crc32_string (const char *message);

