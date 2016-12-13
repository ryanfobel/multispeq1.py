#ifndef ___RPC_BUFFER__H___
#define ___RPC_BUFFER__H___

#include <stdint.h>




#ifdef __AVR_ATmega328P__


/* ## uno settings ## */
#ifndef I2C_PACKET_SIZE
#define I2C_PACKET_SIZE   PACKET_SIZE
#endif  // #ifndef I2C_PACKET_SIZE

#ifndef PACKET_SIZE
#define PACKET_SIZE   80
#endif  // #ifndef PACKET_SIZE

#ifndef COMMAND_ARRAY_BUFFER_SIZE
#define COMMAND_ARRAY_BUFFER_SIZE   
#endif  // #ifndef COMMAND_ARRAY_BUFFER_SIZE


#elif __AVR_ATmega2560__

/* ## mega2560 settings ## */
#ifndef I2C_PACKET_SIZE
#define I2C_PACKET_SIZE   PACKET_SIZE
#endif  // #ifndef I2C_PACKET_SIZE

#ifndef PACKET_SIZE
#define PACKET_SIZE   256
#endif  // #ifndef PACKET_SIZE

#ifndef COMMAND_ARRAY_BUFFER_SIZE
#define COMMAND_ARRAY_BUFFER_SIZE   
#endif  // #ifndef COMMAND_ARRAY_BUFFER_SIZE


#else


/* ## default settings ## */
#ifndef I2C_PACKET_SIZE
#define I2C_PACKET_SIZE   PACKET_SIZE
#endif  // #ifndef I2C_PACKET_SIZE

#ifndef PACKET_SIZE
#define PACKET_SIZE   80
#endif  // #ifndef PACKET_SIZE

#ifndef COMMAND_ARRAY_BUFFER_SIZE
#define COMMAND_ARRAY_BUFFER_SIZE   
#endif  // #ifndef COMMAND_ARRAY_BUFFER_SIZE


#endif


/* To save RAM, the serial-port interface may be disabled by defining
 * `DISABLE_SERIAL`. */
#ifndef DISABLE_SERIAL
extern uint8_t packet_buffer[PACKET_SIZE];
#endif  // #ifndef DISABLE_SERIAL

/*  - Allocate buffer for command-processor to extract/write array data. */
extern uint8_t command_array_buffer[COMMAND_ARRAY_BUFFER_SIZE];

extern uint8_t i2c_packet_buffer[I2C_PACKET_SIZE];

#endif  // #ifndef ___RPC_BUFFER__H___