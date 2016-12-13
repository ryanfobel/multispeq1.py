
// Serial routines that can read/write to either or both Serial devices (Serial (USB) and Serial1 (BLE)) based on a setting
// It also maintains and prints a CRC value
// Can use a packet mode that resends

// Jon Zeeff 2016

// WARNING: Serial_Printf() is limited to 200 characters per call
// Zero is used as a string terminator - it can't be sent. Nor can ETX.

// Should change this to C++ and a class

#include <Arduino.h>
#include "serial.h"
#include "utility/crc32.h"
#include "defines.h"

static void Serial_Print_BLE(const char *str);
static void print_packet(const char *str);
static void flush_BLE();

static int Serial_Port = 3;   // which port to print to: 1 == Serial, 2 == Serial1, 3 = both  (ignored during automatic mode)
static int automatic = 0;     // automatic means that writes will only go to the serial port that last had a byte read (Serial_Port is ignored)
static int last_read = 0;     // where last incoming byte was from, 0 = Serial, 1 = Serial1
static int retry_counter  = 0;  // total number of retries
int packet_mode = 0;          // wait for ACK every n characters, resend if needed
int cut_through = 1;          // send bytes as soon as we receive them (vs when packet buffer is full)

// set baud rates

void Serial_Begin(int rate)
{
  //  assert(rate >= 9600 && rate <= 115200);

  Serial.begin(115200);   // USB serial port, baud rate is irrelevant
  Serial1.begin(rate);    // for BLE, 57600 is standard
}

// discard all available input
void Serial_Flush_Input(void)
{
  while (Serial_Available())
    Serial_Read();
}

// specify which ports to send output to  (1 = USB, 2 = BT, 3 = both, 4 = automatic)
void Serial_Set(int s)
{
  //  assert(s > 0 && s <= 4);

  if (s == 4) {                 // automatic means that writes will only go to the serial port that last had a byte available
    automatic = 1;
    Serial_Port = 3;
  } else {
    automatic = 0;
    Serial_Port = s;
  }
}

// Do a printf to either port
// CAUTION: only 200 bytes

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

int Serial_Read()
{
  for (;;) {
    if (Serial1.available()) {
      last_read = 1;
      return Serial1.read();
    }
    if (Serial && Serial.available())  {
      last_read = 0;
      return Serial.read();
    }
  } // for
}  // Serial_Read()


unsigned Serial_Available()
{
  unsigned count = 0;

  if (Serial)
    count += Serial.available();

  count += Serial1.available();

  return count;
}  // Serial_Available()


int Serial_Peek()
{
  if (Serial1.available())
    return Serial1.peek();

  if (Serial && Serial.available())
    return Serial.peek();

  return -1;
}  // Serial_Available()


// routines to print to two serial ports with a CRC at the end

void
Serial_Print (const char *str)    // other Serial_Print() routines call this one
{
  // output to specified ports
  // Caution:  may be buffered and require a Serial_Flush_Output() to see the entire string

  if (automatic) {             // automatic mode takes precedence over Serial_Port setting
    if (last_read == 0) {
      if (Serial)
        Serial.print(str);
    } else if (packet_mode)
      print_packet(str);
    else
      Serial_Print_BLE(str);
  } else {
    // non-automatic mode
    if (Serial && (Serial_Port & 1))
      Serial.print(str);

    if (Serial_Port & 2) {
      if (packet_mode)
        print_packet(str);
      else
        Serial_Print_BLE(str);
    }
  } // if

  // add to crc value
  crc32_string ((char *)str);

}  // Serial_Print(char *)


// CCIT CRC16

uint16_t crc16(const char* data_p, unsigned char length) {
  unsigned char x;
  uint16_t crc = 0xFFFF;

  while (length--) {
    x = crc >> 8 ^ *data_p++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
  }
  return crc;
} // crc16()

// output to the BLE serial port, but buffer it up into packets with a retry protocol
// TODO - make this double buffered to avoid delays

#define PACKET_SIZE 234          // Note: must end up with a packet size that is a multiple of 20 (not counting null)
#define ETX 04
//#define ETX 'X'
#define ACK 06
//#define ACK 'Z'

static char packet_buffer[PACKET_SIZE + 1 + 4 + 1 + 1];  // extra room for SEQ then CRC then ETX then null - (multiple of 20) + 1
static int packet_count = 0;                         // how many bytes currently in the above buffer
static int seq = 0;                                  // goes A-Z and then wraps back to A - sent with each packet
const int RETRIES = 5;
const int RETRY_DELAY = 250;     // ms

// push a full or partial packet out
// retry until ACK, give up eventually

static void flush_packet()
{
  if (packet_count == 0)     // we have nothing buffered, don't send an empty packet
    return;

  int payload_count = packet_count;

  packet_buffer[packet_count++] = seq + 'A';   // ascii A-Z for sequence number

  // calc and add ascii CRC (exactly 4 chars)
  uint16_t crc = crc16(packet_buffer, packet_count);
  const char nybble_chars[] = "0123456789ABCDEF";
  packet_buffer[packet_count++] = nybble_chars[(crc >> 12) & 0xf];
  packet_buffer[packet_count++] = nybble_chars[(crc >> 8) & 0xf];;
  packet_buffer[packet_count++] = nybble_chars[(crc >> 4) & 0xf];;
  packet_buffer[packet_count++] = nybble_chars[(crc >> 0) & 0xf];;

  // add end of packet marker
  packet_buffer[packet_count++] = ETX;

  // add null
  packet_buffer[packet_count] = '\0';

  // keep sending it until we get an ACK or we give up
  for (int i = 0; i < RETRIES; ++i) {

    while (Serial1.available())          // flush input
      Serial1.read();

    if (cut_through && i == 0)          // first try, only send the SEQ/CRC/ETX - payload already went out
      Serial_Print_BLE(packet_buffer + payload_count);
    else
      Serial_Print_BLE(packet_buffer);  // send entire packet

    flush_BLE();                         // force it out

    // look for ACK, NAK or timeout
    unsigned char c = 0;
    unsigned t = millis();

    while (millis() - t < RETRY_DELAY) {
      if (Serial1.peek() != -1) {
        do {                                // got some character (probably ACK or NAK)
          c = Serial1.read();
        } while (Serial1.peek() != -1);     // remove any excess characters
        break;
      }
    } // while

    if (c == ACK)
      break;                                // note: any other character will cause a retry

    ++retry_counter;                        // for diagnostics

  } // for

  packet_count = 0;          // start new packet
  seq = (seq + 1) % 26;      // move to next SEQ letter A-Z

}  // flush_packet()


// add to output packet, send as needed

static void print_packet(const char *str)
{
  // copy to buffer, sending whenever it reaches n bytes
  while (*str != 0) {                  // until end of string

    if (*str == ETX) {                   // we cannot send this character, skip it
       ++str;
       continue;
    }

    packet_buffer[packet_count] = *str;
    ++packet_count;

    if (cut_through) {
      char s[2] = {*str, 0};
      Serial_Print_BLE(s);  // send to BLE buffer now vs waiting for a full data packet
    }

    ++str;

    if (packet_count == PACKET_SIZE) {  // full buffer - send it
      flush_packet();
    }

  } // while
}  // print_packet()

#define BLE_DELAY 0                    // milli seconds between packets, probably 20 or 10
#define BLE_PACKET_SIZE 20

static char buffer[BLE_PACKET_SIZE + 1];
static int count = 0;                   // how many bytes in the above buffer

// output to the BLE serial port, but buffer into 20 byte packets and put delays between packets to limit to 1Kbytes/sec
// does not add to CRC

static void Serial_Print_BLE(const char *str)
{
  // copy to buffer, sending whenever it reaches n bytes
  while (*str != 0) {                  // until end of string

    buffer[count] = *str;
    ++count;
    ++str;

    if (count == BLE_PACKET_SIZE) {   // full buffer - send it
      flush_BLE();
    }
  } // while
}

// push a full or partial BLE packet out

static void flush_BLE()
{
  if (count == 0)
    return;

  buffer[count] = 0;            // null terminate the string
  Serial1.print(buffer);        // send it  FIXME
  Serial1.flush();              // make sure it goes out
  delay(BLE_DELAY);             // wait for transmission to avoid overrunning buffers
  count = 0;
}

// send any buffered characters
void Serial_Flush_Output()
{
  // for packet buffer
  if (packet_mode)
    flush_packet();

  // for BLE, where we have another buffer
  flush_BLE();
}


void
Serial_Print(const float x, int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, x);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const double xx, int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, xx);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const int i)
{
  char str[50 + 1];
  snprintf(str, 50, "%d", i);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const unsigned u)
{
  char str[50 + 1];
  snprintf(str, 50, "%u", u);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const String string)
{
  // output to both ports
  Serial_Print (string.c_str());
}


void
Serial_Print_Line (const char *str)
{
  // output to both ports
  Serial_Print(str);
  Serial_Print("\n");
}

void
Serial_Print_Line(const String string)
{
  // output to both ports
  Serial_Print (string.c_str());
  Serial_Print("\r\n");
}


void
Serial_Print_Line (const int i)
{
  char str[50 + 1];
  snprintf(str, 50, "%d", i);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const long i)
{
  char str[50 + 1];
  snprintf(str, 50, "%ld", i);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const float x, const int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, x);
  //  assert(strlen(str) < 20);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const double xx, const int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, xx);
  //  assert(strlen(str) < 20);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

// print CRC (8 capital hex digits) and a newline and clear the crc value for the next time
void
Serial_Print_CRC (void)
{
  char *p = int32_to_hex (crc32_value ());

  Serial_Print_Line (p);
  Serial_Flush_Output();          // force it to go out

  crc32_init();
}

#include <string.h>

// read a string, terminating on any of:
//   1) time (in ms) runs out (since last char received)
//   2) a terminating character is received
//   3) input length = max_length (string should be one larger to hold the terminating null)

char *Serial_Input_Chars(char *string, const char *terminators, long unsigned int timeout, unsigned int max_length)  {       // terminating characters must be single characters or a list of single characters (so "+" will terminal on +, while "+-" will terminal on either + or -).
  unsigned long start = millis();
  unsigned count = 0;

  // loop forever or until a condition is met
  for (;;) {
    unsigned now = millis();

    if (timeout > 0 && (now - start) > timeout)       // timeout
      break;                                          // done

    int c = Serial_Peek();
    if (c == -1) {
      sleep_cpu();                         // save power
      continue;                            // nothing available
    }

    char b = Serial_Read();
    if (strchr(terminators, b))                       // terminator char seen - throw it away
      break;

    start = now;                                      // restart interval

    string[count++] = b;                              // add to string

    if (max_length > 0 && count >= max_length)        // too long
      break;

  } // for ever

  string[count] = 0;   // always null terminate the string

  return string;

}  // Serial_Input_Chars()


// read a double value (see above)
// empty string returns NAN

double Serial_Input_Double(const char *terminators, long unsigned int timeout) {
  char S[25];
  Serial_Input_Chars(S, terminators, timeout, sizeof(S) - 1);
  if (strlen(S) == 0)
    return NAN;
  return strtod(S, 0);
}  // Serial_Input_Double()


// read a long value (see above)
// invalid strings return 0

long Serial_Input_Long(const char *terminators, long unsigned int timeout) {
  char S[25];
  Serial_Input_Chars(S, terminators, timeout, sizeof(S) - 1);
  return atol(S);
}  // Serial_Input_Long()


// Caution: only good for short strings

String Serial_Input_String(const char *terminators, long unsigned int timeout)
{
  static String serial_string;
  char str[100 + 1]; // caution - fixed size buffer

  Serial_Input_Chars(str, terminators, timeout, sizeof(str) - 1);

  serial_string = str;

  //  assert(strlen(str) < 100);

  return serial_string;

}  // user_enter_str()

