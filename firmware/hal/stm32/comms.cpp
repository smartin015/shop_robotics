#include "comms.h"
#include "hw.h"
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */
#include <stdio.h>      /* vsnprintf */

#define BUFLEN 128
uint8_t serbuf[BUFLEN];
int16_t idx = -1;
int16_t readlen = 0;

#define PACKET_START_BYTE 0x02

void comms::init() {}

uint8_t* comms::read(uint8_t* sz) {
  return hw::uart_recv(sz);
}
void comms::finishRead() {
  return hw::uart_flip_buffer();
} 

uint8_t* comms::preWrite(uint8_t sz) {
  if (hw::uart_busy(/*logging =*/ false)) {
    return 0;
  }
   uint8_t* ptr = hw::uart_get_write_buffer(/*logging =*/ false);
  (*ptr++) = PACKET_START_BYTE;
  (*ptr++) = sz;
  return ptr;
}

void comms::flush(uint8_t sz) {
  if (!hw::uart_send(sz, /*logging =*/ false)) {
    comms::printf("SNDFAIL (sz %d)", sz);
  }
}

static uint8_t pfbuf[BUFLEN];
void comms::printf(const char* format, ...) {
  va_list argptr;
  va_start(argptr, format);
  char *ptr = (char*) pfbuf;
  (*ptr++) = PACKET_START_BYTE;
  (*ptr++) = PACKET_START_BYTE;
  vsnprintf(ptr, sizeof(pfbuf)-2, format, argptr);
  va_end(argptr);
  if (!hw::uart_busy(/*logging =*/true)) {
    hw::uart_send(BUFLEN, /*logging =*/ true);
  }
}

