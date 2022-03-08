#include "comms.h"
#include "hw.h"
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */
#include <stdio.h>      /* vsnprintf */

#define BUFLEN 128
uint8_t serbuf[BUFLEN];
int16_t idx = -1;
int16_t readlen = 0;

#define PACKET_START_BYTE 0x02

void comms_init() {}

uint8_t* comms_read(uint8_t* sz) {
  return hw_uart_recv(sz);
}
void comms_finishRead() {
  return hw_uart_flip_buffer();
} 

uint8_t* comms_preWrite(uint8_t sz) {
  if (hw_uart_busy(/*logging =*/ false)) {
    return 0;
  }
   uint8_t* ptr = hw_uart_get_write_buffer(/*logging =*/ false);
  (*ptr++) = PACKET_START_BYTE;
  (*ptr++) = sz;
  return ptr;
}

void comms_flush(uint8_t sz) {
  if (!hw_uart_send(sz, /*logging =*/ false)) {
    comms_printf("SNDFAIL (sz %d)", sz);
  }
}

static uint8_t pfbuf[BUFLEN];
void comms_printf(const char* format, ...) {
  va_list argptr;
  va_start(argptr, format);
  char *ptr = (char*) pfbuf;
  (*ptr++) = PACKET_START_BYTE;
  (*ptr++) = PACKET_START_BYTE;
  vsnprintf(ptr, sizeof(pfbuf)-2, format, argptr);
  va_end(argptr);
  if (!hw_uart_busy(/*logging =*/true)) {
    hw_uart_send(BUFLEN, /*logging =*/ true);
  }
}

