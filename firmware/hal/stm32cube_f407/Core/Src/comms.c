#include "comms.h"
#include "hw.h"
#include "log.h"
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */
#include <stdio.h>      /* printf */

uint8_t* comms_read(uint8_t* sz) {
  return hw_uart_recv(sz);
}

void comms_finishRead() {
  return hw_uart_flip_buffer();
} 

uint8_t* comms_preWrite(uint8_t sz) {
  if (hw_uart_busy()) {
    return 0;
  }
   uint8_t* ptr = hw_uart_get_write_buffer();
  (*ptr++) = PACKET_START_BYTE;
  (*ptr++) = sz;
  return ptr;
}

void comms_flush(uint8_t sz) {
  if (!hw_uart_send(sz)) {
    LOG_ERROR("SNDFAIL (sz %d)", sz);
  }
}

