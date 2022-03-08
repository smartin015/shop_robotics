#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

void comms_init();
uint8_t * comms_read(uint8_t* sz);
void comms_finishRead();
uint8_t * comms_preWrite(uint8_t sz);
void comms_flush(uint8_t sz);
void comms_printf(const char* format, ...);

#endif // COMMS_H
