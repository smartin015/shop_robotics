#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

#define PACKET_START_BYTE 0xaa

uint8_t * comms_preWrite(uint8_t sz);
void comms_flush(uint8_t sz);
void comms_printf(const char* format, ...);

#endif // COMMS_H
