#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

namespace comms {

void init();
uint8_t * read(uint8_t* sz);
void finishRead();
uint8_t * preWrite(uint8_t sz);
void flush(uint8_t sz);
void printf(const char* format, ...);

} // namespace comms

#endif // COMMS_H
