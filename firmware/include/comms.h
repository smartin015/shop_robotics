#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

namespace comms {

void init();
int read(uint8_t* buf, int buflen);
void write(uint8_t* buf, int buflen);
bool available();
void printf(const char* format, ...);

} // namespace comms

#endif // COMMS_H
