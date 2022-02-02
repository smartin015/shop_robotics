#ifndef __APP_HAL_H__
#define __APP_HAL_H__

// Device-specific constants and includes go in a hw.h file in `hal/<arch>/
#include "hw.h"

namespace hal {

void init();
void stepDir(int i, bool dir);
void stepDn(int i);
void stepUp(int i);
void stepEnabled(int i, bool en);
bool readLimit(int i);
int readEnc(int idx);
void writeEnc(int idx, int value);
void startMainTimer(int hz, void(*cb)());
void disableInterrupts();
void enableInterrupts();

} // namespace hal

#endif // __APP_HAL_H__
