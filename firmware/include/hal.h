#ifndef __APP_HAL_H__
#define __APP_HAL_H__

// Device-specific constants and includes go in a hw.h file in `hal/<arch>/
#include "hw.h"

namespace hal {

uint32_t micros();
void init();
bool readLimit(uint8_t j);
void setStepRate(uint8_t j, int16_t rate);
int16_t readEnc(uint8_t j);
int32_t readSteps(uint8_t j);

} // namespace hal

#endif // __APP_HAL_H__
