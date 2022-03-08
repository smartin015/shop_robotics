#ifndef __APP_HAL_H__
#define __APP_HAL_H__

#include <stdint.h>

uint32_t hal_micros();
void hal_init();
uint8_t hal_readLimit(uint8_t j);
void hal_setStepRate(uint8_t j, int16_t rate);
int16_t hal_readEnc(uint8_t j);
int32_t hal_readSteps(uint8_t j);

#endif // __APP_HAL_H__
