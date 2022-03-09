#include "hal.h"
#include "hw.h"
#include "log.h"
#include "settings.h"

int8_t dirs[NUM_J];
int64_t usteps[NUM_J];
int16_t rates[NUM_J];

uint32_t hal_micros() {
  return hw_micros();
}

void hal_init() {
  for (int i = 0; i < NUM_J; i++) {
    usteps[i] = 0;
    dirs[i] = 1;
    rates[i] = 0;
  }
}

uint32_t last_sync_usec = 0;
void syncSteps() {
  uint32_t now = hw_micros();
  if (now < last_sync_usec) {
    // Rollover
    // TODO handle this better
    last_sync_usec = now;
    return;
  }

  for (int i = 0; i < NUM_J; i++) {
    usteps[i] += (dirs[i] * rates[i] * (now - last_sync_usec));
  }
  last_sync_usec = now;
}

void hal_setStepRate(uint8_t j, int16_t rate) {
  // TODO set direction GPIO
  syncSteps();
  hw_set_dir_and_pwm(j, rate);
}

uint8_t hal_readLimit(uint8_t j) {
  return hw_limit_read(j);
}
int16_t hal_readEnc(uint8_t i) {
  return 0; // TODO
}
int32_t hal_readSteps(uint8_t j) {
  syncSteps(); 
  return usteps[j]/1000000;
}

