#include "hal.h"
#include "hw.h"
#include "log.h"

namespace hal {

int8_t dirs[NUM_J];
int64_t usteps[NUM_J];
int16_t rates[NUM_J];

uint32_t micros() {
  return hw::micros();
}

void init() {
  hw::init();
  for (int i = 0; i < NUM_J; i++) {
    usteps[i] = 0;
    dirs[i] = 1;
    rates[i] = 0;
  }
}

uint64_t last_sync_usec = 0;
void syncSteps() {
  uint64_t now = hw::micros();
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

void setStepRate(uint8_t j, int16_t rate) {
  // TODO set direction GPIO
  syncSteps();
  hw::set_dir_and_pwm(j, rate);
}

bool readLimit(uint8_t j) {
  return hw::limit_read(j);
}
int16_t readEnc(uint8_t i) {
  return 0; // TODO
}
int32_t readSteps(uint8_t j) {
  syncSteps(); 
  return usteps[j]/1000000;
}

} // namespace hal

void setup();
void loop();
int main() {
  hw::init();
  setup();
  while (1) {
    loop();
  }
}
