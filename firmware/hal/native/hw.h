#ifndef _HW_H
#define _HW_H

#include <unistd.h>
#include <stdint.h>

#define HIGH true
#define LOW false

void delayMicroseconds(uint16_t us);
uint64_t millis();
uint64_t micros();

namespace hw {
  bool get_limit(uint8_t j);
  int32_t get_steps(uint8_t j);
  int16_t get_encoder(uint8_t j);

  void set_rate(uint8_t j, int16_t rate);
  
  void init();
  void loop();
  bool advance(uint16_t usec);

} // namespace hw

#endif
