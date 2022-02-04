#ifndef _HW_H
#define _HW_H

#include <unistd.h>
#include <stdint.h>

#define HIGH true
#define LOW false

void delayMicroseconds(uint16_t us);
uint64_t millis();

namespace hw {
  bool get_limit(int idx);
  int get_encoder(int idx);

  void move_steps(int idx, int delta);
  
  void init();
  void loop();

} // namespace hw

#endif
