#ifndef _HW_H
#define _HW_H

#include <unistd.h>
#include <stdint.h>

#define HIGH true
#define LOW false

void delayMicroseconds(uint16_t us);
uint64_t millis();

namespace hw {
  bool get_cur_cal(int idx);
  int get_steps(int idx);

  void move_steps(int idx, int delta);
  
  void sync(); 
  void init();
  void loop();

} // namespace hw

#endif
