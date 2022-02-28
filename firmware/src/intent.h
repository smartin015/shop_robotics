#ifndef INTENT_H
#define INTENT_H
#include <stdint.h>

namespace intent {

#define MAX_INTENT_LENGTH_USEC (10*1000*1000)
#define INTENT_PACKET_SZ (sizeof(uint8_t) + sizeof(int16_t) + sizeof(int16_t) + sizeof(uint32_t))
struct intent_t {
  uint8_t curve_id;
  int16_t shift_y;
  int16_t scale_y; // fixed-point integer multiplier (1 sign bit, 7 bit whole, 8 bit fractional)
  uint32_t length_usec;
};
bool push(const uint8_t j, const uint8_t *buf);
const intent_t& get(const uint8_t j);
bool pop(const uint8_t j);
bool empty(const uint8_t j);
void reset();
uint8_t free(const uint8_t j);

} //namespace intent

#endif // INTENT_H
