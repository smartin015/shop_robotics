#include "intent.h"
#include "curves.h"
#include "log.h"

#define INTENT_BUFSZ 8

namespace intent {

uint8_t idx[NUM_J];
uint8_t num[NUM_J];
intent_t ring[NUM_J][INTENT_BUFSZ];
const intent_t DECEL = {0, 0, 0, 0};

void reset() {
  for (uint8_t i = 0; i < NUM_J; i++) {
    idx[i] = 0;
    num[i] = 0;
    for (uint8_t j = 0; j < INTENT_BUFSZ; j++) {
      ring[i][j] = DECEL;
    }
  }
}

void deserialize(intent_t* dest, const uint8_t* buf) {
  dest->curve_id = *((uint8_t*)buf++);
  dest->shift_y = *((int16_t*)buf); buf += sizeof(int16_t);
  dest->scale_y = *((int16_t*)buf); buf += sizeof(int16_t);
  dest->length_usec = *((uint32_t*)buf); 
}

bool empty(uint8_t j) {
  return !num[j];
}

bool disjoint(const intent_t* first, const intent_t* second) {
  if (second->curve_id >= NCURVES) {
    return false;
  }
  int16_t first_end = CURVES[first->curve_id][CURVE_SZ-1] * first->scale_y + first->shift_y;
  int16_t second_start = CURVES[second->curve_id][0] * second->scale_y + second->shift_y;
  return first_end != second_start;
}

bool push(uint8_t j, const uint8_t *buf) {
  if (j > NUM_J || num[j] == INTENT_BUFSZ) {
    return false;
  }
  auto* dest = &(ring[j][(idx[j]+num[j]) % INTENT_BUFSZ]);
  deserialize(dest, buf);

  if (!empty(j)) {
    auto* last = &(ring[j][(idx[j]+num[j]-1) % INTENT_BUFSZ]);
    if (disjoint(last, dest)) {
      return false;
    }
  } 

  num[j]++; 
  LOG_DEBUG("< J%d C%d - %d %d - T%u", j, dest->curve_id, dest->shift_y, dest->scale_y, dest->length_usec);
  return true;
}
const intent_t& get(const uint8_t j) {
  return ring[j][idx[j]];
};
bool pop(const uint8_t j) {
  if (num[j] == 0) {
    return false;
  }
  ring[j][idx[j]] = DECEL;
  idx[j] = (idx[j] + 1) % INTENT_BUFSZ;
  num[j]--;
  return true;
}
uint8_t free(const uint8_t j) {
  return INTENT_BUFSZ - num[j];
}


} // namespace intent
