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

status_t push(uint8_t j, const uint8_t *buf, uint8_t sz) {
  if (sz != INTENT_PACKET_SZ) {   
    status_t err = {PUSH_ERR_INVALID, ""};
    snprintf(err.message, STATUS_MSG_SZ, "bad sz %d; want %d", sz, INTENT_PACKET_SZ);
    return err;
  } else if (j > NUM_J) {
    return status_t{PUSH_ERR_INVALID, "invalid joint"};
  } else if (num[j] == INTENT_BUFSZ) {
    return status_t{PUSH_ERR_FULL, "intent queue full"};
  } 
  auto* dest = &(ring[j][(idx[j]+num[j]) % INTENT_BUFSZ]);
  deserialize(dest, buf);
  if (dest->curve_id >= NCURVES) {
    return status_t{PUSH_ERR_INVALID, "invalid curve id"};
  }

  if (!empty(j)) {
    auto* before = &(ring[j][(idx[j]+num[j]-1) % INTENT_BUFSZ]);
    int16_t before_end = (CURVES[before->curve_id][CURVE_SZ-1] * before->scale_y / 256) + before->shift_y;
    int16_t dest_start = (CURVES[dest->curve_id][0] * dest->scale_y / 256) + dest->shift_y;
    if (before_end != dest_start) {
      status_t err = {PUSH_ERR_DISJOINT, ""};
      snprintf(err.message, STATUS_MSG_SZ, "disjoint (%d -> %d)", before_end, dest_start);
      return err;
    }
  } 

  num[j]++; 
  LOG_DEBUG("< J%d C%d - %d %d - T%u", j, dest->curve_id, dest->shift_y, dest->scale_y, dest->length_usec);
  return status_t{PUSH_OK, ""};
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
