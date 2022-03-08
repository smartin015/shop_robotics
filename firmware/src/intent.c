#include "intent.h"
#include "curves.h"
#include "log.h"
#include <stdbool.h>

#define INTENT_BUFSZ 8

uint8_t idx[NUM_J];
uint8_t num[NUM_J];
struct intent_intent_t ring[NUM_J][INTENT_BUFSZ];
const struct intent_intent_t DECEL = {0, 0, 0, 0};

void intent_reset() {
  for (uint8_t i = 0; i < NUM_J; i++) {
    idx[i] = 0;
    num[i] = 0;
    for (uint8_t j = 0; j < INTENT_BUFSZ; j++) {
      ring[i][j] = DECEL;
    }
  }
}

void intent_deserialize(struct intent_intent_t* dest, const uint8_t* buf) {
  dest->curve_id = *((uint8_t*)buf++);
  dest->shift_y = *((int16_t*)buf); buf += sizeof(int16_t);
  dest->scale_y = *((int16_t*)buf); buf += sizeof(int16_t);
  dest->length_usec = *((uint32_t*)buf); 
}

bool intent_empty(uint8_t j) {
  return !num[j];
}

struct intent_status_t intent_push(uint8_t j, const uint8_t *buf, uint8_t sz) {
  struct intent_status_t err;
  if (sz != INTENT_PACKET_SZ) {   
    err.code = PUSH_ERR_INVALID;
    snprintf(err.message, STATUS_MSG_SZ, "bad sz %d; want %d", sz, INTENT_PACKET_SZ);
    return err;
  } else if (j > NUM_J) {
    err.code = PUSH_ERR_INVALID;
    snprintf(err.message, STATUS_MSG_SZ, "invalid joint");
    return err;
  } else if (num[j] == INTENT_BUFSZ) {
    err.code = PUSH_ERR_FULL;
    snprintf(err.message, STATUS_MSG_SZ, "intent queue full");
    return err;
  } 
  struct intent_intent_t* dest = &(ring[j][(idx[j]+num[j]) % INTENT_BUFSZ]);
  intent_deserialize(dest, buf);
  if (dest->curve_id >= NCURVES) {
    err.code = PUSH_ERR_INVALID;
    snprintf(err.message, STATUS_MSG_SZ, "invalid curve id");
    return err;
  }

  if (!intent_empty(j)) {
    struct intent_intent_t* before = &(ring[j][(idx[j]+num[j]-1) % INTENT_BUFSZ]);
    int16_t before_end = (CURVES[before->curve_id][CURVE_SZ-1] * before->scale_y / 256) + before->shift_y;
    int16_t dest_start = (CURVES[dest->curve_id][0] * dest->scale_y / 256) + dest->shift_y;
    if (before_end != dest_start) {
      err.code = PUSH_ERR_DISJOINT;
      snprintf(err.message, STATUS_MSG_SZ, "disjoint (%d -> %d)", before_end, dest_start);
      return err;
    }
  } 

  num[j]++; 
  LOG_DEBUG("< J%d C%d - %d %d - T%u", j, dest->curve_id, dest->shift_y, dest->scale_y, dest->length_usec);
  err.code = PUSH_OK;
  return err;
}
const struct intent_intent_t* intent_get(const uint8_t j) {
  return &(ring[j][idx[j]]);
};
bool intent_pop(const uint8_t j) {
  if (num[j] == 0) {
    return false;
  }
  ring[j][idx[j]] = DECEL;
  idx[j] = (idx[j] + 1) % INTENT_BUFSZ;
  num[j]--;
  return true;
}
uint8_t intent_free(const uint8_t j) {
  return INTENT_BUFSZ - num[j];
}

