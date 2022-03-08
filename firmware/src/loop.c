#include "log.h"
#include "hal.h"
#include "comms.h"
#include "motion.h"
#include "pid.h"
#include "settings.h"
#include "intent.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

void setup() {
  comms_init(); // Init comms first; may be needed to safely log outputs

  LOG_INFO("Setup begin for %d joint robot (report every %dms)", NUM_J, REPORT_PD_MILLIS);
  hal_init();
  pid_reset();
  intent_reset();

  // TODO configure limit switch interrupts

  LOG_INFO("Printing firmware settings:");
  // TODO

  LOG_INFO("Setup complete");
}

#define MOTION_LOOP_PD_MICROS 10000
uint32_t next_motion_update = 0;
void loop() {
  // NOTE: Casting directly to struct requires both the same endianness and same interpetation of floating point
  // numbers. (see https://stackoverflow.com/questions/13775893/converting-struct-to-byte-and-back-to-struct)
  // Explicit deserialization prevents unexpected errors in data format.
  uint8_t sz = 0;
  uint8_t* bufptr;
  struct intent_status_t status;
  if ((bufptr = comms_read(&sz)) != NULL) {
    if (!motion_decelerating()) {
      uint8_t j = bufptr[0];
      status = intent_push(j, (bufptr+1), sz-1);
    }
    comms_finishRead();
  }

  if (bufptr != NULL && status.code != PUSH_OK) {
    LOG_ERROR("PUSH ERR %d: %s", status.code, status.message);
    motion_decelerate();
  }
  
  uint32_t now = hal_micros();
  if (next_motion_update < UINT32_MAX && now > UINT32_MAX) {
    // Handle overflow period
    return;
  } else if (now > next_motion_update) {
    motion_read();
    motion_write();

    uint8_t *wbuf = comms_preWrite(MOTION_MSG_SZ + sizeof(uint64_t));
    if (wbuf != NULL) {
      motion_serialize(wbuf);
      // Also add the current micros so we can check for sync
      *((uint32_t*) (wbuf + MOTION_MSG_SZ)) = hal_micros();
      comms_flush(MOTION_MSG_SZ + sizeof(uint64_t));
    }
    next_motion_update = now + MOTION_LOOP_PD_MICROS;
  }
}