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
  comms::init(); // Init comms first; may be needed to safely log outputs

  LOG_INFO("Setup begin for %d joint robot (report every %dms)", NUM_J, REPORT_PD_MILLIS);
  hal::init();
  pid::reset();
  intent::reset();

  // TODO configure limit switch interrupts

  LOG_INFO("Configuring main timing loop for %d hz", MOTION_WRITE_HZ);
  hal::startMainTimer(MOTION_WRITE_HZ, &motion::write);

  LOG_INFO("Printing firmware settings:");
  // TODO

  LOG_INFO("Setup complete");
}

uint8_t buf[128];
void loop() {
  // NOTE: Casting directly to struct requires both the same endianness and same interpetation of floating point
  // numbers. (see https://stackoverflow.com/questions/13775893/converting-struct-to-byte-and-back-to-struct)
  // Explicit deserialization prevents unexpected errors in data format.
  int sz = comms::read(buf, sizeof(buf));
  if (sz == INTENT_PACKET_SZ + sizeof(uint8_t)) {
    if (!motion::decelerating()) {
      uint8_t j = buf[0];
      if (!intent::push(j, (buf+1))) {
        LOG_ERROR("ERR joint %d disjoint or full, triggering e-stop", j);
        motion::decelerate();
      }
    }
  } else if (sz != 0) {
    LOG_ERROR("ERR BADPACKET SZ %d - WANT %d", sz, INTENT_PACKET_SZ+sizeof(uint8_t));
  }
  
  motion::read();
  motion::write();

  if (comms::available()) {
    motion::serialize(buf);
    // Also add the current micros so we can check for sync
    *((uint64_t*) (buf + MOTION_MSG_SZ)) = micros();
    comms::write(buf, MOTION_MSG_SZ + sizeof(uint64_t));
  }
}
