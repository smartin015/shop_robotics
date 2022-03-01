/* Motion planner for AR3 robotic arm
 *
 * motion::write() frequently updates stepper rates to meet
 * the trajectory.
 */
#include "log.h"
#include "hal.h"
#include "motion.h"
#include "pid.h"
#include "intent.h"
#include "curves.h"
#include "settings.h"

#define ABS(v) ((v > 0) ? v : -v)
#define MIN(a,b) ((a<b) ? a : b)
#define MAX(a,b) ((a<b) ? b : a)

uint8_t motion_mask = 0;
uint8_t limit_mask = 0;
int32_t pos[NUM_J];
int16_t enc[NUM_J];
int16_t vel[NUM_J];
int16_t decel_vel_start[NUM_J];
uint32_t curve_start_usec = 0;

void motion::decelerate() {
  motion_mask = 1;
  curve_start_usec = hal::micros();
  for (int i = 0; i < NUM_J; i++) {
    decel_vel_start[i] = vel[i];
  }
}
bool motion::decelerating() {
  return (motion_mask == 1);
}

// Note: this is likely called inside a timer interrupt
// so care must be taken to keep overall cycles light.
// Logging and other debugging calls are prohibited.
void motion::write() {
  uint32_t now = hal::micros(); // note: truncated from uint64
  if (decelerating()) {
    uint32_t dt_usec = now - curve_start_usec;
    uint32_t velsum = 0;
    for (uint8_t i = 0; i < NUM_J; i++) {
      if (decel_vel_start[i] > 0) {
        vel[i] = MAX(0, decel_vel_start[i] - int(settings::DECEL_RATE[i] * (dt_usec/1000))/1000);
      } else {
        vel[i] = MIN(0, decel_vel_start[i] + int(settings::DECEL_RATE[i] * (dt_usec/1000))/1000);
      }
      velsum += ABS(vel[i]);
    }
    // LOG_DEBUG("ST %d RT %d DT %d -> VEL %d", decel_vel_start[0], settings::DECEL_RATE[0], dt_usec, vel[0]);
    if (!velsum) { // We've stopped
      intent::reset();
      motion_mask = 0;
    }
    return;
  }

  for (uint8_t i = 0; i < NUM_J; i++) {
    if (intent::empty(i)) {
      if (vel[i] != 0) {
        decelerate();
        return;
      }
      continue;
    }
    auto& jc = intent::get(i);
    
    // Compute where we should be in the curve we're playing out
    // TODO handle overflow
    uint16_t curve_idx = ((now - curve_start_usec) * CURVE_SZ) / jc.length_usec;
    // LOG_DEBUG("%u %u %u -> %d", now, curve_start_usec, jc.length_usec, curve_idx);
    if (curve_idx >= CURVE_SZ) {
      intent::pop(i);
      auto& jnext = intent::get(i);
      curve_start_usec = now;
      LOG_DEBUG("pop %d; next: C%d", i, jnext.curve_id);
      return;
    }

    // TODO linear interpolate between curve_idx and next value, where possible

    // Divide by 256 to account for scale_y being fixed-point
    int16_t vel_tgt = (CURVES[jc.curve_id][curve_idx] * jc.scale_y / 256) + jc.shift_y;
    if (!pid::update(&(vel[i]), i, pos[i], enc[i], vel_tgt)) {
      return; // TODO throw error
    }
    hal::setStepRate(i, vel[i]);
  }
}

void motion::read() {
  for (int i = 0; i < NUM_J; i++) {
    enc[i] = hal::readEnc(i);
    pos[i] = hal::readSteps(i);
  }
}

void motion::serialize(uint8_t* buf) {
  uint8_t* ptr = buf;
  *(ptr++) = limit_mask;
  *(ptr++) = motion_mask;
  for (int i = 0; i < NUM_J; i++) {
    *(ptr++) = intent::free(i);
  }
  for (int i = 0; i < NUM_J; i++) {
    *((int32_t*)ptr) = pos[i];
    ptr += sizeof(int32_t);
  } 
  for (int i = 0; i < NUM_J; i++) {
    *((int16_t*)ptr) = vel[i];
    ptr += sizeof(int16_t);
  } 
  for (int i = 0; i < NUM_J; i++) {
    *((int16_t*)ptr) = enc[i];
    ptr += sizeof(int16_t);
  } 
}
