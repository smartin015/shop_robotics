/* Motion planner for AR3 robotic arm
 *
 * motion_write() frequently updates stepper rates to meet
 * the trajectory.
 */
#include "log.h"
#include "hal.h"
#include "pid.h"
#include "settings.h"
#include <stdbool.h>

#define ABS(v) ((v > 0) ? v : -v)
#define MIN(a,b) ((a<b) ? a : b)
#define MAX(a,b) ((a<b) ? b : a)

// To safeguard against e.g. integer overflow causing the robot to lose control,
// any computed P, I, or D contributions beyond this limit trigger an emergency
// deceleration of the robot that can only be reset by cycling power to the robot.
// It is assumed that the robot can be power cycled in a safe manner.
#define HARD_MAX_PID_CONTRIBUTION 100000


float prev_vel[NUM_J];
float err_vel[NUM_J];
float prev_err_vel[NUM_J];

int prev_pos[NUM_J];
int err_pos[NUM_J];
int prev_err_pos[NUM_J];

float pid_updates[NUM_J][3];
bool active[NUM_J];

const uint16_t steps_per_rev[NUM_J] = STEPS_PER_REV;
const uint16_t encoder_usec_per_rev[NUM_J] = ENCODER_TICKS_PER_REV;

void pid_reset() {
  for (int i = 0; i < NUM_J; i++) {
    prev_pos[i] = 0;
    prev_vel[i] = 0;
    prev_err_vel[i] = 0;
    err_vel[i] = 0;
    err_pos[i] = 0;
    for (int j = 0; j < 3; j++) {
      pid_updates[i][j] = 0;
    }
  }
}

bool pid_update(int16_t* dest, uint8_t j, int16_t pos, int16_t pos_target, int16_t vel_target) {
  // NOTE: starting with just a PI controller since we only have an encoder as real input - taking a discrete second derivative 
  // could be quite noisy.
  // TODO tweak dest by applying PID to pos, vel, enc
  // IMPORTANT: discrete derivative can be noisy, should apply filtering or else PID may behave strangely.
  // TODO anti-windup on the I component to prevent the robot fighting a colliding object
  
  *dest = vel_target;
  return true;
} 
