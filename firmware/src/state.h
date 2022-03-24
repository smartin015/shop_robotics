#ifndef STATE_H
#define STATE_H

#define VEL_REAL(x) (x >> 8)
#define VEL_FIXED(x) (x << 8)

#include <stdint.h>

// Enable position or velocity based commands to this joint
// If either of these are set, enable the stepper motor driver for this joint
#define MASK_POS_ENABLED (0b00000001)
#define MASK_VEL_ENABLED (0b00000010)

// For intent, this describes whether or not to ignore
// the braking behavior when any limit is triggered.
// For actual, this indicates a limit was triggered.
// Note the arm will stop moving if any joint isn't matching
// the intent.
#define MASK_LIMIT_TRIGGERED (0b00000100)

// When this flag is active, match targets against encoder positions (not set implies open loop control)
#define MASK_ENCODER_ENABLED (0b00001000)

#define DEFAULT_MAX_ACCEL 40
#define DEFAULT_INITIAL_SPD 10

// Number of stepper steps we're allowed to be off by before we shut off the motor drivers
// and call it close enough. Higher values make for less precision, but also less jitter/noisiness at rest
#define VEL_DEAD_ZONE 10
#define POS_DEAD_ZONE 10

// These are passed as -D flags in platformio.ini
//const uint16_t MOTOR_REV_STEPS = STEPS_PER_REV;
//const uint16_t ENCODER_REV_STEPS = ENCODER_TICKS_PER_REV;

namespace state {

#define MOTION_MSG_SZ (NUM_J * ((sizeof(uint8_t)) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t)))
struct state_t {
  uint8_t mask[NUM_J];
  int32_t pos[NUM_J];
  int16_t enc[NUM_J];
  float vel[NUM_J];
};

#define SETTINGS_MSG_SZ (7*sizeof(int16_t))
struct settings_t {
  float pid[3] = {0.35, 0.5, 0.4}; // Tuning for motion
  int velocity_update_pd_millis = 100;
  float max_accel = DEFAULT_MAX_ACCEL; // NOTE: must be at least as large as state::settings.initial_spd
  float max_spd = 5000;
  float initial_spd = DEFAULT_INITIAL_SPD;
};

extern state_t intent;
extern state_t actual;
extern settings_t settings;

void serialize(uint8_t* buf, state_t* state);
void deserialize(state_t* state, uint8_t* buf);
void apply_settings(settings_t* settings, uint8_t* buf);
void print_settings(const settings_t* settings);

} //namespace state

#endif // STATE_H