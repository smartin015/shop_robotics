#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdint.h>

const int32_t settings_MIN_POS[NUM_J];
const int32_t settings_MAX_POS[NUM_J];
const int16_t settings_MIN_VEL[NUM_J];
const int16_t settings_MAX_VEL[NUM_J];
const int16_t settings_MIN_ACC[NUM_J];
const int16_t settings_MAX_ACC[NUM_J];
const int16_t settings_MIN_JERK[NUM_J];
const int16_t settings_MAX_JERK[NUM_J];

const int32_t settings_MIN_POS_ERR[NUM_J];
const int32_t settings_MAX_POS_ERR[NUM_J];
const int16_t settings_MIN_VEL_ERR[NUM_J];
const int16_t settings_MAX_VEL_ERR[NUM_J];
const int16_t settings_MIN_ACC_ERR[NUM_J];
const int16_t settings_MAX_ACC_ERR[NUM_J];

const uint16_t settings_DECEL_RATE[NUM_J];
// TODO emergency decel rate
// TODO steps/rev
// TODO PID values

#endif // SETTINGS_H
