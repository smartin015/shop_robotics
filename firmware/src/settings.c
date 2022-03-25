#include "settings.h"
#include <stdint.h>

// STM32 implementatoin sets the constant initializers in hw.h
#include "hw.h"

const int32_t settings_MIN_POS[NUM_J]=JOINT_MIN_POS;
const int32_t settings_MAX_POS[NUM_J]=JOINT_MAX_POS;
const int16_t settings_MIN_VEL[NUM_J]=JOINT_MIN_VEL;
const int16_t settings_MAX_VEL[NUM_J]=JOINT_MAX_VEL;
const int16_t settings_MIN_ACC[NUM_J]=JOINT_MIN_ACC;
const int16_t settings_MAX_ACC[NUM_J]=JOINT_MAX_ACC;
const int16_t settings_MIN_JERK[NUM_J]=JOINT_MIN_JERK;
const int16_t settings_MAX_JERK[NUM_J]=JOINT_MAX_JERK;

const int32_t settings_MIN_POS_ERR[NUM_J]=JOINT_MIN_POS_ERR;
const int32_t settings_MAX_POS_ERR[NUM_J]=JOINT_MAX_POS_ERR;
const int16_t settings_MIN_VEL_ERR[NUM_J]=JOINT_MIN_VEL_ERR;
const int16_t settings_MAX_VEL_ERR[NUM_J]=JOINT_MAX_VEL_ERR;
const int16_t settings_MIN_ACC_ERR[NUM_J]=JOINT_MIN_ACC_ERR;
const int16_t settings_MAX_ACC_ERR[NUM_J]=JOINT_MAX_ACC_ERR;

const uint16_t settings_DECEL_RATE[NUM_J]=JOINT_DECEL_RATE;
