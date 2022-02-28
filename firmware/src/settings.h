#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdint.h>

namespace settings {

const int32_t MIN_POS[NUM_J]=JOINT_MIN_POS;
const int32_t MAX_POS[NUM_J]=JOINT_MAX_POS;
const int16_t MIN_VEL[NUM_J]=JOINT_MIN_VEL;
const int16_t MAX_VEL[NUM_J]=JOINT_MAX_VEL;
const int16_t MIN_ACC[NUM_J]=JOINT_MIN_ACC;
const int16_t MAX_ACC[NUM_J]=JOINT_MAX_ACC;
const int16_t MIN_JERK[NUM_J]=JOINT_MIN_JERK;
const int16_t MAX_JERK[NUM_J]=JOINT_MAX_JERK;

const int32_t MIN_POS_ERR[NUM_J]=JOINT_MIN_POS_ERR;
const int32_t MAX_POS_ERR[NUM_J]=JOINT_MAX_POS_ERR;
const int16_t MIN_VEL_ERR[NUM_J]=JOINT_MIN_VEL_ERR;
const int16_t MAX_VEL_ERR[NUM_J]=JOINT_MAX_VEL_ERR;
const int16_t MIN_ACC_ERR[NUM_J]=JOINT_MIN_ACC_ERR;
const int16_t MAX_ACC_ERR[NUM_J]=JOINT_MAX_ACC_ERR;

const uint16_t DECEL_RATE[NUM_J]=JOINT_DECEL_RATE;
// TODO emergency decel rate
// TODO steps/rev
// TODO PID values

} //namespace settings

#endif // SETTINGS_H
