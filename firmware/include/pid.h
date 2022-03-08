#ifndef PID_H
#define PID_H

#include <stdbool.h>

// Initializes motor states.
void pid_reset();

// Returns motor rate to use to conform to the given position and velocity trajectory
// with fine-tuning via encoder. A return value of `false` indicates an error.
bool pid_update(int16_t* dest, uint8_t j, int16_t pos, int16_t pos_target, int16_t vel_target);

#endif // PID_H
