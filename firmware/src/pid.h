#ifndef PID_H
#define PID_H

namespace pid {

// Initializes motor states.
void reset();

// Returns motor rate to use to conform to the given position and velocity trajectory
// with fine-tuning via encoder. A return value of `false` indicates an error.
bool update(int16_t* dest, uint8_t j, int16_t pos, int16_t pos_target, int16_t vel_target);

} // namespace pid

#endif // PID_H
