#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

#define MOTION_MSG_SZ (2*sizeof(uint8_t) + NUM_J*(sizeof(uint8_t) + sizeof(int32_t) + sizeof(int16_t) + sizeof(int16_t)))

void motion_write();
void motion_read();
void motion_serialize(uint8_t* buf);
void motion_decelerate();
bool motion_decelerating();
void motion_resume();

#endif // MOTION_H
