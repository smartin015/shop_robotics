#ifndef MOTION_H
#define MOTION_H

namespace motion {

#define MOTION_MSG_SZ (2*sizeof(uint8_t) + NUM_J*(sizeof(uint8_t) + sizeof(int32_t) + sizeof(int16_t) + sizeof(int16_t)))

void write();
void read();
void serialize(uint8_t* buf);
void decelerate();
bool decelerating();

} // namespace motion

#endif // MOTION_H
