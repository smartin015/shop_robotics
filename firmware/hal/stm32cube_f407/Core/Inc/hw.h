#ifndef _HW_H
#define _HW_H

#include <stdbool.h>
#include <stdint.h>


// TODO RM
#define REPORT_PD_MILLIS 10000
#define NUM_J 6
#define STEPS_PER_REV {4000, 20000, 20000, 5600, 400, 400}
#define ENCODER_TICKS_PER_REV {2048, 2048, 2048, 2048, 2048, 2048}
#define MOTOR_MIN_RADIANS {-2.4, -2.4, -4.5, -2.4, -2.4, -3.14}
#define MOTOR_MAX_RADIANS { 2.4,  1.8,  1.3,  2.4,  2.4,  3.14}
#define JOINT_MIN_POS {-1527, -7639, -14323, -2139, -152, -199}
#define JOINT_MAX_POS {1527,   5729,   4138,  2139,  152,  199}

#define JOINT_MIN_VEL {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_VEL { 100,  100,  100,  100,  100,  100}

#define JOINT_MIN_ACC {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_ACC { 100,  100,  100,  100,  100,  100}

#define JOINT_MIN_JERK {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_JERK { 100,  100,  100,  100,  100,  100}

#define JOINT_MIN_POS_ERR {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_POS_ERR { 100,  100,  100,  100,  100,  100}

#define JOINT_MIN_VEL_ERR {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_VEL_ERR { 100,  100,  100,  100,  100,  100}

#define JOINT_MIN_ACC_ERR {-100, -100, -100, -100, -100, -100}
#define JOINT_MAX_ACC_ERR { 100,  100,  100,  100,  100,  100}

#define JOINT_DECEL_RATE {4000, 20000, 20000, 5600, 400, 400}

#define TIM_CLK 84000000
#define TIM_PSC 64
#define MIN_FREQ 20
#define MAX_FREQ 2400
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency

void hw_init();
void hw_set_dir_and_pwm(uint8_t j, int16_t hz);
bool hw_limit_read(uint8_t j);

bool hw_uart_busy();
uint8_t* hw_uart_get_write_buffer();
bool hw_uart_send(uint8_t len);

bool hw_uart_data_ready();
uint8_t* hw_uart_recv(uint8_t* sz);
void hw_uart_flip_buffer();
uint32_t hw_micros();

#endif
