#ifndef _HW_H
#define _HW_H

#include <stdbool.h>
#include <stdint.h>

#define TIM_CLK 84000000
#define TIM_PSC 64
#define MIN_FREQ 20
#define MAX_FREQ 2400
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency

void hw_init();
void hw_set_dir_and_pwm(uint8_t j, int16_t hz);
bool hw_limit_read(uint8_t j);

bool hw_uart_busy(bool logging);
uint8_t* hw_uart_get_write_buffer(bool logging);
bool hw_uart_send(uint8_t len, bool logging);

bool hw_uart_data_ready();
uint8_t* hw_uart_recv(uint8_t* sz);
void hw_uart_flip_buffer();
uint32_t hw_micros();

#endif
