#ifndef _HW_H
#define _HW_H

#include <stdint.h>

#define TIM_CLK 84000000
#define TIM_PSC 64
#define MIN_FREQ 20
#define MAX_FREQ 2400
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency

namespace hw {

void init();
void set_dir_and_pwm(uint8_t j, int16_t hz);
bool limit_read(uint8_t j);

bool uart_busy(bool logging);
uint8_t* uart_get_write_buffer(bool logging);
bool uart_send(uint8_t len, bool logging);

bool uart_data_ready();
uint8_t* uart_recv(uint8_t* sz);
void uart_flip_buffer();
uint32_t micros();

} // namespace hw

#endif
