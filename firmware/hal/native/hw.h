#ifndef _HW_H
#define _HW_H

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#define HIGH true
#define LOW false

// These are to be implemented in src/loop.c
void on_message(uint8_t *buf, uint8_t sz);
void on_limits_changed();

uint32_t hw_micros();

uint8_t hw_get_limits();
int32_t hw_get_steps(uint8_t j);
int16_t hw_get_encoder(uint8_t j);

void hw_set_rate(uint8_t j, int16_t rate);

void hw_init();
void hw_loop();
bool hw_advance(uint16_t usec);

#endif
