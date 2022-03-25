#include "hal.h"
#include "hw.h"
#include "log.h"
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

// The amount of time we expect a full loop of the firmware to take,
// on an embedded controller.
#define SIMULATED_LOOP_TIME_USEC 100

int cur_dir[NUM_J];
bool prev_step_pin[NUM_J];

// For some reason, Ctrl+C isn't detected when
// running the native program via `pio` in Docker,
// so we have to handle it custom here.
void signal_callback_handler(int signum) {
   printf("Stop request detected, exiting...\n");
   exit(signum);
}

void hal_init() {
  // Nothing to do here; no hardware to initialize
}

uint8_t hal_readLimits() {
  return hw_get_limits();
}

int16_t hal_readEnc(uint8_t j) {
  return hw_get_encoder(j);
}
int32_t hal_readSteps(uint8_t j) {
  return hw_get_steps(j);
}

void hal_setStepRate(uint8_t j, int16_t rate) {
  return hw_set_rate(j, rate);
}

uint32_t hal_micros() {
  return hw_micros();
}

void setup();
void loop();
int main() {
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);
  printf("Init\n");
  hw_init();
  for (int i = 0; i < NUM_J; i++) {
    cur_dir[i] = 1;
    prev_step_pin[i] = true;
  }
  setup();
  printf("Beginning main loop\n");
  while (1) {
    // Hidden hardware emulation; includes setting micros() to sim clock
    // Returns false when we've overrun the simulator clock
    hw_loop(); 
    while (hw_advance(SIMULATED_LOOP_TIME_USEC)) {
      loop();
    }
  }
}
