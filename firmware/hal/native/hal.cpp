#include "hal.h"
#include "hw.h"
#include "log.h"
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <thread>

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

namespace hal {

void init() {
  // Nothing to do here; no hardware to initialize
}

bool readLimit(uint8_t j) {
  return hw::get_limit(j);
}

int16_t readEnc(uint8_t j) {
  return hw::get_encoder(j);
}
int32_t readSteps(uint8_t j) {
  return hw::get_steps(j);
}

void setStepRate(uint8_t j, int16_t rate) {
  return hw::set_rate(j, rate);
}

void disableInterrupts() {}
void enableInterrupts() {}

void runSteps(int hz, void(*cb)()) {
  int micro_pd = 1000000 / hz;
  std::chrono::microseconds tick_pd = std::chrono::microseconds(micro_pd);
  std::chrono::steady_clock::time_point now;
  std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
  while (true) {
    now = std::chrono::steady_clock::now();
    if (last_tick + tick_pd > now) {
      continue;
    }
    // It's likely we need to catch up in calls, given that we're not running on an RTOS
    while (last_tick < now) {
      last_tick += tick_pd;
      cb();
    }
    usleep(micro_pd); // NOTE: not actually a guarantee, likely to be consistently longer
  }
}

} // namespace hal


void setup();
void loop();
int main() {
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);
  hw::init();
  for (int i = 0; i < NUM_J; i++) {
    cur_dir[i] = 1;
    prev_step_pin[i] = true;
  }
  setup();
  while (1) {
    // Hidden hardware emulation; includes setting micros() to sim clock
    // Returns false when we've overrun the simulator clock
    hw::loop(); 
    while (hw::advance(SIMULATED_LOOP_TIME_USEC)) {
      loop();
    }
  }
}
