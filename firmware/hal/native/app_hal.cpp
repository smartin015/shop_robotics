#include "app_hal.h"
#include "hw.h"
#include "log.h"
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <thread>

// The amount of time we expect a full loop of the firmware to take,
// on an embedded controller.
#define SIMULATED_LOOP_TIME_MILLIS 2

int cur_dir[NUM_J];
bool prev_step_pin[NUM_J];
int step_offs[NUM_J];

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

void stepDir(int i, bool dir) {
  cur_dir[i] = (dir) ? 1 : -1;
}
void stepDn(int i) {
  if (prev_step_pin[i]) {
    hw::move_steps(i, cur_dir[i]);
  }
  prev_step_pin[i] = false;
}
void stepUp(int i) {
  prev_step_pin[i] = true;
}
void stepEnabled(int i, bool en) {}

bool readLimit(int i) {
  return hw::get_limit(i);
}

int readEnc(int idx) {
  return hw::get_encoder(idx) + step_offs[idx];
}

void writeEnc(int idx, int value) {
  step_offs[idx] = value - hw::get_encoder(idx);
  LOG_DEBUG("Wrote %d to encoder %d", value, idx);
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

std::thread t;
void startMainTimer(int hz, void(*cb)()) {
  t = std::thread(runSteps, hz, cb);
};

} // namespace hal


void setup();
void loop();
int main() {
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);
  hw::init();
  for (int i = 0; i < NUM_J; i++) {
    cur_dir[i] = 1;
    step_offs[i] = 0;
    prev_step_pin[i] = true;
  }
  setup();
  uint64_t loop_time = 0;
  while (1) {
    hw::loop(); // Hidden hardware emulation; includes setting millis() to sim clock
    while (loop_time < millis()) {
      loop();
      loop_time += SIMULATED_LOOP_TIME_MILLIS;
    }
  }
}
