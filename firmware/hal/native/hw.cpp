#include "hw.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>
#include <chrono>

#define PUB_PD_MILLIS 100
uint64_t sim_millis = 0;
uint64_t millis() {
  return sim_millis;
}

// No need to delay in native environment (non-realtime OS, makes us miss our tick freq deadline)
void delayMicroseconds(uint16_t us) {}

namespace hw {

bool limit[NUM_J];
int32_t steps[NUM_J];
int16_t encoder[NUM_J];

bool get_limit(int idx) { return limit[idx]; }
int get_encoder(int idx) { return encoder[idx]; }

void move_steps(int idx, int delta) { 
  steps[idx] += delta; 
}

// Single socket handles bidirectional syncing between simulation and firmware
zmq::context_t context (1);
zmq::socket_t sock = zmq::socket_t(context, ZMQ_REP);
#define SOCKET_ADDR "ipc:///tmp/shop_robotics_hw.ipc"

void init() {
  int linger = 0;
  zmq_setsockopt(sock, ZMQ_LINGER, &linger, sizeof(linger));
  LOG_DEBUG("HW: REP: %s", SOCKET_ADDR); 
  sock.connect(SOCKET_ADDR);
  for (int i = 0; i < NUM_J; i++) {
    steps[i] = 0;
    limit[i] = true;
  }
}

void loop() {
  zmq::message_t resp;
  sock.recv(resp);

  uint8_t* payload = ((uint8_t*)resp.data());
  sim_millis = *((uint64_t*)payload); // 8 bytes clock in milliseconds
  for (int i = 0; i < NUM_J; i++) {
    // 5th byte is all limit switches in a mask
    limit[i] = (bool) (*(payload+sizeof(uint64_t)) & (0x01 << i));

    // Remaining data is encoders
    encoder[i] = ((int32_t*)(payload+sizeof(uint64_t)+sizeof(uint8_t)))[i];
  }
  // LOG_DEBUG("HW packet: %lu %d %d", millis(), limit[0], encoder[0]);
  zmq::message_t msg(NUM_J * sizeof(uint32_t));
  for (int i = 0; i < NUM_J; i++) {
    ((uint32_t*)msg.data())[i] = steps[i];
  }
  sock.send(msg);  
}

} // namespace hw
