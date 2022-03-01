#include "hw.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>
#include <chrono>

uint64_t target_micros = 0;
uint64_t sim_micros = 0;
uint64_t millis() {
  return sim_micros / 1000;
}
uint64_t micros() {
  return sim_micros;
}

namespace hw {

int16_t rate[NUM_J];
bool limit[NUM_J];
int64_t usteps[NUM_J];
int16_t encoder[NUM_J];

bool advance(uint16_t usec) { 
  if (sim_micros+usec > target_micros) {
    return false;
  }
  sim_micros += usec; 
  for (int j = 0; j < NUM_J; j++) {
    // uSteps = (step/sec) * (ustep/step) * (sec/usec) * (usec)  
    // = (step/sec) * (1000000) * (1/1000000) * (usec)
    usteps[j] += rate[j] * usec;
    
    //if (j == 0) {LOG_DEBUG("advance %d %d -> %d", rate[j], usec, get_steps(j));}
  }
  return true;
}
bool get_limit(uint8_t j) { 
  return limit[j]; 
}
int32_t get_steps(uint8_t j) { 
  return usteps[j] / 1000000; 
}
int16_t get_encoder(uint8_t j) { 
  return encoder[j]; 
}

void set_rate(uint8_t j, int16_t r) { 
  rate[j] = r; 
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
    rate[i] = 0;
    usteps[i] = 0;
    limit[i] = true;
  }
}

void loop() {
  zmq::message_t resp;
  sock.recv(resp);

  uint8_t* payload = ((uint8_t*)resp.data());
  uint8_t* ptr = payload;
  target_micros = *((uint64_t*)ptr); // 64-bit clock in microseconds
  ptr += sizeof(uint64_t);

  // next byte is all limit switches in a mask
  uint8_t limit_mask = *ptr;  
  ptr += sizeof(uint8_t);
  for (int i = 0; i < NUM_J; i++) {
    limit[i] = (bool) limit_mask & (0x01 << i);
  }
  
  // Remaining data is encoders
  for (int i = 0; i < NUM_J; i++) {
    encoder[i] = *((int16_t*)ptr);
    ptr += sizeof(int16_t);
  }
  // LOG_DEBUG("HW packet: %lu %d %d", millis(), limit[0], encoder[0]);

  // Now construct the outgoing message to push step counts to the simulator
  zmq::message_t msg(NUM_J * sizeof(int32_t));
  for (int i = 0; i < NUM_J; i++) {
    ((int32_t*)msg.data())[i] = get_steps(i);
  }
  sock.send(msg, zmq::send_flags::none);  
}

} // namespace hw
