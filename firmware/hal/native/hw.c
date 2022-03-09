#include "hw.h"
#include "comms.h"
#include "log.h"
#include <unistd.h>
#include <czmq.h>
#include <errno.h>

#define ZMQ_PULL_ADDR "tcp://0.0.0.0:5559"
#define ZMQ_PUSH_ADDR "tcp://0.0.0.0:5558"
#define ZMQ_REP_ADDR "ipc:///tmp/shop_robotics_hw.ipc"

uint32_t target_micros = 0;
uint32_t sim_micros = 0;
uint32_t hw_micros() {
  return sim_micros;
}

int16_t rate[NUM_J];
uint8_t hw_limit;
int64_t usteps[NUM_J];
int16_t encoder[NUM_J];

bool hw_advance(uint16_t usec) { 
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
uint8_t hw_get_limits() { 
  return hw_limit; 
}
int32_t hw_get_steps(uint8_t j) { 
  return usteps[j] / 1000000; 
}
int16_t hw_get_encoder(uint8_t j) { 
  return encoder[j]; 
}

void hw_set_rate(uint8_t j, int16_t r) { 
  rate[j] = r; 
}

// Single socket handles bidirectional syncing between simulation and firmware
zsock_t *sock;

// Push and pull sockets to emulate UART communications
zsock_t *push;
zsock_t *pull;

void hw_init() {
  int linger = 0;
  push = zsock_new_push(ZMQ_PUSH_ADDR);
  zsock_set_linger(push, linger);
  pull = zsock_new_pull(ZMQ_PULL_ADDR);
  zsock_set_linger(pull, linger);
  sock = zsock_new_rep(ZMQ_REP_ADDR);
  assert(sock);
  zsock_set_linger(sock, linger);
  printf("Comms initialized (PUSH %s PULL %s REP %s)\n", ZMQ_PUSH_ADDR, ZMQ_PULL_ADDR, ZMQ_REP_ADDR);
  for (int i = 0; i < NUM_J; i++) {
    rate[i] = 0;
    usteps[i] = 0;
    hw_limit = 0;
  }
}

void check_comms() {
  void* lzsock = zsock_resolve(pull);
  assert(lzsock);
  zmq_pollitem_t items[] = {{lzsock, 0, ZMQ_POLLIN, 0}};
  int rc = zmq_poll(items, 1, 0);
  if (rc < 0) {
    printf("check_comms poll err %d\n", errno);
    assert(rc >= 0);
  }
  if (items [0].revents & ZMQ_POLLIN) {
    zframe_t *frame = zframe_recv(pull);
    on_message(zframe_data(frame), zframe_size(frame));
    zframe_destroy (&frame);
  }
}

void sync_hw() {
  zframe_t *resp = zframe_recv(sock);
  if (resp == NULL) {
    printf("HWLOOP RECV ERR %d\n", errno);
    assert(resp != NULL);
  }
  uint8_t* payload = ((uint8_t*)zframe_data(resp));
  uint8_t* ptr = payload;
  target_micros = *((uint32_t*)ptr); // clock in microseconds
  ptr += sizeof(uint32_t);

  // next byte is all limit switches in a mask
  uint8_t new_hw_limit = *ptr;  
  ptr += sizeof(uint8_t);
  if (new_hw_limit != hw_limit) {
    hw_limit = new_hw_limit;
    on_limits_changed();
  }

  // Remaining data is encoders
  for (int i = 0; i < NUM_J; i++) {
    encoder[i] = *((int16_t*)ptr);
    ptr += sizeof(int16_t);
  }
  // LOG_DEBUG("HW packet: %lu %d %d", millis(), limit[0], encoder[0]);
  zframe_destroy(&resp);

  // Now construct the outgoing message to push step counts to the simulator
  zframe_t *msg = zframe_new(NULL, NUM_J * sizeof(int32_t));
  for (int i = 0; i < NUM_J; i++) {
    ((int32_t*)zframe_data(msg))[i] = hw_get_steps(i);
  }
  int rc = zframe_send(&msg, sock, 0); //consumes msg
  if (rc != 0) {
    printf("HWLOOP SEND ERR %d\n", errno);
    assert(rc == 0);
  }

}

void hw_loop() {
  check_comms();
  sync_hw();
}

#define BUFLEN 128
uint8_t comms_tx_buf[BUFLEN];
uint8_t * comms_preWrite(uint8_t sz) {
  if (sz > BUFLEN) {
    printf("comms_preWrite_badsz\n");
    return NULL;
  }
  return comms_tx_buf;
}

void comms_flush(uint8_t sz) {
  zframe_t *frame = zframe_new(comms_tx_buf, sz);
  int rc = zframe_send(&frame, push, 0); // consumes frame
  if (rc != 0) {
    printf("comms_flush SEND ERR %d\n", errno);
    assert(rc == 0);
  } 
}
