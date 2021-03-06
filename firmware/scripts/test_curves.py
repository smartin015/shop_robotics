import struct
import time
import zmq
import threading
from functools import reduce
NUM_J = 6

ctx = zmq.Context()
# Follow default bind/connect behavior in http://czmq.zeromq.org/czmq4-0:zsock
push = ctx.socket(zmq.PUSH)
push.connect("tcp://0.0.0.0:5559")
pull = ctx.socket(zmq.PULL)
pull.bind("tcp://0.0.0.0:5558")
sim = ctx.socket(zmq.REQ)
sim.connect("ipc:///tmp/shop_robotics_hw.ipc")

def sameData(aa, bb):
  if aa is None or bb is None:
    return False
  return reduce(lambda x, y: x and y, [a == b for (a,b) in zip(aa[:-1], bb[:-1])])

data_struct_fmt = "<BB" + "B"*NUM_J + "i"*NUM_J + "h"*NUM_J + "h"*NUM_J + "Q"
queue_avail = 0
stopping = False
fresh = False
def read_forever():
  global queue_avail
  global stopping
  global fresh
  last = None
  while True:
    try:
      packet = pull.recv()
    except zmq.error.Again:
      continue
    data = struct.unpack(data_struct_fmt, packet);
    if not sameData(last, data):
      print("SER: ", data)
    last = data
    queue_avail = data[2]
    stopping = data[1] != 0
    fresh = True
print("Starting read loop")
threading.Thread(target=read_forever, daemon=True).start()

def send(joint_num, curve_id, shift_y, scale_y, length_usec):
  msg = struct.pack("<BBhhI", joint_num, curve_id, shift_y, scale_y, length_usec)
  push.send(msg)

print("Starting req/rep simulation loop")
limit_mask = 0
enc = [0]*NUM_J
micros = 0
REPORT_INTERVAL=60*1000
STEP_USEC=100
next_report_micros=0
writeab = False
while True:
  time.sleep(0.01)
  micros += STEP_USEC
  msg = struct.pack('<QB' + 'i'*NUM_J, micros, limit_mask, *enc)
  sim.send(msg)
  packet = sim.recv()
  if queue_avail > 3 and not stopping and fresh:
    fresh = False
    if writeab:
      print("write 1")
      send(joint_num=0, curve_id=1, shift_y=0, scale_y=10 * 256, length_usec=100*1000)
    else:
      print("write 2")
      send(joint_num=0, curve_id=2, shift_y=150, scale_y=10 * 256, length_usec=100*1000)
    writeab = not writeab
  if micros > next_report_micros:
    steps = struct.unpack("<" + "i"*NUM_J, packet)
    print("HW:", steps)
    next_report_micros += REPORT_INTERVAL 
