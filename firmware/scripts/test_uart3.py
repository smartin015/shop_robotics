import serial
import threading
import time
import struct

s = serial.Serial("/dev/ttyAMA1", 115200)
PACKET_START_BYTE = 0xaa
NUM_J = 6
data_struct_fmt = "<BB" + "B"*NUM_J + "i"*NUM_J + "h"*NUM_J + "h"*NUM_J + "L"

def send(joint_num, curve_id, shift_y, scale_y, length_usec):
  msg = struct.pack("<BBBBhhI", PACKET_START_BYTE, 10, joint_num, curve_id, shift_y, scale_y, length_usec)
  s.write(msg)

queue_avail = 0
stopping = False
read = False
def read_forever():
    global queue_avail
    global stopping
    global read
    while True:
        stuff = s.read_until(bytes([PACKET_START_BYTE]))
        if stuff == b'':
            print("No serial data")
            continue
        sz = s.read(1)
        if len(sz) != 1:
            print("ERR serial could not read size after sync byte; runup:", stuff[-10:])
            continue
        sz = int(sz[0])
        if sz != 64:
            continue
        packet = s.read(sz)
        data = struct.unpack(data_struct_fmt, packet)
        queue_avail=data[2]
        stopping = data[1] != 0
        read=True
        print(data)

print("starting read loop")
threading.Thread(target=read_forever, daemon=True).start()

writeab = True
while True:
    time.sleep(0.1)
    if queue_avail > 3 and not stopping and read:
        if writeab:
            print("write 1")
            send(joint_num=0, curve_id=1, shift_y=0, scale_y=10 * 256, length_usec=1000*1003)
        else:
            print("write 2")
            send(joint_num=0, curve_id=2, shift_y=150, scale_y=10 * 256, length_usec=1000*1004)
        writeab = not writeab
        read = False
