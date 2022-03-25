import math
from enum import Enum

NSAMP = 128

# Returns functions that yield the velocity and acceleration needed
# to satisfy a starting acceleration A0 and ending trajectory at time T with vel VT, acc AT
# such that the position at point T would be the same as if a linear velocity were followed (from 0 at time 0 to VT at time T)
# i.e. the average velocity is guaranteed to be VT/2
def gen_polynomial_accel_fn(A0, AT, VT, T=NSAMP):
  a = (AT/(T**2)) - (1/T**4)*(6*VT - 6*A0*(T**2)-AT*(T**2))
  b = (1/(T**3)) * (6*VT - 6*A0*(T**2) - AT*(T**2))
  c = A0
  accFn = lambda t: a*(t**2) + b*t + c
  velFn = lambda t: (a/3)*(t**3) + (b/2)*(t**2) + c*t
  return (accFn, velFn)

class CURVE(Enum):
  LIN_VEL_CONST_ACC = 1
  VEL_RAMP_DECEL_TO_ZERO = 2
  VEL_RAMP_ACCEL_FROM_ZERO = 3
  VEL_RAMP_DECEL_HALF = 4
  VEL_RAMP_ACCEL_DOUBLE = 5

CURVE_FNS = [
  lambda t: t, # Linear velocity increase, constant accel
  lambda t: round(-(1/NSAMP)*t*t + 2*t), # Velocity 0 to NSAMP
  lambda t: round((1/NSAMP)*t*t), # Velocity 0 to NSAMP
  lambda t: round(2*t - t*t*(1/(2*NSAMP))), # Velocity 0 to NSAMP
  lambda t: round(t + t*t*(1/(2*NSAMP))), # Velocity 0 to NSAMP
]
for fn in CURVE_FNS:
  assert(fn(0) == 0)

CURVE_ACCEL_FNS = [
  lambda t: 1,
  lambda t: 2 * (1 - t/NSAMP),  # Linear decel from 2*S to 0
  lambda t: 2 * t/NSAMP, # Linear accel from 0 to 2*S
  lambda t: 2 - t/NSAMP, # Linear decel from 2*S to S
  lambda t: 1 + t/NSAMP, # Linear accel from S to 2*S
]
CURVE_BOUNDS = [(vf(NSAMP), af(0), af(NSAMP)) for (vf, af) in zip(CURVE_FNS, CURVE_ACCEL_FNS)]

def gen_curve_data(dt, nsamp=NSAMP):
  result = [dt(s) for s in range(nsamp)]
  return str(result).replace("[", "{").replace("]", "}")

def make_curves_c_file(funcs=CURVE_FNS):
  return (
    "#include \"curves.h\"" +
    "\nconst int16_t CURVES[][CURVE_SZ] = {\n  {},\n  " +
    ",\n  ".join([gen_curve_data(f) for f in funcs]) +
    "\n};" 
  )

# Forms a path from one vel+accel to another vel+accel by composing one or more curves
class Curve:
  def __init__(self, joint_num, start_vel, start_acc, end_vel, end_acc, usec):
    self.joint_num = joint_num
    self.curve_id = curve_id
    self.start_vel = start_vel
    self.end_vel = end_vel
    self.start_acc = start_acc
    self.end_acc = end_acc
    self.usec = usec

    self.intents = [] # id, scale, shift
    if start_vel == end_vel:
      # TODO develop a curve which produces the same average velocity for the period but allows
      # Transitioning nicely at start and end
      self.intents.append((LIN_VEL_CONST_ACC, 0, start_vel))
      return

    #if start_acc == 0: # 
    #  self.intents.append((VEL_RAMP_ACCEL_FROM_ZERO)

    #if end_acc == 0:
    #  scale = (end_vel - start_vel) / CURVE_BOUNDS[VEL_RAMP_DECEL_TO_ZERO][0]
    #  shift = start_vel
    #  self.intents.append((VEL_RAMP_DECEL_TO_ZERO, scale, shift))

    # Add a linear portion if we can't match up the acelerations
    # self.intents.append((LIN_VEL_CONST_ACC, 1, 0)) 



  def pack(self):
    return struct.pack("<BBhhI", joint_num, curve_id, shift_y, scale_y, length_usec)


if __name__ == "__main__":
  # Note: curve values can be negated via scale_y
  # print(make_curves_c_file())

  import matplotlib.pyplot as plt
  acc, vel = gen_polynomial_accel_fn(A0=0, AT=0, VT=100)
  plt.subplot(211)
  plt.plot(range(NSAMP), [vel(t) for t in range(NSAMP)])
  plt.subplot(212)
  plt.plot(range(NSAMP), [acc(t) for t in range(NSAMP)])
  plt.show()

  


