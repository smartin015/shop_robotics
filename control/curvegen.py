from pathlib import Path
from sys import path

# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute() / 'ruckig/build'
print(build_path)
path.insert(0, str(build_path))

from ruckig import InputParameter, Ruckig, Trajectory, Result
NUM_J = 6
PI = 3.14159265
epsilon = 0.00000001

def print_trajectory(trajectory):
    print(f'Trajectory duration: {trajectory.duration:0.4f} [s]')
    print(f'Position extremas are {trajectory.position_extrema}')
    for pi, prof in enumerate(trajectory.profiles):
      print(f"===== PROFILE {pi} =====")
      for j, p in enumerate(prof):
        print(f"Joint {j}")
        for i in range(len(p.t)):
          if p.t[i] < epsilon:
            continue
          print(f"@T{p.t_sum[i] - p.t[i]:.2f}, go {p.t[i]:.2f}s: {p.j[i]:6.2f}j {p.a[i]:6.2f}a {p.v[i]:6.2f}v {p.p[i]:6.2f}p")

NSAMP = 128
TNORM = 1.0
MAX_V = 1
MIN_V = -1
MAX_AT = 3
MIN_AT = -3 #MIN_V / NSAMP
MAX_A0 = MAX_AT
MIN_A0 = 0

NUM_A0 = 31 
NUM_AT = 2*NUM_A0+1

def match_curve(A0, J, T, stats=True):
    # Returns a timescale (k) and the index of the baked curve with the closest match to 
    # a curve with v0=0, A0=A0, J=J, spanning t=0...T
    k = T/TNORM

    # Invert the change brought about by scaling the velocity curve in time.
    # v(kt) = 0.5*j*(kt)^2 + A0*(kt)
    # a(kt) = (k^2)*j*t + k*A0
    # j(kt) = (k^2)*j
    # 
    # j_k = (k^2)*j
    # A0_k = k*A0
    A0_orig = A0
    J_orig = J
    J = J*(k**2)
    A0 = A0*k

    # Compute ending instantaneous acceleration
    # A(T) = J*T + A0
    # Note that T_k = T/k = T/(T/TNORM) = TNORM
    AT = J*TNORM + A0
    
    # Now discretize the two acceleration values to find the correct curve to use.
    # Note: only using a sign for scale factor right now, but it's possible we could
    # adjust the Y-axis scale of the v-curve just a tiny bit to reduce the overall error from discretizing acceleration values.`
    scale = 1
    if A0 < 0:
      A0 = -A0
      AT = -AT
      scale = -1
    A0_i = round(NUM_A0 * ((A0-MIN_A0) / (MAX_A0-MIN_A0)))
    AT_i = round(NUM_AT * ((AT-MIN_AT) / (MAX_AT-MIN_AT)))-1
    curve_id = AT_i*NUM_A0 + A0_i

    if stats:
      A0e = abs(A0 - (A0_i/NUM_A0*(MAX_A0-MIN_A0) + MIN_A0))
      ATe = abs(AT - (AT_i/NUM_AT*(MAX_AT-MIN_AT) + MIN_AT))
      Je = (ATe - A0e)/T
      pos_error = (1/2)*Je*(T**2) + A0e*T
      print(f"Initial conditions: A0={A0_orig:0.2f}, J={J_orig:0.2f}, T={T:0.2f}")
      print(f"Modified: A0'={A0:0.2f}, J'={J:0.2f}, k={k:0.2f}")
      print(f"Derived: AT={AT:0.2f}, A0_i={A0_i:0.2f}, AT_i={AT_i:0.2f}")
      print(f"curve_id {curve_id}, scale {scale}, quant error {pos_error}")
    return (curve_id, scale)
    

def convert_profile(prof):
    # Return a NUM_J-element list of [[j0c1, j0c2...], [j1c1, j1c2...], ..., [jNc1, j1c2...]]
    # where each j#c# = (curve_id, vel_shift, usec)
    result = [[]] * NUM_J
    for joint_num, p in enumerate(prof):
      if joint_num != 0:
        continue # TODO RM
      for i in range(len(p.t)):
        if p.t[i] < epsilon:
          continue # Ignore zero-commands

        curve_id, vel_scale = match_curve(p.a[i], p.j[i], p.t[i])
        vel_shift = p.v[i]
        usec = int(p.t[i] * 1000000) # originally in fractional seconds
        result[joint_num].append((curve_id, vel_scale, vel_shift, usec))
    return result

if __name__ == '__main__':
    inp = InputParameter(NUM_J)
    inp.current_position = [0.0] * NUM_J
    inp.current_velocity = [0.0] * NUM_J
    inp.current_acceleration = [0.0] * NUM_J

    inp.target_position = [PI] * NUM_J
    inp.target_velocity = [0.0] * NUM_J
    inp.target_acceleration = [0.0] * NUM_J

    inp.min_velocity = [-1.0] * NUM_J
    inp.max_velocity = [1.0] * NUM_J
    inp.max_acceleration = [3.0] * NUM_J
    inp.min_acceleration = [-3.0] * NUM_J
    inp.max_jerk = [0.5] * NUM_J

    otg = Ruckig(NUM_J)
    trajectory = Trajectory(NUM_J)
    result = otg.calculate(inp, trajectory)
    if result == Result.ErrorInvalidInput:
        raise Exception('Invalid input!')
    print_trajectory(trajectory)
    intents = convert_profile(trajectory.profiles[0])
    print("Joint 0 intents:")
    for curve in intents[0]:
        print(str(curve) + ",")

    # print(match_curve(A0=0, J=0, T=1))
