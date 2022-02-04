import os
import glob
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory
import sys
import signal

def sigterm_handler(_signo, _stack_frame):
    print("SIGTERM received, exiting")
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

class Broker(Node):
    def __init__(self):
        super().__init__('l2_ar3')
        self.declare_parameter("stub", False)
        self.declare_parameter("step_url", "tcp://localhost:5556")
        self.declare_parameter("limit_url", "tcp://localhost:5557")
        self.declare_parameter("num_j", 6)
        self.num_joints = self.get_parameter('num_j').get_parameter_value().integer_value
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.stbc = StaticTransformBroadcaster(self)
        self.tbc = TransformBroadcaster
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.command_sub = self.create_subscription(JointTrajectory, "joint_trajectory_command", self.handle_joint_trajectory, 
                                                    qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Init complete!")

    def joint_state_callback(self):
        pos = [s.getValue() for s in self.sensors]
        now = self.get_clock().now()
        dt = float((now - self.last_joint_state_ts).nanoseconds) / (1000000000)
        if dt < 0.00001:
            vel = [0 for n in self.joint_names]
        else:
            vel=[(a-b)/dt for (a,b) in zip(pos, self.last_joint_state)]

        self.joint_state_pub.publish(JointState(
            header=Header(stamp=now.to_msg()),
            name=self.joint_names,
            position=pos,
            velocity=vel,
            effort=[0 for n in self.joint_names],
        ))
        self.last_joint_state_ts = now
        self.last_joint_state = pos

def main(args=None):
    rclpy.init(args=args)
    server = AR3()
    rclpy.spin(server, executor=server.executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
