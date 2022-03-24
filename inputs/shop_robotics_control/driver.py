import os
import glob
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys
import signal

def sigterm_handler(_signo, _stack_frame):
    print("SIGTERM received, exiting")
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

class Driver(Node):
    PUBLISH_PD = 5  # seconds
    

    def __init__(self):
        super().__init__('Driver')
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.command_sub = self.create_subscription(JointState, "joint_states", self.handle_joint_state,
                                                    qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Init complete!")

    def handle_joint_state(self, msg):
        self.get_logger().info("joint_states subscription in shop_robotics_control/input received the following pos data: %s" % msg.position)
        pass

def main(args=None):
    rclpy.init(args=args)
    server = Driver()
    rclpy.spin(server, executor=server.executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
