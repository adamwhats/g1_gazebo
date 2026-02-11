import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryTest(Node):
    def __init__(self):
        super().__init__('joint_trajectory_test')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', qos)

        self.joint_names = [
            'waist_yaw_joint',
            'left_shoulder_yaw_joint',
            'right_shoulder_yaw_joint',
            'left_elbow_joint',
            'right_elbow_joint',
        ]

        # Amplitude (radians) and frequency (Hz) per joint
        self.amplitudes = [0.5, 0.4, 0.4, 0.6, 0.6]
        self.frequencies = [0.3, 0.5, 0.5, 0.4, 0.4]

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Joint trajectory test node started')

    def timer_callback(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        positions = [amp * math.sin(2.0 * math.pi * freq * t) for amp, freq in zip(self.amplitudes, self.frequencies)]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [point]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
