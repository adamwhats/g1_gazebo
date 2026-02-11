"""Task sequencer: random reach targets, alternating arms, idle arm returns home."""

import math
import random

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from g1_interfaces.srv import ComputeTrajectory

# Arm joints (without waist_yaw, which the IK trajectory already includes)
ARM_JOINTS = {
    'left': [
        'left_shoulder_pitch_joint',
        'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint',
        'left_elbow_joint',
        'left_wrist_roll_joint',
    ],
    'right': [
        'right_shoulder_pitch_joint',
        'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint',
        'right_elbow_joint',
        'right_wrist_roll_joint',
    ],
}

MAX_IK_RETRIES = 20


class TaskSequencer(Node):
    """Simple task sequencer that randomly reaches for poses."""

    def __init__(self):
        super().__init__('g1_task_sequencer')

        self.ik_client = self.create_client(ComputeTrajectory, 'compute_trajectory')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.current_hand = 'left'
        self.is_busy = False
        self.ik_retries = 0

        self.current_joint_states = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service()
        self.get_logger().info('Waiting for arm_controller action server...')
        self.traj_client.wait_for_server()

        self.create_timer(0.5, self.main_loop)
        self.get_logger().info('Task sequencer ready')

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_states[name] = pos

    def generate_random_pose(self, arm: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'pelvis'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = random.uniform(0.2, 0.4)
        if arm == 'left':
            pose.pose.position.y = random.uniform(0.0, 0.3)
        else:
            pose.pose.position.y = random.uniform(-0.3, 0.0)
        pose.pose.position.z = random.uniform(0.1, 0.5)

        pitch = random.uniform(-0.2, 0.2)
        yaw = random.uniform(-0.2, 0.2)
        # Quaternion from RPY (roll=0)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        pose.pose.orientation.x = sp * cy
        pose.pose.orientation.y = cp * sy
        pose.pose.orientation.z = -sp * sy
        pose.pose.orientation.w = cp * cy
        return pose

    def main_loop(self):
        if self.is_busy:
            return

        self.is_busy = True
        self.ik_retries = 0
        self._try_ik()

    def _try_ik(self):
        arm = self.current_hand
        target_pose = self.generate_random_pose(arm)

        self.get_logger().info(
            f'Reaching with {arm} hand to [{target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f}, {target_pose.pose.position.z:.2f}]'
        )

        request = ComputeTrajectory.Request()
        request.arm_name = arm
        request.goal = target_pose

        future = self.ik_client.call_async(request)
        future.add_done_callback(lambda f: self._ik_response_cb(f, arm))

    def _ik_response_cb(self, future, arm: str):
        try:
            response = future.result()

            if not response.success:
                self.ik_retries += 1
                if self.ik_retries < MAX_IK_RETRIES:
                    self.get_logger().info(f'IK failed for {arm} arm, retrying ({self.ik_retries}/{MAX_IK_RETRIES})')
                    self._try_ik()
                else:
                    self.get_logger().warn(f'IK failed after {MAX_IK_RETRIES} attempts, skipping')
                    self.current_hand = 'right' if arm == 'left' else 'left'
                    self.is_busy = False
                return

            self.get_logger().info(f'IK succeeded for {arm} arm, executing trajectory')

            trajectory = response.trajectory
            other_arm = 'right' if arm == 'left' else 'left'
            other_joints = ARM_JOINTS[other_arm]
            other_current = [self.current_joint_states.get(j, 0.0) for j in other_joints]

            # Smoothly return idle arm to neutral position
            neutral = [0.0] * len(other_joints)
            trajectory.joint_names.extend(other_joints)
            num_points = len(trajectory.points)
            for i, point in enumerate(trajectory.points):
                t = i / max(num_points - 1, 1)
                blend = 3 * t**2 - 2 * t**3
                point.positions.extend([s + (g - s) * blend for s, g in zip(other_current, neutral)])
                point.velocities.extend([0.0] * len(other_joints))

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

            send_future = self.traj_client.send_goal_async(goal)
            send_future.add_done_callback(lambda f: self._traj_response_cb(f, arm))

        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
            self.is_busy = False

    def _traj_response_cb(self, future, arm: str):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f'Trajectory rejected for {arm} arm')
                self.is_busy = False
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._traj_result_cb(f, arm))

        except Exception as e:
            self.get_logger().error(f'Trajectory action failed: {e}')
            self.is_busy = False

    def _traj_result_cb(self, future, arm: str):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Trajectory succeeded for {arm} arm')
        else:
            self.get_logger().warn(f'Trajectory failed for {arm} arm (status={result.status})')

        self.current_hand = 'right' if arm == 'left' else 'left'
        self.is_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskSequencer()

    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
