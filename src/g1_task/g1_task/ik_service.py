import xml.etree.ElementTree as ET

import PyKDL as kdl
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from kdl_parser_py.urdf import treeFromFile
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from g1_interfaces.srv import ComputeTrajectory


class IKService(Node):
    def __init__(self):
        super().__init__('g1_ik_service')

        self.declare_parameter('trajectory_duration', 3.0)
        self.declare_parameter('num_waypoints', 10)
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.num_waypoints = self.get_parameter('num_waypoints').value

        pkg_share = get_package_share_directory('g1_description')
        urdf_path = f"{pkg_share}/urdf/g1.urdf"

        self.get_logger().info('Initializing KDL kinematics...')
        try:
            ok, self.kdl_tree = treeFromFile(urdf_path)
            if not ok:
                raise RuntimeError(f"Failed to parse URDF: {urdf_path}")

            # Arm chains: waist -> wrist
            left_joints = [
                'waist_yaw_joint',
                'left_shoulder_pitch_joint',
                'left_shoulder_roll_joint',
                'left_shoulder_yaw_joint',
                'left_elbow_joint',
                'left_wrist_roll_joint',
            ]
            right_joints = [
                'waist_yaw_joint',
                'right_shoulder_pitch_joint',
                'right_shoulder_roll_joint',
                'right_shoulder_yaw_joint',
                'right_elbow_joint',
                'right_wrist_roll_joint',
            ]

            self.arms = {
                'left': {
                    'base_link': 'pelvis',
                    'tip_link': 'left_wrist_roll_rubber_hand',
                    'joint_names': left_joints,
                },
                'right': {
                    'base_link': 'pelvis',
                    'tip_link': 'right_wrist_roll_rubber_hand',
                    'joint_names': right_joints,
                },
            }

            for _, arm_config in self.arms.items():
                chain = self.kdl_tree.getChain(arm_config['base_link'], arm_config['tip_link'])
                arm_config['chain'] = chain
                arm_config['fk_solver'] = kdl.ChainFkSolverPos_recursive(chain)
                arm_config['ik_solver'] = kdl.ChainIkSolverPos_LMA(chain)

            self._parse_joint_limits(urdf_path)
            self.get_logger().info('KDL kinematics initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize kinematics: {e}')
            raise

        self.current_joint_states = {}
        self.joint_state_received = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos)
        self.ik_service = self.create_service(ComputeTrajectory, 'compute_trajectory', self.compute_ik_callback)
        self.get_logger().info('IK Service ready')

    def _parse_joint_limits(self, urdf_path: str):
        """Parse joint limits from URDF XML."""
        tree = ET.parse(urdf_path)
        joints_by_name = {j.attrib['name']: j for j in tree.getroot().findall('joint')}

        for _, arm_config in self.arms.items():
            joint_limits = []
            for joint_name in arm_config['joint_names']:
                joint_elem = joints_by_name.get(joint_name)
                if joint_elem is None:
                    raise ValueError(f"Joint {joint_name} not found in URDF")

                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    lower = float(limit_elem.attrib.get('lower', '-3.14'))
                    upper = float(limit_elem.attrib.get('upper', '3.14'))
                    joint_limits.append((lower, upper))
                else:
                    joint_limits.append((-3.14, 3.14))

            arm_config['joint_limits'] = joint_limits

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_states[name] = position
        self.joint_state_received = True

    def compute_ik_callback(
        self, request: ComputeTrajectory.Request, response: ComputeTrajectory.Response
    ) -> ComputeTrajectory.Response:
        arm = request.arm_name
        pos = request.goal.pose.position
        self.get_logger().info(f'IK request for {arm} arm -> [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')

        if arm not in self.arms:
            self.get_logger().error(f'Invalid arm name: {arm}. Must be "left" or "right"')
            response.success = False
            return response

        if not self.joint_state_received:
            self.get_logger().warn('No joint states received yet')
            response.success = False
            return response

        arm_config = self.arms[arm]

        # Get current joint positions as seed for IK solver
        try:
            seed_joints = [self.current_joint_states.get(j, 0.0) for j in arm_config['joint_names']]
        except KeyError as e:
            self.get_logger().error(f'Missing joint state: {e}')
            response.success = False
            return response

        target_frame = self._pose_to_kdl_frame(request.goal)

        rpy = target_frame.M.GetRPY()
        self.get_logger().debug(
            f"Target: pos=[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}] "
            f"rpy=[{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}] "
            f"frame={request.goal.header.frame_id}"
        )

        solution = self._solve_ik(arm_config, target_frame, seed_joints)

        if solution is None:
            self.get_logger().warn(f'IK failed for {arm} arm')
            response.success = False
            return response

        fk_result = self._solve_fk(arm_config, solution)
        error = (target_frame.p - fk_result.p).Norm()
        self.get_logger().info(f'IK solved for {arm} arm (FK error: {error:.4f} m)')
        self.get_logger().debug(
            f"FK verify: target=[{target_frame.p.x():.4f}, {target_frame.p.y():.4f}, {target_frame.p.z():.4f}] "
            f"actual=[{fk_result.p.x():.4f}, {fk_result.p.y():.4f}, {fk_result.p.z():.4f}]"
        )

        response.success = True
        response.trajectory = self._generate_trajectory(
            arm_config['joint_names'],
            seed_joints,
            solution,
            self.trajectory_duration,
            self.num_waypoints,
        )
        return response

    def _solve_ik(self, arm_config: dict, target_pose: kdl.Frame, seed_joints: list) -> list:
        n_joints = arm_config['chain'].getNrOfJoints()

        q_init = kdl.JntArray(n_joints)
        for i, val in enumerate(seed_joints[:n_joints]):
            q_init[i] = val

        q_out = kdl.JntArray(n_joints)
        result = arm_config['ik_solver'].CartToJnt(q_init, target_pose, q_out)

        if result < 0:
            current_frame = self._solve_fk(arm_config, seed_joints)
            distance = (target_pose.p - current_frame.p).Norm()
            self.get_logger().debug(f"IK solver failed (code={result}, distance={distance:.4f} m)")
            return None

        solution = [q_out[i] for i in range(n_joints)]

        if not self._check_joint_limits(arm_config, solution):
            self.get_logger().debug("IK solution violates joint limits")
            return None

        self.get_logger().debug(f"IK solution: {dict(zip(arm_config['joint_names'], [f'{s:.3f}' for s in solution]))}")
        return solution

    def _solve_fk(self, arm_config: dict, joint_positions: list) -> kdl.Frame:
        n_joints = arm_config['chain'].getNrOfJoints()
        q = kdl.JntArray(n_joints)
        for i, val in enumerate(joint_positions[:n_joints]):
            q[i] = val
        frame = kdl.Frame()
        arm_config['fk_solver'].JntToCart(q, frame)
        return frame

    def _check_joint_limits(self, arm_config: dict, joint_positions: list) -> bool:
        return all(lower <= pos <= upper for pos, (lower, upper) in zip(joint_positions, arm_config['joint_limits']))

    def _pose_to_kdl_frame(self, pose_stamped) -> kdl.Frame:
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        return kdl.Frame(
            kdl.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w),
            kdl.Vector(pos.x, pos.y, pos.z),
        )

    def _generate_trajectory(
        self, joint_names: list, start_positions: list, goal_positions: list, duration: float, num_points: int
    ) -> JointTrajectory:
        """Generate smooth trajectory with cubic interpolation (3t^2 - 2t^3)."""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        for i in range(num_points + 1):
            t = i / num_points
            blend = 3 * t**2 - 2 * t**3

            point = JointTrajectoryPoint()
            point.positions = [s + (g - s) * blend for s, g in zip(start_positions, goal_positions)]

            if i == 0 or i == num_points:
                point.velocities = [0.0] * len(joint_names)
            else:
                # Derivative: 6t - 6t^2
                blend_vel = (6 * t - 6 * t**2) / duration
                point.velocities = [(g - s) * blend_vel for s, g in zip(start_positions, goal_positions)]

            time_sec = duration * t
            point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            trajectory.points.append(point)

        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = IKService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
