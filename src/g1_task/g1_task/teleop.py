import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl
import rclpy
from ament_index_python.packages import get_package_share_directory
from kdl_parser_py.urdf import treeFromFile
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class TeleopNode(Node):
    def __init__(self):
        super().__init__('g1_teleop')

        # Parameters
        self.declare_parameter('reference_frame', 'torso_link')
        self.declare_parameter('velocity_scale', 0.3)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('damping_factor', 0.05)
        self.declare_parameter('max_joint_velocity', 1.5)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.update_rate = self.get_parameter('update_rate').value
        self.damping_factor = self.get_parameter('damping_factor').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value

        # Workspace bounds
        self.x_bounds = (0.2, 0.4)
        self.y_left_bounds = (0.0, 0.3)
        self.y_right_bounds = (-0.3, 0.0)
        self.z_bounds = (0.1, 0.5)

        # Initialize KDL kinematics
        self._setup_kdl()

        # Joint state tracking
        self.current_joint_states = {}
        self.joint_state_received = False
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, qos)

        # Joystick input and commanded velocities for debugging
        self.last_joy = None
        self.left_cart_vel = [0.0, 0.0, 0.0]
        self.right_cart_vel = [0.0, 0.0, 0.0]
        self.reset_requested = False
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)

        # Markers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/ik_target_markers', 10)

        # Velocity command publisher
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm_velocity_controller/commands', 10)

        # Velocity control timer
        self.create_timer(1.0 / self.update_rate, self._velocity_update)

        # Marker update timer
        self.create_timer(0.1, self._publish_markers)

        self.get_logger().info(f'Teleop ready (true velocity control at {self.update_rate}Hz)')

    def _setup_kdl(self):
        """Initialize KDL kinematics"""
        self.get_logger().info('Initializing KDL kinematics...')
        try:
            pkg_share = get_package_share_directory('g1_description')
            urdf_path = f"{pkg_share}/urdf/g1.urdf"

            ok, self.kdl_tree = treeFromFile(urdf_path)
            if not ok:
                raise RuntimeError(f"Failed to parse URDF: {urdf_path}")

            # Define arm joints
            left_joints = [
                'left_shoulder_pitch_joint',
                'left_shoulder_roll_joint',
                'left_shoulder_yaw_joint',
                'left_elbow_joint',
                'left_wrist_roll_joint',
            ]
            right_joints = [
                'right_shoulder_pitch_joint',
                'right_shoulder_roll_joint',
                'right_shoulder_yaw_joint',
                'right_elbow_joint',
                'right_wrist_roll_joint',
            ]

            self.arms = {
                'left': {
                    'base_link': self.reference_frame,
                    'tip_link': 'left_wrist_roll_rubber_hand',
                    'joint_names': left_joints,
                },
                'right': {
                    'base_link': self.reference_frame,
                    'tip_link': 'right_wrist_roll_rubber_hand',
                    'joint_names': right_joints,
                },
            }

            # Create chains and solvers for both arms
            for arm_name, arm_config in self.arms.items():
                chain = self.kdl_tree.getChain(arm_config['base_link'], arm_config['tip_link'])
                arm_config['chain'] = chain
                arm_config['fk_solver'] = kdl.ChainFkSolverPos_recursive(chain)
                arm_config['jac_solver'] = kdl.ChainJntToJacSolver(chain)

            # Parse joint limits
            self._parse_joint_limits(urdf_path)
            self.get_logger().info('KDL kinematics initialized')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize kinematics: {e}')
            raise

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

            arm_config['joint_limits'] = np.array(joint_limits)

    def _joint_state_callback(self, msg: JointState):
        """Track current joint positions from /joint_states."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_states[name] = pos
        self.joint_state_received = True

    def _joy_callback(self, msg: Joy):
        """Receive joystick input."""
        self.last_joy = msg

        # Square button (button 3) triggers reset to home
        if msg.buttons[3] == 1:
            if not self.reset_requested:
                self.reset_requested = True
                self.get_logger().info('Reset to home requested - sending all joints to zero')

    def _get_arm_joint_positions(self, arm_name: str) -> np.ndarray:
        """Get current joint positions for specified arm."""
        arm_config = self.arms[arm_name]
        positions = []
        for joint_name in arm_config['joint_names']:
            positions.append(self.current_joint_states.get(joint_name, 0.0))
        return np.array(positions)

    def _cartesian_to_joint_velocity(self, arm_name: str, cart_vel: list) -> np.ndarray:
        """
        Map Cartesian velocity to joint velocity using Jacobian

        Args:
            arm_name: 'left' or 'right'
            cart_vel: [vx, vy, vz] Cartesian velocity in torso frame (m/s)

        Returns:
            Joint velocities in rad/s
        """
        arm_config = self.arms[arm_name]
        n_joints = arm_config['chain'].getNrOfJoints()

        # Get current joint positions
        current_joints = self._get_arm_joint_positions(arm_name)

        # Build KDL JntArray
        q = kdl.JntArray(n_joints)
        for i, pos in enumerate(current_joints[:n_joints]):
            q[i] = pos

        # Compute Jacobian at current configuration
        jac = kdl.Jacobian(n_joints)
        arm_config['jac_solver'].JntToJac(q, jac)

        # Convert to NumPy (use only linear part, rows 0-2)
        J = np.array([[jac[i, j] for j in range(n_joints)] for i in range(3)])

        # Damped least squares pseudoinverse
        JJt = J @ J.T
        inv_term = np.linalg.inv(JJt + self.damping_factor**2 * np.eye(3))
        q_dot = J.T @ inv_term @ np.array(cart_vel)

        return q_dot

    def _clamp_to_limits(self, arm_name: str, joint_positions: np.ndarray) -> np.ndarray:
        """Clamp joint positions to limits defined in URDF."""
        limits = self.arms[arm_name]['joint_limits']
        return np.clip(joint_positions, limits[:, 0], limits[:, 1])

    def _velocity_update(self):
        """Velocity control loop at configured update rate (default 50Hz)."""
        if self.last_joy is None or not self.joint_state_received:
            return

        # Handle reset to home
        if self.reset_requested:
            self._reset_to_home()
            return

        # Compute Cartesian velocities from joystick
        joy = self.last_joy

        # Both arms: triggers (axes 6, 7)
        l2_trigger = joy.axes[6]  # L2 trigger (0.0-1.0)
        r2_trigger = joy.axes[7]  # R2 trigger (0.0-1.0)
        z_velocity = (r2_trigger - l2_trigger) * self.velocity_scale  # R2=up, L2=down

        # Left arm: left stick (axes 0, 1)
        self.left_cart_vel = [
            joy.axes[1] * self.velocity_scale,
            -joy.axes[0] * self.velocity_scale,
            z_velocity,
        ]

        # Right arm: right stick (axes 2, 3)
        self.right_cart_vel = [
            joy.axes[3] * self.velocity_scale,
            -joy.axes[2] * self.velocity_scale,
            z_velocity,
        ]

        # Compute joint velocities using Jacobian
        left_q_dot = self._cartesian_to_joint_velocity('left', self.left_cart_vel)
        right_q_dot = self._cartesian_to_joint_velocity('right', self.right_cart_vel)

        # Clamp joint velocities to limits (prevent singularity explosions)
        left_q_dot = np.clip(left_q_dot, -self.max_joint_velocity, self.max_joint_velocity)
        right_q_dot = np.clip(right_q_dot, -self.max_joint_velocity, self.max_joint_velocity)

        # Combine velocities and publish
        msg = Float64MultiArray()
        msg.data = list(left_q_dot) + list(right_q_dot)
        self.vel_pub.publish(msg)

    def _reset_to_home(self):
        """Reset all joints to zero (home position)."""
        # Get current joint positions
        left_current = np.array([self.current_joint_states.get(j, 0.0) for j in self.arms['left']['joint_names']])
        right_current = np.array([self.current_joint_states.get(j, 0.0) for j in self.arms['right']['joint_names']])

        # Target is zero for all joints
        target = np.zeros(len(left_current) + len(right_current))
        current = np.concatenate([left_current, right_current])

        # Compute error
        error = target - current
        max_error = np.max(np.abs(error))

        if max_error < 0.01:  # Within 0.01 radians of home
            # Done resetting
            self.reset_requested = False
            self.get_logger().info('Reset complete')
            # Send zero velocities
            msg = Float64MultiArray()
            msg.data = [0.0] * 10
            self.vel_pub.publish(msg)
        else:
            # Proportional control towards home (gentle motion)
            velocity = np.clip(error * 2.0, -0.5, 0.5)  # P-controller with low gain
            msg = Float64MultiArray()
            msg.data = list(velocity)
            self.vel_pub.publish(msg)

    def _publish_markers(self):
        """
        Publish visualization markers showing commanded Cartesian velocities.

        Debug visualization: arrows at fixed positions showing velocity commands.
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Fixed positions in front of robot (pelvis frame)
        # Left marker at (0.3, 0.15, 0.3) and right at (0.3, -0.15, 0.3)
        for arm_name, cart_vel, base_pos, color, marker_id in [
            ('left', self.left_cart_vel, (0.3, 0.15, 0.3), ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9), 0),
            ('right', self.right_cart_vel, (0.3, -0.15, 0.3), ColorRGBA(r=1.0, g=0.0, b=0.5, a=0.9), 1),
        ]:
            # Create arrow marker showing velocity direction and magnitude
            marker = Marker()
            marker.header.frame_id = self.reference_frame
            marker.header.stamp = now
            marker.ns = 'velocity_command'
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Arrow starts at fixed base position
            marker.pose.position.x = base_pos[0]
            marker.pose.position.y = base_pos[1]
            marker.pose.position.z = base_pos[2]

            # Arrow points in velocity direction
            vel_mag = np.linalg.norm(cart_vel)
            if vel_mag > 0.001:  # Show arrow only if velocity is significant
                # Normalize velocity to get direction
                vel_dir = np.array(cart_vel) / vel_mag

                # Convert velocity vector to quaternion
                # Arrow points along +X axis by default, we need to rotate it to point along velocity
                # Use rotation that aligns (1,0,0) with vel_dir
                import math

                # If velocity is near zero, just point forward
                if abs(vel_dir[0]) < 0.99:
                    # Cross product to get rotation axis
                    axis = np.cross([1, 0, 0], vel_dir)
                    axis_norm = np.linalg.norm(axis)
                    if axis_norm > 0.001:
                        axis = axis / axis_norm
                        angle = math.acos(np.clip(vel_dir[0], -1, 1))

                        # Convert axis-angle to quaternion
                        s = math.sin(angle / 2)
                        marker.pose.orientation.x = axis[0] * s
                        marker.pose.orientation.y = axis[1] * s
                        marker.pose.orientation.z = axis[2] * s
                        marker.pose.orientation.w = math.cos(angle / 2)
                    else:
                        marker.pose.orientation.w = 1.0
                else:
                    # Velocity is along +X or -X
                    if vel_dir[0] > 0:
                        marker.pose.orientation.w = 1.0  # No rotation
                    else:
                        marker.pose.orientation.z = 1.0  # 180 degree rotation around Z

                # Scale arrow length by velocity magnitude (scale up for visibility)
                marker.scale.x = vel_mag * 2.0  # Length proportional to velocity
                marker.scale.y = 0.02  # Shaft diameter
                marker.scale.z = 0.02  # Head diameter
            else:
                # Show small sphere if no velocity
                marker.type = Marker.SPHERE
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.pose.orientation.w = 1.0

            marker.color = color
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

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
