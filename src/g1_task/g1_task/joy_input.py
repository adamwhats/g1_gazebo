import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Evdev input event format
EVENT_FORMAT = 'llHHi'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# Event types
EV_KEY = 0x01
EV_ABS = 0x03

# Stick configuration (PS4: 0-255 range, center at 128)
STICK_CENTER = 128
STICK_MAX = 128

# Analog axes: evdev code -> axes index
# axes[0]: left_x, [1]: left_y, [2]: right_x, [3]: right_y, [4]: dpad_x, [5]: dpad_y, [6]: L2, [7]: R2
AXIS_MAP = {
    0: 0,  # ABS_X -> left stick X
    1: 1,  # ABS_Y -> left stick Y
    3: 2,  # ABS_RX -> right stick X
    4: 3,  # ABS_RY -> right stick Y
    16: 4,  # ABS_HAT0X -> D-pad X
    17: 5,  # ABS_HAT0Y -> D-pad Y
    2: 6,  # ABS_Z -> L2 trigger
    5: 7,  # ABS_RZ -> R2 trigger
}

# Button mapping: evdev key code -> buttons index (PS4 layout)
# buttons[0]: X, [1]: Circle, [2]: Triangle, [3]: Square, [4]: L1, [5]: R1,
# [6]: L2, [7]: R2, [8]: Share, [9]: Options, [10]: PS, [11]: L3, [12]: R3
BTN_CODES = {
    304: 0,  # BTN_SOUTH (BTN_A) -> X (Cross)
    305: 1,  # BTN_EAST (BTN_B) -> Circle
    307: 2,  # BTN_NORTH -> Triangle
    308: 3,  # BTN_WEST -> Square
    310: 4,  # BTN_TL -> L1
    311: 5,  # BTN_TR -> R1
    312: 6,  # BTN_TL2 -> L2 (button click)
    313: 7,  # BTN_TR2 -> R2 (button click)
    314: 8,  # BTN_SELECT -> Share
    315: 9,  # BTN_START -> Options
    316: 10,  # BTN_MODE -> PS button
    317: 11,  # BTN_THUMBL -> L3 (left stick press)
    318: 12,  # BTN_THUMBR -> R3 (right stick press)
}

NUM_AXES = 8
NUM_BUTTONS = 13


class TeleopNode(Node):
    def __init__(self):
        super().__init__('g1_teleop')

        self.declare_parameter('device', '/dev/input/event19')
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('publish_rate', 30.0)

        self.device_path = self.get_parameter('device').value
        self.deadzone = self.get_parameter('deadzone').value
        self.rate = self.get_parameter('publish_rate').value

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Raw state updated by reader thread
        self.axes_raw = [0.0] * NUM_AXES
        self.buttons = [0] * NUM_BUTTONS
        self.lock = threading.Lock()

        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(1.0 / self.rate, self._publish)
        self.get_logger().info(f'Teleop ready: device={self.device_path} deadzone={self.deadzone}')

    def _apply_deadzone(self, val: float) -> float:
        if abs(val) < self.deadzone:
            return 0.0
        # Rescale so output starts at 0 just outside deadzone
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self.deadzone) / (1.0 - self.deadzone)

    def _read_loop(self):
        """Blocking evdev read loop — runs in background thread."""
        while rclpy.ok():
            try:
                fd = open(self.device_path, 'rb')
                self.get_logger().info(f'Opened {self.device_path}')
            except (FileNotFoundError, PermissionError) as e:
                self.get_logger().error(f'Cannot open {self.device_path}: {e}')
                import time

                time.sleep(2.0)
                continue

            try:
                while rclpy.ok():
                    data = fd.read(EVENT_SIZE)
                    if not data:
                        break
                    _sec, _usec, ev_type, code, value = struct.unpack(EVENT_FORMAT, data)

                    with self.lock:
                        if ev_type == EV_ABS:
                            if code in AXIS_MAP:
                                idx = AXIS_MAP[code]
                                if idx <= 3:  # Sticks
                                    norm = (value - STICK_CENTER) / STICK_MAX
                                    self.axes_raw[idx] = -norm if (idx % 2) else norm
                                    self.get_logger().debug(f'Stick {idx}: raw={value} norm={self.axes_raw[idx]:.3f}')
                                elif idx <= 5:  # D-pad
                                    self.axes_raw[idx] = float(-value if idx == 5 else value)
                                else:  # Triggers (idx 6-7) - map 0-255 to 0-1
                                    norm = value / 255.0
                                    self.axes_raw[idx] = norm
                                    self.get_logger().debug(f'Trigger {idx}: raw={value} norm={norm:.3f}')
                        elif ev_type == EV_KEY and code in BTN_CODES:
                            self.buttons[BTN_CODES[code]] = value
            except OSError as e:
                self.get_logger().warn(f'Device read error: {e}')
            finally:
                fd.close()

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        with self.lock:
            msg.axes = [self._apply_deadzone(a) for a in self.axes_raw[:4]] + list(self.axes_raw[4:])
            msg.buttons = list(self.buttons)

        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
