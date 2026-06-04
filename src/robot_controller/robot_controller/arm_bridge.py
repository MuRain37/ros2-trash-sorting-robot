import math
import time

from std_msgs.msg import Float64MultiArray

from robot_controller.packet_protocol import (
    PACKET_CMD_SERVO,
    pack_servo_command,
)


DEFAULT_ARM_JOINT_COUNT = 5
DEFAULT_SERVO_SEND_RATE = 10.0
DEFAULT_SERVO_DURATION_MS = 100.0
SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0
JOINT_MAX_DEGREES = (180.0, 120.0, 120.0, 120.0, 120.0)
JOINT_DIRECTIONS = (-1.0, -1.0, -1.0, -1.0, 1.0)


def clamp_servo_raw(position):
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, position))


def joint_command_to_servo_raw(index, value):
    """Convert one arm command value to a raw bus-servo position."""
    value = math.degrees(float(value))
    if index < len(JOINT_MAX_DEGREES):
        max_degrees = JOINT_MAX_DEGREES[index]
        direction = JOINT_DIRECTIONS[index]
        if not -max_degrees <= value <= max_degrees:
            raise ValueError(
                f'joint {index + 1} out of range: {value} '
                f'not in [{-max_degrees}, {max_degrees}] deg'
            )
        return clamp_servo_raw(
            SERVO_CENTER + direction * value * (SERVO_CENTER / max_degrees)
        )

    raise ValueError(f'unsupported arm joint index: {index}')


def servo_raw_to_joint_command(index, raw_position):
    """Convert one raw bus-servo position to the arm command unit."""
    raw_position = float(raw_position)
    if index < len(JOINT_MAX_DEGREES):
        max_degrees = JOINT_MAX_DEGREES[index]
        direction = JOINT_DIRECTIONS[index]
        degrees = (raw_position - SERVO_CENTER) * (
            max_degrees / SERVO_CENTER
        )
        degrees *= direction
        return math.radians(degrees)

    raise ValueError(f'unsupported arm joint index: {index}')


class ArmBridge:
    """Arm bridge for position commands and feedback."""

    def __init__(self, node, cmd_topic='arm_position_cmd',
                 fb_topic='arm_position_fb', cmd_callback=None,
                 open_loop_control=False, print_only=False,
                 send_rate=DEFAULT_SERVO_SEND_RATE,
                 duration_ms=DEFAULT_SERVO_DURATION_MS):
        self._node = node
        self._cmd_callback = cmd_callback
        self._open_loop_control = open_loop_control
        self._joint_count = DEFAULT_ARM_JOINT_COUNT
        self._print_only = print_only
        self._send_period = 1.0 / float(send_rate) if send_rate > 0.0 else 0.0
        self._duration_ms = float(duration_ms)
        self._last_send_time = None
        self._positions = [0.0] * self._joint_count

        self._fb_pub = node.create_publisher(Float64MultiArray, fb_topic, 10)
        self._cmd_sub = node.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self._on_command,
            10,
        )

    def _on_command(self, msg: Float64MultiArray):
        if len(msg.data) < self._joint_count:
            self._node.get_logger().warning(
                f'Expected at least {self._joint_count} arm positions, '
                f'got {len(msg.data)}'
            )
            return

        command = [float(value) for value in msg.data[:self._joint_count]]
        try:
            servo_positions = [
                joint_command_to_servo_raw(index, value)
                for index, value in enumerate(command)
            ]
        except ValueError as e:
            self._node.get_logger().warning(f'Invalid arm command: {e}')
            return

        if self._should_send_now():
            self._log_servo_targets(servo_positions)
            self._send_servo_targets(servo_positions)

        if self._open_loop_control:
            self._positions = command
            self.publish_feedback(command)

    def publish_feedback(self, data: list):
        """Publish current arm joint positions."""
        self._positions = [float(value) for value in data[:self._joint_count]]
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._fb_pub.publish(msg)

    def _log_servo_targets(self, servo_positions):
        targets = ', '.join(
            f'servo{index + 1}={position:.1f}'
            for index, position in enumerate(servo_positions)
        )
        mode = 'print-only' if self._print_only else 'send'
        self._node.get_logger().info(
            f'Arm servo targets ({mode}): {targets}'
        )

    def _send_servo_targets(self, servo_positions):
        if not self._cmd_callback or self._print_only:
            return

        for index, position in enumerate(servo_positions):
            servo_id = index + 1
            payload = pack_servo_command(
                servo_id,
                position,
                self._duration_ms,
            )
            self._cmd_callback(PACKET_CMD_SERVO, payload)

    def _should_send_now(self):
        now = time.monotonic()
        if (
            self._last_send_time is not None
            and now - self._last_send_time < self._send_period
        ):
            return False

        self._last_send_time = now
        return True

    def publish_servo_feedback(self, servo_id, position):
        """Update one 1-based servo feedback value and publish all joints."""
        if not 1 <= servo_id <= self._joint_count:
            self._node.get_logger().warning(
                f'Ignoring servo feedback with invalid id: {servo_id}'
            )
            return

        try:
            self._positions[servo_id - 1] = servo_raw_to_joint_command(
                servo_id - 1,
                position,
            )
        except ValueError as e:
            self._node.get_logger().warning(
                f'Ignoring servo feedback: {e}'
            )
            return
        self.publish_feedback(self._positions)

    def _publish_feedback(self):
        self.publish_feedback(self._positions)
