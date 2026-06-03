from std_msgs.msg import Float64MultiArray

from robot_controller.packet_protocol import (
    PACKET_CMD_SERVO,
    pack_servo_command,
)


class ArmBridge:
    """Arm bridge for position commands and feedback."""

    def __init__(self, node, cmd_topic='arm_position_cmd',
                 fb_topic='arm_position_fb', cmd_callback=None,
                 open_loop_control=False, joint_count=5):
        self._node = node
        self._cmd_callback = cmd_callback
        self._open_loop_control = open_loop_control
        self._joint_count = joint_count
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

        if self._cmd_callback:
            for index, position in enumerate(command):
                servo_id = index + 1
                payload = pack_servo_command(servo_id, position)
                self._cmd_callback(PACKET_CMD_SERVO, payload)

        if self._open_loop_control:
            self._positions = command
            self.publish_feedback(command)

    def publish_feedback(self, data: list):
        """Publish current arm joint positions."""
        self._positions = [float(value) for value in data[:self._joint_count]]
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._fb_pub.publish(msg)

    def publish_servo_feedback(self, servo_id, position):
        """Update one 1-based servo feedback value and publish all joints."""
        if not 1 <= servo_id <= self._joint_count:
            self._node.get_logger().warning(
                f'Ignoring servo feedback with invalid id: {servo_id}'
            )
            return

        self._positions[servo_id - 1] = float(position)
        self.publish_feedback(self._positions)

    def _publish_feedback(self):
        self.publish_feedback(self._positions)
