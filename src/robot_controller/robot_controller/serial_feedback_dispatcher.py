from robot_controller.packet_protocol import (
    PACKET_CMD_MOTOR,
    PACKET_CMD_SERVO,
    unpack_encoder_feedback,
    unpack_servo_feedback,
)


class SerialFeedbackDispatcher:
    """Dispatch decoded serial feedback frames to ROS bridge publishers."""

    def __init__(self, node):
        self._node = node
        self._handlers = {
            PACKET_CMD_MOTOR: self._handle_encoder_feedback,
            PACKET_CMD_SERVO: self._handle_servo_feedback,
        }

    def handle_frame(self, cmd, payload):
        """Handle one decoded serial frame."""
        handler = self._handlers.get(cmd)
        if handler:
            handler(payload)
        elif self._node._debug:
            self._node.get_logger().info(
                f'Ignoring unsupported serial frame cmd=0x{cmd:02X}, '
                f'payload_len={len(payload)}'
            )

    def dispatch_available(self):
        """Read and dispatch all currently available serial feedback frames."""
        for cmd, payload in self._node._serial.read_frames():
            self.handle_frame(cmd, payload)

    def _handle_encoder_feedback(self, payload):
        try:
            encoder = unpack_encoder_feedback(payload)
        except ValueError:
            self._node.get_logger().warning(
                'Ignoring encoder feedback with invalid payload length: '
                f'{len(payload)}'
            )
            return

        self._node.get_logger().info(f'Parsed encoder: {encoder}')
        if not self._node._motor_open_loop_control:
            self._node._motor.publish_feedback(encoder)

    def _handle_servo_feedback(self, payload):
        try:
            servo_id, position = unpack_servo_feedback(payload)
        except ValueError:
            self._node.get_logger().warning(
                'Ignoring servo feedback with invalid payload length: '
                f'{len(payload)}'
            )
            return

        self._node.get_logger().info(
            f'Parsed servo feedback: id={servo_id}, position={position}'
        )
        if not self._node._arm_open_loop_control:
            self._node._arm.publish_servo_feedback(servo_id, position)
