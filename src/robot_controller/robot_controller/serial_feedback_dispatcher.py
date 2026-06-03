from robot_controller.packet_protocol import (
    PACKET_TYPE_MOTOR_STATE,
    unpack_motor_state,
)


class SerialFeedbackDispatcher:
    """Dispatch decoded serial feedback frames to ROS bridge publishers."""

    def __init__(self, node):
        self._node = node
        self._handlers = {
            PACKET_TYPE_MOTOR_STATE: self._handle_motor_state,
        }

    def handle_frame(self, frame_type, payload):
        """Handle one decoded serial frame."""
        handler = self._handlers.get(frame_type)
        if handler:
            handler(payload)
        elif self._node._debug:
            self._node.get_logger().info(
                f'Ignoring unsupported serial frame type=0x{frame_type:02X}, '
                f'payload_len={len(payload)}'
            )

    def dispatch_available(self):
        """Read and dispatch all currently available serial feedback frames."""
        for frame_type, payload in self._node._serial.read_frames():
            self.handle_frame(frame_type, payload)

    def _handle_motor_state(self, payload):
        try:
            motor_state = unpack_motor_state(payload)
        except ValueError:
            self._node.get_logger().warning(
                'Ignoring motor state feedback with invalid payload length: '
                f'{len(payload)}'
            )
            return

        self._node.get_logger().info(f'Parsed motor state: {motor_state}')
        if not self._node._motor_open_loop_control:
            self._node._motor.publish_feedback(motor_state)
