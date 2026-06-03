from std_msgs.msg import Float64MultiArray
from robot_controller.packet_protocol import (
    PACKET_CMD_MOTOR,
    pack_motor_command,
)


class MotorBridge:
    """Bridge motor commands and wheel velocity feedback."""

    def __init__(
        self, node, cmd_topic='diff_test_cmd', fb_topic='diff_test_fb',
        cmd_callback=None, open_loop_control=False
    ):
        self._node = node
        self._cmd_callback = cmd_callback
        self._open_loop_control = open_loop_control

        self._fb_pub = node.create_publisher(
            Float64MultiArray, fb_topic, 10,
        )

        self._cmd_sub = node.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self._on_cmd,
            10,
        )

    def publish_feedback(self, data: list):
        """Publish encoder feedback data."""
        fb = Float64MultiArray()
        fb.data = list(data)
        self._fb_pub.publish(fb)

    def _on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self._node.get_logger().warning(
                f'Expected 4 wheel velocities, got {len(msg.data)}'
            )
            return

        if self._cmd_callback:
            payload = pack_motor_command(msg.data)
            self._cmd_callback(PACKET_CMD_MOTOR, payload)

        if self._open_loop_control:
            self.publish_feedback(msg.data)
