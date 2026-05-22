from std_msgs.msg import Float64MultiArray


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
            cmd_str = 'ARM_POS:' + ','.join(f'{v:.6f}' for v in command)
            self._cmd_callback(cmd_str.encode('utf-8'))

        if self._open_loop_control:
            self._positions = command
            self.publish_feedback(command)

    def publish_feedback(self, data: list):
        """Publish current arm joint positions."""
        self._positions = [float(value) for value in data[:self._joint_count]]
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._fb_pub.publish(msg)

    def _publish_feedback(self):
        self.publish_feedback(self._positions)
