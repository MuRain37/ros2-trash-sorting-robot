from std_msgs.msg import Float64MultiArray


class MotorBridge:
    """电机指令桥接：订阅硬件接口指令，并发布轮速反馈。"""

    def __init__(self, node, cmd_topic='diff_test_cmd', fb_topic='diff_test_fb',
                 cmd_callback=None, open_loop_control=False):
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
        """发布真实编码器反馈数据。"""
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
            cmd_str = 'MOTOR:' + ','.join(f'{v:.3f}' for v in msg.data)
            self._cmd_callback(cmd_str.encode('utf-8'))

        if self._open_loop_control:
            self.publish_feedback(msg.data)
