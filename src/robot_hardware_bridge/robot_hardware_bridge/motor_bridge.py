"""电机桥接模块：轮速指令发送与电机状态反馈发布."""

from std_msgs.msg import Float64MultiArray
from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_MOTOR,
    pack_motor_command,
)


class MotorBridge:
    """
    桥接 diff_drive 速度指令和 STM32 电机控制.

    默认订阅 diff_test_cmd（4个轮速），打包后通过回调发送；
    接收电机状态反馈后发布到 diff_test_fb。
    """

    def __init__(
        self, node, cmd_topic='diff_test_cmd', fb_topic='diff_test_fb',
        cmd_callback=None, open_loop_control=False
    ):
        self._node = node
        # 发送回调，签名为 (cmd, payload)，实际绑定到 serial.write_frame
        self._cmd_callback = cmd_callback
        self._open_loop_control = open_loop_control

        # 发布电机状态反馈
        self._fb_pub = node.create_publisher(
            Float64MultiArray, fb_topic, 10,
        )

        # 订阅轮速指令
        self._cmd_sub = node.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self._on_cmd,
            10,
        )

    def publish_feedback(self, data: list):
        """发布4个轮子的电机状态反馈值."""
        fb = Float64MultiArray()
        fb.data = list(data)
        self._fb_pub.publish(fb)

    def _on_cmd(self, msg: Float64MultiArray):
        """收到轮速指令：打包为16字节负载 → 通过回调发送到串口."""
        if len(msg.data) != 4:
            self._node.get_logger().warning(
                f'Expected 4 wheel velocities, got {len(msg.data)}'
            )
            return

        if self._cmd_callback:
            payload = pack_motor_command(msg.data)
            self._cmd_callback(PACKET_CMD_MOTOR, payload)

        # 开环模式：把指令值直接当反馈发出
        if self._open_loop_control:
            self.publish_feedback(msg.data)
