import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ChassisController(Node):
    """底盘控制节点：接收硬件接口目标指令，发布真实硬件反馈数据。

    当前为开环控制模式：直接将指令回写为反馈，模拟理想执行。
    后续接入真实硬件时替换 open_loop_feedback 中的逻辑即可。
    """

    def __init__(self):
        super().__init__('chassis_controller')

        self.declare_parameter('cmd_topic', 'diff_test_cmd')
        self.declare_parameter('fb_topic', 'diff_test_fb')
        self.declare_parameter('open_loop', True)

        cmd_topic = self.get_parameter('cmd_topic').value
        fb_topic = self.get_parameter('fb_topic').value
        self._open_loop = self.get_parameter('open_loop').value

        # 订阅硬件接口发布的目标指令
        self._cmd_sub = self.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self._cmd_callback,
            10,
        )

        # 发布真实硬件反馈数据
        self._fb_pub = self.create_publisher(
            Float64MultiArray,
            fb_topic,
            10,
        )

        mode = 'open-loop' if self._open_loop else 'hardware'
        self.get_logger().info(
            f'Chassis controller started (mode: {mode}, '
            f'cmd_topic: {cmd_topic}, fb_topic: {fb_topic})'
        )

    def _cmd_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warning(
                f'Expected 4 wheel velocities, got {len(msg.data)}'
            )
            return

        if self._open_loop:
            self._open_loop_feedback(msg)

    def _open_loop_feedback(self, cmd: Float64MultiArray):
        fb = Float64MultiArray()
        fb.data = list(cmd.data)
        self._fb_pub.publish(fb)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
