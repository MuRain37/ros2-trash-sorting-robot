import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmJointPublisher(Node):
    """机械臂关节状态发布节点：以零位发布关节状态，供 robot_state_publisher 生成 TF。"""

    JOINTS = [
        'arm_base_joint',
        'arm_shoulder_joint',
        'arm_elbow_joint',
        'arm_wrist_joint',
    ]

    def __init__(self):
        super().__init__('arm_joint_publisher')

        self.declare_parameter('rate', 10.0)
        rate = self.get_parameter('rate').value

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'Arm joint publisher started ({rate}Hz)')

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINTS
        msg.position = [0.0] * len(self.JOINTS)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
