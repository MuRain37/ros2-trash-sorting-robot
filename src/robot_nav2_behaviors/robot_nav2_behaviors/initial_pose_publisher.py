"""初始位姿发布节点：启动时重复向 AMCL 发布 /initialpose。"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    """按固定次数发布带协方差的初始位姿。"""

    def __init__(self):
        super().__init__('initial_pose_publisher')
        # AMCL 初始位姿话题
        self.declare_parameter('initial_pose_topic', '/initialpose')
        # 初始位姿所在坐标系
        self.declare_parameter('frame_id', 'map')
        # 初始 x 坐标（m）
        self.declare_parameter('initial_pose_x', 0.0)
        # 初始 y 坐标（m）
        self.declare_parameter('initial_pose_y', 0.0)
        # 初始航向角（rad）
        self.declare_parameter('initial_pose_yaw', 0.0)
        # x/y 协方差标准差（m）
        self.declare_parameter('position_stddev', 0.25)
        # yaw 协方差标准差（rad）
        self.declare_parameter('yaw_stddev', 0.3)
        # 重复发布周期（s）
        self.declare_parameter('publish_period', 1.0)
        # 重复发布次数
        self.declare_parameter('publish_attempts', 10)

        topic = self.get_parameter('initial_pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.initial_pose_x = float(self.get_parameter('initial_pose_x').value)
        self.initial_pose_y = float(self.get_parameter('initial_pose_y').value)
        self.initial_pose_yaw = float(
            self.get_parameter('initial_pose_yaw').value
        )
        self.position_stddev = float(
            self.get_parameter('position_stddev').value
        )
        self.yaw_stddev = float(self.get_parameter('yaw_stddev').value)
        publish_period = float(self.get_parameter('publish_period').value)
        self.remaining_attempts = max(
            1,
            int(self.get_parameter('publish_attempts').value),
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            topic,
            10,
        )
        self.timer = self.create_timer(
            publish_period,
            self.publish_initial_pose,
        )
        self.get_logger().info(
            'initial pose publisher ready: '
            f'topic={topic}, frame_id={self.frame_id}, '
            f'pose=({self.initial_pose_x:.3f}, {self.initial_pose_y:.3f}, '
            f'{self.initial_pose_yaw:.3f})'
        )

    def publish_initial_pose(self):
        """发布一次初始位姿，次数耗尽后停止定时器。"""
        if self.remaining_attempts <= 0:
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.orientation.z = math.sin(self.initial_pose_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_pose_yaw / 2.0)
        # AMCL 只关心 x、y、yaw 的初始不确定性，这里填 6x6 协方差矩阵对应位置。
        covariance = [0.0] * 36
        covariance[0] = self.position_stddev ** 2
        covariance[7] = self.position_stddev ** 2
        covariance[35] = self.yaw_stddev ** 2
        msg.pose.covariance = covariance

        self.publisher.publish(msg)
        self.remaining_attempts -= 1
        self.get_logger().info(
            'published initial pose, remaining_attempts='
            f'{self.remaining_attempts}'
        )

        if self.remaining_attempts <= 0:
            self.timer.cancel()


def main(args=None):
    """启动初始位姿发布节点。"""
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
