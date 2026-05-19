from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class CoverageNavigator(Node):
    def __init__(self):
        super().__init__('coverage_navigator')
        self.declare_parameter('coverage_path_topic', 'coverage_path')
        self.declare_parameter('navigate_action', 'navigate_through_poses')
        self.declare_parameter('server_timeout_sec', 5.0)

        coverage_path_topic = self.get_parameter('coverage_path_topic').value
        self.navigate_action = self.get_parameter('navigate_action').value

        self.action_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_action,
        )
        self.path_subscription = self.create_subscription(
            Path,
            coverage_path_topic,
            self.handle_coverage_path,
            10,
        )
        self.active_goal_handle = None
        self.get_logger().info(
            f'coverage navigator ready: topic={coverage_path_topic}, '
            f'action={self.navigate_action}'
        )

    def handle_coverage_path(self, path):
        if not path.poses:
            self.get_logger().warn('received empty coverage path')
            return

        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warn(
                'NavigateThroughPoses action server is not available: '
                f'{self.navigate_action}'
            )
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = path.poses
        self.get_logger().info(
            f'sending coverage path to Nav2 with {len(goal.poses)} poses'
        )
        goal_future = self.action_client.send_goal_async(goal)
        goal_future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('coverage navigation goal rejected')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('coverage navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_navigation_result)

    def handle_navigation_result(self, future):
        result = future.result()
        self.active_goal_handle = None
        self.get_logger().info(
            f'coverage navigation finished with status={result.status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
