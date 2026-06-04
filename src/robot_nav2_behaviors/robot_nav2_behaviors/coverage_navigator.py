"""覆盖路径执行节点：把自定义覆盖 Action 转发给 Nav2 多点导航。"""

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from robot_interfaces.action import ExecuteCoveragePath


class CoverageNavigator(Node):
    """接收覆盖路径并调用 NavigateThroughPoses 执行。"""

    def __init__(self):
        super().__init__('coverage_navigator')
        # 对外提供的覆盖路径执行 Action 名
        self.declare_parameter('execute_action', 'execute_coverage_path')
        # Nav2 NavigateThroughPoses Action 名
        self.declare_parameter('navigate_action', 'navigate_through_poses')
        # 等待 Nav2 Action 服务的超时时间（s）
        self.declare_parameter('server_timeout_sec', 5.0)

        self.execute_action = self.get_parameter('execute_action').value
        self.navigate_action = self.get_parameter('navigate_action').value

        self.navigate_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_action,
        )
        self.execute_server = ActionServer(
            self,
            ExecuteCoveragePath,
            self.execute_action,
            self.execute_callback,
        )
        self.get_logger().info(
            f'coverage navigator ready: action={self.execute_action}, '
            f'nav2_action={self.navigate_action}'
        )

    async def execute_callback(self, goal_handle):
        """执行一条 nav_msgs/Path 覆盖路径。"""
        result = ExecuteCoveragePath.Result()
        path = goal_handle.request.path

        if not path.poses:
            result.success = False
            result.message = 'coverage path is empty'
            goal_handle.abort()
            return result

        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.navigate_client.wait_for_server(timeout_sec=timeout_sec):
            result.success = False
            result.message = (
                'NavigateThroughPoses action server is not available: '
                f'{self.navigate_action}'
            )
            goal_handle.abort()
            return result

        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = path.poses
        self.get_logger().info(
            f'executing coverage path with {len(nav_goal.poses)} poses'
        )

        # 覆盖 Action 不直接导航，只把路径转交给 Nav2 多点导航。
        nav_goal_future = self.navigate_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda feedback: self.publish_feedback(
                goal_handle,
                feedback,
            ),
        )
        nav_goal_handle = await nav_goal_future
        if not nav_goal_handle.accepted:
            result.success = False
            result.message = 'NavigateThroughPoses goal rejected'
            goal_handle.abort()
            return result

        nav_result_future = nav_goal_handle.get_result_async()
        nav_result = await nav_result_future

        # Nav2 的状态决定自定义覆盖 Action 的最终状态。
        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.message = 'coverage path executed'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = (
                'coverage path execution failed with status='
                f'{nav_result.status}'
            )
            goal_handle.abort()

        self.get_logger().info(result.message)
        return result

    def publish_feedback(self, goal_handle, feedback):
        """转发 Nav2 剩余路点数量。"""
        execute_feedback = ExecuteCoveragePath.Feedback()
        execute_feedback.remaining_poses = (
            feedback.feedback.number_of_poses_remaining
        )
        goal_handle.publish_feedback(execute_feedback)


def main(args=None):
    """启动覆盖路径执行节点。"""
    rclpy.init(args=args)
    node = CoverageNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
