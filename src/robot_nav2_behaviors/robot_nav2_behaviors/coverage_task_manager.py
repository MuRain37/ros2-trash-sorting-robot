"""覆盖任务管理节点：排队矩形任务并协调 Nav2 进入与清扫。"""

import math
from collections import deque
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_interfaces.action import ExecuteCoveragePath
from robot_interfaces.srv import AddCoverageRectangle, GenerateCoveragePath
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)


@dataclass
class CoverageTask:
    """单个矩形覆盖任务。"""

    sequence: int
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    frame_id: str

    @property
    def center(self):
        return (
            (self.min_x + self.max_x) / 2.0,
            (self.min_y + self.max_y) / 2.0,
        )

    def contains(self, x, y, margin):
        margin = max(0.0, margin)
        limit_x = max(0.0, (self.max_x - self.min_x) / 2.0 - 1e-6)
        limit_y = max(0.0, (self.max_y - self.min_y) / 2.0 - 1e-6)
        margin_x = min(margin, limit_x)
        margin_y = min(margin, limit_y)
        return (
            self.min_x + margin_x <= x <= self.max_x - margin_x
            and self.min_y + margin_y <= y <= self.max_y - margin_y
        )


class CoverageTaskManager(Node):
    """管理矩形任务队列、中心导航和覆盖路径执行。"""

    def __init__(self):
        super().__init__('coverage_task_manager')
        # 覆盖任务使用的坐标系
        self.declare_parameter('frame_id', 'map')
        # 添加覆盖矩形的服务名
        self.declare_parameter('task_service', 'add_coverage_rectangle')
        # 覆盖路径生成服务名
        self.declare_parameter('coverage_path_service', 'generate_coverage_path')
        # Nav2 单点导航 Action 名
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        # 覆盖路径执行 Action 名
        self.declare_parameter('execute_coverage_action', 'execute_coverage_path')
        # 机器人底盘坐标系
        self.declare_parameter('robot_base_frame', 'base_link')
        # 判断进入区域时内缩的边距（m）
        self.declare_parameter('task_entry_margin', 0.0)
        # 查询 TF 的超时时间（s）
        self.declare_parameter('tf_timeout_sec', 0.2)
        # 等待服务或 Action 的超时时间（s）
        self.declare_parameter('server_timeout_sec', 5.0)

        self.task_service = self.create_service(
            AddCoverageRectangle,
            self.get_parameter('task_service').value,
            self.handle_add_rectangle,
        )
        self.rectangle_client = self.create_client(
            GenerateCoveragePath,
            self.get_parameter('coverage_path_service').value,
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_to_pose_action').value,
        )
        self.coverage_client = ActionClient(
            self,
            ExecuteCoveragePath,
            self.get_parameter('execute_coverage_action').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.queue = deque()
        self.active_task = None
        self.next_sequence = 1
        self.center_goal_handle = None
        self.center_cancel_requested = False
        self.entry_detected = False
        self.scan_started = False
        self.task_timer = self.create_timer(0.2, self.start_next_task)
        self.entry_timer = self.create_timer(0.1, self.monitor_task_entry)

        self.get_logger().info(
            'coverage task manager ready: '
            f'service={self.get_parameter("task_service").value}, '
            f'coverage_path_service='
            f'{self.get_parameter("coverage_path_service").value}'
        )

    def handle_add_rectangle(self, request, response):
        """把矩形区域加入任务队列。"""
        min_x = min(request.min_x, request.max_x)
        max_x = max(request.min_x, request.max_x)
        min_y = min(request.min_y, request.max_y)
        max_y = max(request.min_y, request.max_y)
        if min_x >= max_x or min_y >= max_y:
            response.success = False
            response.message = (
                'rectangle width and height must be greater than 0'
            )
            response.queued_count = self.pending_count()
            return response

        frame_id = request.frame_id or self.get_parameter('frame_id').value
        self.queue.append(
            CoverageTask(
                sequence=self.next_sequence,
                min_x=min_x,
                min_y=min_y,
                max_x=max_x,
                max_y=max_y,
                frame_id=frame_id,
            )
        )
        self.next_sequence += 1

        response.success = True
        response.message = (
            f'coverage rectangle queued: task={self.next_sequence - 1}'
        )
        response.queued_count = self.pending_count()
        return response

    def pending_count(self):
        return len(self.queue) + (1 if self.active_task is not None else 0)

    def start_next_task(self):
        """空闲时启动下一个覆盖任务。"""
        if self.active_task is not None or not self.queue:
            return

        self.active_task = self.queue.popleft()
        self.reset_task_state()
        self.get_logger().info(
            f'starting coverage task={self.active_task.sequence}'
        )
        self.navigate_to_rectangle_center()

    def reset_task_state(self):
        self.center_goal_handle = None
        self.center_cancel_requested = False
        self.entry_detected = False
        self.scan_started = False

    def navigate_to_rectangle_center(self):
        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.navigate_client.wait_for_server(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'NavigateToPose action server is not available',
            )
            return

        center_x, center_y = self.active_task.center
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose_stamped(
            self.active_task.frame_id,
            center_x,
            center_y,
            0.0,
        )
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.handle_center_goal_response)

    def handle_center_goal_response(self, future):
        if self.active_task is None:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'center goal failed: {exc}')
            return

        if not goal_handle.accepted:
            if self.entry_detected:
                # 目标被拒但机器人已在区域内，直接从当前位置开始清扫。
                self.start_scanning_from_current_pose('center goal rejected')
            else:
                self.finish_active_task(False, 'center goal rejected')
            return

        self.center_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_center_result)
        if self.entry_detected:
            # 可能在等待目标响应期间已经进入区域，立即取消中心导航。
            self.cancel_center_goal()

    def monitor_task_entry(self):
        """检测机器人是否已进入当前任务区域。"""
        if self.active_task is None or self.scan_started:
            return

        robot_position = self.get_robot_position(self.active_task.frame_id)
        if robot_position is None:
            return

        if not self.active_task.contains(
            robot_position[0],
            robot_position[1],
            float(self.get_parameter('task_entry_margin').value),
        ):
            return

        if not self.entry_detected:
            self.entry_detected = True
            self.get_logger().info(
                f'robot entered task={self.active_task.sequence}: '
                f'x={robot_position[0]:.3f}, y={robot_position[1]:.3f}'
            )

        # 进入区域后不再强制走到中心点，减少无意义绕行。
        self.cancel_center_goal()

    def cancel_center_goal(self):
        if (
            self.center_goal_handle is None
            or self.center_cancel_requested
            or self.scan_started
        ):
            return

        self.center_cancel_requested = True
        self.center_goal_handle.cancel_goal_async()

    def handle_center_result(self, future):
        if self.active_task is None or self.scan_started:
            return

        try:
            status = future.result().status
        except Exception as exc:
            if self.entry_detected:
                self.start_scanning_from_current_pose(
                    f'center result failed: {exc}'
                )
            else:
                self.finish_active_task(False, f'center result failed: {exc}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED or self.entry_detected:
            # 到达中心或已进入区域，都可以开始生成覆盖路径。
            self.start_scanning_from_current_pose(
                'center navigation completed'
                if status == GoalStatus.STATUS_SUCCEEDED
                else f'center navigation ended with status={status}',
            )
            return

        self.finish_active_task(
            False,
            f'center navigation failed with status={status}',
        )

    def start_scanning_from_current_pose(self, reason):
        """从当前位置请求覆盖路径并开始清扫。"""
        if self.active_task is None or self.scan_started:
            return

        self.scan_started = True
        start_position = self.get_robot_position(self.active_task.frame_id)
        if start_position is None:
            # TF 暂不可用时退回矩形中心，保证任务流程能继续。
            start_position = self.active_task.center

        request = GenerateCoveragePath.Request()
        request.min_x = self.active_task.min_x
        request.min_y = self.active_task.min_y
        request.max_x = self.active_task.max_x
        request.max_y = self.active_task.max_y
        request.frame_id = self.active_task.frame_id
        request.use_start = True
        request.start_x = start_position[0]
        request.start_y = start_position[1]

        self.get_logger().info(
            f'starting coverage scan for task={self.active_task.sequence}: '
            f'{reason}, '
            f'start=({start_position[0]:.3f}, {start_position[1]:.3f})'
        )

        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.rectangle_client.wait_for_service(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'rectangle service is not available',
            )
            return

        future = self.rectangle_client.call_async(request)
        future.add_done_callback(self.handle_rectangle_response)

    def handle_rectangle_response(self, future):
        if self.active_task is None:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'rectangle service failed: {exc}')
            return

        if not response.success:
            self.finish_active_task(False, response.message)
            return

        if not response.coverage_path.poses:
            self.finish_active_task(
                False,
                'coverage path service returned no path',
            )
            return

        # 路径生成完成后交给 CoverageNavigator 统一调用 NavigateThroughPoses。
        if not self.coverage_client.wait_for_server(
            timeout_sec=float(self.get_parameter('server_timeout_sec').value),
        ):
            self.finish_active_task(
                False,
                'ExecuteCoveragePath action server is not available',
            )
            return

        goal = ExecuteCoveragePath.Goal()
        goal.path = response.coverage_path
        future = self.coverage_client.send_goal_async(
            goal,
            feedback_callback=self.handle_coverage_feedback,
        )
        future.add_done_callback(self.handle_coverage_goal_response)

    def handle_coverage_goal_response(self, future):
        if self.active_task is None:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'coverage goal failed: {exc}')
            return

        if not goal_handle.accepted:
            self.finish_active_task(False, 'coverage goal rejected')
            return

        goal_handle.get_result_async().add_done_callback(
            self.handle_coverage_result
        )

    def handle_coverage_feedback(self, feedback):
        self.get_logger().debug(
            'coverage remaining poses: '
            f'{feedback.feedback.remaining_poses}'
        )

    def handle_coverage_result(self, future):
        if self.active_task is None:
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'coverage result failed: {exc}')
            return

        result = wrapped_result.result
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.success
        ):
            self.finish_active_task(True, result.message)
        else:
            self.finish_active_task(
                False,
                f'{result.message}; status={wrapped_result.status}',
            )

    def finish_active_task(self, success, message):
        """结束当前任务并释放队列调度。"""
        if self.active_task is None:
            return

        task = self.active_task
        (self.get_logger().info if success else self.get_logger().warn)(
            f'finished task={task.sequence}: {message}'
        )
        self.active_task = None
        self.reset_task_state()

    def get_robot_position(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.get_parameter('robot_base_frame').value,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter('tf_timeout_sec').value)
                ),
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ):
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def create_pose_stamped(self, frame_id, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    """启动覆盖任务管理节点。"""
    rclpy.init(args=args)
    node = CoverageTaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
