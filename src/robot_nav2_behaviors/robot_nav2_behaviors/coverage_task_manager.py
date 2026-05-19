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
    sequence: int
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    frame_id: str


class CoverageTaskManager(Node):
    def __init__(self):
        super().__init__('coverage_task_manager')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('task_service', 'add_coverage_rectangle')
        self.declare_parameter(
            'coverage_path_service',
            'generate_coverage_path',
        )
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter(
            'execute_coverage_action',
            'execute_coverage_path',
        )
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('task_entry_margin', 0.2)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('server_timeout_sec', 5.0)

        task_service = self.get_parameter('task_service').value
        coverage_path_service = self.get_parameter(
            'coverage_path_service'
        ).value
        navigate_to_pose_action = self.get_parameter(
            'navigate_to_pose_action'
        ).value
        execute_coverage_action = self.get_parameter(
            'execute_coverage_action'
        ).value

        self.task_service = self.create_service(
            AddCoverageRectangle,
            task_service,
            self.handle_add_rectangle,
        )
        self.rectangle_client = self.create_client(
            GenerateCoveragePath,
            coverage_path_service,
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_to_pose_action,
        )
        self.coverage_client = ActionClient(
            self,
            ExecuteCoveragePath,
            execute_coverage_action,
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
        self.tf_lookup_warned = False
        self.task_timer = self.create_timer(0.2, self.start_next_task)
        self.entry_timer = self.create_timer(0.1, self.monitor_task_entry)

        self.get_logger().info(
            f'coverage task manager ready: service={task_service}, '
            f'coverage_path_service={coverage_path_service}'
        )

    def handle_add_rectangle(self, request, response):
        min_x = min(request.min_x, request.max_x)
        max_x = max(request.min_x, request.max_x)
        min_y = min(request.min_y, request.max_y)
        max_y = max(request.min_y, request.max_y)

        if min_x == max_x or min_y == max_y:
            response.success = False
            response.message = (
                'rectangle width and height must be greater than 0'
            )
            response.queued_count = self.pending_count()
            return response

        frame_id = request.frame_id or self.get_parameter('frame_id').value
        task = CoverageTask(
            sequence=self.next_sequence,
            min_x=min_x,
            min_y=min_y,
            max_x=max_x,
            max_y=max_y,
            frame_id=frame_id,
        )
        self.next_sequence += 1
        self.queue.append(task)

        response.success = True
        response.message = f'coverage rectangle queued: task={task.sequence}'
        response.queued_count = self.pending_count()
        self.get_logger().info(
            f'queued task={task.sequence}: '
            f'min=({min_x:.3f}, {min_y:.3f}), '
            f'max=({max_x:.3f}, {max_y:.3f}), frame_id={frame_id}'
        )
        return response

    def pending_count(self):
        active_count = 1 if self.active_task is not None else 0
        return active_count + len(self.queue)

    def start_next_task(self):
        if self.active_task is not None or not self.queue:
            return

        self.active_task = self.queue.popleft()
        self.reset_task_state()
        self.get_logger().info(
            f'starting coverage task={self.active_task.sequence}'
        )
        self.navigate_to_rectangle_center(self.active_task)

    def reset_task_state(self):
        self.center_goal_handle = None
        self.center_cancel_requested = False
        self.entry_detected = False
        self.scan_started = False
        self.tf_lookup_warned = False

    def navigate_to_rectangle_center(self, task):
        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.navigate_client.wait_for_server(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'NavigateToPose action server is not available',
            )
            return

        center_x, center_y = self.get_task_center(task)
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose_stamped(
            task.frame_id,
            center_x,
            center_y,
            0.0,
        )
        self.get_logger().info(
            f'navigating to rectangle center for task={task.sequence}: '
            f'x={center_x:.3f}, y={center_y:.3f}'
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
                self.start_scanning_from_current_pose('center goal rejected')
            else:
                self.finish_active_task(False, 'center goal rejected')
            return

        self.center_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_center_result)

    def monitor_task_entry(self):
        if self.active_task is None or self.scan_started:
            return

        robot_position = self.get_robot_position(self.active_task.frame_id)
        if robot_position is None:
            self.warn_tf_lookup_failure_once(self.active_task.frame_id)
            return

        self.tf_lookup_warned = False
        robot_x, robot_y = robot_position
        if not self.is_inside_entry_zone(self.active_task, robot_x, robot_y):
            return

        if not self.entry_detected:
            self.entry_detected = True
            self.get_logger().info(
                f'robot entered task={self.active_task.sequence}: '
                f'x={robot_x:.3f}, y={robot_y:.3f}'
            )

        if self.center_goal_handle is not None:
            self.cancel_center_goal()

    def cancel_center_goal(self):
        if (
            self.center_goal_handle is None
            or self.center_cancel_requested
            or self.scan_started
        ):
            return

        self.center_cancel_requested = True
        self.get_logger().info(
            f'canceling center navigation for task={self.active_task.sequence}'
        )
        future = self.center_goal_handle.cancel_goal_async()
        future.add_done_callback(self.handle_center_cancel_response)

    def handle_center_cancel_response(self, future):
        if self.active_task is None or self.scan_started:
            return

        try:
            cancel_response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'center goal cancel failed: {exc}')
        else:
            canceled_count = len(cancel_response.goals_canceling)
            self.get_logger().info(
                'center goal cancel response: '
                f'goals_canceling={canceled_count}'
            )

    def handle_center_result(self, future):
        if self.active_task is None or self.scan_started:
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:
            if self.entry_detected:
                self.start_scanning_from_current_pose(
                    f'center result failed after entry: {exc}'
                )
            else:
                self.finish_active_task(False, f'center result failed: {exc}')
            return

        status = wrapped_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'reached rectangle center for '
                f'task={self.active_task.sequence}'
            )
            self.start_scanning_from_current_pose(
                'center navigation completed'
            )
            return

        if status == GoalStatus.STATUS_CANCELED and self.entry_detected:
            self.start_scanning_from_current_pose(
                'center navigation canceled'
            )
            return

        if self.entry_detected:
            self.start_scanning_from_current_pose(
                f'center navigation ended after entry with status={status}'
            )
            return

        self.finish_active_task(
            False,
            f'center navigation failed with status={status}',
        )

    def start_scanning_from_current_pose(self, reason):
        if self.active_task is None or self.scan_started:
            return

        self.scan_started = True
        task = self.active_task
        start_position = self.get_robot_position(task.frame_id)
        if start_position is None:
            self.warn_tf_lookup_failure_once(task.frame_id)
            start_position = self.get_task_center(task)
            self.get_logger().warn(
                'using rectangle center as coverage start reference'
            )

        start_x, start_y = start_position
        self.get_logger().info(
            f'starting coverage scan for task={task.sequence}: '
            f'{reason}, start=({start_x:.3f}, {start_y:.3f})'
        )
        self.request_rectangle_path(task, start_x, start_y)

    def request_rectangle_path(self, task, start_x, start_y):
        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.rectangle_client.wait_for_service(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'rectangle service is not available',
            )
            return

        request = GenerateCoveragePath.Request()
        request.min_x = task.min_x
        request.min_y = task.min_y
        request.max_x = task.max_x
        request.max_y = task.max_y
        request.frame_id = task.frame_id
        request.use_start = True
        request.start_x = start_x
        request.start_y = start_y
        future = self.rectangle_client.call_async(request)
        future.add_done_callback(self.handle_rectangle_response)

    def handle_rectangle_response(self, future):
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

        self.execute_coverage_path(response.coverage_path)

    def execute_coverage_path(self, path):
        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.coverage_client.wait_for_server(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'ExecuteCoveragePath action server is not available',
            )
            return

        goal = ExecuteCoveragePath.Goal()
        goal.path = path
        self.get_logger().info(
            f'executing coverage path for task={self.active_task.sequence}: '
            f'poses={len(path.poses)}'
        )
        future = self.coverage_client.send_goal_async(
            goal,
            feedback_callback=self.handle_coverage_feedback,
        )
        future.add_done_callback(self.handle_coverage_goal_response)

    def handle_coverage_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'coverage goal failed: {exc}')
            return

        if not goal_handle.accepted:
            self.finish_active_task(False, 'coverage goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_coverage_result)

    def handle_coverage_feedback(self, feedback):
        self.get_logger().debug(
            'coverage remaining poses: '
            f'{feedback.feedback.remaining_poses}'
        )

    def handle_coverage_result(self, future):
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
        if self.active_task is None:
            return

        task = self.active_task
        level = self.get_logger().info if success else self.get_logger().warn
        level(f'finished task={task.sequence}: {message}')
        self.active_task = None
        self.reset_task_state()

    def warn_tf_lookup_failure_once(self, frame_id):
        if self.tf_lookup_warned:
            return

        robot_base_frame = self.get_parameter('robot_base_frame').value
        self.get_logger().warn(
            f'failed to lookup TF {frame_id} <- {robot_base_frame}'
        )
        self.tf_lookup_warned = True

    def get_robot_position(self, frame_id):
        robot_base_frame = self.get_parameter('robot_base_frame').value
        timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                robot_base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
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

    def get_task_center(self, task):
        return (
            (task.min_x + task.max_x) / 2.0,
            (task.min_y + task.max_y) / 2.0,
        )

    def is_inside_entry_zone(self, task, x, y):
        margin = max(0.0, float(self.get_parameter('task_entry_margin').value))
        margin_x = min(
            margin,
            max(0.0, (task.max_x - task.min_x) / 2.0 - 1e-6),
        )
        margin_y = min(
            margin,
            max(0.0, (task.max_y - task.min_y) / 2.0 - 1e-6),
        )
        return (
            (task.min_x + margin_x) <= x <= (task.max_x - margin_x)
            and (task.min_y + margin_y) <= y <= (task.max_y - margin_y)
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
    rclpy.init(args=args)
    node = CoverageTaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
