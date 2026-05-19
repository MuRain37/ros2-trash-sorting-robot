import math
from collections import deque
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_interfaces.action import ExecuteCoveragePath
from robot_interfaces.srv import AddCoverageRectangle, GenerateCoveragePath


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
        self.declare_parameter('coverage_path_service', 'generate_coverage_path')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('execute_coverage_action', 'execute_coverage_path')
        self.declare_parameter('approach_distance', 0.5)
        self.declare_parameter('server_timeout_sec', 5.0)

        task_service = self.get_parameter('task_service').value
        coverage_path_service = self.get_parameter('coverage_path_service').value
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

        self.queue = deque()
        self.active_task = None
        self.next_sequence = 1
        self.task_timer = self.create_timer(0.2, self.start_next_task)

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
            response.message = 'rectangle width and height must be greater than 0'
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
        self.get_logger().info(f'starting coverage task={self.active_task.sequence}')
        self.navigate_to_approach(self.active_task)

    def navigate_to_approach(self, task):
        timeout_sec = float(self.get_parameter('server_timeout_sec').value)
        if not self.navigate_client.wait_for_server(timeout_sec=timeout_sec):
            self.finish_active_task(
                False,
                'NavigateToPose action server is not available',
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.create_approach_pose(task)
        self.get_logger().info(
            f'navigating to approach pose for task={task.sequence}: '
            f'x={goal.pose.pose.position.x:.3f}, '
            f'y={goal.pose.pose.position.y:.3f}'
        )
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.handle_approach_goal_response)

    def create_approach_pose(self, task):
        width = task.max_x - task.min_x
        height = task.max_y - task.min_y
        approach_distance = float(
            self.get_parameter('approach_distance').value
        )

        if width >= height:
            x = task.min_x - approach_distance
            y = (task.min_y + task.max_y) / 2.0
            yaw = 0.0
        else:
            x = (task.min_x + task.max_x) / 2.0
            y = task.min_y - approach_distance
            yaw = math.pi / 2.0

        return self.create_pose_stamped(task.frame_id, x, y, yaw)

    def handle_approach_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'approach goal failed: {exc}')
            return

        if not goal_handle.accepted:
            self.finish_active_task(False, 'approach goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_approach_result)

    def handle_approach_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.finish_active_task(False, f'approach result failed: {exc}')
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.finish_active_task(
                False,
                f'approach navigation failed with status={result.status}',
            )
            return

        self.get_logger().info(
            f'approach navigation completed for task={self.active_task.sequence}'
        )
        self.request_rectangle_path(self.active_task)

    def request_rectangle_path(self, task):
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
            self.finish_active_task(False, 'coverage path service returned no path')
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
