"""高层行为树任务调度节点。"""

import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.action import ArmPreset, ArmTarget, RobotTask
from robot_interfaces.srv import (
    DetectDumpBucket,
    DetectGraspTarget,
    DumpTrash,
    GenerateCoveragePath,
    SetCameraTfPreset,
    SetGripper,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from .task_clean_region import CleanRegionTaskMixin
from .task_clean_trash import CleanTrashTaskMixin
from .task_dump_trash import DumpTrashTaskMixin
from .task_hardware import HardwareTaskMixin
from .task_models import MapPose
from .task_motion import MotionMixin
from .task_navigation import NavigationTaskMixin
from .task_ros_utils import RosTaskUtilsMixin
from .task_storage import TaskStorageMixin


SUBSCRIPTIONS = (
    ('_odom_sub', Odometry, 'odom_topic', '_handle_odom'),
    ('_amcl_pose_sub', PoseWithCovarianceStamped, 'amcl_pose_topic', '_handle_amcl_pose'),
    ('_imu_sub', Imu, 'imu_topic', '_handle_imu'),
)

SERVICE_CLIENTS = (
    ('_detect_grasp_client', DetectGraspTarget, 'detect_grasp_service'),
    ('_detect_dump_client', DetectDumpBucket, 'detect_dump_service'),
    ('_dump_pick_bucket_client', DumpTrash, 'dump_pick_bucket_service'),
    ('_dump_pour_bucket_client', DumpTrash, 'dump_pour_bucket_service'),
    ('_dump_place_bucket_client', DumpTrash, 'dump_place_bucket_service'),
    ('_dump_home_client', Trigger, 'dump_home_service'),
    ('_coverage_path_client', GenerateCoveragePath, 'coverage_path_service'),
    (
        '_coverage_controller_params_client',
        SetParameters,
        'coverage_controller_set_parameters_service',
    ),
    (
        '_scan_camera_costmap_gate_client',
        SetBool,
        'coverage_scan_camera_costmap_service',
    ),
    (
        '_coverage_local_clear_costmap_client',
        ClearEntireCostmap,
        'coverage_local_clear_costmap_service',
    ),
    (
        '_coverage_global_clear_costmap_client',
        ClearEntireCostmap,
        'coverage_global_clear_costmap_service',
    ),
    ('_gripper_client', SetGripper, 'gripper_service'),
    ('_camera_tf_preset_client', SetCameraTfPreset, 'camera_tf_preset_service'),
)

ACTION_CLIENTS = (
    ('_arm_target_client', ArmTarget, 'arm_target_action'),
    ('_arm_preset_client', ArmPreset, 'arm_preset_action'),
    ('_navigate_client', NavigateToPose, 'navigate_to_pose_action'),
    ('_navigate_through_client', NavigateThroughPoses, 'navigate_through_poses_action'),
)


class _VisualWorkGoalHandle:

    def __init__(self, task_server):
        self._task_server = task_server

    @property
    def is_cancel_requested(self):
        return (
            not self._task_server._visual_work_enabled
            or self._task_server._visual_work_yield_requested
        )

    def publish_feedback(self, feedback):
        del feedback


class RobotTaskServer(
    CleanTrashTaskMixin,
    DumpTrashTaskMixin,
    CleanRegionTaskMixin,
    NavigationTaskMixin,
    MotionMixin,
    TaskStorageMixin,
    HardwareTaskMixin,
    RosTaskUtilsMixin,
    Node,
):
    """统一接收用户任务，并调度导航、视觉、机械臂和底层硬件能力。"""

    def __init__(self):
        super().__init__('robot_task_server')

        self._declare_ros_task_parameters()
        self._declare_clean_trash_parameters()
        self._declare_dump_trash_parameters()
        self._declare_clean_region_parameters()
        self._declare_navigation_parameters()
        self._declare_hardware_parameters()
        self._declare_storage_parameters()

        callback_group = ReentrantCallbackGroup()
        self._task_lock = threading.Lock()
        self._active_goal = None
        self._stop_requested = False
        self._pause_requested = False
        self._clear_requested = False
        self._visual_work_enabled = False
        self._visual_work_busy = False
        self._visual_work_can_yield = False
        self._visual_work_yield_requested = False
        self._visual_work_pose_ready = False
        self._visual_work_goal_handle = _VisualWorkGoalHandle(self)
        self._dump_bucket_poses = {}
        self._clean_regions = {}
        self._bucket_fill_percent = {1: 0, 2: 0}
        self._latest_odom_yaw = None
        self._latest_odom_xy = None
        self._cleaning_scan_last_odom_xy = None
        self._cleaning_scan_distance_m = 0.0
        self._latest_imu_yaw = None
        self._latest_map_pose = None
        self._load_dump_bucket_poses()
        self._load_clean_regions()
        for attr, msg_type, topic_param, callback_name in SUBSCRIPTIONS:
            setattr(
                self,
                attr,
                self.create_subscription(
                    msg_type,
                    self.get_parameter(topic_param).value,
                    getattr(self, callback_name),
                    10,
                    callback_group=callback_group,
                ),
            )

        for attr, srv_type, service_param in SERVICE_CLIENTS:
            setattr(
                self,
                attr,
                self.create_client(
                    srv_type,
                    self.get_parameter(service_param).value,
                    callback_group=callback_group,
                ),
            )

        for attr, action_type, action_param in ACTION_CLIENTS:
            setattr(
                self,
                attr,
                ActionClient(
                    self,
                    action_type,
                    self.get_parameter(action_param).value,
                    callback_group=callback_group,
                ),
            )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        visual_work_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._visual_work_state_pub = self.create_publisher(
            Bool,
            self.get_parameter('visual_work_state_topic').value,
            visual_work_qos,
        )
        self._visual_work_service = self.create_service(
            SetBool,
            self.get_parameter('visual_work_service').value,
            self._handle_visual_work_enabled,
            callback_group=callback_group,
        )
        self._publish_visual_work_state()
        self._action_server = ActionServer(
            self,
            RobotTask,
            self.get_parameter('action_name').value,
            execute_callback=self._execute_task,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=callback_group,
        )
        self._visual_work_timer = self.create_timer(
            max(
                0.2,
                float(
                    self.get_parameter(
                        'coverage_detect_interval_sec'
                    ).value
                ),
            ),
            self._run_idle_visual_work_cycle,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'robot behavior tree ready: '
            f'action={self.get_parameter("action_name").value}'
        )

    def _handle_odom(self, msg):
        self._latest_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self._latest_odom_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

    def _handle_imu(self, msg):
        self._latest_imu_yaw = self._yaw_from_quaternion(msg.orientation)

    def _handle_amcl_pose(self, msg):
        pose = msg.pose.pose
        self._latest_map_pose = MapPose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=self._yaw_from_quaternion(pose.orientation),
        )

    def _handle_goal(self, goal_request):
        task_type = int(goal_request.task_type)
        if task_type not in self._known_task_types():
            self.get_logger().warning(f'rejecting unknown task_type={task_type}')
            return GoalResponse.REJECT
        if task_type in (RobotTask.Goal.TASK_STOP, RobotTask.Goal.TASK_RESUME):
            return GoalResponse.ACCEPT
        if self._visual_work_busy:
            if not self._visual_work_can_yield:
                self.get_logger().warning(
                    'rejecting task because visual work is grasping'
                )
                return GoalResponse.REJECT
            self._visual_work_yield_requested = True
            deadline = time.monotonic() + float(
                self.get_parameter('service_timeout_sec').value
            )
            while self._visual_work_busy and time.monotonic() < deadline:
                time.sleep(0.02)
            if self._visual_work_busy:
                self._visual_work_yield_requested = False
                self.get_logger().warning(
                    'rejecting task because visual work did not yield'
                )
                return GoalResponse.REJECT
        if self._active_goal is not None:
            self.get_logger().warning('rejecting task because another task is active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        del goal_handle
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _execute_task(self, goal_handle):
        task_type = int(goal_handle.request.task_type)
        if task_type in (RobotTask.Goal.TASK_STOP, RobotTask.Goal.TASK_RESUME):
            lock_acquired = self._task_lock.acquire(blocking=False)
            if not lock_acquired:
                if task_type == RobotTask.Goal.TASK_STOP:
                    self._apply_control_command(goal_handle.request.command)
                    message = 'clear requested' if self._clear_requested else 'pause requested'
                else:
                    self._pause_requested = False
                    message = 'resume requested'
                goal_handle.succeed()
                return self._result(True, message)

            try:
                if task_type == RobotTask.Goal.TASK_STOP:
                    self._apply_control_command(goal_handle.request.command)
                    goal_handle.succeed()
                    return self._result(True, 'robot stopped')

                self._pause_requested = False
                goal_handle.succeed()
                return self._result(True, 'resume accepted')
            finally:
                self._task_lock.release()

        if not self._task_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, 'another task is already running')

        self._visual_work_yield_requested = False
        self._active_goal = goal_handle
        goal = goal_handle.request
        task_name = self._task_name(task_type)
        if task_type not in (RobotTask.Goal.TASK_STOP, RobotTask.Goal.TASK_RESUME):
            self._stop_requested = False
            self._clear_requested = False
        self._feedback(goal_handle, f'{task_name}: started', 0.0)

        try:
            success, message = self._dispatch_task(goal_handle, goal)
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            success = False
            message = f'{task_name} failed: {exc}'
        finally:
            self._publish_stop()
            self._close_gripper_for_cleanup(task_name)
            self._stop_requested = False
            self._pause_requested = False
            self._clear_requested = False
            self._visual_work_yield_requested = False
            self._active_goal = None
            self._task_lock.release()

        result = self._result(success, message)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = f'{task_name} canceled'
        elif success:
            self._feedback(goal_handle, f'{task_name}: completed', 1.0)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _handle_visual_work_enabled(self, request, response):
        self._set_visual_work_enabled(bool(request.data))
        response.success = True
        response.message = (
            'visual work enabled'
            if self._visual_work_enabled
            else 'visual work disabled'
        )
        return response

    def _set_visual_work_enabled(self, enabled):
        enabled = bool(enabled)
        was_enabled = self._visual_work_enabled
        if enabled != self._visual_work_enabled:
            self._visual_work_pose_ready = False
        self._visual_work_enabled = enabled
        self._publish_visual_work_state()
        self.get_logger().info(
            f'visual work enabled={self._visual_work_enabled}'
        )
        if enabled and not was_enabled:
            self._set_coverage_scan_camera_costmap_enabled(None, False)
        if (
            was_enabled
            and not enabled
            and self._active_goal is None
            and not self._visual_work_busy
        ):
            self._restore_visual_work_cleanup()

    def _restore_visual_work_cleanup(self):
        self._publish_stop()
        self._set_coverage_scan_camera_costmap_enabled(None, True)
        self._restore_arm_mapping_for_cleanup(
            'visual_work: restore arm mapping pose'
        )

    def _publish_visual_work_state(self):
        msg = Bool()
        msg.data = bool(self._visual_work_enabled)
        self._visual_work_state_pub.publish(msg)

    def _run_idle_visual_work_cycle(self):
        if (
            not self._visual_work_enabled
            or self._active_goal is not None
            or self._visual_work_busy
        ):
            return
        if not self._task_lock.acquire(blocking=False):
            return

        self._visual_work_busy = True
        self._visual_work_can_yield = True
        goal_handle = self._visual_work_goal_handle
        try:
            if goal_handle.is_cancel_requested or self._active_goal is not None:
                return

            if not self._visual_work_pose_ready:
                self._feedback(
                    goal_handle,
                    'visual_work: arm center',
                    0.05,
                )
                self._send_arm_preset(
                    goal_handle,
                    ArmPreset.Goal.ARM_CENTER,
                )
                if goal_handle.is_cancel_requested:
                    return
                self._visual_work_pose_ready = True

            target = self._detect_reachable_target(goal_handle)
            if target is None or goal_handle.is_cancel_requested:
                return

            self._visual_work_can_yield = False
            self.get_logger().info(
                f'visual_work grasping {target.class_name}'
            )
            bucket_id, bucket_level = self._grasp_target(
                goal_handle,
                target,
            )
            success, message = self._maybe_auto_dump_full_bucket(
                goal_handle,
                bucket_id,
                bucket_level,
            )
            if not success:
                self.get_logger().warning(
                    f'visual work auto dump failed: {message}'
                )
        except Exception as exc:
            self.get_logger().warning(f'visual work cycle failed: {exc}')
        finally:
            if not self._visual_work_enabled:
                self._restore_visual_work_cleanup()
            self._visual_work_can_yield = False
            self._visual_work_busy = False
            self._task_lock.release()

    def _dispatch_task(self, goal_handle, goal):
        task_type = int(goal.task_type)
        if task_type == RobotTask.Goal.TASK_CLEAN_TRASH:
            return self._run_clean_trash_tree(goal_handle)
        if task_type == RobotTask.Goal.TASK_DUMP_TRASH:
            return self._run_dump_trash_tree(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_CLEAN_REGION:
            return self._run_clean_region_tree(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_NAVIGATE_TO:
            return self._run_navigate_tree(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_NAVIGATE_PATH:
            return self._run_navigate_path_tree(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_SET_DUMP_POSE:
            return self._run_set_dump_pose(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_SET_CLEAN_REGION:
            return self._run_set_clean_region(goal_handle, goal)
        if task_type == RobotTask.Goal.TASK_STOP:
            self._apply_control_command(goal.command)
            return True, 'robot stopped'
        if task_type == RobotTask.Goal.TASK_RESUME:
            self._pause_requested = False
            return True, 'resume accepted'
        return False, f'unsupported task_type={task_type}'

    def _apply_control_command(self, command):
        command = str(command).strip().lower()
        self._set_visual_work_enabled(False)
        if command == 'pause':
            self._pause_requested = True
            self._clear_requested = False
            self._stop_requested = False
        else:
            self._clear_requested = True
            self._pause_requested = False
            self._stop_requested = True
        self._publish_stop()

def main(args=None):
    """启动行为树任务调度节点。"""
    rclpy.init(args=args)
    node = RobotTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
