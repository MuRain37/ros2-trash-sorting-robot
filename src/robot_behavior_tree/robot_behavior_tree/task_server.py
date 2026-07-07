"""高层行为树任务调度节点。"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import ArmPreset, ArmTarget, RobotTask
from robot_interfaces.srv import (
    DetectDumpBucket,
    DetectGraspTarget,
    DumpTrash,
    GenerateCoveragePath,
    SetCameraTfPreset,
    SetGripper,
)
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger


@dataclass
class GraspTarget:
    """视觉识别返回的抓取目标。"""

    class_name: str
    confidence: float
    target_x_cm: float
    target_y_cm: float
    target_z_cm: float
    end_angle: float
    gripper_rotation: float
    gripper_close_ratio: float
    bucket_id: int


@dataclass
class DumpBucketPose:
    """地图上标定的卸载桶位置。"""

    bucket_id: int
    x: float
    y: float
    yaw: float
    frame_id: str


@dataclass
class CleanRegion:
    """行为树保存的清扫区域。"""

    region_id: int
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    frame_id: str


@dataclass
class DumpBucketDetection:
    """卸载桶视觉检测结果：中心偏移、深度和是否到达倾倒距离。"""

    detected: bool
    within_depth: bool
    mask_pixels: int
    mean_depth_m: float
    offset_x_px: float
    offset_x_ratio: float
    message: str


@dataclass
class MapPose:
    """机器人当前 map 位姿。"""

    x: float
    y: float
    yaw: float


@dataclass
class ScanCameraObstacle:
    """scan_camera 最近看到的前方近距离障碍。"""

    range_m: float
    bearing_rad: float
    seen_at: float


class RobotTaskServer(Node):
    """统一接收用户任务，并调度导航、视觉、机械臂和底层硬件能力。"""

    def __init__(self):
        super().__init__('robot_task_server')

        self.declare_parameter('action_name', '/robot_task')
        self.declare_parameter('detect_grasp_service', 'vision_grasp/detect')
        self.declare_parameter('detect_dump_service', 'detect_dump_bucket')
        self.declare_parameter('dump_pick_bucket_service', 'dump_pick_bucket')
        self.declare_parameter('dump_pour_bucket_service', 'dump_pour_bucket')
        self.declare_parameter('dump_place_bucket_service', 'dump_place_bucket')
        self.declare_parameter('dump_home_service', 'dump_home')
        self.declare_parameter('coverage_path_service', 'generate_coverage_path')
        self.declare_parameter('arm_target_action', '/arm_target')
        self.declare_parameter('arm_preset_action', '/arm_preset')
        self.declare_parameter(
            'camera_tf_preset_service',
            '/real_camera_static_tf/set_preset',
        )
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', 'navigate_through_poses')
        self.declare_parameter('gripper_service', 'arm_gripper')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_camera_topic', '/scan_camera')
        self.declare_parameter('service_timeout_sec', 3.0)
        self.declare_parameter('action_timeout_sec', 45.0)
        self.declare_parameter('navigation_timeout_sec', 120.0)
        self.declare_parameter('dump_timeout_sec', 240.0)
        self.declare_parameter('clean_region_timeout_sec', 360.0)
        self.declare_parameter('dump_backup_distance_m', 0.30)
        self.declare_parameter('dump_backup_linear_speed', 0.05)
        self.declare_parameter('dump_forward_distance_m', 0.15)
        self.declare_parameter('dump_forward_linear_speed', 0.05)
        # <= 0 means keep scanning/grasping until TASK_STOP or action cancel.
        self.declare_parameter('clean_max_objects', 0)
        self.declare_parameter('detect_retry_count', 3)
        self.declare_parameter('approach_max_attempts', 3)
        self.declare_parameter('approach_x_threshold_cm', 32.0)
        self.declare_parameter('approach_y_threshold_cm', 13.0)
        self.declare_parameter('approach_x_tolerance_cm', 2.0)
        self.declare_parameter('approach_target_x_cm', 20.0)
        self.declare_parameter('approach_max_step_m', 0.04)
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('approach_y_angular_speed', 0.27)
        self.declare_parameter('approach_y_max_rotate_rad', 0.35)
        self.declare_parameter('coverage_detect_interval_sec', 0.3)
        self.declare_parameter('coverage_scan_forward_step_m', 0.30)
        self.declare_parameter(
            'coverage_controller_set_parameters_service',
            '/controller_server/set_parameters',
        )
        self.declare_parameter('coverage_linear_speed', 0.12)
        self.declare_parameter('coverage_restore_linear_speed', 0.24)
        self.declare_parameter('arm_target_duration_sec', 2.0)
        self.declare_parameter('gripper_duration_ms', 500.0)
        self.declare_parameter('gripper_wait_sec', 1.0)
        self.declare_parameter('require_dump_bucket_detect', True)
        self.declare_parameter('dump_visual_align_required', False)
        self.declare_parameter('dump_approach_distance_m', 1.15)
        self.declare_parameter('dump_align_max_attempts', 8)
        self.declare_parameter('dump_align_target_depth_m', 0.35)
        self.declare_parameter('dump_align_depth_tolerance_m', 0.04)
        self.declare_parameter('dump_align_step_m', 0.05)
        self.declare_parameter('dump_align_linear_speed', 0.05)
        self.declare_parameter('dump_rotate_angular_speed', 0.8)
        self.declare_parameter('dump_center_tolerance_ratio', 0.04)
        self.declare_parameter('dump_center_max_attempts',8)
        self.declare_parameter('dump_center_angular_speed', 0.2)
        self.declare_parameter('dump_center_step_sec', 0.30)
        self.declare_parameter('dump_center_max_rotate_rad', 0.75)
        self.declare_parameter('dump_align_max_forward_step_m', 0.08)
        self.declare_parameter('dump_rotate_tolerance_rad', 0.04)
        self.declare_parameter('dump_rotate_timeout_sec', 12.0)
        self.declare_parameter('dump_scan_camera_bucket_check_enabled', True)
        self.declare_parameter('dump_scan_camera_obstacle_min_range_m', 0.05)
        self.declare_parameter('dump_scan_camera_obstacle_max_range_m', 1.4)
        self.declare_parameter('dump_scan_camera_front_angle_rad', 0.8)
        self.declare_parameter('dump_scan_camera_obstacle_stale_sec', 0.6)
        self.declare_parameter('dump_scan_camera_check_interval_sec', 0.5)
        self.declare_parameter('dump_scan_camera_bucket_map_tolerance_m', 0.75)
        self.declare_parameter('dump_camera_horizontal_fov_rad', 1.21)
        self.declare_parameter('dump_camera_lateral_offset_m', 0.033)
        self.declare_parameter(
            'dump_pose_store_path',
            '/home/xjw/ros2_robot/src/robot_behavior_tree/config/dump_bucket_poses.json',
        )
        self.declare_parameter(
            'clean_region_store_path',
            '/home/xjw/ros2_robot/src/robot_behavior_tree/config/clean_regions.json',
        )

        callback_group = ReentrantCallbackGroup()
        self._task_lock = threading.Lock()
        self._active_goal = None
        self._stop_requested = False
        self._pause_requested = False
        self._clear_requested = False
        self._dump_bucket_poses = {}
        self._clean_regions = {}
        self._bucket_fill_percent = {1: 0, 2: 0}
        self._latest_odom_yaw = None
        self._latest_imu_yaw = None
        self._latest_map_pose = None
        self._latest_scan_camera_obstacle = None
        self._load_dump_bucket_poses()
        self._load_clean_regions()
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._handle_odom,
            10,
            callback_group=callback_group,
        )
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_pose_topic').value,
            self._handle_amcl_pose,
            10,
            callback_group=callback_group,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._handle_imu,
            10,
            callback_group=callback_group,
        )
        self._scan_camera_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_camera_topic').value,
            self._handle_scan_camera,
            10,
            callback_group=callback_group,
        )

        self._detect_grasp_client = self.create_client(
            DetectGraspTarget,
            self.get_parameter('detect_grasp_service').value,
            callback_group=callback_group,
        )
        self._detect_dump_client = self.create_client(
            DetectDumpBucket,
            self.get_parameter('detect_dump_service').value,
            callback_group=callback_group,
        )
        self._dump_pick_bucket_client = self.create_client(
            DumpTrash,
            self.get_parameter('dump_pick_bucket_service').value,
            callback_group=callback_group,
        )
        self._dump_pour_bucket_client = self.create_client(
            DumpTrash,
            self.get_parameter('dump_pour_bucket_service').value,
            callback_group=callback_group,
        )
        self._dump_place_bucket_client = self.create_client(
            DumpTrash,
            self.get_parameter('dump_place_bucket_service').value,
            callback_group=callback_group,
        )
        self._dump_home_client = self.create_client(
            Trigger,
            self.get_parameter('dump_home_service').value,
            callback_group=callback_group,
        )
        self._coverage_path_client = self.create_client(
            GenerateCoveragePath,
            self.get_parameter('coverage_path_service').value,
            callback_group=callback_group,
        )
        self._coverage_controller_params_client = self.create_client(
            SetParameters,
            self.get_parameter('coverage_controller_set_parameters_service').value,
            callback_group=callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            self.get_parameter('gripper_service').value,
            callback_group=callback_group,
        )
        self._camera_tf_preset_client = self.create_client(
            SetCameraTfPreset,
            self.get_parameter('camera_tf_preset_service').value,
            callback_group=callback_group,
        )
        self._arm_target_client = ActionClient(
            self,
            ArmTarget,
            self.get_parameter('arm_target_action').value,
            callback_group=callback_group,
        )
        self._arm_preset_client = ActionClient(
            self,
            ArmPreset,
            self.get_parameter('arm_preset_action').value,
            callback_group=callback_group,
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_to_pose_action').value,
            callback_group=callback_group,
        )
        self._navigate_through_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.get_parameter('navigate_through_poses_action').value,
            callback_group=callback_group,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self._action_server = ActionServer(
            self,
            RobotTask,
            self.get_parameter('action_name').value,
            execute_callback=self._execute_task,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'robot behavior tree ready: '
            f'action={self.get_parameter("action_name").value}'
        )

    def _handle_odom(self, msg):
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

    def _handle_scan_camera(self, msg):
        if not bool(
            self.get_parameter('dump_scan_camera_bucket_check_enabled').value
        ):
            return

        min_range = max(
            0.0,
            float(self.get_parameter('dump_scan_camera_obstacle_min_range_m').value),
        )
        max_range = max(
            min_range,
            float(self.get_parameter('dump_scan_camera_obstacle_max_range_m').value),
        )
        front_angle = abs(
            float(self.get_parameter('dump_scan_camera_front_angle_rad').value)
        )

        best_range = None
        best_angle = None
        for index, range_m in enumerate(msg.ranges):
            if not math.isfinite(range_m):
                continue
            range_m = float(range_m)
            if range_m < min_range or range_m > max_range:
                continue
            angle = float(msg.angle_min + index * msg.angle_increment)
            if abs(angle) > front_angle:
                continue
            if best_range is None or range_m < best_range:
                best_range = range_m
                best_angle = angle

        if best_range is None:
            return
        self._latest_scan_camera_obstacle = ScanCameraObstacle(
            range_m=best_range,
            bearing_rad=best_angle,
            seen_at=time.monotonic(),
        )

    def _handle_goal(self, goal_request):
        task_type = int(goal_request.task_type)
        if task_type not in self._known_task_types():
            self.get_logger().warning(f'rejecting unknown task_type={task_type}')
            return GoalResponse.REJECT
        if task_type in (RobotTask.Goal.TASK_STOP, RobotTask.Goal.TASK_RESUME):
            return GoalResponse.ACCEPT
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
        if command == 'pause':
            self._pause_requested = True
            self._clear_requested = False
            self._stop_requested = False
        else:
            self._clear_requested = True
            self._pause_requested = False
            self._stop_requested = True
        self._publish_stop()

    def _run_clean_trash_tree(self, goal_handle):
        """清扫垃圾：识别 -> 必要时前进 -> MoveIt 抓取 -> 投桶 -> 归中。"""
        self._feedback(goal_handle, 'clean_trash: arm center', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)

        picked = 0
        scan_index = 0
        max_objects = int(self.get_parameter('clean_max_objects').value)
        while max_objects <= 0 or picked < max_objects:
            if self._is_canceling(goal_handle):
                return False, 'clean trash canceled'

            scan_index += 1
            self._feedback(
                goal_handle,
                f'clean_trash: detecting target {scan_index}',
                0.10,
            )
            target = self._detect_reachable_target(goal_handle)
            if target is None:
                if max_objects > 0:
                    if picked == 0:
                        return False, 'no reachable trash target detected'
                    return True, f'clean trash completed: picked={picked}'
                self._feedback(
                    goal_handle,
                    'clean_trash: no target, scanning again',
                    0.20,
                )
                time.sleep(0.5)
                continue

            self._feedback(
                goal_handle,
                f'clean_trash: grasping {target.class_name}',
                0.35,
            )
            bucket_id, bucket_level = self._grasp_target(goal_handle, target)
            picked += 1
            success, message = self._maybe_auto_dump_full_bucket(
                goal_handle,
                bucket_id,
                bucket_level,
            )
            if not success:
                return False, message

        return True, f'clean trash completed: picked={picked}'

    def _detect_reachable_target(self, goal_handle):
        last_target = None
        max_attempts = int(self.get_parameter('approach_max_attempts').value)
        for approach_index in range(
            max_attempts + 1
        ):
            target = self._detect_grasp_target(goal_handle)
            if target is None:
                return None
            last_target = target

            x_threshold_cm = float(
                self.get_parameter('approach_x_threshold_cm').value
            )
            x_tolerance_cm = max(
                0.0,
                float(self.get_parameter('approach_x_tolerance_cm').value),
            )
            y_threshold_cm = abs(
                float(self.get_parameter('approach_y_threshold_cm').value)
            )
            if (
                y_threshold_cm > 0.0
                and abs(target.target_y_cm) > y_threshold_cm
            ):
                if approach_index >= max_attempts:
                    break

                self._feedback(
                    goal_handle,
                    'clean_trash: target lateral offset, rotating base',
                    0.20,
                )
                self._rotate_toward_target_y(goal_handle, target)
                continue

            if target.target_x_cm <= x_threshold_cm:
                return target
            if (
                approach_index >= max_attempts
                and target.target_x_cm <= x_threshold_cm + x_tolerance_cm
            ):
                self.get_logger().warning(
                    'target slightly beyond approach threshold; grasp anyway: '
                    f'x={target.target_x_cm:.1f} cm, '
                    f'threshold={x_threshold_cm:.1f} cm, '
                    f'tolerance={x_tolerance_cm:.1f} cm, '
                    f'y={target.target_y_cm:.1f} cm'
                )
                return target

            if approach_index >= max_attempts:
                break

            self._feedback(
                goal_handle,
                'clean_trash: target is far, moving base forward',
                0.20,
            )
            self._drive_forward_toward_target(goal_handle, target.target_x_cm)

        self.get_logger().warning(
            'target still beyond approach threshold: '
            f'x={last_target.target_x_cm:.1f} cm, '
            f'y={last_target.target_y_cm:.1f} cm'
        )
        return None

    def _detect_grasp_target(self, goal_handle):
        retries = int(self.get_parameter('detect_retry_count').value)
        for attempt in range(max(1, retries)):
            if self._is_canceling(goal_handle):
                return None

            request = DetectGraspTarget.Request()
            request.force_refresh = True
            response = self._call_service(
                self._detect_grasp_client,
                request,
                timeout_sec=float(
                    self.get_parameter('service_timeout_sec').value
                ),
                goal_handle=goal_handle,
                label='detect grasp target',
            )
            if response is not None and response.success:
                return GraspTarget(
                    class_name=str(response.class_name),
                    confidence=float(response.confidence),
                    target_x_cm=float(response.target_x_cm),
                    target_y_cm=float(response.target_y_cm),
                    target_z_cm=float(response.target_z_cm),
                    end_angle=float(response.end_angle),
                    gripper_rotation=float(response.gripper_rotation),
                    gripper_close_ratio=float(response.gripper_close_ratio),
                    bucket_id=int(response.bucket_id),
                )
            if response is None:
                self.get_logger().warning(
                    'detect grasp target attempt '
                    f'{attempt + 1}/{max(1, retries)} returned no response'
                )
            else:
                self.get_logger().warning(
                    'detect grasp target attempt '
                    f'{attempt + 1}/{max(1, retries)} failed: '
                    f'{response.message}'
                )
            time.sleep(0.15)
            self._feedback(
                goal_handle,
                f'clean_trash: detection retry {attempt + 1}',
                0.15,
            )
        return None

    def _drive_forward_toward_target(self, goal_handle, target_x_cm):
        target_x = float(self.get_parameter('approach_target_x_cm').value)
        max_step = float(self.get_parameter('approach_max_step_m').value)
        speed = abs(float(self.get_parameter('approach_linear_speed').value))
        if speed <= 0.0:
            return

        distance_m = max(0.0, (target_x_cm - target_x) / 100.0)
        distance_m = min(distance_m, max_step)
        duration = distance_m / speed

        deadline = time.monotonic() + duration
        msg = Twist()
        msg.linear.x = speed
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()
        time.sleep(0.2)

    def _rotate_toward_target_y(self, goal_handle, target):
        speed = abs(float(self.get_parameter('approach_y_angular_speed').value))
        max_rotate = abs(
            float(self.get_parameter('approach_y_max_rotate_rad').value)
        )
        if speed <= 0.0 or max_rotate <= 0.0:
            return

        forward_cm = max(1.0, float(target.target_x_cm))
        rotate_rad = math.atan2(float(target.target_y_cm), forward_cm)
        rotate_rad = max(-max_rotate, min(max_rotate, rotate_rad))
        if abs(rotate_rad) <= 0.001:
            return

        msg = Twist()
        msg.angular.z = speed if rotate_rad > 0.0 else -speed
        self.get_logger().info(
            'grasp lateral align rotate: '
            f'x={target.target_x_cm:.1f} cm, '
            f'y={target.target_y_cm:.1f} cm, '
            f'rotate={rotate_rad:.3f} rad, '
            f'angular_z={msg.angular.z:.3f}'
        )
        self._publish_twist_for(goal_handle, msg, abs(rotate_rad) / speed)
        time.sleep(0.2)

    def _grasp_target(self, goal_handle, target):
        duration_ms = float(self.get_parameter('gripper_duration_ms').value)
        wait_sec = float(self.get_parameter('gripper_wait_sec').value)

        self._set_gripper(goal_handle, 1.0, duration_ms, 'open gripper')
        self._send_arm_target(goal_handle, target)
        self._set_gripper(
            goal_handle,
            target.gripper_close_ratio,
            duration_ms,
            'close gripper',
        )
        time.sleep(wait_sec)

        bucket_id = target.bucket_id if target.bucket_id in (1, 2) else 1
        self._feedback(goal_handle, f'clean_trash: move to bucket {bucket_id}', 0.70)
        self._send_arm_preset(goal_handle, bucket_id)
        self._set_gripper(goal_handle, 1.0, duration_ms, 'release object')
        time.sleep(wait_sec)

        bucket_level = self._increment_bucket_fill(goal_handle, bucket_id)

        self._feedback(goal_handle, 'clean_trash: arm center', 0.90)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        self._set_gripper(goal_handle, 0.0, duration_ms, 'close gripper idle')
        return bucket_id, bucket_level

    def _increment_bucket_fill(self, goal_handle, bucket_id):
        bucket_id = bucket_id if bucket_id in (1, 2) else 1
        level = min(100, int(self._bucket_fill_percent.get(bucket_id, 0)) + 10)
        self._bucket_fill_percent[bucket_id] = level
        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level={level}',
            0.89,
        )
        return level

    def _reset_bucket_fill(self, goal_handle, bucket_id, progress=0.99):
        bucket_id = bucket_id if bucket_id in (1, 2) else 1
        self._bucket_fill_percent[bucket_id] = 0
        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level=0 dumped',
            progress,
        )

    def _maybe_auto_dump_full_bucket(self, goal_handle, bucket_id, bucket_level):
        if bucket_level < 100:
            return True, 'bucket is not full'

        self._load_dump_bucket_poses()
        if bucket_id not in self._dump_bucket_poses:
            self._feedback(
                goal_handle,
                f'bucket_fill: bucket_id={bucket_id} level=100 no_dump_pose',
                0.90,
            )
            return True, f'bucket {bucket_id} full but dump pose is not set'

        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level=100 auto_dump_start',
            0.90,
        )
        success, message = self._dump_single_bucket(
            goal_handle,
            bucket_id,
            base_progress=0.90,
            span=0.09,
        )
        if not success:
            return False, message

        self._reset_bucket_fill(goal_handle, bucket_id)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        return True, message

    def _run_dump_trash_tree(self, goal_handle, goal):
        self._load_dump_bucket_poses()
        raw_bucket_id = int(goal.bucket_id)
        bucket_ids = [raw_bucket_id] if raw_bucket_id in (1, 2) else [1, 2]
        for index, bucket_id in enumerate(bucket_ids):
            success, message = self._dump_single_bucket(
                goal_handle,
                bucket_id,
                base_progress=index / max(1, len(bucket_ids)),
                span=1.0 / max(1, len(bucket_ids)),
            )
            if not success:
                return False, message
            self._reset_bucket_fill(
                goal_handle,
                bucket_id,
                progress=(index + 1) / max(1, len(bucket_ids)),
            )
        if len(bucket_ids) == 1:
            return True, f'dump bucket {bucket_ids[0]} completed'
        return True, 'dump buckets 1 and 2 completed'

    def _dump_single_bucket(self, goal_handle, bucket_id, base_progress=0.0, span=1.0):
        pose = self._dump_bucket_poses.get(bucket_id)
        if pose is None:
            return False, f'dump bucket {bucket_id} pose is not set'

        self._feedback(
            goal_handle,
            f'dump_trash: arm mapping pose for bucket {bucket_id}',
            base_progress + span * 0.05,
        )
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)

        self._feedback(
            goal_handle,
            f'dump_trash: navigate to bucket {bucket_id} front',
            base_progress + span * 0.10,
        )
        nav_result, nav_message = self._navigate_to_dump_front(goal_handle, pose)
        if not nav_result:
            return False, nav_message

        if bool(self.get_parameter('require_dump_bucket_detect').value):
            self._feedback(
                goal_handle,
                f'dump_trash: center bucket {bucket_id}',
                base_progress + span * 0.45,
            )
            if not self._center_dump_bucket(goal_handle):
                message = f'dump bucket {bucket_id} visual center align failed'
                if bool(self.get_parameter('dump_visual_align_required').value):
                    return False, message
                self.get_logger().warning(f'{message}; continue to dump')

        self._feedback(
            goal_handle,
            f'dump_trash: rotate before dumping bucket {bucket_id}',
            base_progress + span * 0.65,
        )
        self._rotate_in_place_180(goal_handle)

        self._feedback(
            goal_handle,
            f'dump_trash: pick onboard bucket {bucket_id}',
            base_progress + span * 0.76,
        )
        success, message = self._call_dump_bucket_service(
            goal_handle,
            self._dump_pick_bucket_client,
            bucket_id,
            f'pick onboard bucket {bucket_id}',
        )
        if not success:
            return False, message

        self._feedback(
            goal_handle,
            f'dump_trash: backup bucket {bucket_id}',
            base_progress + span * 0.80,
        )
        self._drive_backward_step(
            goal_handle,
            float(self.get_parameter('dump_backup_distance_m').value),
            float(self.get_parameter('dump_backup_linear_speed').value),
        )

        self._feedback(
            goal_handle,
            f'dump_trash: pour bucket {bucket_id}',
            base_progress + span * 0.84,
        )
        success, message = self._call_dump_bucket_service(
            goal_handle,
            self._dump_pour_bucket_client,
            bucket_id,
            f'pour onboard bucket {bucket_id}',
        )
        if not success:
            return False, message

        self._feedback(
            goal_handle,
            f'dump_trash: place bucket {bucket_id}',
            base_progress + span * 0.88,
        )
        success, message = self._call_dump_bucket_service(
            goal_handle,
            self._dump_place_bucket_client,
            bucket_id,
            f'place onboard bucket {bucket_id}',
        )
        if not success:
            return False, message

        self._feedback(
            goal_handle,
            f'dump_trash: move forward after bucket {bucket_id}',
            base_progress + span * 0.92,
        )
        self._drive_forward_step(
            goal_handle,
            float(self.get_parameter('dump_forward_distance_m').value),
            float(self.get_parameter('dump_forward_linear_speed').value),
        )

        self._feedback(
            goal_handle,
            'dump_trash: home dump mechanism',
            base_progress + span * 0.96,
        )
        return self._call_dump_trigger(
            goal_handle,
            self._dump_home_client,
            'home dump mechanism',
        )

    def _run_set_dump_pose(self, goal_handle, goal):
        bucket_id = int(goal.bucket_id)
        if bucket_id not in (1, 2):
            return False, 'dump bucket id must be 1 or 2'
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
        pose = DumpBucketPose(
            bucket_id=bucket_id,
            x=float(goal.x),
            y=float(goal.y),
            yaw=float(goal.yaw),
            frame_id=str(frame_id),
        )
        self._dump_bucket_poses[bucket_id] = pose
        self._save_dump_bucket_poses()
        self._feedback(goal_handle, f'set_dump_pose: bucket {bucket_id}', 1.0)
        return (
            True,
            f'dump bucket {bucket_id} pose set: '
            f'({pose.x:.3f}, {pose.y:.3f}, yaw={pose.yaw:.3f})',
        )

    def _run_clean_region_tree(self, goal_handle, goal):
        self._feedback(goal_handle, 'clean_region: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        region, error = self._resolve_clean_region(goal)
        if region is None:
            return False, error

        self._feedback(goal_handle, 'clean_region: generate start path', 0.10)
        preview_path, message = self._generate_coverage_path(goal_handle, region)
        if preview_path is None:
            return False, message

        start_pose = preview_path.poses[0]
        self._feedback(goal_handle, 'clean_region: navigate to start pose', 0.15)
        start_result = self._navigate_pose_with_visual_interrupt(
            goal_handle,
            start_pose,
            'clean region start pose',
            detect_enabled=False,
        )
        if start_result is None:
            return False, 'clean region start pose canceled'

        self._feedback(goal_handle, 'clean_region: arm center for coverage', 0.20)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)

        self._set_coverage_linear_speed(
            goal_handle,
            float(self.get_parameter('coverage_linear_speed').value),
            'set coverage slow speed',
        )
        try:
            picked = self._run_clean_region_scan_loop(goal_handle, region)
            if picked is None:
                return False, 'clean region canceled'
        finally:
            self._set_coverage_linear_speed(
                None,
                float(self.get_parameter('coverage_restore_linear_speed').value),
                'restore coverage normal speed',
        )
        return True, f'clean region completed: picked={picked}'

    def _run_clean_region_scan_loop(self, goal_handle, region):
        picked = 0
        timeout_sec = max(
            1.0,
            float(self.get_parameter('clean_region_timeout_sec').value),
        )
        deadline = time.monotonic() + timeout_sec
        forward_step_m = max(
            0.0,
            float(self.get_parameter('coverage_scan_forward_step_m').value),
        )

        while time.monotonic() < deadline:
            if self._is_canceling_without_pause(goal_handle):
                return None
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                return None

            self._feedback(goal_handle, 'clean_region: detect at current pose', 0.30)
            target = self._detect_reachable_target(goal_handle)
            if target is not None:
                self._feedback(
                    goal_handle,
                    f'clean_region: grasping {target.class_name}',
                    0.45,
                )
                bucket_id, bucket_level = self._grasp_target(goal_handle, target)
                picked += 1
                success, message = self._maybe_auto_dump_full_bucket(
                    goal_handle,
                    bucket_id,
                    bucket_level,
                )
                if not success:
                    raise RuntimeError(message)

                if self._is_canceling(goal_handle):
                    return None
                self._feedback(goal_handle, 'clean_region: arm center after grasp', 0.60)
                self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
                continue

            current_xy = self._current_map_xy()
            if current_xy is None:
                return picked

            path, message = self._generate_coverage_path(
                goal_handle,
                region,
                start_position=current_xy,
            )
            if path is None:
                raise RuntimeError(message)

            next_pose = self._next_coverage_pose(path, forward_step_m)
            if next_pose is None:
                return picked

            self._feedback(goal_handle, 'clean_region: move forward to next scan', 0.70)
            result = self._navigate_pose_with_visual_interrupt(
                goal_handle,
                next_pose,
                'clean region next scan pose',
                detect_enabled=False,
            )
            if result is None:
                return None

        return picked

    def _next_coverage_pose(self, path, step_m):
        if not path.poses:
            return None
        if step_m <= 0.0:
            return path.poses[0]

        previous = path.poses[0]
        travelled = 0.0
        for pose in path.poses[1:]:
            dx = pose.pose.position.x - previous.pose.position.x
            dy = pose.pose.position.y - previous.pose.position.y
            segment = math.hypot(dx, dy)
            if segment <= 1e-6:
                previous = pose
                continue
            if travelled + segment >= step_m:
                ratio = (step_m - travelled) / segment
                next_pose = PoseStamped()
                next_pose.header = pose.header
                next_pose.pose.position.x = (
                    previous.pose.position.x + dx * ratio
                )
                next_pose.pose.position.y = (
                    previous.pose.position.y + dy * ratio
                )
                next_pose.pose.position.z = 0.0
                next_pose.pose.orientation = pose.pose.orientation
                return next_pose
            travelled += segment
            previous = pose
        return None

    def _generate_coverage_path(self, goal_handle, region, start_position=None):
        request = GenerateCoveragePath.Request()
        request.min_x = region.min_x
        request.min_y = region.min_y
        request.max_x = region.max_x
        request.max_y = region.max_y
        request.frame_id = region.frame_id
        if start_position is not None:
            request.use_start = True
            request.start_x = float(start_position[0])
            request.start_y = float(start_position[1])
        response = self._call_service(
            self._coverage_path_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label='generate coverage path',
        )
        if response is None:
            return None, 'coverage path service unavailable or timed out'
        if not response.success:
            return None, response.message
        if not response.coverage_path.poses:
            return None, 'coverage path service returned no path'
        return response.coverage_path, response.message

    def _run_coverage_path_with_grasp(self, goal_handle, path):
        picked = 0
        total = len(path.poses)
        for index, pose in enumerate(path.poses):
            if self._is_canceling(goal_handle):
                return None

            self._feedback(
                goal_handle,
                f'clean_region: coverage waypoint {index + 1}/{total}',
                0.20 + 0.70 * index / max(1, total),
            )
            target = self._navigate_pose_with_visual_interrupt(
                goal_handle,
                pose,
                f'coverage waypoint {index + 1}',
            )
            if not target:
                continue

            self._feedback(
                goal_handle,
                'clean_region: reacquire stopped target',
                0.40,
            )
            target = self._detect_reachable_target(goal_handle)
            if target is None:
                self.get_logger().warning(
                    'clean_region visual interrupt had no reachable target after stop'
                )
                continue

            self._feedback(
                goal_handle,
                f'clean_region: grasping {target.class_name}',
                0.45,
            )
            bucket_id, bucket_level = self._grasp_target(goal_handle, target)
            picked += 1
            success, message = self._maybe_auto_dump_full_bucket(
                goal_handle,
                bucket_id,
                bucket_level,
            )
            if not success:
                raise RuntimeError(message)

            if self._is_canceling(goal_handle):
                return None
            self._feedback(goal_handle, 'clean_region: arm center before resume', 0.60)
            self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)

            if self._is_canceling(goal_handle):
                return None
            self._feedback(
                goal_handle,
                f'clean_region: return to waypoint {index + 1}/{total}',
                0.62,
            )
            self._navigate_pose_with_visual_interrupt(
                goal_handle,
                pose,
                f'return coverage waypoint {index + 1}',
                detect_enabled=False,
            )

        return picked

    def _navigate_pose_with_visual_interrupt(
        self,
        goal_handle,
        pose,
        label,
        detect_enabled=True,
    ):
        if self._is_canceling(goal_handle):
            return None
        if not self._navigate_client.wait_for_server(
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        ):
            raise RuntimeError(f'{label} action server not available')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        send_future = self._navigate_client.send_goal_async(nav_goal)
        if not self._wait_future(
            send_future,
            float(self.get_parameter('service_timeout_sec').value),
            goal_handle,
        ):
            raise RuntimeError(f'{label} goal response timed out')

        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            raise RuntimeError(f'{label} goal rejected')

        result_future = nav_handle.get_result_async()
        next_detect = time.monotonic()
        detect_interval = max(
            0.2,
            float(self.get_parameter('coverage_detect_interval_sec').value),
        )
        deadline = (
            time.monotonic()
            + self._navigation_timeout_sec()
        )

        while rclpy.ok() and not result_future.done():
            if self._clear_requested or self._stop_requested or (
                goal_handle is not None and goal_handle.is_cancel_requested
            ):
                self._cancel_action_goal(nav_handle)
                return None
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                self._cancel_action_goal(nav_handle)
                return None

            if time.monotonic() >= deadline:
                self._cancel_action_goal(nav_handle)
                raise RuntimeError(f'{label} result timed out')

            if detect_enabled and time.monotonic() >= next_detect:
                target = self._detect_grasp_target(goal_handle)
                next_detect = time.monotonic() + detect_interval
                if target is not None:
                    self.get_logger().info(
                        f'clean_region detected {target.class_name}; '
                        f'interrupting {label}'
                    )
                    self._cancel_action_goal(nav_handle)
                    self._publish_stop()
                    return True

            time.sleep(0.05)

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{label} failed: status={wrapped.status}')
        return False

    def _cancel_action_goal(self, action_goal_handle):
        try:
            cancel_future = action_goal_handle.cancel_goal_async()
        except Exception:
            return
        self._wait_future(cancel_future, 2.0, None)

    def _set_coverage_linear_speed(self, goal_handle, speed, label):
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = 'FollowPath.desired_linear_vel'
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(max(0.01, speed))
        request.parameters = [parameter]

        response = self._call_service(
            self._coverage_controller_params_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None:
            self.get_logger().warning(f'{label} failed: no response')
            return False
        for result in response.results:
            if not result.successful:
                self.get_logger().warning(
                    f'{label} rejected: {result.reason}'
                )
                return False
        self.get_logger().info(
            f'{label}: FollowPath.desired_linear_vel={parameter.value.double_value:.3f}'
        )
        return True

    def _run_set_clean_region(self, goal_handle, goal):
        region_id = int(goal.bucket_id)
        if region_id <= 0:
            return False, 'clean region id must be greater than 0'
        min_x = float(goal.min_x)
        min_y = float(goal.min_y)
        max_x = float(goal.max_x)
        max_y = float(goal.max_y)
        if min_x >= max_x or min_y >= max_y:
            return False, 'clean region bounds invalid'
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
        region = CleanRegion(
            region_id=region_id,
            min_x=min_x,
            min_y=min_y,
            max_x=max_x,
            max_y=max_y,
            frame_id=str(frame_id),
        )
        self._clean_regions[region_id] = region
        self._save_clean_regions()
        self._feedback(goal_handle, f'set_clean_region: region {region_id}', 1.0)
        return (
            True,
            f'clean region {region_id} set: '
            f'({min_x:.3f}, {min_y:.3f}) -> ({max_x:.3f}, {max_y:.3f})',
        )

    def _resolve_clean_region(self, goal):
        min_x = float(goal.min_x)
        min_y = float(goal.min_y)
        max_x = float(goal.max_x)
        max_y = float(goal.max_y)
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)

        if min_x < max_x and min_y < max_y:
            return CleanRegion(
                region_id=int(goal.bucket_id) if int(goal.bucket_id) > 0 else 0,
                min_x=min_x,
                min_y=min_y,
                max_x=max_x,
                max_y=max_y,
                frame_id=str(frame_id),
            ), None

        region_id = int(goal.bucket_id)
        if region_id <= 0:
            region_id = 1
        region = self._clean_regions.get(region_id)
        if region is None:
            return None, f'clean region {region_id} is not set'

        return region, None

    def _run_navigate_tree(self, goal_handle, goal):
        self._feedback(goal_handle, 'navigate_to: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        self._feedback(goal_handle, 'navigate_to: sending goal', 0.20)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = (
            goal.frame_id or str(self.get_parameter('map_frame').value)
        )
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(goal.x)
        nav_goal.pose.pose.position.y = float(goal.y)
        nav_goal.pose.pose.position.z = 0.0
        qz, qw = self._yaw_to_quaternion_z_w(float(goal.yaw))
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        result = self._send_action_goal(
            self._navigate_client,
            nav_goal,
            timeout_sec=self._navigation_timeout_sec(),
            goal_handle=goal_handle,
            label='navigate to pose',
        )
        if result is None:
            return False, 'navigate to pose failed'
        return True, 'navigate to pose completed'

    def _run_navigate_path_tree(self, goal_handle, goal):
        path = goal.path
        if not path.poses:
            return False, 'navigate path is empty'

        self._feedback(goal_handle, 'navigate_path: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        total = len(path.poses)
        timeout_sec = self._navigation_timeout_sec()
        for index, pose in enumerate(path.poses, start=1):
            if self._is_canceling(goal_handle):
                return False, 'navigate path canceled'

            progress = 0.10 + 0.85 * ((index - 1) / max(1, total))
            self._feedback(
                goal_handle,
                f'navigate_path: sending point {index}/{total}',
                progress,
            )
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = pose
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            if not nav_goal.pose.header.frame_id:
                nav_goal.pose.header.frame_id = (
                    goal.frame_id or str(self.get_parameter('map_frame').value)
                )

            wrapped = self._send_action_goal_wrapped(
                self._navigate_client,
                nav_goal,
                timeout_sec=timeout_sec,
                goal_handle=goal_handle,
                label=f'navigate path point {index}/{total}',
            )
            if wrapped is None:
                return False, f'navigate path point {index}/{total} failed'
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                return (
                    False,
                    f'navigate path point {index}/{total} failed: '
                    f'status={wrapped.status}',
                )

        return True, f'navigate path completed: points={total}'

    def _navigate_to_dump_front(self, goal_handle, pose):
        distance = max(
            0.0,
            float(self.get_parameter('dump_approach_distance_m').value),
        )
        nav_yaw = self._normalize_angle(pose.yaw + math.pi)
        nav_x = pose.x + math.cos(pose.yaw) * distance
        nav_y = pose.y + math.sin(pose.yaw) * distance

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = pose.frame_id
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = nav_x
        nav_goal.pose.pose.position.y = nav_y
        nav_goal.pose.pose.position.z = 0.0
        qz, qw = self._yaw_to_quaternion_z_w(nav_yaw)
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw
        self.get_logger().info(
            f'dump bucket {pose.bucket_id} nav front goal: '
            f'({nav_x:.3f}, {nav_y:.3f}, yaw={nav_yaw:.3f}), '
            f'bucket=({pose.x:.3f}, {pose.y:.3f}, bucket_yaw={pose.yaw:.3f}), '
            f'offset={distance:.3f}'
        )

        if self._is_canceling(goal_handle):
            return False, f'navigate to dump bucket {pose.bucket_id} front canceled'
        if not self._navigate_client.wait_for_server(
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        ):
            return False, 'navigate to dump front action server not available'

        send_future = self._navigate_client.send_goal_async(nav_goal)
        if not self._wait_future(
            send_future,
            float(self.get_parameter('service_timeout_sec').value),
            goal_handle,
        ):
            return False, 'navigate to dump front goal response timed out'

        action_goal_handle = send_future.result()
        if action_goal_handle is None or not action_goal_handle.accepted:
            return False, 'navigate to dump front goal rejected'

        result_future = action_goal_handle.get_result_async()
        timeout_sec = self._navigation_timeout_sec()
        deadline = time.monotonic() + timeout_sec
        check_interval = max(
            0.1,
            float(self.get_parameter('dump_scan_camera_check_interval_sec').value),
        )
        next_bucket_check = time.monotonic()

        while rclpy.ok() and not result_future.done():
            if self._clear_requested or self._stop_requested or (
                goal_handle is not None and goal_handle.is_cancel_requested
            ):
                self._cancel_action_goal(action_goal_handle)
                return False, f'navigate to dump bucket {pose.bucket_id} front canceled'
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                self._cancel_action_goal(action_goal_handle)
                return False, f'navigate to dump bucket {pose.bucket_id} front canceled'
            if time.monotonic() >= deadline:
                self._cancel_action_goal(action_goal_handle)
                return False, f'navigate to dump bucket {pose.bucket_id} front timed out'

            if time.monotonic() >= next_bucket_check:
                next_bucket_check = time.monotonic() + check_interval
                accepted, reason = self._scan_camera_obstacle_matches_dump_bucket(
                    goal_handle,
                    pose,
                )
                if accepted:
                    self.get_logger().warning(
                        'scan_camera obstacle matches dump bucket; '
                        f'cancel nav and continue to visual alignment: {reason}'
                    )
                    self._cancel_action_goal(action_goal_handle)
                    self._publish_stop()
                    return (
                        True,
                        f'dump bucket {pose.bucket_id} found by scan_camera obstacle',
                    )
                if reason:
                    self.get_logger().debug(f'dump scan_camera check skipped: {reason}')

            time.sleep(0.05)

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return (
                False,
                f'navigate to dump bucket {pose.bucket_id} front failed: '
                f'status={wrapped.status}',
            )
        return True, f'navigated to dump bucket {pose.bucket_id} front'

    def _recent_scan_camera_obstacle(self):
        obstacle = self._latest_scan_camera_obstacle
        if obstacle is None:
            return None
        stale_sec = max(
            0.0,
            float(self.get_parameter('dump_scan_camera_obstacle_stale_sec').value),
        )
        if time.monotonic() - obstacle.seen_at > stale_sec:
            return None
        return obstacle

    def _scan_camera_obstacle_matches_dump_bucket(self, goal_handle, pose):
        if not bool(
            self.get_parameter('dump_scan_camera_bucket_check_enabled').value
        ):
            return False, ''
        if not bool(self.get_parameter('require_dump_bucket_detect').value):
            return False, ''
        if self._is_canceling(goal_handle):
            return False, 'task canceling'

        obstacle = self._recent_scan_camera_obstacle()
        if obstacle is None:
            return False, ''

        detection = self._detect_dump_bucket(goal_handle)
        if detection is None:
            return False, 'dump bucket contour detection unavailable'
        if not detection.detected:
            return False, f'scan obstacle has no usable contour: {detection.message}'

        accepted, reason = self._dump_detection_matches_pose(detection, pose)
        if not accepted:
            return False, reason
        return (
            True,
            f'scan_range={obstacle.range_m:.3f}, '
            f'scan_bearing={obstacle.bearing_rad:.3f}; {reason}',
        )

    def _dump_detection_matches_pose(self, detection, pose):
        robot_pose = self._latest_map_pose
        if robot_pose is None:
            return False, 'map pose unavailable'
        if not math.isfinite(detection.mean_depth_m) or detection.mean_depth_m <= 0.0:
            return False, f'invalid depth={detection.mean_depth_m}'
        if not math.isfinite(detection.offset_x_ratio):
            return False, f'invalid offset={detection.offset_x_ratio}'

        hfov = abs(float(self.get_parameter('dump_camera_horizontal_fov_rad').value))
        if hfov <= 0.0:
            return False, f'invalid camera hfov={hfov}'

        offset_ratio = detection.offset_x_ratio - self._dump_camera_offset_ratio(
            detection.mean_depth_m
        )
        bearing = math.atan(math.tan(hfov * 0.5) * offset_ratio)
        bucket_x = robot_pose.x + detection.mean_depth_m * math.cos(
            robot_pose.yaw + bearing
        )
        bucket_y = robot_pose.y + detection.mean_depth_m * math.sin(
            robot_pose.yaw + bearing
        )
        bucket_error = math.hypot(bucket_x - pose.x, bucket_y - pose.y)
        bucket_tolerance = abs(
            float(self.get_parameter('dump_scan_camera_bucket_map_tolerance_m').value)
        )

        if bucket_error <= bucket_tolerance:
            return (
                True,
                f'est_bucket=({bucket_x:.3f}, {bucket_y:.3f}), '
                f'set_bucket=({pose.x:.3f}, {pose.y:.3f}), '
                f'bucket_error={bucket_error:.3f}',
            )

        return (
            False,
            f'est_bucket=({bucket_x:.3f}, {bucket_y:.3f}), '
            f'set_bucket=({pose.x:.3f}, {pose.y:.3f}), '
            f'bucket_error={bucket_error:.3f}>{bucket_tolerance:.3f}',
        )

    def _detect_dump_bucket(self, goal_handle):
        request = DetectDumpBucket.Request()
        request.force_refresh = True
        response = self._call_service(
            self._detect_dump_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label='detect dump bucket',
        )
        if response is None:
            return None
        return DumpBucketDetection(
            detected=bool(response.detected),
            within_depth=int(response.signal) == 1,
            mask_pixels=int(response.mask_pixels),
            mean_depth_m=float(response.mean_depth_m),
            offset_x_px=float(response.offset_x_px),
            offset_x_ratio=float(response.offset_x_ratio),
            message=str(response.message),
        )

    def _center_dump_bucket(self, goal_handle):
        attempts = max(1, int(self.get_parameter('dump_center_max_attempts').value))
        tolerance = abs(float(self.get_parameter('dump_center_tolerance_ratio').value))
        speed = abs(float(self.get_parameter('dump_center_angular_speed').value))
        step_sec = max(0.05, float(self.get_parameter('dump_center_step_sec').value))
        if speed <= 0.0:
            return False

        for attempt in range(attempts):
            if self._is_canceling(goal_handle):
                return False
            detection = self._detect_dump_bucket(goal_handle)
            if detection is None or not detection.detected:
                if detection is None:
                    self.get_logger().warning(
                        'dump center detect '
                        f'{attempt + 1}/{attempts}: no response'
                    )
                else:
                    self.get_logger().warning(
                        'dump center detect '
                        f'{attempt + 1}/{attempts}: not detected, '
                        f'mask_pixels={detection.mask_pixels}, '
                        f'mean_depth_m={detection.mean_depth_m:.3f}, '
                        f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                        f'message={detection.message}'
                    )
                self._publish_stop()
                time.sleep(0.15)
                continue
            if not math.isfinite(detection.offset_x_ratio):
                self.get_logger().warning(
                    'dump center detect '
                    f'{attempt + 1}/{attempts}: invalid offset, '
                    f'mask_pixels={detection.mask_pixels}, '
                    f'mean_depth_m={detection.mean_depth_m:.3f}, '
                    f'offset_x_ratio={detection.offset_x_ratio}, '
                    f'message={detection.message}'
                )
                return False
            self.get_logger().info(
                'dump center detect '
                f'{attempt + 1}/{attempts}: detected, '
                f'mask_pixels={detection.mask_pixels}, '
                f'mean_depth_m={detection.mean_depth_m:.3f}, '
                f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                f'tolerance={tolerance:.4f}, '
                f'message={detection.message}'
            )
            if abs(detection.offset_x_ratio) <= tolerance:
                return True

            twist = Twist()
            # 图像中心在左侧时 offset_x 为负，向左旋转；反之向右旋转。
            twist.angular.z = -speed if detection.offset_x_ratio > 0.0 else speed
            self.get_logger().info(
                'dump center rotate step: '
                f'angular_z={twist.angular.z:.3f}, step_sec={step_sec:.3f}'
            )
            self._publish_twist_for(goal_handle, twist, step_sec)
            time.sleep(0.12)
        return False

    def _dump_camera_offset_ratio(self, depth_m):
        lateral_offset_m = float(
            self.get_parameter('dump_camera_lateral_offset_m').value
        )
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            depth_m = max(
                0.01,
                float(self.get_parameter('dump_align_target_depth_m').value),
            )
        hfov = abs(float(self.get_parameter('dump_camera_horizontal_fov_rad').value))
        if hfov <= 0.0:
            return 0.0
        return lateral_offset_m / (float(depth_m) * math.tan(hfov * 0.5))

    def _align_dump_bucket_depth(self, goal_handle):
        target_depth = max(
            0.0,
            float(self.get_parameter('dump_align_target_depth_m').value),
        )
        tolerance = abs(
            float(self.get_parameter('dump_align_depth_tolerance_m').value)
        )
        if self._is_canceling(goal_handle):
            return False
        detection = self._detect_dump_bucket(goal_handle)
        if not self._valid_dump_detection(detection, 'dump depth align detect'):
            return False

        remaining_m = detection.mean_depth_m - target_depth
        self.get_logger().info(
            'dump depth align detect: '
            f'mask_pixels={detection.mask_pixels}, '
            f'mean_depth_m={detection.mean_depth_m:.3f}, '
            f'target_depth_m={target_depth:.3f}, '
            f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
            f'message={detection.message}'
        )
        if remaining_m <= tolerance:
            return True

        max_step = max(
            0.0,
            float(self.get_parameter('dump_align_max_forward_step_m').value),
        )
        self._drive_forward_step(
            goal_handle,
            min(max_step, max(0.0, remaining_m)),
            float(self.get_parameter('dump_align_linear_speed').value),
        )
        return True

    def _valid_dump_detection(self, detection, label):
        if detection is None:
            self.get_logger().warning(f'{label}: no response')
            return False
        if not detection.detected:
            self.get_logger().warning(
                f'{label}: not detected, '
                f'mask_pixels={detection.mask_pixels}, '
                f'mean_depth_m={detection.mean_depth_m:.3f}, '
                f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                f'message={detection.message}'
            )
            return False
        if not math.isfinite(detection.offset_x_ratio):
            self.get_logger().warning(
                f'{label}: invalid offset, '
                f'mask_pixels={detection.mask_pixels}, '
                f'mean_depth_m={detection.mean_depth_m:.3f}, '
                f'offset_x_ratio={detection.offset_x_ratio}, '
                f'message={detection.message}'
            )
            return False
        if not math.isfinite(detection.mean_depth_m) or detection.mean_depth_m <= 0.0:
            self.get_logger().warning(
                f'{label}: invalid depth, '
                f'mask_pixels={detection.mask_pixels}, '
                f'mean_depth_m={detection.mean_depth_m}, '
                f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                f'message={detection.message}'
            )
            return False
        return True

    def _drive_forward_step(self, goal_handle, distance_m, speed):
        speed = abs(float(speed))
        distance_m = max(0.0, float(distance_m))
        if speed <= 0.0 or distance_m <= 0.0:
            return
        msg = Twist()
        msg.linear.x = speed
        self._publish_twist_for(goal_handle, msg, distance_m / speed)
        time.sleep(0.15)

    def _drive_backward_step(self, goal_handle, distance_m, speed):
        speed = abs(float(speed))
        distance_m = max(0.0, float(distance_m))
        if speed <= 0.0 or distance_m <= 0.0:
            return
        msg = Twist()
        msg.linear.x = -speed
        self._publish_twist_for(goal_handle, msg, distance_m / speed)
        time.sleep(0.15)

    def _call_dump_trigger(self, goal_handle, client, label):
        response = self._call_service(
            client,
            Trigger.Request(),
            timeout_sec=float(self.get_parameter('dump_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None:
            return False, f'{label} service unavailable or timed out'
        return bool(response.success), response.message

    def _call_dump_bucket_service(self, goal_handle, client, bucket_id, label):
        request = DumpTrash.Request()
        request.bucket_id = int(bucket_id)
        response = self._call_service(
            client,
            request,
            timeout_sec=float(self.get_parameter('dump_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None:
            return False, f'{label} service unavailable or timed out'
        return bool(response.success), response.message

    def _rotate_in_place_180(self, goal_handle):
        speed = abs(float(self.get_parameter('dump_rotate_angular_speed').value))
        if speed <= 0.0:
            return
        start_yaw = self._current_yaw()
        if start_yaw is None:
            self.get_logger().warning('no odom/imu yaw available; skip 180deg rotation')
            return
        target_yaw = self._normalize_angle(start_yaw + math.pi)
        tolerance = abs(float(self.get_parameter('dump_rotate_tolerance_rad').value))
        timeout_sec = max(1.0, float(self.get_parameter('dump_rotate_timeout_sec').value))
        deadline = time.monotonic() + timeout_sec

        msg = Twist()
        while time.monotonic() < deadline:
            if self._is_canceling_without_pause(goal_handle):
                break
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                break
            current_yaw = self._current_yaw()
            if current_yaw is None:
                break
            error = self._normalize_angle(target_yaw - current_yaw)
            if abs(error) <= tolerance:
                break
            msg.angular.z = speed if error > 0.0 else -speed
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()
        time.sleep(0.2)

    def _publish_twist_for(self, goal_handle, msg, duration_sec):
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._is_canceling_without_pause(goal_handle):
                break
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                break
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()

    def _current_yaw(self):
        if self._latest_odom_yaw is not None:
            return self._latest_odom_yaw
        return self._latest_imu_yaw

    def _current_map_xy(self):
        if self._latest_map_pose is None:
            return None
        return self._latest_map_pose.x, self._latest_map_pose.y

    def _load_dump_bucket_poses(self):
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        if not path.exists():
            return
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except Exception as exc:
            self.get_logger().warning(f'load dump bucket poses failed: {exc}')
            return
        poses = raw.get('dump_bucket_poses', raw)
        loaded = {}
        for key, item in poses.items():
            try:
                bucket_id = int(item.get('bucket_id', key))
                if bucket_id not in (1, 2):
                    continue
                yaw = float(
                    item.get(
                        'bucket_theta',
                        item.get('yaw', item.get('theta', 0.0)),
                    )
                )
                loaded[bucket_id] = DumpBucketPose(
                    bucket_id=bucket_id,
                    x=float(item.get('bucket_x', item['x'])),
                    y=float(item.get('bucket_y', item['y'])),
                    yaw=yaw,
                    frame_id=str(
                        item.get('frame_id')
                        or self.get_parameter('map_frame').value
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue
        self._dump_bucket_poses = loaded

    def _save_dump_bucket_poses(self):
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'dump_bucket_poses': {
                    str(bucket_id): {
                        'bucket_id': pose.bucket_id,
                        'x': pose.x,
                        'y': pose.y,
                        'theta': pose.yaw,
                        'yaw': pose.yaw,
                        'frame_id': pose.frame_id,
                    }
                    for bucket_id, pose in sorted(self._dump_bucket_poses.items())
                }
            }
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except Exception as exc:
            self.get_logger().warning(f'save dump bucket poses failed: {exc}')

    def _load_clean_regions(self):
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        if not path.exists():
            return
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except Exception as exc:
            self.get_logger().warning(f'load clean regions failed: {exc}')
            return
        regions = raw.get('clean_regions', raw)
        for key, item in regions.items():
            try:
                region_id = int(item.get('region_id', key))
                if region_id <= 0:
                    continue
                min_x = float(item['min_x'])
                min_y = float(item['min_y'])
                max_x = float(item['max_x'])
                max_y = float(item['max_y'])
                if min_x >= max_x or min_y >= max_y:
                    continue
                self._clean_regions[region_id] = CleanRegion(
                    region_id=region_id,
                    min_x=min_x,
                    min_y=min_y,
                    max_x=max_x,
                    max_y=max_y,
                    frame_id=str(
                        item.get('frame_id')
                        or self.get_parameter('map_frame').value
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue

    def _save_clean_regions(self):
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'clean_regions': {
                    str(region_id): {
                        'region_id': region.region_id,
                        'min_x': region.min_x,
                        'min_y': region.min_y,
                        'max_x': region.max_x,
                        'max_y': region.max_y,
                        'frame_id': region.frame_id,
                    }
                    for region_id, region in sorted(self._clean_regions.items())
                }
            }
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except Exception as exc:
            self.get_logger().warning(f'save clean regions failed: {exc}')

    def _send_arm_target(self, goal_handle, target):
        goal = ArmTarget.Goal()
        goal.x = target.target_x_cm
        goal.y = target.target_y_cm
        goal.z = target.target_z_cm
        goal.duration = float(self.get_parameter('arm_target_duration_sec').value)
        goal.end_angle = target.end_angle
        goal.gripper_rotation = target.gripper_rotation
        result = self._send_action_goal(
            self._arm_target_client,
            goal,
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            goal_handle=goal_handle,
            label='arm target',
        )
        if result is None or not result.success:
            raise RuntimeError('arm target failed')

    def _send_arm_preset(self, goal_handle, action_type):
        goal = ArmPreset.Goal()
        goal.action_type = int(action_type)
        result = self._send_action_goal(
            self._arm_preset_client,
            goal,
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            goal_handle=goal_handle,
            label='arm preset',
        )
        if result is None or not result.success:
            raise RuntimeError('arm preset failed')
        if int(action_type) not in (
            ArmPreset.Goal.ARM_CENTER,
            ArmPreset.Goal.ARM_MAPPING,
        ):
            return
        self._set_camera_tf_preset(goal_handle, action_type)

    def _set_camera_tf_preset(self, goal_handle, action_type):
        request = SetCameraTfPreset.Request()
        request.action_type = int(action_type)
        response = self._call_service(
            self._camera_tf_preset_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label='set camera tf preset',
        )
        if response is None:
            self.get_logger().warning(
                f'camera tf preset service unavailable: action_type={int(action_type)}'
            )
            return
        if not response.success:
            self.get_logger().warning(
                f'camera tf preset rejected: action_type={int(action_type)}, '
                f'message={response.message}'
            )

    def _set_gripper(self, goal_handle, open_ratio, duration_ms, label):
        request = SetGripper.Request()
        request.open_ratio = float(max(0.0, min(1.0, open_ratio)))
        request.duration_ms = float(duration_ms)
        response = self._call_service(
            self._gripper_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None or not response.success:
            raise RuntimeError(f'{label} failed')

    def _close_gripper_for_cleanup(self, task_name):
        request = SetGripper.Request()
        request.open_ratio = 0.0
        request.duration_ms = float(self.get_parameter('gripper_duration_ms').value)
        timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        label = f'{task_name} cleanup close gripper'

        if not self._gripper_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warning(f'{label} service not available')
            return

        future = self._gripper_client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                self.get_logger().warning(f'{label} service timed out')
                return
            time.sleep(0.02)

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'{label} service failed: {exc}')
            return

        if response is None or not response.success:
            message = '' if response is None else f': {response.message}'
            self.get_logger().warning(f'{label} failed{message}')

    def _call_service(self, client, request, timeout_sec, goal_handle, label):
        if self._is_canceling(goal_handle):
            return None
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warning(f'{label} service not available')
            return None

        future = client.call_async(request)
        if not self._wait_future(future, timeout_sec, goal_handle):
            self.get_logger().warning(f'{label} service timed out')
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f'{label} service failed: {exc}')
            return None

    def _send_action_goal(self, client, goal, timeout_sec, goal_handle, label):
        wrapped = self._send_action_goal_wrapped(
            client,
            goal,
            timeout_sec,
            goal_handle,
            label,
        )
        if wrapped is None:
            return None
        return wrapped.result

    def _send_action_goal_wrapped(self, client, goal, timeout_sec, goal_handle, label):
        if self._is_canceling(goal_handle):
            return None
        if not client.wait_for_server(
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        ):
            self.get_logger().warning(f'{label} action server not available')
            return None

        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec, goal_handle):
            self.get_logger().warning(f'{label} goal response timed out')
            return None

        action_goal_handle = send_future.result()
        if action_goal_handle is None or not action_goal_handle.accepted:
            self.get_logger().warning(f'{label} goal rejected')
            return None

        result_future = action_goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec, goal_handle):
            self.get_logger().warning(f'{label} result timed out')
            try:
                action_goal_handle.cancel_goal_async()
            except Exception:
                pass
            return None

        wrapped = result_future.result()
        return wrapped

    def _wait_future(self, future, timeout_sec, goal_handle):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if self._is_canceling_without_pause(goal_handle):
                future.cancel()
                return False
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                future.cancel()
                return False
            if time.monotonic() >= deadline:
                future.cancel()
                return False
            time.sleep(0.02)
        return future.done()

    def _wait_pause_and_extend_deadline(self, goal_handle, deadline):
        if not self._pause_requested:
            return deadline, True
        paused_at = time.monotonic()
        if not self._wait_if_paused(goal_handle):
            return deadline, False
        return deadline + (time.monotonic() - paused_at), True

    def _wait_if_paused(self, goal_handle):
        while rclpy.ok() and self._pause_requested and not self._clear_requested:
            self._publish_stop()
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            time.sleep(0.05)
        return not self._clear_requested

    def _publish_stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _feedback(self, goal_handle, state, progress):
        feedback = RobotTask.Feedback()
        feedback.state = str(state)
        feedback.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(feedback.state)

    def _result(self, success, message):
        result = RobotTask.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    def _navigation_timeout_sec(self):
        return max(
            1.0,
            float(self.get_parameter('navigation_timeout_sec').value),
        )

    def _is_canceling(self, goal_handle):
        if self._is_canceling_without_pause(goal_handle):
            return True
        if not self._wait_if_paused(goal_handle):
            return True
        return self._is_canceling_without_pause(goal_handle)

    def _is_canceling_without_pause(self, goal_handle):
        return (
            self._clear_requested
            or self._stop_requested
            or (goal_handle is not None and goal_handle.is_cancel_requested)
        )

    def _known_task_types(self):
        return {
            RobotTask.Goal.TASK_CLEAN_TRASH,
            RobotTask.Goal.TASK_DUMP_TRASH,
            RobotTask.Goal.TASK_CLEAN_REGION,
            RobotTask.Goal.TASK_NAVIGATE_TO,
            RobotTask.Goal.TASK_NAVIGATE_PATH,
            RobotTask.Goal.TASK_SET_DUMP_POSE,
            RobotTask.Goal.TASK_SET_CLEAN_REGION,
            RobotTask.Goal.TASK_STOP,
            RobotTask.Goal.TASK_RESUME,
        }

    def _task_name(self, task_type):
        names = {
            RobotTask.Goal.TASK_CLEAN_TRASH: 'clean_trash',
            RobotTask.Goal.TASK_DUMP_TRASH: 'dump_trash',
            RobotTask.Goal.TASK_CLEAN_REGION: 'clean_region',
            RobotTask.Goal.TASK_NAVIGATE_TO: 'navigate_to',
            RobotTask.Goal.TASK_NAVIGATE_PATH: 'navigate_path',
            RobotTask.Goal.TASK_SET_DUMP_POSE: 'set_dump_pose',
            RobotTask.Goal.TASK_SET_CLEAN_REGION: 'set_clean_region',
            RobotTask.Goal.TASK_STOP: 'stop',
            RobotTask.Goal.TASK_RESUME: 'resume',
        }
        return names.get(task_type, f'task_{task_type}')

    @staticmethod
    def _yaw_to_quaternion_z_w(yaw):
        half_yaw = float(yaw) * 0.5
        return math.sin(half_yaw), math.cos(half_yaw)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


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
