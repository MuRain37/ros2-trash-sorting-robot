"""高层行为树任务调度节点。"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import ArmPreset, ArmTarget, RobotTask
from robot_interfaces.srv import (
    AddCoverageRectangle,
    DetectDumpBucket,
    DetectGraspTarget,
    DumpTrash,
    SetGripper,
)
from sensor_msgs.msg import Imu


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


class RobotTaskServer(Node):
    """统一接收用户任务，并调度导航、视觉、机械臂和底层硬件能力。"""

    def __init__(self):
        super().__init__('robot_task_server')

        self.declare_parameter('action_name', '/robot_task')
        self.declare_parameter('detect_grasp_service', 'vision_grasp/detect')
        self.declare_parameter('detect_dump_service', 'detect_dump_bucket')
        self.declare_parameter('dump_trash_service', 'dump_trash')
        self.declare_parameter('coverage_task_service', 'add_coverage_rectangle')
        self.declare_parameter('arm_target_action', '/arm_target')
        self.declare_parameter('arm_preset_action', '/arm_preset')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', 'navigate_through_poses')
        self.declare_parameter('gripper_service', 'arm_gripper')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('service_timeout_sec', 3.0)
        self.declare_parameter('action_timeout_sec', 45.0)
        self.declare_parameter('dump_timeout_sec', 90.0)
        self.declare_parameter('clean_max_objects', 1)
        self.declare_parameter('detect_retry_count', 3)
        self.declare_parameter('approach_max_attempts', 3)
        self.declare_parameter('approach_y_threshold_cm', 30.0)
        self.declare_parameter('approach_target_y_cm', 24.0)
        self.declare_parameter('approach_max_step_m', 0.08)
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('gripper_duration_ms', 500.0)
        self.declare_parameter('gripper_wait_sec', 1.0)
        self.declare_parameter('require_dump_bucket_detect', True)
        self.declare_parameter('dump_approach_distance_m', 0.45)
        self.declare_parameter('dump_align_max_attempts', 10)
        self.declare_parameter('dump_align_step_m', 0.03)
        self.declare_parameter('dump_align_linear_speed', 0.05)
        self.declare_parameter('dump_rotate_angular_speed', 0.5)
        self.declare_parameter('dump_center_tolerance_ratio', 0.04)
        self.declare_parameter('dump_center_max_attempts', 12)
        self.declare_parameter('dump_center_angular_speed', 0.18)
        self.declare_parameter('dump_center_step_sec', 0.18)
        self.declare_parameter('dump_rotate_tolerance_rad', 0.04)
        self.declare_parameter('dump_rotate_timeout_sec', 12.0)
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
        self._dump_bucket_poses = {}
        self._clean_regions = {}
        self._latest_odom_yaw = None
        self._latest_imu_yaw = None
        self._load_dump_bucket_poses()
        self._load_clean_regions()
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._handle_odom,
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
        self._dump_trash_client = self.create_client(
            DumpTrash,
            self.get_parameter('dump_trash_service').value,
            callback_group=callback_group,
        )
        self._coverage_client = self.create_client(
            AddCoverageRectangle,
            self.get_parameter('coverage_task_service').value,
            callback_group=callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            self.get_parameter('gripper_service').value,
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

    def _handle_goal(self, goal_request):
        task_type = int(goal_request.task_type)
        if task_type not in self._known_task_types():
            self.get_logger().warning(f'rejecting unknown task_type={task_type}')
            return GoalResponse.REJECT
        if task_type == RobotTask.Goal.TASK_STOP:
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
        if (
            task_type == RobotTask.Goal.TASK_STOP
            and not self._task_lock.acquire(blocking=False)
        ):
            self._stop_requested = True
            self._publish_stop()
            goal_handle.succeed()
            return self._result(True, 'stop requested')

        if not self._task_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, 'another task is already running')

        self._active_goal = goal_handle
        goal = goal_handle.request
        task_name = self._task_name(task_type)
        if task_type != RobotTask.Goal.TASK_STOP:
            self._stop_requested = False
        self._feedback(goal_handle, f'{task_name}: started', 0.0)

        try:
            success, message = self._dispatch_task(goal_handle, goal)
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            success = False
            message = f'{task_name} failed: {exc}'
        finally:
            self._publish_stop()
            self._stop_requested = False
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
            self._stop_requested = True
            self._publish_stop()
            return True, 'robot stopped'
        if task_type == RobotTask.Goal.TASK_RESUME:
            return True, 'resume accepted'
        return False, f'unsupported task_type={task_type}'

    def _run_clean_trash_tree(self, goal_handle):
        """清扫垃圾：识别 -> 必要时前进 -> MoveIt 抓取 -> 投桶 -> 归中。"""
        self._feedback(goal_handle, 'clean_trash: arm center', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)

        picked = 0
        max_objects = int(self.get_parameter('clean_max_objects').value)
        for index in range(max(1, max_objects)):
            if self._is_canceling(goal_handle):
                return False, 'clean trash canceled'

            self._feedback(
                goal_handle,
                f'clean_trash: detecting target {index + 1}',
                0.10,
            )
            target = self._detect_reachable_target(goal_handle)
            if target is None:
                if picked == 0:
                    return False, 'no reachable trash target detected'
                return True, f'clean trash completed: picked={picked}'

            self._feedback(
                goal_handle,
                f'clean_trash: grasping {target.class_name}',
                0.35,
            )
            self._grasp_target(goal_handle, target)
            picked += 1

        return True, f'clean trash completed: picked={picked}'

    def _detect_reachable_target(self, goal_handle):
        last_target = None
        for approach_index in range(
            int(self.get_parameter('approach_max_attempts').value) + 1
        ):
            target = self._detect_grasp_target(goal_handle)
            if target is None:
                return None
            last_target = target

            threshold_cm = float(
                self.get_parameter('approach_y_threshold_cm').value
            )
            if target.target_y_cm <= threshold_cm:
                return target

            if approach_index >= int(
                self.get_parameter('approach_max_attempts').value
            ):
                break

            self._feedback(
                goal_handle,
                'clean_trash: target is far, moving base forward',
                0.20,
            )
            self._drive_forward_toward_target(goal_handle, target.target_y_cm)

        self.get_logger().warning(
            'target still beyond approach threshold: '
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
            time.sleep(0.15)
            self._feedback(
                goal_handle,
                f'clean_trash: detection retry {attempt + 1}',
                0.15,
            )
        return None

    def _drive_forward_toward_target(self, goal_handle, target_y_cm):
        target_y = float(self.get_parameter('approach_target_y_cm').value)
        max_step = float(self.get_parameter('approach_max_step_m').value)
        speed = abs(float(self.get_parameter('approach_linear_speed').value))
        if speed <= 0.0:
            return

        distance_m = max(0.0, (target_y_cm - target_y) / 100.0)
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

        self._feedback(goal_handle, 'clean_trash: arm center', 0.90)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        self._set_gripper(goal_handle, 0.0, duration_ms, 'close gripper idle')

    def _run_dump_trash_tree(self, goal_handle, goal):
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
        if len(bucket_ids) == 1:
            return True, f'dump bucket {bucket_ids[0]} completed'
        return True, 'dump buckets 1 and 2 completed'

    def _dump_single_bucket(self, goal_handle, bucket_id, base_progress=0.0, span=1.0):
        pose = self._dump_bucket_poses.get(bucket_id)
        if pose is None:
            return False, f'dump bucket {bucket_id} pose is not set'

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
                return False, f'dump bucket {bucket_id} visual center align failed'

            self._feedback(
                goal_handle,
                f'dump_trash: depth align bucket {bucket_id}',
                base_progress + span * 0.55,
            )
            if not self._align_dump_bucket_depth(goal_handle):
                return False, f'dump bucket {bucket_id} not detected after align'

        self._feedback(
            goal_handle,
            f'dump_trash: rotate before dumping bucket {bucket_id}',
            base_progress + span * 0.65,
        )
        self._rotate_in_place_180(goal_handle)

        self._feedback(
            goal_handle,
            f'dump_trash: bucket {bucket_id}',
            base_progress + span * 0.82,
        )
        request = DumpTrash.Request()
        request.bucket_id = bucket_id
        response = self._call_service(
            self._dump_trash_client,
            request,
            timeout_sec=float(self.get_parameter('dump_timeout_sec').value),
            goal_handle=goal_handle,
            label='dump trash',
        )
        if response is None:
            return False, 'dump trash service unavailable or timed out'
        return bool(response.success), response.message

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
        self._feedback(goal_handle, 'clean_region: queue coverage rectangle', 0.20)
        region, error = self._resolve_clean_region(goal)
        if region is None:
            return False, error

        request = AddCoverageRectangle.Request()
        request.min_x = region.min_x
        request.min_y = region.min_y
        request.max_x = region.max_x
        request.max_y = region.max_y
        request.frame_id = region.frame_id
        response = self._call_service(
            self._coverage_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label='add coverage rectangle',
        )
        if response is None:
            return False, 'coverage task service unavailable or timed out'
        return bool(response.success), response.message

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
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
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

        self._feedback(goal_handle, 'navigate_path: sending path', 0.20)
        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = path.poses
        wrapped = self._send_action_goal_wrapped(
            self._navigate_through_client,
            nav_goal,
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            goal_handle=goal_handle,
            label='navigate through poses',
        )
        if wrapped is None:
            return False, 'navigate through poses failed'
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'navigate through poses failed: status={wrapped.status}'
        return True, 'navigate path completed'

    def _navigate_to_dump_front(self, goal_handle, pose):
        distance = max(
            0.0,
            float(self.get_parameter('dump_approach_distance_m').value),
        )
        front_x = pose.x - math.cos(pose.yaw) * distance
        front_y = pose.y - math.sin(pose.yaw) * distance

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = pose.frame_id
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = front_x
        nav_goal.pose.pose.position.y = front_y
        nav_goal.pose.pose.position.z = 0.0
        qz, qw = self._yaw_to_quaternion_z_w(pose.yaw)
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        wrapped = self._send_action_goal_wrapped(
            self._navigate_client,
            nav_goal,
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            goal_handle=goal_handle,
            label='navigate to dump front',
        )
        if wrapped is None:
            return False, f'navigate to dump bucket {pose.bucket_id} front failed'
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return (
                False,
                f'navigate to dump bucket {pose.bucket_id} front failed: '
                f'status={wrapped.status}',
            )
        return True, f'navigated to dump bucket {pose.bucket_id} front'

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

        for _ in range(attempts):
            if self._is_canceling(goal_handle):
                return False
            detection = self._detect_dump_bucket(goal_handle)
            if detection is None or not detection.detected:
                self._publish_stop()
                time.sleep(0.15)
                continue
            if not math.isfinite(detection.offset_x_ratio):
                return False
            if abs(detection.offset_x_ratio) <= tolerance:
                return True

            twist = Twist()
            # 图像中心在左侧时 offset_x 为负，向左旋转；反之向右旋转。
            twist.angular.z = -speed if detection.offset_x_ratio > 0.0 else speed
            self._publish_twist_for(goal_handle, twist, step_sec)
            time.sleep(0.12)
        return False

    def _align_dump_bucket_depth(self, goal_handle):
        attempts = max(1, int(self.get_parameter('dump_align_max_attempts').value))
        for attempt in range(attempts):
            if self._is_canceling(goal_handle):
                return False
            detection = self._detect_dump_bucket(goal_handle)
            if detection is not None and detection.detected and detection.within_depth:
                return True
            if attempt < attempts - 1:
                self._drive_forward_step(
                    goal_handle,
                    float(self.get_parameter('dump_align_step_m').value),
                    float(self.get_parameter('dump_align_linear_speed').value),
                )
        return False

    def _drive_forward_step(self, goal_handle, distance_m, speed):
        speed = abs(float(speed))
        distance_m = max(0.0, float(distance_m))
        if speed <= 0.0 or distance_m <= 0.0:
            return
        msg = Twist()
        msg.linear.x = speed
        self._publish_twist_for(goal_handle, msg, distance_m / speed)
        time.sleep(0.15)

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
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
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
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()

    def _current_yaw(self):
        if self._latest_odom_yaw is not None:
            return self._latest_odom_yaw
        return self._latest_imu_yaw

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
        for key, item in poses.items():
            try:
                bucket_id = int(item.get('bucket_id', key))
                if bucket_id not in (1, 2):
                    continue
                self._dump_bucket_poses[bucket_id] = DumpBucketPose(
                    bucket_id=bucket_id,
                    x=float(item['x']),
                    y=float(item['y']),
                    yaw=float(item.get('yaw', item.get('theta', 0.0))),
                    frame_id=str(
                        item.get('frame_id')
                        or self.get_parameter('map_frame').value
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue

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
        goal.duration = 2.0
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
            if self._is_canceling(goal_handle):
                future.cancel()
                return False
            if time.monotonic() >= deadline:
                future.cancel()
                return False
            time.sleep(0.02)
        return future.done()

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

    def _is_canceling(self, goal_handle):
        return (
            self._stop_requested
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
