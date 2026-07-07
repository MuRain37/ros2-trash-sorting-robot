"""Dump-trash task behavior for RobotTaskServer."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from robot_interfaces.action import ArmPreset
from robot_interfaces.srv import DetectDumpBucket, DumpTrash
from std_srvs.srv import Trigger

from .task_models import DumpBucketDetection, DumpBucketPose


PARAMETER_DEFAULTS = (
    # 卸载桶视觉检测服务名，倒垃圾前用于识别和对中外部垃圾桶。
    ('detect_dump_service', 'detect_dump_bucket'),
    # 卸载机构取起车载垃圾桶的服务名。
    ('dump_pick_bucket_service', 'dump_pick_bucket'),
    # 卸载机构执行倾倒动作的服务名。
    ('dump_pour_bucket_service', 'dump_pour_bucket'),
    # 卸载机构放回车载垃圾桶的服务名。
    ('dump_place_bucket_service', 'dump_place_bucket'),
    # 卸载机构回零或收回的服务名。
    ('dump_home_service', 'dump_home'),
    # 倒垃圾硬件服务调用的超时时间。
    ('dump_timeout_sec', 240.0),
    # 取起车载垃圾桶后倒车的距离。
    ('dump_backup_distance_m', 0.30),
    # 取起车载垃圾桶后倒车的线速度。
    ('dump_backup_linear_speed', 0.05),
    # 倒完垃圾后向前恢复的距离。
    ('dump_forward_distance_m', 0.15),
    # 倒完垃圾后向前恢复的线速度。
    ('dump_forward_linear_speed', 0.05),
    # 倒垃圾前是否必须执行卸载桶视觉检测和对中。
    ('require_dump_bucket_detect', True),
    # 视觉对中失败时是否终止倒垃圾任务。
    ('dump_visual_align_required', False),
    # 导航到卸载桶前方时与桶保持的距离。
    ('dump_approach_distance_m', 1.15),
    # 倒垃圾前原地转身使用的角速度。
    ('dump_rotate_angular_speed', 0.8),
    # 卸载桶视觉居中允许的图像中心偏移比例。
    ('dump_center_tolerance_ratio', 0.04),
    # 卸载桶视觉居中最多旋转修正次数。
    ('dump_center_max_attempts', 8),
    # 卸载桶视觉居中单步旋转角速度。
    ('dump_center_angular_speed', 0.2),
    # 卸载桶视觉居中每次旋转持续时间。
    ('dump_center_step_sec', 0.30),
    # 原地转身结束时允许的 yaw 误差。
    ('dump_rotate_tolerance_rad', 0.04),
    # 原地转身动作超时时间。
    ('dump_rotate_timeout_sec', 12.0),
    # 是否启用 scan_camera 障碍物辅助确认卸载桶。
    ('dump_scan_camera_bucket_check_enabled', True),
    # scan_camera 认为有效障碍物的最近距离。
    ('dump_scan_camera_obstacle_min_range_m', 0.05),
    # scan_camera 认为有效障碍物的最远距离。
    ('dump_scan_camera_obstacle_max_range_m', 1.4),
    # scan_camera 只检查正前方该角度范围内的障碍物。
    ('dump_scan_camera_front_angle_rad', 0.8),
    # scan_camera 障碍物数据超过该时间后视为过期。
    ('dump_scan_camera_obstacle_stale_sec', 1.0),
    # 导航去卸载桶前方时 scan_camera 辅助检查间隔。
    ('dump_scan_camera_check_interval_sec', 0.5),
    # 视觉估计桶位置与已标定桶位置允许的地图误差。
    ('dump_scan_camera_bucket_map_tolerance_m', 0.75),
    # 卸载桶相机水平视场角，用于把图像偏移换算为方位角。
    ('dump_camera_horizontal_fov_rad', 1.21),
    # 卸载桶相机相对机器人中心的横向偏移。
    ('dump_camera_lateral_offset_m', 0.033),
)


class DumpTrashTaskMixin:

    def _declare_dump_trash_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

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
            return 0.0
        hfov = abs(float(self.get_parameter('dump_camera_horizontal_fov_rad').value))
        if hfov <= 0.0:
            return 0.0
        return lateral_offset_m / (float(depth_m) * math.tan(hfov * 0.5))

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
