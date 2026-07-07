"""Clean-region task behavior for RobotTaskServer."""

import math
import time

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from robot_interfaces.action import ArmPreset
from robot_interfaces.srv import GenerateCoveragePath

from .task_models import CleanRegion


PARAMETER_DEFAULTS = (
    # 覆盖清扫路径生成服务名。
    ('coverage_path_service', 'generate_coverage_path'),
    # 区域清扫任务最长运行时间。
    ('clean_region_timeout_sec', 360.0),
    # 旧版时间触发视觉检测间隔，保留参数兼容已有配置。
    ('coverage_detect_interval_sec', 0.3),
    # 区域清扫每次向前推进并重新扫描的距离。
    ('coverage_scan_forward_step_m', 0.30),
    # 区域清扫导航中触发一次视觉检测的最小位移。
    ('coverage_detect_distance_m', 0.30),
    # 区域清扫导航中触发一次视觉检测的最小转角。
    ('coverage_detect_yaw_rad', math.radians(45.0)),
    # 动态调整 controller_server 参数的服务名。
    (
        'coverage_controller_set_parameters_service',
        '/controller_server/set_parameters',
    ),
    # 覆盖清扫时临时降低到的 Nav2 线速度。
    ('coverage_linear_speed', 0.12),
    # 覆盖清扫结束后恢复的 Nav2 线速度。
    ('coverage_restore_linear_speed', 0.24),
)


class CleanRegionTaskMixin:

    def _declare_clean_region_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

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

        start_pose = self._nearest_region_entry_pose(region, preview_path)
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

        scan_distance_m = forward_step_m
        step_m = forward_step_m if forward_step_m > 0.0 else 0.30

        while time.monotonic() < deadline:
            if self._is_canceling_without_pause(goal_handle):
                return None
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                return None

            current_xy = self._current_map_xy()
            if current_xy is None:
                return picked
            current_yaw = self._current_yaw()

            next_pose = self._coverage_pose_at_distance(path, scan_distance_m)
            if next_pose is None:
                return picked
            scan_distance_m += step_m

            self._feedback(goal_handle, 'clean_region: move forward to next scan', 0.70)
            result = self._navigate_pose_with_visual_interrupt(
                goal_handle,
                next_pose,
                'clean region next scan pose',
                detect_enabled=True,
            )
            if result is None:
                return None
            if result or self._coverage_detect_due(
                current_xy,
                current_yaw,
                forward_step_m,
                float(self.get_parameter('coverage_detect_yaw_rad').value),
            ):
                picked = self._handle_clean_region_detection(goal_handle, picked)
                if picked is None:
                    return None

        return picked

    def _coverage_pose_at_distance(self, path, target_distance_m):
        if not path.poses:
            return None
        if target_distance_m <= 0.0:
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
            if travelled + segment >= target_distance_m:
                ratio = (target_distance_m - travelled) / segment
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

    def _handle_clean_region_detection(self, goal_handle, picked):
        target = self._detect_reachable_target(goal_handle)
        if target is None:
            self.get_logger().warning(
                'clean_region visual trigger had no reachable target; '
                'continue coverage path'
            )
            return picked

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
        return picked

    def _nearest_region_entry_pose(self, region, fallback_path):
        current_xy = self._current_map_xy()
        if current_xy is None:
            return fallback_path.poses[0]

        x, y = current_xy
        clamped_x = max(region.min_x, min(region.max_x, x))
        clamped_y = max(region.min_y, min(region.max_y, y))
        candidates = (
            (region.min_x, clamped_y),
            (region.max_x, clamped_y),
            (clamped_x, region.min_y),
            (clamped_x, region.max_y),
        )
        entry_x, entry_y = min(
            candidates,
            key=lambda point: math.hypot(point[0] - x, point[1] - y),
        )

        pose = PoseStamped()
        pose.header = fallback_path.poses[0].header
        pose.pose.position.x = float(entry_x)
        pose.pose.position.y = float(entry_y)
        pose.pose.position.z = 0.0
        yaw = math.atan2(entry_y - y, entry_x - x)
        if math.hypot(entry_x - x, entry_y - y) <= 1e-6:
            current_yaw = self._current_yaw()
            yaw = current_yaw if current_yaw is not None else 0.0
        qz, qw = self._yaw_to_quaternion_z_w(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

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
