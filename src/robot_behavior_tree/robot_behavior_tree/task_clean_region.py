"""Clean-region task behavior for RobotTaskServer."""

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from robot_interfaces.action import ArmPreset
from robot_interfaces.srv import GenerateCoveragePath
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import SetBool

from .task_models import CleanRegion


PARAMETER_DEFAULTS = (
    # 覆盖清扫路径生成服务名。
    ('coverage_path_service', 'generate_coverage_path'),
    # 区域清扫任务最长运行时间。
    ('clean_region_timeout_sec', 360.0),
    # 覆盖清扫导航中视觉检测抓取目标的间隔。
    ('coverage_detect_interval_sec', 0.2),
    # 动态调整 controller_server 参数的服务名。
    (
        'coverage_controller_set_parameters_service',
        '/controller_server/set_parameters',
    ),
    # 清扫抓取期间开关 scan_camera 进入 Nav2 costmap 的转发服务名。
    (
        'coverage_scan_camera_costmap_service',
        '/set_scan_camera_costmap_enabled',
    ),
    # 清扫抓取前清除 scan_camera 已写入 Nav2 costmap 的残留障碍。
    (
        'coverage_local_clear_costmap_service',
        '/local_costmap/clear_entirely_local_costmap',
    ),
    (
        'coverage_global_clear_costmap_service',
        '/global_costmap/clear_entirely_global_costmap',
    ),
    # 覆盖清扫时临时降低到的 Nav2 线速度。
    ('coverage_linear_speed', 0.05),
    # 覆盖清扫时临时降低到的 Nav2 角速度。
    ('coverage_angular_speed', 0.22),
    # 覆盖清扫结束后恢复的 Nav2 线速度。
    ('coverage_restore_linear_speed', 0.25),
    # 覆盖清扫结束后恢复的 Nav2 角速度。
    ('coverage_restore_angular_speed', 0.8),
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
        preview_path, message = self._generate_coverage_path(
            goal_handle,
            region,
            start_position=self._current_map_xy(),
        )
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

        self._set_coverage_scan_camera_costmap_enabled(goal_handle, False)
        try:
            self._feedback(goal_handle, 'clean_region: arm center for coverage', 0.20)
            self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
            self._clear_coverage_costmaps(goal_handle)

            self._set_coverage_slow_speed(goal_handle)
            picked = self._run_clean_region_scan_loop(goal_handle, region)
            if picked is None:
                return False, 'clean region canceled'
        finally:
            self._restore_coverage_normal_speed(None)
            self._set_coverage_scan_camera_costmap_enabled(None, True)
        return True, f'clean region completed: picked={picked}'

    def _run_clean_region_scan_loop(self, goal_handle, region):
        picked = 0
        timeout_sec = max(
            1.0,
            float(self.get_parameter('clean_region_timeout_sec').value),
        )
        deadline = time.monotonic() + timeout_sec
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

        waypoint_index = 0

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

            if waypoint_index >= len(path.poses):
                return picked
            next_pose = path.poses[waypoint_index]

            self._feedback(
                goal_handle,
                f'clean_region: coverage waypoint {waypoint_index + 1}/'
                f'{len(path.poses)}',
                0.70,
            )
            result = self._navigate_pose_with_visual_interrupt(
                goal_handle,
                next_pose,
                f'coverage waypoint {waypoint_index + 1}',
                detect_enabled=True,
            )
            if result is None:
                return None
            if result:
                picked = self._handle_clean_region_detection(goal_handle, picked)
                if picked is None:
                    return None
                # 抓取并归中后继续当前未完成 waypoint，由 Nav2 重新规划到该点。
                continue

            waypoint_index += 1

        return picked

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
        if bucket_level >= 100:
            self._restore_coverage_normal_speed(goal_handle)
        success, message = self._maybe_auto_dump_full_bucket(
            goal_handle,
            bucket_id,
            bucket_level,
        )
        if not success:
            raise RuntimeError(message)
        if bucket_level >= 100:
            self._set_coverage_slow_speed(goal_handle)

        if self._is_canceling(goal_handle):
            return None
        self._feedback(goal_handle, 'clean_region: arm center after grasp', 0.60)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        self._clear_coverage_costmaps(goal_handle)
        return picked

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

    def _set_coverage_scan_camera_costmap_enabled(self, goal_handle, enabled):
        label = 'enable scan_camera costmap source' if enabled else (
            'disable scan_camera costmap source'
        )
        request = SetBool.Request()
        request.data = bool(enabled)

        response = self._call_service(
            self._scan_camera_costmap_gate_client,
            request,
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None:
            self.get_logger().warning(f'{label} failed: no response')
            return False
        if not response.success:
            self.get_logger().warning(f'{label} rejected: {response.message}')
            return False
        self.get_logger().info(f'{label}: enabled={bool(enabled)}')
        return True

    def _clear_coverage_costmaps(self, goal_handle):
        local_ok = self._clear_costmap(
            self._coverage_local_clear_costmap_client,
            goal_handle,
            'clear local costmap after scan_camera pause',
        )
        global_ok = self._clear_costmap(
            self._coverage_global_clear_costmap_client,
            goal_handle,
            'clear global costmap after scan_camera pause',
        )
        return local_ok and global_ok

    def _clear_costmap(self, client, goal_handle, label):
        response = self._call_service(
            client,
            ClearEntireCostmap.Request(),
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            goal_handle=goal_handle,
            label=label,
        )
        if response is None:
            self.get_logger().warning(f'{label} failed: no response')
            return False
        self.get_logger().info(label)
        return True

    def _set_coverage_slow_speed(self, goal_handle):
        return self._set_coverage_speed_limits(
            goal_handle,
            float(self.get_parameter('coverage_linear_speed').value),
            float(self.get_parameter('coverage_angular_speed').value),
            'set coverage slow speed',
        )

    def _restore_coverage_normal_speed(self, goal_handle):
        return self._set_coverage_speed_limits(
            goal_handle,
            float(self.get_parameter('coverage_restore_linear_speed').value),
            float(self.get_parameter('coverage_restore_angular_speed').value),
            'restore coverage normal speed',
        )

    def _set_coverage_speed_limits(
        self,
        goal_handle,
        linear_speed,
        angular_speed,
        label,
    ):
        request = SetParameters.Request()
        linear_parameter = Parameter()
        linear_parameter.name = 'FollowPath.vx_max'
        linear_parameter.value = ParameterValue()
        linear_parameter.value.type = ParameterType.PARAMETER_DOUBLE
        linear_parameter.value.double_value = float(max(0.01, linear_speed))

        angular_parameter = Parameter()
        angular_parameter.name = 'FollowPath.wz_max'
        angular_parameter.value = ParameterValue()
        angular_parameter.value.type = ParameterType.PARAMETER_DOUBLE
        angular_parameter.value.double_value = float(max(0.01, angular_speed))

        request.parameters = [linear_parameter, angular_parameter]

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
            f'{label}: '
            f'FollowPath.vx_max={linear_parameter.value.double_value:.3f}, '
            f'FollowPath.wz_max={angular_parameter.value.double_value:.3f}'
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
