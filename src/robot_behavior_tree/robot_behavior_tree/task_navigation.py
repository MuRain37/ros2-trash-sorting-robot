"""Navigation task behavior for RobotTaskServer."""

import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from robot_interfaces.action import ArmPreset


PARAMETER_DEFAULTS = (
    # Nav2 单点导航 Action 名称。
    ('navigate_to_pose_action', 'navigate_to_pose'),
    # Nav2 多点导航 Action 名称。
    ('navigate_through_poses_action', 'navigate_through_poses'),
)

_VISUAL_WORK_DISABLED = object()


class NavigationTaskMixin:

    def _declare_navigation_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

    def _run_navigate_tree(self, goal_handle, goal):
        target_pose = self._navigation_target_pose(goal)
        if self._visual_work_enabled:
            return self._run_visual_navigate_tree(goal_handle, target_pose)

        self._feedback(goal_handle, 'navigate_to: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        self._feedback(goal_handle, 'navigate_to: sending goal', 0.20)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose

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

    def _navigation_target_pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = (
            goal.frame_id or str(self.get_parameter('map_frame').value)
        )
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal.x)
        pose.pose.position.y = float(goal.y)
        pose.pose.position.z = 0.0
        qz, qw = self._yaw_to_quaternion_z_w(float(goal.yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _run_visual_navigate_tree(self, goal_handle, target_pose):
        self._set_coverage_scan_camera_costmap_enabled(goal_handle, False)
        try:
            self._feedback(
                goal_handle,
                'navigate_to: arm center for visual work',
                0.05,
            )
            self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
            self._clear_coverage_costmaps(goal_handle)
            self._set_coverage_slow_speed(goal_handle)

            picked = 0
            while True:
                if not self._visual_work_enabled:
                    return self._continue_navigation_after_visual_work(
                        goal_handle,
                        target_pose,
                        picked,
                    )
                target_pose.header.stamp = self.get_clock().now().to_msg()
                result = self._navigate_pose_with_visual_interrupt(
                    goal_handle,
                    target_pose,
                    'visual navigate to pose',
                    detect_enabled=True,
                    require_visual_work=True,
                )
                if result is _VISUAL_WORK_DISABLED:
                    return self._continue_navigation_after_visual_work(
                        goal_handle,
                        target_pose,
                        picked,
                    )
                if result is None:
                    return False, 'visual navigate to pose canceled'
                if not result:
                    return (
                        True,
                        f'visual navigate to pose completed: picked={picked}',
                    )

                picked = self._handle_clean_region_detection(
                    goal_handle,
                    picked,
                )
                if picked is None:
                    return False, 'visual navigate to pose canceled'
        finally:
            self._publish_stop()
            self._restore_coverage_normal_speed(None)
            self._set_coverage_scan_camera_costmap_enabled(None, True)
            self._restore_arm_mapping_for_cleanup(
                'navigate_to: restore arm mapping pose'
            )

    def _continue_navigation_after_visual_work(
        self,
        goal_handle,
        target_pose,
        picked,
    ):
        self._publish_stop()
        self._feedback(
            goal_handle,
            'navigate_to: restore arm mapping pose',
            0.70,
        )
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        self._clear_coverage_costmaps(goal_handle)
        self._set_coverage_scan_camera_costmap_enabled(goal_handle, True)
        self._restore_coverage_normal_speed(goal_handle)

        target_pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose
        result = self._send_action_goal(
            self._navigate_client,
            nav_goal,
            timeout_sec=self._navigation_timeout_sec(),
            goal_handle=goal_handle,
            label='navigate after visual work',
        )
        if result is None:
            return False, 'navigate after visual work failed'
        return True, f'navigate after visual work completed: picked={picked}'

    def _run_navigate_path_tree(self, goal_handle, goal):
        path = goal.path
        if not path.poses:
            return False, 'navigate path is empty'

        self._feedback(goal_handle, 'navigate_path: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        total = len(path.poses)
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
        for pose in path.poses:
            pose.header.stamp = self.get_clock().now().to_msg()
            if not pose.header.frame_id:
                pose.header.frame_id = frame_id

        self._feedback(
            goal_handle,
            f'navigate_path: sending {total} points',
            0.10,
        )
        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = path.poses

        wrapped = self._send_action_goal_wrapped(
            self._navigate_through_client,
            nav_goal,
            timeout_sec=self._navigation_timeout_sec(),
            goal_handle=goal_handle,
            label=f'navigate path ({total} points)',
        )
        if wrapped is None:
            return False, 'navigate path failed'
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'navigate path failed: status={wrapped.status}'

        return True, f'navigate path completed: points={total}'

    def _navigate_pose_with_visual_interrupt(
        self,
        goal_handle,
        pose,
        label,
        detect_enabled=True,
        require_visual_work=False,
    ):
        if self._is_canceling(goal_handle):
            return None
        if require_visual_work and not self._visual_work_enabled:
            return _VISUAL_WORK_DISABLED
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
        deadline = time.monotonic() + self._navigation_timeout_sec()

        while rclpy.ok() and not result_future.done():
            if self._clear_requested or self._stop_requested or (
                goal_handle is not None and goal_handle.is_cancel_requested
            ):
                self._cancel_action_goal(nav_handle)
                return None
            if require_visual_work and not self._visual_work_enabled:
                self._cancel_action_goal(nav_handle)
                self._publish_stop()
                return _VISUAL_WORK_DISABLED
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

            visual_detection_enabled = detect_enabled and (
                not require_visual_work or self._visual_work_enabled
            )
            if visual_detection_enabled and time.monotonic() >= next_detect:
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
