"""Navigation task behavior for RobotTaskServer."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_interfaces.action import ArmPreset


PARAMETER_DEFAULTS = (
    # Nav2 单点导航 Action 名称。
    ('navigate_to_pose_action', 'navigate_to_pose'),
)


class NavigationTaskMixin:

    def _declare_navigation_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

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
        last_detect_xy = self._current_map_xy()
        last_detect_yaw = self._current_yaw()
        detect_distance_m = max(
            0.0,
            float(self.get_parameter('coverage_detect_distance_m').value),
        )
        detect_yaw_rad = max(
            0.0,
            float(self.get_parameter('coverage_detect_yaw_rad').value),
        )
        deadline = time.monotonic() + self._navigation_timeout_sec()

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

            if detect_enabled and self._coverage_detect_due(
                last_detect_xy,
                last_detect_yaw,
                detect_distance_m,
                detect_yaw_rad,
            ):
                target = self._detect_grasp_target(goal_handle)
                last_detect_xy = self._current_map_xy()
                last_detect_yaw = self._current_yaw()
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

    def _coverage_detect_due(
        self,
        last_detect_xy,
        last_detect_yaw,
        detect_distance_m,
        detect_yaw_rad,
    ):
        current_xy = self._current_map_xy()
        current_yaw = self._current_yaw()
        if last_detect_xy is None or current_xy is None:
            moved = False
        else:
            moved = (
                math.hypot(
                    current_xy[0] - last_detect_xy[0],
                    current_xy[1] - last_detect_xy[1],
                )
                >= detect_distance_m
            )

        if last_detect_yaw is None or current_yaw is None:
            turned = False
        else:
            turned = (
                abs(self._normalize_angle(current_yaw - last_detect_yaw))
                >= detect_yaw_rad
            )
        return moved or turned

    def _cancel_action_goal(self, action_goal_handle):
        try:
            cancel_future = action_goal_handle.cancel_goal_async()
        except Exception:
            return
        self._wait_future(cancel_future, 2.0, None)
