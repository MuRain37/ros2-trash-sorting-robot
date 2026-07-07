"""Shared ROS action/service helpers for RobotTaskServer."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from robot_interfaces.action import RobotTask


PARAMETER_DEFAULTS = (
    # 对外暴露的 RobotTask Action 名称。
    ('action_name', '/robot_task'),
    # 底盘速度控制话题名。
    ('cmd_vel_topic', '/cmd_vel'),
    # 导航、标定和存储位姿使用的地图坐标系。
    ('map_frame', 'map'),
    # 里程计话题名，用于读取底盘 yaw。
    ('odom_topic', '/odom'),
    # AMCL 位姿话题名，用于读取机器人 map 位姿。
    ('amcl_pose_topic', '/amcl_pose'),
    # IMU 话题名，用作 yaw 的兜底来源。
    ('imu_topic', '/imu/data'),
    # 普通服务调用和短等待的超时时间。
    ('service_timeout_sec', 3.0),
    # 导航任务和导航相关等待的超时时间。
    ('navigation_timeout_sec', 120.0),
)


class RosTaskUtilsMixin:

    def _declare_ros_task_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

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
