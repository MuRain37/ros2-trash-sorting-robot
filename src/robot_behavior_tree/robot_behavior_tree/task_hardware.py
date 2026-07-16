"""Hardware action and service helpers for RobotTaskServer."""

import time

import rclpy
from robot_interfaces.action import ArmPreset, ArmTarget
from robot_interfaces.srv import SetCameraTfPreset, SetGripper


PARAMETER_DEFAULTS = (
    # 机械臂笛卡尔目标 Action 名称。
    ('arm_target_action', '/arm_target'),
    # 机械臂预设姿态 Action 名称。
    ('arm_preset_action', '/arm_preset'),
    # 相机静态 TF 预设服务名，用于随机械臂姿态切换视觉外参。
    ('camera_tf_preset_service', '/real_camera_static_tf/set_preset'),
    # 夹爪开合服务名。
    ('gripper_service', 'arm_gripper'),
    # 普通机械臂 Action 完成超时时间。
    ('action_timeout_sec', 45.0),
    # 机械臂移动到视觉抓取目标的动作时长。
    ('arm_target_duration_sec', 2.0),
    # 夹爪开合动作时长。
    ('gripper_duration_ms', 500.0),
    # 夹爪动作后等待物体稳定的时间。
    ('gripper_wait_sec', 1.5),
)


class HardwareTaskMixin:

    def _declare_hardware_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

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

    def _restore_arm_mapping_for_cleanup(self, label='restore arm mapping pose'):
        try:
            self.get_logger().info(label)
            self._send_arm_preset(None, ArmPreset.Goal.ARM_MAPPING)
            return True
        except Exception as exc:
            self.get_logger().warning(f'{label} failed: {exc}')
            return False

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
