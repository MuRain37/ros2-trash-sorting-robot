"""升降-倾倒结构桥接模块：步进电机与三路舵机的开环控制."""

import math

from robot_interfaces.srv import (
    SetGripper,
    SetLiftSteps,
    SetServoAngle,
)

from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_SERVO,
    PACKET_CMD_STEPPER,
    pack_servo_command,
    pack_stepper_command,
)


# 7 号舵机：水平旋转 (Horizontal Rotation)
HORIZONTAL_SERVO_ID = 7

# 8 号舵机：夹爪旋转 (Gripper Rotation)
GRIPPER_ROTATE_SERVO_ID = 8

# 9 号舵机：夹爪开度 (Gripper Open/Close)
LIFT_GRIPPER_SERVO_ID = 9
LIFT_GRIPPER_OPEN_RAW = 500.0       # 夹爪全开时的 raw 值
LIFT_GRIPPER_CLOSE_RAW = 595.0       # 夹爪扣紧时的 raw 值

# 舵机通用默认值
SERVO_DURATION_MS = 100.0            # 默认运动时长（毫秒）

SERVO_CENTER = 500.0                         # 舵机中位 raw 值
SERVO_RAW_MIN = 0.0                          # 舵机 raw 最小值
SERVO_RAW_MAX = 1000.0                       # 舵机 raw 最大值


def clamp_servo_raw(position):
    """将舵机原始位置值限制在 [0, 1000] 安全范围内."""
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, position))


def gripper_open_ratio_to_servo_raw(open_ratio):
    """夹爪开度比例 → 舵机原始位置值."""
    open_ratio = float(open_ratio)
    if not 0.0 <= open_ratio <= 1.0:
        raise ValueError(
            f'gripper open ratio out of range: {open_ratio} '
            'not in [0.0, 1.0]'
        )
    return LIFT_GRIPPER_CLOSE_RAW + open_ratio * (
        LIFT_GRIPPER_OPEN_RAW - LIFT_GRIPPER_CLOSE_RAW
    )


class LiftDumpBridge:
    """升降-倾倒结构桥接器：拆分步进电机和舵机控制服务."""

    def __init__(
        self,
        node,
        lift_steps_service='set_lift_steps',
        servo_angle_service='set_lift_servo_angle',
        gripper_service='set_lift_gripper',
        cmd_callback=None,
        servo_duration_ms=SERVO_DURATION_MS,
    ):
        self._node = node
        self._cmd_callback = cmd_callback
        self._servo_duration_ms = float(servo_duration_ms)

        self._target_steps = 0.0
        self._horizontal_servo_raw = SERVO_CENTER
        self._gripper_rotation_servo_raw = SERVO_CENTER
        self._gripper_servo_raw = LIFT_GRIPPER_OPEN_RAW

        self._lift_steps_service = node.create_service(
            SetLiftSteps,
            lift_steps_service,
            self._on_lift_steps_command,
        )
        self._servo_angle_service = node.create_service(
            SetServoAngle,
            servo_angle_service,
            self._on_servo_angle_command,
        )
        self._gripper_service = node.create_service(
            SetGripper,
            gripper_service,
            self._on_gripper_command,
        )

    def _on_lift_steps_command(self, request, response):
        """接收步进电机目标步数，校验后立即发送一次."""
        target_steps = float(request.target_steps)
        if not math.isfinite(target_steps):
            response.success = False
            response.message = f'target steps must be finite: {target_steps}'
            return response

        if not self._send_stepper_target(target_steps):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        self._target_steps = target_steps

        response.success = True
        response.message = 'lift steps command accepted'
        return response

    def _on_servo_angle_command(self, request, response):
        """接收任意升降-倾倒舵机 raw 目标，校验后立即发送一次."""
        try:
            servo_id = int(request.servo_id)
            if servo_id not in (
                HORIZONTAL_SERVO_ID,
                GRIPPER_ROTATE_SERVO_ID,
                LIFT_GRIPPER_SERVO_ID,
            ):
                raise ValueError(f'servo_id must be 7, 8 or 9: {servo_id}')
            servo_raw = float(request.angle)
            self._validate_servo_raw('servo raw', servo_raw)
            duration_ms = float(request.duration_ms)
            if duration_ms < 0.0:
                raise ValueError(f'duration_ms must be >= 0: {duration_ms}')
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            servo_id,
            servo_raw,
            duration_ms,
        ):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        if servo_id == HORIZONTAL_SERVO_ID:
            self._horizontal_servo_raw = servo_raw
        elif servo_id == GRIPPER_ROTATE_SERVO_ID:
            self._gripper_rotation_servo_raw = servo_raw
        else:
            self._gripper_servo_raw = servo_raw

        response.success = True
        response.message = 'lift servo command accepted'
        return response

    def _on_gripper_command(self, request, response):
        """接收夹爪开度比例，校验后立即发送一次."""
        try:
            gripper_raw = gripper_open_ratio_to_servo_raw(request.open_ratio)
            duration_ms = float(request.duration_ms)
            if duration_ms < 0.0:
                raise ValueError(f'duration_ms must be >= 0: {duration_ms}')
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            LIFT_GRIPPER_SERVO_ID,
            gripper_raw,
            duration_ms,
        ):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        self._gripper_servo_raw = gripper_raw

        response.success = True
        response.message = 'lift gripper command accepted'
        return response

    def _send_stepper_target(self, target_steps):
        """发送步进电机目标到下位机一次."""
        if not self._cmd_callback:
            return False

        self._cmd_callback(
            PACKET_CMD_STEPPER,
            pack_stepper_command(target_steps),
        )
        return True

    def _send_servo_target(self, servo_id, raw_position, duration_ms):
        """发送单路舵机目标到下位机一次."""
        if not self._cmd_callback:
            return False

        self._cmd_callback(
            PACKET_CMD_SERVO,
            pack_servo_command(
                servo_id,
                raw_position,
                duration_ms,
            ),
        )
        return True

    def _validate_servo_raw(self, name, value):
        self._validate_finite(name, value)
        if not SERVO_RAW_MIN <= value <= SERVO_RAW_MAX:
            raise ValueError(
                f'{name} must be in [{SERVO_RAW_MIN}, {SERVO_RAW_MAX}]: '
                f'{value}'
            )

    def _validate_finite(self, name, value):
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite: {value}')
