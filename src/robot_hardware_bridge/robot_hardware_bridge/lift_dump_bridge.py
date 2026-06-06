"""升降-倾倒结构桥接模块：步进电机与三路舵机的开环控制."""

import math

from robot_interfaces.srv import SetGripper, SetLiftSteps, SetServoAngle

from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_SERVO,
    PACKET_CMD_STEPPER,
    pack_servo_command,
    pack_stepper_command,
)


DEFAULT_HORIZONTAL_SERVO_ID = 7
DEFAULT_HORIZONTAL_MAX_DEGREES = 180.0
DEFAULT_HORIZONTAL_DIRECTION = 1.0
DEFAULT_GRIPPER_ROTATE_SERVO_ID = 8
DEFAULT_GRIPPER_ROTATE_MAX_DEGREES = 180.0
DEFAULT_GRIPPER_ROTATE_DIRECTION = 1.0
DEFAULT_LIFT_GRIPPER_SERVO_ID = 9
DEFAULT_LIFT_GRIPPER_OPEN_RAW = 400.0
DEFAULT_LIFT_GRIPPER_CLOSE_RAW = 640.0
DEFAULT_SERVO_DURATION_MS = 100.0

SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0


def clamp_servo_raw(position):
    """将舵机原始位置值限制在 [0, 1000] 安全范围内."""
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, position))


def angle_to_servo_raw(value, max_degrees, direction):
    """角度（度）→ 舵机原始位置值."""
    degrees = float(value)
    if not -max_degrees <= degrees <= max_degrees:
        raise ValueError(
            f'angle out of range: {degrees} not in '
            f'[-{max_degrees}, {max_degrees}] deg'
        )
    return clamp_servo_raw(
        SERVO_CENTER + direction * degrees * (SERVO_CENTER / max_degrees)
    )


def gripper_open_ratio_to_servo_raw(open_ratio):
    """夹爪开度比例 → 舵机原始位置值."""
    open_ratio = float(open_ratio)
    if not 0.0 <= open_ratio <= 1.0:
        raise ValueError(
            f'gripper open ratio out of range: {open_ratio} '
            'not in [0.0, 1.0]'
        )
    return DEFAULT_LIFT_GRIPPER_CLOSE_RAW + open_ratio * (
        DEFAULT_LIFT_GRIPPER_OPEN_RAW - DEFAULT_LIFT_GRIPPER_CLOSE_RAW
    )


class LiftDumpBridge:
    """升降-倾倒结构桥接器：拆分步进电机和舵机控制服务."""

    def __init__(
        self,
        node,
        lift_steps_service='set_lift_steps',
        horizontal_rotation_service='set_lift_horizontal_rotation',
        gripper_rotation_service='set_lift_gripper_rotation',
        gripper_service='set_lift_gripper',
        cmd_callback=None,
        servo_duration_ms=DEFAULT_SERVO_DURATION_MS,
    ):
        self._node = node
        self._cmd_callback = cmd_callback
        self._servo_duration_ms = float(servo_duration_ms)

        self._target_steps = 0.0
        self._horizontal_servo_raw = SERVO_CENTER
        self._gripper_rotation_servo_raw = SERVO_CENTER
        self._gripper_servo_raw = DEFAULT_LIFT_GRIPPER_CLOSE_RAW

        self._lift_steps_service = node.create_service(
            SetLiftSteps,
            lift_steps_service,
            self._on_lift_steps_command,
        )
        self._horizontal_rotation_service = node.create_service(
            SetServoAngle,
            horizontal_rotation_service,
            self._on_horizontal_rotation_command,
        )
        self._gripper_rotation_service = node.create_service(
            SetServoAngle,
            gripper_rotation_service,
            self._on_gripper_rotation_command,
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

    def _on_horizontal_rotation_command(self, request, response):
        """接收水平旋转舵机目标，校验后立即发送一次."""
        try:
            horizontal_raw = angle_to_servo_raw(
                request.angle,
                DEFAULT_HORIZONTAL_MAX_DEGREES,
                DEFAULT_HORIZONTAL_DIRECTION,
            )
            duration_ms = float(request.duration_ms)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            DEFAULT_HORIZONTAL_SERVO_ID,
            horizontal_raw,
            duration_ms,
        ):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        self._horizontal_servo_raw = horizontal_raw

        response.success = True
        response.message = 'lift horizontal rotation command accepted'
        return response

    def _on_gripper_rotation_command(self, request, response):
        """接收夹爪旋转舵机目标，校验后立即发送一次."""
        try:
            gripper_rotation_raw = angle_to_servo_raw(
                request.angle,
                DEFAULT_GRIPPER_ROTATE_MAX_DEGREES,
                DEFAULT_GRIPPER_ROTATE_DIRECTION,
            )
            duration_ms = float(request.duration_ms)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            DEFAULT_GRIPPER_ROTATE_SERVO_ID,
            gripper_rotation_raw,
            duration_ms,
        ):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        self._gripper_rotation_servo_raw = gripper_rotation_raw

        response.success = True
        response.message = 'lift gripper rotation command accepted'
        return response

    def _on_gripper_command(self, request, response):
        """接收夹爪开度比例，校验后立即发送一次."""
        try:
            gripper_raw = gripper_open_ratio_to_servo_raw(request.open_ratio)
            duration_ms = float(request.duration_ms)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            DEFAULT_LIFT_GRIPPER_SERVO_ID,
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
