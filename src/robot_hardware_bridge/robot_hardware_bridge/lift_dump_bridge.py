"""升降-倾倒结构桥接模块：步进电机与三路舵机的开环控制."""

import math

from robot_interfaces.srv import LiftDumpCommand

from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_SERVO,
    PACKET_CMD_STEPPER,
    pack_servo_command,
    pack_stepper_command,
)


DEFAULT_STEPPER_MAX_STEPS = 1000.0
DEFAULT_HORIZONTAL_SERVO_ID = 7
DEFAULT_HORIZONTAL_MAX_DEGREES = 180.0
DEFAULT_HORIZONTAL_DIRECTION = 1.0
DEFAULT_GRIPPER_ROTATE_SERVO_ID = 8
DEFAULT_GRIPPER_ROTATE_MAX_DEGREES = 180.0
DEFAULT_GRIPPER_ROTATE_DIRECTION = 1.0
DEFAULT_LIFT_GRIPPER_SERVO_ID = 9
DEFAULT_LIFT_GRIPPER_OPEN_RAW = 150.0
DEFAULT_LIFT_GRIPPER_CLOSE_RAW = 700.0
DEFAULT_SEND_RATE_HZ = 10.0
DEFAULT_SERVO_DURATION_MS = 100.0

SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0


def clamp_servo_raw(position):
    """将舵机原始位置值限制在 [0, 1000] 安全范围内."""
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, position))


def angle_to_servo_raw(value, max_degrees, direction):
    """角度（弧度）→ 舵机原始位置值."""
    degrees = math.degrees(float(value))
    if not -max_degrees <= degrees <= max_degrees:
        raise ValueError(
            f'angle out of range: {degrees} not in '
            f'[-{max_degrees}, {max_degrees}] deg'
        )
    return clamp_servo_raw(
        SERVO_CENTER + direction * degrees * (SERVO_CENTER / max_degrees)
    )


def lift_position_to_steps(lift_position):
    """升降相对位置 → 步进电机绝对步数."""
    lift_position = float(lift_position)
    if not 0.0 <= lift_position <= 1.0:
        raise ValueError(
            f'lift position out of range: {lift_position} '
            'not in [0.0, 1.0]'
        )
    return lift_position * DEFAULT_STEPPER_MAX_STEPS


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
    """升降-倾倒结构桥接器：服务更新目标，10Hz 持续发送."""

    def __init__(
        self,
        node,
        service_name='lift_dump_cmd',
        cmd_callback=None,
        send_rate=DEFAULT_SEND_RATE_HZ,
        servo_duration_ms=DEFAULT_SERVO_DURATION_MS,
    ):
        self._node = node
        self._cmd_callback = cmd_callback
        self._send_period = 1.0 / float(send_rate) if send_rate > 0.0 else 0.0
        self._servo_duration_ms = float(servo_duration_ms)

        self._target_steps = 0.0
        self._horizontal_servo_raw = SERVO_CENTER
        self._gripper_rotation_servo_raw = SERVO_CENTER
        self._gripper_servo_raw = DEFAULT_LIFT_GRIPPER_CLOSE_RAW

        self._service = node.create_service(
            LiftDumpCommand,
            service_name,
            self._on_command,
        )
        self._timer = node.create_timer(
            self._send_period,
            self._publish_current_targets,
        )
        self._publish_current_targets()

    def _on_command(self, request, response):
        """接收新目标，校验并更新当前开环目标."""
        try:
            lift_position = lift_position_to_steps(request.lift_position)
            horizontal_raw = angle_to_servo_raw(
                request.horizontal_rotation,
                DEFAULT_HORIZONTAL_MAX_DEGREES,
                DEFAULT_HORIZONTAL_DIRECTION,
            )
            gripper_rotation_raw = angle_to_servo_raw(
                request.gripper_rotation,
                DEFAULT_GRIPPER_ROTATE_MAX_DEGREES,
                DEFAULT_GRIPPER_ROTATE_DIRECTION,
            )
            gripper_raw = gripper_open_ratio_to_servo_raw(
                request.gripper_open_ratio
            )
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        self._target_steps = lift_position
        self._horizontal_servo_raw = horizontal_raw
        self._gripper_rotation_servo_raw = gripper_rotation_raw
        self._gripper_servo_raw = gripper_raw

        response.success = True
        response.message = 'lift-dump command accepted'
        return response

    def _publish_current_targets(self):
        """按 10Hz 发送当前目标到下位机."""
        if not self._cmd_callback:
            return

        self._cmd_callback(
            PACKET_CMD_STEPPER,
            pack_stepper_command(self._target_steps),
        )
        self._cmd_callback(
            PACKET_CMD_SERVO,
            pack_servo_command(
                DEFAULT_HORIZONTAL_SERVO_ID,
                self._horizontal_servo_raw,
                self._servo_duration_ms,
            ),
        )
        self._cmd_callback(
            PACKET_CMD_SERVO,
            pack_servo_command(
                DEFAULT_GRIPPER_ROTATE_SERVO_ID,
                self._gripper_rotation_servo_raw,
                self._servo_duration_ms,
            ),
        )
        self._cmd_callback(
            PACKET_CMD_SERVO,
            pack_servo_command(
                DEFAULT_LIFT_GRIPPER_SERVO_ID,
                self._gripper_servo_raw,
                self._servo_duration_ms,
            ),
        )
