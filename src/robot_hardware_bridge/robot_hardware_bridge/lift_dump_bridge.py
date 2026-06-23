"""升降-倾倒结构桥接模块：步进电机与三路舵机的开环控制."""

import math
import threading
import time

from robot_interfaces.srv import (
    DumpTrash,
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
HORIZONTAL_MAX_DEGREES = 180.0       # 最大转动角度（度）
HORIZONTAL_DIRECTION = 1.0           # 转动方向（1.0 正向, -1.0 反向）

# 8 号舵机：夹爪旋转 (Gripper Rotation)
GRIPPER_ROTATE_SERVO_ID = 8
GRIPPER_ROTATE_MAX_DEGREES = 180.0   # 最大转动角度（度）
GRIPPER_ROTATE_DIRECTION = 1.0       # 转动方向（1.0 正向, -1.0 反向）

# 9 号舵机：夹爪开度 (Gripper Open/Close)
LIFT_GRIPPER_SERVO_ID = 9
LIFT_GRIPPER_OPEN_RAW = 500.0       # 夹爪全开时的 raw 值
LIFT_GRIPPER_CLOSE_RAW = 585.0       # 夹爪扣紧时的 raw 值

# 舵机通用默认值
SERVO_DURATION_MS = 100.0            # 默认运动时长（毫秒）
DUMP_SERVO_DURATION_MS = 2000.0       # 倒垃圾流程舵机运动时长
DUMP_GRIPPER_DURATION_MS = 1000.0     # 倒垃圾流程夹爪运动时长
DUMP_STEPPER_WAIT_SEC = 2.8          # 步进电机开环动作等待时间
DUMP_SETTLE_SEC = 0.1                # 每步动作后的额外稳定时间

SERVO_CENTER = 500.0                         # 舵机中位 raw 值
SERVO_RAW_MIN = 0.0                          # 舵机 raw 最小值
SERVO_RAW_MAX = 1000.0                       # 舵机 raw 最大值

# 倒垃圾流程默认 raw 位置。步进电机是绝对步数，现场需按机构行程标定。
DUMP_BUCKET_1_RAW = 883.0
DUMP_BUCKET_2_RAW = 110.0


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
        horizontal_rotation_service='set_lift_horizontal_rotation',
        gripper_rotation_service='set_lift_gripper_rotation',
        gripper_service='set_lift_gripper',
        dump_trash_service='dump_trash',
        cmd_callback=None,
        servo_duration_ms=SERVO_DURATION_MS,
        dump_gripper_duration_ms=DUMP_GRIPPER_DURATION_MS,
        dump_stepper_wait_sec=DUMP_STEPPER_WAIT_SEC,
        dump_settle_sec=DUMP_SETTLE_SEC,
        sleep_fn=time.sleep,
    ):
        self._node = node
        self._cmd_callback = cmd_callback
        self._servo_duration_ms = float(servo_duration_ms)
        self._dump_gripper_duration_ms = float(dump_gripper_duration_ms)
        self._dump_stepper_wait_sec = float(dump_stepper_wait_sec)
        self._dump_settle_sec = float(dump_settle_sec)
        self._sleep = sleep_fn
        self._dump_lock = threading.Lock()

        self._target_steps = 0.0
        self._horizontal_servo_raw = SERVO_CENTER
        self._gripper_rotation_servo_raw = SERVO_CENTER
        self._gripper_servo_raw = LIFT_GRIPPER_OPEN_RAW

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
        self._dump_trash_service = node.create_service(
            DumpTrash,
            dump_trash_service,
            self._on_dump_trash_command,
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
                HORIZONTAL_MAX_DEGREES,
                HORIZONTAL_DIRECTION,
            )
            duration_ms = float(request.duration_ms)
            if duration_ms < 0.0:
                raise ValueError(f'duration_ms must be >= 0: {duration_ms}')
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            HORIZONTAL_SERVO_ID,
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
                GRIPPER_ROTATE_MAX_DEGREES,
                GRIPPER_ROTATE_DIRECTION,
            )
            duration_ms = float(request.duration_ms)
            if duration_ms < 0.0:
                raise ValueError(f'duration_ms must be >= 0: {duration_ms}')
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._send_servo_target(
            GRIPPER_ROTATE_SERVO_ID,
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

    def _on_dump_trash_command(self, request, response):
        """接收垃圾桶编号，执行完整升降-夹取-倾倒流程."""
        try:
            bucket_id = self._normalize_bucket_id(request.bucket_id)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._cmd_callback:
            response.success = False
            response.message = 'command callback unavailable'
            return response

        if not self._dump_lock.acquire(blocking=False):
            response.success = False
            response.message = 'dump trash sequence is already running'
            return response

        try:
            self._execute_dump_trash_sequence(bucket_id)
        except Exception as e:  # pragma: no cover - defensive ROS callback log
            response.success = False
            response.message = f'dump trash sequence failed: {e}'
            return response
        finally:
            self._dump_lock.release()

        response.success = True
        response.message = (f'dump trash sequence completed for bucket {bucket_id}')
        return response

    def _execute_dump_trash_sequence(self, bucket_id):
        """按固定顺序执行倒垃圾动作."""
        bucket_raw = self._bucket_raw(bucket_id)

        self._move_lift(665.0)
        self._move_horizontal(bucket_raw)
        self._move_lift(75.0)
        self._move_gripper(LIFT_GRIPPER_CLOSE_RAW)
        self._move_lift(665.0)

        self._move_horizontal(500.0)
        self._move_gripper_rotation(0.0)
        self._move_gripper_rotation(500.0)

        self._move_horizontal(bucket_raw)
        self._move_lift(75.0)
        self._move_gripper(LIFT_GRIPPER_OPEN_RAW)

        self._move_lift(665.0)
        self._move_horizontal(500.0)
        self._move_lift(75.0)

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

    def _move_lift(self, target_steps):
        """发送步进电机目标并按开环等待时间等待."""
        self._send_stepper_target(target_steps)
        self._target_steps = target_steps
        self._sleep_after_stepper()

    def _move_horizontal(self, raw_position):
        """移动 7 号水平旋转舵机到 raw 位置."""
        self._send_servo_target(HORIZONTAL_SERVO_ID,raw_position,DUMP_SERVO_DURATION_MS,)
        self._horizontal_servo_raw = raw_position
        self._sleep_after_servo(DUMP_SERVO_DURATION_MS)

    def _move_gripper_rotation(self, raw_position):
        """移动 8 号倾倒旋转舵机到 raw 位置."""
        self._send_servo_target(GRIPPER_ROTATE_SERVO_ID,raw_position,DUMP_SERVO_DURATION_MS,)
        self._gripper_rotation_servo_raw = raw_position
        self._sleep_after_servo(DUMP_SERVO_DURATION_MS)

    def _move_gripper(self, raw_position):
        """移动 9 号夹爪开合舵机到 raw 位置."""
        self._send_servo_target(LIFT_GRIPPER_SERVO_ID,raw_position,DUMP_SERVO_DURATION_MS,)
        self._gripper_servo_raw = raw_position
        self._sleep_after_servo(DUMP_SERVO_DURATION_MS)

    def _sleep_after_servo(self, duration_ms):
        wait_sec = max(0.0, duration_ms / 1000.0) + self._dump_settle_sec
        if wait_sec > 0.0:
            self._sleep(wait_sec)

    def _sleep_after_stepper(self):
        wait_sec = max(0.0, self._dump_stepper_wait_sec)
        if wait_sec > 0.0:
            self._sleep(wait_sec)

    def _normalize_bucket_id(self, bucket_id):
        bucket_id = int(bucket_id)
        if bucket_id not in (1, 2):
            raise ValueError(f'bucket_id must be 1 or 2: {bucket_id}')
        return bucket_id

    def _bucket_raw(self, bucket_id):
        if bucket_id == 1:
            return DUMP_BUCKET_1_RAW
        return DUMP_BUCKET_2_RAW

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

    def _validate_non_negative(self, name, value):
        self._validate_finite(name, value)
        if value < 0.0:
            raise ValueError(f'{name} must be >= 0: {value}')
