"""机械臂预设姿态配置：用 5 个舵机 raw 值描述一个 MoveIt 关节目标."""

import math

ACTION_ARM_CENTER = 0
ACTION_LEFT_BUCKET = 1
ACTION_RIGHT_BUCKET = 2
ACTION_ARM_MAPPING = 3
ACTION_NAMES = {
    ACTION_ARM_CENTER: 'arm_center',
    ACTION_LEFT_BUCKET: 'left_bucket',
    ACTION_RIGHT_BUCKET: 'right_bucket',
    ACTION_ARM_MAPPING: 'arm_mapping',  
}

# 舵机 raw 值范围：中位 500，最小 0，最大 1000
SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0
BASE_JOINT_MAX_DEGREES = 180.0
OTHER_JOINT_MAX_DEGREES = 120.0
JOINT_MAX_DEGREES = (
    BASE_JOINT_MAX_DEGREES,
    OTHER_JOINT_MAX_DEGREES,
    OTHER_JOINT_MAX_DEGREES,
    OTHER_JOINT_MAX_DEGREES,
    OTHER_JOINT_MAX_DEGREES,
)
JOINT_DIRECTIONS = (-1.0, -1.0, -1.0, -1.0, 1.0)


def preset_name(action_type):
    """返回预设动作类型的可读名称."""
    return ACTION_NAMES.get(int(action_type), f'unknown_{int(action_type)}')


def joint_max_degrees(index):
    """返回指定关节的半角行程；舵机1与其他舵机分开映射."""
    if index == 0:
        return BASE_JOINT_MAX_DEGREES
    if 0 < index < len(JOINT_MAX_DEGREES):
        return OTHER_JOINT_MAX_DEGREES
    raise ValueError(f'unsupported arm joint index: {index}')


def joint_position_to_servo_raw(index, joint_position):
    """MoveIt 关节角（弧度）→ 舵机原始位置值."""
    degrees = math.degrees(float(joint_position))
    max_degrees = joint_max_degrees(index)
    raw = SERVO_CENTER + JOINT_DIRECTIONS[index] * degrees * (
        SERVO_CENTER / max_degrees
    )
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, raw))


PRESET_SERVO_RAW = {
    ACTION_ARM_CENTER: [500.0, 700.0, 100.0, 100.0, 500.0],
    ACTION_LEFT_BUCKET: [900.0, 650.0, 240.0, 150.0, 500.0],
    ACTION_RIGHT_BUCKET: [100.0, 650.0, 240.0, 150.0, 500.0],
    ACTION_ARM_MAPPING: [500.0, 650.0, 210.0, 160.0, 500.0],
}


def servo_raw_to_joint_position(index, raw_position):
    """单个舵机 raw 值 → MoveIt 关节角（弧度）."""
    raw_position = float(raw_position)
    if not math.isfinite(raw_position):
        raise ValueError(f'servo{index + 1} raw must be finite: {raw_position}')
    if not SERVO_RAW_MIN <= raw_position <= SERVO_RAW_MAX:
        raise ValueError(
            f'servo{index + 1} raw out of range: {raw_position} '
            f'not in [{SERVO_RAW_MIN}, {SERVO_RAW_MAX}]'
        )

    max_degrees = joint_max_degrees(index)
    direction = JOINT_DIRECTIONS[index]
    degrees = (raw_position - SERVO_CENTER) * (max_degrees / SERVO_CENTER)
    degrees *= direction
    return math.radians(degrees)


def servo_raw_to_joint_positions(raw_positions):
    """5 个舵机 raw 值 → 5 个 MoveIt 关节角（弧度）."""
    if len(raw_positions) != len(JOINT_MAX_DEGREES):
        raise ValueError(
            f'expected {len(JOINT_MAX_DEGREES)} servo raw values, '
            f'got {len(raw_positions)}'
        )
    return [
        servo_raw_to_joint_position(index, raw)
        for index, raw in enumerate(raw_positions)
    ]


def preset_to_joint_positions(action_type):
    """按动作类型查找预设 raw 值，并转换为 MoveIt 关节目标."""
    action_type = int(action_type)
    if action_type not in PRESET_SERVO_RAW:
        valid = ', '.join(str(action) for action in sorted(PRESET_SERVO_RAW))
        raise ValueError(
            f'unknown action_type: {action_type}; valid values are {valid}'
        )
    return servo_raw_to_joint_positions(PRESET_SERVO_RAW[action_type])
