"""Preset arm positions expressed as raw servo values."""

import math
from pathlib import Path

import yaml


ACTION_ARM_CENTER = 0
ACTION_LEFT_BUCKET = 1
ACTION_RIGHT_BUCKET = 2

ACTION_NAMES = {
    ACTION_ARM_CENTER: 'arm_center',
    ACTION_LEFT_BUCKET: 'left_bucket',
    ACTION_RIGHT_BUCKET: 'right_bucket',
}

SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0
JOINT_MAX_DEGREES = (180.0, 120.0, 120.0, 120.0, 120.0)
JOINT_DIRECTIONS = (-1.0, -1.0, -1.0, -1.0, 1.0)
ARM_CENTER_JOINTS = (
    'arm_base_joint', 'arm_shoulder_joint', 'arm_elbow_joint',
    'arm_wrist_joint', 'arm_tool_rotate_joint',
)


def preset_name(action_type):
    """Return a stable name for a preset action type."""
    return ACTION_NAMES.get(int(action_type), f'unknown_{int(action_type)}')


def joint_position_to_servo_raw(index, joint_position):
    """Convert one MoveIt joint angle in radians to a raw servo position."""
    degrees = math.degrees(float(joint_position))
    raw = SERVO_CENTER + JOINT_DIRECTIONS[index] * degrees * (
        SERVO_CENTER / JOINT_MAX_DEGREES[index]
    )
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, raw))


def _load_arm_center_raw():
    config_path = (
        Path(__file__).resolve().parents[2]
        / 'robot_description' / 'config' / 'arm_initial_positions.yaml'
    )
    positions = yaml.safe_load(config_path.read_text())['initial_positions']
    return [
        joint_position_to_servo_raw(index, positions[name])
        for index, name in enumerate(ARM_CENTER_JOINTS)
    ]


PRESET_SERVO_RAW = {
    ACTION_ARM_CENTER: _load_arm_center_raw(),
    ACTION_LEFT_BUCKET: [900.0, 600.0, 290.0, 100.0, 500.0],
    ACTION_RIGHT_BUCKET: [100.0, 600.0, 290.0, 100.0, 500.0],
}


def servo_raw_to_joint_position(index, raw_position):
    """Convert one raw servo position to a MoveIt joint angle in radians."""
    raw_position = float(raw_position)
    if not math.isfinite(raw_position):
        raise ValueError(
            f'servo{index + 1} raw must be finite: {raw_position}'
        )
    if not SERVO_RAW_MIN <= raw_position <= SERVO_RAW_MAX:
        raise ValueError(
            f'servo{index + 1} raw out of range: {raw_position} '
            f'not in [{SERVO_RAW_MIN}, {SERVO_RAW_MAX}]'
        )

    max_degrees = JOINT_MAX_DEGREES[index]
    direction = JOINT_DIRECTIONS[index]
    degrees = (raw_position - SERVO_CENTER) * (max_degrees / SERVO_CENTER)
    degrees *= direction
    return math.radians(degrees)


def servo_raw_to_joint_positions(raw_positions):
    """Convert five raw servo positions to five MoveIt joint angles."""
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
    """Look up a preset action and convert it to MoveIt joint angles."""
    action_type = int(action_type)
    if action_type not in PRESET_SERVO_RAW:
        valid = ', '.join(str(action) for action in sorted(PRESET_SERVO_RAW))
        raise ValueError(
            f'unknown action_type: {action_type}; valid values are {valid}'
        )
    return servo_raw_to_joint_positions(PRESET_SERVO_RAW[action_type])
