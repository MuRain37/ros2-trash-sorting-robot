import math

import pytest

from robot_arm.arm_moveit_controller import ArmMoveItController
from robot_manipulation.arm_presets import ACTION_ARM_CENTER
from robot_manipulation.arm_presets import preset_to_joint_positions
from robot_manipulation.arm_presets import servo_raw_to_joint_positions


def test_midpoint_raw_values_convert_to_zero_joint_positions():
    assert servo_raw_to_joint_positions([500, 500, 500, 500, 500]) == [0.0] * 5


def test_center_preset_is_a_valid_joint_target():
    joints = preset_to_joint_positions(ACTION_ARM_CENTER)

    assert len(joints) == 5


def test_servo_raw_values_convert_with_hardware_mapping():
    joints = servo_raw_to_joint_positions([1000, 1000, 1000, 1000, 1000])

    assert joints == [
        pytest.approx(math.radians(-180.0)),
        pytest.approx(math.radians(-120.0)),
        pytest.approx(math.radians(-120.0)),
        pytest.approx(math.radians(-120.0)),
        pytest.approx(math.radians(120.0)),
    ]


def test_servo_raw_values_reject_out_of_range_value():
    with pytest.raises(ValueError, match='servo3 raw out of range'):
        servo_raw_to_joint_positions([500, 500, 1001, 500, 500])


def test_unknown_preset_action_is_rejected():
    with pytest.raises(ValueError, match='unknown action_type: 99'):
        preset_to_joint_positions(99)


def test_moveit_joint_goal_appends_fixed_virtual_joints():
    controller = ArmMoveItController.__new__(ArmMoveItController)
    controller._group_name = 'plan'
    goal = controller._build_move_group_goal([0.1, 0.2, 0.3, 0.4, 0.5, 0, 0, 0])

    constraints = goal.request.goal_constraints[0].joint_constraints
    assert [constraint.joint_name for constraint in constraints] == [
        'arm_base_joint',
        'arm_shoulder_joint',
        'arm_elbow_joint',
        'arm_wrist_joint',
        'arm_tool_rotate_joint',
        'virtual_tool_roll_joint',
        'virtual_tool_pitch_joint',
        'virtual_tool_yaw_joint',
    ]
    assert [constraint.position for constraint in constraints[-3:]] == [0.0] * 3
