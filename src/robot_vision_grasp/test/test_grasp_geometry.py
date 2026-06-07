from robot_vision_grasp.grasp_geometry import get_gripper_yaw
from robot_vision_grasp.grasp_geometry import get_ik_pitch
from robot_vision_grasp.grasp_geometry import gripper_close_open_ratio


def test_gripper_ratio_is_clamped():
    assert 0.0 <= gripper_close_open_ratio('pen') <= 1.0
    assert 0.0 <= gripper_close_open_ratio('bottle') <= 1.0


def test_wide_objects_close_less_than_pen():
    assert gripper_close_open_ratio('bottle') > gripper_close_open_ratio('pen')


def test_known_pitch_rules():
    pitch, _axis, is_up = get_ik_pitch('paper')
    assert pitch == 65.0
    assert is_up is True

    pitch, _axis, is_up = get_ik_pitch('pen')
    assert pitch == 90.0
    assert is_up is False


def test_gripper_yaw_only_for_fallen_objects():
    assert get_gripper_yaw('bottle', 30.0, True) == 0.0
    assert get_gripper_yaw('bottle', 30.0, False) == 30.0
    assert get_gripper_yaw('paper', 30.0, False) == 0.0
