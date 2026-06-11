import numpy as np

from robot_interfaces.action import ArmPreset
from robot_vision_grasp import vision_grasp_node as node_module


def test_paper_gripper_rotation_is_forced_to_zero(monkeypatch):
    node = node_module.VisionGraspNode.__new__(node_module.VisionGraspNode)

    monkeypatch.setattr(
        node_module.VisionGraspNode,
        '_lookup_tf_camera_transform',
        lambda self: (np.identity(3), np.zeros(3), 0.0),
    )

    def fail_axis_angle(*args, **kwargs):
        raise AssertionError('_get_axis_angle should not be called for paper')

    monkeypatch.setattr(node_module, '_get_axis_angle', fail_axis_angle)

    target = {
        'class_name': 'paper',
        'point_cam_m': np.array([0.1, 0.2, 0.3], dtype=float),
        'points_cam': None,
        'mask': None,
        'depth_m': None,
    }

    result = node._camera_target_to_arm_target(target)

    assert result['gripper_rotation_deg'] == 0.0


def test_bucket_mapping_matches_class_groups():
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('paper')
        == ArmPreset.Goal.LEFT_BUCKET
    )
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('yan')
        == ArmPreset.Goal.LEFT_BUCKET
    )
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('bottle')
        == ArmPreset.Goal.LEFT_BUCKET
    )
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('usb')
        == ArmPreset.Goal.RIGHT_BUCKET
    )
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('charger')
        == ArmPreset.Goal.RIGHT_BUCKET
    )
    assert (
        node_module.VisionGraspNode._bucket_preset_for_class('pen')
        == ArmPreset.Goal.RIGHT_BUCKET
    )
