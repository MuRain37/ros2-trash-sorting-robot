import numpy as np

from robot_interfaces.action import ArmPreset
from robot_vision_grasp import vision_grasp_node as node_module


def test_paper_gripper_rotation_is_forced_to_zero(monkeypatch):
    node = node_module.VisionGraspNode.__new__(node_module.VisionGraspNode)
    node.get_parameter = lambda _name: type(
        'ParameterValue',
        (),
        {'value': 'recyclable'},
    )()

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
    expectations = {
        'recyclable': {
            'pen': ArmPreset.Goal.LEFT_BUCKET,
            'paper': ArmPreset.Goal.LEFT_BUCKET,
            'yan': ArmPreset.Goal.RIGHT_BUCKET,
            'battry': ArmPreset.Goal.RIGHT_BUCKET,
            'pit': ArmPreset.Goal.RIGHT_BUCKET,
        },
        'dry_wet': {
            'pen': ArmPreset.Goal.LEFT_BUCKET,
            'paper': ArmPreset.Goal.LEFT_BUCKET,
            'yan': ArmPreset.Goal.LEFT_BUCKET,
            'battry': ArmPreset.Goal.LEFT_BUCKET,
            'pit': ArmPreset.Goal.RIGHT_BUCKET,
        },
        'trash_lost': {
            'pen': ArmPreset.Goal.RIGHT_BUCKET,
            'paper': ArmPreset.Goal.LEFT_BUCKET,
            'yan': ArmPreset.Goal.LEFT_BUCKET,
            'battry': ArmPreset.Goal.RIGHT_BUCKET,
            'pit': ArmPreset.Goal.LEFT_BUCKET,
        },
    }

    for mode, class_buckets in expectations.items():
        for class_name, bucket_id in class_buckets.items():
            assert node_module.VisionGraspNode._bucket_preset_for_class(
                class_name,
                mode,
            ) == bucket_id
