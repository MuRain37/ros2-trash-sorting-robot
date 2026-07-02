import math

from robot_manipulation.arm_scan_lock_action import ARM_JOINT_NAMES
from robot_manipulation.arm_scan_lock_action import ArmScanLockActionServer


class FakePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_server():
    server = ArmScanLockActionServer.__new__(ArmScanLockActionServer)
    server._model_joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5]
    server._hardware_joint_positions = [-0.1, -0.2, -0.3, -0.4, -0.5]
    server._trajectory_duration = 0.25
    server._model_pub = FakePublisher()
    server._hardware_pub = FakePublisher()
    return server


def test_scan_lock_publishes_model_feedback_and_hardware_trajectory():
    server = make_server()

    server._publish_lock_targets()

    assert list(server._model_pub.messages[-1].data) == [
        0.1, 0.2, 0.3, 0.4, 0.5,
    ]
    trajectory = server._hardware_pub.messages[-1]
    assert trajectory.joint_names == ARM_JOINT_NAMES
    assert list(trajectory.points[-1].positions) == [
        -0.1, -0.2, -0.3, -0.4, -0.5, 0.0, 0.0, 0.0,
    ]
    assert trajectory.points[-1].time_from_start.sec == 0
    assert math.isclose(
        trajectory.points[-1].time_from_start.nanosec,
        250_000_000,
    )


def test_fixed_joint_array_rejects_wrong_length():
    server = ArmScanLockActionServer.__new__(ArmScanLockActionServer)

    try:
        server._fixed_joint_array([0.0, 1.0])
    except ValueError as exc:
        assert 'expected 5 joint positions' in str(exc)
    else:
        raise AssertionError('expected ValueError')
