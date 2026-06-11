from rclpy.action import GoalResponse

from robot_arm.arm_moveit_controller import ZERO_ANGLES
from robot_arm.arm_preset_action import ArmPresetActionServer
from robot_arm.arm_presets import preset_to_joint_positions
from robot_interfaces.action import ArmPreset


class FakeLogger:

    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


class FakeMoveIt:

    def __init__(self, success=True, final_angles=None, message='ok'):
        self.success = success
        self.final_angles = final_angles or [1.0, 2.0, 3.0, 4.0, 5.0]
        self.message = message
        self.requests = []

    def move_to_joint_positions(self, joint_positions, feedback_callback=None):
        self.requests.append(list(joint_positions))
        if feedback_callback is not None:
            feedback_callback(0.5, [0.0] * 5)
        return self.success, self.final_angles, self.message


class FakeGoalHandle:

    def __init__(self, action_type, cancel_requested=False):
        self.request = ArmPreset.Goal()
        self.request.action_type = action_type
        self.is_cancel_requested = cancel_requested
        self.feedback = []
        self.status = None

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.status = 'succeed'

    def abort(self):
        self.status = 'abort'

    def canceled(self):
        self.status = 'canceled'


def make_server(fake_moveit):
    server = ArmPresetActionServer.__new__(ArmPresetActionServer)
    server._moveit = fake_moveit
    server.get_logger = lambda: FakeLogger()
    return server


def test_arm_preset_action_accepts_known_action():
    server = make_server(FakeMoveIt())
    goal = ArmPreset.Goal()
    goal.action_type = ArmPreset.Goal.ARM_CENTER

    assert server._handle_arm_preset_goal(goal) == GoalResponse.ACCEPT


def test_arm_preset_action_rejects_unknown_action_without_moveit_call():
    fake_moveit = FakeMoveIt()
    server = make_server(fake_moveit)
    goal = ArmPreset.Goal()
    goal.action_type = 99

    assert server._handle_arm_preset_goal(goal) == GoalResponse.REJECT
    assert fake_moveit.requests == []


def test_arm_preset_action_executes_valid_action():
    fake_moveit = FakeMoveIt()
    server = make_server(fake_moveit)
    goal_handle = FakeGoalHandle(ArmPreset.Goal.ARM_CENTER)

    result = server._execute_arm_preset(goal_handle)

    assert result.success is True
    assert result.message == 'ok'
    assert list(result.final_angles) == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert fake_moveit.requests == [
        preset_to_joint_positions(ArmPreset.Goal.ARM_CENTER)
    ]
    assert [feedback.progress for feedback in goal_handle.feedback] == [
        0.0,
        0.5,
        1.0,
    ]
    assert goal_handle.status == 'succeed'


def test_arm_preset_action_returns_moveit_failure_message():
    fake_moveit = FakeMoveIt(success=False, final_angles=list(ZERO_ANGLES), message='failed')
    server = make_server(fake_moveit)
    goal_handle = FakeGoalHandle(ArmPreset.Goal.LEFT_BUCKET)

    result = server._execute_arm_preset(goal_handle)

    assert result.success is False
    assert result.message == 'failed'
    assert list(result.final_angles) == [0.0] * 5
    assert goal_handle.status == 'abort'
