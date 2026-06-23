import struct
import sys
import threading
import math
from pathlib import Path

from robot_interfaces.srv import (
    DumpTrash,
    SetGripper,
    SetLiftSteps,
    SetServoAngle,
)
from std_msgs.msg import Float64MultiArray


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import robot_hardware_bridge.arm_bridge as arm_bridge_module  # noqa: E402
from robot_hardware_bridge.arm_bridge import ArmBridge  # noqa: E402
from robot_hardware_bridge.lift_dump_bridge import (  # noqa: E402
    LiftDumpBridge,
    gripper_open_ratio_to_servo_raw,
)
from robot_hardware_bridge.motor_bridge import MotorBridge  # noqa: E402
from robot_hardware_bridge.packet_protocol import (  # noqa: E402
    PACKET_CMD_MOTOR,
    PACKET_CMD_SERVO,
    PACKET_CMD_STEPPER,
    PACKET_TYPE_MOTOR_STATE,
    PACKET_TYPE_SERVO_STATE,
    build_frame,
    pack_motor_command,
    pack_servo_command,
    pack_stepper_command,
)
from robot_hardware_bridge.serial_feedback_dispatcher import (  # noqa: E402
    SerialFeedbackDispatcher,
)
from robot_hardware_bridge.serial_port import SerialPort  # noqa: E402


class FakeLogger:

    def __init__(self):
        self.warnings = []
        self.infos = []

    def info(self, msg):
        self.infos.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)


class FakePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:

    def __init__(self):
        self.logger = FakeLogger()
        self.publisher = FakePublisher()
        self.services = {}
        self.timers = []

    def get_logger(self):
        return self.logger

    def create_publisher(self, msg_type, topic, depth):
        return self.publisher

    def create_subscription(self, msg_type, topic, callback, depth):
        return callback

    def create_service(self, srv_type, topic, callback):
        self.services[topic] = callback
        return callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return callback


def make_serial_with_buffer(data):
    port = SerialPort.__new__(SerialPort)
    port._rx_buf = bytearray(data)
    port._lock = threading.Lock()
    port._debug = False
    return port


def unpack_servo_payload(payload):
    return struct.unpack('<ffff', payload)


def unpack_stepper_payload(payload):
    return struct.unpack('<f', payload)[0]


def test_build_motor_frame_checksum():
    payload = pack_motor_command([8.8, 0.0, 8.8, 0.0])
    frame = build_frame(PACKET_CMD_MOTOR, payload)

    assert frame[:4] == bytes([0xAA, 0xBB, PACKET_CMD_MOTOR, 0x10])
    assert frame[4:-1] == payload
    assert frame[-1] == (sum(frame[:-1]) & 0xFF)


def test_build_servo_set_position_frame_matches_protocol_example():
    payload = pack_servo_command(1, 500.0, 1000.0)
    frame = build_frame(PACKET_CMD_SERVO, payload)

    assert frame == bytes.fromhex(
        'AA BB 04 10 '
        '00 00 80 3F '
        '00 00 80 3F '
        '00 00 FA 43 '
        '00 00 7A 44 '
        'F2'
    )


def test_pack_stepper_command_uses_single_float32_payload():
    payload = pack_stepper_command(1000.0)

    assert len(payload) == 4
    assert unpack_stepper_payload(payload) == 1000.0


def test_read_frames_handles_noise_partial_and_multiple_frames():
    encoder_payload = pack_motor_command([1.0, 2.0, 3.0, 4.0])
    servo_payload = pack_servo_command(2, 1.5)
    first = build_frame(PACKET_TYPE_MOTOR_STATE, encoder_payload)
    second = build_frame(PACKET_CMD_SERVO, servo_payload)
    port = make_serial_with_buffer(b'noise' + first + second[:3])

    assert port.read_frames() == [
        (PACKET_TYPE_MOTOR_STATE, encoder_payload)
    ]
    assert bytes(port._rx_buf) == second[:3]

    port._rx_buf.extend(second[3:])
    assert port.read_frames() == [(PACKET_CMD_SERVO, servo_payload)]


def test_read_frames_resyncs_after_bad_checksum():
    bad = bytearray(
        build_frame(
            PACKET_TYPE_MOTOR_STATE,
            pack_motor_command([1.0, 2.0, 3.0, 4.0]),
        )
    )
    bad[-1] ^= 0xFF
    motor_payload = pack_motor_command([4.0, 3.0, 2.0, 1.0])
    good = build_frame(PACKET_TYPE_MOTOR_STATE, motor_payload)
    port = make_serial_with_buffer(bytes(bad) + good)

    assert port.read_frames() == [(PACKET_TYPE_MOTOR_STATE, motor_payload)]


def test_motor_bridge_sends_binary_motor_frame():
    sent = []
    node = FakeNode()
    bridge = MotorBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [1.0, 2.0, 3.0, 4.0]

    bridge._on_cmd(msg)

    assert sent == [(PACKET_CMD_MOTOR, pack_motor_command(msg.data))]


def test_arm_bridge_sends_center_command_as_servo_raw_positions():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

    bridge._on_command(msg)

    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO] * 5
    assert [unpack_servo_payload(payload)[0] for cmd, payload in sent] == [
        1.0, 1.0, 1.0, 1.0, 1.0
    ]
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent] == [
        1, 2, 3, 4, 5
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        500, 500, 500, 500, 500
    ]
    assert [unpack_servo_payload(payload)[3] for cmd, payload in sent] == [
        100, 100, 100, 100, 100
    ]


def test_arm_bridge_sends_positive_limit_as_servo_raw_positions():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [
        math.radians(180.0),
        math.radians(120.0),
        math.radians(120.0),
        math.radians(120.0),
        math.radians(120.0),
    ]

    bridge._on_command(msg)

    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        0, 0, 0, 0, 1000
    ]


def test_arm_bridge_sends_negative_limit_as_servo_raw_positions():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [
        math.radians(-180.0),
        math.radians(-120.0),
        math.radians(-120.0),
        math.radians(-120.0),
        math.radians(-120.0),
    ]

    bridge._on_command(msg)

    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        1000, 1000, 1000, 1000, 0
    ]


def test_arm_bridge_rejects_out_of_range_command():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, math.radians(121.0), 0.0, 0.0]

    bridge._on_command(msg)

    assert sent == []
    assert node.logger.warnings


def test_arm_bridge_skips_unchanged_servo_targets():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

    bridge._on_command(msg)
    bridge._on_command(msg)

    assert len(sent) == 5


def test_arm_bridge_limits_changed_servo_targets_to_10hz(monkeypatch):
    sent = []
    now = 0.0
    node = FakeNode()
    bridge = ArmBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
        send_rate=10.0,
    )
    for index in range(20):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, math.radians(index)]
        monkeypatch.setattr(
            arm_bridge_module.time,
            'monotonic',
            lambda value=now: value,
        )
        bridge._on_command(msg)
        now += 0.01

    assert len(sent) == 10


def test_arm_bridge_gripper_command_maps_open_ratio_to_servo_raw():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    assert node.services['arm_gripper'] == bridge._on_gripper_command

    for open_ratio in [0.0, 0.5, 1.0]:
        request = SetGripper.Request()
        request.open_ratio = open_ratio
        request.duration_ms = 250.0
        response = bridge._on_gripper_command(
            request,
            SetGripper.Response(),
        )
        assert response.success is True
        assert response.message == 'gripper command accepted'

    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO] * 3
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent] == [
        6, 6, 6
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        700, 350, 0
    ]
    assert [unpack_servo_payload(payload)[3] for cmd, payload in sent] == [
        250, 250, 250
    ]


def test_arm_bridge_gripper_command_rejects_out_of_range_ratio():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    request = SetGripper.Request()
    request.open_ratio = 1.1
    request.duration_ms = 100.0

    response = bridge._on_gripper_command(request, SetGripper.Response())

    assert sent == []
    assert response.success is False
    assert response.message == 'open ratio 1.1 not in [0.0, 1.0]'


def make_lift_steps_request(
    target_steps,
):
    request = SetLiftSteps.Request()
    request.target_steps = target_steps
    return request


def make_servo_angle_request(angle, duration_ms=100.0):
    request = SetServoAngle.Request()
    request.angle = angle
    request.duration_ms = duration_ms
    return request


def make_gripper_request(open_ratio, duration_ms=100.0):
    request = SetGripper.Request()
    request.open_ratio = open_ratio
    request.duration_ms = duration_ms
    return request


def make_dump_trash_request(bucket_id):
    request = DumpTrash.Request()
    request.bucket_id = bucket_id
    return request


def test_lift_dump_bridge_does_not_send_default_targets_on_startup():
    sent = []
    node = FakeNode()

    LiftDumpBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))

    assert sent == []


def test_lift_dump_bridge_registers_dump_trash_service():
    node = FakeNode()

    LiftDumpBridge(node, cmd_callback=lambda cmd, payload: None)

    assert 'dump_trash' in node.services


def test_lift_dump_bridge_lift_steps_service_sends_stepper_once():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    request = make_lift_steps_request(-250.0)
    response = SetLiftSteps.Response()

    result = bridge._on_lift_steps_command(request, response)

    assert result.success is True
    assert result.message == 'lift steps command accepted'
    assert [cmd for cmd, payload in sent] == [PACKET_CMD_STEPPER]
    assert unpack_stepper_payload(sent[0][1]) == -250.0


def test_lift_dump_bridge_repeated_lift_steps_service_sends_each_request():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    request = make_lift_steps_request(-250.0)
    response = SetLiftSteps.Response()

    result = bridge._on_lift_steps_command(request, response)
    bridge._on_lift_steps_command(request, SetLiftSteps.Response())

    assert result.success is True
    assert result.message == 'lift steps command accepted'
    assert [cmd for cmd, payload in sent] == [
        PACKET_CMD_STEPPER,
        PACKET_CMD_STEPPER,
    ]
    assert unpack_stepper_payload(sent[0][1]) == -250.0
    assert unpack_stepper_payload(sent[1][1]) == -250.0


def test_lift_dump_bridge_horizontal_rotation_service_sends_servo_once():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    request = make_servo_angle_request(0.0, duration_ms=350.0)
    response = SetServoAngle.Response()

    result = bridge._on_horizontal_rotation_command(request, response)

    assert result.success is True
    assert result.message == 'lift horizontal rotation command accepted'
    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO]
    assert unpack_servo_payload(sent[0][1])[1] == 7
    assert unpack_servo_payload(sent[0][1])[2] == 500
    assert unpack_servo_payload(sent[0][1])[3] == 350


def test_lift_dump_bridge_gripper_rotation_service_sends_servo_once():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    request = make_servo_angle_request(0.0, duration_ms=360.0)
    response = SetServoAngle.Response()

    result = bridge._on_gripper_rotation_command(request, response)

    assert result.success is True
    assert result.message == 'lift gripper rotation command accepted'
    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO]
    assert unpack_servo_payload(sent[0][1])[1] == 8
    assert unpack_servo_payload(sent[0][1])[2] == 500
    assert unpack_servo_payload(sent[0][1])[3] == 360


def test_lift_dump_bridge_gripper_service_sends_servo_once():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    request = make_gripper_request(0.5, duration_ms=370.0)
    response = SetGripper.Response()

    result = bridge._on_gripper_command(request, response)

    assert result.success is True
    assert result.message == 'lift gripper command accepted'
    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO]
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent] == [
        9
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        gripper_open_ratio_to_servo_raw(0.5)
    ]
    assert [unpack_servo_payload(payload)[3] for cmd, payload in sent] == [
        370
    ]


def test_lift_dump_bridge_dump_trash_service_runs_bucket_1_sequence():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
        dump_stepper_wait_sec=0.0,
        dump_settle_sec=0.0,
        sleep_fn=lambda seconds: None,
    )
    request = make_dump_trash_request(1)

    result = bridge._on_dump_trash_command(
        request,
        DumpTrash.Response(),
    )

    assert result.success is True
    assert result.message == 'dump trash sequence completed for bucket 1'
    assert [cmd for cmd, payload in sent] == [
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
    ]

    servo_commands = [
        unpack_servo_payload(payload)
        for cmd, payload in sent
        if cmd == PACKET_CMD_SERVO
    ]
    stepper_commands = [
        unpack_stepper_payload(payload)
        for cmd, payload in sent
        if cmd == PACKET_CMD_STEPPER
    ]

    assert [(command[1], command[2]) for command in servo_commands] == [
        (9, 1000),
        (8, 500),
        (7, 500),
        (7, 100),
        (9, 520),
        (7, 500),
        (8, 0),
        (8, 500),
        (7, 100),
        (9, 1000),
        (7, 500),
        (8, 500),
        (9, 1000),
    ]
    assert stepper_commands == [-250.0, 0.0, -250.0, 0.0]


def test_lift_dump_bridge_dump_trash_service_uses_bucket_2_position():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
        dump_stepper_wait_sec=0.0,
        dump_settle_sec=0.0,
        sleep_fn=lambda seconds: None,
    )

    result = bridge._on_dump_trash_command(
        make_dump_trash_request(2),
        DumpTrash.Response(),
    )

    servo_positions = [
        (unpack_servo_payload(payload)[1], unpack_servo_payload(payload)[2])
        for cmd, payload in sent
        if cmd == PACKET_CMD_SERVO
    ]
    assert result.success is True
    assert (7, 900) in servo_positions
    assert (7, 100) not in servo_positions


def test_lift_dump_bridge_dump_trash_service_rejects_invalid_bucket():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )

    result = bridge._on_dump_trash_command(
        make_dump_trash_request(3),
        DumpTrash.Response(),
    )

    assert result.success is False
    assert result.message == 'bucket_id must be 1 or 2: 3'
    assert sent == []


def test_lift_dump_bridge_rejects_invalid_stepper_target():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    invalid_request = make_lift_steps_request(float('inf'))
    response = SetLiftSteps.Response()

    result = bridge._on_lift_steps_command(invalid_request, response)

    assert result.success is False
    assert result.message == 'target steps must be finite: inf'
    assert sent == []


def test_lift_dump_bridge_rejects_invalid_servo_angle():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    invalid_request = make_servo_angle_request(math.radians(181.0))
    response = SetServoAngle.Response()

    result = bridge._on_horizontal_rotation_command(invalid_request, response)

    assert result.success is False
    assert result.message == (
        'angle out of range: 181.0 not in [-180.0, 180.0] deg'
    )
    assert sent == []


def test_lift_dump_bridge_rejects_invalid_lift_gripper_target():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    invalid_request = make_gripper_request(1.1)
    response = SetGripper.Response()

    result = bridge._on_gripper_command(invalid_request, response)

    assert result.success is False
    assert result.message == (
        'gripper open ratio out of range: 1.1 not in [0.0, 1.0]'
    )
    assert sent == []


def test_feedback_dispatcher_publishes_feedback():
    node = FakeNode()
    hardware = type('FakeHardwareNode', (), {})()
    hardware._debug = False
    hardware._motor_open_loop_control = False
    hardware._arm_open_loop_control = False
    hardware._motor = MotorBridge(node)
    hardware._arm = ArmBridge(node)
    hardware.get_logger = lambda: FakeLogger()
    dispatcher = SerialFeedbackDispatcher(hardware)

    dispatcher.handle_frame(
        PACKET_TYPE_MOTOR_STATE,
        pack_motor_command([1.0, 2.0, 3.0, 4.0]),
    )

    assert len(node.publisher.messages) == 1
    assert list(node.publisher.messages[-1].data) == [1.0, 2.0, 3.0, 4.0]


def test_feedback_dispatcher_publishes_five_joint_arm_feedback():
    node = FakeNode()
    hardware = type('FakeHardwareNode', (), {})()
    hardware._debug = False
    hardware._motor_open_loop_control = False
    hardware._arm_open_loop_control = False
    hardware._motor = MotorBridge(node)
    hardware._arm = ArmBridge(node)
    hardware.get_logger = lambda: FakeLogger()
    dispatcher = SerialFeedbackDispatcher(hardware)

    dispatcher.handle_frame(
        PACKET_TYPE_SERVO_STATE, pack_servo_command(1, 500)
    )
    dispatcher.handle_frame(
        PACKET_TYPE_SERVO_STATE, pack_servo_command(2, 0)
    )
    dispatcher.handle_frame(
        PACKET_TYPE_SERVO_STATE, pack_servo_command(5, 1000)
    )
    dispatcher.handle_frame(
        PACKET_TYPE_SERVO_STATE, pack_servo_command(6, 500)
    )
    dispatcher.handle_frame(
        PACKET_TYPE_SERVO_STATE, pack_servo_command(7, 500)
    )

    assert list(node.publisher.messages[0].data) == [0, 0, 0, 0, 0]
    assert list(node.publisher.messages[1].data) == [
        0, math.radians(120), 0, 0, 0
    ]
    assert list(node.publisher.messages[2].data) == [
        0, math.radians(120), 0, 0, math.radians(120)
    ]
    assert len(node.publisher.messages) == 3
    assert node.logger.warnings[-1] == (
        'Ignoring servo feedback with invalid id: 7'
    )
