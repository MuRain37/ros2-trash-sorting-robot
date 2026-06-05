import struct
import sys
import threading
import math
from pathlib import Path

from std_msgs.msg import Float64, Float64MultiArray


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import robot_hardware_bridge.arm_bridge as arm_bridge_module  # noqa: E402
from robot_hardware_bridge.arm_bridge import ArmBridge  # noqa: E402
from robot_hardware_bridge.lift_dump_bridge import (  # noqa: E402
    LiftDumpBridge,
    LiftDumpCommand,
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


def test_arm_bridge_limits_servo_send_rate_to_10hz(monkeypatch):
    sent = []
    now = 0.0
    node = FakeNode()
    bridge = ArmBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
        send_rate=10.0,
    )
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

    for _ in range(20):
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

    for open_ratio in [0.0, 0.5, 1.0]:
        msg = Float64()
        msg.data = open_ratio
        bridge._on_gripper_command(msg)

    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO] * 3
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent] == [
        6, 6, 6
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent] == [
        700, 425, 150
    ]
    assert [unpack_servo_payload(payload)[3] for cmd, payload in sent] == [
        100, 100, 100
    ]


def test_arm_bridge_gripper_command_rejects_out_of_range_ratio():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64()
    msg.data = 1.1

    bridge._on_gripper_command(msg)

    assert sent == []
    assert node.logger.warnings[-1] == (
        'Invalid gripper command: open ratio 1.1 not in [0.0, 1.0]'
    )


def make_lift_dump_request(
    lift_position,
    horizontal_rotation,
    gripper_rotation,
    gripper_open_ratio,
):
    request = LiftDumpCommand.Request()
    request.lift_position = lift_position
    request.horizontal_rotation = horizontal_rotation
    request.gripper_rotation = gripper_rotation
    request.gripper_open_ratio = gripper_open_ratio
    return request


def test_lift_dump_bridge_sends_default_targets_on_startup():
    sent = []
    node = FakeNode()

    LiftDumpBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))

    assert [cmd for cmd, payload in sent] == [
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
    ]
    assert unpack_stepper_payload(sent[0][1]) == 0.0
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent[1:]] == [
        7, 8, 9
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent[1:]] == [
        500, 500, 700
    ]


def test_lift_dump_bridge_service_updates_targets_for_timer_send():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    sent.clear()
    request = make_lift_dump_request(
        0.5,
        0.0,
        0.0,
        0.5,
    )
    response = LiftDumpCommand.Response()

    result = bridge._on_command(request, response)
    bridge._publish_current_targets()

    assert result.success is True
    assert result.message == 'lift-dump command accepted'
    assert [cmd for cmd, payload in sent] == [
        PACKET_CMD_STEPPER,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
        PACKET_CMD_SERVO,
    ]
    assert unpack_stepper_payload(sent[0][1]) == 500.0
    assert [unpack_servo_payload(payload)[1] for cmd, payload in sent[1:]] == [
        7, 8, 9
    ]
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent[1:]] == [
        500, 500, 425
    ]


def test_lift_dump_bridge_rejects_invalid_command_without_updating_targets():
    sent = []
    node = FakeNode()
    bridge = LiftDumpBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
    )
    valid_request = make_lift_dump_request(0.5, 0.0, 0.0, 0.5)
    bridge._on_command(valid_request, LiftDumpCommand.Response())
    sent.clear()
    invalid_request = make_lift_dump_request(1.1, 0.0, 0.0, 1.0)
    response = LiftDumpCommand.Response()

    result = bridge._on_command(invalid_request, response)
    bridge._publish_current_targets()

    assert result.success is False
    assert result.message == (
        'lift position out of range: 1.1 not in [0.0, 1.0]'
    )
    assert unpack_stepper_payload(sent[0][1]) == 500.0
    assert [unpack_servo_payload(payload)[2] for cmd, payload in sent[1:]] == [
        500, 500, 425
    ]


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
