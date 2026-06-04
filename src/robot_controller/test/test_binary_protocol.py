import struct
import sys
import threading
import math
from pathlib import Path

from std_msgs.msg import Float64MultiArray


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import robot_controller.arm_bridge as arm_bridge_module  # noqa: E402
from robot_controller.arm_bridge import ArmBridge  # noqa: E402
from robot_controller.motor_bridge import MotorBridge  # noqa: E402
from robot_controller.packet_protocol import (  # noqa: E402
    PACKET_CMD_MOTOR,
    PACKET_CMD_SERVO,
    PACKET_TYPE_MOTOR_STATE,
    PACKET_TYPE_SERVO_STATE,
    pack_motor_command,
    pack_servo_command,
)
from robot_controller.serial_feedback_dispatcher import (  # noqa: E402
    SerialFeedbackDispatcher,
)
from robot_controller.serial_port import SerialPort, build_frame  # noqa: E402


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

    def get_logger(self):
        return self.logger

    def create_publisher(self, msg_type, topic, depth):
        return self.publisher

    def create_subscription(self, msg_type, topic, callback, depth):
        return callback


def make_serial_with_buffer(data):
    port = SerialPort.__new__(SerialPort)
    port._rx_buf = bytearray(data)
    port._lock = threading.Lock()
    port._debug = False
    return port


def unpack_servo_payload(payload):
    return struct.unpack('<ffff', payload)


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


def test_arm_bridge_print_only_logs_servo_targets_without_sending():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(
        node,
        cmd_callback=lambda cmd, payload: sent.append((cmd, payload)),
        print_only=True,
    )
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

    bridge._on_command(msg)

    assert sent == []
    assert node.logger.infos[-1] == (
        'Arm servo targets (print-only): '
        'servo1=500.0, servo2=500.0, servo3=500.0, '
        'servo4=500.0, servo5=500.0'
    )


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

    dispatcher.handle_frame(PACKET_TYPE_SERVO_STATE, pack_servo_command(1, 500))
    dispatcher.handle_frame(PACKET_TYPE_SERVO_STATE, pack_servo_command(2, 0))
    dispatcher.handle_frame(PACKET_TYPE_SERVO_STATE, pack_servo_command(5, 1000))
    dispatcher.handle_frame(PACKET_TYPE_SERVO_STATE, pack_servo_command(6, 500))

    assert list(node.publisher.messages[0].data) == [0, 0, 0, 0, 0]
    assert list(node.publisher.messages[1].data) == [
        0, math.radians(120), 0, 0, 0
    ]
    assert list(node.publisher.messages[2].data) == [
        0, math.radians(120), 0, 0, math.radians(120)
    ]
    assert len(node.publisher.messages) == 3
    assert node.logger.warnings[-1] == 'Ignoring servo feedback with invalid id: 6'
