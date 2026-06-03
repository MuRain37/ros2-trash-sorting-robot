import struct
import sys
import threading
from pathlib import Path

from std_msgs.msg import Float64MultiArray


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from robot_controller.arm_bridge import ArmBridge  # noqa: E402
from robot_controller.motor_bridge import MotorBridge  # noqa: E402
from robot_controller.packet_protocol import (  # noqa: E402
    PACKET_CMD_MOTOR,
    PACKET_CMD_SERVO,
    PACKET_TYPE_MOTOR_STATE,
    pack_motor_command,
    pack_servo_command,
)
from robot_controller.serial_feedback_dispatcher import (  # noqa: E402
    SerialFeedbackDispatcher,
)
from robot_controller.serial_port import SerialPort, build_frame  # noqa: E402


class FakeLogger:

    def info(self, msg):
        pass

    def warning(self, msg):
        pass


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


def test_build_motor_frame_checksum():
    payload = pack_motor_command([8.8, 0.0, 8.8, 0.0])
    frame = build_frame(PACKET_CMD_MOTOR, payload)

    assert frame[:4] == bytes([0xAA, 0xBB, PACKET_CMD_MOTOR, 0x10])
    assert frame[4:-1] == payload
    assert frame[-1] == (sum(frame[:-1]) & 0xFF)


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


def test_arm_bridge_sends_one_frame_per_servo():
    sent = []
    node = FakeNode()
    bridge = ArmBridge(node, cmd_callback=lambda cmd, payload: sent.append(
        (cmd, payload)
    ))
    msg = Float64MultiArray()
    msg.data = [0.1, 0.2, 0.3, 0.4, 0.5]

    bridge._on_command(msg)

    assert [cmd for cmd, payload in sent] == [PACKET_CMD_SERVO] * 5
    assert [struct.unpack('<Bf', payload)[0] for cmd, payload in sent] == [
        1, 2, 3, 4, 5
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
    dispatcher.handle_frame(PACKET_CMD_SERVO, pack_servo_command(2, 1.25))

    assert len(node.publisher.messages) == 1
    assert list(node.publisher.messages[-1].data) == [1.0, 2.0, 3.0, 4.0]
