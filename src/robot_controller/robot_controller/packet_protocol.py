import struct


PACKET_CMD_MOTOR = 0x01
PACKET_CMD_SERVO = 0x04

PACKET_TYPE_ACK = 0x80
PACKET_TYPE_ERROR = 0x81
PACKET_TYPE_MOTOR_STATE = 0x82
PACKET_TYPE_SERVO_STATE = 0x83

MOTOR_FORMAT = '<ffff'
SERVO_SET_POSITION_FORMAT = '<ffff'
SERVO_CHANGE_ID_FORMAT = '<fff'
SERVO_PROBE_ID_FORMAT = '<ff'
ENCODER_FORMAT = '<ffff'

SERVO_CMD_SET_POSITION = 1.0
SERVO_CMD_CHANGE_ID = 2.0
SERVO_CMD_PROBE_ID = 3.0

MOTOR_PAYLOAD_SIZE = struct.calcsize(MOTOR_FORMAT)
SERVO_SET_POSITION_PAYLOAD_SIZE = struct.calcsize(SERVO_SET_POSITION_FORMAT)
SERVO_CHANGE_ID_PAYLOAD_SIZE = struct.calcsize(SERVO_CHANGE_ID_FORMAT)
SERVO_PROBE_ID_PAYLOAD_SIZE = struct.calcsize(SERVO_PROBE_ID_FORMAT)
ENCODER_PAYLOAD_SIZE = struct.calcsize(ENCODER_FORMAT)


def pack_motor_command(values):
    """Pack four motor command values as little-endian float32."""
    return struct.pack(MOTOR_FORMAT, *(float(value) for value in values))


def pack_servo_command(servo_id, position, duration_ms=1000.0):
    """Pack one bus-servo set-position command."""
    return struct.pack(
        SERVO_SET_POSITION_FORMAT,
        SERVO_CMD_SET_POSITION,
        float(servo_id),
        float(position),
        float(duration_ms),
    )


def pack_servo_change_id_command(servo_id, new_id):
    """Pack one bus-servo hardware id change command."""
    return struct.pack(
        SERVO_CHANGE_ID_FORMAT,
        SERVO_CMD_CHANGE_ID,
        float(servo_id),
        float(new_id),
    )


def pack_servo_probe_id_command(probe_id):
    """Pack one bus-servo hardware id probe command."""
    return struct.pack(
        SERVO_PROBE_ID_FORMAT,
        SERVO_CMD_PROBE_ID,
        float(probe_id),
    )


def unpack_encoder_feedback(payload):
    """Unpack four encoder feedback values."""
    if len(payload) != ENCODER_PAYLOAD_SIZE:
        raise ValueError(
            'invalid encoder feedback payload length: '
            f'{len(payload)} != {ENCODER_PAYLOAD_SIZE}'
        )
    return list(struct.unpack(ENCODER_FORMAT, payload))


def unpack_motor_state(payload):
    """Unpack four reported motor angular velocities."""
    return unpack_encoder_feedback(payload)


def unpack_servo_feedback(payload):
    """Unpack one reported servo position command payload."""
    if len(payload) != SERVO_SET_POSITION_PAYLOAD_SIZE:
        raise ValueError(
            'invalid servo feedback payload length: '
            f'{len(payload)} != {SERVO_SET_POSITION_PAYLOAD_SIZE}'
        )
    return struct.unpack(SERVO_SET_POSITION_FORMAT, payload)
