import struct


PACKET_CMD_MOTOR = 0x01
PACKET_CMD_SERVO = 0x04

MOTOR_FORMAT = '<ffff'
SERVO_FORMAT = '<Bf'
ENCODER_FORMAT = '<ffff'

MOTOR_PAYLOAD_SIZE = struct.calcsize(MOTOR_FORMAT)
SERVO_PAYLOAD_SIZE = struct.calcsize(SERVO_FORMAT)
ENCODER_PAYLOAD_SIZE = struct.calcsize(ENCODER_FORMAT)


def pack_motor_command(values):
    """Pack four motor command values as little-endian float32."""
    return struct.pack(MOTOR_FORMAT, *(float(value) for value in values))


def pack_servo_command(servo_id, position):
    """Pack one servo id and target position."""
    return struct.pack(SERVO_FORMAT, int(servo_id), float(position))


def unpack_encoder_feedback(payload):
    """Unpack four encoder feedback values."""
    if len(payload) != ENCODER_PAYLOAD_SIZE:
        raise ValueError(
            'invalid encoder feedback payload length: '
            f'{len(payload)} != {ENCODER_PAYLOAD_SIZE}'
        )
    return list(struct.unpack(ENCODER_FORMAT, payload))


def unpack_servo_feedback(payload):
    """Unpack one servo id and feedback position."""
    if len(payload) != SERVO_PAYLOAD_SIZE:
        raise ValueError(
            'invalid servo feedback payload length: '
            f'{len(payload)} != {SERVO_PAYLOAD_SIZE}'
        )
    return struct.unpack(SERVO_FORMAT, payload)
