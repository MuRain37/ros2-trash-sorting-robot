import struct


# 帧头同步字，接收端扫描 AA BB 确定帧起始
FRAME_HEAD = bytes([0xAA, 0xBB])
# 每帧固定开销：帧头2 + 命令1 + 长度1 + 校验1 = 5字节
FRAME_OVERHEAD = 5
# 单帧负载上限；帧总长度 = 负载长度 + FRAME_OVERHEAD
MAX_PAYLOAD_LEN = 250

# 命令码（ROS2 → STM32）
PACKET_CMD_MOTOR = 0x01  # 电机速度指令
PACKET_CMD_SERVO = 0x04  # 舵机控制指令

# 反馈类型码（STM32 → ROS2）
PACKET_TYPE_ACK = 0x80           # 应答
PACKET_TYPE_ERROR = 0x81         # 错误
PACKET_TYPE_MOTOR_STATE = 0x82   # 电机状态反馈
PACKET_TYPE_SERVO_STATE = 0x83   # 舵机状态（位置反馈）

# struct 格式串（'<'=小端，'f'=float32，与 STM32 一致）
MOTOR_FORMAT = '<ffff'               # 4个轮速/电机状态值
SERVO_SET_POSITION_FORMAT = '<ffff'  # [子命令, 舵机ID, 位置, 时长]
SERVO_CHANGE_ID_FORMAT = '<fff'      # [子命令, 当前ID, 新ID]
SERVO_PROBE_ID_FORMAT = '<ff'        # [子命令, 探测ID]
ENCODER_FORMAT = '<ffff'             # 4个电机状态反馈值

# 舵机子命令类型（放在负载第一个 float 字段）
SERVO_CMD_SET_POSITION = 1.0  # 设置位置
SERVO_CMD_CHANGE_ID = 2.0     # 修改硬件ID
SERVO_CMD_PROBE_ID = 3.0      # 探测ID是否存在

# 各负载字节长度
MOTOR_PAYLOAD_SIZE = struct.calcsize(MOTOR_FORMAT)
SERVO_SET_POSITION_PAYLOAD_SIZE = struct.calcsize(SERVO_SET_POSITION_FORMAT)
SERVO_CHANGE_ID_PAYLOAD_SIZE = struct.calcsize(SERVO_CHANGE_ID_FORMAT)
SERVO_PROBE_ID_PAYLOAD_SIZE = struct.calcsize(SERVO_PROBE_ID_FORMAT)
ENCODER_PAYLOAD_SIZE = struct.calcsize(ENCODER_FORMAT)


def pack_motor_command(values):
    """4个电机速度 → 16字节负载（小端 float32）。"""
    return struct.pack(MOTOR_FORMAT, *(float(value) for value in values))


def pack_servo_command(servo_id, position, duration_ms=1000.0):
    """舵机位置指令 → 16字节负载：[子命令|ID|原始位置|时长ms]。"""
    return struct.pack(
        SERVO_SET_POSITION_FORMAT,
        SERVO_CMD_SET_POSITION,
        float(servo_id),
        float(position),
        float(duration_ms),
    )


def pack_servo_change_id_command(servo_id, new_id):
    """舵机改ID → 12字节负载：[子命令|当前ID|新ID]。"""
    return struct.pack(
        SERVO_CHANGE_ID_FORMAT,
        SERVO_CMD_CHANGE_ID,
        float(servo_id),
        float(new_id),
    )


def pack_servo_probe_id_command(probe_id):
    """舵机ID探测 → 8字节负载：[子命令|探测ID]。"""
    return struct.pack(
        SERVO_PROBE_ID_FORMAT,
        SERVO_CMD_PROBE_ID,
        float(probe_id),
    )


def unpack_encoder_feedback(payload):
    """16字节电机状态负载 → [state1, state2, state3, state4]。"""
    if len(payload) != ENCODER_PAYLOAD_SIZE:
        raise ValueError(
            'invalid encoder feedback payload length: '
            f'{len(payload)} != {ENCODER_PAYLOAD_SIZE}'
        )
    return list(struct.unpack(ENCODER_FORMAT, payload))


def unpack_motor_state(payload):
    """电机状态反馈 → 4个轮子状态值。"""
    return unpack_encoder_feedback(payload)


def unpack_servo_feedback(payload):
    """舵机反馈 → (子命令, 舵机ID, 当前位置, 时长)。"""
    if len(payload) != SERVO_SET_POSITION_PAYLOAD_SIZE:
        raise ValueError(
            'invalid servo feedback payload length: '
            f'{len(payload)} != {SERVO_SET_POSITION_PAYLOAD_SIZE}'
        )
    return struct.unpack(SERVO_SET_POSITION_FORMAT, payload)


def build_frame(cmd, payload=b''):
    """构建一帧：帧头 + 命令 + 长度 + 负载 + 累加和校验。"""
    if not 0 <= cmd <= 0xFF:
        raise ValueError('cmd must fit in one byte')
    if payload is None:
        payload = b''
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            f'payload too large: {len(payload)} > {MAX_PAYLOAD_LEN}'
        )

    frame = bytearray(FRAME_HEAD)
    frame.append(cmd)
    frame.append(len(payload))
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)  # 累加和取低8位
    return bytes(frame)
