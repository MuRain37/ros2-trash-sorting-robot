"""串口反馈分发模块：从 SerialPort 读取帧，解析后分发给对应的桥接器."""

from robot_hardware_bridge.packet_protocol import (
    PACKET_TYPE_MOTOR_STATE,
    PACKET_TYPE_SERVO_STATE,
    unpack_motor_state,
    unpack_servo_feedback,
)


class SerialFeedbackDispatcher:
    """定时从串口缓冲区读取帧，按帧类型分发到 MotorBridge 或 ArmBridge."""

    def __init__(self, node):
        self._node = node
        # 帧类型 → 处理函数的映射表
        self._handlers = {
            PACKET_TYPE_MOTOR_STATE: self._handle_motor_state,
            PACKET_TYPE_SERVO_STATE: self._handle_servo_state,
        }

    def handle_frame(self, frame_type, payload):
        """处理单帧数据，根据类型分发到对应 handler."""
        handler = self._handlers.get(frame_type)
        if handler:
            handler(payload)
        elif self._node._debug:
            self._node.get_logger().info(
                f'Ignoring unsupported serial frame type=0x{frame_type:02X}, '
                f'payload_len={len(payload)}'
            )

    def dispatch_available(self):
        """读取并分发当前缓冲区中所有帧（由定时器回调触发）."""
        for frame_type, payload in self._node._serial.read_frames():
            self.handle_frame(frame_type, payload)

    def _handle_motor_state(self, payload):
        """处理电机状态帧：解包4个状态值 → 发布反馈."""
        try:
            motor_state = unpack_motor_state(payload)
        except ValueError:
            self._node.get_logger().warning(
                'Ignoring motor state feedback with invalid payload length: '
                f'{len(payload)}'
            )
            return

        # self._node.get_logger().info(f'Parsed motor state: {motor_state}')
        # 非开环模式下，用真实电机状态更新反馈
        if not self._node._motor_open_loop_control:
            self._node._motor.publish_feedback(motor_state)

    def _handle_servo_state(self, payload):
        """处理舵机状态帧：解包 → 更新 ArmBridge 对应关节角度."""
        try:
            _, servo_id, position, _ = unpack_servo_feedback(payload)
        except ValueError:
            self._node.get_logger().warning(
                'Ignoring servo state feedback with invalid payload length: '
                f'{len(payload)}'
            )
            return

        self._node.get_logger().info(
            f'Parsed servo state: id={servo_id}, position={position}'
        )
        # 非开环模式下，用真实舵机反馈更新关节角度
        if not self._node._arm_open_loop_control:
            self._node._arm.publish_servo_feedback(int(servo_id), position)
