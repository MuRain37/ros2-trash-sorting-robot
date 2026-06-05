"""机械臂桥接模块：关节角度 ↔ 舵机原始位置值的双向转换与指令收发."""

import math
import time

from std_msgs.msg import Float64MultiArray

from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_SERVO,
    pack_servo_command,
)


# 机械臂关节数量（5自由度）
DEFAULT_ARM_JOINT_COUNT = 5
# 舵机指令发送频率（Hz）
DEFAULT_SERVO_SEND_RATE = 10.0
# ArmBridge 默认舵机运动完成时间（ms）
DEFAULT_SERVO_DURATION_MS = 100.0

# 舵机原始位置范围：中位500，最小0，最大1000
SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0

# 各关节半角行程（度），总行程 = 半角 × 2
JOINT_MAX_DEGREES = (180.0, 120.0, 120.0, 120.0, 120.0)
# 各关节方向系数：±1，由舵机安装方向决定
JOINT_DIRECTIONS = (-1.0, -1.0, -1.0, -1.0, 1.0)


def clamp_servo_raw(position):
    """将舵机原始位置值限制在 [0, 1000] 安全范围内."""
    return max(SERVO_RAW_MIN, min(SERVO_RAW_MAX, position))


def joint_command_to_servo_raw(index, value):
    """
    关节角度（弧度）→ 舵机原始位置值.

    映射公式：raw = 500 + direction × deg × (500 / max_degrees)
    """
    value = math.degrees(float(value))
    if index < len(JOINT_MAX_DEGREES):
        max_degrees = JOINT_MAX_DEGREES[index]
        direction = JOINT_DIRECTIONS[index]
        if not -max_degrees <= value <= max_degrees:
            raise ValueError(
                f'joint {index + 1} out of range: {value} '
                f'not in [{-max_degrees}, {max_degrees}] deg'
            )
        return clamp_servo_raw(
            SERVO_CENTER + direction * value * (SERVO_CENTER / max_degrees)
        )

    raise ValueError(f'unsupported arm joint index: {index}')


def servo_raw_to_joint_command(index, raw_position):
    """舵机原始位置值 → 关节角度（弧度），上面公式的逆运算."""
    raw_position = float(raw_position)
    if index < len(JOINT_MAX_DEGREES):
        max_degrees = JOINT_MAX_DEGREES[index]
        direction = JOINT_DIRECTIONS[index]
        degrees = (raw_position - SERVO_CENTER) * (
            max_degrees / SERVO_CENTER
        )
        degrees *= direction
        return math.radians(degrees)

    raise ValueError(f'unsupported arm joint index: {index}')


class ArmBridge:
    """
    机械臂桥接器：订阅关节角度指令，转换为舵机原始位置值后通过串口发送.

    同时发布关节角度反馈。
    """

    def __init__(self, node, cmd_topic='arm_position_cmd',
                 fb_topic='arm_position_fb', cmd_callback=None,
                 open_loop_control=False, print_only=False,
                 send_rate=DEFAULT_SERVO_SEND_RATE,
                 duration_ms=DEFAULT_SERVO_DURATION_MS):
        self._node = node
        self._cmd_callback = cmd_callback  # 发送回调，签名为 (cmd, payload)
        self._open_loop_control = open_loop_control
        self._joint_count = DEFAULT_ARM_JOINT_COUNT
        self._print_only = print_only  # True 时只打印不发送（调试用）
        self._send_period = 1.0 / float(send_rate) if send_rate > 0.0 else 0.0
        self._servo_send_period = (
            self._send_period / self._joint_count
            if self._send_period > 0.0 else 0.0
        )
        self._duration_ms = float(duration_ms)
        self._last_send_time = None
        self._pending_servo_positions = None
        self._next_servo_index = 0
        self._positions = [0.0] * self._joint_count  # 当前各关节角度（弧度）

        # 发布关节角度反馈
        self._fb_pub = node.create_publisher(Float64MultiArray, fb_topic, 10)
        # 订阅关节角度指令
        self._cmd_sub = node.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self._on_command,
            10,
        )
        self._servo_send_timer = None
        if self._servo_send_period > 0.0:
            self._servo_send_timer = node.create_timer(
                self._servo_send_period,
                self._send_next_servo_target,
            )

    def _on_command(self, msg: Float64MultiArray):
        """收到关节角度指令（弧度），转换并发送."""
        if len(msg.data) < self._joint_count:
            self._node.get_logger().warning(
                f'Expected at least {self._joint_count} arm positions, '
                f'got {len(msg.data)}'
            )
            return

        # 角度 → 舵机原始位置值
        command = [float(value) for value in msg.data[:self._joint_count]]
        try:
            servo_positions = [
                joint_command_to_servo_raw(index, value)
                for index, value in enumerate(command)
            ]
        except ValueError as e:
            self._node.get_logger().warning(f'Invalid arm command: {e}')
            return

        # 按发送频率节流，避免打爆串口
        if self._should_send_now():
            self._log_servo_targets(servo_positions)
            self._queue_servo_targets(servo_positions)

        # 开环模式：把目标值直接当反馈发出
        if self._open_loop_control:
            self._positions = command
            self.publish_feedback(command)

    def publish_feedback(self, data: list):
        """发布当前关节角度反馈到默认反馈话题."""
        self._positions = [float(value) for value in data[:self._joint_count]]
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._fb_pub.publish(msg)

    def _log_servo_targets(self, servo_positions):
        """打印舵机目标原始位置值（调试用）."""
        targets = ', '.join(
            f'servo{index + 1}={position:.1f}'
            for index, position in enumerate(servo_positions)
        )
        mode = 'print-only' if self._print_only else 'send'
        self._node.get_logger().info(
            f'Arm servo targets ({mode}): {targets}'
        )

    def _send_servo_targets(self, servo_positions):
        """逐个打包舵机指令并通过回调发送."""
        if not self._cmd_callback or self._print_only:
            return

        for index, position in enumerate(servo_positions):
            self._send_servo_target(index, position)

    def _queue_servo_targets(self, servo_positions):
        """缓存一组舵机目标，并按定时器节奏逐个发送."""
        if not self._servo_send_timer:
            self._send_servo_targets(servo_positions)
            return

        self._pending_servo_positions = list(servo_positions)
        self._next_servo_index = 0

    def _send_next_servo_target(self):
        """发送当前缓存目标中的下一个舵机命令."""
        if not self._pending_servo_positions:
            return

        if self._next_servo_index >= len(self._pending_servo_positions):
            self._pending_servo_positions = None
            self._next_servo_index = 0
            return

        index = self._next_servo_index
        position = self._pending_servo_positions[index]
        self._next_servo_index += 1
        self._send_servo_target(index, position)

        if self._next_servo_index >= len(self._pending_servo_positions):
            self._pending_servo_positions = None
            self._next_servo_index = 0

    def _send_servo_target(self, index, position):
        """发送单个舵机目标."""
        if not self._cmd_callback or self._print_only:
            return

        servo_id = index + 1  # 舵机ID从1开始
        payload = pack_servo_command(
            servo_id,
            position,
            self._duration_ms,
        )
        self._cmd_callback(PACKET_CMD_SERVO, payload)

    def _should_send_now(self):
        """发送节流：距上次发送不足 send_period 则跳过."""
        now = time.monotonic()
        if (
            self._last_send_time is not None
            and now - self._last_send_time < self._send_period
        ):
            return False

        self._last_send_time = now
        return True

    def publish_servo_feedback(self, servo_id, position):
        """
        收到单个舵机反馈（原始位置值），更新对应关节角度并发布.

        servo_id: 1-based 舵机编号
        position: 舵机原始位置值
        """
        if not 1 <= servo_id <= self._joint_count:
            self._node.get_logger().warning(
                f'Ignoring servo feedback with invalid id: {servo_id}'
            )
            return

        try:
            self._positions[servo_id - 1] = servo_raw_to_joint_command(
                servo_id - 1,
                position,
            )
        except ValueError as e:
            self._node.get_logger().warning(
                f'Ignoring servo feedback: {e}'
            )
            return
        self.publish_feedback(self._positions)

    def _publish_feedback(self):
        self.publish_feedback(self._positions)
