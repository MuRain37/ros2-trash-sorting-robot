"""机械臂桥接模块：关节角度 ↔ 舵机原始位置值的双向转换与指令收发."""

import math
import time

from robot_interfaces.srv import SetGripper
from std_msgs.msg import Float64MultiArray

from robot_hardware_bridge.packet_protocol import (
    PACKET_CMD_SERVO,
    pack_servo_command,
)


# 机械臂关节数量（5自由度）
DEFAULT_ARM_JOINT_COUNT = 5

# 夹爪舵机配置
DEFAULT_GRIPPER_SERVO_ID = 6
GRIPPER_OPEN_RAW = 0.0
GRIPPER_CLOSE_RAW = 700.0

# 舵机指令发送频率（Hz）
DEFAULT_SERVO_SEND_RATE = 10.0
# ArmBridge 默认舵机运动完成时间（ms）
DEFAULT_SERVO_DURATION_MS = 100.0

# 舵机原始位置范围：中位500，最小0，最大1000
SERVO_CENTER = 500.0
SERVO_RAW_MIN = 0.0
SERVO_RAW_MAX = 1000.0

# 各关节半角行程（度），总行程 = 半角 × 2
JOINT_MAX_DEGREES = (180.0 , 120.0, 120.0, 120.0, 120.0)
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
                 open_loop_control=False,
                 send_rate=DEFAULT_SERVO_SEND_RATE,
                 duration_ms=DEFAULT_SERVO_DURATION_MS,
                 gripper_service='set_gripper'):
        self._node = node
        self._cmd_callback = cmd_callback  # 发送回调，签名为 (cmd, payload)
        self._open_loop_control = open_loop_control
        self._joint_count = DEFAULT_ARM_JOINT_COUNT
        self._gripper_servo_id = DEFAULT_GRIPPER_SERVO_ID
        self._send_period = 1.0 / float(send_rate) if send_rate > 0.0 else 0.0
        self._duration_ms = float(duration_ms)
        self._last_send_time = None
        self._last_sent_servo_positions = None
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
        # 夹爪开环控制服务：0.0=关，1.0=开
        self._gripper_service = node.create_service(
            SetGripper,
            gripper_service,
            self._on_gripper_command,
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

        if servo_positions == self._last_sent_servo_positions:
            if self._open_loop_control:
                self.publish_feedback(command)
            return

        # 目标未变化时不重复发送；目标变化时按发送频率节流，避免打爆串口
        if (
            self._should_send_now()
            and self._send_servo_targets(servo_positions)
        ):
            self._last_sent_servo_positions = list(servo_positions)

        # 开环模式：把目标值直接当反馈发出
        if self._open_loop_control:
            self.publish_feedback(command)

    def publish_feedback(self, data: list):
        """发布当前关节角度反馈到默认反馈话题."""
        self._positions = [float(value) for value in data[:self._joint_count]]
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._fb_pub.publish(msg)

    def _send_servo_targets(self, servo_positions):
        """逐个打包舵机指令并通过回调发送."""
        if not self._cmd_callback:
            return False

        for index, position in enumerate(servo_positions):
            servo_id = index + 1  # 舵机ID从1开始
            payload = pack_servo_command(
                servo_id,
                position,
                self._duration_ms,
            )
            self._cmd_callback(PACKET_CMD_SERVO, payload)
        return True

    def _on_gripper_command(self, request, response):
        """收到夹爪开度比例服务请求，转换为 servo6 原始位置指令."""
        try:
            open_ratio = float(request.open_ratio)
            if not 0.0 <= open_ratio <= 1.0:
                raise ValueError(
                    f'open ratio {open_ratio} not in [0.0, 1.0]'
                )
            duration_ms = float(request.duration_ms)
            if duration_ms < 0.0:
                raise ValueError(f'duration_ms must be >= 0: {duration_ms}')
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        position = GRIPPER_CLOSE_RAW + open_ratio * (
            GRIPPER_OPEN_RAW - GRIPPER_CLOSE_RAW
        )

        if not self._send_gripper_target(position, duration_ms):
            response.success = False
            response.message = 'command callback unavailable'
            return response

        response.success = True
        response.message = 'gripper command accepted'
        return response

    def _send_gripper_target(self, position, duration_ms):
        """打包夹爪舵机指令并通过回调发送."""
        if not self._cmd_callback:
            return False

        payload = pack_servo_command(
            self._gripper_servo_id,
            position,
            duration_ms,
        )
        self._cmd_callback(PACKET_CMD_SERVO, payload)
        return True

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
        if servo_id == self._gripper_servo_id:
            return

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
