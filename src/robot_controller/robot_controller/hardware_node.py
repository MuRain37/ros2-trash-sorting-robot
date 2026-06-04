"""硬件通信节点：整合串口、电机桥接、舵机桥接和反馈分发。

这是整个 robot_controller 的入口节点，启动后负责 ROS2 与 STM32 之间的所有通信。
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from robot_controller.arm_bridge import ArmBridge
from robot_controller.motor_bridge import MotorBridge
from robot_controller.serial_feedback_dispatcher import (
    SerialFeedbackDispatcher,
)
from robot_controller.serial_port import SerialPort


class HardwareNode(Node):
    """硬件通信主节点。

    组件关系：
    SerialPort（串口收发） → MotorBridge（轮速指令/反馈）
                          → ArmBridge（关节角度指令/反馈）
    SerialFeedbackDispatcher → 定时读取串口帧 → 分发给 MotorBridge / ArmBridge
    """

    def __init__(self):
        super().__init__('hardware_bridge')

        # 串口设备路径，例如 /dev/ttyACM0
        self.declare_parameter('port', '/dev/ttyACM0')
        # 串口波特率，需与 STM32 固件一致
        self.declare_parameter('baudrate', 115200)
        # 串口读取和反馈分发频率（Hz）
        self.declare_parameter('read_rate', 100.0)
        # True 时打印串口收发细节
        self.declare_parameter('debug', False)
        # 旧版总开环开关；分项参数未设置时作为默认值
        self.declare_parameter('open_loop_control', False)
        # True 时机械臂只打印目标，不写串口
        self.declare_parameter('arm_print_only', False)
        # 舵机指令发送频率上限（Hz）
        self.declare_parameter('arm_servo_send_rate', 10.0)
        # 舵机运动完成时间（ms）
        self.declare_parameter('arm_servo_duration_ms', 100.0)
        # 可选参数（dynamic_typing），未设置时回退到 open_loop_control
        optional_parameter = ParameterDescriptor(dynamic_typing=True)
        # 电机开环反馈：True 时直接回传轮速指令
        self.declare_parameter(
            'motor_open_loop_control',
            descriptor=optional_parameter,
        )
        # 机械臂开环反馈：True 时直接回传关节目标
        self.declare_parameter(
            'arm_open_loop_control',
            descriptor=optional_parameter,
        )

        # 读取参数
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        read_rate = self.get_parameter('read_rate').value
        debug = self.get_parameter('debug').value
        arm_print_only = bool(self.get_parameter('arm_print_only').value)
        arm_servo_send_rate = float(
            self.get_parameter('arm_servo_send_rate').value
        )
        arm_servo_duration_ms = float(
            self.get_parameter('arm_servo_duration_ms').value
        )
        self._debug = debug

        # 开环控制：新参数优先，未设置则回退到旧参数 open_loop_control
        legacy_open_loop_control = bool(
            self.get_parameter('open_loop_control').value
        )
        self._motor_open_loop_control = self._optional_bool_parameter(
            'motor_open_loop_control',
            legacy_open_loop_control,
        )
        self._arm_open_loop_control = self._optional_bool_parameter(
            'arm_open_loop_control',
            legacy_open_loop_control,
        )

        # 所有控制指令最终都通过这个串口对象封包发送
        self._serial = SerialPort(
            port, baudrate, debug=debug, logger=self.get_logger()
        )

        # 电机桥接负责轮速话题和电机命令帧之间的转换
        self._motor = MotorBridge(
            self,
            cmd_callback=self._serial.write_frame,
            open_loop_control=self._motor_open_loop_control,
        )

        # 舵机桥接负责关节角度和舵机原始位置之间的转换
        self._arm = ArmBridge(
            self,
            cmd_callback=self._serial.write_frame,
            open_loop_control=self._arm_open_loop_control,
            print_only=arm_print_only,
            send_rate=arm_servo_send_rate,
            duration_ms=arm_servo_duration_ms,
        )

        # 串口反馈分发器
        self._feedback_dispatcher = SerialFeedbackDispatcher(self)

        # 定时器：按 read_rate 频率读取串口帧并分发
        period = 1.0 / read_rate
        self._timer = self.create_timer(
            period, self._feedback_dispatcher.dispatch_available
        )

        self.get_logger().info(
            f'Hardware bridge started (port: {port}, baudrate: {baudrate}, '
            f'rate: {read_rate}Hz, '
            f'motor_open_loop_control: {self._motor_open_loop_control}, '
            f'arm_open_loop_control: {self._arm_open_loop_control}, '
            f'arm_print_only: {arm_print_only}, '
            f'arm_servo_send_rate: {arm_servo_send_rate}Hz, '
            f'arm_servo_duration_ms: {arm_servo_duration_ms})'
        )

    def _optional_bool_parameter(self, name, default_value):
        """读取可选布尔参数，未设置时返回默认值。

        name: ROS 参数名。
        default_value: 参数未设置时使用的布尔值。
        """
        parameter = self.get_parameter(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            return default_value
        return bool(parameter.value)

    def destroy_node(self):
        """销毁节点前关闭串口。"""
        self._serial.close()
        super().destroy_node()


def main(args=None):
    """入口函数：初始化节点并进入 spin 循环。"""
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
