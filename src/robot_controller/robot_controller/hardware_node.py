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
    """Hardware communication node."""

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_rate', 100.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('open_loop_control', False)
        optional_parameter = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            'motor_open_loop_control',
            descriptor=optional_parameter,
        )
        self.declare_parameter(
            'arm_open_loop_control',
            descriptor=optional_parameter,
        )
        self.declare_parameter('arm_joint_count', 5)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        read_rate = self.get_parameter('read_rate').value
        debug = self.get_parameter('debug').value
        self._debug = debug
        arm_joint_count = int(self.get_parameter('arm_joint_count').value)
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

        self._serial = SerialPort(
            port, baudrate, debug=debug, logger=self.get_logger()
        )

        self._motor = MotorBridge(
            self,
            cmd_callback=self._serial.write_frame,
            open_loop_control=self._motor_open_loop_control,
        )

        self._arm = ArmBridge(
            self,
            cmd_callback=self._serial.write_frame,
            open_loop_control=self._arm_open_loop_control,
            joint_count=arm_joint_count,
        )
        self._feedback_dispatcher = SerialFeedbackDispatcher(self)

        period = 1.0 / read_rate
        self._timer = self.create_timer(
            period, self._feedback_dispatcher.dispatch_available
        )

        self.get_logger().info(
            f'Hardware bridge started (port: {port}, baudrate: {baudrate}, '
            f'rate: {read_rate}Hz, '
            f'motor_open_loop_control: {self._motor_open_loop_control}, '
            f'arm_open_loop_control: {self._arm_open_loop_control}, '
            f'arm_joint_count: {arm_joint_count})'
        )

    def _optional_bool_parameter(self, name, default_value):
        parameter = self.get_parameter(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            return default_value
        return bool(parameter.value)

    def destroy_node(self):
        self._serial.close()
        super().destroy_node()


def main(args=None):
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
