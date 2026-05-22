
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from robot_controller.arm_bridge import ArmBridge
from robot_controller.motor_bridge import MotorBridge
from robot_controller.serial_port import SerialPort


class HardwareNode(Node):
    """硬件通信节点：组合串口、电机桥接、IMU 发布。"""

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_topic', 'imu_data')
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

        self._serial = SerialPort(port, baudrate, debug=debug, logger=self.get_logger())

        self._motor = MotorBridge(
            self,
            cmd_callback=self._serial.write,
            open_loop_control=self._motor_open_loop_control,
        )

        self._arm = ArmBridge(
            self,
            cmd_callback=self._serial.write,
            open_loop_control=self._arm_open_loop_control,
            joint_count=arm_joint_count,
        )

        self._imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        period = 1.0 / read_rate
        self._timer = self.create_timer(period, self._read_serial)

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

    def _read_serial(self):
        data = self._serial.read_raw()
        if not data:
            return
        encoder = None
        arm_state = None
        for line in data.split(b'\n'):
            line = line.strip()
            if not line:
                continue
            try:
                text = line.decode('utf-8', errors='replace')
            except Exception:
                continue

            if text.startswith('ENCODER:'):
                parts = text.split(':', 1)[1].split(',')
                if len(parts) == 4:
                    try:
                        encoder = [float(v) for v in parts]
                    except ValueError:
                        pass
            elif text.startswith('ARM_STATE:'):
                parts = text.split(':', 1)[1].split(',')
                try:
                    arm_state = [float(v) for v in parts]
                except ValueError:
                    pass

        if encoder is not None:
            self.get_logger().info(f'Parsed encoder: {encoder}')
            if not self._motor_open_loop_control:
                self._motor.publish_feedback(encoder)

        if arm_state is not None:
            self.get_logger().info(f'Parsed arm state: {arm_state}')
            if not self._arm_open_loop_control:
                self._arm.publish_feedback(arm_state)

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
