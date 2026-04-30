import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from robot_controller.motor_bridge import MotorBridge
from robot_controller.serial_port import SerialPort


class HardwareNode(Node):
    """硬件通信节点：组合串口、电机桥接、IMU 发布。"""

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_topic', 'imu_data')
        self.declare_parameter('read_rate', 100.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        read_rate = self.get_parameter('read_rate').value

        self._serial = SerialPort(port, baudrate)

        self._motor = MotorBridge(self, cmd_callback=self._serial.write)

        self._imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        period = 1.0 / read_rate
        self._timer = self.create_timer(period, self._read_serial)

        self.get_logger().info(
            f'Hardware bridge started (port: {port}, baudrate: {baudrate}, rate: {read_rate}Hz)'
        )

    def _read_serial(self):
        encoder, imu = self._serial.read()
        if encoder is not None:
            self._motor.publish_feedback(encoder)
        if imu is not None:
            self._imu_pub.publish(imu)

    def destroy_node(self):
        self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
