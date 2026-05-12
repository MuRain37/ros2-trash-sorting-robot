
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

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

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        read_rate = self.get_parameter('read_rate').value
        debug = self.get_parameter('debug').value

        self._serial = SerialPort(port, baudrate, debug=debug, logger=self.get_logger())

        self._motor = MotorBridge(self, cmd_callback=self._serial.write)

        self._imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        period = 1.0 / read_rate
        self._timer = self.create_timer(period, self._read_serial)

        self.get_logger().info(
            f'Hardware bridge started (port: {port}, baudrate: {baudrate}, rate: {read_rate}Hz)'
        )

    def _read_serial(self):
        data = self._serial.read_raw()
        if not data:
            return
        encoder = None
        for line in data.split(b'\n'):
            line = line.strip()
            if not line or not line.startswith(b'ENCODER:'):
                continue
            try:
                text = line.decode('utf-8', errors='replace')
            except Exception:
                continue
            parts = text.split(':', 1)[1].split(',')
            if len(parts) == 4:
                try:
                    encoder = [float(v) for v in parts]
                except ValueError:
                    pass
        if encoder is not None:
            self.get_logger().info(f'Parsed encoder: {encoder}')
            self._motor.publish_feedback(encoder)

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
