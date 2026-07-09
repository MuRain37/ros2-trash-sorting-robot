"""Forward scan_camera LaserScan data to Nav2 costmaps when enabled."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class ScanCameraGate(Node):
    """Gate /scan_camera before it reaches Nav2 costmaps."""

    def __init__(self):
        super().__init__('scan_camera_gate')
        self.declare_parameter('input_topic', '/scan_camera')
        self.declare_parameter('output_topic', '/scan_camera_costmap')
        self.declare_parameter(
            'set_enabled_service',
            '/set_scan_camera_costmap_enabled',
        )
        self.declare_parameter('enabled', True)

        self.enabled = bool(self.get_parameter('enabled').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        service_name = self.get_parameter('set_enabled_service').value

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.handle_scan,
            qos_profile_sensor_data,
        )
        self.service = self.create_service(
            SetBool,
            service_name,
            self.handle_set_enabled,
        )
        self.get_logger().info(
            f'scan camera gate ready: {input_topic} -> {output_topic}, '
            f'enabled={self.enabled}'
        )

    def handle_scan(self, msg):
        if self.enabled:
            self.publisher.publish(msg)

    def handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = (
            'scan_camera costmap forwarding enabled'
            if self.enabled
            else 'scan_camera costmap forwarding disabled'
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScanCameraGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
