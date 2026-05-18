import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CenterSquareMarker(Node):
    def __init__(self):
        super().__init__('center_square_marker')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('size', 3.0)
        self.declare_parameter('publish_period', 0.5)

        self.publisher = self.create_publisher(Marker, 'center_square_marker', 10)
        period = self.get_parameter('publish_period').value
        self.timer = self.create_timer(period, self.publish_marker)

    def publish_marker(self):
        frame_id = self.get_parameter('frame_id').value
        size = float(self.get_parameter('size').value)
        half = size / 2.0

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_nav2_behaviors'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        corners = [
            (-half, -half),
            (half, -half),
            (half, half),
            (-half, half),
            (-half, -half),
        ]
        marker.points = [Point(x=x, y=y, z=0.03) for x, y in corners]
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CenterSquareMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
