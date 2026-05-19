import rclpy
from geometry_msgs.msg import Point, PolygonStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CoverageMarkerPublisher(Node):
    def __init__(self):
        super().__init__('coverage_marker_publisher')
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('marker_line_width', 0.02)
        self.declare_parameter('marker_topic', 'center_square_marker')
        self.declare_parameter('rectangle_topic', 'coverage_rectangle')
        self.declare_parameter('coverage_path_topic', 'coverage_path_preview')

        marker_topic = self.get_parameter('marker_topic').value
        rectangle_topic = self.get_parameter('rectangle_topic').value
        coverage_path_topic = self.get_parameter('coverage_path_topic').value

        self.publisher = self.create_publisher(Marker, marker_topic, 10)
        self.rectangle_subscription = self.create_subscription(
            PolygonStamped,
            rectangle_topic,
            self.handle_rectangle,
            10,
        )
        self.path_subscription = self.create_subscription(
            Path,
            coverage_path_topic,
            self.handle_coverage_path,
            10,
        )
        period = self.get_parameter('publish_period').value
        self.timer = self.create_timer(period, self.publish_markers)
        self.rectangle_marker = None
        self.path_marker = None
        self.get_logger().info(
            f'coverage marker publisher ready: marker_topic={marker_topic}'
        )

    def handle_rectangle(self, rectangle):
        points = [
            (point.x, point.y)
            for point in rectangle.polygon.points
        ]
        self.rectangle_marker = self.create_line_marker(
            rectangle.header.frame_id,
            points,
            marker_id=0,
            red=0.0,
            green=1.0,
            blue=0.0,
            z=0.03,
        )
        self.publish_markers()

    def handle_coverage_path(self, path):
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
        self.path_marker = self.create_line_marker(
            path.header.frame_id,
            points,
            marker_id=1,
            red=1.0,
            green=0.0,
            blue=0.0,
            z=0.05,
        )
        self.publish_markers()

    def publish_markers(self):
        for marker in (self.rectangle_marker, self.path_marker):
            if marker is None:
                continue
            marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(marker)

    def create_line_marker(
        self,
        frame_id,
        points,
        marker_id=0,
        namespace='robot_nav2_behaviors',
        red=0.0,
        green=1.0,
        blue=0.0,
        z=0.03,
    ):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.get_parameter('marker_line_width').value)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y in points]
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
