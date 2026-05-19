import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from robot_interfaces.srv import SetRectangle


class CoveragePathGenerator(Node):
    def __init__(self):
        super().__init__('coverage_path_generator')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('zigzag_spacing', 1.0)
        self.declare_parameter('rectangle_service', 'set_center_square_marker')
        self.declare_parameter('rectangle_topic', 'coverage_rectangle')
        self.declare_parameter('coverage_path_topic', 'coverage_path_preview')

        service_name = self.get_parameter('rectangle_service').value
        rectangle_topic = self.get_parameter('rectangle_topic').value
        coverage_path_topic = self.get_parameter('coverage_path_topic').value

        self.rectangle_service = self.create_service(
            SetRectangle,
            service_name,
            self.handle_set_rectangle,
        )
        self.rectangle_publisher = self.create_publisher(
            PolygonStamped,
            rectangle_topic,
            10,
        )
        self.path_publisher = self.create_publisher(
            Path,
            coverage_path_topic,
            10,
        )
        self.get_logger().info(f'coverage path service ready: {service_name}')

    def handle_set_rectangle(self, request, response):
        min_x = min(request.min_x, request.max_x)
        max_x = max(request.min_x, request.max_x)
        min_y = min(request.min_y, request.max_y)
        max_y = max(request.min_y, request.max_y)

        if min_x == max_x or min_y == max_y:
            response.success = False
            response.message = 'rectangle width and height must be greater than 0'
            return response

        frame_id = request.frame_id or self.get_parameter('frame_id').value
        zigzag_points = self.create_zigzag_points(min_x, min_y, max_x, max_y)
        self.publish_rectangle(frame_id, min_x, min_y, max_x, max_y)
        self.publish_coverage_path(frame_id, zigzag_points)

        response.success = True
        response.message = 'coverage path generated'
        response.path_points = [
            Point(x=x, y=y, z=0.0)
            for x, y in zigzag_points
        ]
        self.get_logger().info(
            'generated coverage path: '
            f'min=({min_x:.3f}, {min_y:.3f}), '
            f'max=({max_x:.3f}, {max_y:.3f}), '
            f'points={len(zigzag_points)}, frame_id={frame_id}'
        )
        return response

    def publish_rectangle(self, frame_id, min_x, min_y, max_x, max_y):
        rectangle = PolygonStamped()
        rectangle.header.frame_id = frame_id
        rectangle.header.stamp = self.get_clock().now().to_msg()
        rectangle.polygon.points = [
            Point32(x=float(min_x), y=float(min_y), z=0.0),
            Point32(x=float(max_x), y=float(min_y), z=0.0),
            Point32(x=float(max_x), y=float(max_y), z=0.0),
            Point32(x=float(min_x), y=float(max_y), z=0.0),
            Point32(x=float(min_x), y=float(min_y), z=0.0),
        ]
        self.rectangle_publisher.publish(rectangle)

    def publish_coverage_path(self, frame_id, zigzag_points):
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [
            self.create_pose_stamped(frame_id, x, y)
            for x, y in zigzag_points
        ]
        self.path_publisher.publish(path)

    def create_pose_stamped(self, frame_id, x, y):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def create_zigzag_points(self, min_x, min_y, max_x, max_y):
        spacing = float(self.get_parameter('zigzag_spacing').value)
        if spacing <= 0.0:
            spacing = max_x - min_x

        width = max_x - min_x
        height = max_y - min_y
        if width <= 0.0 or height <= 0.0:
            return []

        if width >= height:
            track_count = max(1, int(height // spacing) + 1)
            margin = (height - (track_count - 1) * spacing) / 2.0
            track_ys = [
                min_y + margin + index * spacing
                for index in range(track_count)
            ]
            return self.build_horizontal_zigzag(track_ys, min_x, max_x)

        track_count = max(1, int(width // spacing) + 1)
        margin = (width - (track_count - 1) * spacing) / 2.0
        track_xs = [
            min_x + margin + index * spacing
            for index in range(track_count)
        ]
        return self.build_vertical_zigzag(track_xs, min_y, max_y)

    def build_vertical_zigzag(self, track_xs, min_y, max_y):
        points = []
        bottom_to_top = True

        for index, x in enumerate(track_xs):
            if bottom_to_top:
                points.append((x, min_y))
                points.append((x, max_y))
            else:
                points.append((x, max_y))
                points.append((x, min_y))

            if index < len(track_xs) - 1:
                next_x = track_xs[index + 1]
                if bottom_to_top:
                    points.append((next_x, max_y))
                else:
                    points.append((next_x, min_y))
            bottom_to_top = not bottom_to_top

        return points

    def build_horizontal_zigzag(self, track_ys, min_x, max_x):
        points = []
        left_to_right = True

        for index, y in enumerate(track_ys):
            if left_to_right:
                points.append((min_x, y))
                points.append((max_x, y))
            else:
                points.append((max_x, y))
                points.append((min_x, y))

            if index < len(track_ys) - 1:
                next_y = track_ys[index + 1]
                if left_to_right:
                    points.append((max_x, next_y))
                else:
                    points.append((min_x, next_y))
            left_to_right = not left_to_right

        return points


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
