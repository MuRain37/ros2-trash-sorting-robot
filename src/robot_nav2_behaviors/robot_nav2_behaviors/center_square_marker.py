import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from robot_interfaces.srv import SetRectangle
from visualization_msgs.msg import Marker


class CenterSquareMarker(Node):
    def __init__(self):
        super().__init__('center_square_marker')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('zigzag_spacing', 1.0)
        self.declare_parameter('marker_line_width', 0.02)
        self.declare_parameter('rectangle_service', 'set_center_square_marker')

        self.publisher = self.create_publisher(Marker, 'center_square_marker', 10)
        service_name = self.get_parameter('rectangle_service').value
        self.rectangle_service = self.create_service(
            SetRectangle,
            service_name,
            self.handle_set_rectangle,
        )
        self.get_logger().info(f'rectangle service ready: {service_name}')
        period = self.get_parameter('publish_period').value
        self.timer = self.create_timer(period, self.publish_marker)
        self.current_markers = None

    def publish_marker(self):
        if self.current_markers is None:
            return

        for marker in self.current_markers:
            marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(marker)

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
        self.current_markers = self.create_rectangle_markers(
            frame_id,
            min_x,
            min_y,
            max_x,
            max_y,
        )
        for marker in self.current_markers:
            self.publisher.publish(marker)

        response.success = True
        response.message = 'rectangle and zigzag markers published'
        self.get_logger().info(
            'set rectangle marker: '
            f'min=({min_x:.3f}, {min_y:.3f}), '
            f'max=({max_x:.3f}, {max_y:.3f}), frame_id={frame_id}'
        )
        return response

    def create_rectangle_markers(self, frame_id, min_x, min_y, max_x, max_y):
        rectangle_corners = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
        ]
        zigzag_points = self.create_zigzag_points(min_x, min_y, max_x, max_y)
        return [
            self.create_line_marker(
                frame_id,
                rectangle_corners,
                marker_id=0,
                red=0.0,
                green=1.0,
                blue=0.0,
                z=0.03,
            ),
            self.create_line_marker(
                frame_id,
                zigzag_points,
                marker_id=1,
                red=1.0,
                green=0.0,
                blue=0.0,
                z=0.05,
            ),
        ]

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

    def create_rectangle_marker(
        self,
        frame_id,
        corners,
        marker_id=0,
        namespace='robot_nav2_behaviors',
    ):
        return self.create_line_marker(
            frame_id,
            corners,
            marker_id,
            namespace,
            red=0.0,
            green=1.0,
            blue=0.0,
            z=0.03,
        )

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
    node = CenterSquareMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
