"""覆盖路径生成节点：将矩形区域转换为往返式清扫路径。"""

import math

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from robot_interfaces.srv import GenerateCoveragePath


class CoveragePathGenerator(Node):
    """提供覆盖路径生成服务，并发布预览矩形和路径。"""

    def __init__(self):
        super().__init__('coverage_path_generator')
        # 输出路径和预览 Marker 使用的默认坐标系
        self.declare_parameter('frame_id', 'map')
        # 往返扫描线间距（m）
        self.declare_parameter('zigzag_spacing', 1.0)
        # 覆盖路线相对矩形边界的内缩距离（m）
        self.declare_parameter('zigzag_edge_inset', 0.10)
        # 覆盖路径生成服务名
        self.declare_parameter(
            'coverage_path_service',
            'generate_coverage_path',
        )
        # 矩形预览话题
        self.declare_parameter('rectangle_topic', 'coverage_rectangle')
        # 路径预览话题
        self.declare_parameter('coverage_path_topic', 'coverage_path_preview')

        service_name = self.get_parameter('coverage_path_service').value
        rectangle_topic = self.get_parameter('rectangle_topic').value
        coverage_path_topic = self.get_parameter('coverage_path_topic').value

        self.coverage_path_service = self.create_service(
            GenerateCoveragePath,
            service_name,
            self.handle_generate_coverage_path,
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

    def handle_generate_coverage_path(self, request, response):
        """根据矩形边界生成覆盖路径。"""
        min_x = min(request.min_x, request.max_x)
        max_x = max(request.min_x, request.max_x)
        min_y = min(request.min_y, request.max_y)
        max_y = max(request.min_y, request.max_y)

        if min_x == max_x or min_y == max_y:
            response.success = False
            response.message = (
                'rectangle width and height must be greater than 0'
            )
            return response

        frame_id = request.frame_id or self.get_parameter('frame_id').value
        start_point = None
        if request.use_start:
            # 起点用于选择离机器人当前位置最近的扫描入口。
            start_point = (
                float(request.start_x),
                float(request.start_y),
            )

        zigzag_points = self.create_zigzag_points(
            min_x,
            min_y,
            max_x,
            max_y,
            start_point,
        )
        coverage_path = self.create_coverage_path(frame_id, zigzag_points)
        self.publish_rectangle(frame_id, min_x, min_y, max_x, max_y)
        self.path_publisher.publish(coverage_path)

        response.success = True
        response.message = 'coverage path generated'
        response.coverage_path = coverage_path
        self.get_logger().info(
            'generated coverage path: '
            f'min=({min_x:.3f}, {min_y:.3f}), '
            f'max=({max_x:.3f}, {max_y:.3f}), '
            f'points={len(zigzag_points)}, '
            f'start={self.format_start_point(start_point)}, '
            f'frame_id={frame_id}'
        )
        return response

    def format_start_point(self, start_point):
        if start_point is None:
            return 'default'

        start_x, start_y = start_point
        return f'({start_x:.3f}, {start_y:.3f})'

    def publish_rectangle(self, frame_id, min_x, min_y, max_x, max_y):
        """发布矩形边界预览。"""
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

    def create_coverage_path(self, frame_id, zigzag_points):
        """把二维点列转换为 nav_msgs/Path。"""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [
            self.create_pose_stamped(frame_id, x, y, yaw)
            for x, y, yaw in self.iter_points_with_yaw(zigzag_points)
        ]
        return path

    def iter_points_with_yaw(self, points):
        last_yaw = 0.0
        for index, point in enumerate(points):
            x, y = point
            if index < len(points) - 1:
                next_x, next_y = points[index + 1]
                dx = next_x - x
                dy = next_y - y
                if dx != 0.0 or dy != 0.0:
                    last_yaw = math.atan2(dy, dx)
            # 最后一个点沿用上一段朝向，避免没有下一点时 yaw 归零。
            yield x, y, last_yaw

    def create_pose_stamped(self, frame_id, x, y, yaw):
        """创建单个路径点。"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def create_zigzag_points(
        self,
        min_x,
        min_y,
        max_x,
        max_y,
        start_point=None,
    ):
        """按区域长边方向生成往返扫描点。"""
        spacing = float(self.get_parameter('zigzag_spacing').value)
        if spacing <= 0.0:
            # 非法间距退化为一条扫描线，保证仍能生成路径。
            spacing = max_x - min_x

        width = max_x - min_x
        height = max_y - min_y
        if width <= 0.0 or height <= 0.0:
            return []

        inset = max(0.0, float(self.get_parameter('zigzag_edge_inset').value))
        max_inset = max(0.0, min(width, height) / 2.0 - 1e-6)
        inset = min(inset, max_inset)
        min_x += inset
        min_y += inset
        max_x -= inset
        max_y -= inset
        width = max_x - min_x
        height = max_y - min_y

        if width >= height:
            # 宽区域沿 x 方向扫，扫描线在 y 方向排列。
            tracks = self.create_tracks(min_y, max_y, spacing)
            return self.select_zigzag(
                tracks,
                start_point,
                self.build_horizontal_zigzag,
                min_x,
                max_x,
            )

        # 高区域沿 y 方向扫，扫描线在 x 方向排列。
        tracks = self.create_tracks(min_x, max_x, spacing)
        return self.select_zigzag(
            tracks,
            start_point,
            self.build_vertical_zigzag,
            min_y,
            max_y,
        )

    def create_tracks(self, min_value, max_value, spacing):
        """生成等间距扫描线坐标。"""
        span = max_value - min_value
        track_count = max(1, int(math.ceil(span / spacing)))
        # 居中分布扫描线，避免最后一条贴边而另一侧留下大空隙。
        margin = (span - (track_count - 1) * spacing) / 2.0
        return [
            min_value + margin + index * spacing
            for index in range(track_count)
        ]

    def select_zigzag(self, tracks, start_point, builder, low, high):
        """选择离起点最近的扫描方向。"""
        reversed_tracks = list(reversed(tracks))
        # 同一组扫描线有四种入口方向，选离 start_point 最近的一种。
        variants = [
            builder(tracks, low, high, True),
            builder(tracks, low, high, False),
            builder(reversed_tracks, low, high, True),
            builder(reversed_tracks, low, high, False),
        ]
        return self.deduplicate_points(
            self.select_closest_variant(variants, start_point)
        )

    def select_closest_variant(self, variants, start_point):
        if not variants:
            return []
        if start_point is None:
            return variants[0]

        start_x, start_y = start_point
        return min(
            variants,
            key=lambda points: math.inf if not points else math.hypot(
                points[0][0] - start_x,
                points[0][1] - start_y,
            ),
        )

    def deduplicate_points(self, points, epsilon=1e-6):
        if not points:
            return []

        deduplicated = [points[0]]
        for x, y in points[1:]:
            last_x, last_y = deduplicated[-1]
            if abs(x - last_x) <= epsilon and abs(y - last_y) <= epsilon:
                continue
            deduplicated.append((x, y))
        return deduplicated

    def build_vertical_zigzag(
        self,
        track_xs,
        min_y,
        max_y,
        start_from_bottom,
    ):
        points = []
        bottom_to_top = start_from_bottom

        for index, x in enumerate(track_xs):
            if bottom_to_top:
                self.append_line_with_midpoint(
                    points,
                    (x, min_y),
                    (x, max_y),
                )
            else:
                self.append_line_with_midpoint(
                    points,
                    (x, max_y),
                    (x, min_y),
                )

            if index < len(track_xs) - 1:
                next_x = track_xs[index + 1]
                if bottom_to_top:
                    points.append((next_x, max_y))
                else:
                    points.append((next_x, min_y))
            bottom_to_top = not bottom_to_top

        return points

    def build_horizontal_zigzag(
        self,
        track_ys,
        min_x,
        max_x,
        start_from_left,
    ):
        points = []
        left_to_right = start_from_left

        for index, y in enumerate(track_ys):
            if left_to_right:
                self.append_line_with_midpoint(
                    points,
                    (min_x, y),
                    (max_x, y),
                )
            else:
                self.append_line_with_midpoint(
                    points,
                    (max_x, y),
                    (min_x, y),
                )

            if index < len(track_ys) - 1:
                next_y = track_ys[index + 1]
                if left_to_right:
                    points.append((max_x, next_y))
                else:
                    points.append((min_x, next_y))
            left_to_right = not left_to_right

        return points

    def append_line_with_midpoint(self, points, start, end):
        start_x, start_y = start
        end_x, end_y = end
        # 每条直线加入中点，让 Nav2 路径更贴近完整覆盖线段。
        points.extend([
            start,
            (
                (start_x + end_x) / 2.0,
                (start_y + end_y) / 2.0,
            ),
            end,
        ])


def main(args=None):
    """启动覆盖路径生成节点。"""
    rclpy.init(args=args)
    node = CoveragePathGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
