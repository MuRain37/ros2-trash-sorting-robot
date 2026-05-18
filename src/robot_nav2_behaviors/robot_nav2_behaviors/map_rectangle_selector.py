import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetRectangle


DEFAULT_MAP_YAML = (
    '/home/murain/WorkSpace/ros2_robot/'
    'src/robot_navgation2/maps/room.yaml'
)


class MapRectangleSelector(Node):
    CONTROL_BAR_HEIGHT = 64
    SEND_BUTTON = (16, 14, 148, 50)

    def __init__(self):
        super().__init__('map_rectangle_selector')
        self.declare_parameter('map_yaml', DEFAULT_MAP_YAML)
        self.declare_parameter('window_name', 'room map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rectangle_service', 'set_center_square_marker')

        self.map_yaml = self.get_parameter('map_yaml').value
        self.window_name = self.get_parameter('window_name').value
        self.frame_id = self.get_parameter('frame_id').value
        service_name = self.get_parameter('rectangle_service').value
        self.rectangle_client = self.create_client(SetRectangle, service_name)
        self.map_image_path, self.resolution, self.origin = self.load_map_info(
            self.map_yaml
        )

        self.image = cv2.imread(self.map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise RuntimeError(f'Failed to read map image: {self.map_image_path}')

        self.map_display = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        self.display = None
        self.dragging = False
        self.start_pixel = None
        self.end_pixel = None
        self.current_rectangle = None
        self.redraw_display()

    def load_map_info(self, yaml_path):
        info = self.read_simple_yaml(yaml_path)
        image_path = info.get('image')
        if image_path is None:
            raise RuntimeError(f'Map yaml has no image field: {yaml_path}')

        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        resolution = float(info.get('resolution', 1.0))
        origin = info.get('origin', [0.0, 0.0, 0.0])
        if isinstance(origin, str):
            origin = [
                float(value.strip())
                for value in origin.strip('[]').split(',')
                if value.strip()
            ]

        return image_path, resolution, origin

    def read_simple_yaml(self, yaml_path):
        data = {}
        with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
            for line in yaml_file:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                data[key.strip()] = value.strip()
        return data

    def pixel_to_map(self, pixel):
        px, py = pixel
        height = self.image.shape[0]
        map_x = self.origin[0] + px * self.resolution
        map_y = self.origin[1] + (height - py) * self.resolution
        return map_x, map_y

    def redraw_display(self):
        self.map_display = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        if self.start_pixel is None or self.end_pixel is None:
            self.draw_controls()
            return

        cv2.rectangle(
            self.map_display,
            self.start_pixel,
            self.end_pixel,
            (0, 0, 255),
            2,
        )
        self.draw_controls()

    def draw_controls(self):
        width = self.map_display.shape[1]
        controls = np.zeros((self.CONTROL_BAR_HEIGHT, width, 3), dtype=np.uint8)
        controls[:] = (245, 245, 245)

        x1, y1, x2, y2 = self.SEND_BUTTON
        enabled = self.current_rectangle is not None
        button_color = (70, 150, 70) if enabled else (170, 170, 170)
        cv2.rectangle(controls, (x1, y1), (x2, y2), button_color, -1)
        cv2.rectangle(controls, (x1, y1), (x2, y2), (80, 80, 80), 1)
        cv2.putText(
            controls,
            'Send',
            (x1 + 38, y1 + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self.display = cv2.vconcat([self.map_display, controls])

    def mouse_callback(self, event, x, y, flags, userdata):
        del flags, userdata
        image_height = self.image.shape[0]
        if y >= image_height:
            self.handle_control_click(event, x, y - image_height)
            return

        current = (x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start_pixel = current
            self.end_pixel = current
            self.current_rectangle = None
            self.redraw_display()
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end_pixel = current
            self.redraw_display()
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            self.end_pixel = current
            self.print_and_store_rectangle()
            self.redraw_display()

    def handle_control_click(self, event, x, y):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        x1, y1, x2, y2 = self.SEND_BUTTON
        if x1 <= x <= x2 and y1 <= y <= y2:
            if self.current_rectangle is None:
                self.get_logger().warn('draw a rectangle before sending')
                return
            self.call_rectangle_service(*self.current_rectangle)

    def get_rectangle_points(self):
        x1, y1 = self.start_pixel
        x2, y2 = self.end_pixel
        left, right = sorted((x1, x2))
        top, bottom = sorted((y1, y2))

        top_left = (left, top)
        top_right = (right, top)
        bottom_right = (right, bottom)
        bottom_left = (left, bottom)
        return top_left, top_right, bottom_right, bottom_left

    def print_and_store_rectangle(self):
        top_left, top_right, bottom_right, bottom_left = (
            self.get_rectangle_points()
        )
        map_top_left = self.pixel_to_map(top_left)
        map_top_right = self.pixel_to_map(top_right)
        map_bottom_right = self.pixel_to_map(bottom_right)
        map_bottom_left = self.pixel_to_map(bottom_left)

        self.get_logger().info(
            'rectangle pixels: '
            f'top_left={top_left}, bottom_right={bottom_right}'
        )
        self.get_logger().info(
            'rectangle map: '
            f'top_left=({map_top_left[0]:.3f}, {map_top_left[1]:.3f}), '
            f'bottom_right=({map_bottom_right[0]:.3f}, '
            f'{map_bottom_right[1]:.3f})'
        )
        min_x = min(map_bottom_left[0], map_bottom_right[0], map_top_right[0],
                    map_top_left[0])
        max_x = max(map_bottom_left[0], map_bottom_right[0], map_top_right[0],
                    map_top_left[0])
        min_y = min(map_bottom_left[1], map_bottom_right[1], map_top_right[1],
                    map_top_left[1])
        max_y = max(map_bottom_left[1], map_bottom_right[1], map_top_right[1],
                    map_top_left[1])
        self.current_rectangle = (min_x, min_y, max_x, max_y)
        self.get_logger().info('rectangle stored, click Send to publish')

    def call_rectangle_service(self, min_x, min_y, max_x, max_y):
        if not self.rectangle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'rectangle service is not available: '
                f'{self.rectangle_client.srv_name}'
            )
            return

        request = SetRectangle.Request()
        request.min_x = min_x
        request.min_y = min_y
        request.max_x = max_x
        request.max_y = max_y
        request.frame_id = self.frame_id
        future = self.rectangle_client.call_async(request)
        future.add_done_callback(self.handle_rectangle_response)

    def handle_rectangle_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'rectangle service call failed: {exc}')
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)

    def run(self):
        self.get_logger().info(f'Loaded map image: {self.map_image_path}')
        self.get_logger().info(
            f'resolution={self.resolution}, origin={self.origin}'
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        while rclpy.ok():
            cv2.imshow(self.window_name, self.display)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), 27):
                break
            rclpy.spin_once(self, timeout_sec=0.0)

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = MapRectangleSelector()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
