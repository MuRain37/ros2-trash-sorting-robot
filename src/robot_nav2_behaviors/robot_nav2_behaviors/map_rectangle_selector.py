import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import AddCoverageRectangle


DEFAULT_MAP_YAML = (
    '/home/murain/WorkSpace/robot/'
    'src/robot_navgation2/maps/room.yaml'
)


class MapRectangleSelector(Node):
    CONTROL_BAR_HEIGHT = 64
    SEND_BUTTON = (16, 14, 148, 50)
    UNDO_BUTTON = (164, 14, 296, 50)

    def __init__(self):
        super().__init__('map_rectangle_selector')
        self.declare_parameter('map_yaml', DEFAULT_MAP_YAML)
        self.declare_parameter('window_name', 'room map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('task_service', 'add_coverage_rectangle')

        self.map_yaml = self.get_parameter('map_yaml').value
        self.window_name = self.get_parameter('window_name').value
        self.frame_id = self.get_parameter('frame_id').value
        service_name = self.get_parameter('task_service').value
        self.task_client = self.create_client(
            AddCoverageRectangle,
            service_name,
        )
        self.map_image_path, self.resolution, self.origin = self.load_map_info(
            self.map_yaml
        )

        self.image = cv2.imread(self.map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise RuntimeError(
                f'Failed to read map image: {self.map_image_path}'
            )

        self.map_display = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        self.display = None
        self.dragging = False
        self.start_pixel = None
        self.end_pixel = None
        self.rectangles = []
        self.sent = False
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
        self.draw_stored_rectangles()

        if self.start_pixel is not None and self.end_pixel is not None:
            cv2.rectangle(
                self.map_display,
                self.start_pixel,
                self.end_pixel,
                (0, 0, 255),
                2,
            )
        self.draw_controls()

    def draw_stored_rectangles(self):
        for rectangle in self.rectangles:
            top_left = rectangle['top_left']
            bottom_right = rectangle['bottom_right']
            color = (0, 0, 255)
            if rectangle['submitting']:
                color = (0, 255, 255)
            elif rectangle['queued']:
                color = (0, 160, 255)
            cv2.rectangle(self.map_display, top_left, bottom_right, color, 2)
            self.draw_rectangle_index(rectangle)

    def draw_rectangle_index(self, rectangle):
        top_left = rectangle['top_left']
        bottom_right = rectangle['bottom_right']
        center_x = (top_left[0] + bottom_right[0]) // 2
        center_y = (top_left[1] + bottom_right[1]) // 2
        text = str(rectangle['sequence'])
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.7
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            text,
            font,
            scale,
            thickness,
        )
        origin = (
            center_x - text_width // 2,
            center_y + (text_height + baseline) // 2,
        )
        cv2.putText(
            self.map_display,
            text,
            origin,
            font,
            scale,
            (0, 0, 0),
            thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            self.map_display,
            text,
            origin,
            font,
            scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def draw_controls(self):
        width = self.map_display.shape[1]
        controls = np.zeros(
            (self.CONTROL_BAR_HEIGHT, width, 3),
            dtype=np.uint8,
        )
        controls[:] = (245, 245, 245)

        x1, y1, x2, y2 = self.SEND_BUTTON
        enabled = self.can_send()
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

        x1, y1, x2, y2 = self.UNDO_BUTTON
        enabled = self.can_undo()
        button_color = (80, 120, 210) if enabled else (170, 170, 170)
        cv2.rectangle(controls, (x1, y1), (x2, y2), button_color, -1)
        cv2.rectangle(controls, (x1, y1), (x2, y2), (80, 80, 80), 1)
        cv2.putText(
            controls,
            'Undo',
            (x1 + 36, y1 + 25),
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

        if self.sent:
            return

        current = (x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start_pixel = current
            self.end_pixel = current
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
            if not self.can_send():
                self.get_logger().warn('draw a rectangle before adding')
                return
            self.call_pending_task_services()
            self.sent = True
            self.redraw_display()
            return

        x1, y1, x2, y2 = self.UNDO_BUTTON
        if x1 <= x <= x2 and y1 <= y <= y2:
            if not self.can_undo():
                self.get_logger().warn('no rectangle to undo')
                return
            removed = self.rectangles.pop()
            self.get_logger().info(
                f'rectangle {removed["sequence"]} removed'
            )
            self.redraw_display()

    def has_pending_rectangles(self):
        return any(
            not rectangle['queued'] and not rectangle['submitting']
            for rectangle in self.rectangles
        )

    def can_send(self):
        return not self.sent and self.has_pending_rectangles()

    def can_undo(self):
        return (
            not self.sent
            and bool(self.rectangles)
            and not any(
                rectangle['submitting'] for rectangle in self.rectangles
            )
        )

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

        if top_left[0] == bottom_right[0] or top_left[1] == bottom_right[1]:
            self.get_logger().warn(
                'rectangle width and height must be greater than 0'
            )
            self.start_pixel = None
            self.end_pixel = None
            return

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
        rectangle = {
            'sequence': len(self.rectangles) + 1,
            'top_left': top_left,
            'bottom_right': bottom_right,
            'map_bounds': (min_x, min_y, max_x, max_y),
            'queued': False,
            'submitting': False,
        }
        self.rectangles.append(rectangle)
        self.start_pixel = None
        self.end_pixel = None
        self.get_logger().info(
            f'rectangle {rectangle["sequence"]} stored, click Send to queue'
        )

    def call_pending_task_services(self):
        for rectangle in self.rectangles:
            if rectangle['queued'] or rectangle['submitting']:
                continue
            self.call_task_service(rectangle)

    def call_task_service(self, rectangle):
        if not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'coverage task service is not available: '
                f'{self.task_client.srv_name}'
            )
            return

        rectangle['submitting'] = True
        self.redraw_display()
        min_x, min_y, max_x, max_y = rectangle['map_bounds']
        request = AddCoverageRectangle.Request()
        request.min_x = min_x
        request.min_y = min_y
        request.max_x = max_x
        request.max_y = max_y
        request.frame_id = self.frame_id
        future = self.task_client.call_async(request)
        future.add_done_callback(
            lambda future, rectangle=rectangle: self.handle_task_response(
                future,
                rectangle,
            )
        )

    def handle_task_response(self, future, rectangle):
        try:
            response = future.result()
        except Exception as exc:
            rectangle['submitting'] = False
            self.get_logger().error(
                f'coverage task service call failed: {exc}'
            )
            self.redraw_display()
            return

        rectangle['submitting'] = False
        if response.success:
            rectangle['queued'] = True
            self.get_logger().info(
                f'rectangle {rectangle["sequence"]}: '
                f'{response.message}; queued={response.queued_count}'
            )
            self.redraw_display()
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
