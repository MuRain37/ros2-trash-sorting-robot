"""垃圾桶识别任务服务：返回桶中心偏移和深度，不执行底盘/机械臂动作."""

import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.srv import DetectDumpBucket
from sensor_msgs.msg import Image


DEFAULT_COLOR_TOPIC = '/camera/real_camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/real_camera/aligned_depth_to_color/image_raw'
DEFAULT_SERVICE_NAME = 'detect_dump_bucket'

DETECTED_MESSAGE = 'largest contour detected within depth threshold'
NOT_DETECTED_MESSAGE = 'largest contour not detected within depth threshold'


class DumpTaskNode(Node):
    """订阅 RGB/Depth 图像，并通过服务返回垃圾桶中心偏移和深度."""

    def __init__(self):
        super().__init__('dump_task')

        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('service_name', DEFAULT_SERVICE_NAME)
        self.declare_parameter('mask_min_pixels', 100)
        self.declare_parameter('depth_threshold_m', 0.35)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('center_depth_window_px', 10)

        self._lock = threading.Lock()
        self._latest_color = None
        self._latest_depth = None

        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        service_name = str(self.get_parameter('service_name').value)
        self._color_topic = color_topic
        self._depth_topic = depth_topic

        self._color_subscription = self.create_subscription(
            Image,
            color_topic,
            self._on_color_image,
            qos_profile_sensor_data,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self._detect_service = self.create_service(
            DetectDumpBucket,
            service_name,
            self._on_detect_request,
        )

        self.get_logger().info(
            'Dump task ready: '
            f'color_topic={color_topic}, depth_topic={depth_topic}, '
            f'service={service_name}'
        )

    def _on_color_image(self, msg):
        with self._lock:
            self._latest_color = msg

    def _on_depth_image(self, msg):
        with self._lock:
            self._latest_depth = msg

    def _on_detect_request(self, request, response):
        del request
        with self._lock:
            color_msg = self._latest_color
            depth_msg = self._latest_depth

        if color_msg is None or depth_msg is None:
            missing = []
            if color_msg is None:
                missing.append(f'color({self._color_topic})')
            if depth_msg is None:
                missing.append(f'depth({self._depth_topic})')
            fill_empty_response(
                response,
                False,
                f'waiting for {", ".join(missing)} images',
            )
            return response

        try:
            result = self._detect_bucket(color_msg, depth_msg)
        except ValueError as e:
            fill_empty_response(response, False, str(e))
            return response

        response.success = result['mask_pixels'] > int(
            self.get_parameter('mask_min_pixels').value
        )
        response.detected = response.success
        response.mask_pixels = int(result['mask_pixels'])
        response.mean_depth_m = float(result['mean_depth_m'])
        response.center_x_px = float(result['center_x_px'])
        response.center_y_px = float(result['center_y_px'])
        response.image_center_x_px = float(result['image_center_x_px'])
        response.image_center_y_px = float(result['image_center_y_px'])
        response.offset_x_px = float(result['offset_x_px'])
        response.offset_y_px = float(result['offset_y_px'])
        response.offset_x_ratio = float(result['offset_x_ratio'])
        response.offset_y_ratio = float(result['offset_y_ratio'])

        if response.success and result['within_depth']:
            response.signal = 1
            response.message = (
                f'{DETECTED_MESSAGE}; signal=1; '
                f'mask_pixels={response.mask_pixels}; '
                f'mean_depth_m={response.mean_depth_m:.3f}; '
                f'offset_x_px={response.offset_x_px:.1f}; '
                f'offset_x_ratio={response.offset_x_ratio:.4f}'
            )
            return response

        response.signal = 2
        depth_text = (
            'nan'
            if math.isnan(response.mean_depth_m)
            else f'{response.mean_depth_m:.3f}'
        )
        response.message = (
            f'{NOT_DETECTED_MESSAGE}; signal=2; '
            f'mask_pixels={response.mask_pixels}; '
            f'mean_depth_m={depth_text}; '
            f'offset_x_px={response.offset_x_px:.1f}; '
            f'offset_x_ratio={response.offset_x_ratio:.4f}'
        )
        return response

    def _detect_bucket(self, color_msg, depth_msg):
        color = image_to_rgb_array(color_msg)
        depth_m = image_to_depth_meters(
            depth_msg,
            float(self.get_parameter('depth_scale').value),
        )

        if color.shape[:2] != depth_m.shape[:2]:
            raise ValueError(
                'color and depth image sizes do not match: '
                f'color={color.shape[:2]}, depth={depth_m.shape[:2]}'
            )

        component = largest_contour_component(color)
        mask_pixels = int(np.count_nonzero(component))

        mask_min_pixels = int(self.get_parameter('mask_min_pixels').value)
        if mask_pixels <= mask_min_pixels:
            return detection_result(color.shape, mask_pixels)

        ys, xs = np.nonzero(component)
        center_x_px = float(np.mean(xs))
        center_y_px = float(np.mean(ys))

        valid_depth = center_window_depth_values(
            depth_m,
            component,
            center_x_px,
            center_y_px,
            int(self.get_parameter('center_depth_window_px').value),
        )
        if valid_depth.size == 0:
            valid_depth = contour_depth_values(depth_m, component)
        if valid_depth.size == 0:
            self.get_logger().warning(
                'dump depth stats: no valid depth near contour center '
                f'or inside contour, mask_pixels={mask_pixels}, '
                f'center=({center_x_px:.1f}, {center_y_px:.1f})'
            )
            return detection_result(
                color.shape,
                mask_pixels,
                center_x_px=center_x_px,
                center_y_px=center_y_px,
            )

        mean_depth_m = robust_depth_m(valid_depth)
        min_depth_m = float(np.min(valid_depth))
        max_depth_m = float(np.max(valid_depth))
        self.get_logger().info(
            'dump depth stats: '
            f'center_mean={mean_depth_m:.3f}m, '
            f'min={min_depth_m:.3f}m, max={max_depth_m:.3f}m, '
            f'valid_pixels={valid_depth.size}'
        )
        depth_threshold_m = float(self.get_parameter('depth_threshold_m').value)
        return detection_result(
            color.shape,
            mask_pixels,
            center_x_px=center_x_px,
            center_y_px=center_y_px,
            mean_depth_m=mean_depth_m,
            within_depth=mean_depth_m <= depth_threshold_m,
        )


def fill_empty_response(response, success, message):
    response.success = bool(success)
    response.message = str(message)
    response.detected = False
    response.mask_pixels = 0
    response.mean_depth_m = math.nan
    response.center_x_px = math.nan
    response.center_y_px = math.nan
    response.image_center_x_px = math.nan
    response.image_center_y_px = math.nan
    response.offset_x_px = math.nan
    response.offset_y_px = math.nan
    response.offset_x_ratio = math.nan
    response.offset_y_ratio = math.nan
    response.signal = 2


def detection_result(
    color_shape,
    mask_pixels,
    center_x_px=math.nan,
    center_y_px=math.nan,
    mean_depth_m=math.nan,
    within_depth=False,
):
    height, width = color_shape[:2]
    image_center_x = (float(width) - 1.0) * 0.5
    image_center_y = (float(height) - 1.0) * 0.5
    offset_x_px = center_x_px - image_center_x
    offset_y_px = center_y_px - image_center_y
    half_width = max(1.0, image_center_x)
    half_height = max(1.0, image_center_y)
    return {
        'mask_pixels': int(mask_pixels),
        'mean_depth_m': float(mean_depth_m),
        'center_x_px': float(center_x_px),
        'center_y_px': float(center_y_px),
        'image_center_x_px': image_center_x,
        'image_center_y_px': image_center_y,
        'offset_x_px': float(offset_x_px),
        'offset_y_px': float(offset_y_px),
        'offset_x_ratio': float(offset_x_px / half_width),
        'offset_y_ratio': float(offset_y_px / half_height),
        'within_depth': bool(within_depth),
    }


def largest_contour_component(color):
    """不做 RGB 色彩筛选，直接取画面中最大的外轮廓区域。"""
    gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((5, 5), dtype=np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    component = np.zeros(gray.shape, dtype=np.uint8)
    if not contours:
        return component.astype(bool)

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) <= 1.0:
        return component.astype(bool)

    cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
    return component.astype(bool)


def center_window_depth_values(depth_m, component, center_x_px, center_y_px, window_px):
    """Return valid depth samples near the detected contour center."""
    height, width = depth_m.shape[:2]
    window_px = max(1, int(window_px))
    if window_px % 2 == 0:
        window_px += 1
    center_x = int(round(center_x_px))
    center_y = int(round(center_y_px))

    for scale in (1, 3, 5, 9):
        radius = (window_px * scale) // 2
        x0 = max(0, center_x - radius)
        x1 = min(width, center_x + radius + 1)
        y0 = max(0, center_y - radius)
        y1 = min(height, center_y + radius + 1)

        depth_roi = depth_m[y0:y1, x0:x1]
        mask_roi = component[y0:y1, x0:x1]
        values = depth_roi[
            mask_roi & np.isfinite(depth_roi) & (depth_roi > 0.0)
        ]
        if values.size:
            return values

        values = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0.0)]
        if values.size:
            return values
    return np.asarray([], dtype=np.float32)


def contour_depth_values(depth_m, component):
    """Return robust valid depth samples inside the detected contour."""
    values = depth_m[component & np.isfinite(depth_m) & (depth_m > 0.0)]
    if not values.size:
        return values
    high = np.percentile(values, 70)
    near_values = values[values <= high]
    return near_values if near_values.size else values


def robust_depth_m(values):
    """Prefer the near surface of the bucket over background depth."""
    values = np.asarray(values, dtype=np.float32)
    values = values[np.isfinite(values) & (values > 0.0)]
    if not values.size:
        return math.nan
    near = values[values <= np.percentile(values, 40)]
    return float(np.median(near if near.size else values))


def image_to_rgb_array(msg):
    """将常见 ROS Image 编码转换为 RGB ndarray."""
    encoding = msg.encoding.lower()
    channels = _encoding_channels(encoding)
    dtype = _encoding_dtype(encoding)
    expected_step = msg.width * channels * np.dtype(dtype).itemsize
    if msg.step < expected_step:
        raise ValueError(
            f'invalid image step for {msg.encoding}: '
            f'step={msg.step}, expected at least {expected_step}'
        )

    array = np.frombuffer(msg.data, dtype=dtype)
    row_items = msg.step // np.dtype(dtype).itemsize
    array = array.reshape((msg.height, row_items))[:, : msg.width * channels]
    array = array.reshape((msg.height, msg.width, channels))

    if channels == 1:
        array = np.repeat(array, 3, axis=2)
    elif channels == 4:
        array = array[:, :, :3]

    if encoding.startswith('bgr'):
        array = array[:, :, ::-1]
    return array.astype(np.uint8, copy=False)


def image_to_depth_meters(msg, depth_scale):
    """将 16UC1/mm 或 32FC1/m 深度图转换为米."""
    encoding = msg.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        depth = _image_to_single_channel(msg, np.uint16).astype(np.float32)
        return depth * depth_scale
    if encoding == '32fc1':
        return _image_to_single_channel(msg, np.float32)
    raise ValueError(f'unsupported depth image encoding: {msg.encoding}')


def _image_to_single_channel(msg, dtype):
    item_size = np.dtype(dtype).itemsize
    expected_step = msg.width * item_size
    if msg.step < expected_step:
        raise ValueError(
            f'invalid image step for {msg.encoding}: '
            f'step={msg.step}, expected at least {expected_step}'
        )

    array = np.frombuffer(msg.data, dtype=dtype)
    row_items = msg.step // item_size
    return array.reshape((msg.height, row_items))[:, : msg.width]


def _encoding_channels(encoding):
    if encoding in ('rgb8', 'bgr8'):
        return 3
    if encoding in ('rgba8', 'bgra8'):
        return 4
    if encoding in ('mono8', '8uc1'):
        return 1
    raise ValueError(f'unsupported color image encoding: {encoding}')


def _encoding_dtype(encoding):
    if encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8', '8uc1'):
        return np.uint8
    raise ValueError(f'unsupported color image encoding: {encoding}')


def main(args=None):
    """启动垃圾桶识别服务节点."""
    rclpy.init(args=args)
    node = DumpTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
