"""垃圾桶识别任务服务：返回桶中心偏移和深度，不执行底盘/机械臂动作."""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import DetectDumpBucket
from sensor_msgs.msg import Image


DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_SERVICE_NAME = 'detect_dump_bucket'

DETECTED_MESSAGE = 'dump bucket detected within depth threshold'
NOT_DETECTED_MESSAGE = 'dump bucket not detected within depth threshold'


class DumpTaskNode(Node):
    """订阅 RGB/Depth 图像，并通过服务返回垃圾桶中心偏移和深度."""

    def __init__(self):
        super().__init__('dump_task')

        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('service_name', DEFAULT_SERVICE_NAME)
        self.declare_parameter('mask_min_pixels', 100)
        self.declare_parameter('depth_threshold_m', 0.20)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('lower_rgb', [70, 70, 70])
        self.declare_parameter('upper_rgb', [115, 110, 100])

        self._lock = threading.Lock()
        self._latest_color = None
        self._latest_depth = None

        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        service_name = str(self.get_parameter('service_name').value)

        self._color_subscription = self.create_subscription(
            Image,
            color_topic,
            self._on_color_image,
            10,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self._on_depth_image,
            10,
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
            fill_empty_response(response, False, 'waiting for color and depth images')
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

        lower_rgb = np.array(
            list(self.get_parameter('lower_rgb').value),
            dtype=np.uint8,
        )
        upper_rgb = np.array(
            list(self.get_parameter('upper_rgb').value),
            dtype=np.uint8,
        )
        mask = np.all((color >= lower_rgb) & (color <= upper_rgb), axis=2)
        component = largest_mask_component(mask)
        mask_pixels = int(np.count_nonzero(component))

        mask_min_pixels = int(self.get_parameter('mask_min_pixels').value)
        if mask_pixels <= mask_min_pixels:
            return detection_result(color.shape, mask_pixels)

        ys, xs = np.nonzero(component)
        center_x_px = float(np.mean(xs))
        center_y_px = float(np.mean(ys))

        masked_depth = depth_m[component]
        valid_depth = masked_depth[np.isfinite(masked_depth) & (masked_depth > 0.0)]
        if valid_depth.size == 0:
            return detection_result(
                color.shape,
                mask_pixels,
                center_x_px=center_x_px,
                center_y_px=center_y_px,
            )

        mean_depth_m = float(np.mean(valid_depth))
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


def largest_mask_component(mask):
    """从 RGB 掩膜中取最大连通区域，避免零散噪声影响中心点."""
    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=bool)
    best_pixels = []

    for start_y, start_x in zip(*np.nonzero(mask)):
        if visited[start_y, start_x]:
            continue
        stack = [(int(start_y), int(start_x))]
        visited[start_y, start_x] = True
        pixels = []
        while stack:
            y, x = stack.pop()
            pixels.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if (
                    0 <= ny < height
                    and 0 <= nx < width
                    and mask[ny, nx]
                    and not visited[ny, nx]
                ):
                    visited[ny, nx] = True
                    stack.append((ny, nx))
        if len(pixels) > len(best_pixels):
            best_pixels = pixels

    component = np.zeros(mask.shape, dtype=bool)
    if best_pixels:
        ys, xs = zip(*best_pixels)
        component[np.array(ys), np.array(xs)] = True
    return component


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
