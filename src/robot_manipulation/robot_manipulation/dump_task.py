"""垃圾桶识别任务服务：只检测并返回信号，不执行机械臂动作."""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_SERVICE_NAME = 'detect_dump_bucket'

DETECTED_MESSAGE = 'dump bucket detected within depth threshold'
NOT_DETECTED_MESSAGE = 'dump bucket not detected within depth threshold'


class DumpTaskNode(Node):
    """订阅 RGB/Depth 图像，并通过服务返回垃圾桶是否在 20cm 内."""

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
            Trigger,
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
            response.success = False
            response.message = 'waiting for color and depth images'
            return response

        try:
            detected, mask_pixels, mean_depth_m = self._detect_bucket(
                color_msg,
                depth_msg,
            )
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if detected:
            response.success = True
            response.message = (
                f'{DETECTED_MESSAGE}; signal=1; '
                f'mask_pixels={mask_pixels}; '
                f'mean_depth_m={mean_depth_m:.3f}'
            )
            return response

        response.success = False
        depth_text = 'nan' if math.isnan(mean_depth_m) else f'{mean_depth_m:.3f}'
        response.message = (
            f'{NOT_DETECTED_MESSAGE}; signal=2; '
            f'mask_pixels={mask_pixels}; mean_depth_m={depth_text}'
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
        mask_pixels = int(np.count_nonzero(mask))

        mask_min_pixels = int(self.get_parameter('mask_min_pixels').value)
        if mask_pixels <= mask_min_pixels:
            return False, mask_pixels, math.nan

        masked_depth = depth_m[mask]
        valid_depth = masked_depth[np.isfinite(masked_depth) & (masked_depth > 0.0)]
        if valid_depth.size == 0:
            return False, mask_pixels, math.nan

        mean_depth_m = float(np.mean(valid_depth))
        depth_threshold_m = float(self.get_parameter('depth_threshold_m').value)
        return mean_depth_m <= depth_threshold_m, mask_pixels, mean_depth_m


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
