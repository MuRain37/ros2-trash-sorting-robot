"""OpenVINO dump-bin detection service for dump-trash behavior."""

import math
import threading
from pathlib import Path

import cv2
import numpy as np
import openvino as ov
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.srv import DetectDumpBucket
from sensor_msgs.msg import Image


DEFAULT_COLOR_TOPIC = '/camera/real_camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/real_camera/aligned_depth_to_color/image_raw'
DEFAULT_SERVICE_NAME = 'detect_dump_bucket'
DEFAULT_CLASS_NAME = 'bin'
DEFAULT_CONFIDENCE = 0.5
DEFAULT_IOU = 0.4
DEFAULT_DEPTH_SCALE = 0.001
DEFAULT_MODEL_DEVICE = 'CPU'


class DumpTaskNode(Node):
    """Detect the external trash bin and report image offset plus camera depth."""

    def __init__(self):
        super().__init__('dump_task')

        callback_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_color = None
        self._latest_depth = None
        self._depth_encoding = 'passthrough'

        self._model_path = str(self._param('model_path', ''))
        self._color_topic = str(self._param('color_topic', DEFAULT_COLOR_TOPIC))
        self._depth_topic = str(self._param('depth_topic', DEFAULT_DEPTH_TOPIC))
        self._service_name = str(self._param('service_name', DEFAULT_SERVICE_NAME))
        self._class_name = str(self._param('class_name', DEFAULT_CLASS_NAME))
        self._confidence_threshold = float(
            self._param('confidence_threshold', DEFAULT_CONFIDENCE)
        )
        self._iou_threshold = float(self._param('iou_threshold', DEFAULT_IOU))
        self._depth_scale = float(self._param('depth_scale', DEFAULT_DEPTH_SCALE))
        self._model_device = str(self._param('model_device', DEFAULT_MODEL_DEVICE))

        self._model = None
        self._input_name = ''
        self._output_name = ''
        self._input_height = 640
        self._input_width = 640
        self._load_model()

        self.create_subscription(
            Image,
            self._color_topic,
            self._on_color_image,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_service(
            DetectDumpBucket,
            self._service_name,
            self._on_detect_request,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'Dump bin OpenVINO detector ready: '
            f'service={self._service_name}, '
            f'color_topic={self._color_topic}, '
            f'depth_topic={self._depth_topic}, '
            f'model={self._model_path or "trash_bin.xml"}, '
            f'device={self._model_device}, '
            f'class={self._class_name}, '
            f'confidence={self._confidence_threshold:.2f}'
        )

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _load_model(self):
        model_path = Path(self._model_path) if self._model_path else (
            Path(__file__).parent / 'models' / 'trash_bin.xml'
        )
        try:
            core = ov.Core()
            model = core.read_model(str(model_path))
            compiled = core.compile_model(model, self._model_device)
            self._model = compiled
            self._input_name = model.input(0).get_any_name()
            self._output_name = model.output(0).get_any_name()
            shape = model.input(0).shape
            self._input_height = int(shape[2])
            self._input_width = int(shape[3])
        except Exception as exc:
            self.get_logger().error(f'Dump bin OpenVINO model load failed: {exc}')
            self._model = None

    def _on_color_image(self, msg):
        try:
            color = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Dump bin color convert failed: {exc}')
            return
        with self._lock:
            self._latest_color = color

    def _on_depth_image(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Dump bin depth convert failed: {exc}')
            return
        with self._lock:
            self._latest_depth = depth
            self._depth_encoding = msg.encoding

    def _on_detect_request(self, request, response):
        del request
        if self._model is None:
            fill_empty_response(response, 'model not loaded')
            return response

        with self._lock:
            color = None if self._latest_color is None else self._latest_color.copy()
            depth = None if self._latest_depth is None else self._latest_depth.copy()
            depth_encoding = self._depth_encoding

        if color is None:
            fill_empty_response(response, f'waiting for color({self._color_topic}) image')
            return response

        try:
            detection = self._detect_bin(color, depth, depth_encoding)
        except Exception as exc:
            fill_empty_response(response, f'detect bin failed: {exc}')
            return response

        if detection is None:
            fill_empty_response(
                response,
                f'no {self._class_name} detection above '
                f'{self._confidence_threshold:.2f}',
            )
            return response

        response.success = True
        response.detected = True
        response.mask_pixels = int(detection['area_px'])
        response.mean_depth_m = float(detection['center_depth_m'])
        response.center_x_px = float(detection['center_x_px'])
        response.center_y_px = float(detection['center_y_px'])
        response.image_center_x_px = float(detection['image_center_x_px'])
        response.image_center_y_px = float(detection['image_center_y_px'])
        response.offset_x_px = float(detection['offset_x_px'])
        response.offset_y_px = float(detection['offset_y_px'])
        response.offset_x_ratio = float(detection['offset_x_ratio'])
        response.offset_y_ratio = float(detection['offset_y_ratio'])
        response.signal = 1
        depth_text = (
            'nan'
            if math.isnan(response.mean_depth_m)
            else f'{response.mean_depth_m:.3f}'
        )
        response.message = (
            f"detected {detection['class_name']}; "
            f"confidence={detection['confidence']:.3f}; "
            f"bbox=({detection['x1']:.1f},{detection['y1']:.1f},"
            f"{detection['x2']:.1f},{detection['y2']:.1f}); "
            f'center_depth_m={depth_text}; '
            f'offset_x_ratio={response.offset_x_ratio:.4f}'
        )
        return response

    def _detect_bin(self, color, depth, depth_encoding):
        detections = self._infer_boxes(color)
        if not detections:
            return None

        best = None
        for xyxy, confidence in detections:
            candidate = self._detection_from_box(
                color,
                depth,
                depth_encoding,
                xyxy,
                self._class_name,
                confidence,
            )
            if best is None or candidate['confidence'] > best['confidence']:
                best = candidate
        return best

    def _infer_boxes(self, color):
        blob, scale, pad_w, pad_h = self._preprocess(color)
        request = self._model.create_infer_request()
        raw = request.infer({self._input_name: blob})[self._output_name]
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.shape[0] == 5:
            pred = pred.T
        if pred.ndim != 2 or pred.shape[1] < 5:
            raise ValueError(f'unexpected OpenVINO output shape: {raw.shape}')

        boxes = []
        scores = []
        for row in pred:
            score = float(row[4])
            if score < self._confidence_threshold:
                continue
            xc, yc, bw, bh = [float(value) for value in row[:4]]
            x1 = (xc - bw * 0.5 - pad_w) / scale
            y1 = (yc - bh * 0.5 - pad_h) / scale
            x2 = (xc + bw * 0.5 - pad_w) / scale
            y2 = (yc + bh * 0.5 - pad_h) / scale
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2, y2])
            scores.append(score)

        if not boxes:
            return []

        boxes_arr = np.asarray(boxes, dtype=np.float32)
        scores_arr = np.asarray(scores, dtype=np.float32)
        indices = self._nms(boxes_arr, scores_arr)
        return [(boxes_arr[index], float(scores_arr[index])) for index in indices]

    def _preprocess(self, color):
        height, width = color.shape[:2]
        scale = min(self._input_height / height, self._input_width / width)
        new_h = int(round(height * scale))
        new_w = int(round(width * scale))
        pad_h = (self._input_height - new_h) // 2
        pad_w = (self._input_width - new_w) // 2

        resized = cv2.resize(color, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full(
            (self._input_height, self._input_width, 3),
            114,
            dtype=np.uint8,
        )
        canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
        blob = np.transpose(canvas.astype(np.float32) / 255.0, (2, 0, 1))
        return blob[np.newaxis, :, :, :], scale, pad_w, pad_h

    def _nms(self, boxes, scores):
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            index = order[0]
            keep.append(index)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[index], x1[order[1:]])
            yy1 = np.maximum(y1[index], y1[order[1:]])
            xx2 = np.minimum(x2[index], x2[order[1:]])
            yy2 = np.minimum(y2[index], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[index] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-6)
            order = order[1:][iou <= self._iou_threshold]
        return keep

    def _detection_from_box(self, color, depth, depth_encoding, xyxy, class_name, confidence):
        height, width = color.shape[:2]
        x1, y1, x2, y2 = np.clip(
            xyxy,
            [0.0, 0.0, 0.0, 0.0],
            [width - 1.0, height - 1.0, width - 1.0, height - 1.0],
        )
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        image_center_x = (float(width) - 1.0) * 0.5
        image_center_y = (float(height) - 1.0) * 0.5
        offset_x = center_x - image_center_x
        offset_y = center_y - image_center_y
        area_px = max(0.0, (x2 - x1) * (y2 - y1))
        center_depth = self._center_depth_m(depth, depth_encoding, center_x, center_y)
        return {
            'class_name': class_name,
            'confidence': float(confidence),
            'x1': float(x1),
            'y1': float(y1),
            'x2': float(x2),
            'y2': float(y2),
            'center_x_px': float(center_x),
            'center_y_px': float(center_y),
            'image_center_x_px': image_center_x,
            'image_center_y_px': image_center_y,
            'offset_x_px': float(offset_x),
            'offset_y_px': float(offset_y),
            'offset_x_ratio': float(offset_x / max(1.0, image_center_x)),
            'offset_y_ratio': float(offset_y / max(1.0, image_center_y)),
            'area_px': int(area_px),
            'center_depth_m': float(center_depth),
        }

    def _center_depth_m(self, depth, depth_encoding, center_x, center_y):
        if depth is None:
            return math.nan
        depth_m = image_to_depth_meters(depth, depth_encoding, self._depth_scale)
        height, width = depth_m.shape[:2]
        cx = max(0, min(width - 1, int(round(center_x))))
        cy = max(0, min(height - 1, int(round(center_y))))
        value = float(depth_m[cy, cx])
        if not math.isfinite(value) or value <= 0.0:
            return math.nan
        return value


def image_to_depth_meters(depth, encoding, depth_scale):
    encoding = str(encoding).lower()
    if encoding in ('16uc1', 'mono16', 'passthrough'):
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * float(depth_scale)
    return depth.astype(np.float32)


def fill_empty_response(response, message):
    response.success = False
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


def main(args=None):
    rclpy.init(args=args)
    node = DumpTaskNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
