"""Vision target detection service for behavior-tree orchestration."""
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_interfaces.srv import DetectGraspTarget
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from robot_vision_grasp.grasp_geometry import _get_axis_angle, get_ik_pitch, gripper_close_open_ratio
from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO


DEFAULT_CLASS_NAMES = ['bottle', 'pen', 'paper', 'charger', 'usb', 'yan']

LEFT_BUCKET_CLASSES = {'paper', 'yan', 'bottle'}
RIGHT_BUCKET_CLASSES = {'usb', 'charger', 'pen'}
TARGET_OFFSETS_CM = {
    'bottle': (0.0, 0.0, -6.0),
    'charger': (1.0, 0.0, -4.5),
    'paper': (1.0, 0.0, -3.5),
}


class VisionGraspNode(Node):
    """Detect segmented objects and return grasp-ready target metadata."""

    def __init__(self):
        super().__init__('vision_grasp_node')

        callback_group = ReentrantCallbackGroup()
        self._load_config()

        self._state_lock = threading.Lock()
        self._vision_lock = threading.Lock()
        self._latest_color = None
        self._depth = None
        self._depth_encoding = 'passthrough'
        self._intrinsics = None

        self._bridge = CvBridge()
        self._model = self._load_model()
        self._last_tf_warning = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        for msg_type, topic, callback in (
            (Image, self._color_topic, self._color_cb),
            (Image, self._depth_topic, self._depth_cb),
            (CameraInfo, self._depth_info_topic, self._info_cb),
        ):
            self.create_subscription(msg_type, topic, callback, 10, callback_group=callback_group)

        self._detect_srv = self.create_service(
            DetectGraspTarget,
            self._detect_service,
            self._detect_cb,
            callback_group=callback_group,
        )
        self.get_logger().info(
            'Vision grasp detection ready: '
            f'service={self._detect_service}, '
            f'color_topic={self._color_topic}, '
            f'depth_topic={self._depth_topic}, '
            f'depth_info_topic={self._depth_info_topic}, '
            f'tf={self._base_frame}->{self._camera_frame}'
        )

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _load_config(self):
        self._model_path = str(self._param('model_path', ''))
        self._model_device = self._param('model_device', 'CPU')
        self._class_names = list(self._param('class_names', DEFAULT_CLASS_NAMES))
        self._confidence_threshold = float(self._param('confidence_threshold', 0.55))
        self._iou_threshold = float(self._param('iou_threshold', 0.65))
        self._mask_threshold = float(self._param('mask_threshold', 0.5))
        self._color_topic = self._param('color_topic', '/camera/camera/color/image_raw')
        self._depth_topic = self._param('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self._depth_info_topic = self._param('depth_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self._base_frame = self._param('base_frame', 'arm_link0')
        self._camera_frame = self._param('camera_frame', 'camera_optical_link')
        self._detect_service = self._param('detect_service', 'vision_grasp/detect')
        self._target_y_scale = float(self._param('target_y_scale', 1.3))

    def _load_model(self):
        model_path = Path(self._model_path) if self._model_path else (
            Path(__file__).parent / 'models' / 'best.xml'
        )
        try:
            return YOLOSegOpenVINO(
                model_path,
                device=self._model_device,
                names={index: name for index, name in enumerate(self._class_names)},
                conf_thres=self._confidence_threshold,
                iou_thres=self._iou_threshold,
            )
        except Exception as exc:
            self.get_logger().error(f'OpenVINO model load failed: {exc}')
            return None

    def _info_cb(self, msg):
        with self._state_lock:
            self._intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def _depth_cb(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'Depth convert failed: {exc}')
            return
        with self._state_lock:
            self._depth = depth
            self._depth_encoding = msg.encoding

    def _color_cb(self, msg):
        """Cache latest color frame for the detection loop."""
        try:
            color = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Color convert failed: {exc}')
            return
        with self._state_lock:
            self._latest_color = color

    def _detect_cb(self, request, response):
        """单次识别服务：只返回目标，不执行抓取动作。"""
        del request
        if self._model is None:
            response.success = False
            response.message = 'model not loaded'
            return response

        with self._state_lock:
            color = None if self._latest_color is None else self._latest_color.copy()
            depth = None if self._depth is None else self._depth.copy()
            depth_encoding = self._depth_encoding
            intrinsics = self._intrinsics

        if color is None or depth is None or intrinsics is None:
            response.success = False
            response.message = 'waiting for color, depth and camera info'
            return response

        try:
            target = self._detect_target(color, depth, depth_encoding, intrinsics)
        except Exception as exc:
            response.success = False
            response.message = f'detect target failed: {exc}'
            return response

        if target is None:
            response.success = False
            response.message = 'no grasp target detected'
            return response

        target_cm = target['target_cm']
        response.success = True
        response.message = f"detected {target['class_name']}"
        response.class_name = str(target['class_name'])
        response.confidence = float(target.get('confidence', 0.0))
        response.target_x_cm = float(target_cm[0])
        response.target_y_cm = float(target_cm[1])
        response.target_z_cm = float(target_cm[2])
        response.end_angle = float(target['best_angle_deg'])
        response.gripper_rotation = float(target['gripper_rotation_deg'])
        response.gripper_close_ratio = float(target['gripper_close_ratio'])
        response.bucket_id = int(target['bucket_id'])
        return response

    # ── 检测 ──────────────────────────────────────────

    def _detect_target(self, color, depth, depth_encoding, intrinsics):
        """单帧检测，返回 target dict 或 None。"""
        if not self._vision_lock.acquire(blocking=False):
            return None
        try:
            targets = self._model.detect_targets(
                color, depth, intrinsics, encoding=depth_encoding,
                conf_thres=self._confidence_threshold, mask_thres=self._mask_threshold,
            )
            if not targets:
                return None

            tf_target = self._camera_target_to_arm_target(targets[0])
            if tf_target is None:
                return None
            return tf_target
        finally:
            self._vision_lock.release()

    def _camera_target_to_arm_target(self, target):
        transform = self._lookup_tf_camera_transform()
        if transform is None:
            return None
        rotation, translation, camera_pitch_deg = transform
        class_name = target['class_name']
        points_cam = target.get('points_cam')
        object_arm_m = translation + rotation @ target['point_cam_m']
        best_angle_deg = get_ik_pitch(
            class_name, points_cam, camera_pitch_deg, target.get('mask'), target.get('depth_m'),
        )[0]
        return {
            'class_name': class_name,
            'confidence': float(target.get('confidence', 0.0)),
            'target_cm': self._target_cm_for_object(object_arm_m, class_name),
            'best_angle_deg': best_angle_deg,
            'gripper_rotation_deg': 0.0 if class_name == 'paper' else _get_axis_angle(points_cam, camera_pitch_deg),
            'gripper_close_ratio': gripper_close_open_ratio(class_name),
            'bucket_id': self._bucket_preset_for_class(class_name),
        }

    def _lookup_tf_camera_transform(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._camera_frame, Time(), timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._warn_tf_failure(exc)
            return None

        q = transform.transform.rotation
        rotation = self._rotation_from_quaternion(q.x, q.y, q.z, q.w)
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=float)
        forward = rotation[:, 2]
        camera_pitch_deg = math.degrees(math.atan2(forward[2], math.hypot(forward[0], forward[1])))
        return rotation, translation, camera_pitch_deg

    def _warn_tf_failure(self, exc):
        now = time.monotonic()
        if now - self._last_tf_warning < 5.0:
            return
        self._last_tf_warning = now
        self.get_logger().warning(f'Camera TF unavailable: {exc}')

    @staticmethod
    def _rotation_from_quaternion(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-12:
            return np.identity(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ])

    def _target_cm_for_object(self, object_arm_m, class_name):
        target_cm = (
            object_arm_m * 100.0
            + np.array(
                TARGET_OFFSETS_CM.get(class_name, (0.0, 0.0, -3.0)),
                dtype=float,
            )
        )
        target_cm[1] *= getattr(self, '_target_y_scale', 1.3)
        return target_cm

    @staticmethod
    def _bucket_preset_for_class(class_name):
        if class_name in LEFT_BUCKET_CLASSES:
            return 1
        return 2


def main(args=None):
    rclpy.init(args=args)
    node = VisionGraspNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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
