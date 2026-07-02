"""Vision object detection to MoveIt arm target — start/stop service mode."""
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_hardware_bridge.arm_bridge import DEFAULT_GRIPPER_SERVICE
from robot_interfaces.action import ArmPreset, ArmTarget
from robot_interfaces.srv import SetGripper
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_vision_grasp.grasp_geometry import _get_axis_angle, get_ik_pitch, gripper_close_open_ratio
from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO


DEFAULT_CLASS_NAMES = ['bottle', 'pen', 'paper', 'charger', 'usb', 'yan']

ACTION_TIMEOUT_SEC = 30.0
ARM_CENTER_PRESET = ArmPreset.Goal.ARM_CENTER
LEFT_BUCKET_CLASSES = {'paper', 'yan', 'bottle'}
RIGHT_BUCKET_CLASSES = {'usb', 'charger', 'pen'}
GRIPPER_WAIT_SEC = 1.4
GRIPPER_DURATION_MS = 500.0
TARGET_OFFSETS_CM = {
    'bottle': (0.0, 0.0, -9.0),
    'charger': (1.5, 0.0, -4.0),
    'paper': (1.5, 0.0, -4.0),
}


class VisionGraspNode(Node):
    """Detect segmented objects and execute grasp motions — start/stop via service."""

    def __init__(self):
        super().__init__('vision_grasp_node')

        callback_group = ReentrantCallbackGroup()
        self._load_config()

        self._state_lock = threading.Lock()
        self._vision_lock = threading.Lock()
        self._sequence_lock = threading.Lock()
        self._latest_color = None
        self._depth = None
        self._depth_encoding = 'passthrough'
        self._intrinsics = None

        self._bridge = CvBridge()
        self._model = self._load_model()
        self._last_tf_warning = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._arm_action = ActionClient(self, ArmTarget, '/arm_target', callback_group=callback_group)
        self._arm_preset_action = ActionClient(self, ArmPreset, '/arm_preset', callback_group=callback_group)
        self._gripper_client = self.create_client(SetGripper, self._gripper_service, callback_group=callback_group)

        for msg_type, topic, callback in (
            (Image, self._color_topic, self._color_cb),
            (Image, self._depth_topic, self._depth_cb),
            (CameraInfo, self._depth_info_topic, self._info_cb),
        ):
            self.create_subscription(msg_type, topic, callback, 10, callback_group=callback_group)

        # 检测循环控制
        self._running = False
        self._grasp_thread = None

        self._start_srv = self.create_service(
            Trigger, 'vision_grasp/start', self._start_cb, callback_group=callback_group,
        )
        self._stop_srv = self.create_service(
            Trigger, 'vision_grasp/stop', self._stop_cb, callback_group=callback_group,
        )
        self.get_logger().info('Vision grasp node ready (start/stop service mode)')

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
        self._arm_goal_duration = float(self._param('arm_goal_duration', 2.0))
        self._gripper_service = self._param('gripper_service', DEFAULT_GRIPPER_SERVICE)

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
            self._latest_color = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Color convert failed: {exc}')

    # ── start / stop 服务 ──────────────────────────────

    def _start_cb(self, request, response):
        """启动持续检测抓取循环。"""
        if self._model is None:
            response.success = False
            response.message = 'model not loaded'
            return response
        if self._running:
            response.success = True
            response.message = 'already running'
            return response

        self._running = True
        self._grasp_thread = threading.Thread(target=self._grasp_loop, daemon=True)
        self._grasp_thread.start()
        self.get_logger().info('Grasp loop started')
        response.success = True
        response.message = 'grasp loop started'
        return response

    def _stop_cb(self, request, response):
        """停止检测抓取循环。"""
        if not self._running:
            response.success = True
            response.message = 'already stopped'
            return response

        self._running = False
        if self._grasp_thread is not None:
            self._grasp_thread.join(timeout=5.0)
            self._grasp_thread = None
        self.get_logger().info('Grasp loop stopped')
        response.success = True
        response.message = 'grasp loop stopped'
        return response

    # ── 持续检测循环 ──────────────────────────────────

    def _grasp_loop(self):
        """循环检测目标，发现后执行抓取序列。每帧之间间隔 1 秒。"""
        while self._running and rclpy.ok():
            with self._state_lock:
                color = self._latest_color
                depth = None if self._depth is None else self._depth.copy()
                depth_encoding = self._depth_encoding
                intrinsics = self._intrinsics

            if color is not None and depth is not None and intrinsics is not None:
                try:
                    target = self._detect_target(color, depth, depth_encoding, intrinsics)
                    if target is not None:
                        self.get_logger().info(
                            f"Grasping {target['class_name']}: "
                            f"({target['target_cm'][0]:.1f}, {target['target_cm'][1]:.1f}, "
                            f"{target['target_cm'][2]:.1f}) cm"
                        )
                        self._execute_grasp_sequence(target)
                except Exception as exc:
                    self.get_logger().warning(f'Grasp loop iteration failed: {exc}')

            # 每秒检测一帧，避免 CPU 满载
            for _ in range(10):
                if not self._running or not rclpy.ok():
                    return
                time.sleep(0.1)

    # ── 检测与抓取 ────────────────────────────────────

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
            self.get_logger().info(
                f"{tf_target['class_name']}: "
                f"target=({tf_target['target_cm'][0]:.1f}, "
                f"{tf_target['target_cm'][1]:.1f}, "
                f"{tf_target['target_cm'][2]:.1f})cm "
                f"best_angle={tf_target['best_angle_deg']:.1f} deg"
            )
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
            'target_cm': self._target_cm_for_object(object_arm_m, class_name),
            'best_angle_deg': best_angle_deg,
            'gripper_rotation_deg': 0.0 if class_name == 'paper' else _get_axis_angle(points_cam, camera_pitch_deg),
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
        return object_arm_m * 100.0 + np.array(TARGET_OFFSETS_CM.get(class_name, (0.0, 0.0, -3.5)), dtype=float)

    def _execute_grasp_sequence(self, target):
        """完整抓取周期：开夹爪 → 移动目标 → 夹紧 → 丢桶 → 归中。"""
        if not self._sequence_lock.acquire(blocking=False):
            return
        try:
            class_name = target['class_name']
            self._call_gripper(1.0, GRIPPER_DURATION_MS)
            if not self._send_arm_target(target):
                self._call_gripper(0.0, GRIPPER_DURATION_MS)
                return
            if not self._call_gripper(gripper_close_open_ratio(class_name), GRIPPER_DURATION_MS):
                return
            time.sleep(GRIPPER_WAIT_SEC)
            bucket = self._bucket_preset_for_class(class_name)
            if bucket is not None and self._send_arm_preset(bucket):
                self._call_gripper(1.0, GRIPPER_DURATION_MS)
                time.sleep(GRIPPER_WAIT_SEC)
            self._send_arm_preset(ARM_CENTER_PRESET)
        finally:
            self._sequence_lock.release()

    @staticmethod
    def _bucket_preset_for_class(class_name):
        if class_name in LEFT_BUCKET_CLASSES:
            return ArmPreset.Goal.LEFT_BUCKET
        return ArmPreset.Goal.RIGHT_BUCKET if class_name in RIGHT_BUCKET_CLASSES else None

    def _send_arm_target(self, target):
        goal = ArmTarget.Goal()
        goal.x, goal.y, goal.z = map(float, target['target_cm'])
        goal.duration = self._arm_goal_duration
        goal.end_angle = float(target['best_angle_deg'])
        goal.gripper_rotation = float(target['gripper_rotation_deg'])
        return self._send_goal(self._arm_action, goal, ACTION_TIMEOUT_SEC, 'Arm target')

    def _send_arm_preset(self, action_type):
        goal = ArmPreset.Goal()
        goal.action_type = int(action_type)
        return self._send_goal(self._arm_preset_action, goal, ACTION_TIMEOUT_SEC, 'Arm preset')

    def _send_goal(self, client, goal, timeout_sec, label):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning(f'{label} server not ready')
            return False

        goal_handle = self._wait_for_future(client.send_goal_async(goal), 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(f'{label} rejected')
            return False

        wrapped = self._wait_for_future(goal_handle.get_result_async(), timeout_sec)
        if wrapped is None:
            self.get_logger().warning(f'{label} timed out')
            return False

        result = wrapped.result
        if not result.success:
            self.get_logger().warning(f'{label} failed')
            return False
        return True

    def _call_gripper(self, open_ratio, duration_ms):
        if not self._gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(f'Gripper service not ready: {self._gripper_service}')
            return False

        request = SetGripper.Request()
        request.open_ratio = float(max(0.0, min(1.0, open_ratio)))
        request.duration_ms = float(duration_ms)
        response = self._wait_for_future(self._gripper_client.call_async(request), 2.0)
        if response is None:
            self.get_logger().warning('Gripper command timed out')
            return False
        if not response.success:
            self.get_logger().warning(f'Gripper command failed: {response.message}')
            return False
        return True

    def _wait_for_future(self, future, timeout_sec):
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout_sec):
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f'Async ROS call failed: {exc}')
            return None


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
