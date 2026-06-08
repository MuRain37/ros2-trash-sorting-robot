"""Vision object detection to MoveIt arm target bridge."""

import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_arm.arm_link_lengths import install_urdf_link_length_loader
from robot_interfaces.action import ArmTarget
from robot_interfaces.srv import SetGripper
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from robot_vision_grasp.grasp_geometry import get_gripper_yaw
from robot_vision_grasp.grasp_geometry import get_ik_pitch
from robot_vision_grasp.grasp_geometry import gripper_close_open_ratio
from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO


MOVEIT_JOINTS = [
    'arm_base_joint',
    'arm_shoulder_joint',
    'arm_elbow_joint',
    'arm_wrist_joint',
    'arm_tool_rotate_joint',
    'virtual_tool_roll_joint',
    'virtual_tool_pitch_joint',
    'virtual_tool_yaw_joint',
]
FALLBACK_LINK_LENGTHS_CM = (3.38648, 12.9416, 12.9444, 11.38648)


class VisionGraspNode(Node):
    """Detect segmented objects and execute grasp motions through MoveIt."""

    def __init__(self):
        super().__init__('vision_grasp_node')

        callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._load_parameters()

        self._state_lock = threading.Lock()
        self._vision_lock = threading.Lock()
        self._sequence_lock = threading.Lock()
        self._depth = None
        self._camera_info_ready = False
        self._intrinsics = (383.0, 383.0, 320.0, 240.0)
        self._joint_positions = {}
        self._arm_link_lengths_cm = FALLBACK_LINK_LENGTHS_CM

        self._bridge = CvBridge()
        self._model = self._load_model()
        self._busy = False
        self._last_target_time = None
        self._last_tf_warning = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._arm_action = ActionClient(
            self,
            ArmTarget,
            self._arm_action_name,
            callback_group=callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            self._gripper_service_name,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            self._color_topic,
            self._color_cb,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_cb,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            CameraInfo,
            self._depth_info_topic,
            self._info_cb,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_cb,
            10,
            callback_group=callback_group,
        )
        self._link_length_sub = install_urdf_link_length_loader(
            self,
            self._set_link_lengths,
        )

        self.get_logger().info(
            'Vision grasp node ready: '
            f'color={self._color_topic}, depth={self._depth_topic}, '
            f'action={self._arm_action_name}, '
            f'gripper={self._gripper_service_name}'
        )

    def _declare_parameters(self):
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_device', 'CPU')
        self.declare_parameter(
            'class_names',
            ['bottle', 'pen', 'cup', 'can', 'paper'],
        )
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('iou_threshold', 0.65)
        self.declare_parameter('mask_threshold', 0.5)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'depth_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('use_tf_camera_pose', False)
        self.declare_parameter('fallback_manual_fk', True)
        self.declare_parameter('arm_origin_m', [0.10, 0.0, 0.025])
        self.declare_parameter(
            'manual_camera_offset_m',
            [-0.033, 0.089, 0.020],
        )
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('depth_reg_x', 0)
        self.declare_parameter('depth_reg_y', 0)
        self.declare_parameter('target_timeout', 2.0)
        self.declare_parameter('target_forward_offset_cm', 0.0)
        self.declare_parameter('target_right_offset_cm', 0.0)
        self.declare_parameter('target_up_offset_cm', 2.0)
        self.declare_parameter('invert_target_lateral', True)
        self.declare_parameter('erode_iterations', 2)
        self.declare_parameter('arm_action_name', '/arm_target')
        self.declare_parameter('gripper_service_name', '/set_gripper')
        self.declare_parameter('arm_goal_duration', 3.0)
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('retry_auto_pitch_on_failure', True)
        self.declare_parameter('gripper_duration_ms', 1000.0)
        self.declare_parameter('gripper_open_wait_sec', 1.0)
        self.declare_parameter('gripper_close_wait_sec', 1.0)
        self.declare_parameter('open_gripper_before_move', True)
        self.declare_parameter('enable_tool_yaw', True)
        self.declare_parameter('return_home', True)
        self.declare_parameter(
            'home_joint_positions',
            [
                0.0,
                -1.2566370614359172,
                2.0943951023931953,
                1.4660765716752369,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.declare_parameter('home_duration', 2.0)

    def _load_parameters(self):
        self._model_path = str(self.get_parameter('model_path').value)
        self._model_device = self.get_parameter('model_device').value
        self._class_names = list(self.get_parameter('class_names').value)
        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._iou_threshold = float(
            self.get_parameter('iou_threshold').value
        )
        self._mask_threshold = float(
            self.get_parameter('mask_threshold').value
        )
        self._color_topic = self.get_parameter('color_topic').value
        self._depth_topic = self.get_parameter('depth_topic').value
        self._depth_info_topic = self.get_parameter('depth_info_topic').value
        self._joint_states_topic = self.get_parameter(
            'joint_states_topic'
        ).value
        self._base_frame = self.get_parameter('base_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._use_tf_camera_pose = bool(
            self.get_parameter('use_tf_camera_pose').value
        )
        self._fallback_manual_fk = bool(
            self.get_parameter('fallback_manual_fk').value
        )
        self._arm_origin_m = np.array(
            self.get_parameter('arm_origin_m').value,
            dtype=float,
        )
        self._manual_camera_offset_m = np.array(
            self.get_parameter('manual_camera_offset_m').value,
            dtype=float,
        )
        self._depth_scale = float(self.get_parameter('depth_scale').value)
        self._min_depth_m = float(self.get_parameter('min_depth_m').value)
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        self._reg_x = int(self.get_parameter('depth_reg_x').value)
        self._reg_y = int(self.get_parameter('depth_reg_y').value)
        self._target_timeout = float(
            self.get_parameter('target_timeout').value
        )
        self._target_offsets_cm = np.array([
            float(self.get_parameter('target_forward_offset_cm').value),
            float(self.get_parameter('target_right_offset_cm').value),
            float(self.get_parameter('target_up_offset_cm').value),
        ])
        self._invert_target_lateral = bool(
            self.get_parameter('invert_target_lateral').value
        )
        self._erode_iterations = int(
            self.get_parameter('erode_iterations').value
        )
        self._arm_action_name = self.get_parameter('arm_action_name').value
        self._gripper_service_name = self.get_parameter(
            'gripper_service_name'
        ).value
        self._arm_goal_duration = float(
            self.get_parameter('arm_goal_duration').value
        )
        self._action_timeout = float(
            self.get_parameter('action_timeout').value
        )
        self._retry_auto_pitch_on_failure = bool(
            self.get_parameter('retry_auto_pitch_on_failure').value
        )
        self._gripper_duration_ms = float(
            self.get_parameter('gripper_duration_ms').value
        )
        self._gripper_open_wait_sec = float(
            self.get_parameter('gripper_open_wait_sec').value
        )
        self._gripper_close_wait_sec = float(
            self.get_parameter('gripper_close_wait_sec').value
        )
        self._open_gripper_before_move = bool(
            self.get_parameter('open_gripper_before_move').value
        )
        self._enable_tool_yaw = bool(
            self.get_parameter('enable_tool_yaw').value
        )
        self._return_home = bool(self.get_parameter('return_home').value)
        self._home_joint_positions = [
            float(v) for v in self.get_parameter('home_joint_positions').value
        ]
        self._home_duration = float(self.get_parameter('home_duration').value)

    def _load_model(self):
        model_path = Path(self._model_path) if self._model_path else (
            Path(__file__).parent / 'models' / 'best.xml'
        )
        names = {index: name for index, name in enumerate(self._class_names)}
        try:
            model = YOLOSegOpenVINO(
                model_path,
                device=self._model_device,
                names=names,
                conf_thres=self._confidence_threshold,
                iou_thres=self._iou_threshold,
            )
        except Exception as exc:
            self.get_logger().error(f'OpenVINO model load failed: {exc}')
            return None

        self.get_logger().info(f'OpenVINO model loaded: {model_path}')
        return model

    def _set_link_lengths(self, lengths):
        with self._state_lock:
            self._arm_link_lengths_cm = tuple(float(v) for v in lengths)
        self.get_logger().info(
            'Loaded FK link lengths: '
            f'l1={lengths[0]:.2f}, l2={lengths[1]:.2f}, '
            f'l3={lengths[2]:.2f}, l4={lengths[3]:.2f} cm'
        )

    def _info_cb(self, msg):
        intrinsics = (
            float(msg.k[0]),
            float(msg.k[4]),
            float(msg.k[2]),
            float(msg.k[5]),
        )
        with self._state_lock:
            self._intrinsics = intrinsics
            self._camera_info_ready = True
        self.get_logger().info(
            f'Camera intrinsics: fx={intrinsics[0]:.2f}, '
            f'fy={intrinsics[1]:.2f}, cx={intrinsics[2]:.2f}, '
            f'cy={intrinsics[3]:.2f}',
            once=True,
        )

    def _depth_cb(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough',
            )
            depth = self._depth_to_meters(depth, msg.encoding)
            if depth.shape[0] >= 5 and depth.shape[1] >= 5:
                depth = cv2.medianBlur(depth, 5)
        except Exception as exc:
            self.get_logger().error(f'Depth convert failed: {exc}')
            return

        with self._state_lock:
            self._depth = depth

    def _depth_to_meters(self, depth, encoding):
        depth = np.asarray(depth)
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        if encoding in ('16UC1', 'mono16') or depth.dtype == np.uint16:
            return depth.astype(np.float32) * self._depth_scale
        return depth.astype(np.float32)

    def _joint_state_cb(self, msg):
        updates = {}
        for name, position in zip(msg.name, msg.position):
            if name in MOVEIT_JOINTS:
                updates[name] = float(position)
        if not updates:
            return
        with self._state_lock:
            self._joint_positions.update(updates)

    def _color_cb(self, msg):
        if self._model is None:
            return
        if not self._target_timeout_elapsed():
            return
        if not self._sequence_lock.acquire(blocking=False):
            return

        sequence_locked = True
        vision_locked = False
        try:
            if not self._vision_lock.acquire(blocking=False):
                return
            vision_locked = True

            with self._state_lock:
                depth = None if self._depth is None else self._depth.copy()
                camera_info_ready = self._camera_info_ready
                intrinsics = tuple(self._intrinsics)

            if depth is None or not camera_info_ready:
                return

            try:
                cv_img = self._bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding='bgr8',
                )
            except Exception as exc:
                self.get_logger().error(f'Color convert failed: {exc}')
                return

            try:
                target = self._select_target(cv_img, depth, intrinsics)
            except Exception as exc:
                self.get_logger().warning(f'Vision frame skipped: {exc}')
                return

            if target is None:
                return

            with self._state_lock:
                self._busy = True
                self._last_target_time = self.get_clock().now()
            thread = threading.Thread(
                target=self._run_grasp_sequence,
                args=(target,),
                daemon=True,
            )
            thread.start()
            sequence_locked = False
        finally:
            if vision_locked:
                self._vision_lock.release()
            if sequence_locked:
                with self._state_lock:
                    self._busy = False
                self._sequence_lock.release()

    def _target_timeout_elapsed(self):
        with self._state_lock:
            last_target_time = self._last_target_time
        if last_target_time is None:
            return True
        now = self.get_clock().now()
        elapsed_ns = (now - last_target_time).nanoseconds
        return elapsed_ns >= self._target_timeout * 1e9

    def _select_target(self, cv_img, depth, intrinsics):
        results = self._model(cv_img, verbose=False)
        candidates = []
        for result in results:
            if result.masks is None:
                continue
            for index, box in enumerate(result.boxes):
                candidate = self._candidate_from_box(
                    result,
                    index,
                    box,
                    depth,
                    intrinsics,
                )
                if candidate is not None:
                    candidates.append(candidate)

        if not candidates:
            return None

        candidates.sort(key=lambda item: item['confidence'], reverse=True)
        target = candidates[0]
        self.get_logger().info(
            f"Detect [{target['class_name']}] conf={target['confidence']:.2f} "
            f"base=({target['object_base_m'][0] * 100.0:.1f}, "
            f"{target['object_base_m'][1] * 100.0:.1f}, "
            f"{target['object_base_m'][2] * 100.0:.1f})cm "
            f"target=({target['target_cm'][0]:.1f}, "
            f"{target['target_cm'][1]:.1f}, {target['target_cm'][2]:.1f})cm "
            f"pitch={target['pitch_deg']:.1f} yaw={target['yaw_deg']:.1f}"
        )
        return target

    def _candidate_from_box(self, result, index, box, depth, intrinsics):
        try:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = result.names.get(cls_id, str(cls_id))
        except (IndexError, TypeError, ValueError, AttributeError):
            return None

        mask = self._mask_for_result(result, index, depth.shape)
        if mask is None or not np.any(mask):
            return None

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        if self._erode_iterations > 0:
            centroid_mask = cv2.erode(
                mask.astype(np.uint8),
                kernel,
                iterations=self._erode_iterations,
            ).astype(bool)
        else:
            centroid_mask = mask.astype(bool)

        if not np.any(centroid_mask):
            centroid_mask = mask.astype(bool)

        centroid = self._mask_centroid(centroid_mask, depth, intrinsics)
        if centroid is None:
            return None

        object_base_m = centroid['object_base_m']
        camera_pitch = centroid['camera_pitch_deg']
        points_cam = centroid['points_cam']
        pitch_deg, axis_angle, is_upright = get_ik_pitch(
            class_name,
            points_cam,
            camera_pitch,
            mask.astype(bool),
            depth,
        )
        yaw_deg = get_gripper_yaw(class_name, axis_angle, is_upright)
        target_cm = self._target_cm_for_object(
            object_base_m,
            class_name,
            pitch_deg,
            is_upright,
        )

        return {
            'class_name': class_name,
            'confidence': confidence,
            'object_base_m': object_base_m,
            'target_cm': target_cm,
            'pitch_deg': pitch_deg,
            'yaw_deg': yaw_deg,
            'is_upright': is_upright,
        }

    def _mask_for_result(self, result, index, depth_shape):
        masks = result.masks
        try:
            if getattr(masks, 'soft_data', None) is not None:
                mask = np.asarray(masks.soft_data[index], dtype=np.float32)
            else:
                mask = np.asarray(masks.data[index], dtype=np.float32)
        except (IndexError, TypeError, ValueError):
            return None

        height, width = depth_shape[:2]
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(
                mask,
                (width, height),
                interpolation=cv2.INTER_LINEAR,
            )
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask_bin = (mask > self._mask_threshold).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_bin = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, kernel)
        return mask_bin.astype(bool)

    def _mask_centroid(self, mask, depth, intrinsics):
        ys, xs = np.where(mask)
        if len(ys) == 0:
            return None

        height, width = depth.shape[:2]
        xs_depth = np.clip(xs + self._reg_x, 0, width - 1)
        ys_depth = np.clip(ys + self._reg_y, 0, height - 1)
        depths = depth[ys_depth, xs_depth]
        valid = (
            np.isfinite(depths)
            & (depths >= self._min_depth_m)
            & (depths <= self._max_depth_m)
        )
        if not np.any(valid):
            return None

        xs_valid = xs_depth[valid].astype(float)
        ys_valid = ys_depth[valid].astype(float)
        depths_valid = depths[valid].astype(float)
        fx, fy, cx, cy = intrinsics
        u = float(xs_valid.mean())
        v = float(ys_valid.mean())
        depth_m = float(depths_valid.mean())

        transform = self._camera_transform()
        if transform is None:
            return None
        rotation, translation, camera_pitch_deg = transform

        point_cam = np.array([
            (u - cx) * depth_m / fx,
            (v - cy) * depth_m / fy,
            depth_m,
        ])
        object_base_m = translation + rotation @ point_cam

        points_cam = np.column_stack([
            (xs_valid - cx) * depths_valid / fx,
            (ys_valid - cy) * depths_valid / fy,
            depths_valid,
        ])
        return {
            'object_base_m': object_base_m,
            'camera_pitch_deg': camera_pitch_deg,
            'points_cam': points_cam,
        }

    def _camera_transform(self):
        if self._use_tf_camera_pose:
            transform = self._lookup_tf_camera_transform()
            if transform is not None:
                self.get_logger().info('Using TF camera pose', once=True)
                return transform
        if self._fallback_manual_fk:
            self.get_logger().info('Using manual FK camera pose', once=True)
            return self._manual_fk_camera_transform()
        return None

    def _lookup_tf_camera_transform(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._camera_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._warn_tf_failure(exc)
            return None

        q = transform.transform.rotation
        rotation = self._rotation_from_quaternion(q.x, q.y, q.z, q.w)
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=float)
        forward = rotation[:, 2]
        camera_pitch_deg = math.degrees(
            math.atan2(forward[2], math.hypot(forward[0], forward[1]))
        )
        return rotation, translation, camera_pitch_deg

    def _warn_tf_failure(self, exc):
        now = time.monotonic()
        if now - self._last_tf_warning < 5.0:
            return
        self._last_tf_warning = now
        self.get_logger().warning(
            f'Camera TF unavailable ({self._base_frame} <- '
            f'{self._camera_frame}): {exc}. Trying manual FK fallback.'
        )

    def _manual_fk_camera_transform(self):
        with self._state_lock:
            positions = dict(self._joint_positions)
            lengths_cm = tuple(self._arm_link_lengths_cm)

        missing = [name for name in MOVEIT_JOINTS[:4] if name not in positions]
        if missing:
            return None

        theta1 = positions['arm_base_joint']
        theta2 = positions['arm_shoulder_joint']
        theta3 = positions['arm_elbow_joint']
        theta4 = positions['arm_wrist_joint']
        l1, l2, l3, _l4 = lengths_cm

        pitch4 = math.pi / 2.0 - (theta2 + theta3 + theta4)
        c1, s1 = math.cos(theta1), math.sin(theta1)
        cp, sp = math.cos(pitch4), math.sin(pitch4)
        pitch3 = math.pi / 2.0 - (theta2 + theta3)
        cp3, sp3 = math.cos(pitch3), math.sin(pitch3)

        joint3_cm = np.array([
            (l2 * math.sin(theta2) + l3 * cp3) * c1,
            (l2 * math.sin(theta2) + l3 * cp3) * s1,
            l1 + l2 * math.cos(theta2) + l3 * sp3,
        ])
        forward_dir = np.array([cp * c1, cp * s1, sp])
        up_dir = np.array([-sp * c1, -sp * s1, cp])
        right_dir = np.array([-s1, c1, 0.0])

        camera_pos_arm_m = joint3_cm / 100.0
        camera_pos_arm_m += (
            self._manual_camera_offset_m[0] * right_dir
            + self._manual_camera_offset_m[1] * forward_dir
            + self._manual_camera_offset_m[2] * up_dir
        )
        translation = self._arm_origin_m + camera_pos_arm_m

        rotation = np.column_stack([right_dir, -up_dir, forward_dir])
        camera_pitch_deg = math.degrees(math.atan2(sp, cp))
        return rotation, translation, camera_pitch_deg

    @staticmethod
    def _rotation_from_quaternion(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-12:
            return np.identity(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])

    def _target_cm_for_object(
        self,
        object_base_m,
        class_name,
        pitch_deg,
        is_upright,
    ):
        arm_relative_cm = (object_base_m - self._arm_origin_m) * 100.0
        target_cm = arm_relative_cm + self._target_offsets_cm
        target_cm[2] -= 2.0

        if class_name == 'paper':
            target_cm[0] -= (1.0 - pitch_deg / 90.0)
            target_cm[2] += 0.8 * pitch_deg / 90.0
        elif class_name in ('bottle', 'can', 'cup'):
            target_cm[2] -= 4.0
            if is_upright:
                target_cm[0] -= 5.0 * pitch_deg / 90.0
                target_cm[1] -= 3.0
                target_cm[2] -= 2.0
        elif class_name == 'pen':
            target_cm[2] += 2.0

        if self._invert_target_lateral:
            target_cm[1] = -target_cm[1]

        return target_cm

    def _run_grasp_sequence(self, target):
        try:
            class_name = target['class_name']
            if self._open_gripper_before_move:
                if self._call_gripper(1.0, self._gripper_duration_ms):
                    time.sleep(max(0.0, self._gripper_open_wait_sec))

            if not self._send_arm_target(target):
                return

            close_ratio = gripper_close_open_ratio(class_name)
            if self._call_gripper(close_ratio, self._gripper_duration_ms):
                time.sleep(max(0.0, self._gripper_close_wait_sec))

            if self._return_home:
                home_reached = self._send_home_via_moveit()
                if home_reached and self._home_duration > 0.0:
                    time.sleep(self._home_duration)
                elif not home_reached:
                    self.get_logger().warning(
                        'HOME target was not reached; keeping vision paused '
                        'until this grasp sequence exits'
                    )

            self.get_logger().info(f'Grasp sequence finished for {class_name}')
        except Exception as exc:
            self.get_logger().error(f'Grasp sequence failed: {exc}')
        finally:
            with self._state_lock:
                self._busy = False
                self._last_target_time = self.get_clock().now()
            self._sequence_lock.release()

    def _send_arm_target(self, target):
        if not self._arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not ready: {self._arm_action_name}'
            )
            return False

        pitch = float(target['pitch_deg'])
        if self._send_single_arm_target(target, pitch):
            return True

        if not self._retry_auto_pitch_on_failure:
            return False

        self.get_logger().warning(
            f'Arm target failed at pitch={self._pitch_text(pitch)}; '
            'retrying with MoveIt auto pitch'
        )
        return self._send_single_arm_target(target, -1.0)

    def _send_single_arm_target(self, target, pitch):
        goal = ArmTarget.Goal()
        goal.x = float(target['target_cm'][0])
        goal.y = float(target['target_cm'][1])
        goal.z = float(target['target_cm'][2])
        goal.duration = self._arm_goal_duration
        goal.end_angle = float(pitch)
        if self._enable_tool_yaw:
            goal.tool_yaw = float(target.get('yaw_deg', 0.0))
        else:
            goal.tool_yaw = 0.0

        send_future = self._arm_action.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                f'Arm target rejected at pitch={self._pitch_text(pitch)}'
            )
            return False

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_for_future(result_future, self._action_timeout)
        if wrapped is None:
            self.get_logger().error('Arm target timed out')
            return False

        result = wrapped.result
        if not result.success:
            self.get_logger().warning(
                f'Arm target failed at pitch={self._pitch_text(pitch)}'
            )
            return False

        self.get_logger().info(
            f'Arm target reached at pitch={self._pitch_text(pitch)}: '
            + ', '.join(f'{angle:.1f}' for angle in result.final_angles)
            + f' deg, tool_yaw={goal.tool_yaw:.1f} deg'
        )
        return True

    @staticmethod
    def _pitch_text(pitch):
        return 'auto' if pitch < -0.5 else f'{pitch:.1f} deg'

    def _send_home_via_moveit(self):
        """Return to HOME through /arm_target so MoveIt plans the motion."""
        if not self._arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not ready: {self._arm_action_name}'
            )
            return False

        if len(self._home_joint_positions) < 4:
            self.get_logger().warning(
                'Cannot send MoveIt HOME target; '
                'expected at least 4 joint values, '
                f'got {len(self._home_joint_positions)}'
            )
            return False

        target_cm, pitch_deg = self._home_target_from_joints(
            self._home_joint_positions
        )
        self.get_logger().info(
            f'Sending MoveIt HOME target: '
            f'x={target_cm[0]:.1f}, y={target_cm[1]:.1f}, '
            f'z={target_cm[2]:.1f} cm, pitch={pitch_deg:.1f} deg'
        )

        home_target = {
            'target_cm': target_cm,
            'pitch_deg': pitch_deg,
            'yaw_deg': 0.0,
        }
        return self._send_single_arm_target(home_target, pitch_deg)

    def _home_target_from_joints(self, joint_positions):
        theta1, theta2, theta3, theta4 = joint_positions[:4]
        l1, l2, l3, l4 = self._arm_link_lengths_cm

        pitch2 = math.pi / 2.0 - theta2
        pitch3 = math.pi / 2.0 - (theta2 + theta3)
        pitch4 = math.pi / 2.0 - (theta2 + theta3 + theta4)
        c1, s1 = math.cos(theta1), math.sin(theta1)

        x = (
            l2 * math.cos(pitch2)
            + l3 * math.cos(pitch3)
            + l4 * math.cos(pitch4)
        ) * c1
        y = (
            l2 * math.cos(pitch2)
            + l3 * math.cos(pitch3)
            + l4 * math.cos(pitch4)
        ) * s1
        z = (
            l1
            + l2 * math.sin(pitch2)
            + l3 * math.sin(pitch3)
            + l4 * math.sin(pitch4)
        )
        pitch_deg = math.degrees(theta2 + theta3 + theta4) - 90.0
        return np.array([x, y, z], dtype=float), pitch_deg

    def _call_gripper(self, open_ratio, duration_ms):
        if not self._gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f'Gripper service not ready: {self._gripper_service_name}'
            )
            return False

        request = SetGripper.Request()
        request.open_ratio = float(max(0.0, min(1.0, open_ratio)))
        request.duration_ms = float(duration_ms)
        future = self._gripper_client.call_async(request)
        response = self._wait_for_future(future, 5.0)
        if response is None:
            self.get_logger().warning('Gripper command timed out')
            return False
        if not response.success:
            self.get_logger().warning(
                f'Gripper command failed: {response.message}'
            )
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
