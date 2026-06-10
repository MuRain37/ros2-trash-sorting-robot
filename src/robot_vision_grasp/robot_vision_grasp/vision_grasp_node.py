"""Vision object detection to MoveIt arm target bridge."""
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
from robot_arm.arm_link_lengths import install_urdf_link_length_loader
from robot_interfaces.action import ArmTarget
from robot_interfaces.srv import SetGripper
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from robot_vision_grasp.grasp_geometry import get_gripper_yaw
from robot_vision_grasp.grasp_geometry import get_ik_pitch
from robot_vision_grasp.grasp_geometry import gripper_close_open_ratio
from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO


FALLBACK_LINK_LENGTHS_CM = (3.38648, 12.9416, 12.9444, 17.28648)
DEFAULT_CLASS_NAMES = ['bottle', 'pen', 'paper', 'charger', 'usb', 'yan']
DEFAULT_HOME_JOINT_POSITIONS = [
    0.0,
    -1.0471975511965976,
    2.0943951023931953,
    1.4660765716752369,
    0.0,
    0.0,
    0.0,
    0.0,
]

ACTION_TIMEOUT_SEC = 30.0
ARM_ACTION_NAME = '/arm_target'
GRIPPER_CLOSE_WAIT_SEC = 1.2
GRIPPER_DURATION_MS = 1000.0
GRIPPER_OPEN_WAIT_SEC = 1.2
GRIPPER_SERVICE_NAME = '/set_gripper'
HOME_DURATION_SEC = 2.0
TARGET_TIMEOUT_SEC = 2.0


class VisionGraspNode(Node):
    """Detect segmented objects and execute grasp motions through MoveIt."""

    def __init__(self):
        super().__init__('vision_grasp_node')

        callback_group = ReentrantCallbackGroup()
        self._load_config()

        self._state_lock = threading.Lock()
        self._vision_lock = threading.Lock()
        self._sequence_lock = threading.Lock()
        self._depth = None
        self._depth_encoding = 'passthrough'
        self._intrinsics = None
        self._arm_link_lengths_cm = FALLBACK_LINK_LENGTHS_CM

        self._bridge = CvBridge()
        self._model = self._load_model()
        self._last_target_time = None
        self._last_tf_warning = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._arm_action = ActionClient(
            self,
            ArmTarget,
            ARM_ACTION_NAME,
            callback_group=callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            GRIPPER_SERVICE_NAME,
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
        self._link_length_sub = install_urdf_link_length_loader(
            self,
            self._set_link_lengths,
        )

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _load_config(self):
        self._model_path = str(self._param('model_path', ''))
        self._model_device = self._param('model_device', 'CPU')
        self._class_names = list(
            self._param('class_names', DEFAULT_CLASS_NAMES)
        )
        self._confidence_threshold = float(
            self._param('confidence_threshold', 0.55)
        )
        self._iou_threshold = float(self._param('iou_threshold', 0.65))
        self._mask_threshold = float(self._param('mask_threshold', 0.5))
        self._color_topic = self._param(
            'color_topic',
            '/camera/camera/color/image_raw',
        )
        self._depth_topic = self._param(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self._depth_info_topic = self._param(
            'depth_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self._base_frame = self._param('base_frame', 'arm_link0')
        self._camera_frame = self._param(
            'camera_frame',
            'camera_optical_link',
        )
        self._arm_goal_duration = float(self._param('arm_goal_duration', 2.0))
        self._return_home = bool(self._param('return_home', True))
        self._home_joint_positions = [
            float(v)
            for v in self._param(
                'home_joint_positions',
                DEFAULT_HOME_JOINT_POSITIONS,
            )
        ]

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

        return model

    def _set_link_lengths(self, origin, lengths):
        with self._state_lock:
            self._arm_link_lengths_cm = tuple(float(v) for v in lengths)

    def _info_cb(self, msg):
        intrinsics = (
            float(msg.k[0]),
            float(msg.k[4]),
            float(msg.k[2]),
            float(msg.k[5]),
        )
        with self._state_lock:
            self._intrinsics = intrinsics

    def _depth_cb(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough',
            )
        except Exception as exc:
            self.get_logger().error(f'Depth convert failed: {exc}')
            return

        with self._state_lock:
            self._depth = depth
            self._depth_encoding = msg.encoding

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
                depth_encoding = self._depth_encoding
                intrinsics = self._intrinsics

            if depth is None or intrinsics is None:
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
                target = self._select_target(
                    cv_img,
                    depth,
                    depth_encoding,
                    intrinsics,
                )
            except Exception as exc:
                self.get_logger().warning(f'Vision frame skipped: {exc}')
                return

            if target is None:
                return

            with self._state_lock:
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
                self._sequence_lock.release()

    def _target_timeout_elapsed(self):
        with self._state_lock:
            last_target_time = self._last_target_time
        if last_target_time is None:
            return True
        now = self.get_clock().now()
        elapsed_ns = (now - last_target_time).nanoseconds
        return elapsed_ns >= TARGET_TIMEOUT_SEC * 1e9

    def _select_target(self, cv_img, depth, depth_encoding, intrinsics):
        targets = self._model.detect_targets(
            cv_img,
            depth,
            intrinsics,
            encoding=depth_encoding,
            conf_thres=self._confidence_threshold,
            mask_thres=self._mask_threshold,
        )
        if not targets:
            return None

        raw_target = targets[0]
        tf_target = self._camera_target_to_arm_target(raw_target)
        if tf_target is None:
            return None
        self.get_logger().info(
            f"{tf_target['class_name']}: "
            f"arm=({tf_target['object_arm_cm'][0]:.1f}, "
            f"{tf_target['object_arm_cm'][1]:.1f}, "
            f"{tf_target['object_arm_cm'][2]:.1f})cm "
            f"yaw={tf_target['yaw_deg']:.1f} deg "
            f"pitch={tf_target['pitch_deg']:.1f} deg"
        )
        return tf_target

    def _camera_target_to_arm_target(self, target):
        transform = self._lookup_tf_camera_transform()
        if transform is None:
            return None
        rotation, translation, _camera_pitch_deg = transform
        object_arm_m = translation + rotation @ target['point_cam_m']

        pitch_deg, axis_angle, is_upright = get_ik_pitch(
            target['class_name'],
            target['points_cam'],
            _camera_pitch_deg,
            target['mask'],
            target['depth_m'],
        )
        yaw_deg = get_gripper_yaw(
            target['class_name'],
            axis_angle,
            is_upright,
        )
        target_cm = self._target_cm_for_object(
            object_arm_m,
            target['class_name'],
            pitch_deg,
            is_upright,
        )
        return {
            'class_name': target['class_name'],
            'object_arm_cm': object_arm_m * 100.0,
            'target_cm': target_cm,
            'pitch_deg': pitch_deg,
            'yaw_deg': yaw_deg,
            'point_cam_m': target['point_cam_m'],
            'points_cam': target['points_cam'],
            'depth_m': target['depth_m'],
        }

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
            f'{self._camera_frame}): {exc}'
        )

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
        object_arm_m,
        class_name,
        pitch_deg,
        is_upright,
    ):
        arm_relative_cm = object_arm_m * 100.0
        target_cm = arm_relative_cm.copy()
        # target_cm[0] += (1.4+(abs_angle-45)**(abs_angle-45)/2025)
        if class_name == 'paper':
            target_cm[0] -= (2.0 - pitch_deg / 90.0)
        elif class_name == 'bottle':
            if is_upright:
                target_cm[0] -= 1.0
                target_cm[1] -= 3.0

        return target_cm

    def _run_grasp_sequence(self, target):
        try:
            class_name = target['class_name']
            if self._call_gripper(1.0, GRIPPER_DURATION_MS):
                time.sleep(GRIPPER_OPEN_WAIT_SEC)

            if not self._send_arm_target(target):
                return

            close_ratio = gripper_close_open_ratio(class_name)
            if self._call_gripper(close_ratio, GRIPPER_DURATION_MS):
                time.sleep(GRIPPER_CLOSE_WAIT_SEC)

            if self._return_home:
                home_reached = self._send_home_via_moveit()
                if home_reached:
                    time.sleep(HOME_DURATION_SEC)
                elif not home_reached:
                    self.get_logger().warning(
                        'HOME target was not reached; keeping vision paused '
                        'until this grasp sequence exits'
                    )
        except Exception as exc:
            self.get_logger().error(f'Grasp sequence failed: {exc}')
        finally:
            with self._state_lock:
                self._last_target_time = self.get_clock().now()
            self._sequence_lock.release()

    def _send_arm_target(self, target):
        if not self._arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not ready: {ARM_ACTION_NAME}'
            )
            return False

        pitch = float(target['pitch_deg'])
        if self._send_single_arm_target(target, pitch):
            return True

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
        goal.tool_yaw = float(target.get('yaw_deg', 0.0))

        send_future = self._arm_action.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                f'Arm target rejected at pitch={self._pitch_text(pitch)}'
            )
            return False

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_for_future(result_future, ACTION_TIMEOUT_SEC)
        if wrapped is None:
            self.get_logger().error('Arm target timed out')
            return False

        result = wrapped.result
        if not result.success:
            self.get_logger().warning(
                f'Arm target failed at pitch={self._pitch_text(pitch)}'
            )
            return False

        return True

    @staticmethod
    def _pitch_text(pitch):
        return 'auto' if pitch < -0.5 else f'{pitch:.1f} deg'

    def _send_home_via_moveit(self):
        """Return to HOME through /arm_target so MoveIt plans the motion."""
        if not self._arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not ready: {ARM_ACTION_NAME}'
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
                f'Gripper service not ready: {GRIPPER_SERVICE_NAME}'
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
