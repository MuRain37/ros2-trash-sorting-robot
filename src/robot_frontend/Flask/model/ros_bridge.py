import math
import os
from pathlib import Path as FilesystemPath
import signal
import shlex
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, LaserScan
    from tf2_ros import Buffer, TransformListener
except Exception as exc:
    rclpy = None
    Node = object
    RosPath = object
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None

try:
    from nav2_msgs.srv import LoadMap
except Exception as exc:
    LoadMap = None
    LOAD_MAP_IMPORT_ERROR = exc
else:
    LOAD_MAP_IMPORT_ERROR = None

try:
    from lifecycle_msgs.msg import State, Transition
    from lifecycle_msgs.srv import ChangeState, GetState
except Exception as exc:
    ChangeState = None
    GetState = None
    State = None
    Transition = None
    LIFECYCLE_IMPORT_ERROR = exc
else:
    LIFECYCLE_IMPORT_ERROR = None

try:
    from robot_interfaces.action import ArmPreset
except Exception as exc:
    ArmPreset = None
    ARM_PRESET_IMPORT_ERROR = exc
else:
    ARM_PRESET_IMPORT_ERROR = None

try:
    from robot_interfaces.action import RobotTask
except Exception as exc:
    RobotTask = None
    ROBOT_TASK_IMPORT_ERROR = exc
else:
    ROBOT_TASK_IMPORT_ERROR = None

try:
    from robot_interfaces.srv import SetCameraTfPreset
except Exception as exc:
    SetCameraTfPreset = None
    CAMERA_TF_PRESET_IMPORT_ERROR = exc
else:
    CAMERA_TF_PRESET_IMPORT_ERROR = None

DEFAULT_MONITOR_CLASS_NAMES = ['bottle', 'pen', 'paper', 'charger', 'usb', 'yan']
VISION_MODEL_IMPORT_ERROR = None
try:
    from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO
except Exception as exc:
    vision_package_path = FilesystemPath(__file__).resolve().parents[3] / "robot_vision_grasp"
    if vision_package_path.exists() and str(vision_package_path) not in sys.path:
        sys.path.append(str(vision_package_path))
    try:
        from robot_vision_grasp.yolo_seg_openvino import YOLOSegOpenVINO
    except Exception as retry_exc:
        YOLOSegOpenVINO = None
        VISION_MODEL_IMPORT_ERROR = retry_exc
    else:
        VISION_MODEL_IMPORT_ERROR = None
else:
    VISION_MODEL_IMPORT_ERROR = None


def _unique_paths(paths):
    seen = set()
    unique = []
    for path in paths:
        if not path:
            continue
        text = str(path)
        if text not in seen:
            seen.add(text)
            unique.append(text)
    return unique


def _repo_root():
    current = FilesystemPath(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "robot_vision_grasp").exists():
            return parent
    return current.parents[4]


def _vision_model_candidates(configured_path):
    default_relative = "src/robot_vision_grasp/robot_vision_grasp/models/best.xml"
    module_relative = (
        FilesystemPath(__file__).resolve().parents[3]
        / "robot_vision_grasp"
        / "robot_vision_grasp"
        / "models"
        / "best.xml"
    )
    repo_relative = _repo_root() / default_relative
    candidates = []
    if configured_path:
        configured = FilesystemPath(str(configured_path)).expanduser()
        candidates.append(configured)
        if not configured.is_absolute():
            candidates.append(_repo_root() / configured)
            candidates.append(FilesystemPath.cwd() / configured)
    candidates.extend([repo_relative, module_relative])
    return _unique_paths(candidates)


def _resolve_vision_model_path(configured_path):
    candidates = _vision_model_candidates(configured_path)
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate, candidates
    return None, candidates

def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _now_iso():
    return time.strftime("%Y-%m-%d %H:%M:%S")


class FlaskRosNode(Node):
    def __init__(self, bridge, config):
        super().__init__(config["node_name"])
        self.bridge = bridge
        self.config = config
        self.cv_bridge = CvBridge()
        self.map_frame = config["map_frame"]
        self.robot_base_frame = config["robot_base_frame"]

        self.robot_task_client = None
        if RobotTask is not None:
            self.robot_task_client = ActionClient(
                self,
                RobotTask,
                config["robot_task_action"],
            )
        self.arm_preset_client = None
        if ArmPreset is not None:
            self.arm_preset_client = ActionClient(
                self,
                ArmPreset,
                config["arm_preset_action"],
            )
        self.camera_tf_preset_client = None
        if SetCameraTfPreset is not None:
            self.camera_tf_preset_client = self.create_client(
                SetCameraTfPreset,
                config["camera_tf_preset_service"],
            )
        self.map_server_load_client = None
        if LoadMap is not None:
            self.map_server_load_client = self.create_client(
                LoadMap,
                config["map_server_load_map_service"],
            )
        self.amcl_get_state_client = None
        self.amcl_change_state_client = None
        if GetState is not None and ChangeState is not None:
            amcl_node = config["amcl_lifecycle_node"]
            self.amcl_get_state_client = self.create_client(
                GetState,
                f"{amcl_node}/get_state",
            )
            self.amcl_change_state_client = self.create_client(
                ChangeState,
                f"{amcl_node}/change_state",
            )
        self.cmd_vel_pub = self.create_publisher(Twist, config["cmd_vel_topic"], 10)
        self._velocity_lock = threading.Lock()
        self._velocity_command = Twist()
        self._velocity_updated_at = 0.0
        self._velocity_active = False
        self._velocity_stop_sent = True
        self._velocity_timeout_sec = float(config.get("velocity_command_timeout_sec", 0.6))
        self.create_timer(0.05, self.publish_velocity_timer)

        self.depth_topics = list(dict.fromkeys(config["depth_camera_topics"]))
        self.color_topics = list(dict.fromkeys(config["color_camera_topics"]))
        for topic in self.depth_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.handle_depth_image(msg, topic),
                qos_profile_sensor_data,
            )
        for topic in self.color_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.handle_color_image(msg, topic),
                qos_profile_sensor_data,
            )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            config["map_topic"],
            self.handle_map,
            map_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            config["amcl_pose_topic"],
            self.handle_amcl_pose,
            10,
        )
        self.create_subscription(
            LaserScan,
            config["scan_topic"],
            self.handle_scan,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.06, self.update_tf_pose)

        self.get_logger().info(
            "Flask ROS bridge ready: "
            f"depth={self.depth_topics}, "
            f"color={self.color_topics}, map={config['map_topic']}, "
            f"robot_task={config['robot_task_action']}"
        )

    def handle_depth_image(self, msg, topic):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            jpeg = self.bridge.depth_image_to_jpeg(image)
            self.bridge.set_camera_frame(
                "depth",
                jpeg,
                msg.header.frame_id,
                _stamp_to_float(msg.header.stamp),
                topic,
                image=image,
            )
        except Exception as exc:
            self.bridge.set_error("depth_camera", str(exc))

    def handle_color_image(self, msg, topic):
        try:
            try:
                image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if image.ndim == 2:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                elif image.ndim == 3 and image.shape[2] == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
            if ok:
                self.bridge.set_camera_frame(
                    "color",
                    buffer.tobytes(),
                    msg.header.frame_id,
                    _stamp_to_float(msg.header.stamp),
                    topic,
                    image=image,
                )
        except Exception as exc:
            self.bridge.set_error("color_camera", str(exc))

    def handle_map(self, msg):
        try:
            self.bridge.set_map(msg)
        except Exception as exc:
            self.bridge.set_error("map", str(exc))

    def handle_amcl_pose(self, msg):
        if self.bridge.is_mapping_active():
            return
        if self.bridge.prefers_tf_pose():
            return
        pose = msg.pose.pose
        self.bridge.set_robot_pose(
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
            source="amcl_pose",
            stamp=_stamp_to_float(msg.header.stamp),
        )

    def handle_scan(self, msg):
        self.bridge.set_scan(msg)

    def update_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                Time(),
            )
        except Exception:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.bridge.set_robot_pose(
            translation.x,
            translation.y,
            _yaw_from_quaternion(rotation),
            source="tf",
            stamp=_stamp_to_float(transform.header.stamp),
        )

    def publish_stop(self):
        self.set_velocity_command(0.0, 0.0)
        self.cmd_vel_pub.publish(Twist())

    def publish_velocity(self, linear_x, angular_z):
        self.set_velocity_command(linear_x, angular_z)

    def set_velocity_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        with self._velocity_lock:
            self._velocity_command = msg
            self._velocity_updated_at = time.time()
            self._velocity_active = True
            self._velocity_stop_sent = False

    def publish_velocity_timer(self):
        with self._velocity_lock:
            age = time.time() - self._velocity_updated_at
            if not self._velocity_active:
                return
            if age <= self._velocity_timeout_sec:
                msg = self._velocity_command
            elif not self._velocity_stop_sent:
                msg = Twist()
                self._velocity_stop_sent = True
            else:
                self._velocity_active = False
                return
        self.cmd_vel_pub.publish(msg)

    def send_robot_task_goal(self, task_type, **kwargs):
        if RobotTask is None or self.robot_task_client is None:
            raise RuntimeError(f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}")
        feedback_callback = kwargs.pop("feedback_callback", None)
        goal = RobotTask.Goal()
        goal.task_type = int(task_type)
        goal.command = str(kwargs.get("command", ""))
        goal.bucket_id = int(kwargs.get("bucket_id", 0) or 0)
        goal.x = float(kwargs.get("x", 0.0) or 0.0)
        goal.y = float(kwargs.get("y", 0.0) or 0.0)
        goal.yaw = float(kwargs.get("yaw", 0.0) or 0.0)
        goal.min_x = float(kwargs.get("min_x", 0.0) or 0.0)
        goal.min_y = float(kwargs.get("min_y", 0.0) or 0.0)
        goal.max_x = float(kwargs.get("max_x", 0.0) or 0.0)
        goal.max_y = float(kwargs.get("max_y", 0.0) or 0.0)
        goal.frame_id = kwargs.get("frame_id") or self.map_frame
        if kwargs.get("path") is not None:
            goal.path = kwargs["path"]
        return self.robot_task_client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )


class RosBridge:
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.node = None
        self.executor = None
        self.thread = None
        self.mapping_process = None
        self.started = False
        self.start_error = None

        self.frames = {
            "depth": {"jpeg": None, "frame_id": None, "stamp": None, "received_at": None, "topic": None, "image": None},
            "color": {"jpeg": None, "frame_id": None, "stamp": None, "received_at": None, "topic": None, "image": None},
        }
        self.vision_lock = threading.Lock()
        self.vision_model = None
        self.vision_model_error = None
        self.vision_last_frame = None
        self.vision_last_completed_at = None
        self.vision_info = {
            "enabled": bool(self.config.get("vision_monitor_enabled", True)),
            "model_loaded": False,
            "connected": False,
            "message": "等待彩色摄像头",
            "objects": [],
            "updated_at": None,
            "topic": None,
            "frame_id": None,
            "depth_topic": None,
            "depth_frame_id": None,
        }
        self.map = None
        self.map_png = None
        self.map_version = 0
        self.accept_live_map = True
        self.mapping_active = False
        self.robot_pose = None
        self.last_scan = None
        self.target_point = None
        self.latest_velocity_command_session = None
        self.latest_velocity_command_seq = -1
        self.nav_status = {
            "state": "idle",
            "message": "未发送导航目标",
            "updated_at": _now_iso(),
        }
        self.robot_task_status = {
            "state": "idle",
            "message": "未发送行为树任务",
            "updated_at": _now_iso(),
        }
        self.bucket_fill_levels = {1: 0, 2: 0}
        self.errors = {}
        self.startup_static_map_yaml = None
        self.load_static_map(remember_startup=True)

    def _map_yaml_candidates(self, yaml_path=None):
        candidates = []
        for raw_path in (
            yaml_path,
            self.config.get("static_map_yaml"),
            self.config.get("map_save_prefix"),
        ):
            if not raw_path:
                continue
            raw_path = str(raw_path).strip()
            if not raw_path:
                continue
            if raw_path.endswith((".yaml", ".yml")):
                path_candidates = (raw_path,)
            else:
                path_candidates = (raw_path, f"{raw_path}.yaml", f"{raw_path}.yml")
            for candidate in path_candidates:
                if candidate not in candidates:
                    candidates.append(candidate)
        return candidates

    def _resolve_map_yaml_path(self, yaml_path=None):
        candidates = self._map_yaml_candidates(yaml_path)
        for candidate in candidates:
            if os.path.isfile(candidate):
                return candidate, candidates
        return (candidates[0] if candidates else yaml_path), candidates

    def load_static_map(self, yaml_path=None, remember_startup=False):
        yaml_path, candidates = self._resolve_map_yaml_path(yaml_path)
        if not yaml_path or not os.path.exists(yaml_path):
            tried = "，".join(candidates) if candidates else str(yaml_path)
            message = f"保存地图文件不存在，已尝试: {tried}"
            self.set_error("static_map", message)
            return {
                "success": False,
                "message": message,
                "yaml_path": yaml_path,
                "candidates": candidates,
            }

        try:
            info = {}
            with open(yaml_path, "r", encoding="utf-8") as yaml_file:
                for line in yaml_file:
                    line = line.split("#", 1)[0].strip()
                    if not line or ":" not in line:
                        continue
                    key, value = line.split(":", 1)
                    info[key.strip()] = value.strip()

            image_path = info.get("image")
            if not image_path:
                message = f"地图 YAML 缺少 image 字段: {yaml_path}"
                self.set_error("static_map", message)
                return {"success": False, "message": message, "yaml_path": yaml_path}
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)
            if not os.path.exists(image_path):
                message = f"保存地图图片不存在: {image_path}"
                self.set_error("static_map", message)
                return {
                    "success": False,
                    "message": message,
                    "yaml_path": yaml_path,
                    "image_path": image_path,
                }
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                message = f"保存地图图片读取失败: {image_path}"
                self.set_error("static_map", message)
                return {
                    "success": False,
                    "message": message,
                    "yaml_path": yaml_path,
                    "image_path": image_path,
                }

            resolution = float(info.get("resolution", 1.0))
            origin_text = info.get("origin", "[0, 0, 0]").strip()
            origin = [
                float(value.strip())
                for value in origin_text.strip("[]").split(",")
                if value.strip()
            ]
            while len(origin) < 3:
                origin.append(0.0)

            height, width = image.shape[:2]
            color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            ok, buffer = cv2.imencode(".png", color)
            if not ok:
                message = f"保存地图 PNG 编码失败: {image_path}"
                self.set_error("static_map", message)
                return {
                    "success": False,
                    "message": message,
                    "yaml_path": yaml_path,
                    "image_path": image_path,
                }

            with self.lock:
                self.map = {
                    "frame_id": self.config["map_frame"],
                    "width": int(width),
                    "height": int(height),
                    "resolution": resolution,
                    "origin": {
                        "x": origin[0],
                        "y": origin[1],
                        "z": 0.0,
                        "theta": origin[2],
                    },
                    "received_at": time.time(),
                    "stamp": None,
                    "source": "static_file",
                }
                self.map_png = buffer.tobytes()
                self.map_version += 1
                self.errors.pop("static_map", None)
                if remember_startup or self.startup_static_map_yaml is None:
                    self.startup_static_map_yaml = yaml_path
            return {
                "success": True,
                "message": "已从文件加载保存地图",
                "yaml_path": yaml_path,
                "image_path": image_path,
                "width": int(width),
                "height": int(height),
                "resolution": resolution,
            }
        except Exception as exc:
            message = f"保存地图加载失败: {exc}"
            self.set_error("static_map", message)
            return {"success": False, "message": message, "yaml_path": yaml_path}

    def start(self):
        if self.started:
            return True
        if not self.config.get("enabled", True):
            self.start_error = "ROS bridge 已被配置禁用"
            return False
        if rclpy is None:
            self.start_error = f"ROS Python 依赖导入失败: {ROS_IMPORT_ERROR}"
            return False

        try:
            os.environ.setdefault("ROS_LOG_DIR", "/tmp/flask_ros_logs")
            os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = FlaskRosNode(self, self.config)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, name="flask-ros2-spin", daemon=True)
            self.thread.start()
            self.started = True
            self.start_error = None
            return True
        except Exception as exc:
            self.start_error = str(exc)
            self.started = False
            return False

    def shutdown(self):
        self.stop_mapping_process()
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        self.started = False

    def set_error(self, key, message):
        with self.lock:
            self.errors[key] = {
                "message": message,
                "updated_at": _now_iso(),
            }

    def set_camera_frame(self, camera_id, jpeg, frame_id, stamp, topic, image=None):
        with self.lock:
            self.frames[camera_id] = {
                "jpeg": jpeg,
                "frame_id": frame_id,
                "stamp": stamp,
                "received_at": time.time(),
                "topic": topic,
                "image": image.copy() if image is not None else None,
            }
            self.errors.pop(f"{camera_id}_camera", None)

    def depth_image_to_jpeg(self, image):
        if image.ndim == 3:
            image = image[:, :, 0]

        depth = image.astype(np.float32)
        if image.dtype == np.uint16:
            depth = depth / 1000.0

        max_depth = max(0.1, float(self.config.get("depth_visual_max_m", 8.0)))
        valid = np.isfinite(depth) & (depth > 0.0)
        normalized = np.zeros(depth.shape, dtype=np.uint8)
        normalized[valid] = np.clip((1.0 - depth[valid] / max_depth) * 255.0, 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        color[~valid] = (0, 0, 0)
        ok, buffer = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            raise RuntimeError("深度图 JPEG 编码失败")
        return buffer.tobytes()

    def set_map(self, msg):
        if not self.accept_live_map:
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return

        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[data < 0] = (150, 150, 150)
        image[(data >= 0) & (data < 50)] = (245, 245, 245)
        image[data >= 50] = (28, 31, 35)
        image = np.flipud(image)
        ok, buffer = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError("地图 PNG 编码失败")

        origin = msg.info.origin
        with self.lock:
            self.map = {
                "frame_id": msg.header.frame_id or self.config["map_frame"],
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin": {
                    "x": float(origin.position.x),
                    "y": float(origin.position.y),
                    "z": float(origin.position.z),
                    "theta": _yaw_from_quaternion(origin.orientation),
                },
                "received_at": time.time(),
                "stamp": _stamp_to_float(msg.header.stamp),
                "source": "ros_topic",
            }
            self.map_png = buffer.tobytes()
            self.map_version += 1
            self.errors.pop("map", None)

    def set_robot_pose(self, x, y, theta, source, stamp):
        with self.lock:
            current = self.robot_pose
            if (
                source != "tf"
                and current
                and current.get("source") == "tf"
                and time.time() - current.get("received_at", 0.0) < 0.5
            ):
                return
            self.robot_pose = {
                "x": float(x),
                "y": float(y),
                "theta": float(theta),
                "theta_deg": math.degrees(float(theta)),
                "source": source,
                "stamp": stamp,
                "received_at": time.time(),
            }

    def prefers_tf_pose(self):
        with self.lock:
            return (
                self.robot_pose is not None
                and self.robot_pose.get("source") == "tf"
                and time.time() - self.robot_pose.get("received_at", 0.0) < 0.5
            )

    def is_mapping_active(self):
        with self.lock:
            return bool(self.mapping_active)

    def set_scan(self, msg):
        valid_ranges = [value for value in msg.ranges if math.isfinite(value)]
        with self.lock:
            self.last_scan = {
                "frame_id": msg.header.frame_id,
                "count": len(msg.ranges),
                "valid_count": len(valid_ranges),
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "received_at": time.time(),
            }

    def wait_for_camera_frame(self, camera_id, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self.get_camera_frame(camera_id)
            if frame:
                return frame
            time.sleep(0.03)
        return None

    def get_camera_frame(self, camera_id):
        if camera_id in {"main", "depth_camera"}:
            camera_id = "depth"
        if camera_id not in self.frames:
            camera_id = "depth"
        with self.lock:
            frame = self.frames[camera_id]
            return frame["jpeg"]

    def get_camera_info(self):
        with self.lock:
            return {
                camera_id: {
                    "frame_id": frame["frame_id"],
                    "stamp": frame["stamp"],
                    "topic": frame["topic"],
                    "age_sec": round(time.time() - frame["received_at"], 2) if frame["received_at"] else None,
                    "connected": frame["received_at"] is not None and (time.time() - frame["received_at"]) < 5.0,
                }
                for camera_id, frame in self.frames.items()
            }

    def _load_vision_model(self):
        if self.vision_model is not None:
            return self.vision_model
        if self.vision_model_error:
            return None
        if not self.config.get("vision_monitor_enabled", True):
            self.vision_model_error = "实例分割监控已禁用"
            return None
        if YOLOSegOpenVINO is None:
            self.vision_model_error = f"YOLO 分割模型导入失败: {VISION_MODEL_IMPORT_ERROR}"
            return None

        model_path, model_candidates = _resolve_vision_model_path(
            self.config.get("vision_model_path")
        )
        if model_path is None:
            tried = "，".join(model_candidates)
            self.vision_model_error = f"YOLO 分割模型文件不存在，已尝试: {tried}"
            return None
        class_names = self.config.get("vision_class_names") or DEFAULT_MONITOR_CLASS_NAMES
        try:
            self.vision_model = YOLOSegOpenVINO(
                model_path,
                device=self.config.get("vision_model_device", "CPU"),
                names={index: name for index, name in enumerate(class_names)},
                conf_thres=float(self.config.get("vision_confidence_threshold", 0.55)),
                iou_thres=float(self.config.get("vision_iou_threshold", 0.65)),
            )
            self.vision_model_error = None
            return self.vision_model
        except Exception as exc:
            self.vision_model_error = f"YOLO 分割模型加载失败: {exc}; model_path={model_path}"
            return None

    def get_vision_frame(self):
        with self.lock:
            color_frame = self.frames.get("color", {})
            depth_frame = self.frames.get("depth", {})
            color_image = color_frame.get("image")
            depth_image = depth_frame.get("image")
            cached_frame = self.vision_last_frame
            last_completed_at = self.vision_last_completed_at
            frame_meta = {
                "topic": color_frame.get("topic"),
                "frame_id": color_frame.get("frame_id"),
                "stamp": color_frame.get("stamp"),
                "depth_topic": depth_frame.get("topic"),
                "depth_frame_id": depth_frame.get("frame_id"),
                "connected": (
                    color_frame.get("received_at") is not None
                    and (time.time() - color_frame.get("received_at")) < 5.0
                ),
            }

        max_fps = max(0.5, float(self.config.get("vision_monitor_max_fps", 6.0)))
        if cached_frame is not None and last_completed_at and time.time() - last_completed_at < 1.0 / max_fps:
            return cached_frame

        if color_image is None:
            frame = placeholder_jpeg("No color frame")
            self._set_vision_info([], "等待彩色摄像头", frame_meta, frame, connected=False)
            return frame

        if not self.vision_lock.acquire(blocking=False):
            if cached_frame is not None:
                return cached_frame
            return self._encode_vision_frame(color_image, [], "Detecting...")

        try:
            model = self._load_vision_model()
            if model is None:
                message = self.vision_model_error or "YOLO segmentation model unavailable"
                frame = self._encode_vision_frame(color_image, [], message)
                self._set_vision_info([], message, frame_meta, frame, connected=frame_meta["connected"])
                return frame

            results = model(color_image, verbose=False)
            detections = self._vision_detections_from_results(results, color_image.shape[:2])
            self._update_vision_timestamp()
            self._attach_vision_pose_estimates(detections, depth_image, color_image.shape[:2])
            message = "识别到物体" if detections else "未识别到物体"
            frame = self._encode_vision_frame(color_image, detections, message)
            self._set_vision_info(detections, message, frame_meta, frame, connected=frame_meta["connected"])
            return frame
        except Exception as exc:
            message = f"实例分割失败: {exc}"
            frame = self._encode_vision_frame(color_image, [], message)
            self._set_vision_info([], message, frame_meta, frame, connected=frame_meta["connected"])
            return frame
        finally:
            self.vision_lock.release()

    def _update_vision_timestamp(self):
        self.vision_last_completed_at = time.time()

    def _vision_detections_from_results(self, results, image_shape):
        detections = []
        conf_threshold = float(self.config.get("vision_confidence_threshold", 0.55))
        mask_threshold = float(self.config.get("vision_mask_threshold", 0.5))
        for result in results or []:
            boxes = getattr(result, "boxes", None) or []
            names = getattr(result, "names", {}) or {}
            for index, box in enumerate(boxes):
                try:
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    xyxy = np.asarray(box.xyxy[0], dtype=float)
                except (IndexError, TypeError, ValueError, AttributeError):
                    continue
                if confidence < conf_threshold:
                    continue
                mask = self._vision_mask_for_result(result, index, image_shape, mask_threshold)
                if mask is None or not np.any(mask):
                    continue
                rotation_deg = self._vision_mask_rotation_deg(mask)
                detections.append(
                    {
                        "class_name": names.get(cls_id, str(cls_id)),
                        "confidence": confidence,
                        "bbox": np.clip(
                            xyxy,
                            [0, 0, 0, 0],
                            [image_shape[1] - 1, image_shape[0] - 1, image_shape[1] - 1, image_shape[0] - 1],
                        ).astype(int).tolist(),
                        "mask": mask,
                        "mask_pixels": int(np.count_nonzero(mask)),
                        "rotation_deg": rotation_deg,
                    }
                )
        detections.sort(key=lambda item: item["confidence"], reverse=True)
        return detections

    @staticmethod
    def _vision_mask_rotation_deg(mask):
        try:
            mask_u8 = mask.astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) <= 1.0:
                return None
            (_, _), (_, _), angle = cv2.minAreaRect(contour)
            if angle < -45.0:
                angle += 90.0
            return round(float(angle), 1)
        except Exception:
            return None

    def _attach_vision_pose_estimates(self, detections, depth_image, image_shape):
        if not detections:
            return

        height, width = image_shape
        center_x = width * 0.5
        center_y = height * 0.5
        hfov = math.radians(float(self.config.get("vision_camera_hfov_deg", 69.0)))
        vfov = math.radians(float(self.config.get("vision_camera_vfov_deg", 42.0)))

        depth = None
        if depth_image is not None:
            depth = depth_image[:, :, 0] if depth_image.ndim == 3 else depth_image
            depth = depth.astype(np.float32)
            if depth_image.dtype == np.uint16:
                depth = depth / 1000.0
            if depth.shape[:2] != (height, width):
                depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)

        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            mask = detection.get("mask")
            if mask is not None and np.any(mask):
                ys, xs = np.where(mask)
                pixel_x = float(np.mean(xs))
                pixel_y = float(np.mean(ys))
            else:
                pixel_x = (x1 + x2) * 0.5
                pixel_y = (y1 + y2) * 0.5

            yaw = math.atan2(pixel_x - center_x, max(1.0, center_x) / math.tan(hfov * 0.5))
            pitch = math.atan2(center_y - pixel_y, max(1.0, center_y) / math.tan(vfov * 0.5))
            z_m = None
            if depth is not None:
                x1c, x2c = max(0, int(x1)), min(depth.shape[1], int(x2) + 1)
                y1c, y2c = max(0, int(y1)), min(depth.shape[0], int(y2) + 1)
                if x1c < x2c and y1c < y2c:
                    region = depth[y1c:y2c, x1c:x2c]
                    if mask is not None and mask.shape[:2] == depth.shape[:2]:
                        values = region[mask[y1c:y2c, x1c:x2c]]
                    else:
                        values = region.reshape(-1)
                    values = values[np.isfinite(values) & (values > 0.05)]
                    if values.size:
                        z_m = float(np.median(values))

            x_m = y_m = distance_m = None
            if z_m is not None:
                x_m = math.tan(yaw) * z_m
                y_m = math.tan(pitch) * z_m
                distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)

            detection["pose"] = {
                "pixel": {"x": round(pixel_x, 1), "y": round(pixel_y, 1)},
                "x_m": round(x_m, 3) if x_m is not None else None,
                "y_m": round(y_m, 3) if y_m is not None else None,
                "z_m": round(z_m, 3) if z_m is not None else None,
                "distance_m": round(distance_m, 3) if distance_m is not None else None,
                "yaw_deg": round(math.degrees(yaw), 1),
                "pitch_deg": round(math.degrees(pitch), 1),
                "rotation_deg": detection.get("rotation_deg"),
            }

    @staticmethod
    def _vision_mask_for_result(result, index, image_shape, mask_threshold):
        masks = getattr(result, "masks", None)
        if masks is None:
            return None
        try:
            if getattr(masks, "soft_data", None) is not None:
                mask = np.asarray(masks.soft_data[index], dtype=np.float32)
            else:
                mask = np.asarray(masks.data[index], dtype=np.float32)
        except (IndexError, TypeError, ValueError):
            return None

        height, width = image_shape
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_LINEAR)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask_bin = (mask > mask_threshold).astype(np.uint8)
        mask_bin = cv2.morphologyEx(
            mask_bin,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )
        return mask_bin.astype(bool)

    def _encode_vision_frame(self, image, detections, message):
        frame = image.copy()
        palette = [
            (38, 166, 154),
            (255, 171, 64),
            (66, 165, 245),
            (171, 71, 188),
            (102, 187, 106),
            (239, 83, 80),
        ]

        if detections:
            overlay = frame.copy()
            for index, detection in enumerate(detections):
                color = np.array(palette[index % len(palette)], dtype=np.uint8)
                overlay[detection["mask"]] = (
                    overlay[detection["mask"]].astype(np.float32) * 0.35
                    + color.astype(np.float32) * 0.65
                ).astype(np.uint8)
            frame = cv2.addWeighted(overlay, 0.55, frame, 0.45, 0)

            for index, detection in enumerate(detections):
                color = palette[index % len(palette)]
                mask_u8 = detection["mask"].astype(np.uint8) * 255
                contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(frame, contours, -1, color, 2)
                x1, y1, x2, y2 = detection["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                pose = detection.get("pose") or {}
                label = f"{detection['class_name']} {detection['confidence']:.2f}"
                if pose.get("z_m") is not None and pose.get("yaw_deg") is not None:
                    label = f"{label} Z {pose['z_m']:.2f}m Yaw {pose['yaw_deg']:+.1f}"
                cv2.putText(
                    frame,
                    label,
                    (x1, max(24, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    color,
                    2,
                    cv2.LINE_AA,
                )
        else:
            cv2.putText(
                frame,
                "No object detected",
                (24, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.78,
                (220, 230, 235),
                2,
                cv2.LINE_AA,
            )

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 32), (16, 24, 28), -1)
        if detections:
            pose = detections[0].get("pose") or {}
            if pose.get("z_m") is not None:
                banner = (
                    f"X {pose['x_m']:+.2f}m  Y {pose['y_m']:+.2f}m  "
                    f"Z {pose['z_m']:.2f}m  Rot {pose.get('rotation_deg') if pose.get('rotation_deg') is not None else '-'}deg"
                )
            else:
                banner = "Target pose: waiting depth"
        else:
            banner = str(message or "No object detected")[:80]
        cv2.putText(
            frame,
            banner,
            (16, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (235, 245, 240),
            2,
            cv2.LINE_AA,
        )
        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return buffer.tobytes() if ok else b""

    def _set_vision_info(self, detections, message, frame_meta, jpeg, connected):
        objects = [
            {
                "class_name": detection["class_name"],
                "confidence": round(float(detection["confidence"]), 3),
                "mask_pixels": int(detection["mask_pixels"]),
                "bbox": list(detection["bbox"]),
                "pose": dict(detection.get("pose") or {}),
            }
            for detection in detections
        ]
        with self.lock:
            self.vision_last_frame = jpeg
            self.vision_info = {
                "enabled": bool(self.config.get("vision_monitor_enabled", True)),
                "model_loaded": self.vision_model is not None,
                "model_error": self.vision_model_error,
                "connected": bool(connected),
                "message": message,
                "objects": objects,
                "updated_at": _now_iso(),
                "topic": frame_meta.get("topic"),
                "frame_id": frame_meta.get("frame_id"),
                "depth_topic": frame_meta.get("depth_topic"),
                "depth_frame_id": frame_meta.get("depth_frame_id"),
                "stamp": frame_meta.get("stamp"),
            }

    def get_vision_info(self):
        with self.lock:
            info = dict(self.vision_info)
            info["objects"] = [dict(item) for item in self.vision_info.get("objects", [])]
            return info

    def get_map_png(self):
        with self.lock:
            return self.map_png

    def get_map_data(self):
        with self.lock:
            info = dict(self.map) if self.map else None
            if info:
                info["width_m"] = info["width"] * info["resolution"]
                info["height_m"] = info["height"] * info["resolution"]
                info["image_version"] = self.map_version
                info["age_sec"] = round(time.time() - info["received_at"], 3) if info.get("received_at") else None

            robot_position = dict(self.robot_pose) if self.robot_pose else None
            if robot_position:
                robot_position["age_sec"] = (
                    round(time.time() - robot_position["received_at"], 3)
                    if robot_position.get("received_at")
                    else None
                )

            scan = dict(self.last_scan) if self.last_scan else None
            if scan:
                scan["age_sec"] = round(time.time() - scan["received_at"], 3) if scan.get("received_at") else None
            return {
                "has_map": self.map is not None,
                "map": info,
                "robot_position": robot_position,
                "target_point": dict(self.target_point) if self.target_point else None,
                "nav_status": dict(self.nav_status),
                "scan": scan,
            }

    def get_robot_pose_data(self):
        with self.lock:
            robot_position = dict(self.robot_pose) if self.robot_pose else None
            if robot_position:
                robot_position["age_sec"] = (
                    round(time.time() - robot_position["received_at"], 3)
                    if robot_position.get("received_at")
                    else None
                )
            return {
                "has_pose": robot_position is not None,
                "robot_position": robot_position,
            }

    def reset_mapping(self):
        prepare_result = self.prepare_mapping_camera_pose()
        if not prepare_result.get("success"):
            with self.lock:
                self.nav_status = {
                    "state": "mapping_prepare_failed",
                    "message": prepare_result.get("message", "建图姿态准备失败"),
                    "updated_at": _now_iso(),
                }
            return prepare_result

        amcl_result = self.deactivate_amcl_for_mapping()
        if not amcl_result.get("success"):
            with self.lock:
                self.nav_status = {
                    "state": "mapping_prepare_failed",
                    "message": amcl_result.get("message", "AMCL 停止失败"),
                    "updated_at": _now_iso(),
                }
            return amcl_result

        process_result = self.start_mapping_process()
        if not process_result.get("success"):
            self.activate_amcl_after_mapping()
        with self.lock:
            self.mapping_active = bool(process_result.get("success"))
            self.accept_live_map = True
            self.map = None
            self.map_png = None
            self.map_version += 1
            self.target_point = None
            self.nav_status = {
                "state": "mapping",
                "message": process_result["message"],
                "updated_at": _now_iso(),
            }
            self.errors.pop("static_map", None)
        return {
            "success": process_result["success"],
            "message": process_result["message"],
            "command": process_result.get("command"),
            "amcl_lifecycle": amcl_result,
        }

    def prepare_mapping_camera_pose(self):
        if not self.started or not self.node:
            return {
                "success": False,
                "message": self.start_error or "ROS bridge 未启动",
            }
        if ArmPreset is None or self.node.arm_preset_client is None:
            return {
                "success": False,
                "message": f"ArmPreset 接口未加载: {ARM_PRESET_IMPORT_ERROR}",
            }
        if (
            SetCameraTfPreset is None
            or self.node.camera_tf_preset_client is None
        ):
            return {
                "success": False,
                "message": (
                    "SetCameraTfPreset 接口未加载: "
                    f"{CAMERA_TF_PRESET_IMPORT_ERROR}"
                ),
            }

        timeout_sec = float(
            self.config.get("mapping_prepare_timeout_sec", 60.0)
        )
        action_type = ArmPreset.Goal.ARM_MAPPING
        arm_result = self._send_arm_preset_sync(action_type, timeout_sec)
        if not arm_result.get("success"):
            return arm_result

        tf_result = self._set_camera_tf_preset_sync(
            action_type,
            float(self.config.get("goal_timeout_sec", 3.0)),
        )
        if not tf_result.get("success"):
            return tf_result

        return {
            "success": True,
            "message": "机械臂已切换建图姿态，真实相机 TF 已更新",
        }

    def _send_arm_preset_sync(self, action_type, timeout_sec):
        client = self.node.arm_preset_client
        if not client.wait_for_server(
            timeout_sec=float(self.config.get("goal_timeout_sec", 3.0))
        ):
            return {
                "success": False,
                "message": f"ArmPreset Action 不可用: {self.config['arm_preset_action']}",
            }

        goal = ArmPreset.Goal()
        goal.action_type = int(action_type)
        done_event = threading.Event()
        result = {}

        def on_result(done):
            try:
                wrapped = done.result()
                action_result = wrapped.result
                result.update(
                    {
                        "success": bool(action_result.success),
                        "message": action_result.message,
                    }
                )
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"ArmPreset 结果失败: {exc}",
                    }
                )
            finally:
                done_event.set()

        def on_goal(done):
            try:
                goal_handle = done.result()
                if goal_handle is None or not goal_handle.accepted:
                    result.update(
                        {
                            "success": False,
                            "message": "ArmPreset 建图姿态目标被拒绝",
                        }
                    )
                    done_event.set()
                    return
                goal_handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"ArmPreset 发送失败: {exc}",
                    }
                )
                done_event.set()

        client.send_goal_async(goal).add_done_callback(on_goal)
        if not done_event.wait(float(timeout_sec)):
            return {
                "success": False,
                "message": "ArmPreset 建图姿态执行超时",
            }
        return result

    def _set_camera_tf_preset_sync(self, action_type, timeout_sec):
        client = self.node.camera_tf_preset_client
        if not client.wait_for_service(timeout_sec=float(timeout_sec)):
            return {
                "success": False,
                "message": (
                    "真实相机 TF preset 服务不可用: "
                    f"{self.config['camera_tf_preset_service']}"
                ),
            }

        request = SetCameraTfPreset.Request()
        request.action_type = int(action_type)
        done_event = threading.Event()
        result = {}

        def on_response(done):
            try:
                response = done.result()
                result.update(
                    {
                        "success": bool(response.success),
                        "message": response.message,
                    }
                )
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"真实相机 TF preset 调用失败: {exc}",
                    }
                )
            finally:
                done_event.set()

        client.call_async(request).add_done_callback(on_response)
        if not done_event.wait(float(timeout_sec)):
            return {
                "success": False,
                "message": "真实相机 TF preset 调用超时",
            }
        return result

    def _lifecycle_service_ready(self):
        if not self.started or not self.node:
            return {
                "success": False,
                "message": self.start_error or "ROS bridge 未启动",
            }
        if (
            GetState is None
            or ChangeState is None
            or State is None
            or Transition is None
            or self.node.amcl_get_state_client is None
            or self.node.amcl_change_state_client is None
        ):
            return {
                "success": False,
                "message": f"Lifecycle 接口未加载: {LIFECYCLE_IMPORT_ERROR}",
            }
        return {"success": True}

    def _call_lifecycle_get_state(self, timeout_sec):
        ready = self._lifecycle_service_ready()
        if not ready.get("success"):
            return ready

        client = self.node.amcl_get_state_client
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {
                "success": False,
                "message": (
                    "AMCL get_state 服务不可用: "
                    f"{self.config['amcl_lifecycle_node']}/get_state"
                ),
            }

        done_event = threading.Event()
        result = {}

        def on_response(done):
            try:
                response = done.result()
                result.update(
                    {
                        "success": True,
                        "state_id": int(response.current_state.id),
                        "state_label": response.current_state.label,
                    }
                )
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"AMCL get_state 调用失败: {exc}",
                    }
                )
            finally:
                done_event.set()

        client.call_async(GetState.Request()).add_done_callback(on_response)
        if not done_event.wait(float(timeout_sec)):
            return {
                "success": False,
                "message": "AMCL get_state 调用超时",
            }
        return result

    def _change_amcl_state(self, transition_id, label, timeout_sec):
        ready = self._lifecycle_service_ready()
        if not ready.get("success"):
            return ready

        client = self.node.amcl_change_state_client
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {
                "success": False,
                "message": (
                    "AMCL change_state 服务不可用: "
                    f"{self.config['amcl_lifecycle_node']}/change_state"
                ),
            }

        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        done_event = threading.Event()
        result = {}

        def on_response(done):
            try:
                response = done.result()
                result.update(
                    {
                        "success": bool(response.success),
                        "message": (
                            f"AMCL 已{label}"
                            if response.success
                            else f"AMCL {label}请求被拒绝"
                        ),
                    }
                )
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"AMCL {label}调用失败: {exc}",
                    }
                )
            finally:
                done_event.set()

        client.call_async(request).add_done_callback(on_response)
        if not done_event.wait(float(timeout_sec)):
            return {
                "success": False,
                "message": f"AMCL {label}调用超时",
            }
        return result

    def deactivate_amcl_for_mapping(self):
        timeout_sec = float(self.config.get("amcl_lifecycle_timeout_sec", 5.0))
        state = self._call_lifecycle_get_state(timeout_sec)
        if not state.get("success"):
            return state

        state_id = state.get("state_id")
        if state_id == State.PRIMARY_STATE_INACTIVE:
            return {
                "success": True,
                "message": "AMCL 已处于 inactive，建图可继续",
                "state_before": state,
            }
        if state_id != State.PRIMARY_STATE_ACTIVE:
            return {
                "success": False,
                "message": f"AMCL 当前状态不能直接 deactivate: {state.get('state_label')}",
                "state_before": state,
            }

        result = self._change_amcl_state(
            Transition.TRANSITION_DEACTIVATE,
            "切换到 inactive",
            timeout_sec,
        )
        result["state_before"] = state
        return result

    def activate_amcl_after_mapping(self):
        timeout_sec = float(self.config.get("amcl_lifecycle_timeout_sec", 5.0))
        state = self._call_lifecycle_get_state(timeout_sec)
        if not state.get("success"):
            return state

        state_id = state.get("state_id")
        if state_id == State.PRIMARY_STATE_ACTIVE:
            return {
                "success": True,
                "message": "AMCL 已处于 active",
                "state_before": state,
            }
        if state_id != State.PRIMARY_STATE_INACTIVE:
            return {
                "success": False,
                "message": f"AMCL 当前状态不能直接 activate: {state.get('state_label')}",
                "state_before": state,
            }

        result = self._change_amcl_state(
            Transition.TRANSITION_ACTIVATE,
            "切换到 active",
            timeout_sec,
        )
        result["state_before"] = state
        return result

    def reload_nav2_map_server(self, yaml_path):
        if not self.started or not self.node:
            return {
                "success": False,
                "message": self.start_error or "ROS bridge 未启动",
                "yaml_path": yaml_path,
            }
        if LoadMap is None or self.node.map_server_load_client is None:
            return {
                "success": False,
                "message": f"LoadMap 服务接口未加载: {LOAD_MAP_IMPORT_ERROR}",
                "yaml_path": yaml_path,
            }
        if not yaml_path or not os.path.exists(yaml_path):
            return {
                "success": False,
                "message": f"Nav2 地图 YAML 不存在: {yaml_path}",
                "yaml_path": yaml_path,
            }

        timeout_sec = float(self.config.get("map_server_load_timeout_sec", 5.0))
        client = self.node.map_server_load_client
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {
                "success": False,
                "message": (
                    "Nav2 map_server load_map 服务不可用: "
                    f"{self.config['map_server_load_map_service']}"
                ),
                "yaml_path": yaml_path,
            }

        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        done_event = threading.Event()
        result = {}

        def on_response(done):
            try:
                response = done.result()
                success = response.result == LoadMap.Response.RESULT_SUCCESS
                result.update(
                    {
                        "success": success,
                        "message": (
                            "Nav2 map_server 已重新加载地图"
                            if success
                            else f"Nav2 map_server 重新加载地图失败: {response.result}"
                        ),
                        "result_code": int(response.result),
                        "yaml_path": str(yaml_path),
                    }
                )
            except Exception as exc:
                result.update(
                    {
                        "success": False,
                        "message": f"Nav2 map_server 重新加载地图异常: {exc}",
                        "yaml_path": str(yaml_path),
                    }
                )
            finally:
                done_event.set()

        client.call_async(request).add_done_callback(on_response)
        if not done_event.wait(timeout_sec):
            return {
                "success": False,
                "message": "Nav2 map_server 重新加载地图超时",
                "yaml_path": str(yaml_path),
            }
        return result

    def finish_mapping(self, save=False):
        if save:
            try:
                result = self.save_current_map()
            finally:
                self.stop_mapping_process()
                with self.lock:
                    self.mapping_active = False
            if not result.get("success"):
                amcl_result = self.activate_amcl_after_mapping()
                result["amcl_lifecycle"] = amcl_result
                result.setdefault("replace", {})["amcl_lifecycle"] = amcl_result
                if not amcl_result.get("success"):
                    result["message"] = (
                        f"{result.get('message', '地图保存失败')}；"
                        f"同时 AMCL 恢复失败: "
                        f"{amcl_result.get('message', 'unknown')}"
                    )
                return result
            if result.get("success"):
                map_load_result = self.load_static_map()
                result["map_load_after_stop"] = map_load_result
                yaml_path = (
                    map_load_result.get("yaml_path")
                    or result.get("replace", {}).get("yaml_path")
                    or self.map_file_paths(result.get("map_prefix", ""))["yaml"]
                )
                nav2_load_result = self.reload_nav2_map_server(yaml_path)
                result["nav2_map_load"] = nav2_load_result
                result.setdefault("replace", {})["nav2_map_load"] = nav2_load_result
                amcl_result = self.activate_amcl_after_mapping()
                result["amcl_lifecycle"] = amcl_result
                result.setdefault("replace", {})["amcl_lifecycle"] = amcl_result
                if nav2_load_result.get("success") and amcl_result.get("success"):
                    result["message"] = "地图已保存并重新加载到 Nav2，AMCL 已恢复"
                    with self.lock:
                        self.nav_status = {
                            "state": "mapping_saved",
                            "message": result["message"],
                            "updated_at": _now_iso(),
                        }
                else:
                    result["success"] = False
                    if not nav2_load_result.get("success"):
                        result["message"] = nav2_load_result.get(
                            "message",
                            "地图已保存，但 Nav2 map_server 重新加载失败",
                        )
                    else:
                        result["message"] = amcl_result.get(
                            "message",
                            "地图已保存，但 AMCL 恢复失败",
                        )
                    with self.lock:
                        self.nav_status = {
                            "state": "mapping_reload_failed",
                            "message": result["message"],
                            "updated_at": _now_iso(),
                        }
            return result

        self.stop_mapping_process()
        self.remove_temp_map_files()
        with self.lock:
            self.mapping_active = False
            self.accept_live_map = False
            self.map = None
            self.map_png = None
            self.map_version += 1
        load_result = self.load_static_map(self.startup_static_map_yaml)
        if not load_result.get("success"):
            with self.lock:
                self.nav_status = {
                    "state": "mapping_discard_failed",
                    "message": load_result.get("message", "恢复原保存地图失败"),
                    "updated_at": _now_iso(),
                }
            return {
                "success": False,
                "message": load_result.get("message", "恢复原保存地图失败"),
                "map_load": load_result,
            }
        with self.lock:
            self.nav_status = {
                "state": "mapping_discarded",
                "message": "已放弃本次建图，恢复使用原保存地图",
                "updated_at": _now_iso(),
            }
        amcl_result = self.activate_amcl_after_mapping()
        if not amcl_result.get("success"):
            with self.lock:
                self.nav_status = {
                    "state": "mapping_discard_failed",
                    "message": amcl_result.get("message", "AMCL 恢复失败"),
                    "updated_at": _now_iso(),
                }
            return {
                "success": False,
                "message": amcl_result.get("message", "AMCL 恢复失败"),
                "map_load": load_result,
                "amcl_lifecycle": amcl_result,
            }
        return {
            "success": True,
            "message": "已放弃本次建图，已从文件恢复原保存地图，AMCL 已恢复",
            "map_load": load_result,
            "amcl_lifecycle": amcl_result,
            "map": self.get_map_data().get("map"),
        }

    def map_file_paths(self, prefix):
        prefix = (prefix or "").strip()
        if prefix.endswith(".yaml"):
            prefix = prefix[:-5]
        elif prefix.endswith(".yml"):
            prefix = prefix[:-4]
        elif prefix.endswith(".pgm"):
            prefix = prefix[:-4]
        return {
            "yaml": f"{prefix}.yaml",
            "yml": f"{prefix}.yml",
            "pgm": f"{prefix}.pgm",
        }

    def temp_map_prefix(self):
        prefix = (self.config.get("map_save_prefix") or "").strip()
        configured = (self.config.get("map_temp_prefix") or "").strip()
        if configured:
            return configured
        if not prefix:
            return ""
        return f"{prefix}_new"

    def remove_temp_map_files(self):
        temp_prefix = self.temp_map_prefix()
        if not temp_prefix:
            return
        for path in self.map_file_paths(temp_prefix).values():
            try:
                os.remove(path)
            except FileNotFoundError:
                pass

    def start_mapping_process(self):
        command = (self.config.get("slam_rebuild_command") or "").strip()
        if not command:
            self.remove_temp_map_files()
            return {
                "success": True,
                "message": "已清空前端静态地图缓存，等待外部 SLAM 节点发布新的 /map",
            }

        try:
            self.stop_mapping_process()
            self.remove_temp_map_files()

            log_path = "/tmp/robot_frontend_slam_rebuild.log"
            log_file = open(log_path, "ab")
            self.mapping_process = subprocess.Popen(
                shlex.split(command),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            return {
                "success": True,
                "message": "已启动重新建图，前端不再使用原静态地图，等待 SLAM 从激光雷达生成新的 /map",
                "command": command,
            }
        except Exception as exc:
            return {
                "success": False,
                "message": f"重新建图命令启动失败: {exc}",
                "command": command,
            }

    def stop_mapping_process(self):
        if self.mapping_process and self.mapping_process.poll() is None:
            try:
                os.killpg(self.mapping_process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.mapping_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.mapping_process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.mapping_process.wait(timeout=1.0)
        self.mapping_process = None

    def save_current_map(self):
        prefix = (self.config.get("map_save_prefix") or "").strip()
        temp_prefix = self.temp_map_prefix()
        command_template = (self.config.get("map_save_command") or "").strip()
        if not prefix or not temp_prefix or not command_template:
            return {"success": False, "message": "地图保存命令未配置"}

        self.remove_temp_map_files()
        command = command_template.format(map_prefix=temp_prefix)
        try:
            completed = subprocess.run(
                shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=float(self.config.get("map_save_timeout_sec", 20.0)),
            )
        except subprocess.TimeoutExpired as exc:
            command_output = exc.stdout if exc.stdout is not None else exc.output
            if isinstance(command_output, bytes):
                command_output = command_output.decode(errors="replace")
            output_tail = (command_output or "").strip()[-2000:]
            message = (
                "地图保存命令超时，"
                f"超过 {float(self.config.get('map_save_timeout_sec', 20.0)):.1f}s"
            )
            if output_tail:
                message = f"{message}，输出: {output_tail[-800:]}"
            return {
                "success": False,
                "message": message,
                "command": command,
                "returncode": None,
                "output": output_tail,
            }
        except Exception as exc:
            return {
                "success": False,
                "message": f"地图保存失败: {exc}",
                "command": command,
            }

        command_output = (completed.stdout or "").strip()
        output_tail = command_output[-2000:]
        success = completed.returncode == 0
        replace_result = {}
        failure_message = None
        if not success:
            failure_message = f"地图保存命令失败，退出码={completed.returncode}"
            if output_tail:
                failure_message = f"{failure_message}，输出: {output_tail[-800:]}"
        if success:
            replace_result = self.commit_temp_map(temp_prefix, prefix)
            success = replace_result.get("success", False)
            if not success:
                failure_message = replace_result.get("message", "替换正式地图失败")
        if success:
            load_result = self.load_static_map(prefix, remember_startup=True)
            success = load_result.get("success", False)
            replace_result["map_load"] = load_result
            if not success:
                failure_message = load_result.get("message", "保存地图加载失败")
        with self.lock:
            self.nav_status = {
                "state": "mapping_saved" if success else "mapping_save_failed",
                "message": (
                    f"地图已保存到 {prefix}.yaml / {prefix}.pgm"
                    if success
                    else (
                        failure_message
                        or replace_result.get("map_load", {}).get("message")
                        or replace_result.get("message", "地图保存失败")
                    )
                ),
                "updated_at": _now_iso(),
            }
        return {
            "success": success,
            "message": "地图已保存" if success else (failure_message or "地图保存失败"),
            "command": command,
            "returncode": int(completed.returncode),
            "output": output_tail,
            "map_prefix": prefix,
            "temp_map_prefix": temp_prefix,
            "replace": replace_result,
        }

    def commit_temp_map(self, temp_prefix, final_prefix):
        temp_paths = self.map_file_paths(temp_prefix)
        final_paths = self.map_file_paths(final_prefix)
        temp_yaml = temp_paths["yaml"] if os.path.exists(temp_paths["yaml"]) else temp_paths["yml"]
        if not os.path.exists(temp_yaml):
            return {
                "success": False,
                "message": f"临时地图 YAML 不存在: {temp_paths['yaml']}",
            }
        if not os.path.exists(temp_paths["pgm"]):
            return {
                "success": False,
                "message": f"临时地图 PGM 不存在: {temp_paths['pgm']}",
            }

        try:
            final_dir = os.path.dirname(final_paths["yaml"])
            if final_dir:
                os.makedirs(final_dir, exist_ok=True)
            shutil.copyfile(temp_paths["pgm"], final_paths["pgm"])
            yaml_text = open(temp_yaml, "r", encoding="utf-8").read()
            final_image = os.path.basename(final_paths["pgm"])
            lines = []
            replaced_image = False
            for line in yaml_text.splitlines():
                if line.strip().startswith("image:"):
                    lines.append(f"image: {final_image}")
                    replaced_image = True
                else:
                    lines.append(line)
            if not replaced_image:
                lines.insert(0, f"image: {final_image}")
            with open(final_paths["yaml"], "w", encoding="utf-8") as yaml_file:
                yaml_file.write("\n".join(lines).rstrip() + "\n")
            self.remove_temp_map_files()
            return {
                "success": True,
                "message": "临时地图已替换正式地图",
                "yaml_path": final_paths["yaml"],
                "image_path": final_paths["pgm"],
            }
        except Exception as exc:
            return {
                "success": False,
                "message": f"替换正式地图失败: {exc}",
                "temp_prefix": temp_prefix,
                "map_prefix": final_prefix,
            }

    def send_navigation_goal(self, x, y, yaw=0.0):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }

        result = self.send_robot_task(
            RobotTask.Goal.TASK_NAVIGATE_TO,
            wait_result=False,
            x=x,
            y=y,
            yaw=yaw,
            frame_id=self.config["map_frame"],
            command="navigate_to",
        )

        with self.lock:
            self.target_point = {"x": float(x), "y": float(y), "theta": float(yaw)}
        return result

    def send_navigation_path(self, waypoints):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        parsed = []
        for waypoint in waypoints:
            try:
                raw_theta = waypoint.get("theta")
                parsed.append(
                    {
                        "x": float(waypoint["x"]),
                        "y": float(waypoint["y"]),
                        "theta": float(raw_theta) if raw_theta is not None else None,
                    }
                )
            except (KeyError, TypeError, ValueError):
                return {"success": False, "error": "路径点坐标无效"}

        if not parsed:
            return {"success": False, "error": "路径至少需要 1 个目标点"}

        for index, point in enumerate(parsed):
            if point["theta"] is not None:
                continue
            if index < len(parsed) - 1:
                next_point = parsed[index + 1]
                point["theta"] = math.atan2(next_point["y"] - point["y"], next_point["x"] - point["x"])
            elif len(parsed) > 1:
                prev_point = parsed[index - 1]
                point["theta"] = math.atan2(point["y"] - prev_point["y"], point["x"] - prev_point["x"])
            else:
                point["theta"] = 0.0

        via_count = max(0, len(parsed) - 1)
        final_goal = parsed[-1]
        path_summary = (
            f"{via_count} 个途经点，最终目标点 "
            f"({final_goal['x']:.3f}, {final_goal['y']:.3f})"
        )

        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }

        path = RosPath()
        path.header.frame_id = self.config["map_frame"]
        path.header.stamp = self.node.get_clock().now().to_msg()
        for waypoint in parsed:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(waypoint["x"])
            pose.pose.position.y = float(waypoint["y"])
            pose.pose.position.z = 0.0
            quat = _quaternion_from_yaw(float(waypoint.get("theta", 0.0)))
            pose.pose.orientation.x = quat["x"]
            pose.pose.orientation.y = quat["y"]
            pose.pose.orientation.z = quat["z"]
            pose.pose.orientation.w = quat["w"]
            path.poses.append(pose)

        result = self.send_robot_task(
            RobotTask.Goal.TASK_NAVIGATE_PATH,
            wait_result=False,
            path=path,
            command="navigate_path",
        )
        if result.get("success"):
            with self.lock:
                self.nav_status = {
                    "state": "accepted",
                    "message": f"路径已提交行为树：{path_summary}",
                    "updated_at": _now_iso(),
                }

        with self.lock:
            self.target_point = dict(parsed[-1])
        return {
            **result,
            "waypoint_count": len(parsed),
            "via_count": via_count,
            "target_point": dict(parsed[-1]),
        }

    def send_robot_task(self, task_type, wait_result=False, **kwargs):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        if RobotTask is None or self.node.robot_task_client is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }

        service_timeout = float(self.config.get("goal_timeout_sec", 3.0))
        if not self.node.robot_task_client.wait_for_server(timeout_sec=service_timeout):
            return {
                "success": False,
                "error": f"行为树 Action 不可用: {self.config['robot_task_action']}",
            }

        response_event = threading.Event()
        result = {}

        def update_status(state, message):
            with self.lock:
                self.robot_task_status = {
                    "state": state,
                    "message": message,
                    "updated_at": _now_iso(),
                }

        def on_feedback(feedback_msg):
            feedback = feedback_msg.feedback
            message = str(feedback.state)
            self._handle_bucket_fill_feedback(message)
            with self.lock:
                self.robot_task_status = {
                    "state": "running",
                    "message": message,
                    "progress": float(feedback.progress),
                    "updated_at": _now_iso(),
                }

        def on_task_result(done):
            try:
                wrapped = done.result()
                task_result = wrapped.result
                success = bool(task_result.success)
                message = task_result.message
                result.update(
                    {
                        "success": success,
                        "message": message,
                        "accepted": True,
                        "task_type": int(task_type),
                    }
                )
                update_status("succeeded" if success else "failed", message)
            except Exception as exc:
                result.update({"success": False, "error": str(exc)})
                update_status("error", str(exc))
            finally:
                response_event.set()

        def on_goal_response(done):
            try:
                goal_handle = done.result()
                if not goal_handle.accepted:
                    result.update(
                        {
                            "success": False,
                            "accepted": False,
                            "error": "行为树拒绝任务",
                            "task_type": int(task_type),
                        }
                    )
                    update_status("rejected", "行为树拒绝任务")
                    response_event.set()
                    return

                result.update(
                    {
                        "success": True,
                        "accepted": True,
                        "pending": not wait_result,
                        "task_type": int(task_type),
                    }
                )
                update_status("accepted", f"行为树任务已接受: {task_type}")
                result_future = goal_handle.get_result_async()
                if wait_result:
                    result_future.add_done_callback(on_task_result)
                else:
                    result_future.add_done_callback(self._handle_robot_task_result)
                    response_event.set()
            except Exception as exc:
                result.update({"success": False, "error": str(exc)})
                update_status("error", str(exc))
                response_event.set()

        future = self.node.send_robot_task_goal(
            task_type,
            feedback_callback=on_feedback,
            **kwargs,
        )
        future.add_done_callback(on_goal_response)

        timeout = (
            float(self.config.get("task_timeout_sec", 90.0))
            if wait_result
            else service_timeout
        )
        if not response_event.wait(timeout):
            if wait_result:
                return {
                    "success": False,
                    "error": "行为树任务响应超时",
                    "task_type": int(task_type),
                }
            return {
                "success": True,
                "accepted": None,
                "pending": True,
                "task_type": int(task_type),
            }
        return result

    def add_coverage_rectangle(self, min_x, min_y, max_x, max_y, frame_id=None):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        if min_x >= max_x or min_y >= max_y:
            return {"success": False, "error": "区域宽度和高度必须大于 0"}

        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }

        return self.send_robot_task(
            RobotTask.Goal.TASK_CLEAN_REGION,
            wait_result=True,
            min_x=min_x,
            min_y=min_y,
            max_x=max_x,
            max_y=max_y,
            frame_id=frame_id or self.config["map_frame"],
            command="clean_region",
        )

    def set_clean_region(self, region_id, min_x, min_y, max_x, max_y, frame_id=None):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        try:
            region_id = int(region_id)
            min_x = float(min_x)
            min_y = float(min_y)
            max_x = float(max_x)
            max_y = float(max_y)
        except (TypeError, ValueError):
            return {"success": False, "error": "区域坐标无效"}

        if region_id <= 0:
            return {"success": False, "error": "区域编号必须大于 0"}
        if min_x >= max_x or min_y >= max_y:
            return {"success": False, "error": "区域宽度和高度必须大于 0"}
        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }

        return self.send_robot_task(
            RobotTask.Goal.TASK_SET_CLEAN_REGION,
            wait_result=True,
            bucket_id=region_id,
            min_x=min_x,
            min_y=min_y,
            max_x=max_x,
            max_y=max_y,
            frame_id=frame_id or self.config["map_frame"],
            command="set_clean_region",
        )

    def set_dump_bucket_pose(self, bucket_id, x, y, yaw=0.0, frame_id=None):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        try:
            bucket_id = int(bucket_id)
            x = float(x)
            y = float(y)
            yaw = float(yaw)
        except (TypeError, ValueError):
            return {"success": False, "error": "卸载桶坐标无效"}

        if bucket_id not in (1, 2):
            return {"success": False, "error": "卸载桶编号必须是 1 或 2"}

        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
                "bucket_id": bucket_id,
            }

        result = self.send_robot_task(
            RobotTask.Goal.TASK_SET_DUMP_POSE,
            wait_result=True,
            bucket_id=bucket_id,
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id or self.config["map_frame"],
            command="set_dump_pose",
        )
        result.setdefault("bucket_id", bucket_id)
        result.setdefault(
            "pose",
            {
                "bucket_id": bucket_id,
                "x": x,
                "y": y,
                "theta": yaw,
                "yaw": yaw,
                "frame_id": frame_id or self.config["map_frame"],
            },
        )
        return result

    def dump_trash(self, bucket_id):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}

        try:
            bucket_id = int(bucket_id)
        except (TypeError, ValueError):
            return {"success": False, "error": "垃圾桶编号无效"}

        if bucket_id not in (0, 1, 2):
            return {"success": False, "error": "垃圾桶编号必须是 0、1 或 2"}

        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
                "bucket_id": bucket_id,
            }

        result = self.send_robot_task(
            RobotTask.Goal.TASK_DUMP_TRASH,
            wait_result=True,
            bucket_id=bucket_id,
            command="dump_trash",
        )
        result.setdefault("bucket_id", bucket_id)
        if result.get("success"):
            bucket_ids = (1, 2) if bucket_id == 0 else (bucket_id,)
            for item in bucket_ids:
                self.reset_bucket_fill_level(item)
        return result

    @staticmethod
    def _clamp_bucket_percent(value):
        return int(max(0, min(100, round(float(value)))))

    def get_bucket_fill_levels(self):
        with self.lock:
            left = self._clamp_bucket_percent(self.bucket_fill_levels.get(1, 0))
            right = self._clamp_bucket_percent(self.bucket_fill_levels.get(2, 0))
        return {
            "left": left,
            "right": right,
            "bucket_1": left,
            "bucket_2": right,
        }

    def reset_bucket_fill_level(self, bucket_id):
        try:
            bucket_id = int(bucket_id)
        except (TypeError, ValueError):
            return
        if bucket_id not in (1, 2):
            return
        with self.lock:
            self.bucket_fill_levels[bucket_id] = 0

    def _handle_bucket_fill_feedback(self, message):
        text = str(message or "")
        if not text.startswith("bucket_fill:"):
            return

        values = {}
        for token in text.replace(":", " ").split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            values[key.strip()] = value.strip()

        try:
            bucket_id = int(values.get("bucket_id", 0))
            level = self._clamp_bucket_percent(values["level"])
        except (KeyError, TypeError, ValueError):
            return

        if bucket_id not in (1, 2):
            return
        with self.lock:
            self.bucket_fill_levels[bucket_id] = level

    def _handle_robot_task_result(self, done):
        try:
            wrapped = done.result()
            task_result = wrapped.result
            success = bool(task_result.success)
            state = "succeeded" if success else "failed"
            message = task_result.message
        except Exception as exc:
            state = "error"
            message = str(exc)

        with self.lock:
            self.robot_task_status = {
                "state": state,
                "message": message,
                "updated_at": _now_iso(),
            }

    def publish_stop(self):
        if self.started and self.node:
            self.node.publish_stop()

    def clear_robot_tasks(self, command="standby"):
        self.publish_stop()
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}
        if RobotTask is None:
            return {
                "success": False,
                "error": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }
        return self.send_robot_task(
            RobotTask.Goal.TASK_STOP,
            wait_result=False,
            command=command,
        )

    def publish_velocity(self, linear_x, angular_z, command_session=None, command_seq=None):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}
        parsed_command_seq = None
        if command_seq is not None:
            try:
                parsed_command_seq = int(command_seq)
            except (TypeError, ValueError):
                return {"success": False, "error": "速度命令序号无效"}
            command_session = str(command_session or "default")
            with self.lock:
                if command_session != self.latest_velocity_command_session:
                    self.latest_velocity_command_session = command_session
                    self.latest_velocity_command_seq = -1
                if parsed_command_seq < self.latest_velocity_command_seq:
                    return {
                        "success": True,
                        "ignored": True,
                        "command_session": command_session,
                        "command_seq": parsed_command_seq,
                        "latest_command_seq": self.latest_velocity_command_seq,
                    }
                self.latest_velocity_command_seq = parsed_command_seq
        self.node.publish_velocity(linear_x, angular_z)
        result = {
            "success": True,
            "linear_x": float(linear_x),
            "angular_z": float(angular_z),
        }
        if parsed_command_seq is not None:
            result["command_session"] = command_session
            result["command_seq"] = parsed_command_seq
        return result

    def call_grasp_service(self, action):
        if not self.started or not self.node:
            return {"success": False, "message": self.start_error or "ROS bridge not started"}
        if RobotTask is None:
            return {
                "success": False,
                "message": f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}",
            }
        if action == "start":
            task_result = self.send_robot_task(
                RobotTask.Goal.TASK_CLEAN_TRASH,
                wait_result=False,
                command="clean_trash",
            )
            return {
                "success": task_result.get("success", False),
                "message": task_result.get("message", "行为树清扫任务已发送"),
            }
        if action == "stop":
            self.publish_stop()
            task_result = self.send_robot_task(
                RobotTask.Goal.TASK_STOP,
                wait_result=False,
                command="standby",
            )
            return {
                "success": task_result.get("success", False),
                "message": task_result.get("message", "行为树停止任务已发送"),
            }
        return {"success": False, "message": f"unknown grasp action: {action}"}

    def status(self):
        with self.lock:
            return {
                "enabled": self.config.get("enabled", True),
                "started": self.started,
                "start_error": self.start_error,
                "topics": {
                    "depth_camera": self.config["depth_camera_topic"],
                    "depth_camera_candidates": self.config["depth_camera_topics"],
                    "color_camera": self.config["color_camera_topic"],
                    "color_camera_candidates": self.config["color_camera_topics"],
                    "map": self.config["map_topic"],
                    "amcl_pose": self.config["amcl_pose_topic"],
                    "scan": self.config["scan_topic"],
                    "robot_task_action": self.config["robot_task_action"],
                    "arm_preset_action": self.config["arm_preset_action"],
                    "camera_tf_preset_service": self.config[
                        "camera_tf_preset_service"
                    ],
                },
                "camera": self.get_camera_info(),
                "has_map": self.map is not None,
                "has_robot_pose": self.robot_pose is not None,
                "robot_task_interface_loaded": RobotTask is not None,
                "robot_task_interface_error": str(ROBOT_TASK_IMPORT_ERROR) if ROBOT_TASK_IMPORT_ERROR else None,
                "arm_preset_interface_loaded": ArmPreset is not None,
                "arm_preset_interface_error": str(ARM_PRESET_IMPORT_ERROR) if ARM_PRESET_IMPORT_ERROR else None,
                "camera_tf_preset_interface_loaded": SetCameraTfPreset is not None,
                "camera_tf_preset_interface_error": str(CAMERA_TF_PRESET_IMPORT_ERROR) if CAMERA_TF_PRESET_IMPORT_ERROR else None,
                "nav_status": dict(self.nav_status),
                "robot_task_status": dict(self.robot_task_status),
                "errors": dict(self.errors),
            }


def placeholder_jpeg(label):
    frame = np.zeros((480, 720, 3), dtype=np.uint8)
    frame[:, :] = (32, 38, 36)
    cv2.rectangle(frame, (20, 20), (700, 460), (90, 103, 99), 2)
    cv2.putText(frame, label, (52, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.86, (240, 246, 242), 2)
    cv2.putText(frame, "Waiting for ROS image topic", (52, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (180, 194, 188), 2)
    ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return buffer.tobytes() if ok else b""


def placeholder_png(label):
    frame = np.zeros((600, 900, 3), dtype=np.uint8)
    frame[:, :] = (238, 242, 245)
    cv2.rectangle(frame, (24, 24), (876, 576), (180, 190, 200), 2)
    cv2.putText(frame, label, (54, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.92, (70, 78, 88), 2)
    cv2.putText(frame, "Waiting for /map OccupancyGrid", (54, 326), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (105, 116, 128), 2)
    ok, buffer = cv2.imencode(".png", frame)
    return buffer.tobytes() if ok else b""
