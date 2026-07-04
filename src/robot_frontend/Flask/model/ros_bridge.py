import math
import os
import signal
import shlex
import subprocess
import threading
import time

import cv2
import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path
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
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None

try:
    from robot_interfaces.action import RobotTask
except Exception as exc:
    RobotTask = None
    ROBOT_TASK_IMPORT_ERROR = exc
else:
    ROBOT_TASK_IMPORT_ERROR = None

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
        self.cmd_vel_pub = self.create_publisher(Twist, config["cmd_vel_topic"], 10)

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
            self.bridge.set_camera_frame("depth", jpeg, msg.header.frame_id, _stamp_to_float(msg.header.stamp), topic)
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
                self.bridge.set_camera_frame("color", buffer.tobytes(), msg.header.frame_id, _stamp_to_float(msg.header.stamp), topic)
        except Exception as exc:
            self.bridge.set_error("color_camera", str(exc))

    def handle_map(self, msg):
        try:
            self.bridge.set_map(msg)
        except Exception as exc:
            self.bridge.set_error("map", str(exc))

    def handle_amcl_pose(self, msg):
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
        self.cmd_vel_pub.publish(Twist())

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def send_robot_task_goal(self, task_type, **kwargs):
        if RobotTask is None or self.robot_task_client is None:
            raise RuntimeError(f"RobotTask 接口未加载: {ROBOT_TASK_IMPORT_ERROR}")
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
        return self.robot_task_client.send_goal_async(goal)


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
            "depth": {"jpeg": None, "frame_id": None, "stamp": None, "received_at": None, "topic": None},
            "color": {"jpeg": None, "frame_id": None, "stamp": None, "received_at": None, "topic": None},
        }
        self.map = None
        self.map_png = None
        self.map_version = 0
        self.robot_pose = None
        self.last_scan = None
        self.target_point = None
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
        self.errors = {}
        self.load_static_map()

    def load_static_map(self):
        yaml_path = self.config.get("static_map_yaml")
        if not yaml_path or not os.path.exists(yaml_path):
            return

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
                return
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                return

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
                return

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
        except Exception as exc:
            self.set_error("static_map", str(exc))

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

    def set_camera_frame(self, camera_id, jpeg, frame_id, stamp, topic):
        with self.lock:
            self.frames[camera_id] = {
                "jpeg": jpeg,
                "frame_id": frame_id,
                "stamp": stamp,
                "received_at": time.time(),
                "topic": topic,
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
            self.robot_pose = {
                "x": float(x),
                "y": float(y),
                "theta": float(theta),
                "theta_deg": math.degrees(float(theta)),
                "source": source,
                "stamp": stamp,
                "received_at": time.time(),
            }

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
        process_result = self.start_mapping_process()
        with self.lock:
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
        }

    def finish_mapping(self, save=False):
        if save:
            try:
                result = self.save_current_map()
            finally:
                self.stop_mapping_process()
            if result.get("success"):
                self.load_static_map()
            return result

        self.stop_mapping_process()
        self.load_static_map()
        with self.lock:
            self.nav_status = {
                "state": "mapping_discarded",
                "message": "已放弃本次建图，恢复使用原保存地图",
                "updated_at": _now_iso(),
            }
        return {"success": True, "message": "已放弃本次建图，未保存新地图"}

    def start_mapping_process(self):
        command = (self.config.get("slam_rebuild_command") or "").strip()
        if not command:
            return {
                "success": True,
                "message": "已清空前端静态地图缓存，等待外部 SLAM 节点发布新的 /map",
            }

        try:
            self.stop_mapping_process()

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
        command_template = (self.config.get("map_save_command") or "").strip()
        if not prefix or not command_template:
            return {"success": False, "message": "地图保存命令未配置"}

        command = command_template.format(map_prefix=prefix)
        try:
            completed = subprocess.run(
                shlex.split(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=float(self.config.get("map_save_timeout_sec", 20.0)),
            )
        except Exception as exc:
            return {
                "success": False,
                "message": f"地图保存失败: {exc}",
                "command": command,
            }

        success = completed.returncode == 0
        if success:
            with self.lock:
                self.nav_status = {
                    "state": "mapping_saved",
                    "message": f"地图已保存到 {prefix}.yaml / {prefix}.pgm",
                    "updated_at": _now_iso(),
                }
        return {
            "success": success,
            "message": "地图已保存" if success else "地图保存失败",
            "command": command,
            "output": (completed.stdout or "")[-2000:],
            "map_prefix": prefix,
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

        path = Path()
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

        future = self.node.send_robot_task_goal(task_type, **kwargs)
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
        return result

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

    def publish_velocity(self, linear_x, angular_z):
        if not self.started or not self.node:
            return {"success": False, "error": self.start_error or "ROS bridge 未启动"}
        self.node.publish_velocity(linear_x, angular_z)
        return {
            "success": True,
            "linear_x": float(linear_x),
            "angular_z": float(angular_z),
        }

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
                command="stop",
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
                },
                "camera": self.get_camera_info(),
                "has_map": self.map is not None,
                "has_robot_pose": self.robot_pose is not None,
                "robot_task_interface_loaded": RobotTask is not None,
                "robot_task_interface_error": str(ROBOT_TASK_IMPORT_ERROR) if ROBOT_TASK_IMPORT_ERROR else None,
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
