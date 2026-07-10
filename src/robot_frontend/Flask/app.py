from datetime import datetime, timedelta
import atexit
import json
import math
from pathlib import Path
import time

from flask import Flask, Response, has_request_context, jsonify, render_template, request
from flask_cors import CORS

from config.config import CAMERA_CONFIG, DEBUG, MAP_CONFIG, ROBOT_CONFIG, ROS_CONFIG, SECRET_KEY, VOICE_CONFIG
from model.database import Database, init_database
from model.ros_bridge import RosBridge, placeholder_jpeg, placeholder_png


app = Flask(__name__)
app.config["SECRET_KEY"] = SECRET_KEY
CORS(app)


START_TIME = datetime.now()
ROS_BRIDGE = RosBridge(ROS_CONFIG)
atexit.register(ROS_BRIDGE.shutdown)
DATABASE_READY = False
MEMORY_LOGS = []
REGIONS = []
DUMP_BUCKET_POSE_STORE = Path(ROS_CONFIG["dump_pose_store_path"])
DUMP_BUCKET_NAV_OFFSET_M = 0.45


def load_dump_bucket_poses():
    if not DUMP_BUCKET_POSE_STORE.exists():
        return {}
    try:
        raw = json.loads(DUMP_BUCKET_POSE_STORE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    poses = raw.get("dump_bucket_poses", raw)
    loaded = {}
    for key, item in poses.items():
        try:
            bucket_id = int(item.get("bucket_id", key))
            if bucket_id not in (1, 2):
                continue
            yaw = float(item.get("yaw", item.get("theta", 0.0)))
            x = float(item["x"])
            y = float(item["y"])
            loaded[str(bucket_id)] = {
                "bucket_id": bucket_id,
                "x": x,
                "y": y,
                "theta": yaw,
                "yaw": yaw,
                "frame_id": item.get("frame_id") or ROS_CONFIG["map_frame"],
                "bucket_x": float(item.get("bucket_x", x)),
                "bucket_y": float(item.get("bucket_y", y)),
                "bucket_theta": float(item.get("bucket_theta", yaw)),
                "nav_offset_m": float(
                    item.get("nav_offset_m", DUMP_BUCKET_NAV_OFFSET_M)
                ),
            }
        except (KeyError, TypeError, ValueError):
            continue
    return loaded


def save_dump_bucket_poses():
    DUMP_BUCKET_POSE_STORE.parent.mkdir(parents=True, exist_ok=True)
    payload = {"dump_bucket_poses": DUMP_BUCKET_POSES}
    DUMP_BUCKET_POSE_STORE.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


DUMP_BUCKET_POSES = load_dump_bucket_poses()


def client_ip():
    forwarded = request.headers.get("X-Forwarded-For", "")
    if forwarded:
        return forwarded.split(",")[0].strip()
    return request.remote_addr


def add_log(action_type, description, status="完成", success=True, metadata=None, user_id="web_user"):
    global MEMORY_LOGS
    ip_address = client_ip() if has_request_context() else None
    if Database.available():
        saved = Database.log_operation(
            action_type=action_type,
            action_details=description,
            user_id=user_id,
            ip_address=ip_address,
            status=status,
            success=success,
            metadata=metadata,
        )
        if saved:
            return

    MEMORY_LOGS.append(
        {
            "id": len(MEMORY_LOGS) + 1,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "user_id": user_id,
            "action_type": action_type,
            "action_details": str(description),
            "status": status,
            "success": bool(success),
            "ip_address": ip_address,
            "metadata": metadata,
        }
    )
    MEMORY_LOGS = MEMORY_LOGS[-1000:]


def memory_logs_page(page=1, page_size=20, keyword=None, action_type=None):
    page = max(1, int(page or 1))
    page_size = min(100, max(1, int(page_size or 20)))
    rows = list(reversed(MEMORY_LOGS))
    if action_type:
        rows = [row for row in rows if row["action_type"] == action_type]
    if keyword:
        keyword_lower = keyword.lower()
        rows = [
            row for row in rows
            if keyword_lower in row["action_type"].lower()
            or keyword_lower in row["action_details"].lower()
            or keyword_lower in row["user_id"].lower()
            or keyword_lower in row["status"].lower()
        ]

    total = len(rows)
    start = (page - 1) * page_size
    end = start + page_size
    return {
        "items": rows[start:end],
        "total": total,
        "page": page,
        "page_size": page_size,
        "pages": max(1, (total + page_size - 1) // page_size),
        "storage": "memory",
    }


def submit_region_clean(region_id):
    region = next((item for item in REGIONS if item["region_id"] == region_id), None)
    if not region:
        return {"success": False, "error": f"未找到区域 {region_id}"}, 404

    bounds = region.get("bounds") or {}
    try:
        min_x = float(bounds["min_x"])
        min_y = float(bounds["min_y"])
        max_x = float(bounds["max_x"])
        max_y = float(bounds["max_y"])
    except (KeyError, TypeError, ValueError):
        return {"success": False, "error": "区域坐标无效，请重新框选"}, 400

    result = ROS_BRIDGE.add_coverage_rectangle(
        min_x,
        min_y,
        max_x,
        max_y,
        region.get("frame_id") or ROS_CONFIG["map_frame"],
    )
    success = bool(result.get("success"))
    add_log(
        "区域清扫",
        f"提交清扫区域 ID：{region_id}",
        "进行中" if success else "失败",
        success=success,
        metadata={"region": region, "service_result": result},
    )
    status_code = 200 if success else 503
    return {"region_id": region_id, **result}, status_code


def submit_dump_bucket(bucket_id):
    try:
        bucket_id = int(bucket_id)
    except (TypeError, ValueError):
        return {"success": False, "error": "bucket_id 必须是 0、1 或 2"}, 400

    if bucket_id not in (0, 1, 2):
        return {"success": False, "error": "bucket_id 必须是 0、1 或 2"}, 400

    required = (1, 2) if bucket_id == 0 else (bucket_id,)
    missing = [item for item in required if str(item) not in DUMP_BUCKET_POSES]
    if missing:
        names = "、".join(f"卸载桶{item}" for item in missing)
        return {"success": False, "error": f"请先在导航地图设定{names}位置"}, 400

    result = ROS_BRIDGE.dump_trash(bucket_id)
    success = bool(result.get("success"))
    if success:
        for item in required:
            ROS_BRIDGE.reset_bucket_fill_level(item)
        result["bins"] = RuntimeData.bin_levels()
    description = "执行卸载桶 1 和 2" if bucket_id == 0 else f"执行卸载桶 {bucket_id}"
    add_log(
        "卸载垃圾桶",
        description,
        status="完成" if success else "失败",
        success=success,
        metadata={
            "bucket_id": bucket_id,
            "poses": DUMP_BUCKET_POSES,
            "service_result": result,
        },
    )
    return result, 200 if success else 503


def dump_bucket_nav_goal_from_bucket_pose(bucket_id, x, y, yaw, frame_id):
    nav_x = x - math.cos(yaw) * DUMP_BUCKET_NAV_OFFSET_M
    nav_y = y - math.sin(yaw) * DUMP_BUCKET_NAV_OFFSET_M
    return {
        "bucket_id": bucket_id,
        "x": nav_x,
        "y": nav_y,
        "theta": yaw,
        "yaw": yaw,
        "frame_id": frame_id,
        "bucket_x": x,
        "bucket_y": y,
        "bucket_theta": yaw,
        "nav_offset_m": DUMP_BUCKET_NAV_OFFSET_M,
    }


class RuntimeData:
    @staticmethod
    def battery_level():
        elapsed_minutes = max(0, int((datetime.now() - START_TIME).total_seconds() // 60))
        return max(0, 100 - elapsed_minutes // 5)

    @staticmethod
    def bin_levels():
        return ROS_BRIDGE.get_bucket_fill_levels()

    @staticmethod
    def robot_status():
        map_data = ROS_BRIDGE.get_map_data()
        ros_status = ROS_BRIDGE.status()
        nav_state = map_data["nav_status"].get("state")
        status_map = {
            "idle": "待机",
            "pending": "导航发送中",
            "accepted": "导航中",
            "succeeded": "导航完成",
            "finished": "导航结束",
            "rejected": "目标拒绝",
            "error": "异常",
        }
        return {
            "status": status_map.get(nav_state, "待机"),
            "battery_level": RuntimeData.battery_level(),
            "bins": RuntimeData.bin_levels(),
            "position": map_data.get("robot_position"),
            "target_point": map_data.get("target_point"),
            "nav_status": map_data.get("nav_status"),
            "robot_task_status": ros_status.get("robot_task_status", {}),
            "visual_work_enabled": bool(
                ros_status.get("visual_work_enabled", False)
            ),
            "last_update": datetime.now().isoformat(),
        }

    @staticmethod
    def command_history():
        return [
            {
                "timestamp": (datetime.now() - timedelta(minutes=8)).isoformat(),
                "text": "查看当前状态",
                "type": "text",
                "confidence": None,
                "status": "completed",
            }
        ]


@app.context_processor
def inject_now():
    return {"now": datetime.now()}


@app.route("/")
def index():
    return render_template(
        "index.html",
        robot_status=RuntimeData.robot_status(),
        robot_config=ROBOT_CONFIG,
    )


@app.route("/map")
def map_page():
    return render_template(
        "map.html",
        map_data=ROS_BRIDGE.get_map_data(),
        regions=REGIONS,
        dump_bucket_poses=DUMP_BUCKET_POSES,
        map_config=MAP_CONFIG,
        robot_config=ROBOT_CONFIG,
        ros_config=ROS_CONFIG,
    )


@app.route("/mapping")
def mapping_page():
    return render_template(
        "mapping.html",
        map_data=ROS_BRIDGE.get_map_data(),
        map_config=MAP_CONFIG,
        robot_config=ROBOT_CONFIG,
        ros_config=ROS_CONFIG,
    )


@app.route("/monitor")
def monitor_page():
    return render_template(
        "monitor.html",
        camera_config=CAMERA_CONFIG,
        robot_config=ROBOT_CONFIG,
        ros_config=ROS_CONFIG,
    )


@app.route("/logs")
def logs_page():
    return render_template("logs.html", robot_config=ROBOT_CONFIG)


@app.route("/api/robot/status", methods=["GET"])
def get_robot_status():
    return jsonify(RuntimeData.robot_status())


@app.route("/api/robot/control", methods=["POST"])
def control_robot():
    data = request.get_json(silent=True) or {}
    command_type = data.get("type")
    labels = {
        "standby": "待机",
        "start": "启动",
        "grasp_start": "启动清扫工作",
        "grasp_stop": "停止清扫工作",
    }
    if command_type not in labels:
        return jsonify({"success": False, "error": "不支持的控制命令"}), 400

    if command_type == "standby":
        ROS_BRIDGE.clear_robot_tasks("standby")

    if command_type in ("grasp_start", "grasp_stop"):
        action = "start" if command_type == "grasp_start" else "stop"
        result = ROS_BRIDGE.call_grasp_service(action)
        return jsonify({
            "success": result.get("success", False),
            "message": result.get("message", ""),
            "command_type": command_type,
            "status": RuntimeData.robot_status(),
        })

    label = labels[command_type]
    add_log(
        "机器人控制",
        f"执行命令：{label}",
        status="完成",
        success=True,
        metadata={"command_type": command_type},
    )
    return jsonify(
        {
            "success": True,
            "message": f"{label}命令已记录",
            "command_type": command_type,
            "status": RuntimeData.robot_status(),
        }
    )


@app.route("/api/robot/dump_bucket", methods=["POST"])
def dump_bucket():
    data = request.get_json(silent=True) or {}
    result, status_code = submit_dump_bucket(data.get("bucket_id"))
    return jsonify(result), status_code


@app.route("/api/robot/text_command", methods=["POST"])
def text_command():
    data = request.get_json(silent=True) or {}
    command_text = data.get("text", "").strip()
    user_id = data.get("user_id") or "web_user"
    if not command_text:
        return jsonify({"success": False, "error": "缺少指令文本"}), 400

    command_type = "unknown"
    command_data = {}
    if "待机" in command_text or "停止" in command_text:
        command_type = "standby"
        ROS_BRIDGE.clear_robot_tasks("standby")
    elif "启动" in command_text or "开始" in command_text:
        command_type = "start"
    elif "卸载" in command_text or "倒垃圾" in command_text:
        command_type = "unload"
    elif "清扫" in command_text or "打扫" in command_text:
        command_type = "clean_region"
        command_data = {"region_id": 1}
    elif "回家" in command_text or "返回" in command_text:
        command_type = "go_home"

    add_log(
        "文本指令",
        f"收到文本指令：{command_text}",
        user_id=user_id,
        metadata={"command_type": command_type, "command_data": command_data},
    )
    if Database.available():
        Database.execute_query(
            """
            INSERT INTO text_commands (command_text, user_id, status, response)
            VALUES (%s, %s, %s, %s)
            """,
            (command_text, user_id, "completed", command_type),
        )
    response = {
        "success": True,
        "command_type": command_type,
        "command_data": command_data,
        "timestamp": datetime.now().isoformat(),
    }
    if command_type == "clean_region":
        clean_result, _ = submit_region_clean(command_data["region_id"])
        response["clean_result"] = clean_result
        response["success"] = bool(clean_result.get("success"))
    elif command_type == "unload":
        dump_result, _ = submit_dump_bucket(0)
        response["dump_result"] = dump_result
        response["success"] = bool(dump_result.get("success"))
    return jsonify(response)


@app.route("/api/robot/voice_command", methods=["POST"])
def voice_command():
    data = request.get_json(silent=True) or {}
    command_text = data.get("text", "").strip()
    confidence = data.get("confidence")
    if not command_text:
        return jsonify({"success": False, "error": "语音文本为空"}), 400

    if "卸载" in command_text or "倒垃圾" in command_text:
        command_type = "unload"
        command_data = {}
    elif "清扫" in command_text or "打扫" in command_text:
        command_type = "clean_region"
        command_data = {"region_id": 1}
    else:
        command_type = "start"
        command_data = {}
    add_log(
        "语音指令",
        f"收到语音指令：{command_text}",
        metadata={"command_type": command_type, "confidence": confidence},
    )
    if Database.available():
        Database.execute_query(
            """
            INSERT INTO voice_commands
            (command_text, recognized_text, confidence, status, response)
            VALUES (%s, %s, %s, %s, %s)
            """,
            (command_text, command_text, confidence, "completed", command_type),
        )
    response = {
        "success": True,
        "recognized_text": command_text,
        "command_type": command_type,
        "command_data": command_data,
    }
    if command_type == "clean_region":
        clean_result, _ = submit_region_clean(command_data["region_id"])
        response["clean_result"] = clean_result
        response["success"] = bool(clean_result.get("success"))
    elif command_type == "unload":
        dump_result, _ = submit_dump_bucket(0)
        response["dump_result"] = dump_result
        response["success"] = bool(dump_result.get("success"))
    return jsonify(response)


@app.route("/api/map/data", methods=["GET"])
def map_data():
    data = ROS_BRIDGE.get_map_data()
    data["dump_bucket_poses"] = DUMP_BUCKET_POSES
    return jsonify(data)


@app.route("/api/robot/pose", methods=["GET"])
def robot_pose():
    return jsonify(ROS_BRIDGE.get_robot_pose_data())


@app.route("/api/robot/initial_pose", methods=["POST"])
def set_robot_initial_pose():
    data = request.get_json(silent=True) or {}
    try:
        x = float(data.get("x"))
        y = float(data.get("y"))
        yaw = float(data.get("yaw", data.get("theta", 0.0)))
    except (TypeError, ValueError):
        return jsonify({"success": False, "error": "机器人位姿坐标无效"}), 400

    frame_id = data.get("frame_id") or ROS_CONFIG["map_frame"]
    result = ROS_BRIDGE.publish_initial_pose(x, y, yaw, frame_id=frame_id)
    success = bool(result.get("success"))
    add_log(
        "设定机器人位姿",
        f"发布机器人初始位姿：({x:.3f}, {y:.3f}, theta={yaw:.3f})",
        status="完成" if success else "失败",
        success=success,
        metadata={
            "pose": {"x": x, "y": y, "theta": yaw, "yaw": yaw, "frame_id": frame_id},
            "result": result,
        },
    )
    return jsonify(result), 200 if success else 503


@app.route("/api/map/rebuild", methods=["POST"])
def rebuild_map():
    result = ROS_BRIDGE.reset_mapping()
    success = bool(result.get("success"))
    add_log(
        "重新建图",
        result.get("message", "重新建图"),
        status="进行中" if success else "失败",
        success=success,
        metadata={"map_result": result},
    )
    return jsonify(result), 200 if success else 503


@app.route("/api/map/finish", methods=["POST"])
def finish_map():
    data = request.get_json(silent=True) or {}
    save = bool(data.get("save"))
    result = ROS_BRIDGE.finish_mapping(save=save)
    success = bool(result.get("success"))
    add_log(
        "保存建图" if save else "放弃建图",
        result.get("message", "建图收尾"),
        status="完成" if success else "失败",
        success=success,
        metadata={"save": save, "map_result": result},
    )
    return jsonify(result), 200 if success else 503


@app.route("/api/robot/velocity", methods=["POST"])
def robot_velocity():
    data = request.get_json(silent=True) or {}
    try:
        linear_x = float(data.get("linear_x", 0.0))
        angular_z = float(data.get("angular_z", 0.0))
    except (TypeError, ValueError):
        return jsonify({"success": False, "error": "速度参数无效"}), 400

    max_linear = float(data.get("max_linear", 1.2) or 1.2)
    max_angular = float(data.get("max_angular", 2.4) or 2.4)
    command_session = data.get("command_session")
    command_seq = data.get("command_seq")
    linear_x = max(-max_linear, min(max_linear, linear_x))
    angular_z = max(-max_angular, min(max_angular, angular_z))
    result = ROS_BRIDGE.publish_velocity(
        linear_x,
        angular_z,
        command_session=command_session,
        command_seq=command_seq,
    )
    return jsonify(result), 200 if result.get("success") else 503


@app.route("/api/map/image", methods=["GET"])
def map_image():
    png = ROS_BRIDGE.get_map_png() or placeholder_png("No map received")
    return Response(
        png,
        mimetype="image/png",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.route("/api/map/target", methods=["POST"])
def set_target():
    data = request.get_json(silent=True) or {}
    try:
        x = float(data.get("x"))
        y = float(data.get("y"))
        theta = float(data.get("theta", 0.0))
    except (TypeError, ValueError):
        return jsonify({"success": False, "error": "目标点坐标无效"}), 400

    result = ROS_BRIDGE.send_navigation_goal(x, y, theta)
    success = bool(result.get("success"))
    add_log(
        "导航目标",
        f"发送导航目标：({x:.3f}, {y:.3f}, theta={theta:.3f})",
        status="进行中" if success else "失败",
        success=success,
        metadata={"target": {"x": x, "y": y, "theta": theta}, "nav_result": result},
    )
    status_code = 200 if success else 503
    return jsonify(
        {
            **result,
            "target_point": {"x": x, "y": y, "theta": theta},
            "nav_status": ROS_BRIDGE.get_map_data().get("nav_status"),
        }
    ), status_code


@app.route("/api/map/path", methods=["POST"])
def set_path():
    data = request.get_json(silent=True) or {}
    waypoints = data.get("waypoints") or []
    if not isinstance(waypoints, list):
        return jsonify({"success": False, "error": "waypoints 必须是数组"}), 400
    if not waypoints:
        return jsonify({"success": False, "error": "请先在地图上点击至少一个目标点"}), 400

    result = ROS_BRIDGE.send_navigation_path(waypoints)
    success = bool(result.get("success"))
    via_count = max(0, len(waypoints) - 1)
    add_log(
        "导航路径",
        f"发送导航路径：{via_count} 个途经点，1 个最终目标点",
        status="进行中" if success else "失败",
        success=success,
        metadata={"waypoints": waypoints, "nav_result": result},
    )
    return jsonify(
        {
            **result,
            "target_point": waypoints[-1] if waypoints else None,
            "nav_status": ROS_BRIDGE.get_map_data().get("nav_status"),
        }
    ), 200 if success else 503


@app.route("/api/map/dump_bucket_pose", methods=["GET", "POST"])
def dump_bucket_pose():
    if request.method == "GET":
        return jsonify({"success": True, "poses": DUMP_BUCKET_POSES})

    data = request.get_json(silent=True) or {}
    try:
        bucket_id = int(data.get("bucket_id"))
        x = float(data.get("x"))
        y = float(data.get("y"))
        yaw = float(data.get("yaw", data.get("theta", 0.0)))
    except (TypeError, ValueError):
        return jsonify({"success": False, "error": "卸载桶坐标无效"}), 400

    if bucket_id not in (1, 2):
        return jsonify({"success": False, "error": "卸载桶编号必须是 1 或 2"}), 400

    frame_id = data.get("frame_id") or ROS_CONFIG["map_frame"]
    try:
        nav_goal = dump_bucket_nav_goal_from_bucket_pose(
            bucket_id,
            x,
            y,
            yaw,
            frame_id,
        )
        DUMP_BUCKET_POSES[str(bucket_id)] = nav_goal
        save_dump_bucket_poses()

        result = ROS_BRIDGE.set_dump_bucket_pose(
            bucket_id,
            nav_goal["x"],
            nav_goal["y"],
            nav_goal["yaw"],
            frame_id,
        )
        task_synced = bool(result.get("success"))
        success = True

        add_log(
            "设定卸载桶位置",
            (
                f"设定卸载桶 {bucket_id} Nav2 Goal："
                f"({nav_goal['x']:.3f}, {nav_goal['y']:.3f}, "
                f"theta={nav_goal['yaw']:.3f})"
            ),
            status="完成" if task_synced else "完成（待行为树同步）",
            success=success,
            metadata={
                "input_pose": {
                    "bucket_id": bucket_id,
                    "x": x,
                    "y": y,
                    "theta": yaw,
                    "yaw": yaw,
                    "frame_id": frame_id,
                },
                "nav_goal": nav_goal,
                "pose": DUMP_BUCKET_POSES.get(str(bucket_id)),
                "task_result": result,
                "task_synced": task_synced,
            },
        )
        message = "卸载桶位置已保存"
        if not task_synced:
            message = (
                "卸载桶位置已保存；行为树 Action 暂不可用，"
                "启动 robot_task_server 后会从配置文件读取"
            )
        return jsonify(
            {
                "success": True,
                "message": result.get("message") if task_synced else message,
                "task_synced": task_synced,
                "task_result": result,
                "pose": DUMP_BUCKET_POSES.get(str(bucket_id)),
                "poses": DUMP_BUCKET_POSES,
            }
        ), 200
    except Exception as exc:
        app.logger.exception("dump bucket pose save failed")
        message = f"卸载桶位置保存失败: {exc}"
        try:
            add_log(
                "设定卸载桶位置",
                f"设定卸载桶 {bucket_id}：({x:.3f}, {y:.3f}, theta={yaw:.3f})",
                status="失败",
                success=False,
                metadata={"error": str(exc)},
            )
        except Exception:
            app.logger.exception("dump bucket pose failure log failed")
        return jsonify({"success": False, "error": message}), 500


@app.route("/api/map/regions", methods=["GET", "POST", "DELETE"])
def manage_regions():
    if request.method == "GET":
        return jsonify(REGIONS)

    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        region = {
            "region_id": int(data.get("region_id") or len(REGIONS) + 1),
            "region_name": data.get("name") or f"区域 {len(REGIONS) + 1}",
            "vertices": data.get("vertices") or [],
            "bounds": data.get("bounds") or {},
            "frame_id": data.get("frame_id") or ROS_CONFIG["map_frame"],
            "color": data.get("color") or "#2f80ed",
        }
        bounds = region["bounds"] or {}
        try:
            task_result = ROS_BRIDGE.set_clean_region(
                region["region_id"],
                float(bounds["min_x"]),
                float(bounds["min_y"]),
                float(bounds["max_x"]),
                float(bounds["max_y"]),
                region["frame_id"],
            )
        except (KeyError, TypeError, ValueError):
            return jsonify({"success": False, "error": "区域坐标无效，请重新框选"}), 400
        if not task_result.get("success"):
            return jsonify(task_result), 503
        REGIONS.append(region)
        add_log(
            "划定区域",
            f"新增区域：{region['region_name']}",
            metadata={"region": region, "task_result": task_result},
        )
        return jsonify({"success": True, "region": region, "task_result": task_result})

    region_id = request.args.get("region_id", type=int)
    if region_id is None:
        return jsonify({"success": False, "error": "缺少 region_id"}), 400
    before = len(REGIONS)
    REGIONS[:] = [region for region in REGIONS if region["region_id"] != region_id]
    if len(REGIONS) != before:
        add_log("删除区域", f"删除区域 ID：{region_id}", metadata={"region_id": region_id})
    return jsonify({"success": True, "region_id": region_id})


@app.route("/api/map/clean_region", methods=["POST"])
def clean_region():
    data = request.get_json(silent=True) or {}
    region_id = int(data.get("region_id") or 0)
    if not region_id:
        return jsonify({"success": False, "error": "请先选择要清扫的区域"}), 400
    result, status_code = submit_region_clean(region_id)
    return jsonify(result), status_code


@app.route("/api/camera/stream/<camera_id>")
def camera_stream(camera_id):
    frame_interval = 1.0 / max(1, int(CAMERA_CONFIG.get("frame_rate", 15)))

    def generate_frames():
        while True:
            frame = ROS_BRIDGE.wait_for_camera_frame(camera_id, timeout=0.5)
            if frame is None:
                frame = placeholder_jpeg(f"No {camera_id} frame")
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(frame_interval)

    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/camera/info", methods=["GET"])
def camera_info():
    return jsonify(ROS_BRIDGE.get_camera_info())


@app.route("/api/vision/stream")
def vision_stream():
    frame_interval = 1.0 / max(0.5, float(ROS_CONFIG.get("vision_monitor_max_fps", 6.0)))

    def generate_frames():
        while True:
            frame = ROS_BRIDGE.get_vision_frame()
            if frame is None:
                frame = placeholder_jpeg("No vision frame")
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(frame_interval)

    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/vision/info", methods=["GET"])
def vision_info():
    return jsonify(ROS_BRIDGE.get_vision_info())


@app.route("/api/logs", methods=["GET", "DELETE"])
def logs_api():
    if request.method == "DELETE":
        if Database.available():
            ok = Database.clear_operation_logs()
        else:
            MEMORY_LOGS.clear()
            ok = True
        return jsonify({"success": ok})

    page = request.args.get("page", default=1, type=int)
    page_size = request.args.get("page_size", default=20, type=int)
    keyword = (request.args.get("keyword") or "").strip() or None
    action_type = (request.args.get("action_type") or "").strip() or None

    if Database.available():
        data = Database.fetch_operation_logs(page, page_size, keyword, action_type)
        if data is not None:
            data["storage"] = "mysql"
            return jsonify({"success": True, **data})

    return jsonify({"success": True, **memory_logs_page(page, page_size, keyword, action_type)})


@app.route("/api/logs/<int:log_id>", methods=["DELETE"])
def delete_log(log_id):
    if Database.available():
        ok = Database.delete_operation_log(log_id)
    else:
        before = len(MEMORY_LOGS)
        MEMORY_LOGS[:] = [row for row in MEMORY_LOGS if int(row["id"]) != int(log_id)]
        ok = len(MEMORY_LOGS) != before
    return jsonify({"success": ok, "log_id": log_id})


@app.route("/api/system/info", methods=["GET"])
def system_info():
    ros_status = ROS_BRIDGE.status()
    return jsonify(
        {
            "system": "机器人控制系统",
            "version": "2.0.0-ros2",
            "robot_name": ROBOT_CONFIG["name"],
            "robot_type": ROBOT_CONFIG["type"],
            "mqtt_connected": False,
            "database_connected": Database.available(),
            "database_error": Database.last_error,
            "ros_connected": ros_status["started"],
            "ros_status": ros_status,
            "mode": "ROS2 实机/仿真模式",
            "timestamp": datetime.now().isoformat(),
        }
    )


def init_system():
    global DATABASE_READY
    print("正在初始化机器人控制系统...")
    DATABASE_READY = init_database()
    print(f"MySQL 初始化: {'成功' if DATABASE_READY else '失败或不可用'}")
    ros_ok = ROS_BRIDGE.start()
    print(f"ROS bridge 启动: {'成功' if ros_ok else ROS_BRIDGE.start_error}")
    add_log(
        "系统",
        "Flask ROS2 控制系统启动",
        status="完成" if ros_ok else "ROS未连接",
        success=ros_ok,
        metadata={"ros": ROS_BRIDGE.status(), "mysql": DATABASE_READY},
    )
    print("系统初始化完成")


if __name__ == "__main__":
    init_system()
    print("启动机器人控制系统...")
    print("访问地址: http://127.0.0.1:5000")
    app.run(host="0.0.0.0", port=5000, debug=DEBUG, threaded=True, use_reloader=False)
