from datetime import datetime, timedelta
import math
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
DATABASE_READY = False
MEMORY_LOGS = []
REGIONS = []


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


class RuntimeData:
    @staticmethod
    def battery_level():
        elapsed_minutes = max(0, int((datetime.now() - START_TIME).total_seconds() // 60))
        return max(0, 100 - elapsed_minutes // 5)

    @staticmethod
    def bin_levels():
        elapsed_minutes = max(0, int((datetime.now() - START_TIME).total_seconds() // 60))
        left = min(100, 18 + elapsed_minutes // 3)
        right = min(100, 11 + elapsed_minutes // 4)
        return {"left": left, "right": right}

    @staticmethod
    def robot_status():
        map_data = ROS_BRIDGE.get_map_data()
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
        map_config=MAP_CONFIG,
        robot_config=ROBOT_CONFIG,
        ros_config=ROS_CONFIG,
    )


@app.route("/chat")
def chat_page():
    return render_template(
        "chat.html",
        command_history=RuntimeData.command_history(),
        voice_config=VOICE_CONFIG,
        robot_config=ROBOT_CONFIG,
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
        "unload": "卸载垃圾",
        "grasp_start": "视觉抓取启动",
        "grasp_stop": "视觉抓取停止",
    }
    if command_type not in labels:
        return jsonify({"success": False, "error": "不支持的控制命令"}), 400

    if command_type == "standby":
        ROS_BRIDGE.publish_stop()

    if command_type in ("grasp_start", "grasp_stop"):
        action = "start" if command_type == "grasp_start" else "stop"
        result = ROS_BRIDGE.call_grasp_service(action)
        return jsonify({"success": result.get("success", False), "message": result.get("message", ""), "command_type": command_type})

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
        ROS_BRIDGE.publish_stop()
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
    return jsonify(response)


@app.route("/api/robot/voice_command", methods=["POST"])
def voice_command():
    data = request.get_json(silent=True) or {}
    command_text = data.get("text", "").strip()
    confidence = data.get("confidence")
    if not command_text:
        return jsonify({"success": False, "error": "语音文本为空"}), 400

    command_type = "clean_region" if ("清扫" in command_text or "打扫" in command_text) else "start"
    command_data = {"region_id": 1} if command_type == "clean_region" else {}
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
    return jsonify(response)


@app.route("/api/map/data", methods=["GET"])
def map_data():
    return jsonify(ROS_BRIDGE.get_map_data())


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
        REGIONS.append(region)
        add_log("划定区域", f"新增区域：{region['region_name']}", metadata=region)
        return jsonify({"success": True, "region": region})

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
