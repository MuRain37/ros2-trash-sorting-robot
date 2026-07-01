import os


# 机器人控制系统配置


def _env_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_list(name, default):
    value = os.getenv(name)
    if not value:
        return default
    return [item.strip() for item in value.split(",") if item.strip()]

# Flask配置
SECRET_KEY = os.getenv('FLASK_SECRET_KEY', 'robot-control-system-secret-key')
DEBUG = _env_bool('FLASK_DEBUG', True)

# MySQL数据库配置
DB_CONFIG = {
    'host': os.getenv('MYSQL_HOST', 'localhost'),
    'port': int(os.getenv('MYSQL_PORT', '3306')),
    'user': os.getenv('MYSQL_USER', 'root'),
    'password': os.getenv('MYSQL_PASSWORD', '123456'),  # 请根据你的MySQL密码修改
    'database': os.getenv('MYSQL_DATABASE', 'robot_control_db'),
    'charset': os.getenv('MYSQL_CHARSET', 'utf8mb4'),
    'autocommit': False,
}
if os.getenv('MYSQL_UNIX_SOCKET'):
    DB_CONFIG['unix_socket'] = os.getenv('MYSQL_UNIX_SOCKET')

# MQTT配置
MQTT_CONFIG = {
    'broker': '192.168.1.100',  # MQTT代理地址
    'port': 1883,
    'username': 'robot',         # MQTT用户名
    'password': 'robot123',      # MQTT密码
    'keepalive': 60
}

# MQTT主题配置
MQTT_TOPICS = {
    # 机器人状态主题
    'robot_status': 'robot/status',
    'robot_sensors': 'robot/sensors',
    'robot_position': 'robot/position',
    'robot_lidar': 'robot/lidar',
    
    # 控制主题
    'robot_control': 'robot/control',
    'robot_voice_command': 'robot/voice/command',
    'robot_text_command': 'robot/text/command',
    
    # 摄像头主题
    'robot_camera': 'robot/camera'
}

# 机器人配置
ROBOT_CONFIG = {
    'name': '清洁机器人一号',
    'type': '垃圾拾取机器人',
    'battery_capacity': 100,  # 电池容量百分比
    'max_speed': 1.5,         # 最大速度 m/s
    'max_payload': 5.0,       # 最大载重 kg
    'sensors': ['摄像头', '激光雷达', 'IMU']
}

# 摄像头配置
CAMERA_CONFIG = {
    'main_camera_url': '/api/camera/stream/depth',
    'frame_rate': 30,
    'resolution': '800x600'
}

# 地图配置
MAP_CONFIG = {
    'grid_size': 0.1,          # 网格大小（米）
    'map_width': 20,           # 地图宽度（米）
    'map_height': 20,          # 地图高度（米）
    'max_obstacle_distance': 5 # 最大障碍物检测距离（米）
}

# ROS2 话题与 Nav2 Action 配置
ROS_CONFIG = {
    'enabled': _env_bool('ROS_ENABLED', True),
    'node_name': os.getenv('ROS_WEB_NODE_NAME', 'flask_ros_bridge'),
    'depth_camera_topics': _env_list(
        'ROS_DEPTH_CAMERA_TOPICS',
        [
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/depth/image_rect_raw',
            '/camera/depth/image_raw',
            '/camera/depth/image_rect_raw',
        ],
    ),
    'color_camera_topics': _env_list(
        'ROS_COLOR_CAMERA_TOPICS',
        [
            '/camera/camera/color/image_raw',
            '/camera/color/image_raw',
            '/camera/image_raw',
        ],
    ),
    'map_topic': os.getenv('ROS_MAP_TOPIC', '/map'),
    'amcl_pose_topic': os.getenv('ROS_AMCL_POSE_TOPIC', '/amcl_pose'),
    'scan_topic': os.getenv('ROS_SCAN_TOPIC', '/scan'),
    'cmd_vel_topic': os.getenv('ROS_CMD_VEL_TOPIC', '/cmd_vel'),
    'static_map_yaml': os.getenv(
        'ROS_STATIC_MAP_YAML',
        '/home/xjw/ros2_robot/src/robot_nav2/maps/room.yaml',
    ),
    'map_frame': os.getenv('ROS_MAP_FRAME', 'map'),
    'robot_base_frame': os.getenv('ROS_ROBOT_BASE_FRAME', 'base_footprint'),
    'navigate_to_pose_action': os.getenv('ROS_NAVIGATE_TO_POSE_ACTION', 'navigate_to_pose'),
    'coverage_task_service': os.getenv('ROS_COVERAGE_TASK_SERVICE', 'add_coverage_rectangle'),
    'dump_trash_service': os.getenv('ROS_DUMP_TRASH_SERVICE', 'dump_trash'),
    'depth_visual_max_m': float(os.getenv('ROS_DEPTH_VISUAL_MAX_M', '8.0')),
    'goal_timeout_sec': float(os.getenv('ROS_GOAL_TIMEOUT_SEC', '3.0')),
    'dump_timeout_sec': float(os.getenv('ROS_DUMP_TIMEOUT_SEC', '60.0')),
}
ROS_CONFIG['depth_camera_topic'] = ROS_CONFIG['depth_camera_topics'][0]
ROS_CONFIG['color_camera_topic'] = ROS_CONFIG['color_camera_topics'][0]

# 语音识别配置
VOICE_CONFIG = {
    'language': 'zh-CN',
    'sample_rate': 16000,
    'max_duration': 10         # 最大录音时长（秒）
}
