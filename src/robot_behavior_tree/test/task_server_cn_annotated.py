# 此文件是 task_server.py 的中文逐行注释阅读版。
# 生成位置：src/robot_behavior_tree/test/task_server_cn_annotated.py
# 用途：帮助阅读和调试行为树任务调度逻辑；运行代码仍以原文件为准。
# 说明：每一行原代码前都添加了“中文注释”，原代码内容保持不删减。

# 中文注释：第 1 行，文档字符串，说明当前模块、类或方法的用途。
"""高层行为树任务调度节点。"""
# 中文注释：第 2 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 3 行，导入 dataclass 装饰器，用来声明只保存数据的轻量类。
from dataclasses import dataclass
# 中文注释：第 4 行，导入 json，用来读写桶位姿和清扫区域配置文件。
import json
# 中文注释：第 5 行，导入 math，用来做角度、三角函数和数值有效性判断。
import math
# 中文注释：第 6 行，导入 Path，用来处理配置文件路径。
from pathlib import Path
# 中文注释：第 7 行，导入 threading，用互斥锁保证同一时间只执行一个任务。
import threading
# 中文注释：第 8 行，导入 time，用单调时钟控制超时、等待和运动持续时间。
import time
# 中文注释：第 9 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 10 行，导入 ROS 2 Python 客户端库。
import rclpy
# 中文注释：第 11 行，导入 action 状态常量，用于判断导航结果。
from action_msgs.msg import GoalStatus
# 中文注释：第 12 行，导入位姿和速度消息类型。
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
# 中文注释：第 13 行，导入里程计消息类型。
from nav_msgs.msg import Odometry
# 中文注释：第 14 行，导入 Nav2 导航 action 类型。
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
# 中文注释：第 15 行，导入 ROS 2 action 客户端、服务端和响应枚举。
from rclpy.action import ActionClient, ActionServer
# 中文注释：第 16 行，导入 ROS 2 action 客户端、服务端和响应枚举。
from rclpy.action import CancelResponse, GoalResponse
# 中文注释：第 17 行，导入可重入回调组，让服务和 action 回调并发执行。
from rclpy.callback_groups import ReentrantCallbackGroup
# 中文注释：第 18 行，导入多线程执行器。
from rclpy.executors import MultiThreadedExecutor
# 中文注释：第 19 行，导入 ROS 2 节点基类。
from rclpy.node import Node
# 中文注释：第 20 行，导入项目自定义机械臂和任务 action 类型。
from robot_interfaces.action import ArmPreset, ArmTarget, RobotTask
# 中文注释：第 21 行，导入项目自定义视觉、路径和夹爪服务类型。
from robot_interfaces.srv import (
    # 中文注释：第 22 行，执行当前代码行，作为所在逻辑块的一部分。
    DetectDumpBucket,
    # 中文注释：第 23 行，执行当前代码行，作为所在逻辑块的一部分。
    DetectGraspTarget,
    # 中文注释：第 24 行，执行当前代码行，作为所在逻辑块的一部分。
    GenerateCoveragePath,
    # 中文注释：第 25 行，执行当前代码行，作为所在逻辑块的一部分。
    SetCameraTfPreset,
    # 中文注释：第 26 行，执行当前代码行，作为所在逻辑块的一部分。
    SetGripper,
# 中文注释：第 27 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
)
# 中文注释：第 28 行，导入 IMU 和 LaserScan 传感器消息。
from sensor_msgs.msg import Imu, LaserScan
# 中文注释：第 29 行，导入 Trigger 服务，用于无参数硬件动作。
from std_srvs.srv import Trigger
# 中文注释：第 30 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 31 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 32 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 33 行，定义视觉抓取目标的数据结构。
class GraspTarget:
    # 中文注释：第 34 行，文档字符串，说明当前模块、类或方法的用途。
    """视觉识别返回的抓取目标。"""
# 中文注释：第 35 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 36 行，保存视觉识别到的目标类别名。
    class_name: str
    # 中文注释：第 37 行，保存视觉识别置信度。
    confidence: float
    # 中文注释：第 38 行，保存目标在相机/抓取坐标中的 x 坐标，单位厘米。
    target_x_cm: float
    # 中文注释：第 39 行，保存目标在相机/抓取坐标中的 y 坐标，单位厘米。
    target_y_cm: float
    # 中文注释：第 40 行，保存目标在相机/抓取坐标中的 z 坐标，单位厘米。
    target_z_cm: float
    # 中文注释：第 41 行，保存机械臂末端姿态角。
    end_angle: float
    # 中文注释：第 42 行，保存夹爪旋转角或控制量。
    gripper_rotation: float
    # 中文注释：第 43 行，保存夹爪闭合比例。
    gripper_close_ratio: float
    # 中文注释：第 44 行，保存目标对应的车载垃圾桶编号。
    bucket_id: int
# 中文注释：第 45 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 46 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 47 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 48 行，定义地图中卸货桶标定位姿的数据结构。
class DumpBucketPose:
    # 中文注释：第 49 行，文档字符串，说明当前模块、类或方法的用途。
    """地图上标定的卸载桶位置。"""
# 中文注释：第 50 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 51 行，保存目标对应的车载垃圾桶编号。
    bucket_id: int
    # 中文注释：第 52 行，保存地图坐标 x。
    x: float
    # 中文注释：第 53 行，保存地图坐标 y。
    y: float
    # 中文注释：第 54 行，保存地图航向角。
    yaw: float
    # 中文注释：第 55 行，保存坐标系名称。
    frame_id: str
# 中文注释：第 56 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 57 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 58 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 59 行，定义清扫区域边界的数据结构。
class CleanRegion:
    # 中文注释：第 60 行，文档字符串，说明当前模块、类或方法的用途。
    """行为树保存的清扫区域。"""
# 中文注释：第 61 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 62 行，保存清扫区域编号。
    region_id: int
    # 中文注释：第 63 行，保存清扫区域 x 下界。
    min_x: float
    # 中文注释：第 64 行，保存清扫区域 y 下界。
    min_y: float
    # 中文注释：第 65 行，保存清扫区域 x 上界。
    max_x: float
    # 中文注释：第 66 行，保存清扫区域 y 上界。
    max_y: float
    # 中文注释：第 67 行，保存坐标系名称。
    frame_id: str
# 中文注释：第 68 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 69 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 70 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 71 行，定义卸货桶视觉检测结果的数据结构。
class DumpBucketDetection:
    # 中文注释：第 72 行，文档字符串，说明当前模块、类或方法的用途。
    """卸载桶视觉检测结果：中心偏移、深度和是否到达倾倒距离。"""
# 中文注释：第 73 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 74 行，保存视觉是否检测到卸货桶。
    detected: bool
    # 中文注释：第 75 行，保存视觉深度是否已达到倾倒距离要求。
    within_depth: bool
    # 中文注释：第 76 行，保存色彩分割轮廓的像素数量。
    mask_pixels: int
    # 中文注释：第 77 行，保存检测区域平均深度，单位米。
    mean_depth_m: float
    # 中文注释：第 78 行，保存桶中心相对图像中心的像素偏移。
    offset_x_px: float
    # 中文注释：第 79 行，保存桶中心相对图像中心的归一化水平偏移。
    offset_x_ratio: float
    # 中文注释：第 80 行，保存服务或检测流程返回的说明信息。
    message: str
# 中文注释：第 81 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 82 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 83 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 84 行，定义机器人当前地图位姿的数据结构。
class MapPose:
    # 中文注释：第 85 行，文档字符串，说明当前模块、类或方法的用途。
    """机器人当前 map 位姿。"""
# 中文注释：第 86 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 87 行，保存地图坐标 x。
    x: float
    # 中文注释：第 88 行，保存地图坐标 y。
    y: float
    # 中文注释：第 89 行，保存地图航向角。
    yaw: float
# 中文注释：第 90 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 91 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 92 行，把下面的类声明为 dataclass，自动生成初始化等数据类方法。
@dataclass
# 中文注释：第 93 行，定义 scan_camera 最近障碍物的数据结构。
class ScanCameraObstacle:
    # 中文注释：第 94 行，文档字符串，说明当前模块、类或方法的用途。
    """scan_camera 最近看到的前方近距离障碍。"""
# 中文注释：第 95 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 96 行，保存雷达测得的最近障碍物距离，单位米。
    range_m: float
    # 中文注释：第 97 行，保存雷达障碍物相对正前方的角度，单位弧度。
    bearing_rad: float
    # 中文注释：第 98 行，保存障碍物被看到时的单调时钟时间。
    seen_at: float
# 中文注释：第 99 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 100 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 101 行，定义统一任务调度节点，协调导航、视觉、机械臂和硬件服务。
class RobotTaskServer(Node):
    # 中文注释：第 102 行，文档字符串，说明当前模块、类或方法的用途。
    """统一接收用户任务，并调度导航、视觉、机械臂和底层硬件能力。"""
# 中文注释：第 103 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 104 行，初始化 ROS 节点、参数、订阅者、服务客户端、Action 客户端和任务服务器。
    def __init__(self):
        # 中文注释：第 105 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        super().__init__('robot_task_server')
# 中文注释：第 106 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 107 行，声明 ROS 参数 action_name：任务 action 服务名。
        self.declare_parameter('action_name', '/robot_task')
        # 中文注释：第 108 行，声明 ROS 参数 detect_grasp_service：抓取目标视觉检测服务名。
        self.declare_parameter('detect_grasp_service', 'vision_grasp/detect')
        # 中文注释：第 109 行，声明 ROS 参数 detect_dump_service：卸货垃圾桶视觉检测服务名。
        self.declare_parameter('detect_dump_service', 'detect_dump_bucket')
        # 中文注释：第 110 行，声明 ROS 参数 dump_bucket_1_position_service：把升降/旋转机构移动并锁定到车载桶 1 的服务名。
        self.declare_parameter('dump_bucket_1_position_service', 'dump_bucket_1_position')
        # 中文注释：第 111 行，声明 ROS 参数 dump_bucket_2_position_service：把升降/旋转机构移动并锁定到车载桶 2 的服务名。
        self.declare_parameter('dump_bucket_2_position_service', 'dump_bucket_2_position')
        # 中文注释：第 112 行，声明 ROS 参数 dump_bucket_1_pour_service：执行车载桶 1 倾倒流程的服务名。
        self.declare_parameter('dump_bucket_1_pour_service', 'dump_bucket_1_pour')
        # 中文注释：第 113 行，声明 ROS 参数 dump_bucket_2_pour_service：执行车载桶 2 倾倒流程的服务名。
        self.declare_parameter('dump_bucket_2_pour_service', 'dump_bucket_2_pour')
        # 中文注释：第 114 行，声明 ROS 参数 coverage_path_service：区域覆盖路径生成服务名。
        self.declare_parameter('coverage_path_service', 'generate_coverage_path')
        # 中文注释：第 115 行，声明 ROS 参数 arm_target_action：机械臂目标位姿 action 名。
        self.declare_parameter('arm_target_action', '/arm_target')
        # 中文注释：第 116 行，声明 ROS 参数 arm_preset_action：机械臂预设姿态 action 名。
        self.declare_parameter('arm_preset_action', '/arm_preset')
        # 中文注释：第 117 行，声明 ROS 参数 参数：供节点运行时配置使用。
        self.declare_parameter(
            # 中文注释：第 118 行，执行当前代码行，作为所在逻辑块的一部分。
            'camera_tf_preset_service',
            # 中文注释：第 119 行，执行当前代码行，作为所在逻辑块的一部分。
            '/real_camera_static_tf/set_preset',
        # 中文注释：第 120 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 121 行，声明 ROS 参数 navigate_to_pose_action：Nav2 单点导航 action 名。
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        # 中文注释：第 122 行，声明 ROS 参数 navigate_through_poses_action：Nav2 多点导航 action 名。
        self.declare_parameter('navigate_through_poses_action', 'navigate_through_poses')
        # 中文注释：第 123 行，声明 ROS 参数 gripper_service：夹爪控制服务名。
        self.declare_parameter('gripper_service', 'arm_gripper')
        # 中文注释：第 124 行，声明 ROS 参数 cmd_vel_topic：底盘速度命令话题名。
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # 中文注释：第 125 行，声明 ROS 参数 map_frame：地图坐标系名称。
        self.declare_parameter('map_frame', 'map')
        # 中文注释：第 126 行，声明 ROS 参数 odom_topic：里程计话题名。
        self.declare_parameter('odom_topic', '/odom')
        # 中文注释：第 127 行，声明 ROS 参数 amcl_pose_topic：AMCL 位姿话题名。
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        # 中文注释：第 128 行，声明 ROS 参数 imu_topic：IMU 话题名。
        self.declare_parameter('imu_topic', '/imu/data')
        # 中文注释：第 129 行，声明 ROS 参数 scan_camera_topic：前方 scan_camera 雷达话题名。
        self.declare_parameter('scan_camera_topic', '/scan_camera')
        # 中文注释：第 130 行，声明 ROS 参数 service_timeout_sec：普通服务等待和调用超时时间。
        self.declare_parameter('service_timeout_sec', 3.0)
        # 中文注释：第 131 行，声明 ROS 参数 action_timeout_sec：机械臂 action 默认超时时间。
        self.declare_parameter('action_timeout_sec', 45.0)
        # 中文注释：第 132 行，声明 ROS 参数 navigation_timeout_sec：导航 action 超时时间。
        self.declare_parameter('navigation_timeout_sec', 300.0)
        # 中文注释：第 133 行，声明 ROS 参数 dump_timeout_sec：倒桶硬件服务超时时间。
        self.declare_parameter('dump_timeout_sec', 90.0)
        # 中文注释：第 134 行，声明 ROS 参数 dump_backup_distance_m：锁住车载桶后底盘后退距离。
        self.declare_parameter('dump_backup_distance_m', 0.10)
        # 中文注释：第 135 行，声明 ROS 参数 dump_backup_linear_speed：锁住车载桶后底盘后退速度。
        self.declare_parameter('dump_backup_linear_speed', 0.05)
        # 中文注释：第 136 行，保留原代码中的注释，说明下面逻辑的意图或约束。
        # <= 0 means keep scanning/grasping until TASK_STOP or action cancel.
        # 中文注释：第 137 行，声明 ROS 参数 clean_max_objects：单次清扫最多抓取目标数，小于等于 0 表示一直清扫。
        self.declare_parameter('clean_max_objects', 0)
        # 中文注释：第 138 行，声明 ROS 参数 detect_retry_count：视觉检测失败后的重试次数。
        self.declare_parameter('detect_retry_count', 3)
        # 中文注释：第 139 行，声明 ROS 参数 approach_max_attempts：抓取前底盘靠近目标的最大尝试次数。
        self.declare_parameter('approach_max_attempts', 3)
        # 中文注释：第 140 行，声明 ROS 参数 approach_y_threshold_cm：判定目标可抓取的 y 距离阈值。
        self.declare_parameter('approach_y_threshold_cm', 30.0)
        # 中文注释：第 141 行，声明 ROS 参数 approach_target_y_cm：靠近后希望目标到达的 y 距离。
        self.declare_parameter('approach_target_y_cm', 24.0)
        # 中文注释：第 142 行，声明 ROS 参数 approach_max_step_m：每次靠近目标的最大前进距离。
        self.declare_parameter('approach_max_step_m', 0.08)
        # 中文注释：第 143 行，声明 ROS 参数 approach_linear_speed：抓取靠近阶段底盘前进速度。
        self.declare_parameter('approach_linear_speed', 0.08)
        # 中文注释：第 144 行，声明 ROS 参数 coverage_detect_interval_sec：覆盖路径导航中视觉检测周期。
        self.declare_parameter('coverage_detect_interval_sec', 0.3)
        # 中文注释：第 145 行，声明 ROS 参数 gripper_duration_ms：夹爪动作持续时间。
        self.declare_parameter('gripper_duration_ms', 500.0)
        # 中文注释：第 146 行，声明 ROS 参数 gripper_wait_sec：夹爪动作后的等待时间。
        self.declare_parameter('gripper_wait_sec', 1.0)
        # 中文注释：第 147 行，声明 ROS 参数 require_dump_bucket_detect：倒桶前是否要求视觉检测卸货桶。
        self.declare_parameter('require_dump_bucket_detect', True)
        # 中文注释：第 148 行，声明 ROS 参数 dump_visual_align_required：视觉居中或深度对齐失败时是否直接失败。
        self.declare_parameter('dump_visual_align_required', False)
        # 中文注释：第 149 行，声明 ROS 参数 dump_approach_distance_m：导航到卸货桶前方时保留的距离。
        self.declare_parameter('dump_approach_distance_m', 1.15)
        # 中文注释：第 150 行，声明 ROS 参数 dump_align_max_attempts：视觉深度对齐最大尝试次数。
        self.declare_parameter('dump_align_max_attempts', 10)
        # 中文注释：第 151 行，声明 ROS 参数 dump_align_target_depth_m：视觉深度对齐目标距离。
        self.declare_parameter('dump_align_target_depth_m', 0.40)
        # 中文注释：第 152 行，声明 ROS 参数 dump_align_depth_tolerance_m：视觉深度对齐容许误差。
        self.declare_parameter('dump_align_depth_tolerance_m', 0.04)
        # 中文注释：第 153 行，声明 ROS 参数 dump_align_step_m：视觉深度对齐每次前进步长。
        self.declare_parameter('dump_align_step_m', 0.06)
        # 中文注释：第 154 行，声明 ROS 参数 dump_align_linear_speed：视觉深度对齐前进速度。
        self.declare_parameter('dump_align_linear_speed', 0.05)
        # 中文注释：第 155 行，声明 ROS 参数 dump_rotate_angular_speed：倒桶前 180 度旋转角速度。
        self.declare_parameter('dump_rotate_angular_speed', 0.5)
        # 中文注释：第 156 行，声明 ROS 参数 dump_center_tolerance_ratio：视觉居中允许的归一化水平偏差。
        self.declare_parameter('dump_center_tolerance_ratio', 0.04)
        # 中文注释：第 157 行，声明 ROS 参数 dump_center_max_attempts：视觉居中最大旋转尝试次数。
        self.declare_parameter('dump_center_max_attempts', 12)
        # 中文注释：第 158 行，声明 ROS 参数 dump_center_angular_speed：视觉居中旋转角速度。
        self.declare_parameter('dump_center_angular_speed', 0.18)
        # 中文注释：第 159 行，声明 ROS 参数 dump_center_step_sec：视觉居中每次旋转持续时间。
        self.declare_parameter('dump_center_step_sec', 0.18)
        # 中文注释：第 160 行，声明 ROS 参数 dump_rotate_tolerance_rad：180 度旋转结束角度容差。
        self.declare_parameter('dump_rotate_tolerance_rad', 0.04)
        # 中文注释：第 161 行，声明 ROS 参数 dump_rotate_timeout_sec：180 度旋转超时时间。
        self.declare_parameter('dump_rotate_timeout_sec', 12.0)
        # 中文注释：第 162 行，声明 ROS 参数 dump_scan_camera_bucket_check_enabled：是否启用 scan_camera 障碍物提前确认卸货桶。
        self.declare_parameter('dump_scan_camera_bucket_check_enabled', True)
        # 中文注释：第 163 行，声明 ROS 参数 dump_scan_camera_obstacle_min_range_m：scan_camera 障碍物最小有效距离。
        self.declare_parameter('dump_scan_camera_obstacle_min_range_m', 0.05)
        # 中文注释：第 164 行，声明 ROS 参数 dump_scan_camera_obstacle_max_range_m：scan_camera 障碍物最大有效距离。
        self.declare_parameter('dump_scan_camera_obstacle_max_range_m', 1.4)
        # 中文注释：第 165 行，声明 ROS 参数 dump_scan_camera_front_angle_rad：scan_camera 前方检测角度范围。
        self.declare_parameter('dump_scan_camera_front_angle_rad', 0.8)
        # 中文注释：第 166 行，声明 ROS 参数 dump_scan_camera_obstacle_stale_sec：scan_camera 障碍物缓存有效时间。
        self.declare_parameter('dump_scan_camera_obstacle_stale_sec', 0.6)
        # 中文注释：第 167 行，声明 ROS 参数 dump_scan_camera_check_interval_sec：导航中检查 scan_camera 障碍物的周期。
        self.declare_parameter('dump_scan_camera_check_interval_sec', 0.5)
        # 中文注释：第 168 行，声明 ROS 参数 dump_scan_camera_bucket_map_tolerance_m：视觉估计桶位置和地图标定桶位置的匹配容差。
        self.declare_parameter('dump_scan_camera_bucket_map_tolerance_m', 0.75)
        # 中文注释：第 169 行，声明 ROS 参数 dump_camera_horizontal_fov_rad：相机水平视场角，用于像素偏移转角度。
        self.declare_parameter('dump_camera_horizontal_fov_rad', 1.21)
        # 中文注释：第 170 行，声明 ROS 参数 参数：供节点运行时配置使用。
        self.declare_parameter(
            # 中文注释：第 171 行，执行当前代码行，作为所在逻辑块的一部分。
            'dump_pose_store_path',
            # 中文注释：第 172 行，执行当前代码行，作为所在逻辑块的一部分。
            '/home/xjw/ros2_robot/src/robot_behavior_tree/config/dump_bucket_poses.json',
        # 中文注释：第 173 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 174 行，声明 ROS 参数 参数：供节点运行时配置使用。
        self.declare_parameter(
            # 中文注释：第 175 行，执行当前代码行，作为所在逻辑块的一部分。
            'clean_region_store_path',
            # 中文注释：第 176 行，执行当前代码行，作为所在逻辑块的一部分。
            '/home/xjw/ros2_robot/src/robot_behavior_tree/config/clean_regions.json',
        # 中文注释：第 177 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 178 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 179 行，计算或保存局部变量 callback_group，供后续逻辑使用。
        callback_group = ReentrantCallbackGroup()
        # 中文注释：第 180 行，设置实例成员 self._task_lock，保存节点运行状态或 ROS 通信对象。
        self._task_lock = threading.Lock()
        # 中文注释：第 181 行，设置实例成员 self._active_goal，保存节点运行状态或 ROS 通信对象。
        self._active_goal = None
        # 中文注释：第 182 行，设置实例成员 self._stop_requested，保存节点运行状态或 ROS 通信对象。
        self._stop_requested = False
        # 中文注释：第 183 行，设置实例成员 self._dump_bucket_poses，保存节点运行状态或 ROS 通信对象。
        self._dump_bucket_poses = {}
        # 中文注释：第 184 行，设置实例成员 self._clean_regions，保存节点运行状态或 ROS 通信对象。
        self._clean_regions = {}
        # 中文注释：第 185 行，设置实例成员 self._latest_odom_yaw，保存节点运行状态或 ROS 通信对象。
        self._latest_odom_yaw = None
        # 中文注释：第 186 行，设置实例成员 self._latest_imu_yaw，保存节点运行状态或 ROS 通信对象。
        self._latest_imu_yaw = None
        # 中文注释：第 187 行，设置实例成员 self._latest_map_pose，保存节点运行状态或 ROS 通信对象。
        self._latest_map_pose = None
        # 中文注释：第 188 行，设置实例成员 self._latest_scan_camera_obstacle，保存节点运行状态或 ROS 通信对象。
        self._latest_scan_camera_obstacle = None
        # 中文注释：第 189 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._load_dump_bucket_poses()
        # 中文注释：第 190 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._load_clean_regions()
        # 中文注释：第 191 行，创建 ROS 订阅者，监听对应话题并绑定回调函数。
        self._odom_sub = self.create_subscription(
            # 中文注释：第 192 行，执行当前代码行，作为所在逻辑块的一部分。
            Odometry,
            # 中文注释：第 193 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('odom_topic').value,
            # 中文注释：第 194 行，执行当前代码行，作为所在逻辑块的一部分。
            self._handle_odom,
            # 中文注释：第 195 行，执行当前代码行，作为所在逻辑块的一部分。
            10,
            # 中文注释：第 196 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 197 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 198 行，创建 ROS 订阅者，监听对应话题并绑定回调函数。
        self._amcl_pose_sub = self.create_subscription(
            # 中文注释：第 199 行，执行当前代码行，作为所在逻辑块的一部分。
            PoseWithCovarianceStamped,
            # 中文注释：第 200 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('amcl_pose_topic').value,
            # 中文注释：第 201 行，执行当前代码行，作为所在逻辑块的一部分。
            self._handle_amcl_pose,
            # 中文注释：第 202 行，执行当前代码行，作为所在逻辑块的一部分。
            10,
            # 中文注释：第 203 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 204 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 205 行，创建 ROS 订阅者，监听对应话题并绑定回调函数。
        self._imu_sub = self.create_subscription(
            # 中文注释：第 206 行，执行当前代码行，作为所在逻辑块的一部分。
            Imu,
            # 中文注释：第 207 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('imu_topic').value,
            # 中文注释：第 208 行，执行当前代码行，作为所在逻辑块的一部分。
            self._handle_imu,
            # 中文注释：第 209 行，执行当前代码行，作为所在逻辑块的一部分。
            10,
            # 中文注释：第 210 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 211 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 212 行，创建 ROS 订阅者，监听对应话题并绑定回调函数。
        self._scan_camera_sub = self.create_subscription(
            # 中文注释：第 213 行，执行当前代码行，作为所在逻辑块的一部分。
            LaserScan,
            # 中文注释：第 214 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('scan_camera_topic').value,
            # 中文注释：第 215 行，执行当前代码行，作为所在逻辑块的一部分。
            self._handle_scan_camera,
            # 中文注释：第 216 行，执行当前代码行，作为所在逻辑块的一部分。
            10,
            # 中文注释：第 217 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 218 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 219 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 220 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
        self._detect_grasp_client = self.create_client(
            # 中文注释：第 221 行，执行当前代码行，作为所在逻辑块的一部分。
            DetectGraspTarget,
            # 中文注释：第 222 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('detect_grasp_service').value,
            # 中文注释：第 223 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 224 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 225 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
        self._detect_dump_client = self.create_client(
            # 中文注释：第 226 行，执行当前代码行，作为所在逻辑块的一部分。
            DetectDumpBucket,
            # 中文注释：第 227 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('detect_dump_service').value,
            # 中文注释：第 228 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 229 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 230 行，设置实例成员 self._dump_bucket_position_clients，保存节点运行状态或 ROS 通信对象。
        self._dump_bucket_position_clients = {
            # 中文注释：第 231 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
            1: self.create_client(
                # 中文注释：第 232 行，执行当前代码行，作为所在逻辑块的一部分。
                Trigger,
                # 中文注释：第 233 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('dump_bucket_1_position_service').value,
                # 中文注释：第 234 行，计算或保存局部变量 callback_group，供后续逻辑使用。
                callback_group=callback_group,
            # 中文注释：第 235 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            ),
            # 中文注释：第 236 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
            2: self.create_client(
                # 中文注释：第 237 行，执行当前代码行，作为所在逻辑块的一部分。
                Trigger,
                # 中文注释：第 238 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('dump_bucket_2_position_service').value,
                # 中文注释：第 239 行，计算或保存局部变量 callback_group，供后续逻辑使用。
                callback_group=callback_group,
            # 中文注释：第 240 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            ),
        # 中文注释：第 241 行，结束前面开始的多行结构或函数调用。
        }
        # 中文注释：第 242 行，设置实例成员 self._dump_bucket_pour_clients，保存节点运行状态或 ROS 通信对象。
        self._dump_bucket_pour_clients = {
            # 中文注释：第 243 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
            1: self.create_client(
                # 中文注释：第 244 行，执行当前代码行，作为所在逻辑块的一部分。
                Trigger,
                # 中文注释：第 245 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('dump_bucket_1_pour_service').value,
                # 中文注释：第 246 行，计算或保存局部变量 callback_group，供后续逻辑使用。
                callback_group=callback_group,
            # 中文注释：第 247 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            ),
            # 中文注释：第 248 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
            2: self.create_client(
                # 中文注释：第 249 行，执行当前代码行，作为所在逻辑块的一部分。
                Trigger,
                # 中文注释：第 250 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('dump_bucket_2_pour_service').value,
                # 中文注释：第 251 行，计算或保存局部变量 callback_group，供后续逻辑使用。
                callback_group=callback_group,
            # 中文注释：第 252 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            ),
        # 中文注释：第 253 行，结束前面开始的多行结构或函数调用。
        }
        # 中文注释：第 254 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
        self._coverage_path_client = self.create_client(
            # 中文注释：第 255 行，执行当前代码行，作为所在逻辑块的一部分。
            GenerateCoveragePath,
            # 中文注释：第 256 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('coverage_path_service').value,
            # 中文注释：第 257 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 258 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 259 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
        self._gripper_client = self.create_client(
            # 中文注释：第 260 行，执行当前代码行，作为所在逻辑块的一部分。
            SetGripper,
            # 中文注释：第 261 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('gripper_service').value,
            # 中文注释：第 262 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 263 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 264 行，创建 ROS 服务客户端，后续通过该客户端调用外部能力。
        self._camera_tf_preset_client = self.create_client(
            # 中文注释：第 265 行，执行当前代码行，作为所在逻辑块的一部分。
            SetCameraTfPreset,
            # 中文注释：第 266 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('camera_tf_preset_service').value,
            # 中文注释：第 267 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 268 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 269 行，创建 ROS action 客户端，用来发送异步动作目标。
        self._arm_target_client = ActionClient(
            # 中文注释：第 270 行，执行当前代码行，作为所在逻辑块的一部分。
            self,
            # 中文注释：第 271 行，执行当前代码行，作为所在逻辑块的一部分。
            ArmTarget,
            # 中文注释：第 272 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('arm_target_action').value,
            # 中文注释：第 273 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 274 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 275 行，创建 ROS action 客户端，用来发送异步动作目标。
        self._arm_preset_client = ActionClient(
            # 中文注释：第 276 行，执行当前代码行，作为所在逻辑块的一部分。
            self,
            # 中文注释：第 277 行，执行当前代码行，作为所在逻辑块的一部分。
            ArmPreset,
            # 中文注释：第 278 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('arm_preset_action').value,
            # 中文注释：第 279 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 280 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 281 行，创建 ROS action 客户端，用来发送异步动作目标。
        self._navigate_client = ActionClient(
            # 中文注释：第 282 行，执行当前代码行，作为所在逻辑块的一部分。
            self,
            # 中文注释：第 283 行，执行当前代码行，作为所在逻辑块的一部分。
            NavigateToPose,
            # 中文注释：第 284 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('navigate_to_pose_action').value,
            # 中文注释：第 285 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 286 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 287 行，创建 ROS action 客户端，用来发送异步动作目标。
        self._navigate_through_client = ActionClient(
            # 中文注释：第 288 行，执行当前代码行，作为所在逻辑块的一部分。
            self,
            # 中文注释：第 289 行，执行当前代码行，作为所在逻辑块的一部分。
            NavigateThroughPoses,
            # 中文注释：第 290 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('navigate_through_poses_action').value,
            # 中文注释：第 291 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 292 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 293 行，创建 ROS 发布者，用来发布对应话题消息。
        self._cmd_vel_pub = self.create_publisher(
            # 中文注释：第 294 行，执行当前代码行，作为所在逻辑块的一部分。
            Twist,
            # 中文注释：第 295 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('cmd_vel_topic').value,
            # 中文注释：第 296 行，执行当前代码行，作为所在逻辑块的一部分。
            10,
        # 中文注释：第 297 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 298 行，创建 ROS action 服务端，用来接收上层任务请求。
        self._action_server = ActionServer(
            # 中文注释：第 299 行，执行当前代码行，作为所在逻辑块的一部分。
            self,
            # 中文注释：第 300 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask,
            # 中文注释：第 301 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('action_name').value,
            # 中文注释：第 302 行，计算或保存局部变量 execute_callback，供后续逻辑使用。
            execute_callback=self._execute_task,
            # 中文注释：第 303 行，计算或保存局部变量 goal_callback，供后续逻辑使用。
            goal_callback=self._handle_goal,
            # 中文注释：第 304 行，计算或保存局部变量 cancel_callback，供后续逻辑使用。
            cancel_callback=self._handle_cancel,
            # 中文注释：第 305 行，计算或保存局部变量 callback_group，供后续逻辑使用。
            callback_group=callback_group,
        # 中文注释：第 306 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 307 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 308 行，输出 info 级别日志，记录当前流程状态或关键参数。
        self.get_logger().info(
            # 中文注释：第 309 行，执行当前代码行，作为所在逻辑块的一部分。
            'robot behavior tree ready: '
            # 中文注释：第 310 行，计算或保存局部变量 f'action，供后续逻辑使用。
            f'action={self.get_parameter("action_name").value}'
        # 中文注释：第 311 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 312 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 313 行，处理里程计消息并缓存当前航向角。
    def _handle_odom(self, msg):
        # 中文注释：第 314 行，设置实例成员 self._latest_odom_yaw，保存节点运行状态或 ROS 通信对象。
        self._latest_odom_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
# 中文注释：第 315 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 316 行，处理 IMU 消息并缓存当前航向角。
    def _handle_imu(self, msg):
        # 中文注释：第 317 行，设置实例成员 self._latest_imu_yaw，保存节点运行状态或 ROS 通信对象。
        self._latest_imu_yaw = self._yaw_from_quaternion(msg.orientation)
# 中文注释：第 318 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 319 行，处理 AMCL 位姿并缓存机器人在 map 坐标系下的位置。
    def _handle_amcl_pose(self, msg):
        # 中文注释：第 320 行，计算或保存局部变量 pose，供后续逻辑使用。
        pose = msg.pose.pose
        # 中文注释：第 321 行，设置实例成员 self._latest_map_pose，保存节点运行状态或 ROS 通信对象。
        self._latest_map_pose = MapPose(
            # 中文注释：第 322 行，计算或保存局部变量 x，供后续逻辑使用。
            x=float(pose.position.x),
            # 中文注释：第 323 行，计算或保存局部变量 y，供后续逻辑使用。
            y=float(pose.position.y),
            # 中文注释：第 324 行，计算或保存局部变量 yaw，供后续逻辑使用。
            yaw=self._yaw_from_quaternion(pose.orientation),
        # 中文注释：第 325 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 326 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 327 行，处理 scan_camera 雷达数据，记录前方最近障碍物。
    def _handle_scan_camera(self, msg):
        # 中文注释：第 328 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not bool(
            # 中文注释：第 329 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('dump_scan_camera_bucket_check_enabled').value
        # 中文注释：第 330 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 331 行，结束当前函数或方法，不返回额外数据。
            return
# 中文注释：第 332 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 333 行，计算或保存局部变量 min_range，供后续逻辑使用。
        min_range = max(
            # 中文注释：第 334 行，执行当前代码行，作为所在逻辑块的一部分。
            0.0,
            # 中文注释：第 335 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_obstacle_min_range_m').value),
        # 中文注释：第 336 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 337 行，计算或保存局部变量 max_range，供后续逻辑使用。
        max_range = max(
            # 中文注释：第 338 行，执行当前代码行，作为所在逻辑块的一部分。
            min_range,
            # 中文注释：第 339 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_obstacle_max_range_m').value),
        # 中文注释：第 340 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 341 行，计算或保存局部变量 front_angle，供后续逻辑使用。
        front_angle = abs(
            # 中文注释：第 342 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_front_angle_rad').value)
        # 中文注释：第 343 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 344 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 345 行，计算或保存局部变量 best_range，供后续逻辑使用。
        best_range = None
        # 中文注释：第 346 行，计算或保存局部变量 best_angle，供后续逻辑使用。
        best_angle = None
        # 中文注释：第 347 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for index, range_m in enumerate(msg.ranges):
            # 中文注释：第 348 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not math.isfinite(range_m):
                # 中文注释：第 349 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
            # 中文注释：第 350 行，计算或保存局部变量 range_m，供后续逻辑使用。
            range_m = float(range_m)
            # 中文注释：第 351 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if range_m < min_range or range_m > max_range:
                # 中文注释：第 352 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
            # 中文注释：第 353 行，计算或保存局部变量 angle，供后续逻辑使用。
            angle = float(msg.angle_min + index * msg.angle_increment)
            # 中文注释：第 354 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if abs(angle) > front_angle:
                # 中文注释：第 355 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
            # 中文注释：第 356 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if best_range is None or range_m < best_range:
                # 中文注释：第 357 行，计算或保存局部变量 best_range，供后续逻辑使用。
                best_range = range_m
                # 中文注释：第 358 行，计算或保存局部变量 best_angle，供后续逻辑使用。
                best_angle = angle
# 中文注释：第 359 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 360 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if best_range is None:
            # 中文注释：第 361 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 362 行，设置实例成员 self._latest_scan_camera_obstacle，保存节点运行状态或 ROS 通信对象。
        self._latest_scan_camera_obstacle = ScanCameraObstacle(
            # 中文注释：第 363 行，计算或保存局部变量 range_m，供后续逻辑使用。
            range_m=best_range,
            # 中文注释：第 364 行，计算或保存局部变量 bearing_rad，供后续逻辑使用。
            bearing_rad=best_angle,
            # 中文注释：第 365 行，计算或保存局部变量 seen_at，供后续逻辑使用。
            seen_at=time.monotonic(),
        # 中文注释：第 366 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 367 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 368 行，校验并决定是否接收新的任务目标。
    def _handle_goal(self, goal_request):
        # 中文注释：第 369 行，计算或保存局部变量 task_type，供后续逻辑使用。
        task_type = int(goal_request.task_type)
        # 中文注释：第 370 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type not in self._known_task_types():
            # 中文注释：第 371 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'rejecting unknown task_type={task_type}')
            # 中文注释：第 372 行，返回当前函数或方法的执行结果。
            return GoalResponse.REJECT
        # 中文注释：第 373 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_STOP:
            # 中文注释：第 374 行，返回当前函数或方法的执行结果。
            return GoalResponse.ACCEPT
        # 中文注释：第 375 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._active_goal is not None:
            # 中文注释：第 376 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning('rejecting task because another task is active')
            # 中文注释：第 377 行，返回当前函数或方法的执行结果。
            return GoalResponse.REJECT
        # 中文注释：第 378 行，返回当前函数或方法的执行结果。
        return GoalResponse.ACCEPT
# 中文注释：第 379 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 380 行，响应任务取消请求并立即发布停车命令。
    def _handle_cancel(self, goal_handle):
        # 中文注释：第 381 行，删除不再使用的局部引用，避免未使用变量告警。
        del goal_handle
        # 中文注释：第 382 行，立即发布零速度，确保底盘停止。
        self._publish_stop()
        # 中文注释：第 383 行，返回当前函数或方法的执行结果。
        return CancelResponse.ACCEPT
# 中文注释：第 384 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 385 行，执行 action 目标，负责加锁、分发、清理和回传结果。
    def _execute_task(self, goal_handle):
        # 中文注释：第 386 行，计算或保存局部变量 task_type，供后续逻辑使用。
        task_type = int(goal_handle.request.task_type)
        # 中文注释：第 387 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if (
            # 中文注释：第 388 行，执行当前代码行，作为所在逻辑块的一部分。
            task_type == RobotTask.Goal.TASK_STOP
            # 中文注释：第 389 行，计算或保存局部变量 and not self._task_lock.acquire(blocking，供后续逻辑使用。
            and not self._task_lock.acquire(blocking=False)
        # 中文注释：第 390 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 391 行，设置实例成员 self._stop_requested，保存节点运行状态或 ROS 通信对象。
            self._stop_requested = True
            # 中文注释：第 392 行，立即发布零速度，确保底盘停止。
            self._publish_stop()
            # 中文注释：第 393 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal_handle.succeed()
            # 中文注释：第 394 行，返回当前函数或方法的执行结果。
            return self._result(True, 'stop requested')
# 中文注释：第 395 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 396 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._task_lock.acquire(blocking=False):
            # 中文注释：第 397 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal_handle.abort()
            # 中文注释：第 398 行，返回当前函数或方法的执行结果。
            return self._result(False, 'another task is already running')
# 中文注释：第 399 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 400 行，设置实例成员 self._active_goal，保存节点运行状态或 ROS 通信对象。
        self._active_goal = goal_handle
        # 中文注释：第 401 行，构造 action 目标或读取请求目标对象。
        goal = goal_handle.request
        # 中文注释：第 402 行，计算或保存局部变量 task_name，供后续逻辑使用。
        task_name = self._task_name(task_type)
        # 中文注释：第 403 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type != RobotTask.Goal.TASK_STOP:
            # 中文注释：第 404 行，设置实例成员 self._stop_requested，保存节点运行状态或 ROS 通信对象。
            self._stop_requested = False
        # 中文注释：第 405 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, f'{task_name}: started', 0.0)
# 中文注释：第 406 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 407 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 408 行，接收子流程返回的成功标志和说明信息。
            success, message = self._dispatch_task(goal_handle, goal)
        # 中文注释：第 409 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            # 中文注释：第 410 行，计算或保存局部变量 success，供后续逻辑使用。
            success = False
            # 中文注释：第 411 行，计算或保存局部变量 message，供后续逻辑使用。
            message = f'{task_name} failed: {exc}'
        # 中文注释：第 412 行，声明数据字段 finally 及其类型。
        finally:
            # 中文注释：第 413 行，立即发布零速度，确保底盘停止。
            self._publish_stop()
            # 中文注释：第 414 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._close_gripper_for_cleanup(task_name)
            # 中文注释：第 415 行，设置实例成员 self._stop_requested，保存节点运行状态或 ROS 通信对象。
            self._stop_requested = False
            # 中文注释：第 416 行，设置实例成员 self._active_goal，保存节点运行状态或 ROS 通信对象。
            self._active_goal = None
            # 中文注释：第 417 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._task_lock.release()
# 中文注释：第 418 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 419 行，保存动作、服务或函数调用的结果。
        result = self._result(success, message)
        # 中文注释：第 420 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if goal_handle.is_cancel_requested:
            # 中文注释：第 421 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal_handle.canceled()
            # 中文注释：第 422 行，计算或保存局部变量 result.success，供后续逻辑使用。
            result.success = False
            # 中文注释：第 423 行，计算或保存局部变量 result.message，供后续逻辑使用。
            result.message = f'{task_name} canceled'
        # 中文注释：第 424 行，继续判断前面 if 未命中的另一种条件。
        elif success:
            # 中文注释：第 425 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(goal_handle, f'{task_name}: completed', 1.0)
            # 中文注释：第 426 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal_handle.succeed()
        # 中文注释：第 427 行，声明数据字段 else 及其类型。
        else:
            # 中文注释：第 428 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal_handle.abort()
        # 中文注释：第 429 行，返回当前函数或方法的执行结果。
        return result
# 中文注释：第 430 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 431 行，根据任务类型分派到具体行为树流程。
    def _dispatch_task(self, goal_handle, goal):
        # 中文注释：第 432 行，计算或保存局部变量 task_type，供后续逻辑使用。
        task_type = int(goal.task_type)
        # 中文注释：第 433 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_CLEAN_TRASH:
            # 中文注释：第 434 行，返回当前函数或方法的执行结果。
            return self._run_clean_trash_tree(goal_handle)
        # 中文注释：第 435 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_DUMP_TRASH:
            # 中文注释：第 436 行，返回当前函数或方法的执行结果。
            return self._run_dump_trash_tree(goal_handle, goal)
        # 中文注释：第 437 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_CLEAN_REGION:
            # 中文注释：第 438 行，返回当前函数或方法的执行结果。
            return self._run_clean_region_tree(goal_handle, goal)
        # 中文注释：第 439 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_NAVIGATE_TO:
            # 中文注释：第 440 行，返回当前函数或方法的执行结果。
            return self._run_navigate_tree(goal_handle, goal)
        # 中文注释：第 441 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_NAVIGATE_PATH:
            # 中文注释：第 442 行，返回当前函数或方法的执行结果。
            return self._run_navigate_path_tree(goal_handle, goal)
        # 中文注释：第 443 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_SET_DUMP_POSE:
            # 中文注释：第 444 行，返回当前函数或方法的执行结果。
            return self._run_set_dump_pose(goal_handle, goal)
        # 中文注释：第 445 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_SET_CLEAN_REGION:
            # 中文注释：第 446 行，返回当前函数或方法的执行结果。
            return self._run_set_clean_region(goal_handle, goal)
        # 中文注释：第 447 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_STOP:
            # 中文注释：第 448 行，设置实例成员 self._stop_requested，保存节点运行状态或 ROS 通信对象。
            self._stop_requested = True
            # 中文注释：第 449 行，立即发布零速度，确保底盘停止。
            self._publish_stop()
            # 中文注释：第 450 行，返回当前函数或方法的执行结果。
            return True, 'robot stopped'
        # 中文注释：第 451 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if task_type == RobotTask.Goal.TASK_RESUME:
            # 中文注释：第 452 行，返回当前函数或方法的执行结果。
            return True, 'resume accepted'
        # 中文注释：第 453 行，返回当前函数或方法的执行结果。
        return False, f'unsupported task_type={task_type}'
# 中文注释：第 454 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 455 行，执行清扫垃圾流程：识别、靠近、抓取、投放和复位。
    def _run_clean_trash_tree(self, goal_handle):
        # 中文注释：第 456 行，文档字符串，说明当前模块、类或方法的用途。
        """清扫垃圾：识别 -> 必要时前进 -> MoveIt 抓取 -> 投桶 -> 归中。"""
        # 中文注释：第 457 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'clean_trash: arm center', 0.05)
        # 中文注释：第 458 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
# 中文注释：第 459 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 460 行，计算或保存局部变量 picked，供后续逻辑使用。
        picked = 0
        # 中文注释：第 461 行，计算或保存局部变量 scan_index，供后续逻辑使用。
        scan_index = 0
        # 中文注释：第 462 行，计算或保存局部变量 max_objects，供后续逻辑使用。
        max_objects = int(self.get_parameter('clean_max_objects').value)
        # 中文注释：第 463 行，开始 while 循环，在条件满足期间持续执行。
        while max_objects <= 0 or picked < max_objects:
            # 中文注释：第 464 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 465 行，返回当前函数或方法的执行结果。
                return False, 'clean trash canceled'
# 中文注释：第 466 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 467 行，计算或保存局部变量 scan_index +，供后续逻辑使用。
            scan_index += 1
            # 中文注释：第 468 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 469 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 470 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_trash: detecting target {scan_index}',
                # 中文注释：第 471 行，执行当前代码行，作为所在逻辑块的一部分。
                0.10,
            # 中文注释：第 472 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 473 行，计算或保存局部变量 target，供后续逻辑使用。
            target = self._detect_reachable_target(goal_handle)
            # 中文注释：第 474 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if target is None:
                # 中文注释：第 475 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if max_objects > 0:
                    # 中文注释：第 476 行，开始条件判断，满足条件时执行下面的分支逻辑。
                    if picked == 0:
                        # 中文注释：第 477 行，返回当前函数或方法的执行结果。
                        return False, 'no reachable trash target detected'
                    # 中文注释：第 478 行，返回当前函数或方法的执行结果。
                    return True, f'clean trash completed: picked={picked}'
                # 中文注释：第 479 行，向 action 客户端发布当前任务进度反馈。
                self._feedback(
                    # 中文注释：第 480 行，执行当前代码行，作为所在逻辑块的一部分。
                    goal_handle,
                    # 中文注释：第 481 行，执行当前代码行，作为所在逻辑块的一部分。
                    'clean_trash: no target, scanning again',
                    # 中文注释：第 482 行，执行当前代码行，作为所在逻辑块的一部分。
                    0.20,
                # 中文注释：第 483 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 484 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
                time.sleep(0.5)
                # 中文注释：第 485 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
# 中文注释：第 486 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 487 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 488 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 489 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_trash: grasping {target.class_name}',
                # 中文注释：第 490 行，执行当前代码行，作为所在逻辑块的一部分。
                0.35,
            # 中文注释：第 491 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 492 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._grasp_target(goal_handle, target)
            # 中文注释：第 493 行，计算或保存局部变量 picked +，供后续逻辑使用。
            picked += 1
# 中文注释：第 494 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 495 行，返回当前函数或方法的执行结果。
        return True, f'clean trash completed: picked={picked}'
# 中文注释：第 496 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 497 行，循环检测目标，必要时让底盘前进直到目标可抓取。
    def _detect_reachable_target(self, goal_handle):
        # 中文注释：第 498 行，计算或保存局部变量 last_target，供后续逻辑使用。
        last_target = None
        # 中文注释：第 499 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for approach_index in range(
            # 中文注释：第 500 行，执行当前代码行，作为所在逻辑块的一部分。
            int(self.get_parameter('approach_max_attempts').value) + 1
        # 中文注释：第 501 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 502 行，计算或保存局部变量 target，供后续逻辑使用。
            target = self._detect_grasp_target(goal_handle)
            # 中文注释：第 503 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if target is None:
                # 中文注释：第 504 行，返回当前函数或方法的执行结果。
                return None
            # 中文注释：第 505 行，计算或保存局部变量 last_target，供后续逻辑使用。
            last_target = target
# 中文注释：第 506 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 507 行，计算或保存局部变量 threshold_cm，供后续逻辑使用。
            threshold_cm = float(
                # 中文注释：第 508 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('approach_y_threshold_cm').value
            # 中文注释：第 509 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 510 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if target.target_y_cm <= threshold_cm:
                # 中文注释：第 511 行，返回当前函数或方法的执行结果。
                return target
# 中文注释：第 512 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 513 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if approach_index >= int(
                # 中文注释：第 514 行，执行当前代码行，作为所在逻辑块的一部分。
                self.get_parameter('approach_max_attempts').value
            # 中文注释：第 515 行，执行当前代码行，作为所在逻辑块的一部分。
            ):
                # 中文注释：第 516 行，跳出当前循环。
                break
# 中文注释：第 517 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 518 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 519 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 520 行，执行当前代码行，作为所在逻辑块的一部分。
                'clean_trash: target is far, moving base forward',
                # 中文注释：第 521 行，执行当前代码行，作为所在逻辑块的一部分。
                0.20,
            # 中文注释：第 522 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 523 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._drive_forward_toward_target(goal_handle, target.target_y_cm)
# 中文注释：第 524 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 525 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
        self.get_logger().warning(
            # 中文注释：第 526 行，执行当前代码行，作为所在逻辑块的一部分。
            'target still beyond approach threshold: '
            # 中文注释：第 527 行，计算或保存局部变量 f'y，供后续逻辑使用。
            f'y={last_target.target_y_cm:.1f} cm'
        # 中文注释：第 528 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 529 行，返回当前函数或方法的执行结果。
        return None
# 中文注释：第 530 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 531 行，调用视觉抓取服务获取垃圾目标。
    def _detect_grasp_target(self, goal_handle):
        # 中文注释：第 532 行，计算或保存局部变量 retries，供后续逻辑使用。
        retries = int(self.get_parameter('detect_retry_count').value)
        # 中文注释：第 533 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for attempt in range(max(1, retries)):
            # 中文注释：第 534 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 535 行，返回当前函数或方法的执行结果。
                return None
# 中文注释：第 536 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 537 行，构造服务请求对象，准备填入调用参数。
            request = DetectGraspTarget.Request()
            # 中文注释：第 538 行，计算或保存局部变量 request.force_refresh，供后续逻辑使用。
            request.force_refresh = True
            # 中文注释：第 539 行，调用服务或读取服务响应结果。
            response = self._call_service(
                # 中文注释：第 540 行，执行当前代码行，作为所在逻辑块的一部分。
                self._detect_grasp_client,
                # 中文注释：第 541 行，执行当前代码行，作为所在逻辑块的一部分。
                request,
                # 中文注释：第 542 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
                timeout_sec=float(
                    # 中文注释：第 543 行，执行当前代码行，作为所在逻辑块的一部分。
                    self.get_parameter('service_timeout_sec').value
                # 中文注释：第 544 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                ),
                # 中文注释：第 545 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
                goal_handle=goal_handle,
                # 中文注释：第 546 行，计算或保存局部变量 label，供后续逻辑使用。
                label='detect grasp target',
            # 中文注释：第 547 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 548 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if response is not None and response.success:
                # 中文注释：第 549 行，返回当前函数或方法的执行结果。
                return GraspTarget(
                    # 中文注释：第 550 行，计算或保存局部变量 class_name，供后续逻辑使用。
                    class_name=str(response.class_name),
                    # 中文注释：第 551 行，计算或保存局部变量 confidence，供后续逻辑使用。
                    confidence=float(response.confidence),
                    # 中文注释：第 552 行，计算或保存局部变量 target_x_cm，供后续逻辑使用。
                    target_x_cm=float(response.target_x_cm),
                    # 中文注释：第 553 行，计算或保存局部变量 target_y_cm，供后续逻辑使用。
                    target_y_cm=float(response.target_y_cm),
                    # 中文注释：第 554 行，计算或保存局部变量 target_z_cm，供后续逻辑使用。
                    target_z_cm=float(response.target_z_cm),
                    # 中文注释：第 555 行，计算或保存局部变量 end_angle，供后续逻辑使用。
                    end_angle=float(response.end_angle),
                    # 中文注释：第 556 行，计算或保存局部变量 gripper_rotation，供后续逻辑使用。
                    gripper_rotation=float(response.gripper_rotation),
                    # 中文注释：第 557 行，计算或保存局部变量 gripper_close_ratio，供后续逻辑使用。
                    gripper_close_ratio=float(response.gripper_close_ratio),
                    # 中文注释：第 558 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
                    bucket_id=int(response.bucket_id),
                # 中文注释：第 559 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 560 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if response is None:
                # 中文注释：第 561 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 562 行，执行当前代码行，作为所在逻辑块的一部分。
                    'detect grasp target attempt '
                    # 中文注释：第 563 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{max(1, retries)} returned no response'
                # 中文注释：第 564 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 565 行，声明数据字段 else 及其类型。
            else:
                # 中文注释：第 566 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 567 行，执行当前代码行，作为所在逻辑块的一部分。
                    'detect grasp target attempt '
                    # 中文注释：第 568 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{max(1, retries)} failed: '
                    # 中文注释：第 569 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{response.message}'
                # 中文注释：第 570 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 571 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.15)
            # 中文注释：第 572 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 573 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 574 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_trash: detection retry {attempt + 1}',
                # 中文注释：第 575 行，执行当前代码行，作为所在逻辑块的一部分。
                0.15,
            # 中文注释：第 576 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
        # 中文注释：第 577 行，返回当前函数或方法的执行结果。
        return None
# 中文注释：第 578 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 579 行，按目标距离计算小步前进时间并驱动底盘。
    def _drive_forward_toward_target(self, goal_handle, target_y_cm):
        # 中文注释：第 580 行，计算或保存局部变量 target_y，供后续逻辑使用。
        target_y = float(self.get_parameter('approach_target_y_cm').value)
        # 中文注释：第 581 行，计算或保存局部变量 max_step，供后续逻辑使用。
        max_step = float(self.get_parameter('approach_max_step_m').value)
        # 中文注释：第 582 行，计算或保存局部变量 speed，供后续逻辑使用。
        speed = abs(float(self.get_parameter('approach_linear_speed').value))
        # 中文注释：第 583 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if speed <= 0.0:
            # 中文注释：第 584 行，结束当前函数或方法，不返回额外数据。
            return
# 中文注释：第 585 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 586 行，计算或保存局部变量 distance_m，供后续逻辑使用。
        distance_m = max(0.0, (target_y_cm - target_y) / 100.0)
        # 中文注释：第 587 行，计算或保存局部变量 distance_m，供后续逻辑使用。
        distance_m = min(distance_m, max_step)
        # 中文注释：第 588 行，计算或保存局部变量 duration，供后续逻辑使用。
        duration = distance_m / speed
# 中文注释：第 589 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 590 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + duration
        # 中文注释：第 591 行，创建底盘速度消息，后续填入线速度或角速度。
        msg = Twist()
        # 中文注释：第 592 行，计算或保存局部变量 msg.linear.x，供后续逻辑使用。
        msg.linear.x = speed
        # 中文注释：第 593 行，开始 while 循环，在条件满足期间持续执行。
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            # 中文注释：第 594 行，发布底盘速度命令。
            self._cmd_vel_pub.publish(msg)
            # 中文注释：第 595 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.05)
        # 中文注释：第 596 行，立即发布零速度，确保底盘停止。
        self._publish_stop()
        # 中文注释：第 597 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(0.2)
# 中文注释：第 598 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 599 行，控制夹爪和机械臂完成一次抓取与投放。
    def _grasp_target(self, goal_handle, target):
        # 中文注释：第 600 行，计算或保存局部变量 duration_ms，供后续逻辑使用。
        duration_ms = float(self.get_parameter('gripper_duration_ms').value)
        # 中文注释：第 601 行，计算或保存局部变量 wait_sec，供后续逻辑使用。
        wait_sec = float(self.get_parameter('gripper_wait_sec').value)
# 中文注释：第 602 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 603 行，发送夹爪开合控制命令。
        self._set_gripper(goal_handle, 1.0, duration_ms, 'open gripper')
        # 中文注释：第 604 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._send_arm_target(goal_handle, target)
        # 中文注释：第 605 行，发送夹爪开合控制命令。
        self._set_gripper(
            # 中文注释：第 606 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 607 行，执行当前代码行，作为所在逻辑块的一部分。
            target.gripper_close_ratio,
            # 中文注释：第 608 行，执行当前代码行，作为所在逻辑块的一部分。
            duration_ms,
            # 中文注释：第 609 行，执行当前代码行，作为所在逻辑块的一部分。
            'close gripper',
        # 中文注释：第 610 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 611 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(wait_sec)
# 中文注释：第 612 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 613 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
        bucket_id = target.bucket_id if target.bucket_id in (1, 2) else 1
        # 中文注释：第 614 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, f'clean_trash: move to bucket {bucket_id}', 0.70)
        # 中文注释：第 615 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, bucket_id)
        # 中文注释：第 616 行，发送夹爪开合控制命令。
        self._set_gripper(goal_handle, 1.0, duration_ms, 'release object')
        # 中文注释：第 617 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(wait_sec)
# 中文注释：第 618 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 619 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'clean_trash: arm center', 0.90)
        # 中文注释：第 620 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        # 中文注释：第 621 行，发送夹爪开合控制命令。
        self._set_gripper(goal_handle, 0.0, duration_ms, 'close gripper idle')
# 中文注释：第 622 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 623 行，执行倒垃圾总流程，按指定桶或两个桶依次处理。
    def _run_dump_trash_tree(self, goal_handle, goal):
        # 中文注释：第 624 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._load_dump_bucket_poses()
        # 中文注释：第 625 行，计算或保存局部变量 raw_bucket_id，供后续逻辑使用。
        raw_bucket_id = int(goal.bucket_id)
        # 中文注释：第 626 行，计算或保存局部变量 bucket_ids，供后续逻辑使用。
        bucket_ids = [raw_bucket_id] if raw_bucket_id in (1, 2) else [1, 2]
        # 中文注释：第 627 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for index, bucket_id in enumerate(bucket_ids):
            # 中文注释：第 628 行，接收子流程返回的成功标志和说明信息。
            success, message = self._dump_single_bucket(
                # 中文注释：第 629 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 630 行，执行当前代码行，作为所在逻辑块的一部分。
                bucket_id,
                # 中文注释：第 631 行，计算或保存局部变量 base_progress，供后续逻辑使用。
                base_progress=index / max(1, len(bucket_ids)),
                # 中文注释：第 632 行，计算或保存局部变量 span，供后续逻辑使用。
                span=1.0 / max(1, len(bucket_ids)),
            # 中文注释：第 633 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 634 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not success:
                # 中文注释：第 635 行，返回当前函数或方法的执行结果。
                return False, message
        # 中文注释：第 636 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if len(bucket_ids) == 1:
            # 中文注释：第 637 行，返回当前函数或方法的执行结果。
            return True, f'dump bucket {bucket_ids[0]} completed'
        # 中文注释：第 638 行，返回当前函数或方法的执行结果。
        return True, 'dump buckets 1 and 2 completed'
# 中文注释：第 639 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 640 行，执行单个车载桶的导航、视觉对齐、后退和倾倒流程。
    def _dump_single_bucket(self, goal_handle, bucket_id, base_progress=0.0, span=1.0):
        # 中文注释：第 641 行，计算或保存局部变量 pose，供后续逻辑使用。
        pose = self._dump_bucket_poses.get(bucket_id)
        # 中文注释：第 642 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if pose is None:
            # 中文注释：第 643 行，返回当前函数或方法的执行结果。
            return False, f'dump bucket {bucket_id} pose is not set'
# 中文注释：第 644 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 645 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 646 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 647 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: arm mapping pose for bucket {bucket_id}',
            # 中文注释：第 648 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.05,
        # 中文注释：第 649 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 650 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
# 中文注释：第 651 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 652 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 653 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 654 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: navigate to bucket {bucket_id} front',
            # 中文注释：第 655 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.10,
        # 中文注释：第 656 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 657 行，计算或保存局部变量 nav_result, nav_message，供后续逻辑使用。
        nav_result, nav_message = self._navigate_to_dump_front(goal_handle, pose)
        # 中文注释：第 658 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not nav_result:
            # 中文注释：第 659 行，返回当前函数或方法的执行结果。
            return False, nav_message
# 中文注释：第 660 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 661 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if bool(self.get_parameter('require_dump_bucket_detect').value):
            # 中文注释：第 662 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 663 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 664 行，执行当前代码行，作为所在逻辑块的一部分。
                f'dump_trash: center bucket {bucket_id}',
                # 中文注释：第 665 行，执行当前代码行，作为所在逻辑块的一部分。
                base_progress + span * 0.45,
            # 中文注释：第 666 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 667 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not self._center_dump_bucket(goal_handle):
                # 中文注释：第 668 行，计算或保存局部变量 message，供后续逻辑使用。
                message = f'dump bucket {bucket_id} visual center align failed'
                # 中文注释：第 669 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if bool(self.get_parameter('dump_visual_align_required').value):
                    # 中文注释：第 670 行，返回当前函数或方法的执行结果。
                    return False, message
                # 中文注释：第 671 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(f'{message}; continue to dump')
# 中文注释：第 672 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 673 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 674 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 675 行，执行当前代码行，作为所在逻辑块的一部分。
                f'dump_trash: depth align bucket {bucket_id}',
                # 中文注释：第 676 行，执行当前代码行，作为所在逻辑块的一部分。
                base_progress + span * 0.55,
            # 中文注释：第 677 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 678 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not self._align_dump_bucket_depth(goal_handle):
                # 中文注释：第 679 行，计算或保存局部变量 message，供后续逻辑使用。
                message = f'dump bucket {bucket_id} not detected after align'
                # 中文注释：第 680 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if bool(self.get_parameter('dump_visual_align_required').value):
                    # 中文注释：第 681 行，返回当前函数或方法的执行结果。
                    return False, message
                # 中文注释：第 682 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(f'{message}; continue to dump')
# 中文注释：第 683 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 684 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 685 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 686 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: rotate before dumping bucket {bucket_id}',
            # 中文注释：第 687 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.65,
        # 中文注释：第 688 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 689 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._rotate_in_place_180(goal_handle)
# 中文注释：第 690 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 691 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 692 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 693 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: lock onboard bucket {bucket_id}',
            # 中文注释：第 694 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.82,
        # 中文注释：第 695 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 696 行，接收子流程返回的成功标志和说明信息。
        success, message = self._call_dump_trigger(
            # 中文注释：第 697 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 698 行，执行当前代码行，作为所在逻辑块的一部分。
            self._dump_bucket_position_clients[bucket_id],
            # 中文注释：第 699 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump bucket {bucket_id} position',
        # 中文注释：第 700 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 701 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not success:
            # 中文注释：第 702 行，返回当前函数或方法的执行结果。
            return False, message
# 中文注释：第 703 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 704 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 705 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 706 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: backup bucket {bucket_id}',
            # 中文注释：第 707 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.86,
        # 中文注释：第 708 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 709 行，执行当前代码行，作为所在逻辑块的一部分。
        self._drive_backward_step(
            # 中文注释：第 710 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 711 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_backup_distance_m').value),
            # 中文注释：第 712 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_backup_linear_speed').value),
        # 中文注释：第 713 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 714 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 715 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(
            # 中文注释：第 716 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 717 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump_trash: pour bucket {bucket_id}',
            # 中文注释：第 718 行，执行当前代码行，作为所在逻辑块的一部分。
            base_progress + span * 0.90,
        # 中文注释：第 719 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 720 行，返回当前函数或方法的执行结果。
        return self._call_dump_trigger(
            # 中文注释：第 721 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 722 行，执行当前代码行，作为所在逻辑块的一部分。
            self._dump_bucket_pour_clients[bucket_id],
            # 中文注释：第 723 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump bucket {bucket_id} pour',
        # 中文注释：第 724 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 725 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 726 行，保存卸货垃圾桶在地图中的标定位置。
    def _run_set_dump_pose(self, goal_handle, goal):
        # 中文注释：第 727 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
        bucket_id = int(goal.bucket_id)
        # 中文注释：第 728 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if bucket_id not in (1, 2):
            # 中文注释：第 729 行，返回当前函数或方法的执行结果。
            return False, 'dump bucket id must be 1 or 2'
        # 中文注释：第 730 行，计算或保存局部变量 frame_id，供后续逻辑使用。
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
        # 中文注释：第 731 行，计算或保存局部变量 pose，供后续逻辑使用。
        pose = DumpBucketPose(
            # 中文注释：第 732 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
            bucket_id=bucket_id,
            # 中文注释：第 733 行，计算或保存局部变量 x，供后续逻辑使用。
            x=float(goal.x),
            # 中文注释：第 734 行，计算或保存局部变量 y，供后续逻辑使用。
            y=float(goal.y),
            # 中文注释：第 735 行，计算或保存局部变量 yaw，供后续逻辑使用。
            yaw=float(goal.yaw),
            # 中文注释：第 736 行，计算或保存局部变量 frame_id，供后续逻辑使用。
            frame_id=str(frame_id),
        # 中文注释：第 737 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 738 行，设置实例成员 self._dump_bucket_poses[bucket_id]，保存节点运行状态或 ROS 通信对象。
        self._dump_bucket_poses[bucket_id] = pose
        # 中文注释：第 739 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._save_dump_bucket_poses()
        # 中文注释：第 740 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, f'set_dump_pose: bucket {bucket_id}', 1.0)
        # 中文注释：第 741 行，返回当前函数或方法的执行结果。
        return (
            # 中文注释：第 742 行，执行当前代码行，作为所在逻辑块的一部分。
            True,
            # 中文注释：第 743 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump bucket {bucket_id} pose set: '
            # 中文注释：第 744 行，计算或保存局部变量 f'({pose.x:.3f}, {pose.y:.3f}, yaw，供后续逻辑使用。
            f'({pose.x:.3f}, {pose.y:.3f}, yaw={pose.yaw:.3f})',
        # 中文注释：第 745 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 746 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 747 行，执行区域清扫流程，生成覆盖路径并沿路识别抓取。
    def _run_clean_region_tree(self, goal_handle, goal):
        # 中文注释：第 748 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'clean_region: arm mapping pose', 0.05)
        # 中文注释：第 749 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        # 中文注释：第 750 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'clean_region: generate coverage path', 0.15)
        # 中文注释：第 751 行，计算或保存局部变量 region, error，供后续逻辑使用。
        region, error = self._resolve_clean_region(goal)
        # 中文注释：第 752 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if region is None:
            # 中文注释：第 753 行，返回当前函数或方法的执行结果。
            return False, error
# 中文注释：第 754 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 755 行，计算或保存局部变量 path, message，供后续逻辑使用。
        path, message = self._generate_coverage_path(goal_handle, region)
        # 中文注释：第 756 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if path is None:
            # 中文注释：第 757 行，返回当前函数或方法的执行结果。
            return False, message
# 中文注释：第 758 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 759 行，计算或保存局部变量 picked，供后续逻辑使用。
        picked = self._run_coverage_path_with_grasp(goal_handle, path)
        # 中文注释：第 760 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if picked is None:
            # 中文注释：第 761 行，返回当前函数或方法的执行结果。
            return False, 'clean region canceled'
        # 中文注释：第 762 行，返回当前函数或方法的执行结果。
        return True, f'clean region completed: picked={picked}'
# 中文注释：第 763 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 764 行，调用覆盖路径服务生成区域清扫路径。
    def _generate_coverage_path(self, goal_handle, region):
        # 中文注释：第 765 行，构造服务请求对象，准备填入调用参数。
        request = GenerateCoveragePath.Request()
        # 中文注释：第 766 行，计算或保存局部变量 request.min_x，供后续逻辑使用。
        request.min_x = region.min_x
        # 中文注释：第 767 行，计算或保存局部变量 request.min_y，供后续逻辑使用。
        request.min_y = region.min_y
        # 中文注释：第 768 行，计算或保存局部变量 request.max_x，供后续逻辑使用。
        request.max_x = region.max_x
        # 中文注释：第 769 行，计算或保存局部变量 request.max_y，供后续逻辑使用。
        request.max_y = region.max_y
        # 中文注释：第 770 行，计算或保存局部变量 request.frame_id，供后续逻辑使用。
        request.frame_id = region.frame_id
        # 中文注释：第 771 行，调用服务或读取服务响应结果。
        response = self._call_service(
            # 中文注释：第 772 行，执行当前代码行，作为所在逻辑块的一部分。
            self._coverage_path_client,
            # 中文注释：第 773 行，执行当前代码行，作为所在逻辑块的一部分。
            request,
            # 中文注释：第 774 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 775 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 776 行，计算或保存局部变量 label，供后续逻辑使用。
            label='generate coverage path',
        # 中文注释：第 777 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 778 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None:
            # 中文注释：第 779 行，返回当前函数或方法的执行结果。
            return None, 'coverage path service unavailable or timed out'
        # 中文注释：第 780 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not response.success:
            # 中文注释：第 781 行，返回当前函数或方法的执行结果。
            return None, response.message
        # 中文注释：第 782 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not response.coverage_path.poses:
            # 中文注释：第 783 行，返回当前函数或方法的执行结果。
            return None, 'coverage path service returned no path'
        # 中文注释：第 784 行，返回当前函数或方法的执行结果。
        return response.coverage_path, response.message
# 中文注释：第 785 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 786 行，沿覆盖路径导航，并在发现目标时中断导航进行抓取。
    def _run_coverage_path_with_grasp(self, goal_handle, path):
        # 中文注释：第 787 行，计算或保存局部变量 picked，供后续逻辑使用。
        picked = 0
        # 中文注释：第 788 行，计算或保存局部变量 total，供后续逻辑使用。
        total = len(path.poses)
        # 中文注释：第 789 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for index, pose in enumerate(path.poses):
            # 中文注释：第 790 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 791 行，返回当前函数或方法的执行结果。
                return None
# 中文注释：第 792 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 793 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 794 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 795 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_region: coverage waypoint {index + 1}/{total}',
                # 中文注释：第 796 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                0.20 + 0.70 * index / max(1, total),
            # 中文注释：第 797 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 798 行，计算或保存局部变量 target，供后续逻辑使用。
            target = self._navigate_pose_with_visual_interrupt(
                # 中文注释：第 799 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 800 行，执行当前代码行，作为所在逻辑块的一部分。
                pose,
                # 中文注释：第 801 行，执行当前代码行，作为所在逻辑块的一部分。
                f'coverage waypoint {index + 1}',
            # 中文注释：第 802 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 803 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not target:
                # 中文注释：第 804 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
# 中文注释：第 805 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 806 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 807 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 808 行，执行当前代码行，作为所在逻辑块的一部分。
                'clean_region: reacquire stopped target',
                # 中文注释：第 809 行，执行当前代码行，作为所在逻辑块的一部分。
                0.40,
            # 中文注释：第 810 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 811 行，计算或保存局部变量 target，供后续逻辑使用。
            target = self._detect_reachable_target(goal_handle)
            # 中文注释：第 812 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if target is None:
                # 中文注释：第 813 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 814 行，执行当前代码行，作为所在逻辑块的一部分。
                    'clean_region visual interrupt had no reachable target after stop'
                # 中文注释：第 815 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 816 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
# 中文注释：第 817 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 818 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 819 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 820 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_region: grasping {target.class_name}',
                # 中文注释：第 821 行，执行当前代码行，作为所在逻辑块的一部分。
                0.45,
            # 中文注释：第 822 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 823 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._grasp_target(goal_handle, target)
            # 中文注释：第 824 行，计算或保存局部变量 picked +，供后续逻辑使用。
            picked += 1
# 中文注释：第 825 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 826 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 827 行，返回当前函数或方法的执行结果。
                return None
            # 中文注释：第 828 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(goal_handle, 'clean_region: resume coverage pose', 0.60)
            # 中文注释：第 829 行，发送机械臂预设姿态命令。
            self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
# 中文注释：第 830 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 831 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 832 行，返回当前函数或方法的执行结果。
                return None
            # 中文注释：第 833 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 834 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 835 行，执行当前代码行，作为所在逻辑块的一部分。
                f'clean_region: return to waypoint {index + 1}/{total}',
                # 中文注释：第 836 行，执行当前代码行，作为所在逻辑块的一部分。
                0.62,
            # 中文注释：第 837 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 838 行，执行当前代码行，作为所在逻辑块的一部分。
            self._navigate_pose_with_visual_interrupt(
                # 中文注释：第 839 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 840 行，执行当前代码行，作为所在逻辑块的一部分。
                pose,
                # 中文注释：第 841 行，执行当前代码行，作为所在逻辑块的一部分。
                f'return coverage waypoint {index + 1}',
                # 中文注释：第 842 行，计算或保存局部变量 detect_enabled，供后续逻辑使用。
                detect_enabled=False,
            # 中文注释：第 843 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
# 中文注释：第 844 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 845 行，返回当前函数或方法的执行结果。
        return picked
# 中文注释：第 846 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 847 行，向单个导航点移动，同时按周期检测可抓取目标。
    def _navigate_pose_with_visual_interrupt(
        # 中文注释：第 848 行，执行当前代码行，作为所在逻辑块的一部分。
        self,
        # 中文注释：第 849 行，执行当前代码行，作为所在逻辑块的一部分。
        goal_handle,
        # 中文注释：第 850 行，执行当前代码行，作为所在逻辑块的一部分。
        pose,
        # 中文注释：第 851 行，执行当前代码行，作为所在逻辑块的一部分。
        label,
        # 中文注释：第 852 行，计算或保存局部变量 detect_enabled，供后续逻辑使用。
        detect_enabled=True,
    # 中文注释：第 853 行，执行当前代码行，作为所在逻辑块的一部分。
    ):
        # 中文注释：第 854 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._is_canceling(goal_handle):
            # 中文注释：第 855 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 856 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._navigate_client.wait_for_server(
            # 中文注释：第 857 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        # 中文注释：第 858 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 859 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError(f'{label} action server not available')
# 中文注释：第 860 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 861 行，构造 Nav2 导航 action 目标。
        nav_goal = NavigateToPose.Goal()
        # 中文注释：第 862 行，计算或保存局部变量 nav_goal.pose，供后续逻辑使用。
        nav_goal.pose = pose
        # 中文注释：第 863 行，计算或保存局部变量 send_future，供后续逻辑使用。
        send_future = self._navigate_client.send_goal_async(nav_goal)
        # 中文注释：第 864 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._wait_future(
            # 中文注释：第 865 行，执行当前代码行，作为所在逻辑块的一部分。
            send_future,
            # 中文注释：第 866 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 867 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
        # 中文注释：第 868 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 869 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError(f'{label} goal response timed out')
# 中文注释：第 870 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 871 行，计算或保存局部变量 nav_handle，供后续逻辑使用。
        nav_handle = send_future.result()
        # 中文注释：第 872 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if nav_handle is None or not nav_handle.accepted:
            # 中文注释：第 873 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError(f'{label} goal rejected')
# 中文注释：第 874 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 875 行，计算或保存局部变量 result_future，供后续逻辑使用。
        result_future = nav_handle.get_result_async()
        # 中文注释：第 876 行，计算或保存局部变量 next_detect，供后续逻辑使用。
        next_detect = time.monotonic()
        # 中文注释：第 877 行，计算或保存局部变量 detect_interval，供后续逻辑使用。
        detect_interval = max(
            # 中文注释：第 878 行，执行当前代码行，作为所在逻辑块的一部分。
            0.2,
            # 中文注释：第 879 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('coverage_detect_interval_sec').value),
        # 中文注释：第 880 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 881 行，计算超时截止时间，用于后续循环判断。
        deadline = (
            # 中文注释：第 882 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            time.monotonic()
            # 中文注释：第 883 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            + self._navigation_timeout_sec()
        # 中文注释：第 884 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 885 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 886 行，开始 while 循环，在条件满足期间持续执行。
        while rclpy.ok() and not result_future.done():
            # 中文注释：第 887 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 888 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                self._cancel_action_goal(nav_handle)
                # 中文注释：第 889 行，返回当前函数或方法的执行结果。
                return None
# 中文注释：第 890 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 891 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if time.monotonic() >= deadline:
                # 中文注释：第 892 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                self._cancel_action_goal(nav_handle)
                # 中文注释：第 893 行，抛出异常，让上层流程感知当前步骤失败。
                raise RuntimeError(f'{label} result timed out')
# 中文注释：第 894 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 895 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if detect_enabled and time.monotonic() >= next_detect:
                # 中文注释：第 896 行，计算或保存局部变量 target，供后续逻辑使用。
                target = self._detect_grasp_target(goal_handle)
                # 中文注释：第 897 行，计算或保存局部变量 next_detect，供后续逻辑使用。
                next_detect = time.monotonic() + detect_interval
                # 中文注释：第 898 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if target is not None:
                    # 中文注释：第 899 行，输出 info 级别日志，记录当前流程状态或关键参数。
                    self.get_logger().info(
                        # 中文注释：第 900 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'clean_region detected {target.class_name}; '
                        # 中文注释：第 901 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'interrupting {label}'
                    # 中文注释：第 902 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                    # 中文注释：第 903 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    self._cancel_action_goal(nav_handle)
                    # 中文注释：第 904 行，立即发布零速度，确保底盘停止。
                    self._publish_stop()
                    # 中文注释：第 905 行，返回当前函数或方法的执行结果。
                    return True
# 中文注释：第 906 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 907 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.05)
# 中文注释：第 908 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 909 行，计算或保存局部变量 wrapped，供后续逻辑使用。
        wrapped = result_future.result()
        # 中文注释：第 910 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            # 中文注释：第 911 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError(f'{label} failed: status={wrapped.status}')
        # 中文注释：第 912 行，返回当前函数或方法的执行结果。
        return False
# 中文注释：第 913 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 914 行，取消已经发送的 action 目标。
    def _cancel_action_goal(self, action_goal_handle):
        # 中文注释：第 915 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 916 行，取消正在等待的服务或 action，避免任务停止后继续执行。
            cancel_future = action_goal_handle.cancel_goal_async()
        # 中文注释：第 917 行，处理 try 代码块中捕获到的异常。
        except Exception:
            # 中文注释：第 918 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 919 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._wait_future(cancel_future, 2.0, None)
# 中文注释：第 920 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 921 行，保存清扫区域的地图边界。
    def _run_set_clean_region(self, goal_handle, goal):
        # 中文注释：第 922 行，计算或保存局部变量 region_id，供后续逻辑使用。
        region_id = int(goal.bucket_id)
        # 中文注释：第 923 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if region_id <= 0:
            # 中文注释：第 924 行，返回当前函数或方法的执行结果。
            return False, 'clean region id must be greater than 0'
        # 中文注释：第 925 行，计算或保存局部变量 min_x，供后续逻辑使用。
        min_x = float(goal.min_x)
        # 中文注释：第 926 行，计算或保存局部变量 min_y，供后续逻辑使用。
        min_y = float(goal.min_y)
        # 中文注释：第 927 行，计算或保存局部变量 max_x，供后续逻辑使用。
        max_x = float(goal.max_x)
        # 中文注释：第 928 行，计算或保存局部变量 max_y，供后续逻辑使用。
        max_y = float(goal.max_y)
        # 中文注释：第 929 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if min_x >= max_x or min_y >= max_y:
            # 中文注释：第 930 行，返回当前函数或方法的执行结果。
            return False, 'clean region bounds invalid'
        # 中文注释：第 931 行，计算或保存局部变量 frame_id，供后续逻辑使用。
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
        # 中文注释：第 932 行，计算或保存局部变量 region，供后续逻辑使用。
        region = CleanRegion(
            # 中文注释：第 933 行，计算或保存局部变量 region_id，供后续逻辑使用。
            region_id=region_id,
            # 中文注释：第 934 行，计算或保存局部变量 min_x，供后续逻辑使用。
            min_x=min_x,
            # 中文注释：第 935 行，计算或保存局部变量 min_y，供后续逻辑使用。
            min_y=min_y,
            # 中文注释：第 936 行，计算或保存局部变量 max_x，供后续逻辑使用。
            max_x=max_x,
            # 中文注释：第 937 行，计算或保存局部变量 max_y，供后续逻辑使用。
            max_y=max_y,
            # 中文注释：第 938 行，计算或保存局部变量 frame_id，供后续逻辑使用。
            frame_id=str(frame_id),
        # 中文注释：第 939 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 940 行，设置实例成员 self._clean_regions[region_id]，保存节点运行状态或 ROS 通信对象。
        self._clean_regions[region_id] = region
        # 中文注释：第 941 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._save_clean_regions()
        # 中文注释：第 942 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, f'set_clean_region: region {region_id}', 1.0)
        # 中文注释：第 943 行，返回当前函数或方法的执行结果。
        return (
            # 中文注释：第 944 行，执行当前代码行，作为所在逻辑块的一部分。
            True,
            # 中文注释：第 945 行，执行当前代码行，作为所在逻辑块的一部分。
            f'clean region {region_id} set: '
            # 中文注释：第 946 行，执行当前代码行，作为所在逻辑块的一部分。
            f'({min_x:.3f}, {min_y:.3f}) -> ({max_x:.3f}, {max_y:.3f})',
        # 中文注释：第 947 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 948 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 949 行，从请求参数或已保存配置中解析清扫区域。
    def _resolve_clean_region(self, goal):
        # 中文注释：第 950 行，计算或保存局部变量 min_x，供后续逻辑使用。
        min_x = float(goal.min_x)
        # 中文注释：第 951 行，计算或保存局部变量 min_y，供后续逻辑使用。
        min_y = float(goal.min_y)
        # 中文注释：第 952 行，计算或保存局部变量 max_x，供后续逻辑使用。
        max_x = float(goal.max_x)
        # 中文注释：第 953 行，计算或保存局部变量 max_y，供后续逻辑使用。
        max_y = float(goal.max_y)
        # 中文注释：第 954 行，计算或保存局部变量 frame_id，供后续逻辑使用。
        frame_id = goal.frame_id or str(self.get_parameter('map_frame').value)
# 中文注释：第 955 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 956 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if min_x < max_x and min_y < max_y:
            # 中文注释：第 957 行，返回当前函数或方法的执行结果。
            return CleanRegion(
                # 中文注释：第 958 行，计算或保存局部变量 region_id，供后续逻辑使用。
                region_id=int(goal.bucket_id) if int(goal.bucket_id) > 0 else 0,
                # 中文注释：第 959 行，计算或保存局部变量 min_x，供后续逻辑使用。
                min_x=min_x,
                # 中文注释：第 960 行，计算或保存局部变量 min_y，供后续逻辑使用。
                min_y=min_y,
                # 中文注释：第 961 行，计算或保存局部变量 max_x，供后续逻辑使用。
                max_x=max_x,
                # 中文注释：第 962 行，计算或保存局部变量 max_y，供后续逻辑使用。
                max_y=max_y,
                # 中文注释：第 963 行，计算或保存局部变量 frame_id，供后续逻辑使用。
                frame_id=str(frame_id),
            # 中文注释：第 964 行，执行当前代码行，作为所在逻辑块的一部分。
            ), None
# 中文注释：第 965 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 966 行，计算或保存局部变量 region_id，供后续逻辑使用。
        region_id = int(goal.bucket_id)
        # 中文注释：第 967 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if region_id <= 0:
            # 中文注释：第 968 行，计算或保存局部变量 region_id，供后续逻辑使用。
            region_id = 1
        # 中文注释：第 969 行，计算或保存局部变量 region，供后续逻辑使用。
        region = self._clean_regions.get(region_id)
        # 中文注释：第 970 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if region is None:
            # 中文注释：第 971 行，返回当前函数或方法的执行结果。
            return None, f'clean region {region_id} is not set'
# 中文注释：第 972 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 973 行，返回当前函数或方法的执行结果。
        return region, None
# 中文注释：第 974 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 975 行，执行单点导航任务。
    def _run_navigate_tree(self, goal_handle, goal):
        # 中文注释：第 976 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'navigate_to: arm mapping pose', 0.05)
        # 中文注释：第 977 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        # 中文注释：第 978 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'navigate_to: sending goal', 0.20)
        # 中文注释：第 979 行，构造 Nav2 导航 action 目标。
        nav_goal = NavigateToPose.Goal()
        # 中文注释：第 980 行，计算或保存局部变量 nav_goal.pose，供后续逻辑使用。
        nav_goal.pose = PoseStamped()
        # 中文注释：第 981 行，计算或保存局部变量 nav_goal.pose.header.frame_id，供后续逻辑使用。
        nav_goal.pose.header.frame_id = (
            # 中文注释：第 982 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            goal.frame_id or str(self.get_parameter('map_frame').value)
        # 中文注释：第 983 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 984 行，计算或保存局部变量 nav_goal.pose.header.stamp，供后续逻辑使用。
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        # 中文注释：第 985 行，计算或保存局部变量 nav_goal.pose.pose.position.x，供后续逻辑使用。
        nav_goal.pose.pose.position.x = float(goal.x)
        # 中文注释：第 986 行，计算或保存局部变量 nav_goal.pose.pose.position.y，供后续逻辑使用。
        nav_goal.pose.pose.position.y = float(goal.y)
        # 中文注释：第 987 行，计算或保存局部变量 nav_goal.pose.pose.position.z，供后续逻辑使用。
        nav_goal.pose.pose.position.z = 0.0
        # 中文注释：第 988 行，把 yaw 转成四元数 z/w 分量用于导航目标姿态。
        qz, qw = self._yaw_to_quaternion_z_w(float(goal.yaw))
        # 中文注释：第 989 行，计算或保存局部变量 nav_goal.pose.pose.orientation.z，供后续逻辑使用。
        nav_goal.pose.pose.orientation.z = qz
        # 中文注释：第 990 行，计算或保存局部变量 nav_goal.pose.pose.orientation.w，供后续逻辑使用。
        nav_goal.pose.pose.orientation.w = qw
# 中文注释：第 991 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 992 行，保存动作、服务或函数调用的结果。
        result = self._send_action_goal(
            # 中文注释：第 993 行，执行当前代码行，作为所在逻辑块的一部分。
            self._navigate_client,
            # 中文注释：第 994 行，执行当前代码行，作为所在逻辑块的一部分。
            nav_goal,
            # 中文注释：第 995 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=self._navigation_timeout_sec(),
            # 中文注释：第 996 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 997 行，计算或保存局部变量 label，供后续逻辑使用。
            label='navigate to pose',
        # 中文注释：第 998 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 999 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if result is None:
            # 中文注释：第 1000 行，返回当前函数或方法的执行结果。
            return False, 'navigate to pose failed'
        # 中文注释：第 1001 行，返回当前函数或方法的执行结果。
        return True, 'navigate to pose completed'
# 中文注释：第 1002 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1003 行，执行多点路径导航任务。
    def _run_navigate_path_tree(self, goal_handle, goal):
        # 中文注释：第 1004 行，计算或保存局部变量 path，供后续逻辑使用。
        path = goal.path
        # 中文注释：第 1005 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not path.poses:
            # 中文注释：第 1006 行，返回当前函数或方法的执行结果。
            return False, 'navigate path is empty'
# 中文注释：第 1007 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1008 行，向 action 客户端发布当前任务进度反馈。
        self._feedback(goal_handle, 'navigate_path: arm mapping pose', 0.05)
        # 中文注释：第 1009 行，发送机械臂预设姿态命令。
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        # 中文注释：第 1010 行，计算或保存局部变量 total，供后续逻辑使用。
        total = len(path.poses)
        # 中文注释：第 1011 行，读取或计算当前步骤使用的超时时间。
        timeout_sec = self._navigation_timeout_sec()
        # 中文注释：第 1012 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for index, pose in enumerate(path.poses, start=1):
            # 中文注释：第 1013 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 1014 行，返回当前函数或方法的执行结果。
                return False, 'navigate path canceled'
# 中文注释：第 1015 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 1016 行，计算或保存局部变量 progress，供后续逻辑使用。
            progress = 0.10 + 0.85 * ((index - 1) / max(1, total))
            # 中文注释：第 1017 行，向 action 客户端发布当前任务进度反馈。
            self._feedback(
                # 中文注释：第 1018 行，执行当前代码行，作为所在逻辑块的一部分。
                goal_handle,
                # 中文注释：第 1019 行，执行当前代码行，作为所在逻辑块的一部分。
                f'navigate_path: sending point {index}/{total}',
                # 中文注释：第 1020 行，执行当前代码行，作为所在逻辑块的一部分。
                progress,
            # 中文注释：第 1021 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 1022 行，构造 Nav2 导航 action 目标。
            nav_goal = NavigateToPose.Goal()
            # 中文注释：第 1023 行，计算或保存局部变量 nav_goal.pose，供后续逻辑使用。
            nav_goal.pose = pose
            # 中文注释：第 1024 行，计算或保存局部变量 nav_goal.pose.header.stamp，供后续逻辑使用。
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            # 中文注释：第 1025 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not nav_goal.pose.header.frame_id:
                # 中文注释：第 1026 行，计算或保存局部变量 nav_goal.pose.header.frame_id，供后续逻辑使用。
                nav_goal.pose.header.frame_id = (
                    # 中文注释：第 1027 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    goal.frame_id or str(self.get_parameter('map_frame').value)
                # 中文注释：第 1028 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
# 中文注释：第 1029 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 1030 行，计算或保存局部变量 wrapped，供后续逻辑使用。
            wrapped = self._send_action_goal_wrapped(
                # 中文注释：第 1031 行，执行当前代码行，作为所在逻辑块的一部分。
                self._navigate_client,
                # 中文注释：第 1032 行，执行当前代码行，作为所在逻辑块的一部分。
                nav_goal,
                # 中文注释：第 1033 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
                timeout_sec=timeout_sec,
                # 中文注释：第 1034 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
                goal_handle=goal_handle,
                # 中文注释：第 1035 行，计算或保存局部变量 label，供后续逻辑使用。
                label=f'navigate path point {index}/{total}',
            # 中文注释：第 1036 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 1037 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if wrapped is None:
                # 中文注释：第 1038 行，返回当前函数或方法的执行结果。
                return False, f'navigate path point {index}/{total} failed'
            # 中文注释：第 1039 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                # 中文注释：第 1040 行，返回当前函数或方法的执行结果。
                return (
                    # 中文注释：第 1041 行，执行当前代码行，作为所在逻辑块的一部分。
                    False,
                    # 中文注释：第 1042 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'navigate path point {index}/{total} failed: '
                    # 中文注释：第 1043 行，计算或保存局部变量 f'status，供后续逻辑使用。
                    f'status={wrapped.status}',
                # 中文注释：第 1044 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
# 中文注释：第 1045 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1046 行，返回当前函数或方法的执行结果。
        return True, f'navigate path completed: points={total}'
# 中文注释：第 1047 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：导航到卸货垃圾桶前方预留距离的位置。
    def _navigate_to_dump_front(self, goal_handle, pose):
        # 中文注释：第 1049 行，计算或保存局部变量 distance，供后续逻辑使用。
        distance = max(
            # 中文注释：第 1050 行，执行当前代码行，作为所在逻辑块的一部分。
            0.0,
            # 中文注释：第 1051 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_approach_distance_m').value),
        # 中文注释：第 1052 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1053 行，计算或保存局部变量 nav_yaw，供后续逻辑使用。
        nav_yaw = self._normalize_angle(pose.yaw + math.pi)
        # 中文注释：第 1054 行，计算或保存局部变量 nav_x，供后续逻辑使用。
        nav_x = pose.x + math.cos(pose.yaw) * distance
        # 中文注释：第 1055 行，计算或保存局部变量 nav_y，供后续逻辑使用。
        nav_y = pose.y + math.sin(pose.yaw) * distance
# 中文注释：第 1056 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1057 行，构造 Nav2 导航 action 目标。
        nav_goal = NavigateToPose.Goal()
        # 中文注释：第 1058 行，计算或保存局部变量 nav_goal.pose，供后续逻辑使用。
        nav_goal.pose = PoseStamped()
        # 中文注释：第 1059 行，计算或保存局部变量 nav_goal.pose.header.frame_id，供后续逻辑使用。
        nav_goal.pose.header.frame_id = pose.frame_id
        # 中文注释：第 1060 行，计算或保存局部变量 nav_goal.pose.header.stamp，供后续逻辑使用。
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        # 中文注释：第 1061 行，计算或保存局部变量 nav_goal.pose.pose.position.x，供后续逻辑使用。
        nav_goal.pose.pose.position.x = nav_x
        # 中文注释：第 1062 行，计算或保存局部变量 nav_goal.pose.pose.position.y，供后续逻辑使用。
        nav_goal.pose.pose.position.y = nav_y
        # 中文注释：第 1063 行，计算或保存局部变量 nav_goal.pose.pose.position.z，供后续逻辑使用。
        nav_goal.pose.pose.position.z = 0.0
        # 中文注释：第 1064 行，把 yaw 转成四元数 z/w 分量用于导航目标姿态。
        qz, qw = self._yaw_to_quaternion_z_w(nav_yaw)
        # 中文注释：第 1065 行，计算或保存局部变量 nav_goal.pose.pose.orientation.z，供后续逻辑使用。
        nav_goal.pose.pose.orientation.z = qz
        # 中文注释：第 1066 行，计算或保存局部变量 nav_goal.pose.pose.orientation.w，供后续逻辑使用。
        nav_goal.pose.pose.orientation.w = qw
        # 中文注释：第 1067 行，输出 info 级别日志，记录当前流程状态或关键参数。
        self.get_logger().info(
            # 中文注释：第 1068 行，执行当前代码行，作为所在逻辑块的一部分。
            f'dump bucket {pose.bucket_id} nav front goal: '
            # 中文注释：第 1069 行，计算或保存局部变量 f'({nav_x:.3f}, {nav_y:.3f}, yaw，供后续逻辑使用。
            f'({nav_x:.3f}, {nav_y:.3f}, yaw={nav_yaw:.3f}), '
            # 中文注释：第 1070 行，计算或保存局部变量 f'bucket，供后续逻辑使用。
            f'bucket=({pose.x:.3f}, {pose.y:.3f}, bucket_yaw={pose.yaw:.3f}), '
            # 中文注释：第 1071 行，计算或保存局部变量 f'offset，供后续逻辑使用。
            f'offset={distance:.3f}'
        # 中文注释：第 1072 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1073 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1074 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._is_canceling(goal_handle):
            # 中文注释：第 1075 行，返回当前函数或方法的执行结果。
            return False, f'navigate to dump bucket {pose.bucket_id} front canceled'
        # 中文注释：第 1076 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._navigate_client.wait_for_server(
            # 中文注释：第 1077 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        # 中文注释：第 1078 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 1079 行，返回当前函数或方法的执行结果。
            return False, 'navigate to dump front action server not available'
# 中文注释：第 1080 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1081 行，计算或保存局部变量 send_future，供后续逻辑使用。
        send_future = self._navigate_client.send_goal_async(nav_goal)
        # 中文注释：第 1082 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._wait_future(
            # 中文注释：第 1083 行，执行当前代码行，作为所在逻辑块的一部分。
            send_future,
            # 中文注释：第 1084 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 1085 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
        # 中文注释：第 1086 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 1087 行，返回当前函数或方法的执行结果。
            return False, 'navigate to dump front goal response timed out'
# 中文注释：第 1088 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1089 行，计算或保存局部变量 action_goal_handle，供后续逻辑使用。
        action_goal_handle = send_future.result()
        # 中文注释：第 1090 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if action_goal_handle is None or not action_goal_handle.accepted:
            # 中文注释：第 1091 行，返回当前函数或方法的执行结果。
            return False, 'navigate to dump front goal rejected'
# 中文注释：第 1092 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1093 行，计算或保存局部变量 result_future，供后续逻辑使用。
        result_future = action_goal_handle.get_result_async()
        # 中文注释：第 1094 行，读取或计算当前步骤使用的超时时间。
        timeout_sec = self._navigation_timeout_sec()
        # 中文注释：第 1095 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + timeout_sec
        # 中文注释：第 1096 行，计算或保存局部变量 check_interval，供后续逻辑使用。
        check_interval = max(
            # 中文注释：第 1097 行，执行当前代码行，作为所在逻辑块的一部分。
            0.1,
            # 中文注释：第 1098 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_check_interval_sec').value),
        # 中文注释：第 1099 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1100 行，计算或保存局部变量 next_bucket_check，供后续逻辑使用。
        next_bucket_check = time.monotonic()
# 中文注释：第 1101 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1102 行，开始 while 循环，在条件满足期间持续执行。
        while rclpy.ok() and not result_future.done():
            # 中文注释：第 1103 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 1104 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                self._cancel_action_goal(action_goal_handle)
                # 中文注释：第 1105 行，返回当前函数或方法的执行结果。
                return False, f'navigate to dump bucket {pose.bucket_id} front canceled'
            # 中文注释：第 1106 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if time.monotonic() >= deadline:
                # 中文注释：第 1107 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                self._cancel_action_goal(action_goal_handle)
                # 中文注释：第 1108 行，返回当前函数或方法的执行结果。
                return False, f'navigate to dump bucket {pose.bucket_id} front timed out'
# 中文注释：第 1109 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 1110 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if time.monotonic() >= next_bucket_check:
                # 中文注释：第 1111 行，计算或保存局部变量 next_bucket_check，供后续逻辑使用。
                next_bucket_check = time.monotonic() + check_interval
                # 中文注释：第 1112 行，计算或保存局部变量 accepted, reason，供后续逻辑使用。
                accepted, reason = self._scan_camera_obstacle_matches_dump_bucket(
                    # 中文注释：第 1113 行，执行当前代码行，作为所在逻辑块的一部分。
                    goal_handle,
                    # 中文注释：第 1114 行，执行当前代码行，作为所在逻辑块的一部分。
                    pose,
                # 中文注释：第 1115 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 1116 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if accepted:
                    # 中文注释：第 1117 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                    self.get_logger().warning(
                        # 中文注释：第 1118 行，执行当前代码行，作为所在逻辑块的一部分。
                        'scan_camera obstacle matches dump bucket; '
                        # 中文注释：第 1119 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'cancel nav and continue to visual alignment: {reason}'
                    # 中文注释：第 1120 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                    # 中文注释：第 1121 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    self._cancel_action_goal(action_goal_handle)
                    # 中文注释：第 1122 行，立即发布零速度，确保底盘停止。
                    self._publish_stop()
                    # 中文注释：第 1123 行，返回当前函数或方法的执行结果。
                    return (
                        # 中文注释：第 1124 行，执行当前代码行，作为所在逻辑块的一部分。
                        True,
                        # 中文注释：第 1125 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'dump bucket {pose.bucket_id} found by scan_camera obstacle',
                    # 中文注释：第 1126 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                # 中文注释：第 1127 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if reason:
                    # 中文注释：第 1128 行，输出 debug 级别日志，记录较细的诊断信息。
                    self.get_logger().debug(f'dump scan_camera check skipped: {reason}')
# 中文注释：第 1129 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 1130 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.05)
# 中文注释：第 1131 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1132 行，计算或保存局部变量 wrapped，供后续逻辑使用。
        wrapped = result_future.result()
        # 中文注释：第 1133 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            # 中文注释：第 1134 行，返回当前函数或方法的执行结果。
            return (
                # 中文注释：第 1135 行，执行当前代码行，作为所在逻辑块的一部分。
                False,
                # 中文注释：第 1136 行，执行当前代码行，作为所在逻辑块的一部分。
                f'navigate to dump bucket {pose.bucket_id} front failed: '
                # 中文注释：第 1137 行，计算或保存局部变量 f'status，供后续逻辑使用。
                f'status={wrapped.status}',
            # 中文注释：第 1138 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
        # 中文注释：第 1139 行，返回当前函数或方法的执行结果。
        return True, f'navigated to dump bucket {pose.bucket_id} front'
# 中文注释：第 1140 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1141 行，返回仍在有效时间内的 scan_camera 障碍物。
    def _recent_scan_camera_obstacle(self):
        # 中文注释：第 1142 行，计算或保存局部变量 obstacle，供后续逻辑使用。
        obstacle = self._latest_scan_camera_obstacle
        # 中文注释：第 1143 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if obstacle is None:
            # 中文注释：第 1144 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1145 行，计算或保存局部变量 stale_sec，供后续逻辑使用。
        stale_sec = max(
            # 中文注释：第 1146 行，执行当前代码行，作为所在逻辑块的一部分。
            0.0,
            # 中文注释：第 1147 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_obstacle_stale_sec').value),
        # 中文注释：第 1148 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1149 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if time.monotonic() - obstacle.seen_at > stale_sec:
            # 中文注释：第 1150 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1151 行，返回当前函数或方法的执行结果。
        return obstacle
# 中文注释：第 1152 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1153 行，结合雷达障碍物、视觉检测和地图位姿判断是否遇到目标卸货桶。
    def _scan_camera_obstacle_matches_dump_bucket(self, goal_handle, pose):
        # 中文注释：第 1154 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not bool(
            # 中文注释：第 1155 行，执行当前代码行，作为所在逻辑块的一部分。
            self.get_parameter('dump_scan_camera_bucket_check_enabled').value
        # 中文注释：第 1156 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 1157 行，返回当前函数或方法的执行结果。
            return False, ''
        # 中文注释：第 1158 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not bool(self.get_parameter('require_dump_bucket_detect').value):
            # 中文注释：第 1159 行，返回当前函数或方法的执行结果。
            return False, ''
        # 中文注释：第 1160 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._is_canceling(goal_handle):
            # 中文注释：第 1161 行，返回当前函数或方法的执行结果。
            return False, 'task canceling'
# 中文注释：第 1162 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1163 行，计算或保存局部变量 obstacle，供后续逻辑使用。
        obstacle = self._recent_scan_camera_obstacle()
        # 中文注释：第 1164 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if obstacle is None:
            # 中文注释：第 1165 行，返回当前函数或方法的执行结果。
            return False, ''
# 中文注释：第 1166 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1167 行，计算或保存局部变量 detection，供后续逻辑使用。
        detection = self._detect_dump_bucket(goal_handle)
        # 中文注释：第 1168 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if detection is None:
            # 中文注释：第 1169 行，返回当前函数或方法的执行结果。
            return False, 'dump bucket color detection unavailable'
        # 中文注释：第 1170 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not detection.detected:
            # 中文注释：第 1171 行，返回当前函数或方法的执行结果。
            return False, f'scan obstacle is not color bucket: {detection.message}'
# 中文注释：第 1172 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1173 行，计算或保存局部变量 accepted, reason，供后续逻辑使用。
        accepted, reason = self._dump_detection_matches_pose(detection, pose)
        # 中文注释：第 1174 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not accepted:
            # 中文注释：第 1175 行，返回当前函数或方法的执行结果。
            return False, reason
        # 中文注释：第 1176 行，返回当前函数或方法的执行结果。
        return (
            # 中文注释：第 1177 行，执行当前代码行，作为所在逻辑块的一部分。
            True,
            # 中文注释：第 1178 行，计算或保存局部变量 f'scan_range，供后续逻辑使用。
            f'scan_range={obstacle.range_m:.3f}, '
            # 中文注释：第 1179 行，计算或保存局部变量 f'scan_bearing，供后续逻辑使用。
            f'scan_bearing={obstacle.bearing_rad:.3f}; {reason}',
        # 中文注释：第 1180 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1181 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1182 行，把视觉深度和像素偏移投影到地图坐标并匹配标定桶位置。
    def _dump_detection_matches_pose(self, detection, pose):
        # 中文注释：第 1183 行，计算或保存局部变量 robot_pose，供后续逻辑使用。
        robot_pose = self._latest_map_pose
        # 中文注释：第 1184 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if robot_pose is None:
            # 中文注释：第 1185 行，返回当前函数或方法的执行结果。
            return False, 'map pose unavailable'
        # 中文注释：第 1186 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not math.isfinite(detection.mean_depth_m) or detection.mean_depth_m <= 0.0:
            # 中文注释：第 1187 行，返回当前函数或方法的执行结果。
            return False, f'invalid depth={detection.mean_depth_m}'
        # 中文注释：第 1188 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not math.isfinite(detection.offset_x_ratio):
            # 中文注释：第 1189 行，返回当前函数或方法的执行结果。
            return False, f'invalid offset={detection.offset_x_ratio}'
# 中文注释：第 1190 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1191 行，计算或保存局部变量 hfov，供后续逻辑使用。
        hfov = abs(float(self.get_parameter('dump_camera_horizontal_fov_rad').value))
        # 中文注释：第 1192 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if hfov <= 0.0:
            # 中文注释：第 1193 行，返回当前函数或方法的执行结果。
            return False, f'invalid camera hfov={hfov}'
# 中文注释：第 1194 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1195 行，计算或保存局部变量 bearing，供后续逻辑使用。
        bearing = math.atan(math.tan(hfov * 0.5) * detection.offset_x_ratio)
        # 中文注释：第 1196 行，计算或保存局部变量 bucket_x，供后续逻辑使用。
        bucket_x = robot_pose.x + detection.mean_depth_m * math.cos(
            # 中文注释：第 1197 行，执行当前代码行，作为所在逻辑块的一部分。
            robot_pose.yaw + bearing
        # 中文注释：第 1198 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1199 行，计算或保存局部变量 bucket_y，供后续逻辑使用。
        bucket_y = robot_pose.y + detection.mean_depth_m * math.sin(
            # 中文注释：第 1200 行，执行当前代码行，作为所在逻辑块的一部分。
            robot_pose.yaw + bearing
        # 中文注释：第 1201 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1202 行，计算或保存局部变量 bucket_error，供后续逻辑使用。
        bucket_error = math.hypot(bucket_x - pose.x, bucket_y - pose.y)
        # 中文注释：第 1203 行，计算或保存局部变量 bucket_tolerance，供后续逻辑使用。
        bucket_tolerance = abs(
            # 中文注释：第 1204 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_scan_camera_bucket_map_tolerance_m').value)
        # 中文注释：第 1205 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1206 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1207 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if bucket_error <= bucket_tolerance:
            # 中文注释：第 1208 行，返回当前函数或方法的执行结果。
            return (
                # 中文注释：第 1209 行，执行当前代码行，作为所在逻辑块的一部分。
                True,
                # 中文注释：第 1210 行，计算或保存局部变量 f'est_bucket，供后续逻辑使用。
                f'est_bucket=({bucket_x:.3f}, {bucket_y:.3f}), '
                # 中文注释：第 1211 行，计算或保存局部变量 f'set_bucket，供后续逻辑使用。
                f'set_bucket=({pose.x:.3f}, {pose.y:.3f}), '
                # 中文注释：第 1212 行，计算或保存局部变量 f'bucket_error，供后续逻辑使用。
                f'bucket_error={bucket_error:.3f}',
            # 中文注释：第 1213 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
# 中文注释：第 1214 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1215 行，返回当前函数或方法的执行结果。
        return (
            # 中文注释：第 1216 行，执行当前代码行，作为所在逻辑块的一部分。
            False,
            # 中文注释：第 1217 行，计算或保存局部变量 f'est_bucket，供后续逻辑使用。
            f'est_bucket=({bucket_x:.3f}, {bucket_y:.3f}), '
            # 中文注释：第 1218 行，计算或保存局部变量 f'set_bucket，供后续逻辑使用。
            f'set_bucket=({pose.x:.3f}, {pose.y:.3f}), '
            # 中文注释：第 1219 行，计算或保存局部变量 f'bucket_error，供后续逻辑使用。
            f'bucket_error={bucket_error:.3f}>{bucket_tolerance:.3f}',
        # 中文注释：第 1220 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1221 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1222 行，调用视觉服务检测卸货垃圾桶。
    def _detect_dump_bucket(self, goal_handle):
        # 中文注释：第 1223 行，构造服务请求对象，准备填入调用参数。
        request = DetectDumpBucket.Request()
        # 中文注释：第 1224 行，计算或保存局部变量 request.force_refresh，供后续逻辑使用。
        request.force_refresh = True
        # 中文注释：第 1225 行，调用服务或读取服务响应结果。
        response = self._call_service(
            # 中文注释：第 1226 行，执行当前代码行，作为所在逻辑块的一部分。
            self._detect_dump_client,
            # 中文注释：第 1227 行，执行当前代码行，作为所在逻辑块的一部分。
            request,
            # 中文注释：第 1228 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 1229 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1230 行，计算或保存局部变量 label，供后续逻辑使用。
            label='detect dump bucket',
        # 中文注释：第 1231 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1232 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None:
            # 中文注释：第 1233 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1234 行，返回当前函数或方法的执行结果。
        return DumpBucketDetection(
            # 中文注释：第 1235 行，计算或保存局部变量 detected，供后续逻辑使用。
            detected=bool(response.detected),
            # 中文注释：第 1236 行，执行当前代码行，作为所在逻辑块的一部分。
            within_depth=int(response.signal) == 1,
            # 中文注释：第 1237 行，计算或保存局部变量 mask_pixels，供后续逻辑使用。
            mask_pixels=int(response.mask_pixels),
            # 中文注释：第 1238 行，计算或保存局部变量 mean_depth_m，供后续逻辑使用。
            mean_depth_m=float(response.mean_depth_m),
            # 中文注释：第 1239 行，计算或保存局部变量 offset_x_px，供后续逻辑使用。
            offset_x_px=float(response.offset_x_px),
            # 中文注释：第 1240 行，计算或保存局部变量 offset_x_ratio，供后续逻辑使用。
            offset_x_ratio=float(response.offset_x_ratio),
            # 中文注释：第 1241 行，计算或保存局部变量 message，供后续逻辑使用。
            message=str(response.message),
        # 中文注释：第 1242 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1243 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1244 行，通过底盘旋转让视觉检测到的桶中心对齐图像中心。
    def _center_dump_bucket(self, goal_handle):
        # 中文注释：第 1245 行，计算或保存局部变量 attempts，供后续逻辑使用。
        attempts = max(1, int(self.get_parameter('dump_center_max_attempts').value))
        # 中文注释：第 1246 行，计算或保存局部变量 tolerance，供后续逻辑使用。
        tolerance = abs(float(self.get_parameter('dump_center_tolerance_ratio').value))
        # 中文注释：第 1247 行，计算或保存局部变量 speed，供后续逻辑使用。
        speed = abs(float(self.get_parameter('dump_center_angular_speed').value))
        # 中文注释：第 1248 行，计算或保存局部变量 step_sec，供后续逻辑使用。
        step_sec = max(0.05, float(self.get_parameter('dump_center_step_sec').value))
        # 中文注释：第 1249 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if speed <= 0.0:
            # 中文注释：第 1250 行，返回当前函数或方法的执行结果。
            return False
# 中文注释：第 1251 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1252 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for attempt in range(attempts):
            # 中文注释：第 1253 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 1254 行，返回当前函数或方法的执行结果。
                return False
            # 中文注释：第 1255 行，计算或保存局部变量 detection，供后续逻辑使用。
            detection = self._detect_dump_bucket(goal_handle)
            # 中文注释：第 1256 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if detection is None or not detection.detected:
                # 中文注释：第 1257 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if detection is None:
                    # 中文注释：第 1258 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                    self.get_logger().warning(
                        # 中文注释：第 1259 行，执行当前代码行，作为所在逻辑块的一部分。
                        'dump center detect '
                        # 中文注释：第 1260 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'{attempt + 1}/{attempts}: no response'
                    # 中文注释：第 1261 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                # 中文注释：第 1262 行，声明数据字段 else 及其类型。
                else:
                    # 中文注释：第 1263 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                    self.get_logger().warning(
                        # 中文注释：第 1264 行，执行当前代码行，作为所在逻辑块的一部分。
                        'dump center detect '
                        # 中文注释：第 1265 行，执行当前代码行，作为所在逻辑块的一部分。
                        f'{attempt + 1}/{attempts}: not detected, '
                        # 中文注释：第 1266 行，计算或保存局部变量 f'mask_pixels，供后续逻辑使用。
                        f'mask_pixels={detection.mask_pixels}, '
                        # 中文注释：第 1267 行，计算或保存局部变量 f'mean_depth_m，供后续逻辑使用。
                        f'mean_depth_m={detection.mean_depth_m:.3f}, '
                        # 中文注释：第 1268 行，计算或保存局部变量 f'offset_x_ratio，供后续逻辑使用。
                        f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                        # 中文注释：第 1269 行，计算或保存局部变量 f'message，供后续逻辑使用。
                        f'message={detection.message}'
                    # 中文注释：第 1270 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                # 中文注释：第 1271 行，立即发布零速度，确保底盘停止。
                self._publish_stop()
                # 中文注释：第 1272 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
                time.sleep(0.15)
                # 中文注释：第 1273 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
            # 中文注释：第 1274 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if not math.isfinite(detection.offset_x_ratio):
                # 中文注释：第 1275 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 1276 行，执行当前代码行，作为所在逻辑块的一部分。
                    'dump center detect '
                    # 中文注释：第 1277 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{attempts}: invalid offset, '
                    # 中文注释：第 1278 行，计算或保存局部变量 f'mask_pixels，供后续逻辑使用。
                    f'mask_pixels={detection.mask_pixels}, '
                    # 中文注释：第 1279 行，计算或保存局部变量 f'mean_depth_m，供后续逻辑使用。
                    f'mean_depth_m={detection.mean_depth_m:.3f}, '
                    # 中文注释：第 1280 行，计算或保存局部变量 f'offset_x_ratio，供后续逻辑使用。
                    f'offset_x_ratio={detection.offset_x_ratio}, '
                    # 中文注释：第 1281 行，计算或保存局部变量 f'message，供后续逻辑使用。
                    f'message={detection.message}'
                # 中文注释：第 1282 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 1283 行，返回当前函数或方法的执行结果。
                return False
            # 中文注释：第 1284 行，输出 info 级别日志，记录当前流程状态或关键参数。
            self.get_logger().info(
                # 中文注释：第 1285 行，执行当前代码行，作为所在逻辑块的一部分。
                'dump center detect '
                # 中文注释：第 1286 行，执行当前代码行，作为所在逻辑块的一部分。
                f'{attempt + 1}/{attempts}: detected, '
                # 中文注释：第 1287 行，计算或保存局部变量 f'mask_pixels，供后续逻辑使用。
                f'mask_pixels={detection.mask_pixels}, '
                # 中文注释：第 1288 行，计算或保存局部变量 f'mean_depth_m，供后续逻辑使用。
                f'mean_depth_m={detection.mean_depth_m:.3f}, '
                # 中文注释：第 1289 行，计算或保存局部变量 f'offset_x_ratio，供后续逻辑使用。
                f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                # 中文注释：第 1290 行，计算或保存局部变量 f'tolerance，供后续逻辑使用。
                f'tolerance={tolerance:.4f}, '
                # 中文注释：第 1291 行，计算或保存局部变量 f'message，供后续逻辑使用。
                f'message={detection.message}'
            # 中文注释：第 1292 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 1293 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if abs(detection.offset_x_ratio) <= tolerance:
                # 中文注释：第 1294 行，返回当前函数或方法的执行结果。
                return True
# 中文注释：第 1295 行，空行，用来分隔相邻逻辑块，提升代码可读性。

            # 中文注释：第 1296 行，创建底盘速度消息，后续填入线速度或角速度。
            twist = Twist()
            # 中文注释：第 1297 行，保留原代码中的注释，说明下面逻辑的意图或约束。
            # 图像中心在左侧时 offset_x 为负，向左旋转；反之向右旋转。
            # 中文注释：第 1298 行，计算或保存局部变量 twist.angular.z，供后续逻辑使用。
            twist.angular.z = -speed if detection.offset_x_ratio > 0.0 else speed
            # 中文注释：第 1299 行，输出 info 级别日志，记录当前流程状态或关键参数。
            self.get_logger().info(
                # 中文注释：第 1300 行，执行当前代码行，作为所在逻辑块的一部分。
                'dump center rotate step: '
                # 中文注释：第 1301 行，计算或保存局部变量 f'angular_z，供后续逻辑使用。
                f'angular_z={twist.angular.z:.3f}, step_sec={step_sec:.3f}'
            # 中文注释：第 1302 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 1303 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            self._publish_twist_for(goal_handle, twist, step_sec)
            # 中文注释：第 1304 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.12)
        # 中文注释：第 1305 行，返回当前函数或方法的执行结果。
        return False
# 中文注释：第 1306 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1307 行，通过小步前进把卸货桶深度调整到目标距离。
    def _align_dump_bucket_depth(self, goal_handle):
        # 中文注释：第 1308 行，计算或保存局部变量 attempts，供后续逻辑使用。
        attempts = max(1, int(self.get_parameter('dump_align_max_attempts').value))
        # 中文注释：第 1309 行，计算或保存局部变量 target_depth，供后续逻辑使用。
        target_depth = max(
            # 中文注释：第 1310 行，执行当前代码行，作为所在逻辑块的一部分。
            0.0,
            # 中文注释：第 1311 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_align_target_depth_m').value),
        # 中文注释：第 1312 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1313 行，计算或保存局部变量 tolerance，供后续逻辑使用。
        tolerance = abs(
            # 中文注释：第 1314 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('dump_align_depth_tolerance_m').value)
        # 中文注释：第 1315 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1316 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for attempt in range(attempts):
            # 中文注释：第 1317 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 1318 行，返回当前函数或方法的执行结果。
                return False
            # 中文注释：第 1319 行，计算或保存局部变量 detection，供后续逻辑使用。
            detection = self._detect_dump_bucket(goal_handle)
            # 中文注释：第 1320 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if (
                # 中文注释：第 1321 行，执行当前代码行，作为所在逻辑块的一部分。
                detection is not None
                # 中文注释：第 1322 行，执行当前代码行，作为所在逻辑块的一部分。
                and detection.detected
                # 中文注释：第 1323 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                and math.isfinite(detection.mean_depth_m)
                # 中文注释：第 1324 行，执行当前代码行，作为所在逻辑块的一部分。
                and detection.mean_depth_m > 0.0
                # 中文注释：第 1325 行，执行当前代码行，作为所在逻辑块的一部分。
                and detection.mean_depth_m <= target_depth + tolerance
            # 中文注释：第 1326 行，执行当前代码行，作为所在逻辑块的一部分。
            ):
                # 中文注释：第 1327 行，输出 info 级别日志，记录当前流程状态或关键参数。
                self.get_logger().info(
                    # 中文注释：第 1328 行，执行当前代码行，作为所在逻辑块的一部分。
                    'dump depth align detect '
                    # 中文注释：第 1329 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{attempts}: reached target depth, '
                    # 中文注释：第 1330 行，计算或保存局部变量 f'mask_pixels，供后续逻辑使用。
                    f'mask_pixels={detection.mask_pixels}, '
                    # 中文注释：第 1331 行，计算或保存局部变量 f'mean_depth_m，供后续逻辑使用。
                    f'mean_depth_m={detection.mean_depth_m:.3f}, '
                    # 中文注释：第 1332 行，计算或保存局部变量 f'target_depth_m，供后续逻辑使用。
                    f'target_depth_m={target_depth:.3f}, '
                    # 中文注释：第 1333 行，计算或保存局部变量 f'offset_x_ratio，供后续逻辑使用。
                    f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                    # 中文注释：第 1334 行，计算或保存局部变量 f'message，供后续逻辑使用。
                    f'message={detection.message}'
                # 中文注释：第 1335 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 1336 行，返回当前函数或方法的执行结果。
                return True
            # 中文注释：第 1337 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if detection is None:
                # 中文注释：第 1338 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 1339 行，执行当前代码行，作为所在逻辑块的一部分。
                    'dump depth align detect '
                    # 中文注释：第 1340 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{attempts}: no response'
                # 中文注释：第 1341 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 1342 行，声明数据字段 else 及其类型。
            else:
                # 中文注释：第 1343 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(
                    # 中文注释：第 1344 行，执行当前代码行，作为所在逻辑块的一部分。
                    'dump depth align detect '
                    # 中文注释：第 1345 行，执行当前代码行，作为所在逻辑块的一部分。
                    f'{attempt + 1}/{attempts}: not within depth, '
                    # 中文注释：第 1346 行，计算或保存局部变量 f'detected，供后续逻辑使用。
                    f'detected={detection.detected}, '
                    # 中文注释：第 1347 行，计算或保存局部变量 f'within_depth，供后续逻辑使用。
                    f'within_depth={detection.within_depth}, '
                    # 中文注释：第 1348 行，计算或保存局部变量 f'mask_pixels，供后续逻辑使用。
                    f'mask_pixels={detection.mask_pixels}, '
                    # 中文注释：第 1349 行，计算或保存局部变量 f'mean_depth_m，供后续逻辑使用。
                    f'mean_depth_m={detection.mean_depth_m:.3f}, '
                    # 中文注释：第 1350 行，计算或保存局部变量 f'target_depth_m，供后续逻辑使用。
                    f'target_depth_m={target_depth:.3f}, '
                    # 中文注释：第 1351 行，计算或保存局部变量 f'offset_x_ratio，供后续逻辑使用。
                    f'offset_x_ratio={detection.offset_x_ratio:.4f}, '
                    # 中文注释：第 1352 行，计算或保存局部变量 f'message，供后续逻辑使用。
                    f'message={detection.message}'
                # 中文注释：第 1353 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 1354 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if attempt < attempts - 1:
                # 中文注释：第 1355 行，计算或保存局部变量 step_m，供后续逻辑使用。
                step_m = float(self.get_parameter('dump_align_step_m').value)
                # 中文注释：第 1356 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if detection is not None and math.isfinite(detection.mean_depth_m):
                    # 中文注释：第 1357 行，计算或保存局部变量 remaining_m，供后续逻辑使用。
                    remaining_m = detection.mean_depth_m - target_depth
                    # 中文注释：第 1358 行，计算或保存局部变量 step_m，供后续逻辑使用。
                    step_m = min(step_m, max(0.0, remaining_m))
                # 中文注释：第 1359 行，执行当前代码行，作为所在逻辑块的一部分。
                self._drive_forward_step(
                    # 中文注释：第 1360 行，执行当前代码行，作为所在逻辑块的一部分。
                    goal_handle,
                    # 中文注释：第 1361 行，执行当前代码行，作为所在逻辑块的一部分。
                    step_m,
                    # 中文注释：第 1362 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    float(self.get_parameter('dump_align_linear_speed').value),
                # 中文注释：第 1363 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
        # 中文注释：第 1364 行，返回当前函数或方法的执行结果。
        return False
# 中文注释：第 1365 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1366 行，按固定距离和速度驱动底盘前进。
    def _drive_forward_step(self, goal_handle, distance_m, speed):
        # 中文注释：第 1367 行，计算或保存局部变量 speed，供后续逻辑使用。
        speed = abs(float(speed))
        # 中文注释：第 1368 行，计算或保存局部变量 distance_m，供后续逻辑使用。
        distance_m = max(0.0, float(distance_m))
        # 中文注释：第 1369 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if speed <= 0.0 or distance_m <= 0.0:
            # 中文注释：第 1370 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1371 行，创建底盘速度消息，后续填入线速度或角速度。
        msg = Twist()
        # 中文注释：第 1372 行，计算或保存局部变量 msg.linear.x，供后续逻辑使用。
        msg.linear.x = speed
        # 中文注释：第 1373 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._publish_twist_for(goal_handle, msg, distance_m / speed)
        # 中文注释：第 1374 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(0.15)
# 中文注释：第 1375 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1376 行，按固定距离和速度驱动底盘后退。
    def _drive_backward_step(self, goal_handle, distance_m, speed):
        # 中文注释：第 1377 行，计算或保存局部变量 speed，供后续逻辑使用。
        speed = abs(float(speed))
        # 中文注释：第 1378 行，计算或保存局部变量 distance_m，供后续逻辑使用。
        distance_m = max(0.0, float(distance_m))
        # 中文注释：第 1379 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if speed <= 0.0 or distance_m <= 0.0:
            # 中文注释：第 1380 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1381 行，创建底盘速度消息，后续填入线速度或角速度。
        msg = Twist()
        # 中文注释：第 1382 行，计算或保存局部变量 msg.linear.x，供后续逻辑使用。
        msg.linear.x = -speed
        # 中文注释：第 1383 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._publish_twist_for(goal_handle, msg, distance_m / speed)
        # 中文注释：第 1384 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(0.15)
# 中文注释：第 1385 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1386 行，调用一个 Trigger 类型的倒桶硬件服务。
    def _call_dump_trigger(self, goal_handle, client, label):
        # 中文注释：第 1387 行，调用服务或读取服务响应结果。
        response = self._call_service(
            # 中文注释：第 1388 行，执行当前代码行，作为所在逻辑块的一部分。
            client,
            # 中文注释：第 1389 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            Trigger.Request(),
            # 中文注释：第 1390 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('dump_timeout_sec').value),
            # 中文注释：第 1391 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1392 行，计算或保存局部变量 label，供后续逻辑使用。
            label=label,
        # 中文注释：第 1393 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1394 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None:
            # 中文注释：第 1395 行，返回当前函数或方法的执行结果。
            return False, f'{label} service unavailable or timed out'
        # 中文注释：第 1396 行，返回当前函数或方法的执行结果。
        return bool(response.success), response.message
# 中文注释：第 1397 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1398 行，根据 odom 或 IMU 航向控制底盘原地旋转 180 度。
    def _rotate_in_place_180(self, goal_handle):
        # 中文注释：第 1399 行，计算或保存局部变量 speed，供后续逻辑使用。
        speed = abs(float(self.get_parameter('dump_rotate_angular_speed').value))
        # 中文注释：第 1400 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if speed <= 0.0:
            # 中文注释：第 1401 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1402 行，计算或保存局部变量 start_yaw，供后续逻辑使用。
        start_yaw = self._current_yaw()
        # 中文注释：第 1403 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if start_yaw is None:
            # 中文注释：第 1404 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning('no odom/imu yaw available; skip 180deg rotation')
            # 中文注释：第 1405 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1406 行，计算或保存局部变量 target_yaw，供后续逻辑使用。
        target_yaw = self._normalize_angle(start_yaw + math.pi)
        # 中文注释：第 1407 行，计算或保存局部变量 tolerance，供后续逻辑使用。
        tolerance = abs(float(self.get_parameter('dump_rotate_tolerance_rad').value))
        # 中文注释：第 1408 行，读取或计算当前步骤使用的超时时间。
        timeout_sec = max(1.0, float(self.get_parameter('dump_rotate_timeout_sec').value))
        # 中文注释：第 1409 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + timeout_sec
# 中文注释：第 1410 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1411 行，创建底盘速度消息，后续填入线速度或角速度。
        msg = Twist()
        # 中文注释：第 1412 行，开始 while 循环，在条件满足期间持续执行。
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            # 中文注释：第 1413 行，计算或保存局部变量 current_yaw，供后续逻辑使用。
            current_yaw = self._current_yaw()
            # 中文注释：第 1414 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if current_yaw is None:
                # 中文注释：第 1415 行，跳出当前循环。
                break
            # 中文注释：第 1416 行，计算或保存局部变量 error，供后续逻辑使用。
            error = self._normalize_angle(target_yaw - current_yaw)
            # 中文注释：第 1417 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if abs(error) <= tolerance:
                # 中文注释：第 1418 行，跳出当前循环。
                break
            # 中文注释：第 1419 行，计算或保存局部变量 msg.angular.z，供后续逻辑使用。
            msg.angular.z = speed if error > 0.0 else -speed
            # 中文注释：第 1420 行，发布底盘速度命令。
            self._cmd_vel_pub.publish(msg)
            # 中文注释：第 1421 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.05)
        # 中文注释：第 1422 行，立即发布零速度，确保底盘停止。
        self._publish_stop()
        # 中文注释：第 1423 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
        time.sleep(0.2)
# 中文注释：第 1424 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1425 行，在指定时长内持续发布速度命令。
    def _publish_twist_for(self, goal_handle, msg, duration_sec):
        # 中文注释：第 1426 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        # 中文注释：第 1427 行，开始 while 循环，在条件满足期间持续执行。
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            # 中文注释：第 1428 行，发布底盘速度命令。
            self._cmd_vel_pub.publish(msg)
            # 中文注释：第 1429 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.05)
        # 中文注释：第 1430 行，立即发布零速度，确保底盘停止。
        self._publish_stop()
# 中文注释：第 1431 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1432 行，读取当前可用的航向角，优先使用里程计。
    def _current_yaw(self):
        # 中文注释：第 1433 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._latest_odom_yaw is not None:
            # 中文注释：第 1434 行，返回当前函数或方法的执行结果。
            return self._latest_odom_yaw
        # 中文注释：第 1435 行，返回当前函数或方法的执行结果。
        return self._latest_imu_yaw
# 中文注释：第 1436 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1437 行，从 JSON 文件加载卸货桶地图位姿。
    def _load_dump_bucket_poses(self):
        # 中文注释：第 1438 行，计算或保存局部变量 path，供后续逻辑使用。
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        # 中文注释：第 1439 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not path.exists():
            # 中文注释：第 1440 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1441 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1442 行，计算或保存局部变量 raw，供后续逻辑使用。
            raw = json.loads(path.read_text(encoding='utf-8'))
        # 中文注释：第 1443 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1444 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'load dump bucket poses failed: {exc}')
            # 中文注释：第 1445 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1446 行，计算或保存局部变量 poses，供后续逻辑使用。
        poses = raw.get('dump_bucket_poses', raw)
        # 中文注释：第 1447 行，计算或保存局部变量 loaded，供后续逻辑使用。
        loaded = {}
        # 中文注释：第 1448 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for key, item in poses.items():
            # 中文注释：第 1449 行，声明数据字段 try 及其类型。
            try:
                # 中文注释：第 1450 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
                bucket_id = int(item.get('bucket_id', key))
                # 中文注释：第 1451 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if bucket_id not in (1, 2):
                    # 中文注释：第 1452 行，跳过本次循环剩余逻辑，进入下一轮循环。
                    continue
                # 中文注释：第 1453 行，计算或保存局部变量 yaw，供后续逻辑使用。
                yaw = float(
                    # 中文注释：第 1454 行，执行当前代码行，作为所在逻辑块的一部分。
                    item.get(
                        # 中文注释：第 1455 行，执行当前代码行，作为所在逻辑块的一部分。
                        'bucket_theta',
                        # 中文注释：第 1456 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                        item.get('yaw', item.get('theta', 0.0)),
                    # 中文注释：第 1457 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    )
                # 中文注释：第 1458 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
                # 中文注释：第 1459 行，计算或保存局部变量 loaded[bucket_id]，供后续逻辑使用。
                loaded[bucket_id] = DumpBucketPose(
                    # 中文注释：第 1460 行，计算或保存局部变量 bucket_id，供后续逻辑使用。
                    bucket_id=bucket_id,
                    # 中文注释：第 1461 行，计算或保存局部变量 x，供后续逻辑使用。
                    x=float(item.get('bucket_x', item['x'])),
                    # 中文注释：第 1462 行，计算或保存局部变量 y，供后续逻辑使用。
                    y=float(item.get('bucket_y', item['y'])),
                    # 中文注释：第 1463 行，计算或保存局部变量 yaw，供后续逻辑使用。
                    yaw=yaw,
                    # 中文注释：第 1464 行，计算或保存局部变量 frame_id，供后续逻辑使用。
                    frame_id=str(
                        # 中文注释：第 1465 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                        item.get('frame_id')
                        # 中文注释：第 1466 行，执行当前代码行，作为所在逻辑块的一部分。
                        or self.get_parameter('map_frame').value
                    # 中文注释：第 1467 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    ),
                # 中文注释：第 1468 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 1469 行，处理 try 代码块中捕获到的异常。
            except (KeyError, TypeError, ValueError):
                # 中文注释：第 1470 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
        # 中文注释：第 1471 行，设置实例成员 self._dump_bucket_poses，保存节点运行状态或 ROS 通信对象。
        self._dump_bucket_poses = loaded
# 中文注释：第 1472 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1473 行，把卸货桶地图位姿保存到 JSON 文件。
    def _save_dump_bucket_poses(self):
        # 中文注释：第 1474 行，计算或保存局部变量 path，供后续逻辑使用。
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        # 中文注释：第 1475 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1476 行，计算或保存局部变量 path.parent.mkdir(parents，供后续逻辑使用。
            path.parent.mkdir(parents=True, exist_ok=True)
            # 中文注释：第 1477 行，计算或保存局部变量 payload，供后续逻辑使用。
            payload = {
                # 中文注释：第 1478 行，执行当前代码行，作为所在逻辑块的一部分。
                'dump_bucket_poses': {
                    # 中文注释：第 1479 行，执行当前代码行，作为所在逻辑块的一部分。
                    str(bucket_id): {
                        # 中文注释：第 1480 行，执行当前代码行，作为所在逻辑块的一部分。
                        'bucket_id': pose.bucket_id,
                        # 中文注释：第 1481 行，执行当前代码行，作为所在逻辑块的一部分。
                        'x': pose.x,
                        # 中文注释：第 1482 行，执行当前代码行，作为所在逻辑块的一部分。
                        'y': pose.y,
                        # 中文注释：第 1483 行，执行当前代码行，作为所在逻辑块的一部分。
                        'theta': pose.yaw,
                        # 中文注释：第 1484 行，执行当前代码行，作为所在逻辑块的一部分。
                        'yaw': pose.yaw,
                        # 中文注释：第 1485 行，执行当前代码行，作为所在逻辑块的一部分。
                        'frame_id': pose.frame_id,
                    # 中文注释：第 1486 行，结束前面开始的多行结构或函数调用。
                    }
                    # 中文注释：第 1487 行，开始 for 循环，逐项处理集合或固定次数的任务。
                    for bucket_id, pose in sorted(self._dump_bucket_poses.items())
                # 中文注释：第 1488 行，结束前面开始的多行结构或函数调用。
                }
            # 中文注释：第 1489 行，结束前面开始的多行结构或函数调用。
            }
            # 中文注释：第 1490 行，执行当前代码行，作为所在逻辑块的一部分。
            path.write_text(
                # 中文注释：第 1491 行，计算或保存局部变量 json.dumps(payload, ensure_ascii，供后续逻辑使用。
                json.dumps(payload, ensure_ascii=False, indent=2),
                # 中文注释：第 1492 行，计算或保存局部变量 encoding，供后续逻辑使用。
                encoding='utf-8',
            # 中文注释：第 1493 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
        # 中文注释：第 1494 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1495 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'save dump bucket poses failed: {exc}')
# 中文注释：第 1496 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1497 行，从 JSON 文件加载已保存的清扫区域。
    def _load_clean_regions(self):
        # 中文注释：第 1498 行，计算或保存局部变量 path，供后续逻辑使用。
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        # 中文注释：第 1499 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not path.exists():
            # 中文注释：第 1500 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1501 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1502 行，计算或保存局部变量 raw，供后续逻辑使用。
            raw = json.loads(path.read_text(encoding='utf-8'))
        # 中文注释：第 1503 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1504 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'load clean regions failed: {exc}')
            # 中文注释：第 1505 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1506 行，计算或保存局部变量 regions，供后续逻辑使用。
        regions = raw.get('clean_regions', raw)
        # 中文注释：第 1507 行，开始 for 循环，逐项处理集合或固定次数的任务。
        for key, item in regions.items():
            # 中文注释：第 1508 行，声明数据字段 try 及其类型。
            try:
                # 中文注释：第 1509 行，计算或保存局部变量 region_id，供后续逻辑使用。
                region_id = int(item.get('region_id', key))
                # 中文注释：第 1510 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if region_id <= 0:
                    # 中文注释：第 1511 行，跳过本次循环剩余逻辑，进入下一轮循环。
                    continue
                # 中文注释：第 1512 行，计算或保存局部变量 min_x，供后续逻辑使用。
                min_x = float(item['min_x'])
                # 中文注释：第 1513 行，计算或保存局部变量 min_y，供后续逻辑使用。
                min_y = float(item['min_y'])
                # 中文注释：第 1514 行，计算或保存局部变量 max_x，供后续逻辑使用。
                max_x = float(item['max_x'])
                # 中文注释：第 1515 行，计算或保存局部变量 max_y，供后续逻辑使用。
                max_y = float(item['max_y'])
                # 中文注释：第 1516 行，开始条件判断，满足条件时执行下面的分支逻辑。
                if min_x >= max_x or min_y >= max_y:
                    # 中文注释：第 1517 行，跳过本次循环剩余逻辑，进入下一轮循环。
                    continue
                # 中文注释：第 1518 行，设置实例成员 self._clean_regions[region_id]，保存节点运行状态或 ROS 通信对象。
                self._clean_regions[region_id] = CleanRegion(
                    # 中文注释：第 1519 行，计算或保存局部变量 region_id，供后续逻辑使用。
                    region_id=region_id,
                    # 中文注释：第 1520 行，计算或保存局部变量 min_x，供后续逻辑使用。
                    min_x=min_x,
                    # 中文注释：第 1521 行，计算或保存局部变量 min_y，供后续逻辑使用。
                    min_y=min_y,
                    # 中文注释：第 1522 行，计算或保存局部变量 max_x，供后续逻辑使用。
                    max_x=max_x,
                    # 中文注释：第 1523 行，计算或保存局部变量 max_y，供后续逻辑使用。
                    max_y=max_y,
                    # 中文注释：第 1524 行，计算或保存局部变量 frame_id，供后续逻辑使用。
                    frame_id=str(
                        # 中文注释：第 1525 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                        item.get('frame_id')
                        # 中文注释：第 1526 行，执行当前代码行，作为所在逻辑块的一部分。
                        or self.get_parameter('map_frame').value
                    # 中文注释：第 1527 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                    ),
                # 中文注释：第 1528 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
                )
            # 中文注释：第 1529 行，处理 try 代码块中捕获到的异常。
            except (KeyError, TypeError, ValueError):
                # 中文注释：第 1530 行，跳过本次循环剩余逻辑，进入下一轮循环。
                continue
# 中文注释：第 1531 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1532 行，把清扫区域保存到 JSON 文件。
    def _save_clean_regions(self):
        # 中文注释：第 1533 行，计算或保存局部变量 path，供后续逻辑使用。
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        # 中文注释：第 1534 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1535 行，计算或保存局部变量 path.parent.mkdir(parents，供后续逻辑使用。
            path.parent.mkdir(parents=True, exist_ok=True)
            # 中文注释：第 1536 行，计算或保存局部变量 payload，供后续逻辑使用。
            payload = {
                # 中文注释：第 1537 行，执行当前代码行，作为所在逻辑块的一部分。
                'clean_regions': {
                    # 中文注释：第 1538 行，执行当前代码行，作为所在逻辑块的一部分。
                    str(region_id): {
                        # 中文注释：第 1539 行，执行当前代码行，作为所在逻辑块的一部分。
                        'region_id': region.region_id,
                        # 中文注释：第 1540 行，执行当前代码行，作为所在逻辑块的一部分。
                        'min_x': region.min_x,
                        # 中文注释：第 1541 行，执行当前代码行，作为所在逻辑块的一部分。
                        'min_y': region.min_y,
                        # 中文注释：第 1542 行，执行当前代码行，作为所在逻辑块的一部分。
                        'max_x': region.max_x,
                        # 中文注释：第 1543 行，执行当前代码行，作为所在逻辑块的一部分。
                        'max_y': region.max_y,
                        # 中文注释：第 1544 行，执行当前代码行，作为所在逻辑块的一部分。
                        'frame_id': region.frame_id,
                    # 中文注释：第 1545 行，结束前面开始的多行结构或函数调用。
                    }
                    # 中文注释：第 1546 行，开始 for 循环，逐项处理集合或固定次数的任务。
                    for region_id, region in sorted(self._clean_regions.items())
                # 中文注释：第 1547 行，结束前面开始的多行结构或函数调用。
                }
            # 中文注释：第 1548 行，结束前面开始的多行结构或函数调用。
            }
            # 中文注释：第 1549 行，执行当前代码行，作为所在逻辑块的一部分。
            path.write_text(
                # 中文注释：第 1550 行，计算或保存局部变量 json.dumps(payload, ensure_ascii，供后续逻辑使用。
                json.dumps(payload, ensure_ascii=False, indent=2),
                # 中文注释：第 1551 行，计算或保存局部变量 encoding，供后续逻辑使用。
                encoding='utf-8',
            # 中文注释：第 1552 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
        # 中文注释：第 1553 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1554 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'save clean regions failed: {exc}')
# 中文注释：第 1555 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1556 行，向机械臂目标 action 发送抓取位姿。
    def _send_arm_target(self, goal_handle, target):
        # 中文注释：第 1557 行，构造 action 目标或读取请求目标对象。
        goal = ArmTarget.Goal()
        # 中文注释：第 1558 行，计算或保存局部变量 goal.x，供后续逻辑使用。
        goal.x = target.target_x_cm
        # 中文注释：第 1559 行，计算或保存局部变量 goal.y，供后续逻辑使用。
        goal.y = target.target_y_cm
        # 中文注释：第 1560 行，计算或保存局部变量 goal.z，供后续逻辑使用。
        goal.z = target.target_z_cm
        # 中文注释：第 1561 行，计算或保存局部变量 goal.duration，供后续逻辑使用。
        goal.duration = 2.0
        # 中文注释：第 1562 行，计算或保存局部变量 goal.end_angle，供后续逻辑使用。
        goal.end_angle = target.end_angle
        # 中文注释：第 1563 行，计算或保存局部变量 goal.gripper_rotation，供后续逻辑使用。
        goal.gripper_rotation = target.gripper_rotation
        # 中文注释：第 1564 行，保存动作、服务或函数调用的结果。
        result = self._send_action_goal(
            # 中文注释：第 1565 行，执行当前代码行，作为所在逻辑块的一部分。
            self._arm_target_client,
            # 中文注释：第 1566 行，执行当前代码行，作为所在逻辑块的一部分。
            goal,
            # 中文注释：第 1567 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            # 中文注释：第 1568 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1569 行，计算或保存局部变量 label，供后续逻辑使用。
            label='arm target',
        # 中文注释：第 1570 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1571 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if result is None or not result.success:
            # 中文注释：第 1572 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError('arm target failed')
# 中文注释：第 1573 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1574 行，向机械臂预设 action 发送预设姿态。
    def _send_arm_preset(self, goal_handle, action_type):
        # 中文注释：第 1575 行，构造 action 目标或读取请求目标对象。
        goal = ArmPreset.Goal()
        # 中文注释：第 1576 行，计算或保存局部变量 goal.action_type，供后续逻辑使用。
        goal.action_type = int(action_type)
        # 中文注释：第 1577 行，保存动作、服务或函数调用的结果。
        result = self._send_action_goal(
            # 中文注释：第 1578 行，执行当前代码行，作为所在逻辑块的一部分。
            self._arm_preset_client,
            # 中文注释：第 1579 行，执行当前代码行，作为所在逻辑块的一部分。
            goal,
            # 中文注释：第 1580 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('action_timeout_sec').value),
            # 中文注释：第 1581 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1582 行，计算或保存局部变量 label，供后续逻辑使用。
            label='arm preset',
        # 中文注释：第 1583 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1584 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if result is None or not result.success:
            # 中文注释：第 1585 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError('arm preset failed')
        # 中文注释：第 1586 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if int(action_type) not in (
            # 中文注释：第 1587 行，执行当前代码行，作为所在逻辑块的一部分。
            ArmPreset.Goal.ARM_CENTER,
            # 中文注释：第 1588 行，执行当前代码行，作为所在逻辑块的一部分。
            ArmPreset.Goal.ARM_MAPPING,
        # 中文注释：第 1589 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 1590 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1591 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        self._set_camera_tf_preset(goal_handle, action_type)
# 中文注释：第 1592 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1593 行，同步更新相机静态 TF 的预设姿态。
    def _set_camera_tf_preset(self, goal_handle, action_type):
        # 中文注释：第 1594 行，构造服务请求对象，准备填入调用参数。
        request = SetCameraTfPreset.Request()
        # 中文注释：第 1595 行，计算或保存局部变量 request.action_type，供后续逻辑使用。
        request.action_type = int(action_type)
        # 中文注释：第 1596 行，调用服务或读取服务响应结果。
        response = self._call_service(
            # 中文注释：第 1597 行，执行当前代码行，作为所在逻辑块的一部分。
            self._camera_tf_preset_client,
            # 中文注释：第 1598 行，执行当前代码行，作为所在逻辑块的一部分。
            request,
            # 中文注释：第 1599 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 1600 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1601 行，计算或保存局部变量 label，供后续逻辑使用。
            label='set camera tf preset',
        # 中文注释：第 1602 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1603 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None:
            # 中文注释：第 1604 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(
                # 中文注释：第 1605 行，计算或保存局部变量 f'camera tf preset service unavailable: action_type，供后续逻辑使用。
                f'camera tf preset service unavailable: action_type={int(action_type)}'
            # 中文注释：第 1606 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
            # 中文注释：第 1607 行，结束当前函数或方法，不返回额外数据。
            return
        # 中文注释：第 1608 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not response.success:
            # 中文注释：第 1609 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(
                # 中文注释：第 1610 行，计算或保存局部变量 f'camera tf preset rejected: action_type，供后续逻辑使用。
                f'camera tf preset rejected: action_type={int(action_type)}, '
                # 中文注释：第 1611 行，计算或保存局部变量 f'message，供后续逻辑使用。
                f'message={response.message}'
            # 中文注释：第 1612 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            )
# 中文注释：第 1613 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1614 行，调用夹爪服务设置开合比例和动作时长。
    def _set_gripper(self, goal_handle, open_ratio, duration_ms, label):
        # 中文注释：第 1615 行，构造服务请求对象，准备填入调用参数。
        request = SetGripper.Request()
        # 中文注释：第 1616 行，计算或保存局部变量 request.open_ratio，供后续逻辑使用。
        request.open_ratio = float(max(0.0, min(1.0, open_ratio)))
        # 中文注释：第 1617 行，计算或保存局部变量 request.duration_ms，供后续逻辑使用。
        request.duration_ms = float(duration_ms)
        # 中文注释：第 1618 行，调用服务或读取服务响应结果。
        response = self._call_service(
            # 中文注释：第 1619 行，执行当前代码行，作为所在逻辑块的一部分。
            self._gripper_client,
            # 中文注释：第 1620 行，执行当前代码行，作为所在逻辑块的一部分。
            request,
            # 中文注释：第 1621 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value),
            # 中文注释：第 1622 行，计算或保存局部变量 goal_handle，供后续逻辑使用。
            goal_handle=goal_handle,
            # 中文注释：第 1623 行，计算或保存局部变量 label，供后续逻辑使用。
            label=label,
        # 中文注释：第 1624 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1625 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None or not response.success:
            # 中文注释：第 1626 行，抛出异常，让上层流程感知当前步骤失败。
            raise RuntimeError(f'{label} failed')
# 中文注释：第 1627 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1628 行，任务结束清理时尝试闭合夹爪。
    def _close_gripper_for_cleanup(self, task_name):
        # 中文注释：第 1629 行，构造服务请求对象，准备填入调用参数。
        request = SetGripper.Request()
        # 中文注释：第 1630 行，计算或保存局部变量 request.open_ratio，供后续逻辑使用。
        request.open_ratio = 0.0
        # 中文注释：第 1631 行，计算或保存局部变量 request.duration_ms，供后续逻辑使用。
        request.duration_ms = float(self.get_parameter('gripper_duration_ms').value)
        # 中文注释：第 1632 行，读取或计算当前步骤使用的超时时间。
        timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        # 中文注释：第 1633 行，计算或保存局部变量 label，供后续逻辑使用。
        label = f'{task_name} cleanup close gripper'
# 中文注释：第 1634 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1635 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._gripper_client.wait_for_service(timeout_sec=timeout_sec):
            # 中文注释：第 1636 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} service not available')
            # 中文注释：第 1637 行，结束当前函数或方法，不返回额外数据。
            return
# 中文注释：第 1638 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1639 行，计算或保存局部变量 future，供后续逻辑使用。
        future = self._gripper_client.call_async(request)
        # 中文注释：第 1640 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + timeout_sec
        # 中文注释：第 1641 行，开始 while 循环，在条件满足期间持续执行。
        while rclpy.ok() and not future.done():
            # 中文注释：第 1642 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if time.monotonic() >= deadline:
                # 中文注释：第 1643 行，取消正在等待的服务或 action，避免任务停止后继续执行。
                future.cancel()
                # 中文注释：第 1644 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
                self.get_logger().warning(f'{label} service timed out')
                # 中文注释：第 1645 行，结束当前函数或方法，不返回额外数据。
                return
            # 中文注释：第 1646 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.02)
# 中文注释：第 1647 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1648 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1649 行，调用服务或读取服务响应结果。
            response = future.result()
        # 中文注释：第 1650 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1651 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} service failed: {exc}')
            # 中文注释：第 1652 行，结束当前函数或方法，不返回额外数据。
            return
# 中文注释：第 1653 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1654 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if response is None or not response.success:
            # 中文注释：第 1655 行，计算或保存局部变量 message，供后续逻辑使用。
            message = '' if response is None else f': {response.message}'
            # 中文注释：第 1656 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} failed{message}')
# 中文注释：第 1657 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1658 行，统一调用 ROS 服务并处理超时、取消和异常。
    def _call_service(self, client, request, timeout_sec, goal_handle, label):
        # 中文注释：第 1659 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._is_canceling(goal_handle):
            # 中文注释：第 1660 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1661 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not client.wait_for_service(timeout_sec=timeout_sec):
            # 中文注释：第 1662 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} service not available')
            # 中文注释：第 1663 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1664 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1665 行，计算或保存局部变量 future，供后续逻辑使用。
        future = client.call_async(request)
        # 中文注释：第 1666 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._wait_future(future, timeout_sec, goal_handle):
            # 中文注释：第 1667 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} service timed out')
            # 中文注释：第 1668 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1669 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1670 行，声明数据字段 try 及其类型。
        try:
            # 中文注释：第 1671 行，返回当前函数或方法的执行结果。
            return future.result()
        # 中文注释：第 1672 行，处理 try 代码块中捕获到的异常。
        except Exception as exc:
            # 中文注释：第 1673 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} service failed: {exc}')
            # 中文注释：第 1674 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1675 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1676 行，发送 action 目标并只返回业务结果。
    def _send_action_goal(self, client, goal, timeout_sec, goal_handle, label):
        # 中文注释：第 1677 行，计算或保存局部变量 wrapped，供后续逻辑使用。
        wrapped = self._send_action_goal_wrapped(
            # 中文注释：第 1678 行，执行当前代码行，作为所在逻辑块的一部分。
            client,
            # 中文注释：第 1679 行，执行当前代码行，作为所在逻辑块的一部分。
            goal,
            # 中文注释：第 1680 行，执行当前代码行，作为所在逻辑块的一部分。
            timeout_sec,
            # 中文注释：第 1681 行，执行当前代码行，作为所在逻辑块的一部分。
            goal_handle,
            # 中文注释：第 1682 行，执行当前代码行，作为所在逻辑块的一部分。
            label,
        # 中文注释：第 1683 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
        # 中文注释：第 1684 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if wrapped is None:
            # 中文注释：第 1685 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1686 行，返回当前函数或方法的执行结果。
        return wrapped.result
# 中文注释：第 1687 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1688 行，发送 action 目标并返回包含状态的完整结果。
    def _send_action_goal_wrapped(self, client, goal, timeout_sec, goal_handle, label):
        # 中文注释：第 1689 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if self._is_canceling(goal_handle):
            # 中文注释：第 1690 行，返回当前函数或方法的执行结果。
            return None
        # 中文注释：第 1691 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not client.wait_for_server(
            # 中文注释：第 1692 行，计算或保存局部变量 timeout_sec，供后续逻辑使用。
            timeout_sec=float(self.get_parameter('service_timeout_sec').value)
        # 中文注释：第 1693 行，执行当前代码行，作为所在逻辑块的一部分。
        ):
            # 中文注释：第 1694 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} action server not available')
            # 中文注释：第 1695 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1696 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1697 行，计算或保存局部变量 send_future，供后续逻辑使用。
        send_future = client.send_goal_async(goal)
        # 中文注释：第 1698 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._wait_future(send_future, timeout_sec, goal_handle):
            # 中文注释：第 1699 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} goal response timed out')
            # 中文注释：第 1700 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1701 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1702 行，计算或保存局部变量 action_goal_handle，供后续逻辑使用。
        action_goal_handle = send_future.result()
        # 中文注释：第 1703 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if action_goal_handle is None or not action_goal_handle.accepted:
            # 中文注释：第 1704 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} goal rejected')
            # 中文注释：第 1705 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1706 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1707 行，计算或保存局部变量 result_future，供后续逻辑使用。
        result_future = action_goal_handle.get_result_async()
        # 中文注释：第 1708 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if not self._wait_future(result_future, timeout_sec, goal_handle):
            # 中文注释：第 1709 行，输出 warning 级别日志，提示服务失败、检测失败或流程异常。
            self.get_logger().warning(f'{label} result timed out')
            # 中文注释：第 1710 行，声明数据字段 try 及其类型。
            try:
                # 中文注释：第 1711 行，取消正在等待的服务或 action，避免任务停止后继续执行。
                action_goal_handle.cancel_goal_async()
            # 中文注释：第 1712 行，处理 try 代码块中捕获到的异常。
            except Exception:
                # 中文注释：第 1713 行，占位语句，表示该分支无需额外处理。
                pass
            # 中文注释：第 1714 行，返回当前函数或方法的执行结果。
            return None
# 中文注释：第 1715 行，空行，用来分隔相邻逻辑块，提升代码可读性。

        # 中文注释：第 1716 行，计算或保存局部变量 wrapped，供后续逻辑使用。
        wrapped = result_future.result()
        # 中文注释：第 1717 行，返回当前函数或方法的执行结果。
        return wrapped
# 中文注释：第 1718 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1719 行，轮询等待异步 future 完成，同时检查超时和取消。
    def _wait_future(self, future, timeout_sec, goal_handle):
        # 中文注释：第 1720 行，计算超时截止时间，用于后续循环判断。
        deadline = time.monotonic() + timeout_sec
        # 中文注释：第 1721 行，开始 while 循环，在条件满足期间持续执行。
        while rclpy.ok() and not future.done():
            # 中文注释：第 1722 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if self._is_canceling(goal_handle):
                # 中文注释：第 1723 行，取消正在等待的服务或 action，避免任务停止后继续执行。
                future.cancel()
                # 中文注释：第 1724 行，返回当前函数或方法的执行结果。
                return False
            # 中文注释：第 1725 行，开始条件判断，满足条件时执行下面的分支逻辑。
            if time.monotonic() >= deadline:
                # 中文注释：第 1726 行，取消正在等待的服务或 action，避免任务停止后继续执行。
                future.cancel()
                # 中文注释：第 1727 行，返回当前函数或方法的执行结果。
                return False
            # 中文注释：第 1728 行，短暂等待，让硬件动作、服务响应或传感器数据稳定。
            time.sleep(0.02)
        # 中文注释：第 1729 行，返回当前函数或方法的执行结果。
        return future.done()
# 中文注释：第 1730 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1731 行，发布零速度命令让底盘停止。
    def _publish_stop(self):
        # 中文注释：第 1732 行，发布底盘速度命令。
        self._cmd_vel_pub.publish(Twist())
# 中文注释：第 1733 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1734 行，发布 action 反馈并写入日志。
    def _feedback(self, goal_handle, state, progress):
        # 中文注释：第 1735 行，计算或保存局部变量 feedback，供后续逻辑使用。
        feedback = RobotTask.Feedback()
        # 中文注释：第 1736 行，计算或保存局部变量 feedback.state，供后续逻辑使用。
        feedback.state = str(state)
        # 中文注释：第 1737 行，计算或保存局部变量 feedback.progress，供后续逻辑使用。
        feedback.progress = float(max(0.0, min(1.0, progress)))
        # 中文注释：第 1738 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        goal_handle.publish_feedback(feedback)
        # 中文注释：第 1739 行，输出 info 级别日志，记录当前流程状态或关键参数。
        self.get_logger().info(feedback.state)
# 中文注释：第 1740 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1741 行，构造 RobotTask action 的结果消息。
    def _result(self, success, message):
        # 中文注释：第 1742 行，保存动作、服务或函数调用的结果。
        result = RobotTask.Result()
        # 中文注释：第 1743 行，计算或保存局部变量 result.success，供后续逻辑使用。
        result.success = bool(success)
        # 中文注释：第 1744 行，计算或保存局部变量 result.message，供后续逻辑使用。
        result.message = str(message)
        # 中文注释：第 1745 行，返回当前函数或方法的执行结果。
        return result
# 中文注释：第 1746 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1747 行，读取并规范化导航超时时间。
    def _navigation_timeout_sec(self):
        # 中文注释：第 1748 行，返回当前函数或方法的执行结果。
        return max(
            # 中文注释：第 1749 行，执行当前代码行，作为所在逻辑块的一部分。
            1.0,
            # 中文注释：第 1750 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            float(self.get_parameter('navigation_timeout_sec').value),
        # 中文注释：第 1751 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1752 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1753 行，判断当前任务是否正在停止或取消。
    def _is_canceling(self, goal_handle):
        # 中文注释：第 1754 行，返回当前函数或方法的执行结果。
        return (
            # 中文注释：第 1755 行，执行当前代码行，作为所在逻辑块的一部分。
            self._stop_requested
            # 中文注释：第 1756 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
            or (goal_handle is not None and goal_handle.is_cancel_requested)
        # 中文注释：第 1757 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
        )
# 中文注释：第 1758 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1759 行，返回行为树支持的任务类型集合。
    def _known_task_types(self):
        # 中文注释：第 1760 行，返回当前函数或方法的执行结果。
        return {
            # 中文注释：第 1761 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_CLEAN_TRASH,
            # 中文注释：第 1762 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_DUMP_TRASH,
            # 中文注释：第 1763 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_CLEAN_REGION,
            # 中文注释：第 1764 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_NAVIGATE_TO,
            # 中文注释：第 1765 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_NAVIGATE_PATH,
            # 中文注释：第 1766 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_SET_DUMP_POSE,
            # 中文注释：第 1767 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_SET_CLEAN_REGION,
            # 中文注释：第 1768 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_STOP,
            # 中文注释：第 1769 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_RESUME,
        # 中文注释：第 1770 行，结束前面开始的多行结构或函数调用。
        }
# 中文注释：第 1771 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1772 行，把任务类型常量转换成日志使用的任务名。
    def _task_name(self, task_type):
        # 中文注释：第 1773 行，计算或保存局部变量 names，供后续逻辑使用。
        names = {
            # 中文注释：第 1774 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_CLEAN_TRASH: 'clean_trash',
            # 中文注释：第 1775 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_DUMP_TRASH: 'dump_trash',
            # 中文注释：第 1776 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_CLEAN_REGION: 'clean_region',
            # 中文注释：第 1777 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_NAVIGATE_TO: 'navigate_to',
            # 中文注释：第 1778 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_NAVIGATE_PATH: 'navigate_path',
            # 中文注释：第 1779 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_SET_DUMP_POSE: 'set_dump_pose',
            # 中文注释：第 1780 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_SET_CLEAN_REGION: 'set_clean_region',
            # 中文注释：第 1781 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_STOP: 'stop',
            # 中文注释：第 1782 行，执行当前代码行，作为所在逻辑块的一部分。
            RobotTask.Goal.TASK_RESUME: 'resume',
        # 中文注释：第 1783 行，结束前面开始的多行结构或函数调用。
        }
        # 中文注释：第 1784 行，返回当前函数或方法的执行结果。
        return names.get(task_type, f'task_{task_type}')
# 中文注释：第 1785 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1786 行，声明下面的方法是静态方法，不依赖 self 实例状态。
    @staticmethod
    # 中文注释：第 1787 行，把 yaw 角转换为平面导航使用的四元数 z、w 分量。
    def _yaw_to_quaternion_z_w(yaw):
        # 中文注释：第 1788 行，计算或保存局部变量 half_yaw，供后续逻辑使用。
        half_yaw = float(yaw) * 0.5
        # 中文注释：第 1789 行，返回当前函数或方法的执行结果。
        return math.sin(half_yaw), math.cos(half_yaw)
# 中文注释：第 1790 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1791 行，声明下面的方法是静态方法，不依赖 self 实例状态。
    @staticmethod
    # 中文注释：第 1792 行，从四元数中提取 yaw 航向角。
    def _yaw_from_quaternion(q):
        # 中文注释：第 1793 行，计算或保存局部变量 siny_cosp，供后续逻辑使用。
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # 中文注释：第 1794 行，计算或保存局部变量 cosy_cosp，供后续逻辑使用。
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # 中文注释：第 1795 行，返回当前函数或方法的执行结果。
        return math.atan2(siny_cosp, cosy_cosp)
# 中文注释：第 1796 行，空行，用来分隔相邻逻辑块，提升代码可读性。

    # 中文注释：第 1797 行，声明下面的方法是静态方法，不依赖 self 实例状态。
    @staticmethod
    # 中文注释：第 1798 行，把角度归一化到 -pi 到 pi 范围。
    def _normalize_angle(angle):
        # 中文注释：第 1799 行，返回当前函数或方法的执行结果。
        return math.atan2(math.sin(angle), math.cos(angle))
# 中文注释：第 1800 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 1801 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 1802 行，创建节点并启动多线程执行器。
def main(args=None):
    # 中文注释：第 1803 行，文档字符串，说明当前模块、类或方法的用途。
    """启动行为树任务调度节点。"""
    # 中文注释：第 1804 行，初始化 ROS 2 Python 运行环境。
    rclpy.init(args=args)
    # 中文注释：第 1805 行，计算或保存局部变量 node，供后续逻辑使用。
    node = RobotTaskServer()
    # 中文注释：第 1806 行，计算或保存局部变量 executor，供后续逻辑使用。
    executor = MultiThreadedExecutor(num_threads=4)
    # 中文注释：第 1807 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
    executor.add_node(node)
    # 中文注释：第 1808 行，声明数据字段 try 及其类型。
    try:
        # 中文注释：第 1809 行，启动执行器循环处理 ROS 回调。
        executor.spin()
    # 中文注释：第 1810 行，处理 try 代码块中捕获到的异常。
    except KeyboardInterrupt:
        # 中文注释：第 1811 行，占位语句，表示该分支无需额外处理。
        pass
    # 中文注释：第 1812 行，声明数据字段 finally 及其类型。
    finally:
        # 中文注释：第 1813 行，销毁 ROS 节点并释放相关资源。
        node.destroy_node()
        # 中文注释：第 1814 行，开始条件判断，满足条件时执行下面的分支逻辑。
        if rclpy.ok():
            # 中文注释：第 1815 行，关闭 ROS 2 Python 运行环境。
            rclpy.shutdown()
# 中文注释：第 1816 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 1817 行，空行，用来分隔相邻逻辑块，提升代码可读性。

# 中文注释：第 1818 行，开始条件判断，满足条件时执行下面的分支逻辑。
if __name__ == '__main__':
    # 中文注释：第 1819 行，继续或完成一次函数/方法调用，传入当前步骤需要的参数。
    main()
