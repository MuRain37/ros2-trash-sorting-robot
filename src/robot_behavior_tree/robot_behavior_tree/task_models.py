"""Shared data models for robot behavior tasks."""

from dataclasses import dataclass


@dataclass
class GraspTarget:
    """视觉识别返回的抓取目标。"""

    class_name: str
    confidence: float
    target_x_cm: float
    target_y_cm: float
    target_z_cm: float
    end_angle: float
    gripper_rotation: float
    gripper_close_ratio: float
    bucket_id: int


@dataclass
class DumpBucketPose:
    """地图上标定的卸载桶位置。"""

    bucket_id: int
    x: float
    y: float
    yaw: float
    frame_id: str


@dataclass
class CleanRegion:
    """行为树保存的清扫区域。"""

    region_id: int
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    frame_id: str


@dataclass
class DumpBucketDetection:
    """卸载桶视觉检测结果：中心偏移、深度和是否到达倾倒距离。"""

    detected: bool
    within_depth: bool
    mask_pixels: int
    mean_depth_m: float
    offset_x_px: float
    offset_x_ratio: float
    message: str


@dataclass
class MapPose:
    """机器人当前 map 位姿。"""

    x: float
    y: float
    yaw: float


@dataclass
class ScanCameraObstacle:
    """scan_camera 最近看到的前方近距离障碍。"""

    range_m: float
    bearing_rad: float
    seen_at: float
