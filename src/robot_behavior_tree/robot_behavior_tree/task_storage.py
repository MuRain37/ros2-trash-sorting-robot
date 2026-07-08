"""Persistent task state helpers for RobotTaskServer."""

import json
import os
from pathlib import Path

from .task_models import CleanRegion, DumpBucketPose


STATE_ROOT = os.getenv(
    'ROS_STATE_ROOT',
    os.path.join(os.path.expanduser('~'), '.ros2_robot'),
)

PARAMETER_DEFAULTS = (
    # 卸载桶标定位姿 JSON 文件保存路径。
    (
        'dump_pose_store_path',
        os.path.join(
            STATE_ROOT,
            'robot_behavior_tree',
            'config',
            'dump_bucket_poses.json',
        ),
    ),
    # 区域清扫边界 JSON 文件保存路径。
    (
        'clean_region_store_path',
        os.path.join(
            STATE_ROOT,
            'robot_behavior_tree',
            'config',
            'clean_regions.json',
        ),
    ),
)


class TaskStorageMixin:

    def _declare_storage_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

    def _load_dump_bucket_poses(self):
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        if not path.exists():
            return
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except Exception as exc:
            self.get_logger().warning(f'load dump bucket poses failed: {exc}')
            return
        poses = raw.get('dump_bucket_poses', raw)
        loaded = {}
        for key, item in poses.items():
            try:
                bucket_id = int(item.get('bucket_id', key))
                if bucket_id not in (1, 2):
                    continue
                yaw = float(
                    item.get(
                        'bucket_theta',
                        item.get('yaw', item.get('theta', 0.0)),
                    )
                )
                loaded[bucket_id] = DumpBucketPose(
                    bucket_id=bucket_id,
                    x=float(item.get('bucket_x', item['x'])),
                    y=float(item.get('bucket_y', item['y'])),
                    yaw=yaw,
                    frame_id=str(
                        item.get('frame_id')
                        or self.get_parameter('map_frame').value
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue
        self._dump_bucket_poses = loaded

    def _save_dump_bucket_poses(self):
        path = Path(str(self.get_parameter('dump_pose_store_path').value))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'dump_bucket_poses': {
                    str(bucket_id): {
                        'bucket_id': pose.bucket_id,
                        'x': pose.x,
                        'y': pose.y,
                        'theta': pose.yaw,
                        'yaw': pose.yaw,
                        'frame_id': pose.frame_id,
                    }
                    for bucket_id, pose in sorted(self._dump_bucket_poses.items())
                }
            }
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except Exception as exc:
            self.get_logger().warning(f'save dump bucket poses failed: {exc}')

    def _load_clean_regions(self):
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        if not path.exists():
            return
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except Exception as exc:
            self.get_logger().warning(f'load clean regions failed: {exc}')
            return
        regions = raw.get('clean_regions', raw)
        for key, item in regions.items():
            try:
                region_id = int(item.get('region_id', key))
                if region_id <= 0:
                    continue
                min_x = float(item['min_x'])
                min_y = float(item['min_y'])
                max_x = float(item['max_x'])
                max_y = float(item['max_y'])
                if min_x >= max_x or min_y >= max_y:
                    continue
                self._clean_regions[region_id] = CleanRegion(
                    region_id=region_id,
                    min_x=min_x,
                    min_y=min_y,
                    max_x=max_x,
                    max_y=max_y,
                    frame_id=str(
                        item.get('frame_id')
                        or self.get_parameter('map_frame').value
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue

    def _save_clean_regions(self):
        path = Path(str(self.get_parameter('clean_region_store_path').value))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'clean_regions': {
                    str(region_id): {
                        'region_id': region.region_id,
                        'min_x': region.min_x,
                        'min_y': region.min_y,
                        'max_x': region.max_x,
                        'max_y': region.max_y,
                        'frame_id': region.frame_id,
                    }
                    for region_id, region in sorted(self._clean_regions.items())
                }
            }
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except Exception as exc:
            self.get_logger().warning(f'save clean regions failed: {exc}')
    