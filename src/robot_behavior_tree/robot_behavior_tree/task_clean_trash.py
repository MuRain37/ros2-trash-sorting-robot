"""Clean-trash task behavior for RobotTaskServer."""

import math
import time

from geometry_msgs.msg import Twist
from robot_interfaces.action import ArmPreset
from robot_interfaces.srv import DetectGraspTarget

from .task_models import GraspTarget


PARAMETER_DEFAULTS = (
    # 抓取目标视觉检测服务名，清扫垃圾时用于识别可抓取物体。
    ('detect_grasp_service', 'vision_grasp/detect'),
    # 单次清扫最多抓取数量；<=0 表示持续清扫直到停止或取消。
    ('clean_max_objects', 0),
    # 抓取目标检测失败后的重试次数。
    ('detect_retry_count', 3),
    # 抓取前底盘靠近或旋转对准目标的最大尝试次数。
    ('approach_max_attempts', 3),
    # 目标前向距离阈值，超过该距离会先驱动底盘靠近。
    ('approach_x_threshold_cm', 32.0),
    # 目标横向偏移阈值，超过该距离会先旋转底盘对准。
    ('approach_y_threshold_cm', 12.0),
    # 最大尝试后允许直接抓取的前向距离容差。
    ('approach_x_tolerance_cm', 2.0),
    # 靠近目标时希望保留的前向距离。
    ('approach_target_x_cm', 20.0),
    # 单次靠近目标允许前进的最大距离。
    ('approach_max_step_m', 0.04),
    # 靠近目标时底盘前进线速度。
    ('approach_linear_speed', 0.08),
    # 横向对准目标时底盘旋转角速度。
    ('approach_y_angular_speed', 0.27),
    # 横向对准目标时单次允许旋转的最大角度。
    ('approach_y_max_rotate_rad', 0.35),
)


class CleanTrashTaskMixin:

    def _declare_clean_trash_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

    def _run_clean_trash_tree(self, goal_handle):
        """清扫垃圾：识别 -> 必要时前进 -> MoveIt 抓取 -> 投桶 -> 归中。"""
        self._feedback(goal_handle, 'clean_trash: arm center', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)

        picked = 0
        scan_index = 0
        max_objects = int(self.get_parameter('clean_max_objects').value)
        while max_objects <= 0 or picked < max_objects:
            if self._is_canceling(goal_handle):
                return False, 'clean trash canceled'

            scan_index += 1
            self._feedback(
                goal_handle,
                f'clean_trash: detecting target {scan_index}',
                0.10,
            )
            target = self._detect_reachable_target(goal_handle)
            if target is None:
                if max_objects > 0:
                    if picked == 0:
                        return False, 'no reachable trash target detected'
                    return True, f'clean trash completed: picked={picked}'
                self._feedback(
                    goal_handle,
                    'clean_trash: no target, scanning again',
                    0.20,
                )
                time.sleep(0.5)
                continue

            self._feedback(
                goal_handle,
                f'clean_trash: grasping {target.class_name}',
                0.35,
            )
            bucket_id, bucket_level = self._grasp_target(goal_handle, target)
            picked += 1
            success, message = self._maybe_auto_dump_full_bucket(
                goal_handle,
                bucket_id,
                bucket_level,
            )
            if not success:
                return False, message

        return True, f'clean trash completed: picked={picked}'

    def _detect_reachable_target(self, goal_handle):
        last_target = None
        max_attempts = int(self.get_parameter('approach_max_attempts').value)
        for approach_index in range(
            max_attempts + 1
        ):
            target = self._detect_grasp_target(goal_handle)
            if target is None:
                return None
            last_target = target

            x_threshold_cm = float(
                self.get_parameter('approach_x_threshold_cm').value
            )
            x_tolerance_cm = max(
                0.0,
                float(self.get_parameter('approach_x_tolerance_cm').value),
            )
            y_threshold_cm = abs(
                float(self.get_parameter('approach_y_threshold_cm').value)
            )
            if (
                y_threshold_cm > 0.0
                and abs(target.target_y_cm) > y_threshold_cm
            ):
                if approach_index >= max_attempts:
                    break

                self._feedback(
                    goal_handle,
                    'clean_trash: target lateral offset, rotating base',
                    0.20,
                )
                self._rotate_toward_target_y(goal_handle, target)
                continue

            if target.target_x_cm <= x_threshold_cm:
                return target
            if (
                approach_index >= max_attempts
                and target.target_x_cm <= x_threshold_cm + x_tolerance_cm
            ):
                self.get_logger().warning(
                    'target slightly beyond approach threshold; grasp anyway: '
                    f'x={target.target_x_cm:.1f} cm, '
                    f'threshold={x_threshold_cm:.1f} cm, '
                    f'tolerance={x_tolerance_cm:.1f} cm, '
                    f'y={target.target_y_cm:.1f} cm'
                )
                return target

            if approach_index >= max_attempts:
                break

            self._feedback(
                goal_handle,
                'clean_trash: target is far, moving base forward',
                0.20,
            )
            self._drive_forward_toward_target(goal_handle, target.target_x_cm)

        self.get_logger().warning(
            'target still beyond approach threshold: '
            f'x={last_target.target_x_cm:.1f} cm, '
            f'y={last_target.target_y_cm:.1f} cm'
        )
        return None

    def _detect_grasp_target(self, goal_handle):
        retries = int(self.get_parameter('detect_retry_count').value)
        for attempt in range(max(1, retries)):
            if self._is_canceling(goal_handle):
                return None

            request = DetectGraspTarget.Request()
            request.force_refresh = True
            response = self._call_service(
                self._detect_grasp_client,
                request,
                timeout_sec=float(
                    self.get_parameter('service_timeout_sec').value
                ),
                goal_handle=goal_handle,
                label='detect grasp target',
            )
            if response is not None and response.success:
                return GraspTarget(
                    class_name=str(response.class_name),
                    confidence=float(response.confidence),
                    target_x_cm=float(response.target_x_cm),
                    target_y_cm=float(response.target_y_cm),
                    target_z_cm=float(response.target_z_cm),
                    end_angle=float(response.end_angle),
                    gripper_rotation=float(response.gripper_rotation),
                    gripper_close_ratio=float(response.gripper_close_ratio),
                    bucket_id=int(response.bucket_id),
                )
            if response is None:
                self.get_logger().warning(
                    'detect grasp target attempt '
                    f'{attempt + 1}/{max(1, retries)} returned no response'
                )
            else:
                self.get_logger().warning(
                    'detect grasp target attempt '
                    f'{attempt + 1}/{max(1, retries)} failed: '
                    f'{response.message}'
                )
            time.sleep(0.15)
            self._feedback(
                goal_handle,
                f'clean_trash: detection retry {attempt + 1}',
                0.15,
            )
        return None

    def _drive_forward_toward_target(self, goal_handle, target_x_cm):
        target_x = float(self.get_parameter('approach_target_x_cm').value)
        max_step = float(self.get_parameter('approach_max_step_m').value)
        speed = abs(float(self.get_parameter('approach_linear_speed').value))
        if speed <= 0.0:
            return

        distance_m = max(0.0, (target_x_cm - target_x) / 100.0)
        distance_m = min(distance_m, max_step)
        duration = distance_m / speed

        deadline = time.monotonic() + duration
        msg = Twist()
        msg.linear.x = speed
        while time.monotonic() < deadline and not self._is_canceling(goal_handle):
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()
        time.sleep(0.2)

    def _rotate_toward_target_y(self, goal_handle, target):
        speed = abs(float(self.get_parameter('approach_y_angular_speed').value))
        max_rotate = abs(
            float(self.get_parameter('approach_y_max_rotate_rad').value)
        )
        if speed <= 0.0 or max_rotate <= 0.0:
            return

        forward_cm = max(1.0, float(target.target_x_cm))
        rotate_rad = math.atan2(float(target.target_y_cm), forward_cm)
        rotate_rad = max(-max_rotate, min(max_rotate, rotate_rad))
        if abs(rotate_rad) <= 0.001:
            return

        msg = Twist()
        msg.angular.z = speed if rotate_rad > 0.0 else -speed
        self.get_logger().info(
            'grasp lateral align rotate: '
            f'x={target.target_x_cm:.1f} cm, '
            f'y={target.target_y_cm:.1f} cm, '
            f'rotate={rotate_rad:.3f} rad, '
            f'angular_z={msg.angular.z:.3f}'
        )
        self._publish_twist_for(goal_handle, msg, abs(rotate_rad) / speed)
        time.sleep(0.2)

    def _grasp_target(self, goal_handle, target):
        duration_ms = float(self.get_parameter('gripper_duration_ms').value)
        wait_sec = float(self.get_parameter('gripper_wait_sec').value)

        self._set_gripper(goal_handle, 1.0, duration_ms, 'open gripper')
        self._send_arm_target(goal_handle, target)
        self._set_gripper(
            goal_handle,
            target.gripper_close_ratio,
            duration_ms,
            'close gripper',
        )
        time.sleep(wait_sec)

        bucket_id = target.bucket_id if target.bucket_id in (1, 2) else 1
        self._feedback(goal_handle, f'clean_trash: move to bucket {bucket_id}', 0.70)
        self._send_arm_preset(goal_handle, bucket_id)
        self._set_gripper(goal_handle, 1.0, duration_ms, 'release object')
        time.sleep(wait_sec)

        bucket_level = self._increment_bucket_fill(goal_handle, bucket_id)

        self._feedback(goal_handle, 'clean_trash: arm center', 0.90)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        self._set_gripper(goal_handle, 0.0, duration_ms, 'close gripper idle')
        return bucket_id, bucket_level

    def _increment_bucket_fill(self, goal_handle, bucket_id):
        bucket_id = bucket_id if bucket_id in (1, 2) else 1
        level = min(100, int(self._bucket_fill_percent.get(bucket_id, 0)) + 10)
        self._bucket_fill_percent[bucket_id] = level
        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level={level}',
            0.89,
        )
        return level

    def _reset_bucket_fill(self, goal_handle, bucket_id, progress=0.99):
        bucket_id = bucket_id if bucket_id in (1, 2) else 1
        self._bucket_fill_percent[bucket_id] = 0
        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level=0 dumped',
            progress,
        )

    def _maybe_auto_dump_full_bucket(self, goal_handle, bucket_id, bucket_level):
        if bucket_level < 100:
            return True, 'bucket is not full'

        self._load_dump_bucket_poses()
        if bucket_id not in self._dump_bucket_poses:
            self._feedback(
                goal_handle,
                f'bucket_fill: bucket_id={bucket_id} level=100 no_dump_pose',
                0.90,
            )
            return True, f'bucket {bucket_id} full but dump pose is not set'

        self._feedback(
            goal_handle,
            f'bucket_fill: bucket_id={bucket_id} level=100 auto_dump_start',
            0.90,
        )
        success, message = self._dump_single_bucket(
            goal_handle,
            bucket_id,
            base_progress=0.90,
            span=0.09,
        )
        if not success:
            return False, message

        self._reset_bucket_fill(goal_handle, bucket_id)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_CENTER)
        return True, message
