"""机械臂倒垃圾流程桥接：通过调用硬件桥服务实现固定顺序动作."""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_interfaces.srv import DumpTrash
from robot_interfaces.srv import SetGripper
from robot_interfaces.srv import SetLiftSteps
from robot_interfaces.srv import SetServoAngle


# 硬件桥服务名
LIFT_STEPS_SERVICE = 'set_lift_steps'
HORIZONTAL_SERVICE = 'set_lift_horizontal_rotation'
GRIPPER_ROTATION_SERVICE = 'set_lift_gripper_rotation'
GRIPPER_SERVICE = 'set_lift_gripper'

# 倒垃圾流程参数（与硬件桥校准值保持一致）
SERVO_CENTER = 500.0
HORIZONTAL_MAX_DEGREES = 180.0
HORIZONTAL_DIRECTION = 1.0
GRIPPER_ROTATE_MAX_DEGREES = 180.0
GRIPPER_ROTATE_DIRECTION = 1.0
LIFT_GRIPPER_OPEN_RAW = 500.0
LIFT_GRIPPER_CLOSE_RAW = 585.0

DUMP_SERVO_DURATION_MS = 2000.0
DUMP_GRIPPER_DURATION_MS = 1000.0
DUMP_STEPPER_WAIT_SEC = 2.8
DUMP_SETTLE_SEC = 0.1

DUMP_BUCKET_1_RAW = 883.0
DUMP_BUCKET_2_RAW = 110.0


def raw_to_angle(raw_position, max_degrees, direction):
    """舵机 raw 值 → 角度（度）."""
    degrees = (raw_position - SERVO_CENTER) * (max_degrees / SERVO_CENTER)
    return degrees * direction


def raw_to_gripper_open_ratio(raw_position):
    """夹爪舵机 raw 值 → 开度比例 [0, 1]."""
    return (raw_position - LIFT_GRIPPER_CLOSE_RAW) / (
        LIFT_GRIPPER_OPEN_RAW - LIFT_GRIPPER_CLOSE_RAW
    )


class DumpTrashBridge:
    """倒垃圾流程桥接器：通过调用硬件桥服务执行固定顺序倒垃圾动作."""

    def __init__(
        self,
        node,
        dump_trash_service='dump_trash',
        servo_duration_ms=DUMP_SERVO_DURATION_MS,
        gripper_duration_ms=DUMP_GRIPPER_DURATION_MS,
        stepper_wait_sec=DUMP_STEPPER_WAIT_SEC,
        settle_sec=DUMP_SETTLE_SEC,
        sleep_fn=time.sleep,
    ):
        self._node = node
        self._servo_duration_ms = float(servo_duration_ms)
        self._gripper_duration_ms = float(gripper_duration_ms)
        self._stepper_wait_sec = float(stepper_wait_sec)
        self._settle_sec = float(settle_sec)
        self._sleep = sleep_fn
        self._dump_lock = threading.Lock()

        callback_group = ReentrantCallbackGroup()

        self._lift_steps_client = node.create_client(SetLiftSteps, LIFT_STEPS_SERVICE, callback_group=callback_group,)
        self._horizontal_client = node.create_client(SetServoAngle, HORIZONTAL_SERVICE, callback_group=callback_group,)
        self._gripper_rotation_client = node.create_client(SetServoAngle, GRIPPER_ROTATION_SERVICE, callback_group=callback_group,)
        self._gripper_client = node.create_client(SetGripper, GRIPPER_SERVICE, callback_group=callback_group,)
        self._dump_trash_service = node.create_service( DumpTrash, dump_trash_service, self._on_dump_trash_command,)

    def _on_dump_trash_command(self, request, response):
        """接收垃圾桶编号，执行完整升降-夹取-倾倒流程."""
        try:
            bucket_id = self._normalize_bucket_id(request.bucket_id)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        if not self._dump_lock.acquire(blocking=False):
            response.success = False
            response.message = 'dump trash sequence is already running'
            return response

        try:
            self._execute_dump_trash_sequence(bucket_id)
        except Exception as e:
            response.success = False
            response.message = f'dump trash sequence failed: {e}'
            return response
        finally:
            self._dump_lock.release()

        response.success = True
        response.message = f'dump trash sequence completed for bucket {bucket_id}'
        return response

    def _execute_dump_trash_sequence(self, bucket_id):
        """按固定顺序执行倒垃圾动作（升降→水平→夹取→倾倒→复位）."""
        bucket_raw = self._bucket_raw(bucket_id)

        # 1. 升起到桶位高度
        self._call_lift(665.0)
        # 2. 水平旋转到目标桶
        self._call_horizontal(bucket_raw)
        # 3. 下降到桶位
        self._call_lift(75.0)
        # 4. 夹爪扣紧
        self._call_gripper(LIFT_GRIPPER_CLOSE_RAW)
        # 5. 升起
        self._call_lift(665.0)

        # 6. 水平回中
        self._call_horizontal(SERVO_CENTER)
        # 7. 夹爪旋转到倾倒方向
        self._call_gripper_rotation(0.0)
        # 8. 继续旋转
        self._call_gripper_rotation(SERVO_CENTER)

        # 9. 回到桶位水平
        self._call_horizontal(bucket_raw)
        # 10. 下降到桶位
        self._call_lift(75.0)
        # 11. 松开夹爪
        self._call_gripper(LIFT_GRIPPER_OPEN_RAW)

        # 12. 升起到桶位高度
        self._call_lift(665.0)
        # 13. 水平回中
        self._call_horizontal(SERVO_CENTER)
        # 14. 降回初始高度
        self._call_lift(75.0)

    def _call_lift(self, target_steps):
        """调用步进电机服务并等待动作完成."""
        request = SetLiftSteps.Request()
        request.target_steps = float(target_steps)
        if self._lift_steps_client.wait_for_service(timeout_sec=1.0):
            self._lift_steps_client.call(request)
        self._sleep(self._stepper_wait_sec + self._settle_sec)

    def _call_horizontal(self, raw_position):
        """调用水平旋转舵机服务并等待动作完成."""
        angle = raw_to_angle(raw_position, HORIZONTAL_MAX_DEGREES, HORIZONTAL_DIRECTION)
        request = SetServoAngle.Request()
        request.angle = float(angle)
        request.duration_ms = self._servo_duration_ms
        if self._horizontal_client.wait_for_service(timeout_sec=1.0):
            self._horizontal_client.call(request)
        self._sleep(self._servo_duration_ms / 1000.0 + self._settle_sec)

    def _call_gripper_rotation(self, raw_position):
        """调用夹爪旋转舵机服务并等待动作完成."""
        angle = raw_to_angle(
            raw_position, GRIPPER_ROTATE_MAX_DEGREES, GRIPPER_ROTATE_DIRECTION,
        )
        request = SetServoAngle.Request()
        request.angle = float(angle)
        request.duration_ms = self._servo_duration_ms
        if self._gripper_rotation_client.wait_for_service(timeout_sec=1.0):
            self._gripper_rotation_client.call(request)
        self._sleep(self._servo_duration_ms / 1000.0 + self._settle_sec)

    def _call_gripper(self, raw_position):
        """调用夹爪开合舵机服务并等待动作完成."""
        open_ratio = raw_to_gripper_open_ratio(raw_position)
        request = SetGripper.Request()
        request.open_ratio = float(open_ratio)
        request.duration_ms = self._gripper_duration_ms
        if self._gripper_client.wait_for_service(timeout_sec=1.0):
            self._gripper_client.call(request)
        self._sleep(self._gripper_duration_ms / 1000.0 + self._settle_sec)

    def _normalize_bucket_id(self, bucket_id):
        bucket_id = int(bucket_id)
        if bucket_id not in (1, 2):
            raise ValueError(f'bucket_id must be 1 or 2: {bucket_id}')
        return bucket_id

    def _bucket_raw(self, bucket_id):
        if bucket_id == 1:
            return DUMP_BUCKET_1_RAW
        return DUMP_BUCKET_2_RAW


class DumpTrashNode(Node):
    """倒垃圾流程节点：提供 /dump_trash Action 服务."""

    def __init__(self):
        super().__init__('dump_trash_node')
        self._bridge = DumpTrashBridge(self)
        self.get_logger().info('Dump trash node ready')


def main(args=None):
    """启动倒垃圾流程节点."""
    rclpy.init(args=args)
    node = DumpTrashNode()
    executor = MultiThreadedExecutor()
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
