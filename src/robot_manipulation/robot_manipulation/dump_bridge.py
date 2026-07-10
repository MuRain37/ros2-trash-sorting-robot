"""机械臂倒垃圾流程桥接：通过调用硬件桥服务实现固定顺序动作."""

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
from std_srvs.srv import Trigger


# 硬件桥服务名
LIFT_STEPS_SERVICE = 'set_lift_steps'
SERVO_ANGLE_SERVICE = 'set_lift_servo_angle'
GRIPPER_SERVICE = 'set_lift_gripper'
PICK_BUCKET_SERVICE = 'dump_pick_bucket'
POUR_BUCKET_SERVICE = 'dump_pour_bucket'
PLACE_BUCKET_SERVICE = 'dump_place_bucket'
HOME_SERVICE = 'dump_home'

# 倒垃圾流程参数（与硬件桥校准值保持一致）
SERVO_CENTER = 500.0
HORIZONTAL_SERVO_ID = 7
GRIPPER_ROTATE_SERVO_ID = 8
LIFT_GRIPPER_OPEN_RAW = 400.0
LIFT_GRIPPER_CLOSE_RAW = 465.0

DUMP_SERVO_DURATION_MS = 2000.0
DUMP_GRIPPER_DURATION_MS = 1000.0
DUMP_STEPPER_WAIT_SEC = 2.8
DUMP_SERVO_WAIT_SEC = DUMP_SERVO_DURATION_MS / 1000.0
DUMP_GRIPPER_WAIT_SEC = DUMP_GRIPPER_DURATION_MS / 1000.0
DUMP_POUR_HORIZONTAL_DELAY_SEC = 0.1
DUMP_SETTLE_SEC = 0.1
DUMP_LIFT_UP_STEPS = 665.0
DUMP_LIFT_DOWN_STEPS = 76.0

DUMP_BUCKET_1_RAW = 883.0
DUMP_BUCKET_2_RAW = 110.0
DUMP_BUCKET_1_ROTATION_RAW = 1000.0
DUMP_BUCKET_2_ROTATION_RAW = 0.0


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
        dump_trash_service='',
        pick_bucket_service=PICK_BUCKET_SERVICE,
        pour_bucket_service=POUR_BUCKET_SERVICE,
        place_bucket_service=PLACE_BUCKET_SERVICE,
        home_service=HOME_SERVICE,
        servo_duration_ms=DUMP_SERVO_DURATION_MS,
        gripper_duration_ms=DUMP_GRIPPER_DURATION_MS,
        stepper_wait_sec=DUMP_STEPPER_WAIT_SEC,
        servo_wait_sec=DUMP_SERVO_WAIT_SEC,
        gripper_wait_sec=DUMP_GRIPPER_WAIT_SEC,
        pour_horizontal_delay_sec=DUMP_POUR_HORIZONTAL_DELAY_SEC,
        settle_sec=DUMP_SETTLE_SEC,
        sleep_fn=time.sleep,
    ):
        self._node = node
        self._servo_duration_ms = float(servo_duration_ms)
        self._gripper_duration_ms = float(gripper_duration_ms)
        self._stepper_wait_sec = float(stepper_wait_sec)
        self._servo_wait_sec = float(servo_wait_sec)
        self._gripper_wait_sec = float(gripper_wait_sec)
        self._pour_horizontal_delay_sec = float(pour_horizontal_delay_sec)
        self._settle_sec = float(settle_sec)
        self._sleep = sleep_fn
        self._dump_lock = threading.Lock()

        callback_group = ReentrantCallbackGroup()

        self._lift_steps_client = node.create_client(SetLiftSteps, LIFT_STEPS_SERVICE, callback_group=callback_group,)
        self._servo_angle_client = node.create_client(SetServoAngle, SERVO_ANGLE_SERVICE, callback_group=callback_group,)
        self._gripper_client = node.create_client(SetGripper, GRIPPER_SERVICE, callback_group=callback_group,)
        self._dump_trash_service = None
        if dump_trash_service:
            self._dump_trash_service = node.create_service(
                DumpTrash,
                dump_trash_service,
                self._on_dump_trash_command,
            )
        self._pick_bucket_service = node.create_service(
            DumpTrash,
            pick_bucket_service,
            self._on_pick_bucket_command,
        )
        self._pour_bucket_service = node.create_service(
            DumpTrash,
            pour_bucket_service,
            self._on_pour_bucket_command,
        )
        self._place_bucket_service = node.create_service(
            DumpTrash,
            place_bucket_service,
            self._on_place_bucket_command,
        )
        self._home_service = node.create_service(
            Trigger,
            home_service,
            self._on_home_command,
        )

    def _on_pick_bucket_command(self, request, response):
        try:
            bucket_id = self._normalize_bucket_id(request.bucket_id)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response
        return self._run_locked_step(
            response,
            f'pick onboard bucket {bucket_id}',
            lambda: self._pick_bucket(bucket_id),
        )

    def _on_pour_bucket_command(self, request, response):
        try:
            bucket_id = self._normalize_bucket_id(request.bucket_id)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response
        return self._run_locked_step(
            response,
            f'pour onboard bucket {bucket_id}',
            lambda: self._pour_bucket(bucket_id),
        )

    def _on_place_bucket_command(self, request, response):
        try:
            bucket_id = self._normalize_bucket_id(request.bucket_id)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response
        return self._run_locked_step(
            response,
            f'place onboard bucket {bucket_id}',
            lambda: self._place_bucket(bucket_id),
        )

    def _on_home_command(self, request, response):
        del request
        return self._run_locked_step(response, 'home dump mechanism', self._home)

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
        """保留旧完整服务：机构动作本身不包含底盘后退。"""
        self._pick_bucket(bucket_id)
        self._pour_bucket(bucket_id)
        self._place_bucket(bucket_id)
        self._home()

    def _pick_bucket(self, bucket_id):
        bucket_raw = self._bucket_raw(bucket_id)
        self._call_lift(DUMP_LIFT_UP_STEPS)
        self._call_horizontal(bucket_raw)
        self._call_lift(DUMP_LIFT_DOWN_STEPS)
        self._call_gripper(LIFT_GRIPPER_CLOSE_RAW)

    def _pour_bucket(self, bucket_id):
        dump_rotation_raw = self._dump_rotation_raw(bucket_id)
        self._call_lift(DUMP_LIFT_UP_STEPS)
        self._call_horizontal(SERVO_CENTER, self._pour_horizontal_delay_sec)
        self._sleep(0.5)
        self._call_gripper_rotation(dump_rotation_raw)

    def _place_bucket(self, bucket_id):
        bucket_raw = self._bucket_raw(bucket_id)
        self._call_gripper_rotation_async(SERVO_CENTER)
        self._sleep(0.5)
        self._call_horizontal(bucket_raw)
        self._call_lift(DUMP_LIFT_DOWN_STEPS)
        self._call_gripper(LIFT_GRIPPER_OPEN_RAW)

    def _home(self):
        self._call_lift(DUMP_LIFT_UP_STEPS)
        self._call_horizontal_async(SERVO_CENTER)
        self._call_gripper_rotation_async(SERVO_CENTER) 
        self._call_gripper_async(LIFT_GRIPPER_OPEN_RAW)
        self._sleep(max(self._servo_wait_sec, self._gripper_wait_sec))
        self._call_lift(DUMP_LIFT_DOWN_STEPS)

    def _call_lift(self, target_steps):
        """调用步进电机服务并等待动作完成."""
        self._call_lift_async(target_steps)
        self._sleep(self._stepper_wait_sec + self._settle_sec)

    def _call_lift_async(self, target_steps):
        """调用步进电机服务，不等待动作完成。"""
        request = SetLiftSteps.Request()
        request.target_steps = float(target_steps)
        self._node.get_logger().info(
            f'dump lift target_steps={request.target_steps:.1f}'
        )
        self._call_required_service(
            self._lift_steps_client,
            request,
            'lift steps',
        )

    def _call_horizontal(self, raw_position, wait_sec=None):
        """调用水平旋转舵机服务并等待动作完成."""
        self._call_horizontal_async(raw_position)
        self._sleep_with_settle(self._servo_wait_sec if wait_sec is None else wait_sec)

    def _call_horizontal_async(self, raw_position):
        """调用水平旋转舵机服务，不等待动作完成。"""
        self._node.get_logger().info(
            'dump horizontal servo: '
            f'raw={float(raw_position):.1f}, '
            f'duration_ms={self._servo_duration_ms:.0f}'
        )
        self._call_servo(HORIZONTAL_SERVO_ID, raw_position, 'lift horizontal rotation')

    def _call_gripper_rotation(self, raw_position, wait_sec=None):
        """调用夹爪旋转舵机服务并等待动作完成."""
        self._call_gripper_rotation_async(raw_position)
        self._sleep_with_settle(self._servo_wait_sec if wait_sec is None else wait_sec)

    def _call_gripper_rotation_async(self, raw_position):
        """调用夹爪旋转舵机服务，不等待动作完成。"""
        self._node.get_logger().info(
            'dump gripper rotation servo: '
            f'raw={float(raw_position):.1f}, '
            f'duration_ms={self._servo_duration_ms:.0f}'
        )
        self._call_servo(
            GRIPPER_ROTATE_SERVO_ID,
            raw_position,
            'lift gripper rotation',
        )

    def _call_servo(self, servo_id, raw_position, label):
        """调用通用升降-倾倒舵机服务。"""
        request = SetServoAngle.Request()
        request.servo_id = int(servo_id)
        request.angle = float(raw_position)
        request.duration_ms = self._servo_duration_ms
        self._call_required_service(
            self._servo_angle_client,
            request,
            label,
        )

    def _call_gripper(self, raw_position):
        """调用夹爪开合舵机服务并等待动作完成."""
        self._call_gripper_async(raw_position)
        self._sleep_with_settle(self._gripper_wait_sec)

    def _call_gripper_async(self, raw_position):
        """调用夹爪开合舵机服务，不等待动作完成。"""
        open_ratio = raw_to_gripper_open_ratio(raw_position)
        request = SetGripper.Request()
        request.open_ratio = float(open_ratio)
        request.duration_ms = self._gripper_duration_ms
        self._node.get_logger().info(
            'dump gripper servo: '
            f'raw={float(raw_position):.1f}, '
            f'open_ratio={request.open_ratio:.3f}, '
            f'duration_ms={request.duration_ms:.0f}'
        )
        self._call_required_service(
            self._gripper_client,
            request,
            'lift gripper',
        )

    def _call_required_service(self, client, request, label):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'{label} service not available')

        response = client.call(request)
        if response is None:
            raise RuntimeError(f'{label} service returned no response')
        if not response.success:
            raise RuntimeError(f'{label} rejected: {response.message}')

        self._node.get_logger().info(
            f'{label} accepted: {response.message}'
        )
        return response

    def _run_locked_step(self, response, label, action):
        if not self._dump_lock.acquire(blocking=False):
            response.success = False
            response.message = 'dump trash sequence is already running'
            return response

        try:
            action()
        except Exception as e:
            response.success = False
            response.message = f'{label} failed: {e}'
            return response
        finally:
            self._dump_lock.release()

        response.success = True
        response.message = f'{label} completed'
        return response

    def _normalize_bucket_id(self, bucket_id):
        bucket_id = int(bucket_id)
        if bucket_id not in (1, 2):
            raise ValueError(f'bucket_id must be 1 or 2: {bucket_id}')
        return bucket_id

    def _bucket_raw(self, bucket_id):
        if bucket_id == 1:
            return DUMP_BUCKET_1_RAW
        return DUMP_BUCKET_2_RAW

    def _dump_rotation_raw(self, bucket_id):
        if bucket_id == 1:
            return DUMP_BUCKET_1_ROTATION_RAW
        return DUMP_BUCKET_2_ROTATION_RAW

    def _sleep_with_settle(self, wait_sec):
        self._sleep(max(0.0, float(wait_sec)) + self._settle_sec)


class DumpTrashNode(Node):
    """倒垃圾流程节点：提供 /dump_trash Action 服务."""

    def __init__(self):
        super().__init__('dump_trash_node')
        self.declare_parameter('dump_trash_service', '')
        self.declare_parameter('pick_bucket_service', PICK_BUCKET_SERVICE)
        self.declare_parameter('pour_bucket_service', POUR_BUCKET_SERVICE)
        self.declare_parameter('place_bucket_service', PLACE_BUCKET_SERVICE)
        self.declare_parameter('home_service', HOME_SERVICE)
        self.declare_parameter('servo_duration_ms', DUMP_SERVO_DURATION_MS)
        self.declare_parameter('gripper_duration_ms', DUMP_GRIPPER_DURATION_MS)
        self.declare_parameter('stepper_wait_sec', DUMP_STEPPER_WAIT_SEC)
        self.declare_parameter('servo_wait_sec', DUMP_SERVO_WAIT_SEC)
        self.declare_parameter('gripper_wait_sec', DUMP_GRIPPER_WAIT_SEC)
        self.declare_parameter(
            'pour_horizontal_delay_sec',
            DUMP_POUR_HORIZONTAL_DELAY_SEC,
        )
        self.declare_parameter('settle_sec', DUMP_SETTLE_SEC)
        self._bridge = DumpTrashBridge(
            self,
            dump_trash_service=str(self.get_parameter('dump_trash_service').value),
            pick_bucket_service=str(self.get_parameter('pick_bucket_service').value),
            pour_bucket_service=str(self.get_parameter('pour_bucket_service').value),
            place_bucket_service=str(self.get_parameter('place_bucket_service').value),
            home_service=str(self.get_parameter('home_service').value),
            servo_duration_ms=float(self.get_parameter('servo_duration_ms').value),
            gripper_duration_ms=float(
                self.get_parameter('gripper_duration_ms').value
            ),
            stepper_wait_sec=float(self.get_parameter('stepper_wait_sec').value),
            servo_wait_sec=float(self.get_parameter('servo_wait_sec').value),
            gripper_wait_sec=float(self.get_parameter('gripper_wait_sec').value),
            pour_horizontal_delay_sec=float(
                self.get_parameter('pour_horizontal_delay_sec').value
            ),
            settle_sec=float(self.get_parameter('settle_sec').value),
        )
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
