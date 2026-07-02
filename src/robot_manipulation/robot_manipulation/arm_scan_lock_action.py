"""机械臂扫描锁定 Action：用两组姿态对齐真实机械臂与 RViz/TF."""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_interfaces.action import ArmScanLock


ARM_JOINT_NAMES = [
    'arm_base_joint',
    'arm_shoulder_joint',
    'arm_elbow_joint',
    'arm_wrist_joint',
    'arm_tool_rotate_joint',
    'virtual_tool_roll_joint',
    'virtual_tool_pitch_joint',
    'virtual_tool_yaw_joint',
]
ARM_JOINT_COUNT = 5
VIRTUAL_JOINT_COUNT = 3
ZERO_JOINTS = [0.0] * ARM_JOINT_COUNT
ZERO_VIRTUAL_JOINTS = [0.0] * VIRTUAL_JOINT_COUNT


class ArmScanLockActionServer(Node):
    """
    提供 /arm_scan_lock Action.

    扫描期间：
    - model_joint_positions 发布到 arm_position_fb，供 ros2_control/joint_states/TF 使用。
    - hardware_joint_positions 发布到 plan_controller/joint_trajectory，驱动真实机械臂。
    """

    def __init__(self):
        super().__init__('arm_scan_lock_action_server')

        self.declare_parameter('action_name', '/arm_scan_lock')
        self.declare_parameter('model_feedback_topic', '/arm_position_fb')
        self.declare_parameter(
            'hardware_trajectory_topic',
            '/plan_controller/joint_trajectory',
        )
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('trajectory_duration', 0.2)
        self.declare_parameter('model_joint_positions', list(ZERO_JOINTS))
        self.declare_parameter('hardware_joint_positions', list(ZERO_JOINTS))

        self._action_name = self.get_parameter('action_name').value
        self._model_feedback_topic = (
            self.get_parameter('model_feedback_topic').value
        )
        self._hardware_trajectory_topic = (
            self.get_parameter('hardware_trajectory_topic').value
        )
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._trajectory_duration = float(
            self.get_parameter('trajectory_duration').value
        )
        self._model_joint_positions = self._fixed_joint_array(
            self.get_parameter('model_joint_positions').value
        )
        self._hardware_joint_positions = self._fixed_joint_array(
            self.get_parameter('hardware_joint_positions').value
        )

        self._model_pub = self.create_publisher(
            Float64MultiArray,
            self._model_feedback_topic,
            10,
        )
        self._hardware_pub = self.create_publisher(
            JointTrajectory,
            self._hardware_trajectory_topic,
            10,
        )

        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ArmScanLock,
            self._action_name,
            execute_callback=self._execute_scan_lock,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f'Arm scan lock ready: action={self._action_name}, '
            f'model_feedback_topic={self._model_feedback_topic}, '
            f'hardware_trajectory_topic={self._hardware_trajectory_topic}'
        )

    def _handle_goal(self, goal_request):
        """接受非负保持时间；0 表示一直保持到 cancel."""
        if float(goal_request.hold_seconds) < 0.0:
            self.get_logger().warning(
                f'Rejecting arm scan lock hold_seconds='
                f'{goal_request.hold_seconds}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        """允许调用方结束扫描锁定."""
        self.get_logger().info('Arm scan lock cancel requested')
        return CancelResponse.ACCEPT

    def _execute_scan_lock(self, goal_handle):
        """周期性发布模型反馈和真实机械臂目标，直到超时或取消."""
        hold_seconds = float(goal_handle.request.hold_seconds)
        start_time = time.monotonic()
        period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.1

        self.get_logger().info(
            f'Executing arm scan lock: hold_seconds={hold_seconds}'
        )

        # 硬件目标只发送一次，避免持续替换控制器轨迹导致机械臂抖动。
        self._publish_hardware_target()

        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            self._publish_model_feedback()
            self._publish_feedback(goal_handle, elapsed)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    False,
                    'arm scan lock canceled',
                )

            if hold_seconds > 0.0 and elapsed >= hold_seconds:
                goal_handle.succeed()
                return self._result(True, 'arm scan lock completed')

            time.sleep(period)

        goal_handle.abort()
        return self._result(False, 'rclpy shutdown during arm scan lock')

    def _publish_model_feedback(self):
        """发布给 RViz/TF 使用的模型关节反馈."""
        feedback_msg = Float64MultiArray()
        feedback_msg.data = list(self._model_joint_positions)
        self._model_pub.publish(feedback_msg)

    def _publish_hardware_target(self):
        """发布一次真实机械臂轨迹目标."""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = (
            list(self._hardware_joint_positions) + list(ZERO_VIRTUAL_JOINTS)
        )
        point.time_from_start.sec = int(self._trajectory_duration)
        point.time_from_start.nanosec = int(
            (self._trajectory_duration % 1.0) * 1_000_000_000
        )
        trajectory_msg.points = [point]
        self._hardware_pub.publish(trajectory_msg)

    def _publish_lock_targets(self):
        """发布模型反馈和真实机械臂轨迹目标，用于测试和手动触发."""
        self._publish_model_feedback()
        self._publish_hardware_target()

    def _publish_feedback(self, goal_handle, elapsed):
        feedback = ArmScanLock.Feedback()
        feedback.elapsed_seconds = float(elapsed)
        feedback.model_joint_positions = list(self._model_joint_positions)
        feedback.hardware_joint_positions = list(self._hardware_joint_positions)
        goal_handle.publish_feedback(feedback)

    def _result(self, success, message):
        result = ArmScanLock.Result()
        result.success = bool(success)
        result.message = str(message)
        result.model_joint_positions = list(self._model_joint_positions)
        result.hardware_joint_positions = list(self._hardware_joint_positions)
        return result

    def _fixed_joint_array(self, values):
        """确保参数是 5 个有限浮点数."""
        if values is None:
            values = ZERO_JOINTS
        values = [float(value) for value in values]
        if len(values) != ARM_JOINT_COUNT:
            raise ValueError(
                f'expected {ARM_JOINT_COUNT} joint positions, '
                f'got {len(values)}'
            )
        return values


def main(args=None):
    """启动机械臂扫描锁定 Action 节点."""
    rclpy.init(args=args)
    node = ArmScanLockActionServer()
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
