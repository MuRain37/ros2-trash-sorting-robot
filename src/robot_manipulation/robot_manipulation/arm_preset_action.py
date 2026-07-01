"""机械臂预设姿态 Action 节点：接收动作类型并委托 MoveIt 执行."""

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_arm.arm_moveit_controller import ArmMoveItController
from robot_arm.arm_moveit_controller import ZERO_ANGLES
from robot_interfaces.action import ArmPreset
from robot_manipulation.arm_presets import preset_name
from robot_manipulation.arm_presets import preset_to_joint_positions


class ArmPresetActionServer(Node):
    """提供 /arm_preset Action，用于归中、左桶、右桶三个预设姿态."""

    def __init__(self):
        super().__init__('arm_preset_action_server')

        # MoveIt 规划组名
        self.declare_parameter('group_name', 'plan')
        # MoveGroup Action 名
        self.declare_parameter('move_action_name', '/move_action')
        # 对外提供的预设姿态 Action 名
        self.declare_parameter('action_name', '/arm_preset')

        self._group_name = self.get_parameter('group_name').value
        self._move_action_name = self.get_parameter('move_action_name').value
        self._action_name = self.get_parameter('action_name').value

        callback_group = ReentrantCallbackGroup()
        # 复用 MoveIt 控制器，最终仍通过 /move_action 规划和执行
        self._moveit = ArmMoveItController(
            self,
            self._group_name,
            self._move_action_name,
            callback_group,
        )
        # 预设姿态 Action：目标是动作类型，反馈是执行进度和当前关节角
        self._action_server = ActionServer(
            self,
            ArmPreset,
            self._action_name,
            execute_callback=self._execute_arm_preset,
            goal_callback=self._handle_arm_preset_goal,
            cancel_callback=self._handle_cancel,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f'Arm preset action ready: action={self._action_name}, '
            f'move_group={self._move_action_name}, group={self._group_name}'
        )

    def _handle_arm_preset_goal(self, goal_request):
        """只接受已定义的预设动作类型."""
        action_type = int(goal_request.action_type)
        if not self._is_known_action(action_type):
            self.get_logger().warning(
                f'Rejecting unknown arm preset action_type={action_type}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Arm preset goal accepted: action_type={action_type} '
            f'({preset_name(action_type)})'
        )
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        """接受取消请求，实际中断由反馈回调通知 MoveIt 等待循环."""
        self.get_logger().info('Arm preset cancel requested')
        return CancelResponse.ACCEPT

    def _execute_arm_preset(self, goal_handle):
        """将预设 raw 舵机值转换为 MoveIt 关节目标并执行."""
        goal = goal_handle.request
        action_type = int(goal.action_type)
        action_name = preset_name(action_type)
        self.get_logger().info(
            f'Executing arm preset: action_type={action_type} ({action_name})'
        )

        # 先发布 0 进度，方便调用端马上看到目标已进入执行流程
        self._publish_feedback(goal_handle, 0.0, ZERO_ANGLES)
        try:
            joint_positions = preset_to_joint_positions(action_type)
        except ValueError as exc:
            result = self._result(False, str(exc), ZERO_ANGLES)
            goal_handle.abort()
            return result

        # move_to_joint_positions 会在发送/等待 MoveIt 结果时持续调用反馈回调
        success, final_angles, message = self._moveit.move_to_joint_positions(
            joint_positions,
            lambda progress, angles: self._publish_feedback(
                goal_handle,
                progress,
                angles,
            ),
        )

        # 如果调用端已经取消目标，把 action 标记为 canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, 'arm preset canceled', final_angles)

        # MoveIt 成功才标记 succeed；失败则 abort，并把失败信息放进结果
        if success:
            self._publish_feedback(goal_handle, 1.0, final_angles)
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return self._result(success, message, final_angles)

    def _publish_feedback(self, goal_handle, progress, angles):
        """发布 action 反馈；返回 False 时通知控制器停止等待."""
        feedback = ArmPreset.Feedback()
        feedback.progress = float(progress)
        feedback.current_angles = self._fixed_angle_array(angles)
        goal_handle.publish_feedback(feedback)
        return not goal_handle.is_cancel_requested

    def _result(self, success, message, final_angles):
        """构造固定格式的 action 结果."""
        result = ArmPreset.Result()
        result.success = bool(success)
        result.message = str(message)
        result.final_angles = self._fixed_angle_array(final_angles)
        return result

    def _fixed_angle_array(self, angles):
        """保证 final_angles/current_angles 总是 5 个浮点数."""
        if angles is None or len(angles) != 5:
            return list(ZERO_ANGLES)
        return [float(angle) for angle in angles]

    def _is_known_action(self, action_type):
        """通过预设查表和 raw 值校验判断目标是否可执行."""
        try:
            preset_to_joint_positions(action_type)
        except ValueError:
            return False
        return True


def main(args=None):
    """启动机械臂预设姿态 Action 节点."""
    rclpy.init(args=args)
    node = ArmPresetActionServer()
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
