"""机械臂目标 Action 节点：接收坐标目标并委托 MoveIt 执行。"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from robot_arm.arm_link_lengths import install_urdf_link_length_loader
from robot_arm.arm_moveit_controller import ArmMoveItController
from robot_arm.arm_moveit_controller import ZERO_ANGLES
from robot_interfaces.action import ArmTarget


class ArmMoveItTargetController(Node):
    """提供 /arm_moveit_target Action 服务。"""

    def __init__(self):
        super().__init__('arm_moveit_target_controller')

        # MoveIt 规划组名
        self.declare_parameter('group_name', 'plan')
        # MoveGroup Action 名
        self.declare_parameter('move_action_name', '/move_action')
        # 对外提供的机械臂目标 Action 名
        self.declare_parameter('action_name', '/arm_moveit_target')

        self._group_name = self.get_parameter('group_name').value
        self._move_action_name = self.get_parameter('move_action_name').value
        self._action_name = self.get_parameter('action_name').value

        callback_group = ReentrantCallbackGroup()
        self._moveit = ArmMoveItController(
            self,
            self._group_name,
            self._move_action_name,
            callback_group,
        )
        self._action_server = ActionServer(
            self,
            ArmTarget,
            self._action_name,
            execute_callback=self._execute_arm_target,
            goal_callback=self._handle_arm_target,
            callback_group=callback_group,
        )
        self._link_length_sub = install_urdf_link_length_loader(
            self,
            self._moveit.set_link_lengths,
        )

        self.get_logger().info(
            f'Arm MoveIt target controller ready: action={self._action_name}, '
            f'move_group={self._move_action_name}, group={self._group_name}'
        )

    def _handle_arm_target(self, goal_request):
        """接受机械臂目标请求。"""
        self.get_logger().info(
            f'MoveIt target: [{goal_request.x:.2f}, '
            f'{goal_request.y:.2f}, {goal_request.z:.2f}] cm, '
            f'pitch={goal_request.end_angle:.2f} deg, '
            f'gripper_rotation={goal_request.gripper_rotation:.2f} deg'
        )
        return GoalResponse.ACCEPT

    def _execute_arm_target(self, goal_handle):
        """执行目标并回传进度。"""
        goal = goal_handle.request
        pitch = None if goal.end_angle < -0.5 else goal.end_angle

        self._publish_feedback(goal_handle, 0.0, ZERO_ANGLES)
        success, final_angles = self._moveit.move_to_target(
            goal.x,
            goal.y,
            goal.z,
            pitch,
            goal.gripper_rotation,
            lambda progress, angles: self._publish_feedback(
                goal_handle,
                progress,
                angles,
            ),
        )

        result = ArmTarget.Result()
        result.success = success
        result.final_angles = self._fixed_angle_array(final_angles)

        if success:
            self._publish_feedback(goal_handle, 1.0, final_angles)
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _publish_feedback(self, goal_handle, progress, angles):
        feedback = ArmTarget.Feedback()
        feedback.progress = progress
        feedback.current_angles = self._fixed_angle_array(angles)
        goal_handle.publish_feedback(feedback)

    def _fixed_angle_array(self, angles):
        if angles is None or len(angles) != 5:
            return list(ZERO_ANGLES)
        return [float(angle) for angle in angles]


def main(args=None):
    """启动机械臂 MoveIt 目标节点。"""
    rclpy.init(args=args)
    node = ArmMoveItTargetController()
    executor = rclpy.executors.MultiThreadedExecutor()
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
