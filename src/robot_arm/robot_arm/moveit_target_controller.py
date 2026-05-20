import math
import threading
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from robot_arm.kinematics import solve
from robot_interfaces.action import ArmTarget


DEG_TO_RAD = math.pi / 180.0
ZERO_ANGLES = [0.0, 0.0, 0.0, 0.0]


class ArmMoveItTargetController(Node):
    """Send x/y/z/pitch arm targets through MoveIt's move_group action."""

    MOVEIT_JOINTS = [
        'arm_base_joint',
        'arm_shoulder_joint',
        'arm_elbow_joint',
        'arm_wrist_joint',
        'arm_tool_rotate_joint',
        'virtual_tool_roll_joint',
        'virtual_tool_pitch_joint',
        'virtual_tool_yaw_joint',
    ]

    def __init__(self):
        super().__init__('arm_moveit_target_controller')

        self.declare_parameter('group_name', 'plan')
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('action_name', '/arm_moveit_target')
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
        self.declare_parameter('goal_joint_tolerance', 0.001)
        self.declare_parameter('wait_for_move_group_timeout', 10.0)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('shutdown_after_startup_goal', True)
        self.declare_parameter('x')
        self.declare_parameter('y')
        self.declare_parameter('z')
        self.declare_parameter('pitch', -1.0)

        self._group_name = self.get_parameter('group_name').value
        self._move_action_name = self.get_parameter('move_action_name').value
        self._action_name = self.get_parameter('action_name').value
        self._allowed_planning_time = float(
            self.get_parameter('allowed_planning_time').value
        )
        self._num_planning_attempts = int(
            self.get_parameter('num_planning_attempts').value
        )
        self._velocity_scaling = float(
            self.get_parameter('max_velocity_scaling_factor').value
        )
        self._acceleration_scaling = float(
            self.get_parameter('max_acceleration_scaling_factor').value
        )
        self._goal_joint_tolerance = float(
            self.get_parameter('goal_joint_tolerance').value
        )
        self._wait_timeout = float(
            self.get_parameter('wait_for_move_group_timeout').value
        )
        self._plan_only = bool(self.get_parameter('plan_only').value)
        self._shutdown_after_startup_goal = bool(
            self.get_parameter('shutdown_after_startup_goal').value
        )

        callback_group = ReentrantCallbackGroup()
        self._move_group = ActionClient(
            self,
            MoveGroup,
            self._move_action_name,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            ArmTarget,
            self._action_name,
            execute_callback=self._execute_arm_target,
            goal_callback=self._handle_arm_target,
            callback_group=callback_group,
        )

        self._startup_timer = None
        startup_target = self._read_startup_target()
        if startup_target is not None:
            self._startup_timer = self.create_timer(
                0.1,
                lambda: self._send_startup_target(startup_target),
                callback_group=callback_group,
            )

        self.get_logger().info(
            f'Arm MoveIt target controller ready: action={self._action_name}, '
            f'move_group={self._move_action_name}, group={self._group_name}'
        )

    def _read_startup_target(self):
        x = self._optional_float_parameter('x')
        y = self._optional_float_parameter('y')
        z = self._optional_float_parameter('z')
        if x is None or y is None or z is None:
            return None

        pitch = float(self.get_parameter('pitch').value)
        return (x, y, z, pitch)

    def _optional_float_parameter(self, name):
        parameter = self.get_parameter(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            return None
        return float(parameter.value)

    def _send_startup_target(self, target):
        if self._startup_timer is not None:
            self._startup_timer.cancel()

        x, y, z, pitch = target
        success, _angles = self._move_to_target(x, y, z, pitch)

        if success:
            self.get_logger().info('Startup target completed')
        else:
            self.get_logger().error('Startup target failed')

        if self._shutdown_after_startup_goal:
            threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def _handle_arm_target(self, goal_request):
        self.get_logger().info(
            f'MoveIt target: [{goal_request.x:.2f}, '
            f'{goal_request.y:.2f}, {goal_request.z:.2f}] cm, '
            f'pitch={goal_request.end_angle:.2f} deg'
        )
        return GoalResponse.ACCEPT

    def _execute_arm_target(self, goal_handle):
        goal = goal_handle.request
        pitch = None if goal.end_angle < -0.5 else goal.end_angle

        feedback = ArmTarget.Feedback()
        feedback.progress = 0.0
        feedback.current_angles = list(ZERO_ANGLES)
        goal_handle.publish_feedback(feedback)

        success, final_angles = self._move_to_target(
            goal.x,
            goal.y,
            goal.z,
            pitch,
            goal_handle,
        )

        result = ArmTarget.Result()
        result.success = success
        result.final_angles = self._fixed_angle_array(final_angles)

        if success:
            feedback.progress = 1.0
            feedback.current_angles = self._fixed_angle_array(final_angles)
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _move_to_target(self, x, y, z, pitch, arm_goal_handle=None):
        ik_result = solve(
            [x, y, z],
            angle=pitch,
            verify=False,
            plot=False,
            collision_check=True,
        )
        if ik_result is False:
            self.get_logger().warning(
                f'IK failed for target [{x:.2f}, {y:.2f}, {z:.2f}] cm'
            )
            return False, list(ZERO_ANGLES)

        theta1, theta2, theta3, theta4, solved_pitch = ik_result
        target_angles_deg = [theta1, theta2, theta3, theta4]
        joint_positions = [
            theta1 * DEG_TO_RAD,
            theta2 * DEG_TO_RAD,
            theta3 * DEG_TO_RAD,
            theta4 * DEG_TO_RAD,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        formatted_angles = self._format_angles(target_angles_deg)
        self.get_logger().info(
            f'IK solution: joints={formatted_angles} deg, '
            f'pitch={solved_pitch:.2f} deg'
        )

        server_ready = self._move_group.wait_for_server(
            timeout_sec=self._wait_timeout
        )
        if not server_ready:
            self.get_logger().error(
                f'MoveIt action server {self._move_action_name} is not ready'
            )
            return False, target_angles_deg

        goal_msg = self._build_move_group_goal(joint_positions)
        send_future = self._move_group.send_goal_async(goal_msg)
        if not self._wait_for_future(
            send_future,
            arm_goal_handle,
            0.1,
            target_angles_deg,
        ):
            self.get_logger().error('Interrupted while sending MoveIt goal')
            return False, target_angles_deg

        move_goal_handle = send_future.result()
        if move_goal_handle is None or not move_goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the target')
            return False, target_angles_deg

        result_future = move_goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            arm_goal_handle,
            0.5,
            target_angles_deg,
        ):
            self.get_logger().error(
                'Interrupted while waiting for MoveIt result'
            )
            return False, target_angles_deg

        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'MoveIt failed with error_code={error_code}'
            )
            return False, target_angles_deg

        self.get_logger().info('MoveIt execution complete')
        return True, target_angles_deg

    def _build_move_group_goal(self, joint_positions):
        goal_msg = MoveGroup.Goal()

        request = goal_msg.request
        request.group_name = self._group_name
        request.num_planning_attempts = self._num_planning_attempts
        request.allowed_planning_time = self._allowed_planning_time
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = self._acceleration_scaling
        request.start_state.is_diff = True

        request.workspace_parameters.header.frame_id = 'base_link'
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        constraints.name = 'arm_xyz_pitch_joint_goal'
        constraints.joint_constraints = [
            self._joint_constraint(joint_name, position)
            for joint_name, position
            in zip(self.MOVEIT_JOINTS, joint_positions)
        ]
        request.goal_constraints = [constraints]

        goal_msg.planning_options.plan_only = self._plan_only
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 2
        goal_msg.planning_options.replan_delay = 0.1
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        planning_scene = goal_msg.planning_options.planning_scene_diff
        planning_scene.robot_state.is_diff = True

        return goal_msg

    def _joint_constraint(self, joint_name, position):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = position
        constraint.tolerance_above = self._goal_joint_tolerance
        constraint.tolerance_below = self._goal_joint_tolerance
        constraint.weight = 1.0
        return constraint

    def _wait_for_future(
        self,
        future,
        arm_goal_handle=None,
        progress=0.0,
        current_angles=None,
    ):
        feedback_time = 0.0
        while rclpy.ok() and not future.done():
            should_publish = (
                arm_goal_handle is not None
                and time.monotonic() >= feedback_time
            )
            if should_publish:
                feedback = ArmTarget.Feedback()
                feedback.progress = progress
                feedback.current_angles = self._fixed_angle_array(
                    current_angles
                )
                arm_goal_handle.publish_feedback(feedback)
                feedback_time = time.monotonic() + 0.5
            time.sleep(0.02)
        return future.done()

    def _fixed_angle_array(self, angles):
        if angles is None or len(angles) != 4:
            return list(ZERO_ANGLES)
        return [float(angle) for angle in angles]

    def _format_angles(self, angles):
        return '[' + ', '.join(f'{angle:.2f}' for angle in angles) + ']'


def main(args=None):
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
