import math
import threading
import time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from robot_interfaces.action import ArmTarget


DEG_TO_RAD = math.pi / 180.0
ZERO_ANGLES = [0.0, 0.0, 0.0, 0.0]
MOVEIT_ERROR_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


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
        self.declare_parameter('wait_for_ik_timeout', 10.0)
        self.declare_parameter('ik_timeout', 0.2)
        self.declare_parameter('target_frame', 'arm_link0')
        self.declare_parameter('target_link', 'tool0')
        self.declare_parameter('auto_pitch_step', 5.0)
        self.declare_parameter('locked_tool_rotate_joint', 'arm_tool_rotate_joint')
        self.declare_parameter('locked_tool_rotate_position', 0.0)
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
        self._wait_for_ik_timeout = float(
            self.get_parameter('wait_for_ik_timeout').value
        )
        self._ik_timeout = float(self.get_parameter('ik_timeout').value)
        self._target_frame = self.get_parameter('target_frame').value
        self._target_link = self.get_parameter('target_link').value
        self._auto_pitch_step = float(
            self.get_parameter('auto_pitch_step').value
        )
        self._locked_tool_rotate_joint = self.get_parameter(
            'locked_tool_rotate_joint'
        ).value
        self._locked_tool_rotate_position = float(
            self.get_parameter('locked_tool_rotate_position').value
        )
        self._plan_only = bool(self.get_parameter('plan_only').value)
        self._shutdown_after_startup_goal = bool(
            self.get_parameter('shutdown_after_startup_goal').value
        )
        self._latest_joint_state = None

        callback_group = ReentrantCallbackGroup()
        self._move_group = ActionClient(
            self,
            MoveGroup,
            self._move_action_name,
            callback_group=callback_group,
        )
        self._compute_ik = self.create_client(
            GetPositionIK,
            '/compute_ik',
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
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            10,
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

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg

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
        pitch_text = 'auto' if pitch is None else f'{pitch:.2f} deg'
        self.get_logger().info(
            f'MoveIt pose target: link={self._target_link}, '
            f'frame={self._target_frame}, '
            f'position=[{x:.2f}, {y:.2f}, {z:.2f}] cm, '
            f'pitch={pitch_text}'
        )

        ik_result = self._compute_moveit_ik(x, y, z, pitch, arm_goal_handle)
        if ik_result is None:
            return False, list(ZERO_ANGLES)

        joint_positions, target_angles_deg = ik_result
        self.get_logger().info(
            f'MoveIt IK solution: joints={self._format_angles(target_angles_deg)} deg, '
            f'{self._locked_tool_rotate_joint}='
            f'{self._locked_tool_rotate_position:.3f} rad'
        )

        server_ready = self._move_group.wait_for_server(
            timeout_sec=self._wait_timeout
        )
        if not server_ready:
            self.get_logger().error(
                f'MoveIt action server {self._move_action_name} is not ready'
            )
            return False, list(ZERO_ANGLES)

        goal_msg = self._build_move_group_goal(joint_positions)
        send_future = self._move_group.send_goal_async(goal_msg)
        if not self._wait_for_future(
            send_future,
            arm_goal_handle,
            0.1,
            target_angles_deg,
        ):
            self.get_logger().error('Interrupted while sending MoveIt goal')
            return False, list(ZERO_ANGLES)

        move_goal_handle = send_future.result()
        if move_goal_handle is None or not move_goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the target')
            return False, list(ZERO_ANGLES)

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
            return False, list(ZERO_ANGLES)

        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            error_name = MOVEIT_ERROR_NAMES.get(error_code, 'UNKNOWN')
            self.get_logger().error(
                f'MoveIt failed with error_code={error_code} '
                f'({error_name})'
            )
            return False, list(ZERO_ANGLES)

        final_angles = self._extract_final_angles(result)
        self.get_logger().info('MoveIt execution complete')
        return True, final_angles

    def _compute_moveit_ik(self, x_cm, y_cm, z_cm, pitch_deg, arm_goal_handle):
        if not self._compute_ik.wait_for_service(
            timeout_sec=self._wait_for_ik_timeout
        ):
            self.get_logger().error('MoveIt /compute_ik service is not ready')
            return None

        for candidate_pitch in self._pitch_candidates(pitch_deg):
            request = self._build_ik_request(x_cm, y_cm, z_cm, candidate_pitch)
            future = self._compute_ik.call_async(request)
            if not self._wait_for_future(
                future,
                arm_goal_handle,
                0.1,
                None,
            ):
                self.get_logger().error('Interrupted while waiting for MoveIt IK')
                return None

            response = future.result()
            if response is None:
                continue

            error_code = response.error_code.val
            if error_code != MoveItErrorCodes.SUCCESS:
                continue

            positions = self._extract_ik_positions(response.solution.joint_state)
            if positions is None:
                continue

            locked_position = positions.get(self._locked_tool_rotate_joint)
            if locked_position is None:
                continue
            if abs(locked_position - self._locked_tool_rotate_position) > (
                self._goal_joint_tolerance
            ):
                continue

            joint_positions = [
                positions.get(joint_name, 0.0)
                for joint_name in self.MOVEIT_JOINTS
            ]
            target_angles_deg = [
                joint_positions[i] / DEG_TO_RAD
                for i in range(4)
            ]
            if pitch_deg is None:
                self.get_logger().info(
                    f'MoveIt IK selected pitch={candidate_pitch:.2f} deg'
                )
            return joint_positions, target_angles_deg

        self.get_logger().error(
            'MoveIt IK failed for target while keeping '
            f'{self._locked_tool_rotate_joint} fixed'
        )
        return None

    def _build_ik_request(self, x_cm, y_cm, z_cm, pitch_deg):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self._group_name
        ik_request.robot_state.joint_state = self._seed_joint_state()
        ik_request.robot_state.is_diff = True
        ik_request.constraints.joint_constraints = [
            self._joint_constraint(
                self._locked_tool_rotate_joint,
                self._locked_tool_rotate_position,
            ),
        ]
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = self._target_link
        ik_request.pose_stamped = self._target_pose(x_cm, y_cm, z_cm, pitch_deg)
        ik_request.timeout.sec = int(self._ik_timeout)
        ik_request.timeout.nanosec = int(
            (self._ik_timeout - int(self._ik_timeout)) * 1e9
        )
        return request

    def _seed_joint_state(self):
        if self._latest_joint_state is not None:
            positions = dict(zip(
                self._latest_joint_state.name,
                self._latest_joint_state.position,
            ))
        else:
            positions = {}

        msg = JointState()
        msg.name = list(self.MOVEIT_JOINTS)
        msg.position = [
            positions.get(joint_name, 0.0)
            for joint_name in self.MOVEIT_JOINTS
        ]

        if self._locked_tool_rotate_joint in msg.name:
            index = msg.name.index(self._locked_tool_rotate_joint)
            msg.position[index] = self._locked_tool_rotate_position

        return msg

    def _target_pose(self, x_cm, y_cm, z_cm, pitch_deg):
        pose = PoseStamped()
        pose.header.frame_id = self._target_frame
        pose.pose.position.x = x_cm / 100.0
        pose.pose.position.y = y_cm / 100.0
        pose.pose.position.z = z_cm / 100.0
        pose.pose.orientation = self._target_orientation(x_cm, y_cm, pitch_deg)
        return pose

    def _target_orientation(self, x_cm, y_cm, pitch_deg):
        yaw = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        pitch_from_vertical = math.pi / 2.0 + pitch_deg * DEG_TO_RAD
        return self._quaternion_from_rpy(0.0, pitch_from_vertical, yaw)

    def _pitch_candidates(self, pitch_deg):
        if pitch_deg is not None:
            return [pitch_deg]

        step = self._auto_pitch_step if self._auto_pitch_step > 0.0 else 5.0
        values = []
        pitch = -90.0
        while pitch <= 90.0 + 1e-9:
            values.append(pitch)
            pitch += step
        return sorted(values, key=lambda value: abs(value - 45.0))

    def _extract_ik_positions(self, joint_state):
        if not joint_state.name or len(joint_state.name) != len(joint_state.position):
            return None
        return dict(zip(joint_state.name, joint_state.position))

    def _build_move_group_goal(self, joint_positions):
        goal_msg = MoveGroup.Goal()

        request = goal_msg.request
        request.group_name = self._group_name
        request.num_planning_attempts = self._num_planning_attempts
        request.allowed_planning_time = self._allowed_planning_time
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = self._acceleration_scaling
        request.start_state.is_diff = True

        request.workspace_parameters.header.frame_id = self._target_frame
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        constraints.name = 'moveit_ik_joint_goal'
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

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        orientation = Pose().orientation
        orientation.w = cr * cp * cy + sr * sp * sy
        orientation.x = sr * cp * cy - cr * sp * sy
        orientation.y = cr * sp * cy + sr * cp * sy
        orientation.z = cr * cp * sy - sr * sp * cy
        return orientation

    def _joint_constraint(self, joint_name, position):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = position
        constraint.tolerance_above = self._goal_joint_tolerance
        constraint.tolerance_below = self._goal_joint_tolerance
        constraint.weight = 1.0
        return constraint

    def _extract_final_angles(self, result):
        trajectory = result.executed_trajectory
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            joint_trajectory = result.planned_trajectory.joint_trajectory
        if not joint_trajectory.points:
            return list(ZERO_ANGLES)

        final_positions = joint_trajectory.points[-1].positions
        positions_by_name = dict(zip(joint_trajectory.joint_names, final_positions))
        angles = []
        for joint_name in self.MOVEIT_JOINTS[:4]:
            position = positions_by_name.get(joint_name)
            if position is None:
                return list(ZERO_ANGLES)
            angles.append(position / DEG_TO_RAD)
        return angles

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
