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
    FIXED_END_JOINTS = [
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
        self.declare_parameter('ik_timeout', 0.5)
        self.declare_parameter('ik_call_timeout', 1.0)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('target_link', 'tool0')
        self.declare_parameter('target_origin_x', 0.10)
        self.declare_parameter('target_origin_y', 0.0)
        self.declare_parameter('target_origin_z', 0.025)
        self.declare_parameter('arm_l1_cm', 3.38648)
        self.declare_parameter('arm_l2_cm', 12.9416)
        self.declare_parameter('arm_l3_cm', 12.9444)
        self.declare_parameter('arm_l4_cm', 11.38648)
        self.declare_parameter('auto_pitch_step', 5.0)
        self.declare_parameter(
            'auto_pitch_values',
            '45,60,30,75,15,90,0,-15,-30,-45,-60,-75,-90',
        )
        self.declare_parameter('fixed_end_joint_position', 0.0)
        self.declare_parameter('avoid_collisions_in_ik', False)
        self.declare_parameter('use_current_state_as_ik_seed', False)
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
        self._ik_call_timeout = float(
            self.get_parameter('ik_call_timeout').value
        )
        self._target_frame = self.get_parameter('target_frame').value
        self._target_link = self.get_parameter('target_link').value
        self._target_origin_x = float(
            self.get_parameter('target_origin_x').value
        )
        self._target_origin_y = float(
            self.get_parameter('target_origin_y').value
        )
        self._target_origin_z = float(
            self.get_parameter('target_origin_z').value
        )
        self._arm_l1_cm = float(self.get_parameter('arm_l1_cm').value)
        self._arm_l2_cm = float(self.get_parameter('arm_l2_cm').value)
        self._arm_l3_cm = float(self.get_parameter('arm_l3_cm').value)
        self._arm_l4_cm = float(self.get_parameter('arm_l4_cm').value)
        self._auto_pitch_step = float(
            self.get_parameter('auto_pitch_step').value
        )
        self._auto_pitch_values = self.get_parameter('auto_pitch_values').value
        self._fixed_end_joint_position = float(
            self.get_parameter('fixed_end_joint_position').value
        )
        self._avoid_collisions_in_ik = bool(
            self.get_parameter('avoid_collisions_in_ik').value
        )
        self._use_current_state_as_ik_seed = bool(
            self.get_parameter('use_current_state_as_ik_seed').value
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

        last_failure = None
        for candidate_pitch in self._pitch_candidates(pitch):
            ik_result = self._compute_moveit_ik(
                x,
                y,
                z,
                candidate_pitch,
                arm_goal_handle,
            )
            if ik_result is None:
                last_failure = (
                    f'IK failed at pitch={candidate_pitch:.2f} deg'
                    + (f': {self._last_ik_error}' if self._last_ik_error else '')
                )
                continue

            joint_positions, target_angles_deg = ik_result
            self.get_logger().info(
                f'MoveIt IK solution: pitch={candidate_pitch:.2f} deg, '
                f'joints={self._format_angles(target_angles_deg)} deg, '
                f'end joints fixed at {self._fixed_end_joint_position:.3f} rad'
            )

            success, final_angles, failure = self._execute_moveit_joint_goal(
                joint_positions,
                target_angles_deg,
                arm_goal_handle,
            )
            if success:
                if pitch is None:
                    self.get_logger().info(
                        f'MoveIt selected pitch={candidate_pitch:.2f} deg'
                    )
                self.get_logger().info('MoveIt execution complete')
                return True, final_angles

            last_failure = (
                f'MoveIt failed at pitch={candidate_pitch:.2f} deg: {failure}'
            )
            if pitch is not None:
                break

        self.get_logger().error(
            'MoveIt target failed'
            + (f': {last_failure}' if last_failure else '')
        )
        return False, list(ZERO_ANGLES)

    def _execute_moveit_joint_goal(
        self,
        joint_positions,
        target_angles_deg,
        arm_goal_handle,
    ):
        server_ready = self._move_group.wait_for_server(
            timeout_sec=self._wait_timeout
        )
        if not server_ready:
            return (
                False,
                list(ZERO_ANGLES),
                f'action server {self._move_action_name} is not ready',
            )

        goal_msg = self._build_move_group_goal(joint_positions)
        send_future = self._move_group.send_goal_async(goal_msg)
        if not self._wait_for_future(
            send_future,
            arm_goal_handle,
            0.1,
            target_angles_deg,
        ):
            return False, list(ZERO_ANGLES), 'interrupted while sending goal'

        move_goal_handle = send_future.result()
        if move_goal_handle is None or not move_goal_handle.accepted:
            return False, list(ZERO_ANGLES), 'target rejected'

        result_future = move_goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            arm_goal_handle,
            0.5,
            target_angles_deg,
        ):
            return False, list(ZERO_ANGLES), 'interrupted while waiting result'

        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            return (
                False,
                list(ZERO_ANGLES),
                self._moveit_error_text(error_code),
            )

        final_angles = self._extract_final_angles(result, target_angles_deg)
        return True, final_angles, None

    def _compute_moveit_ik(self, x_cm, y_cm, z_cm, pitch_deg, arm_goal_handle):
        if not self._compute_ik.wait_for_service(
            timeout_sec=self._wait_for_ik_timeout
        ):
            self.get_logger().error('MoveIt /compute_ik service is not ready')
            return None

        self._last_ik_error = None
        for candidate_pitch in self._pitch_candidates(pitch_deg):
            for seed_positions in self._ik_seed_positions(
                x_cm,
                y_cm,
                z_cm,
                candidate_pitch,
            ):
                request = self._build_ik_request(
                    x_cm,
                    y_cm,
                    z_cm,
                    candidate_pitch,
                    seed_positions,
                )
                future = self._compute_ik.call_async(request)
                if not self._wait_for_future(
                    future,
                    arm_goal_handle,
                    0.1,
                    None,
                    self._ik_call_timeout,
                ):
                    self.get_logger().error(
                        'Interrupted or timed out while waiting for MoveIt IK'
                    )
                    return None

                response = future.result()
                if response is None:
                    self._last_ik_error = 'empty response'
                    continue

                error_code = response.error_code.val
                if error_code != MoveItErrorCodes.SUCCESS:
                    self._last_ik_error = self._moveit_error_text(error_code)
                    continue

                positions = self._extract_ik_positions(
                    response.solution.joint_state
                )
                if positions is None:
                    self._last_ik_error = 'IK response has no complete joint state'
                    continue

                if not self._fixed_end_joints_match(positions):
                    self._last_ik_error = (
                        'IK changed fixed end joints: '
                        f'{self._format_joint_positions(positions, self.FIXED_END_JOINTS)}'
                    )
                    continue

                joint_positions = [
                    positions.get(joint_name, 0.0)
                    for joint_name in self.MOVEIT_JOINTS
                ]
                target_angles_deg = [
                    joint_positions[i] / DEG_TO_RAD
                    for i in range(4)
                ]
                return joint_positions, target_angles_deg

        return None

    def _fixed_end_joints_match(self, positions):
        for joint_name in self.FIXED_END_JOINTS:
            position = positions.get(joint_name)
            if position is None:
                continue
            if abs(position - self._fixed_end_joint_position) > (
                self._goal_joint_tolerance
            ):
                return False
        return True

    def _build_ik_request(self, x_cm, y_cm, z_cm, pitch_deg, seed_positions):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self._group_name
        ik_request.robot_state.joint_state = self._seed_joint_state(
            seed_positions
        )
        ik_request.robot_state.is_diff = False
        ik_request.constraints.joint_constraints = [
            self._joint_constraint(joint_name, self._fixed_end_joint_position)
            for joint_name in self.FIXED_END_JOINTS
        ]
        ik_request.avoid_collisions = self._avoid_collisions_in_ik
        ik_request.ik_link_name = self._target_link
        ik_request.pose_stamped = self._target_pose(x_cm, y_cm, z_cm, pitch_deg)
        ik_request.timeout.sec = int(self._ik_timeout)
        ik_request.timeout.nanosec = int(
            (self._ik_timeout - int(self._ik_timeout)) * 1e9
        )
        return request

    def _seed_joint_state(self, positions):
        msg = JointState()
        msg.name = list(self.MOVEIT_JOINTS)
        msg.position = [
            positions.get(joint_name, 0.0)
            for joint_name in self.MOVEIT_JOINTS
        ]

        for joint_name in self.FIXED_END_JOINTS:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                msg.position[index] = self._fixed_end_joint_position

        return msg

    def _ik_seed_positions(self, x_cm, y_cm, z_cm, pitch_deg):
        yaw = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        seeds = []

        if self._use_current_state_as_ik_seed and self._latest_joint_state is not None:
            seeds.append(dict(zip(
                self._latest_joint_state.name,
                self._latest_joint_state.position,
            )))

        seeds.extend(self._geometric_seed_positions(x_cm, y_cm, z_cm, pitch_deg))
        seeds.extend([
            {},
            {
                'arm_base_joint': yaw,
                'arm_shoulder_joint': -0.4155,
                'arm_elbow_joint': 0.9526,
                'arm_wrist_joint': 1.0134,
            },
            {
                'arm_base_joint': yaw,
                'arm_shoulder_joint': -0.25,
                'arm_elbow_joint': 2.45,
                'arm_wrist_joint': 0.15,
            },
            {
                'arm_base_joint': yaw,
                'arm_shoulder_joint': 0.5,
                'arm_elbow_joint': 2.0,
                'arm_wrist_joint': 1.0,
            },
        ])

        unique_seeds = []
        seen = set()
        for seed in seeds:
            fixed_seed = {
                joint_name: seed.get(joint_name, 0.0)
                for joint_name in self.MOVEIT_JOINTS
            }
            for joint_name in self.FIXED_END_JOINTS:
                fixed_seed[joint_name] = self._fixed_end_joint_position
            key = tuple(
                round(fixed_seed[joint_name], 4)
                for joint_name in self.MOVEIT_JOINTS
            )
            if key not in seen:
                seen.add(key)
                unique_seeds.append(fixed_seed)

        return unique_seeds

    def _geometric_seed_positions(self, x_cm, y_cm, z_cm, pitch_deg):
        l1 = self._arm_l1_cm
        l2 = self._arm_l2_cm
        l3 = self._arm_l3_cm
        l4 = self._arm_l4_cm

        theta1 = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        end_pitch = -pitch_deg * DEG_TO_RAD
        end_dir_x = math.cos(end_pitch) * math.cos(theta1)
        end_dir_y = math.cos(end_pitch) * math.sin(theta1)
        end_dir_z = math.sin(end_pitch)

        wrist_x = x_cm - l4 * end_dir_x
        wrist_y = y_cm - l4 * end_dir_y
        wrist_z = z_cm - l4 * end_dir_z
        r = wrist_x * math.cos(theta1) + wrist_y * math.sin(theta1)
        z_from_shoulder = wrist_z - l1
        distance = math.hypot(r, z_from_shoulder)

        if distance > l2 + l3 or distance < abs(l2 - l3) or distance < 1e-9:
            return []

        cos_elbow_inner = (l2 * l2 + l3 * l3 - distance * distance) / (
            2.0 * l2 * l3
        )
        cos_elbow_inner = max(-1.0, min(1.0, cos_elbow_inner))
        elbow_inner = math.acos(cos_elbow_inner)

        alpha = math.atan2(z_from_shoulder, r)
        cos_beta = (l2 * l2 + distance * distance - l3 * l3) / (
            2.0 * l2 * distance
        )
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        candidates = []
        theta2 = math.pi / 2.0 - (alpha + beta)
        theta3 = math.pi - elbow_inner
        theta4 = math.pi / 2.0 + pitch_deg * DEG_TO_RAD - theta2 - theta3
        candidates.append((theta1, theta2, theta3, theta4))

        if beta > 1e-9:
            theta2 = math.pi / 2.0 - (alpha - beta)
            theta3 = elbow_inner - math.pi
            theta4 = math.pi / 2.0 + pitch_deg * DEG_TO_RAD - theta2 - theta3
            candidates.append((theta1, theta2, theta3, theta4))

        seeds = []
        for theta1, theta2, theta3, theta4 in candidates:
            seeds.append({
                'arm_base_joint': self._normalize_angle(theta1),
                'arm_shoulder_joint': self._normalize_angle(theta2),
                'arm_elbow_joint': self._normalize_angle(theta3),
                'arm_wrist_joint': self._normalize_angle(theta4),
            })
        return seeds

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _target_pose(self, x_cm, y_cm, z_cm, pitch_deg):
        pose = PoseStamped()
        pose.header.frame_id = self._target_frame
        pose.pose.position.x = self._target_origin_x + x_cm / 100.0
        pose.pose.position.y = self._target_origin_y + y_cm / 100.0
        pose.pose.position.z = self._target_origin_z + z_cm / 100.0
        pose.pose.orientation = self._target_orientation(x_cm, y_cm, pitch_deg)
        return pose

    def _target_orientation(self, x_cm, y_cm, pitch_deg):
        yaw = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        pitch_from_vertical = math.pi / 2.0 + pitch_deg * DEG_TO_RAD
        return self._quaternion_from_rpy(0.0, pitch_from_vertical, yaw)

    def _pitch_candidates(self, pitch_deg):
        if pitch_deg is not None:
            return [pitch_deg]

        configured_values = self._parse_pitch_values(self._auto_pitch_values)
        if configured_values:
            return configured_values

        step = self._auto_pitch_step if self._auto_pitch_step > 0.0 else 5.0
        values = []
        pitch = -90.0
        while pitch <= 90.0 + 1e-9:
            values.append(pitch)
            pitch += step
        return sorted(values, key=lambda value: abs(value - 45.0))

    def _parse_pitch_values(self, value):
        values = []
        for item in str(value).split(','):
            item = item.strip()
            if not item:
                continue
            try:
                pitch = float(item)
            except ValueError:
                self.get_logger().warning(
                    f'Ignoring invalid auto_pitch_values entry: {item}'
                )
                continue
            if pitch < -90.0 or pitch > 90.0:
                self.get_logger().warning(
                    f'Ignoring out-of-range auto pitch: {pitch:.2f}'
                )
                continue
            values.append(pitch)
        return values

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

    def _moveit_error_text(self, error_code):
        error_name = MOVEIT_ERROR_NAMES.get(error_code, 'UNKNOWN')
        return f'error_code={error_code} ({error_name})'

    def _format_joint_positions(self, positions, joint_names):
        values = []
        for joint_name in joint_names:
            position = positions.get(joint_name)
            if position is None:
                values.append(f'{joint_name}=missing')
            else:
                values.append(f'{joint_name}={position:.4f}')
        return ', '.join(values)

    def _extract_final_angles(self, result, fallback_angles):
        trajectory = result.executed_trajectory
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            joint_trajectory = result.planned_trajectory.joint_trajectory
        if not joint_trajectory.points:
            return self._fixed_angle_array(fallback_angles)

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
        timeout_sec=None,
    ):
        start_time = time.monotonic()
        feedback_time = 0.0
        while rclpy.ok() and not future.done():
            if (
                timeout_sec is not None
                and time.monotonic() - start_time >= timeout_sec
            ):
                future.cancel()
                return False

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
