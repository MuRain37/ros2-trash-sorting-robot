"""机械臂 MoveIt 控制器：计算 IK 并发送 MoveGroup 执行目标。"""

import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState


DEG_TO_RAD = math.pi / 180.0
ZERO_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0]
MOVEIT_ERROR_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


class ArmMoveItController:
    ALLOWED_PLANNING_TIME = 5.0
    FIXED_END_JOINT_POSITION = 0.0
    GOAL_JOINT_TOLERANCE = 0.001
    IK_CALL_TIMEOUT, IK_TIMEOUT = 1.0, 0.5
    NUM_PLANNING_ATTEMPTS = 10
    TARGET_FRAME = 'base_link'
    TARGET_JOINT_COUNT = 5
    TARGET_LINK = 'tool0'
    DEFAULT_TARGET_ORIGIN_CM = (10.0, 0.0, 2.5)
    VELOCITY_SCALING = 0.2
    ACCELERATION_SCALING = 0.1
    WAIT_FOR_IK_TIMEOUT = 10.0
    WAIT_FOR_MOVE_GROUP_TIMEOUT = 10.0
    WORKSPACE_MIN = (-1.0, -1.0, -1.0)
    WORKSPACE_MAX = (1.0, 1.0, 1.0)
    HARDWARE_JOINT_LIMITS_DEG = {
        'arm_base_joint': 180.0,
        'arm_shoulder_joint': 121.0,
        'arm_elbow_joint': 121.0,
        'arm_wrist_joint': 121.0,
        'arm_tool_rotate_joint': 121.0,
    }

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
        'virtual_tool_roll_joint',
        'virtual_tool_pitch_joint',
        'virtual_tool_yaw_joint',
    ]

    def __init__(self, node, group_name, move_action_name, callback_group):
        self._node, self._group_name = node, group_name
        self._move_action_name = move_action_name
        self._arm_origin_cm = self.DEFAULT_TARGET_ORIGIN_CM
        self._arm_link_lengths_cm = None
        self._last_ik_error = None

        self._move_group = ActionClient(
            node, MoveGroup, move_action_name, callback_group=callback_group
        )
        self._compute_ik = node.create_client(
            GetPositionIK, '/compute_ik', callback_group=callback_group
        )

    def set_link_lengths(self, origin, lengths):
        """更新从 URDF 解析出的机械臂安装原点和连杆长度。"""
        self._arm_origin_cm = origin
        self._arm_link_lengths_cm = lengths
        self._node.get_logger().info(
            'Loaded arm geometry from /robot_description: '
            f'origin=[{origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f}] cm, '
            f'l1={lengths[0]:.2f}, l2={lengths[1]:.2f}, '
            f'l3={lengths[2]:.2f}, l4={lengths[3]:.2f} cm'
        )

    def move_to_target(self, x, y, z,ideal_pitch=90.0, gripper_rotation_deg=0.0, feedback_callback=None):
        """在 90°~-0° 逐度搜索 IK，选出最接近 ideal_pitch 的解并执行。

        ideal_pitch：get_ik_pitch 返回的理想俯仰角（度）。
                    None 表示自动模式，取最高可行俯仰角。
        """
        self._node.get_logger().info(
            f'position=[{x:.2f}, {y:.2f}, {z:.2f}] cm, '
            f'gripper_rotation={gripper_rotation_deg:.2f} deg'
        )

        tool_limit = self.HARDWARE_JOINT_LIMITS_DEG['arm_tool_rotate_joint']
        if abs(gripper_rotation_deg) > tool_limit:
            self._node.get_logger().error(
                'MoveIt target failed: gripper_rotation='
                f'{gripper_rotation_deg:.2f} deg not in +-120'
            )
            return False, list(ZERO_ANGLES)

        last_failure = None
        best_ik = None       # (joint_positions, target_angles_deg, candidate_pitch)
        best_error = float('inf')

        # 从 90° 到 0° 逐度搜索=
        for candidate_pitch in range(90, 0, -2):
            self._node.get_logger().info(f'pitch={candidate_pitch}')
            ik_result = self._compute_moveit_ik(
                x, y, z, candidate_pitch, gripper_rotation_deg, feedback_callback
            )
            if ik_result is None:
                last_failure = (
                    f'IK failed at pitch={candidate_pitch} deg'
                    + (f': {self._last_ik_error}' if self._last_ik_error else '')
                )
                continue

            joint_positions, target_angles_deg = ik_result

            if ideal_pitch is not None:
                error = abs(candidate_pitch - ideal_pitch)
                if error < best_error:
                    best_error = error
                    best_ik = (joint_positions, target_angles_deg, candidate_pitch)
                    if error < 5:
                        break  # 完美匹配
            else:
                # 自动模式：取最高可行俯仰角
                best_ik = (joint_positions, target_angles_deg, candidate_pitch)
                break

        if best_ik is None:
            self._node.get_logger().error(
                'MoveIt target failed'
                + (f': {last_failure}' if last_failure else '')
            )
            return False, list(ZERO_ANGLES)

        joint_positions, target_angles_deg, selected_pitch = best_ik

        if ideal_pitch is not None:
            self._node.get_logger().info(
                f'MoveIt IK solution: pitch={selected_pitch} deg '
                f'(ideal={ideal_pitch:.0f} deg, error={best_error:.0f} deg), '
                f'joints={self._format_angles(target_angles_deg)} deg'
            )
        else:
            self._node.get_logger().info(
                f'MoveIt IK solution: pitch={selected_pitch} deg, '
                f'joints={self._format_angles(target_angles_deg)} deg'
            )

        success, final_angles, failure = self._execute_moveit_joint_goal(
            joint_positions,
            target_angles_deg,
            feedback_callback,
        )
        if success:
            self._node.get_logger().info('MoveIt execution complete')
            return True, final_angles

        self._node.get_logger().error(
            f'MoveIt execution failed at pitch={selected_pitch} deg: {failure}'
        )
        return False, list(ZERO_ANGLES)

    def move_to_joint_positions(self, joint_positions, feedback_callback=None):
        """直接执行五个真实机械臂关节的 MoveIt joint goal。"""
        if len(joint_positions) != self.TARGET_JOINT_COUNT:
            message = (
                f'expected {self.TARGET_JOINT_COUNT} joints, '
                f'got {len(joint_positions)}'
            )
            self._node.get_logger().error(
                'MoveIt joint target failed: ' + message
            )
            return False, list(ZERO_ANGLES), message

        joint_positions = [float(position) for position in joint_positions]
        target_angles_deg = [
            position / DEG_TO_RAD
            for position in joint_positions
        ]

        limit_errors = []
        for joint_name, position in zip(
            self.MOVEIT_JOINTS[:self.TARGET_JOINT_COUNT],
            joint_positions,
        ):
            max_degrees = self.HARDWARE_JOINT_LIMITS_DEG[joint_name]
            degrees = position / DEG_TO_RAD
            if abs(degrees) > max_degrees:
                limit_errors.append(
                    f'{joint_name}={degrees:.2f} deg not in '
                    f'[-{max_degrees:.1f}, {max_degrees:.1f}]'
                )

        if limit_errors:
            message = 'joint target exceeds hardware limits: ' + ', '.join(
                limit_errors
            )
            self._node.get_logger().error(
                'MoveIt joint target failed: ' + message
            )
            return False, list(ZERO_ANGLES), message

        moveit_positions = list(joint_positions) + [
            self.FIXED_END_JOINT_POSITION
        ] * len(self.FIXED_END_JOINTS)
        self._node.get_logger().info(
            'MoveIt joint target: '
            f'joints={self._format_angles(target_angles_deg)} deg, '
            f'end joints fixed at {self.FIXED_END_JOINT_POSITION:.3f} rad'
        )

        success, final_angles, failure = self._execute_moveit_joint_goal(
            moveit_positions,
            target_angles_deg,
            feedback_callback,
        )
        if success:
            self._node.get_logger().info('MoveIt joint execution complete')
            return True, final_angles, 'joint target accepted'

        message = f'MoveIt joint target failed: {failure}'
        self._node.get_logger().error(message)
        return False, final_angles, message

    def _execute_moveit_joint_goal(
        self,
        joint_positions,
        target_angles_deg,
        feedback_callback,
    ):
        if not self._move_group.wait_for_server(
            timeout_sec=self.WAIT_FOR_MOVE_GROUP_TIMEOUT
        ):
            return (
                False,
                list(ZERO_ANGLES),
                f'action server {self._move_action_name} is not ready',
            )

        send_future = self._move_group.send_goal_async(
            self._build_move_group_goal(joint_positions)
        )
        if not self._wait_for_future(send_future, feedback_callback, 0.1, target_angles_deg):
            return False, list(ZERO_ANGLES), 'interrupted while sending goal'

        move_goal_handle = send_future.result()
        if move_goal_handle is None or not move_goal_handle.accepted:
            return False, list(ZERO_ANGLES), 'target rejected'

        result_future = move_goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future, feedback_callback, 0.5, target_angles_deg
        ):
            return False, list(ZERO_ANGLES), 'interrupted while waiting result'

        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            error_name = MOVEIT_ERROR_NAMES.get(error_code, 'UNKNOWN')
            return (
                False,
                list(ZERO_ANGLES),
                f'error_code={error_code} ({error_name})',
            )
        return True, self._extract_final_angles(result, target_angles_deg), None

        final_angles = self._extract_final_angles(result, target_angles_deg)
        return True, final_angles, None

    def _compute_moveit_ik(
            self, x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg, feedback_callback
    ):
        """使用多个初始种子求解 MoveIt IK。"""
        if not self._compute_ik.wait_for_service(timeout_sec=self.WAIT_FOR_IK_TIMEOUT):
            self._node.get_logger().error('MoveIt /compute_ik service is not ready')
            return None

        self._last_ik_error = None
        for seed_positions in self._ik_seed_positions(
            x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg
        ):
            future = self._compute_ik.call_async(self._build_ik_request(
                x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg, seed_positions)
            )
            if not self._wait_for_future(
                future, feedback_callback, 0.1, None, self.IK_CALL_TIMEOUT
            ):
                self._node.get_logger().error(
                    'Interrupted or timed out while waiting for MoveIt IK'
                )
                return None
            response = future.result()
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                error_code = response.error_code.val
                self._last_ik_error = (
                    f'error_code={error_code} '
                    f'({MOVEIT_ERROR_NAMES.get(error_code, "UNKNOWN")})'
                )
                continue

            joint_state = response.solution.joint_state
            if (
                not joint_state.name
                or len(joint_state.name) != len(joint_state.position)
            ):
                self._last_ik_error = 'IK response has no complete joint state'
                continue
            positions = dict(zip(joint_state.name, joint_state.position))
            # arm_tool_rotate_joint=0 求前4轴，成功后再把第5轴写入 joint goal。
            positions['arm_tool_rotate_joint'] = (
                gripper_rotation_deg * DEG_TO_RAD
            )

            fixed_errors = [
                f'{joint}={positions[joint]:.4f}'
                for joint in self.FIXED_END_JOINTS
                if (
                    joint in positions
                    and abs(positions[joint] - self.FIXED_END_JOINT_POSITION)
                    > self.GOAL_JOINT_TOLERANCE
                )
            ]
            if fixed_errors:
                self._last_ik_error = (
                    'IK changed fixed end joints: ' + ', '.join(fixed_errors)
                )
                continue

            limit_errors = []
            for joint, max_degrees in self.HARDWARE_JOINT_LIMITS_DEG.items():
                if joint not in positions:
                    limit_errors.append(f'{joint}=missing')
                    continue
                degrees = positions[joint] / DEG_TO_RAD
                if abs(degrees) > max_degrees:
                    limit_errors.append(
                        f'{joint}={degrees:.2f} deg '
                        f'not in [{-max_degrees:.1f}, {max_degrees:.1f}]'
                    )
            if limit_errors:
                self._last_ik_error = (
                    'IK exceeds hardware joint limits: ' + ', '.join(limit_errors)
                )
                continue

            joint_positions = [positions.get(joint_name, 0.0) 
                               for joint_name in self.MOVEIT_JOINTS]
            target_angles_deg = [joint_positions[i] / DEG_TO_RAD
                                 for i in range(5)]
            return joint_positions, target_angles_deg
        return None

    def _build_ik_request(self, x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg, seed_positions):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self._group_name
        ik_request.robot_state.joint_state = JointState(
            name=list(self.MOVEIT_JOINTS),
            position=[seed_positions.get(joint, 0.0) for joint in self.MOVEIT_JOINTS],
        )
        ik_request.robot_state.is_diff = False
        ik_request.constraints.joint_constraints = [
            self._joint_constraint(joint, 0.0) for joint in self.FIXED_END_JOINTS]
        ik_request.constraints.joint_constraints.append(
            self._joint_constraint(
                'arm_tool_rotate_joint',
                self.FIXED_END_JOINT_POSITION,
            )
        )
        ik_request.avoid_collisions = False
        ik_request.ik_link_name = self.TARGET_LINK
        ik_request.pose_stamped = self._target_pose(x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg)
        ik_request.timeout.sec = int(self.IK_TIMEOUT)
        ik_request.timeout.nanosec = int((self.IK_TIMEOUT - int(self.IK_TIMEOUT)) * 1e9)
        return request

    def _ik_seed_positions(self, x_cm, y_cm, z_cm, pitch_deg, gripper_rotation_deg):
        yaw = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        seeds = self._geometric_seed_positions(x_cm, y_cm, z_cm, pitch_deg)
        seeds.extend([{}, {'arm_base_joint': yaw}])

        unique_seeds = []
        seen = set()
        for seed in seeds:
            fixed_seed = {
                joint_name: seed.get(joint_name, 0.0)
                for joint_name in self.MOVEIT_JOINTS
            }
            for joint_name in self.FIXED_END_JOINTS:
                fixed_seed[joint_name] = self.FIXED_END_JOINT_POSITION
            fixed_seed['arm_tool_rotate_joint'] = self.FIXED_END_JOINT_POSITION
            key = tuple(
                round(fixed_seed[joint_name], 4)
                for joint_name in self.MOVEIT_JOINTS
            )
            if key not in seen:
                seen.add(key)
                unique_seeds.append(fixed_seed)
        return unique_seeds

    def _geometric_seed_positions(self, x_cm, y_cm, z_cm, pitch_deg):
        if self._arm_link_lengths_cm is None:
            return []

        l1, l2, l3, l4 = self._arm_link_lengths_cm
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

        cos_elbow_inner = max(
            -1.0,
            min(
                1.0,
                (l2 * l2 + l3 * l3 - distance * distance) / (2.0 * l2 * l3),
            ),
        )
        alpha = math.atan2(z_from_shoulder, r)
        cos_beta = max(
            -1.0,
            min(
                1.0,
                (l2 * l2 + distance * distance - l3 * l3)
                / (2.0 * l2 * distance),
            ),
        )
        elbow_inner = math.acos(cos_elbow_inner)
        beta = math.acos(cos_beta)
        candidates = []
        for shoulder_sign in (1.0, -1.0):
            if shoulder_sign < 0.0 and beta <= 1e-9:
                continue
            theta2 = math.pi / 2.0 - (alpha + shoulder_sign * beta)
            theta3 = (
                math.pi - elbow_inner
                if shoulder_sign > 0.0
                else elbow_inner - math.pi
            )
            theta4 = math.pi / 2.0 + pitch_deg * DEG_TO_RAD - theta2 - theta3
            candidates.append((theta1, theta2, theta3, theta4))

        return [
            {
                'arm_base_joint': math.atan2(math.sin(t1), math.cos(t1)),
                'arm_shoulder_joint': math.atan2(math.sin(t2), math.cos(t2)),
                'arm_elbow_joint': math.atan2(math.sin(t3), math.cos(t3)),
                'arm_wrist_joint': math.atan2(math.sin(t4), math.cos(t4)),
            }
            for t1, t2, t3, t4 in candidates
        ]

    def _target_pose(self, x_cm, y_cm, z_cm, pitch_deg=90.0, gripper_rotation_deg=0.0):
        pose = PoseStamped()
        origin_x, origin_y, origin_z = (v / 100.0 for v in self._arm_origin_cm)
        pose.header.frame_id = self.TARGET_FRAME
        pose.pose.position.x = origin_x + x_cm / 100.0
        pose.pose.position.y = origin_y + y_cm / 100.0
        pose.pose.position.z = origin_z + z_cm / 100.0
        yaw = math.atan2(y_cm, x_cm) if x_cm != 0.0 or y_cm != 0.0 else 0.0
        pose.pose.orientation = self._quaternion_from_rpy(
            0.0,
            math.pi / 2.0 + pitch_deg * DEG_TO_RAD,
            yaw,
        )
        return pose

    def _build_move_group_goal(self, joint_positions):
        goal_msg = MoveGroup.Goal()

        request = goal_msg.request
        request.group_name = self._group_name
        request.num_planning_attempts = self.NUM_PLANNING_ATTEMPTS
        request.allowed_planning_time = self.ALLOWED_PLANNING_TIME
        request.max_velocity_scaling_factor = self.VELOCITY_SCALING
        request.max_acceleration_scaling_factor = self.ACCELERATION_SCALING
        request.start_state.is_diff = True

        request.workspace_parameters.header.frame_id = self.TARGET_FRAME
        request.workspace_parameters.min_corner.x = self.WORKSPACE_MIN[0]
        request.workspace_parameters.min_corner.y = self.WORKSPACE_MIN[1]
        request.workspace_parameters.min_corner.z = self.WORKSPACE_MIN[2]
        request.workspace_parameters.max_corner.x = self.WORKSPACE_MAX[0]
        request.workspace_parameters.max_corner.y = self.WORKSPACE_MAX[1]
        request.workspace_parameters.max_corner.z = self.WORKSPACE_MAX[2]

        constraints = Constraints()
        constraints.name = 'moveit_ik_joint_goal'
        constraints.joint_constraints = [
            self._joint_constraint(joint_name, position)
            for joint_name, position
            in zip(self.MOVEIT_JOINTS, joint_positions)
        ]
        path_constraints = Constraints()
        path_constraints.name = 'fixed_end_joints_path'
        path_constraints.joint_constraints = [
            self._joint_constraint(joint_name, self.FIXED_END_JOINT_POSITION)
            for joint_name in self.FIXED_END_JOINTS
        ]
        request.goal_constraints = [constraints]
        request.path_constraints = path_constraints

        options = goal_msg.planning_options
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 2
        options.replan_delay = 0.1
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        return goal_msg

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        cy, sy = math.cos(half_yaw), math.sin(half_yaw)

        orientation = Pose().orientation
        orientation.w = cr * cp * cy + sr * sp * sy
        orientation.x = sr * cp * cy - cr * sp * sy
        orientation.y = cr * sp * cy + sr * cp * sy
        orientation.z = cr * cp * sy - sr * sp * cy
        return orientation

    def _multiply_quaternions(self, first, second):
        orientation = Pose().orientation
        orientation.w = (
            first.w * second.w
            - first.x * second.x
            - first.y * second.y
            - first.z * second.z
        )
        orientation.x = (
            first.w * second.x
            + first.x * second.w
            + first.y * second.z
            - first.z * second.y
        )
        orientation.y = (
            first.w * second.y
            - first.x * second.z
            + first.y * second.w
            + first.z * second.x
        )
        orientation.z = (
            first.w * second.z
            + first.x * second.y
            - first.y * second.x
            + first.z * second.w
        )
        return orientation

    def _build_move_group_goal(self, joint_positions):
        goal_msg = MoveGroup.Goal()
        request = goal_msg.request
        request.group_name = self._group_name
        request.num_planning_attempts = self.NUM_PLANNING_ATTEMPTS
        request.allowed_planning_time = self.ALLOWED_PLANNING_TIME
        request.max_velocity_scaling_factor = self.VELOCITY_SCALING
        request.max_acceleration_scaling_factor = self.ACCELERATION_SCALING
        request.start_state.is_diff = True

        request.workspace_parameters.header.frame_id = self.TARGET_FRAME
        request.workspace_parameters.min_corner.x = self.WORKSPACE_MIN[0]
        request.workspace_parameters.min_corner.y = self.WORKSPACE_MIN[1]
        request.workspace_parameters.min_corner.z = self.WORKSPACE_MIN[2]
        request.workspace_parameters.max_corner.x = self.WORKSPACE_MAX[0]
        request.workspace_parameters.max_corner.y = self.WORKSPACE_MAX[1]
        request.workspace_parameters.max_corner.z = self.WORKSPACE_MAX[2]

        constraints = Constraints()
        constraints.name = 'moveit_ik_joint_goal'
        constraints.joint_constraints = [
            self._joint_constraint(joint, position)
            for joint, position in zip(self.MOVEIT_JOINTS, joint_positions)
        ]
        path_constraints = Constraints()
        path_constraints.name = 'fixed_end_joints_path'
        path_constraints.joint_constraints = [
            self._joint_constraint(joint, 0.0) for joint in self.FIXED_END_JOINTS
        ]
        request.goal_constraints = [constraints]
        request.path_constraints = path_constraints

        options = goal_msg.planning_options
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 2
        options.replan_delay = 0.1
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True
        return goal_msg

    def _joint_constraint(self, joint_name, position):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = float(position)
        constraint.tolerance_above = self.GOAL_JOINT_TOLERANCE
        constraint.tolerance_below = self.GOAL_JOINT_TOLERANCE
        constraint.weight = 1.0
        return constraint

    def _extract_final_angles(self, result, fallback_angles):
        joint_trajectory = result.executed_trajectory.joint_trajectory
        if not joint_trajectory.points:
            joint_trajectory = result.planned_trajectory.joint_trajectory
        if not joint_trajectory.points:
            return self._fixed_angle_array(fallback_angles)

        positions = dict(
            zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions)
        )
        try:
            return [
                positions[joint] / DEG_TO_RAD
                for joint in self.MOVEIT_JOINTS[:5]
            ]
        except KeyError:
            return list(ZERO_ANGLES)

    def _wait_for_future(
        self, future, feedback_callback=None, progress=0.0, current_angles=None,
        timeout_sec=None,
    ):
        start_time = time.monotonic()
        feedback_time = 0.0
        while rclpy.ok() and not future.done():
            if timeout_sec is not None and time.monotonic() - start_time >= timeout_sec:
                future.cancel()
                return False

            if (
                feedback_callback is not None
                and time.monotonic() >= feedback_time
            ):
                feedback_callback(
                    progress, self._fixed_angle_array(current_angles)
                )
                feedback_time = time.monotonic() + 0.5
            time.sleep(0.02)
        return future.done()

    def _fixed_angle_array(self, angles):
        if angles is None or len(angles) != 5:
            return list(ZERO_ANGLES)
        return [float(angle) for angle in angles]
    
    def _format_angles(self, angles):
        return '[' + ', '.join(f'{angle:.2f}' for angle in angles) + ']'
