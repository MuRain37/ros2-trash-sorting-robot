"""Navigation task behavior for RobotTaskServer."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_interfaces.action import ArmPreset


PARAMETER_DEFAULTS = (
    # Nav2 单点导航 Action 名称。
    ('navigate_to_pose_action', 'navigate_to_pose'),
)


class NavigationTaskMixin:

    def _declare_navigation_parameters(self):
        for name, default in PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

    def _run_navigate_tree(self, goal_handle, goal):
        self._feedback(goal_handle, 'navigate_to: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        self._feedback(goal_handle, 'navigate_to: sending goal', 0.20)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = (
            goal.frame_id or str(self.get_parameter('map_frame').value)
        )
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(goal.x)
        nav_goal.pose.pose.position.y = float(goal.y)
        nav_goal.pose.pose.position.z = 0.0
        qz, qw = self._yaw_to_quaternion_z_w(float(goal.yaw))
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        result = self._send_action_goal(
            self._navigate_client,
            nav_goal,
            timeout_sec=self._navigation_timeout_sec(),
            goal_handle=goal_handle,
            label='navigate to pose',
        )
        if result is None:
            return False, 'navigate to pose failed'
        return True, 'navigate to pose completed'

    def _run_navigate_path_tree(self, goal_handle, goal):
        path = goal.path
        if not path.poses:
            return False, 'navigate path is empty'

        self._feedback(goal_handle, 'navigate_path: arm mapping pose', 0.05)
        self._send_arm_preset(goal_handle, ArmPreset.Goal.ARM_MAPPING)
        total = len(path.poses)
        timeout_sec = self._navigation_timeout_sec()
        for index, pose in enumerate(path.poses, start=1):
            if self._is_canceling(goal_handle):
                return False, 'navigate path canceled'

            progress = 0.10 + 0.85 * ((index - 1) / max(1, total))
            self._feedback(
                goal_handle,
                f'navigate_path: sending point {index}/{total}',
                progress,
            )
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = pose
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            if not nav_goal.pose.header.frame_id:
                nav_goal.pose.header.frame_id = (
                    goal.frame_id or str(self.get_parameter('map_frame').value)
                )

            wrapped = self._send_action_goal_wrapped(
                self._navigate_client,
                nav_goal,
                timeout_sec=timeout_sec,
                goal_handle=goal_handle,
                label=f'navigate path point {index}/{total}',
            )
            if wrapped is None:
                return False, f'navigate path point {index}/{total} failed'
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                return (
                    False,
                    f'navigate path point {index}/{total} failed: '
                    f'status={wrapped.status}',
                )

        return True, f'navigate path completed: points={total}'
