import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_arm.kinematics import solve
from robot_interfaces.action import ArmTarget
from std_msgs.msg import Float64MultiArray

DEG_TO_RAD = math.pi / 180.0


class ArmController(Node):
    """机械臂 Action 服务器：接收目标位置，IK 求解，平滑插值更新关节。"""

    JOINTS = [
        'arm_base_joint',
        'arm_shoulder_joint',
        'arm_elbow_joint',
        'arm_wrist_joint',
    ]

    def __init__(self):
        super().__init__('arm_controller')

        self.declare_parameter('rate', 50.0)
        self.declare_parameter('initial_pose', [0, -25, 80, 80])
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('publish_forward_command', False)

        rate = self.get_parameter('rate').value
        self._publish_js = self.get_parameter('publish_joint_states').value
        self._publish_fwd = self.get_parameter('publish_forward_command').value

        self._current = list(self.get_parameter('initial_pose').value)

        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._cmd_pub = self.create_publisher(Float64MultiArray, '/arm_forward_controller/commands', 10)

        self._action_server = ActionServer(
            self,
            ArmTarget,
            '/arm_target',
            execute_callback=self._execute,
            goal_callback=self._handle_goal,
            callback_group=ReentrantCallbackGroup(),
        )

        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'Arm controller ready ({rate}Hz)')

    def _handle_goal(self, goal_request):
        self.get_logger().info(
            f'Target: [{goal_request.x:.1f}, {goal_request.y:.1f}, '
            f'{goal_request.z:.1f}] in {goal_request.duration:.1f}s'
        )
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request

        end_angle = None if goal.end_angle < -0.5 else goal.end_angle
        result = solve([goal.x, goal.y, goal.z], angle=end_angle,
                       verify=False, plot=False, collision_check=True)

        if result is False:
            goal_handle.abort()
            r = ArmTarget.Result()
            r.success = False
            self.get_logger().warning('Target unreachable')
            return r

        theta1, theta2, theta3, theta4, _ = result
        target = [theta1, theta2, theta3, theta4]
        start = list(self._current)
        start_time = time.time()

        feedback = ArmTarget.Feedback()

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= goal.duration:
                self._current = list(target)
                break

            t = elapsed / goal.duration
            self._current = [
                s + (e - s) * t for s, e in zip(start, target)
            ]

            feedback.progress = t
            feedback.current_angles = self._current
            goal_handle.publish_feedback(feedback)

            time.sleep(0.01)

        r = ArmTarget.Result()
        r.success = True
        r.final_angles = self._current
        self.get_logger().info('Trajectory complete')
        return r

    def _publish(self):
        angles_rad = [a * DEG_TO_RAD for a in self._current]

        if self._publish_js:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.JOINTS
            msg.position = angles_rad
            self._joint_pub.publish(msg)

        if self._publish_fwd:
            cmd = Float64MultiArray()
            cmd.data = angles_rad
            self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
