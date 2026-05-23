import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def custom_control_actions():
    robot_description_pkg = get_package_share_directory('robot_description')
    moveit_pkg = get_package_share_directory('robot_moveit_config')

    robot_controllers = os.path.join(
        robot_description_pkg,
        'config',
        'robot_custom_controllers.yaml',
    )
    moveit_controllers = os.path.join(
        moveit_pkg,
        'config',
        'ros2_controllers.yaml',
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            robot_controllers,
            moveit_controllers,
        ],
        remappings=[
            ('/robot_diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
        output='both',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            'controller_manager',
        ],
    )

    robot_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robot_diff_drive_controller',
            '--controller-manager',
            'controller_manager',
        ],
    )

    plan_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'plan_controller',
            '--controller-manager',
            'controller_manager',
        ],
    )

    hardware_node = Node(
        package='robot_controller',
        executable='hardware_node',
        name='hardware_bridge',
        output='screen',
        parameters=[{
            'debug': True,
            'open_loop_control': ParameterValue(
                LaunchConfiguration('open_loop_control'),
                value_type=bool,
            ),
            'motor_open_loop_control': ParameterValue(
                LaunchConfiguration('motor_open_loop_control'),
                value_type=bool,
            ),
            'arm_open_loop_control': ParameterValue(
                LaunchConfiguration('arm_open_loop_control'),
                value_type=bool,
            ),
        }],
    )

    start_drive_and_plan = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot_diff_drive_controller, plan_controller],
        ),
    )

    return (
        control_node,
        joint_state_broadcaster,
        plan_controller,
        start_drive_and_plan,
        hardware_node,
    )


def generate_launch_description():
    declare_model = DeclareLaunchArgument(
        'model',
        description='Robot xacro file path',
    )

    declare_open_loop_control = DeclareLaunchArgument(
        'open_loop_control',
        default_value='false',
        description='Default open-loop feedback for motor and arm bridges',
    )

    declare_motor_open_loop_control = DeclareLaunchArgument(
        'motor_open_loop_control',
        default_value=LaunchConfiguration('open_loop_control'),
        description='Whether to echo wheel commands as feedback',
    )

    declare_arm_open_loop_control = DeclareLaunchArgument(
        'arm_open_loop_control',
        default_value=LaunchConfiguration('open_loop_control'),
        description='Whether to echo arm commands as feedback',
    )

    control_node, joint_state_broadcaster, _plan_controller, start_controllers, hardware_node = (
        custom_control_actions()
    )

    return LaunchDescription([
        declare_model,
        declare_open_loop_control,
        declare_motor_open_loop_control,
        declare_arm_open_loop_control,
        control_node,
        joint_state_broadcaster,
        start_controllers,
        hardware_node,
    ])
