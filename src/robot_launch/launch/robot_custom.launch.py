from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    default_model_path = pkg_path + '/urdf/robot.custom.xacro'
    robot_controllers = pkg_path + '/config/robot_custom_controllers.yaml'

    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF absolute path')

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('model')])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            robot_controllers,
        ],
        remappings=[
            ('/robot_diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robot_joint_state_broadcaster',
            '--controller-manager', 'controller_manager',
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robot_diff_drive_controller',
            '--controller-manager', 'controller_manager',
        ],
    )

    delay_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    hardware_node = Node(
        package='robot_controller',
        executable='hardware_node',
        name='hardware_bridge',
        output='screen',
        parameters=[{'debug': True}],
    )

    arm_controller = Node(
        package='robot_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
    )

    return LaunchDescription([
        declare_model,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_controller_after_jsb,
        hardware_node,
        arm_controller,
    ])
