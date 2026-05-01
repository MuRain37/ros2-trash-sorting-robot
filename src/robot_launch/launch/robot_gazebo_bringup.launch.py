from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_description'),
            '/launch/robot_gazebo.launch.py',
        ]),
    )


    arm_controller = Node(
        package='robot_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_joint_states': False,    # 由 arm_joint_state_broadcaster 发布
            'publish_forward_command': True,   # 通过 ForwardCommandController 驱动 Gazebo 关节
        }],
    )
    return LaunchDescription([
        gazebo_launch,
        arm_controller,
    ])
