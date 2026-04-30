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
    )
    return LaunchDescription([
        gazebo_launch,
        arm_controller,
    ])
