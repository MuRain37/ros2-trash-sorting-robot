from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    custom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_description'),
            '/launch/robot_custom.launch.py',
        ]),
    )

    chassis_controller = Node(
        package='robot_controller',
        executable='chassis_controller',
        name='chassis_controller',
        output='screen',
    )

    return LaunchDescription([
        custom_launch,
        chassis_controller,
    ])
