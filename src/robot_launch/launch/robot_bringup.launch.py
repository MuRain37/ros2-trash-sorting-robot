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

    hardware_node = Node(
        package='robot_controller',
        executable='hardware_node',
        name='hardware_bridge',
        output='screen',
    )

    arm_joint_publisher = Node(
        package='robot_arm',
        executable='arm_joint_publisher',
        name='arm_joint_publisher',
        output='screen',
    )

    return LaunchDescription([
        custom_launch,
        hardware_node,
        arm_joint_publisher,
    ])
