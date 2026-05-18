import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_launch')
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share))))
    default_rviz = os.path.join(workspace, 'src', 'robot_launch', 'rviz', 'custom.rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz config file path')

    custom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_launch'),
            '/launch/robot_custom.launch.py',
        ]),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        declare_rviz,
        custom_launch,
        rviz,
    ])
