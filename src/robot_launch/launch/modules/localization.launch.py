"""启动机器人定位节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成定位相关启动项。"""
    robot_launch_pkg = get_package_share_directory('robot_launch')
    ekf_params = os.path.join(robot_launch_pkg, 'config', 'ekf.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for localization',
    )

    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_filter,
    ])
