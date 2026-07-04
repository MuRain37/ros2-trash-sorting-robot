"""启动 Nav2 和覆盖导航辅助节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成导航相关启动项。"""
    default_map_yaml = os.path.join(
        get_package_share_directory('robot_nav2'),
        'maps',
        'gazebo.yaml',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock for Nav2',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to the map YAML file for Nav2',
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_nav2'),
            '/launch/navigation2.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    rectangle_selector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_nav2_behaviors'),
            '/launch/map_rectangle_selector.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_yaml': LaunchConfiguration('map'),
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        navigation2,
        rectangle_selector,
    ])
