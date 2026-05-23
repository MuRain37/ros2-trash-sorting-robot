from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock for Nav2',
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_navgation2'),
            '/launch/navigation2.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rectangle_selector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_nav2_behaviors'),
            '/launch/map_rectangle_selector.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        navigation2,
        rectangle_selector,
    ])
