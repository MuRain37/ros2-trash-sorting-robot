import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('robot_description'),
        'world',
        'custom_room.world',
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file path',
    )

    declare_gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='false',
        description='Start Gazebo client GUI',
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Gazebo entity name',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py',
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': 'true',
            'gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            '/robot_description',
            '-entity',
            LaunchConfiguration('robot_name'),
        ],
    )

    return LaunchDescription([
        declare_world,
        declare_gazebo_gui,
        declare_robot_name,
        gazebo,
        spawn_entity,
    ])
