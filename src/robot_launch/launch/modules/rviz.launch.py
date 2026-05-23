from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_rviz(context):
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_moveit = _as_bool(LaunchConfiguration('use_moveit').perform(context))

    if use_moveit:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('robot_moveit_config'),
                    '/launch/moveit_rviz.launch.py',
                ]),
                launch_arguments={
                    'rviz_config': rviz_config,
                }.items(),
            ),
        ]

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        description='RViz config file path',
    )

    declare_use_moveit = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Load MoveIt parameters into RViz',
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_use_moveit,
        OpaqueFunction(function=launch_rviz),
    ])
