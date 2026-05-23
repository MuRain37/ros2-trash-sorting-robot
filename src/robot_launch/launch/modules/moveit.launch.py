from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_moveit_config'),
            '/launch/move_group.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    arm_target = Node(
        package='robot_arm',
        executable='arm_moveit_target',
        name='arm_moveit_target_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'action_name': '/arm_target',
            'group_name': 'plan',
            'move_action_name': '/move_action',
            'shutdown_after_startup_goal': False,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group,
        arm_target,
    ])
