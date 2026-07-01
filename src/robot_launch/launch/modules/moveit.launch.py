"""启动 MoveIt move_group、RViz 和机械臂目标 Action 节点。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 MoveIt 相关启动项。"""
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
        }],
    )

    arm_preset = Node(
        package='robot_manipulation',
        executable='arm_preset_action',
        name='arm_preset_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'action_name': '/arm_preset',
            'group_name': 'plan',
            'move_action_name': '/move_action',
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group,
        arm_target,
        arm_preset,
    ])