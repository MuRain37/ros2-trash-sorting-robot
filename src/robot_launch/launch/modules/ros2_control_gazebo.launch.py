"""启动 Gazebo 模式的 ros2_control 控制器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gazebo_controller_spawners():
    """创建 Gazebo 控制器 spawner 节点。"""
    controller_config = os.path.join(
        get_package_share_directory('robot_description'),
        'config',
        'robot_gazebo_controllers.yaml',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c',
            '/controller_manager',
            '--param-file',
            controller_config,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    plan_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'plan_controller',
            '-c',
            '/controller_manager',
            '--param-file',
            controller_config,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    start_plan_after_joint_states = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[plan_controller],
        ),
    )

    return joint_state_broadcaster, plan_controller, start_plan_after_joint_states


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    joint_state_broadcaster, _plan_controller, start_plan = gazebo_controller_spawners()

    return LaunchDescription([
        declare_use_sim_time,
        joint_state_broadcaster,
        start_plan,
    ])
