"""启动 robot_state_publisher 并发布由 xacro 生成的机器人描述。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成机器人描述发布节点。"""
    declare_model = DeclareLaunchArgument(
        'model',
        description='Robot xacro file path',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        declare_model,
        declare_use_sim_time,
        robot_state_publisher,
    ])
