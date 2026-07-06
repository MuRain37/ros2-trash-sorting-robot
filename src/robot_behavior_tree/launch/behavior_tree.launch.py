"""启动机器人高层行为树调度节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成行为树调度 launch。"""
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_task_server = Node(
        package='robot_behavior_tree',
        executable='robot_task_server',
        name='robot_task_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    dump_task = Node(
        package='robot_manipulation',
        executable='dump_task',
        name='dump_task',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'color_topic': '/camera/real_camera/color/image_raw',
            'depth_topic': '/camera/real_camera/aligned_depth_to_color/image_raw',
        }],
    )

    dump_trash_node = Node(
        package='robot_manipulation',
        executable='dump_trash_node',
        name='dump_trash_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_task_server,
        dump_task,
        dump_trash_node,
    ])
