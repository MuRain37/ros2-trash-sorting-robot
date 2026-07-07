"""启动机器人高层行为树调度节点。"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成行为树调度 launch。"""
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros_src_root = os.getenv('ROS_SRC_ROOT', '/ws/ros2_xjw/src')
    dump_pose_store_path = os.getenv(
        'ROS_DUMP_POSE_STORE_PATH',
        os.path.join(
            ros_src_root,
            'robot_behavior_tree',
            'config',
            'dump_bucket_poses.json',
        ),
    )
    dump_model_path = os.path.join(
        ros_src_root,
        'robot_vision_grasp',
        'robot_vision_grasp',
        'models',
        'trash_bin.pt',
    )

    robot_task_server = Node(
        package='robot_behavior_tree',
        executable='robot_task_server',
        name='robot_task_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'detect_dump_service': '/detect_dump_bucket',
            'service_timeout_sec': 8.0,
            'require_dump_bucket_detect': True,
            'dump_visual_align_required': False,
            'dump_pose_store_path': dump_pose_store_path,
        }],
    )

    dump_task = Node(
        package='robot_vision_grasp',
        executable='dump_task',
        name='dump_task',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'service_name': '/detect_dump_bucket',
            'model_path': dump_model_path,
            'color_topic': '/camera/real_camera/color/image_raw',
            'depth_topic': '/camera/real_camera/aligned_depth_to_color/image_raw',
            'class_name': 'bin',
            'confidence_threshold': 0.75,
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
