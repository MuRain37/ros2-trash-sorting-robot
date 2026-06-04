"""启动仿真传感器数据转换节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Gazebo 传感器辅助节点。"""
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/camera/points'),
            ('scan', '/scan_camera'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': -0.02,
            'max_height': 0.01,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.017453,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 8.0,
            'transform_tolerance': 0.1,
        }],
    )

    return LaunchDescription([pointcloud_to_laserscan])
