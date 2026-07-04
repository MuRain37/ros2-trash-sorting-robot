"""启动覆盖区域选择、路径生成、任务管理和 RViz 预览节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成覆盖导航辅助节点的 LaunchDescription。"""
    default_map_yaml = os.path.join(
        get_package_share_directory('robot_nav2'),
        'maps',
        'gazebo.yaml',
    )

    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')
    marker_topic = LaunchConfiguration('marker_topic')
    rectangle_topic = LaunchConfiguration('rectangle_topic')
    coverage_path_topic = LaunchConfiguration('coverage_path_topic')
    task_service = LaunchConfiguration('task_service')
    coverage_path_service = LaunchConfiguration('coverage_path_service')
    navigate_to_pose_action = LaunchConfiguration('navigate_to_pose_action')
    navigate_through_poses_action = LaunchConfiguration(
        'navigate_through_poses_action'
    )
    execute_coverage_action = LaunchConfiguration('execute_coverage_action')
    zigzag_spacing = LaunchConfiguration('zigzag_spacing')
    marker_line_width = LaunchConfiguration('marker_line_width')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    task_entry_margin = LaunchConfiguration('task_entry_margin')
    tf_timeout_sec = LaunchConfiguration('tf_timeout_sec')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_publish_period = LaunchConfiguration(
        'initial_pose_publish_period'
    )
    initial_pose_publish_attempts = LaunchConfiguration(
        'initial_pose_publish_attempts'
    )

    coverage_path_generator = Node(
        package='robot_nav2_behaviors',
        executable='coverage_path_generator',
        name='coverage_path_generator',
        output='screen',
        parameters=[
            {
                'frame_id': frame_id,
                'zigzag_spacing': zigzag_spacing,
                'coverage_path_service': coverage_path_service,
                'rectangle_topic': rectangle_topic,
                'coverage_path_topic': coverage_path_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    coverage_marker_publisher = Node(
        package='robot_nav2_behaviors',
        executable='coverage_marker_publisher',
        name='coverage_marker_publisher',
        output='screen',
        parameters=[
            {
                'marker_topic': marker_topic,
                'rectangle_topic': rectangle_topic,
                'coverage_path_topic': coverage_path_topic,
                'marker_line_width': marker_line_width,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    coverage_task_manager = Node(
        package='robot_nav2_behaviors',
        executable='coverage_task_manager',
        name='coverage_task_manager',
        output='screen',
        parameters=[
            {
                'frame_id': frame_id,
                'task_service': task_service,
                'coverage_path_service': coverage_path_service,
                'navigate_to_pose_action': navigate_to_pose_action,
                'execute_coverage_action': execute_coverage_action,
                'robot_base_frame': robot_base_frame,
                'task_entry_margin': task_entry_margin,
                'tf_timeout_sec': tf_timeout_sec,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    map_rectangle_selector = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_nav2_behaviors',
                executable='map_rectangle_selector',
                name='map_rectangle_selector',
                output='screen',
                parameters=[
                    {
                        'map_yaml': map_yaml,
                        'frame_id': frame_id,
                        'task_service': task_service,
                        'use_sim_time': use_sim_time,
                    }
                ],
            )
        ],
    )

    coverage_navigator = Node(
        package='robot_nav2_behaviors',
        executable='coverage_navigator',
        name='coverage_navigator',
        output='screen',
        parameters=[
            {
                'execute_action': execute_coverage_action,
                'navigate_action': navigate_through_poses_action,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    initial_pose_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_nav2_behaviors',
                executable='initial_pose_publisher',
                name='initial_pose_publisher',
                output='screen',
                parameters=[
                    {
                        'frame_id': frame_id,
                        'initial_pose_x': initial_pose_x,
                        'initial_pose_y': initial_pose_y,
                        'initial_pose_yaw': initial_pose_yaw,
                        'publish_period': initial_pose_publish_period,
                        'publish_attempts': initial_pose_publish_attempts,
                        'use_sim_time': use_sim_time,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
        ),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('zigzag_spacing', default_value='1.0'),
        DeclareLaunchArgument('marker_line_width', default_value='0.02'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('task_entry_margin', default_value='0.0'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_pose_publish_period',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'initial_pose_publish_attempts',
            default_value='10',
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='center_square_marker',
        ),
        DeclareLaunchArgument(
            'rectangle_topic',
            default_value='coverage_rectangle',
        ),
        DeclareLaunchArgument(
            'coverage_path_topic',
            default_value='coverage_path_preview',
        ),
        DeclareLaunchArgument(
            'task_service',
            default_value='add_coverage_rectangle',
        ),
        DeclareLaunchArgument(
            'coverage_path_service',
            default_value='generate_coverage_path',
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_action',
            default_value='navigate_to_pose',
        ),
        DeclareLaunchArgument(
            'navigate_through_poses_action',
            default_value='navigate_through_poses',
        ),
        DeclareLaunchArgument(
            'execute_coverage_action',
            default_value='execute_coverage_path',
        ),
        coverage_path_generator,
        coverage_marker_publisher,
        initial_pose_publisher,
        coverage_task_manager,
        map_rectangle_selector,
        coverage_navigator,
    ])
