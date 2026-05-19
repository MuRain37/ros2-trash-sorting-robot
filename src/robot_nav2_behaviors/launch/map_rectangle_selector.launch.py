from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    frame_id = LaunchConfiguration('frame_id')
    marker_topic = LaunchConfiguration('marker_topic')
    rectangle_topic = LaunchConfiguration('rectangle_topic')
    coverage_path_topic = LaunchConfiguration('coverage_path_topic')
    task_service = LaunchConfiguration('task_service')
    rectangle_service = LaunchConfiguration('rectangle_service')
    navigate_to_pose_action = LaunchConfiguration('navigate_to_pose_action')
    navigate_through_poses_action = LaunchConfiguration(
        'navigate_through_poses_action'
    )
    execute_coverage_action = LaunchConfiguration('execute_coverage_action')
    zigzag_spacing = LaunchConfiguration('zigzag_spacing')
    marker_line_width = LaunchConfiguration('marker_line_width')

    coverage_path_generator = Node(
        package='robot_nav2_behaviors',
        executable='coverage_path_generator',
        name='coverage_path_generator',
        output='screen',
        parameters=[
            {
                'frame_id': frame_id,
                'zigzag_spacing': zigzag_spacing,
                'rectangle_service': rectangle_service,
                'rectangle_topic': rectangle_topic,
                'coverage_path_topic': coverage_path_topic,
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
                'rectangle_service': rectangle_service,
                'navigate_to_pose_action': navigate_to_pose_action,
                'execute_coverage_action': execute_coverage_action,
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
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=(
                '/home/murain/WorkSpace/robot/'
                'src/robot_navgation2/maps/room.yaml'
            ),
        ),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('zigzag_spacing', default_value='1.0'),
        DeclareLaunchArgument('marker_line_width', default_value='0.02'),
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
            'rectangle_service',
            default_value='set_center_square_marker',
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
        coverage_task_manager,
        map_rectangle_selector,
        coverage_navigator,
    ])
