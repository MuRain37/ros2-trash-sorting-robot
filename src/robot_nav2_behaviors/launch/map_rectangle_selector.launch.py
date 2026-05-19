from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    frame_id = LaunchConfiguration('frame_id')
    marker_topic = LaunchConfiguration('marker_topic')
    rectangle_service = LaunchConfiguration('rectangle_service')
    coverage_path_topic = LaunchConfiguration('coverage_path_topic')
    navigate_action = LaunchConfiguration('navigate_action')
    zigzag_spacing = LaunchConfiguration('zigzag_spacing')
    marker_line_width = LaunchConfiguration('marker_line_width')

    center_square_marker = Node(
        package='robot_nav2_behaviors',
        executable='center_square_marker',
        name='center_square_marker',
        output='screen',
        remappings=[
            ('center_square_marker', marker_topic),
        ],
        parameters=[
            {
                'frame_id': frame_id,
                'zigzag_spacing': zigzag_spacing,
                'marker_line_width': marker_line_width,
                'rectangle_service': rectangle_service,
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
                        'rectangle_service': rectangle_service,
                        'coverage_path_topic': coverage_path_topic,
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
                'coverage_path_topic': coverage_path_topic,
                'navigate_action': navigate_action,
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
            'rectangle_service',
            default_value='set_center_square_marker',
        ),
        DeclareLaunchArgument(
            'coverage_path_topic',
            default_value='coverage_path',
        ),
        DeclareLaunchArgument(
            'navigate_action',
            default_value='navigate_through_poses',
        ),
        center_square_marker,
        map_rectangle_selector,
        coverage_navigator,
    ])
