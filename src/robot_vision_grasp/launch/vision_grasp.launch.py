"""Launch the vision grasp node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/camera/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_info_topic',
            default_value='/camera/camera/aligned_depth_to_color/camera_info',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_optical_link',
        ),
        DeclareLaunchArgument('base_frame', default_value='arm_link0'),
        DeclareLaunchArgument('return_home', default_value='true'),
        Node(
            package='robot_vision_grasp',
            executable='vision_grasp_node',
            name='vision_grasp_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_info_topic': LaunchConfiguration('depth_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'return_home': ParameterValue(
                    LaunchConfiguration('return_home'),
                    value_type=bool,
                ),
            }],
        ),
    ])
