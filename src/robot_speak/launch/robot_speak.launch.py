"""Launch the WonderEcho voice serial bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for robot_speak."""
    package_share = get_package_share_directory('robot_speak')
    default_commands = os.path.join(
        package_share,
        'config',
        'default_commands.yaml',
    )
    default_audio = os.path.join(package_share, 'audio')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/myvoice',
            description='Serial port connected to the WonderEcho module.',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='WonderEcho serial baudrate.',
        ),
        DeclareLaunchArgument(
            'commands_config',
            default_value=default_commands,
            description='YAML file mapping frames to command names.',
        ),
        DeclareLaunchArgument(
            'speak_on_recognition',
            default_value='true',
            description='Speak the response via TTS (host-side) when a command is recognized.',
        ),
        DeclareLaunchArgument(
            'speak_delay_sec',
            default_value='0.4',
            description='Delay before sending serial frames (legacy, not used for TTS).',
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Print serial RX/TX frames.',
        ),
        DeclareLaunchArgument(
            'tts_enabled',
            default_value='true',
            description='Enable voice responses (audio files or TTS).',
        ),
        DeclareLaunchArgument(
            'tts_command',
            default_value='',
            description='Fallback TTS command if no audio file. Use {text} as placeholder.',
        ),
        DeclareLaunchArgument(
            'audio_dir',
            default_value=default_audio,
            description='Directory with pre-recorded audio files (named {command}.mp3).',
        ),
        DeclareLaunchArgument(
            'audio_play_cmd',
            default_value="mpg123 -q -a plughw:1,0 '{path}'",
            description='Shell command to play audio files. Use {path} as placeholder.',
        ),
        Node(
            package='robot_speak',
            executable='voice_serial_node',
            name='voice_serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'commands_config': LaunchConfiguration('commands_config'),
                'speak_on_recognition': LaunchConfiguration(
                    'speak_on_recognition'
                ),
                'speak_delay_sec': LaunchConfiguration('speak_delay_sec'),
                'debug': LaunchConfiguration('debug'),
                'tts_enabled': LaunchConfiguration('tts_enabled'),
                'tts_command': LaunchConfiguration('tts_command'),
                'audio_dir': LaunchConfiguration('audio_dir'),
                'audio_play_cmd': LaunchConfiguration('audio_play_cmd'),
            }],
        ),
    ])
