import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name_in_model = "robot"
    pkg_path = get_package_share_directory('robot_description')
    default_model_path = pkg_path + '/urdf/robot.gazebo.xacro'
    default_world_path = pkg_path + '/world/custom_room.world'
    default_rviz_config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            get_package_share_directory('robot_launch'))))),
        'src', 'robot_launch', 'config', 'gazebo.rviz')
    controller_config_path = os.path.join(
        pkg_path, 'config', 'robot_gazebo_controllers.yaml')

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF absolute path')

    declare_rviz = launch.actions.DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='RViz config file path')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='whether to use sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
        }]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments=[
            ('world', default_world_path),
            ('headless', 'true'),
        ]
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
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
        }]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', launch.substitutions.LaunchConfiguration('rviz_config')],
    )

    arm_forward_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_forward_controller',
            '-c', '/controller_manager',
            '--param-file', controller_config_path,
        ],
    )

    arm_joint_state_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_joint_state_broadcaster',
            '-c', '/controller_manager',
            '--param-file', controller_config_path,
        ],
    )

    arm_controller = launch_ros.actions.Node(
        package='robot_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_joint_states': False,
            'publish_forward_command': True,
        }],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        declare_rviz,
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        arm_forward_spawner,
        arm_joint_state_spawner,
        arm_controller,
        pointcloud_to_laserscan_node,
        rviz_node,
    ])
