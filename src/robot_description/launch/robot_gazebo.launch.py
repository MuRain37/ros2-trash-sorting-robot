import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "robot"
    pkg_path = get_package_share_directory('robot_description')
    default_model_path = pkg_path + '/urdf/robot.gazebo.xacro'
    default_world_path = pkg_path + '/world/custom_room.world'
    default_rviz_config_path = pkg_path + '/config/rviz/display_model.rviz'
    controller_config_path = os.path.join(
        pkg_path, 'config', 'robot_gazebo_controllers.yaml')

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')

    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    
    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='whether to use sim_time')
  	
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
        }]
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch/gazebo.launch.py']),
      	# 传递参数
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    # 深度图像转激光扫描节点
    depthimage_to_laserscan_node = launch_ros.actions.Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan_camera'),
        ],
        parameters=[{
            'scan_height': 1,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 8.0,
            'output_frame': 'base_link',
        }]
    )

    # 机械臂 ros2_control spawners
    # controller_manager 由 gazebo_ros2_control 插件内部创建
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

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        arm_forward_spawner,
        arm_joint_state_spawner,
        rviz_node,
        depthimage_to_laserscan_node,
    ])
