from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include_drive_launch_actions(context):
    use_drive = LaunchConfiguration('use_drive').perform(context).lower()
    if use_drive != 'true':
        return []

    ydlidar_pkg_path = get_package_share_directory('ydlidar_ros2_driver')
    imu_filter_params = (
        get_package_share_directory('imu_ros2_device')
        + '/config/imu_filter_param.yaml'
    )

    # Realsense 深度相机
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]),
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
        }.items(),
    )

    # IMU 驱动 + 滤波
    imu_device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_params],
    )

    # 点云转激光扫描
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/camera/depth/color/points'),
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

    # YDLidar 激光雷达驱动
    ydlidar_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=ydlidar_pkg_path + '/params/ydlidar.yaml',
        description='YDLidar参数文件路径')

    lidar_driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        namespace='/',
    )

    return [
        rs_launch,
        pointcloud_to_laserscan_node,
        imu_device_node,
        imu_filter_node,
        ydlidar_params_declare,
        lidar_driver_node,
    ]


def include_moveit_launch_actions(context, joint_state_broadcaster_spawner):
    arm_plan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'plan_controller',
            '--controller-manager', 'controller_manager',
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_moveit_config'),
                'launch', 'move_group.launch.py',
            ]),
        ]),
    )

    arm_controller = Node(
        package='robot_arm',
        executable='arm_moveit_target',
        name='arm_moveit_target_controller',
        output='screen',
        parameters=[{
            'action_name': '/arm_target',
            'group_name': 'plan',
            'move_action_name': '/move_action',
            'shutdown_after_startup_goal': False,
        }],
    )

    delay_arm_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_plan_controller_spawner],
        ),
    )

    return [
        delay_arm_controller_after_jsb,
        move_group,
        arm_controller,
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    moveit_pkg_path = get_package_share_directory('robot_moveit_config')
    default_model_path = pkg_path + '/urdf/robot.custom.xacro'
    robot_controllers = pkg_path + '/config/robot_custom_controllers.yaml'
    moveit_ros2_controllers = moveit_pkg_path + '/config/ros2_controllers.yaml'

    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF absolute path')

    declare_use_drive = DeclareLaunchArgument(
        name='use_drive',
        default_value='false',
        description='Whether to launch drive sensors and drivers')

    declare_open_loop_control = DeclareLaunchArgument(
        name='open_loop_control',
        default_value='false',
        description='Default open-loop feedback for motor and arm bridges')

    declare_motor_open_loop_control = DeclareLaunchArgument(
        name='motor_open_loop_control',
        default_value=LaunchConfiguration('open_loop_control'),
        description='Whether to echo wheel commands as feedback')

    declare_arm_open_loop_control = DeclareLaunchArgument(
        name='arm_open_loop_control',
        default_value=LaunchConfiguration('open_loop_control'),
        description='Whether to echo arm position commands as feedback')

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('model')])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            robot_controllers,
            moveit_ros2_controllers,
        ],
        remappings=[
            ('/robot_diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 'controller_manager',
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robot_diff_drive_controller',
            '--controller-manager', 'controller_manager',
        ],
    )

    delay_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    hardware_node = Node(
        package='robot_controller',
        executable='hardware_node',
        name='hardware_bridge',
        output='screen',
        parameters=[{
            'debug': True,
            'open_loop_control': ParameterValue(
                LaunchConfiguration('open_loop_control'),
                value_type=bool,
            ),
            'motor_open_loop_control': ParameterValue(
                LaunchConfiguration('motor_open_loop_control'),
                value_type=bool,
            ),
            'arm_open_loop_control': ParameterValue(
                LaunchConfiguration('arm_open_loop_control'),
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        declare_model,
        declare_use_drive,
        declare_open_loop_control,
        declare_motor_open_loop_control,
        declare_arm_open_loop_control,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_controller_after_jsb,
        OpaqueFunction(
            function=include_moveit_launch_actions,
            args=[joint_state_broadcaster_spawner],
        ),
        hardware_node,
        OpaqueFunction(function=include_drive_launch_actions),
    ])
