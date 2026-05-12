from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    default_model_path = pkg_path + '/urdf/robot.custom.xacro'
    robot_controllers = pkg_path + '/config/robot_custom_controllers.yaml'
    ydlidar_pkg_path = get_package_share_directory('ydlidar_ros2_driver')

    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF absolute path')

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('model')])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            robot_controllers,
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
            'robot_joint_state_broadcaster',
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
        parameters=[{'debug': True}],
    )

    arm_controller = Node(
        package='robot_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
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
        parameters=[get_package_share_directory('imu_ros2_device') + '/config/imu_filter_param.yaml'],
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

    return LaunchDescription([
        declare_model,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_controller_after_jsb,
        hardware_node,
        arm_controller,
        rs_launch,
        pointcloud_to_laserscan_node,
        imu_device_node,
        imu_filter_node,
        ydlidar_params_declare,
        lidar_driver_node,
    ])
