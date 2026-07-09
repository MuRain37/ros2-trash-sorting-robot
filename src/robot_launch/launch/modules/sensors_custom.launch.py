"""启动真实硬件传感器驱动。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


YDLIDAR_POINT_COUNT_FILTER_PREFIX = (
    'bash -c \'"$@" 2>&1 | grep -Ev '
    '"Real point count [0-9]+ > fixed point count [0-9]+"; '
    'exit ${PIPESTATUS[0]}\' --'
)


def generate_launch_description():
    """生成真实传感器启动项。"""
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    imu_filter_params = (
        get_package_share_directory('imu_ros2_device')
        + '/config/imu_filter_param.yaml'
    )
    declare_ydlidar_params_file = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=ydlidar_pkg + '/params/ydlidar.yaml',
        description='YDLidar parameter file path',
    )

    real_camera_static_tf = Node(
        package='robot_manipulation',
        executable='real_camera_static_tf',
        name='real_camera_static_tf',
        output='screen',
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ]),
        ]),
        launch_arguments={
            'camera_name': 'real_camera',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'align_depth.enable': 'true',
        }.items(),
    )

    imu_device = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_params],
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/real_camera/depth/color/points'),
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

    scan_camera_gate = Node(
        package='robot_nav2_behaviors',
        executable='scan_camera_gate',
        name='scan_camera_gate',
        output='screen',
        parameters=[{
            'input_topic': '/scan_camera',
            'output_topic': '/scan_camera_costmap',
            'set_enabled_service': '/set_scan_camera_costmap_enabled',
            'enabled': True,
        }],
    )

    lidar = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        prefix=YDLIDAR_POINT_COUNT_FILTER_PREFIX,
        parameters=[LaunchConfiguration('ydlidar_params_file')],
        namespace='/',
    )

    return LaunchDescription([
        declare_ydlidar_params_file,
        real_camera_static_tf,
        realsense,
        pointcloud_to_laserscan,
        scan_camera_gate,
        imu_device,
        imu_filter,
        lidar,
    ])
"""启动真实硬件传感器驱动。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


YDLIDAR_POINT_COUNT_FILTER_PREFIX = (
    'bash -c \'"$@" 2>&1 | grep -Ev '
    '"Real point count [0-9]+ > fixed point count [0-9]+"; '
    'exit ${PIPESTATUS[0]}\' --'
)


def generate_launch_description():
    """生成真实传感器启动项。"""
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    imu_filter_params = (
        get_package_share_directory('imu_ros2_device')
        + '/config/imu_filter_param.yaml'
    )
    declare_ydlidar_params_file = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=ydlidar_pkg + '/params/ydlidar.yaml',
        description='YDLidar parameter file path',
    )

    real_camera_static_tf = Node(
        package='robot_manipulation',
        executable='real_camera_static_tf',
        name='real_camera_static_tf',
        output='screen',
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ]),
        ]),
        launch_arguments={
            'camera_name': 'real_camera',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'align_depth.enable': 'true',
        }.items(),
    )

    imu_device = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_params],
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/real_camera/depth/color/points'),
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

    scan_camera_gate = Node(
        package='robot_nav2_behaviors',
        executable='scan_camera_gate',
        name='scan_camera_gate',
        output='screen',
        parameters=[{
            'input_topic': '/scan_camera',
            'output_topic': '/scan_camera_costmap',
            'set_enabled_service': '/set_scan_camera_costmap_enabled',
            'enabled': True,
        }],
    )

    lidar = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        prefix=YDLIDAR_POINT_COUNT_FILTER_PREFIX,
        parameters=[LaunchConfiguration('ydlidar_params_file')],
        namespace='/',
    )

    return LaunchDescription([
        declare_ydlidar_params_file,
        real_camera_static_tf,
        realsense,
        pointcloud_to_laserscan,
        scan_camera_gate,
        imu_device,
        imu_filter,
        lidar,
    ])
