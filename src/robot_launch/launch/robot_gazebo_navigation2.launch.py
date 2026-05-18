import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')

    robot_gazebo_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_launch'),
            '/launch/robot_gazebo.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_navgation2'),
            '/launch/navigation2.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation clock for Gazebo and Nav2'),
        robot_gazebo_launch,
        navigation2_launch,
    ])
