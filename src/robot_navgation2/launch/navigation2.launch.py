"""Nav2 启动入口：合并拆分参数文件并启动 bringup 与 RViz。"""

import os
import tempfile

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


NAV2_PARAM_FILES = [
    'amcl.yaml',
    'bt_navigator.yaml',
    'controller_server.yaml',
    'costmaps.yaml',
    'map_server.yaml',
    'planner_server.yaml',
    'behavior_server.yaml',
    'robot_state_publisher.yaml',
    'waypoint_follower.yaml',
    'velocity_smoother.yaml',
]


def _merged_nav2_params(context):
    """返回单个 Nav2 参数文件；未指定时合并 config/nav2_params。"""
    params_file = launch.substitutions.LaunchConfiguration(
        'params_file').perform(context)
    if params_file:
        return params_file

    params_dir = launch.substitutions.LaunchConfiguration(
        'params_dir').perform(context)
    merged_params = {}
    for filename in NAV2_PARAM_FILES:
        path = os.path.join(params_dir, filename)
        with open(path, 'r') as param_file:
            data = yaml.safe_load(param_file) or {}
        duplicate_nodes = set(merged_params).intersection(data)
        if duplicate_nodes:
            duplicates = ', '.join(sorted(duplicate_nodes))
            raise RuntimeError(
                f'Duplicate Nav2 parameter sections in {path}: {duplicates}')
        merged_params.update(data)

    output = tempfile.NamedTemporaryFile(
        mode='w', prefix='robot_navgation2_nav2_', suffix='.yaml', delete=False)
    with output:
        yaml.safe_dump(merged_params, output, default_flow_style=False, sort_keys=False)
    return output.name


def _include_nav2_bringup(context, nav2_bringup_dir, map_yaml_path, use_sim_time):
    """包含 nav2_bringup 的 bringup_launch.py。"""
    nav2_param_path = _merged_nav2_params(context)
    return [
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    ]


def generate_launch_description():
    """生成 Nav2 bringup 与 RViz 启动项。"""
    robot_navgation2_dir = get_package_share_directory(
        'robot_navgation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            robot_navgation2_dir)))),
        'src', 'robot_navgation2', 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(robot_navgation2_dir, 'maps', 'room.yaml'))
    nav2_param_dir = launch.substitutions.LaunchConfiguration(
        'params_dir', default=os.path.join(robot_navgation2_dir, 'config', 'nav2_params'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default='')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_dir', default_value=nav2_param_dir,
                                             description='Directory containing split Nav2 param files'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to a single Nav2 param file to load. Overrides params_dir when set.'),

        launch.actions.OpaqueFunction(
            function=_include_nav2_bringup,
            args=[nav2_bringup_dir, map_yaml_path, use_sim_time],
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
