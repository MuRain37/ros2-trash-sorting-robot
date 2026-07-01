"""机器人总启动入口：按模式组合描述、控制、传感器、Nav2、MoveIt 和 RViz。"""

import os
import importlib.util

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
from launch.substitutions import LaunchConfiguration


def _as_bool(value):
    """把 Launch 字符串参数解析为布尔值。"""
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _module_source(name):
    """返回 robot_launch/modules 下指定 launch 模块的来源。"""
    return PythonLaunchDescriptionSource([
        get_package_share_directory('robot_launch'),
        '/launch/modules/',
        name,
        '.launch.py',
    ])


def _load_module(name):
    """直接加载 launch 模块以复用其中的动作工厂函数。"""
    module_path = os.path.join(
        get_package_share_directory('robot_launch'),
        'launch',
        'modules',
        name + '.launch.py',
    )
    spec = importlib.util.spec_from_file_location(name + '_launch', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _include_module(name, launch_arguments=None):
    """按名称包含 robot_launch/modules 下的 launch 文件。"""
    return IncludeLaunchDescription(
        _module_source(name),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _resolve_auto(value, default):
    """把 auto 参数替换为当前模式的默认值。"""
    return default if value == 'auto' else value


def _source_rviz_path(robot_launch_pkg, filename):
    """优先使用源码目录中的 RViz 配置，安装后回退到 share 目录。"""
    workspace = os.path.dirname(
        os.path.dirname(
            os.path.dirname(
                os.path.dirname(robot_launch_pkg)
            )
        )
    )
    source_path = os.path.join(
        workspace,
        'src',
        'robot_launch',
        'rviz',
        filename,
    )
    if os.path.exists(source_path):
        return source_path

    return os.path.join(robot_launch_pkg, 'rviz', filename)


def launch_setup(context):
    """根据 mode 和功能开关组装实际启动动作。"""
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('gazebo', 'custom'):
        raise RuntimeError("mode must be 'gazebo' or 'custom'")

    robot_description_pkg = get_package_share_directory('robot_description')
    robot_launch_pkg = get_package_share_directory('robot_launch')

    default_model = os.path.join(
        robot_description_pkg,
        'urdf',
        'robot.gazebo.xacro' if mode == 'gazebo' else 'robot.custom.xacro',
    )
    default_rviz = _source_rviz_path(
        robot_launch_pkg,
        'gazebo.rviz' if mode == 'gazebo' else 'custom.rviz',
    )

    use_sim_time = _resolve_auto(
        LaunchConfiguration('use_sim_time').perform(context),
        'true' if mode == 'gazebo' else 'false',
    )
    model = default_model
    rviz_config = default_rviz
    use_moveit = _as_bool(LaunchConfiguration('use_moveit').perform(context))

    actions = [
        _include_module(
            'description',
            {
                'model': model,
                'use_sim_time': use_sim_time,
            },
        ),
    ]
    plan_controller = None

    if mode == 'gazebo':
        gazebo_control = _load_module('ros2_control_gazebo')
        joint_state_broadcaster, plan_controller, start_plan = (
            gazebo_control.gazebo_controller_spawners()
        )
        actions.extend([
            _include_module(
                'gazebo',
                {
                    'world': os.path.join(
                        robot_description_pkg,
                        'world',
                        'custom_room.world',
                    ),
                    'gazebo_gui': LaunchConfiguration('gazebo_gui').perform(context),
                    'robot_name': 'robot',
                },
            ),
            joint_state_broadcaster,
            start_plan,
            _include_module('sensors_gazebo'),
        ])
    else:
        custom_control = _load_module('ros2_control_custom')
        (
            control_node,
            joint_state_broadcaster,
            plan_controller,
            start_controllers,
            hardware_node,
        ) = custom_control.custom_control_actions()
        actions.extend([
            control_node,
            joint_state_broadcaster,
            start_controllers,
            hardware_node,
        ])
        if _as_bool(LaunchConfiguration('use_drive').perform(context)):
            actions.append(_include_module('sensors_custom'))

    if use_moveit:
        moveit = _include_module(
            'moveit',
            {'use_sim_time': use_sim_time},
        )
        if plan_controller is None:
            actions.append(moveit)
        else:
            actions.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=plan_controller,
                        on_exit=[moveit],
                    ),
                )
            )

    if _as_bool(LaunchConfiguration('use_nav2').perform(context)):
        actions.append(
            _include_module(
                'nav2',
                {'use_sim_time': use_sim_time},
            )
        )

    if _as_bool(LaunchConfiguration('use_voice').perform(context)):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('robot_speak'),
                    '/launch/robot_speak.launch.py',
                ]),
            )
        )

    if _as_bool(LaunchConfiguration('use_rviz').perform(context)):
        actions.append(
            _include_module(
                'rviz',
                {
                    'rviz_config': rviz_config,
                    'use_moveit': LaunchConfiguration('use_moveit').perform(context),
                },
            )
        )

    return actions


def generate_launch_description():
    """声明总启动参数并延迟生成启动动作。"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='custom',
            description='Robot backend: gazebo or custom',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='auto',
            description='Use simulation clock, or auto',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='Start MoveIt move_group and /arm_target bridge',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Start Nav2 and coverage tools',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Start Gazebo GUI when mode=gazebo',
        ),
        DeclareLaunchArgument(
            'use_drive',
            default_value='false',
            description='Start real sensors and drivers when mode=custom',
        ),
        DeclareLaunchArgument(
            'use_voice',
            default_value='false',
            description='Start voice module (robot_speak)',
        ),
        DeclareLaunchArgument(
            'open_loop_control',
            default_value='false',
            description='Default open-loop feedback for hardware bridges',
        ),
        DeclareLaunchArgument(
            'motor_open_loop_control',
            default_value=LaunchConfiguration('open_loop_control'),
            description='Whether to echo wheel commands as feedback',
        ),
        DeclareLaunchArgument(
            'arm_open_loop_control',
            default_value=LaunchConfiguration('open_loop_control'),
            description='Whether to echo arm commands as feedback',
        ),
        OpaqueFunction(function=launch_setup),
    ])
