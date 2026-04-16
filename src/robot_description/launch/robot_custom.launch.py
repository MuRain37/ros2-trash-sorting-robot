import launch
import launch.actions
import launch.event_handlers
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取默认路径
    pkg_path = get_package_share_directory('robot_description')
    default_model_path = pkg_path + '/urdf/robot.custom.xacro'
    default_rviz_config_path = pkg_path + '/config/rviz/display_model.rviz'

    # 声明 Launch 参数
    action_declare_arg_mode_path = DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')

    # 生成 robot_description
    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('model')])

    robot_description = {"robot_description": robot_description_content}

    # 控制器配置文件路径
    robot_controllers = pkg_path + '/config/robot_custom_controllers.yaml'

    # ros2_control_node - 运行控制器管理器
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("/robot_diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="both",
    )

    # 状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 关节状态广播器 spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    # 差速驱动控制器 spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_diff_drive_controller", "--controller-manager", "controller_manager"],
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    # 延迟 RViz 启动，在 joint_state_broadcaster 之后
    delay_rviz_after_joint_state_broadcaster_spawner = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # 延迟 robot_controller 启动，在 joint_state_broadcaster 之后
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        action_declare_arg_mode_path,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ])