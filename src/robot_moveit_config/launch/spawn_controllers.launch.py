"""MoveIt 控制器 spawner 启动入口。"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    """生成控制器 spawner launch。"""
    moveit_config = MoveItConfigsBuilder("four_wheel_robot", package_name="robot_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
