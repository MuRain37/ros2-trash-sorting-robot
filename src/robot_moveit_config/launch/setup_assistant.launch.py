"""MoveIt Setup Assistant 启动入口。"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    """生成 Setup Assistant launch。"""
    moveit_config = MoveItConfigsBuilder("four_wheel_robot", package_name="robot_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
