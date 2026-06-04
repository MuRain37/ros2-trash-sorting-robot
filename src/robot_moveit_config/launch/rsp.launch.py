"""MoveIt robot_state_publisher 启动入口。"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    """生成 MoveIt RSP launch。"""
    moveit_config = MoveItConfigsBuilder("four_wheel_robot", package_name="robot_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
