"""MoveIt warehouse 数据库启动入口。"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    """生成 warehouse 数据库 launch。"""
    moveit_config = MoveItConfigsBuilder("four_wheel_robot", package_name="robot_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
