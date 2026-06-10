"""从 /robot_description 发布的 URDF 中提取机械臂几何参数。"""

import xml.etree.ElementTree as ET

from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


_TARGET_JOINTS = [
    'arm_base_joint',
    'arm_shoulder_joint',
    'arm_elbow_joint',
    'arm_wrist_joint',
    'arm_tool_rotate_joint',
    'virtual_tool_roll_joint',
]


def extract_arm_geometry(urdf_text):
    """返回厘米单位的机械臂几何参数，解析失败返回 None。"""
    root = ET.fromstring(urdf_text)

    joint_origins = {}
    for joint in root.iter('joint'):
        name = joint.get('name', '')
        if name not in _TARGET_JOINTS:
            continue

        origin = joint.find('origin')
        if origin is None:
            continue

        xyz = origin.get('xyz', '0 0 0')
        values = tuple(float(value) for value in xyz.split())
        if len(values) != 3:
            return None
        joint_origins[name] = values

    if len(joint_origins) != len(_TARGET_JOINTS):
        return None

    origin = tuple(value * 100.0 for value in joint_origins['arm_base_joint'])
    lengths = (
        joint_origins['arm_shoulder_joint'][2] * 100.0,
        joint_origins['arm_elbow_joint'][2] * 100.0,
        joint_origins['arm_wrist_joint'][2] * 100.0,
        (
            joint_origins['arm_tool_rotate_joint'][2]
            + joint_origins['virtual_tool_roll_joint'][2]
        ) * 100.0,
    )
    return origin, lengths


def extract_link_lengths(urdf_text):
    """返回厘米单位的 (l1, l2, l3, l4)，解析失败返回 None。"""
    geometry = extract_arm_geometry(urdf_text)
    if geometry is None:
        return None
    return geometry[1]


def install_urdf_link_length_loader(node, on_geometry):
    """订阅 /robot_description 并在解析成功后回调 on_geometry。"""

    def callback(msg):
        try:
            geometry = extract_arm_geometry(msg.data)
        except (ET.ParseError, ValueError, IndexError) as exc:
            node.get_logger().warning(
                f'Failed to parse arm geometry from /robot_description: {exc}'
            )
            return

        if geometry is None:
            node.get_logger().warning(
                'Failed to extract all arm geometry from /robot_description'
            )
            return

        on_geometry(*geometry)

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    return node.create_subscription(String, '/robot_description', callback, qos)
