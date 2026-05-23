"""从 URDF 中自动提取机械臂连杆长度，避免与 kinematics.py 硬编码不同步。"""

import xml.etree.ElementTree as ET

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


# 需要提取 origin.z 的关节名
_TARGET_JOINTS = [
    'arm_shoulder_joint',       # l1
    'arm_elbow_joint',          # l2
    'arm_wrist_joint',          # l3
    'arm_tool_rotate_joint',    # l4 前半段：腕到夹爪底座
    'virtual_tool_roll_joint',  # l4 后半段：夹爪底座到 tool0
]


def extract_link_lengths(urdf_str):
    """从 URDF XML 字符串中提取连杆长度（m → cm）。

    返回:
        (l1, l2, l3, l4) 单位 cm，失败返回 None
    """
    root = ET.fromstring(urdf_str)

    joint_z = {}
    for joint in root.iter('joint'):
        name = joint.get('name', '')
        if name not in _TARGET_JOINTS:
            continue
        origin = joint.find('origin')
        if origin is None:
            continue
        xyz = origin.get('xyz', '0 0 0')
        joint_z[name] = float(xyz.split()[2])

    if len(joint_z) != len(_TARGET_JOINTS):
        return None

    return (
        joint_z['arm_shoulder_joint'] * 100,
        joint_z['arm_elbow_joint'] * 100,
        joint_z['arm_wrist_joint'] * 100,
        (joint_z['arm_tool_rotate_joint']
         + joint_z['virtual_tool_roll_joint']) * 100,
    )


def install_urdf_loader(node):
    """订阅 /robot_description 话题，收到后自动提取连杆长度并设置到 kinematics。

    robot_state_publisher 使用 transient_local QoS 发布该话题，
    因此即使订阅晚于发布，也能收到最后一条消息。

    注册一个订阅回调后立即返回，不阻塞节点初始化。
    """
    from robot_arm.kinematics import set_link_lengths

    def callback(msg):
        lengths = extract_link_lengths(msg.data)
        if lengths is not None:
            set_link_lengths(*lengths)
            node.get_logger().info(
                f'从 /robot_description 加载连杆长度: '
                f'l1={lengths[0]:.2f}, l2={lengths[1]:.2f}, '
                f'l3={lengths[2]:.2f}, l4={lengths[3]:.2f} cm'
            )
        else:
            node.get_logger().warning(
                '无法从 /robot_description 中提取连杆长度，将使用默认值'
            )
        node.destroy_subscription(sub)

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    sub = node.create_subscription(
        String, '/robot_description', callback, qos)
