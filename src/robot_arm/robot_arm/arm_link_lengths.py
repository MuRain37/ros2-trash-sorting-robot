"""Extract arm link lengths from the URDF published on /robot_description."""

import xml.etree.ElementTree as ET

from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


_TARGET_JOINTS = [
    'arm_shoulder_joint',
    'arm_elbow_joint',
    'arm_wrist_joint',
    'arm_tool_rotate_joint',
    'virtual_tool_roll_joint',
]


def extract_link_lengths(urdf_text):
    """Return (l1, l2, l3, l4) in centimeters, or None on parse failure."""
    root = ET.fromstring(urdf_text)

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
        joint_z['arm_shoulder_joint'] * 100.0,
        joint_z['arm_elbow_joint'] * 100.0,
        joint_z['arm_wrist_joint'] * 100.0,
        (
            joint_z['arm_tool_rotate_joint']
            + joint_z['virtual_tool_roll_joint']
        ) * 100.0,
    )


def install_urdf_link_length_loader(node, on_lengths):
    """Subscribe to /robot_description and call on_lengths((l1,l2,l3,l4))."""

    def callback(msg):
        try:
            lengths = extract_link_lengths(msg.data)
        except (ET.ParseError, ValueError, IndexError) as exc:
            node.get_logger().warning(
                f'Failed to parse arm link lengths from /robot_description: {exc}'
            )
            return

        if lengths is None:
            node.get_logger().warning(
                'Failed to extract all arm link lengths from /robot_description'
            )
            return

        on_lengths(lengths)

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    return node.create_subscription(String, '/robot_description', callback, qos)
