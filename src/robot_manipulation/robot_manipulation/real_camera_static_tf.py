"""发布真实相机相对底盘的静态 TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class RealCameraStaticTf(Node):
    """发布 base_link 到真实相机坐标系的固定变换."""

    def __init__(self):
        super().__init__('real_camera_static_tf')

        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'real_camera_link')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish_transform()

    def _publish_transform(self):
        parent_frame = str(self.get_parameter('parent_frame').value)
        child_frame = str(self.get_parameter('child_frame').value)
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        qx, qy, qz, qw = _quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f'Real camera static TF: {parent_frame} -> {child_frame}, '
            f'xyz=({x:.4f}, {y:.4f}, {z:.4f}), '
            f'rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})'
        )


def _quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main(args=None):
    """启动真实相机静态 TF 发布节点."""
    rclpy.init(args=args)
    node = RealCameraStaticTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
