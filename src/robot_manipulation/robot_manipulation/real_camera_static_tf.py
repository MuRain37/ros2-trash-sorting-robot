"""发布真实相机相对底盘的静态 TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import StaticTransformBroadcaster

from robot_interfaces.srv import SetCameraTfPreset
from robot_manipulation.arm_presets import ACTION_ARM_CENTER
from robot_manipulation.arm_presets import ACTION_ARM_MAPPING
from robot_manipulation.arm_presets import preset_name


CAMERA_TF_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')

# 标定时只需要改这里：每行依次对应 x, y, z, roll, pitch, yaw
PRESET_CAMERA_TF = {
    ACTION_ARM_CENTER: {
        'parameter_prefix': 'arm_center',
        'values': [0.27, 0.03, 0.28, 0.0, 0.95, 0.0],
    },
    ACTION_ARM_MAPPING: {
        'parameter_prefix': 'arm_mapping',
        'values': [0.23, 0.0, 0.28, 0.0, 0.7, 0.0],
    },
}


class RealCameraStaticTf(Node):
    """发布 base_link 到真实相机坐标系的固定变换."""

    def __init__(self):
        super().__init__('real_camera_static_tf')

        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'real_camera_link')
        self.declare_parameter('x', 0.30)
        self.declare_parameter('y', 0.03)
        self.declare_parameter('z', 0.28)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.95)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter(
            'set_preset_service',
            '/real_camera_static_tf/set_preset',
        )

        for preset in PRESET_CAMERA_TF.values():
            prefix = preset['parameter_prefix']
            for field, value in zip(CAMERA_TF_FIELDS, preset['values']):
                self.declare_parameter(f'{prefix}.{field}', value)

        self._broadcaster = StaticTransformBroadcaster(self)
        set_preset_service = str(
            self.get_parameter('set_preset_service').value
        )
        self._set_preset_service = self.create_service(
            SetCameraTfPreset,
            set_preset_service,
            self._handle_set_preset,
        )

        self.add_on_set_parameters_callback(self._on_parameter_changed)

        self._publish_transform('initial')

    def _handle_set_preset(self, request, response):
        """服务请求进入关键姿态时，按对应参数重发真实相机 TF."""
        action_type = int(request.action_type)
        if action_type not in PRESET_CAMERA_TF:
            response.success = False
            response.message = (
                f'unsupported camera tf preset action_type={action_type}'
            )
            return response

        parameter_prefix = PRESET_CAMERA_TF[action_type]['parameter_prefix']
        self._publish_transform(parameter_prefix)
        response.success = True
        response.message = f'real camera tf set for {parameter_prefix}'
        return response

    def _on_parameter_changed(self, params):
        """参数变更时自动重发 TF（使用回调中的新值，避免读到旧参数）."""
        # 按 prefix 分组收集回调中的新值
        prefix_overrides = {}
        for param in params:
            name = param.name
            parts = name.split('.', 1)
            if len(parts) == 2 and parts[0] in self._preset_parameter_prefixes():
                prefix = parts[0]
                if prefix not in prefix_overrides:
                    prefix_overrides[prefix] = {}
                prefix_overrides[prefix][name] = param.value
            elif name in CAMERA_TF_FIELDS:
                if None not in prefix_overrides:
                    prefix_overrides[None] = {}
                prefix_overrides[None][name] = param.value

        for prefix, overrides in prefix_overrides.items():
            self._publish_transform(prefix, overrides)

        return SetParametersResult(successful=True)

    def _publish_transform(self, parameter_prefix=None, overrides=None):
        parent_frame = str(self.get_parameter('parent_frame').value)
        child_frame = str(self.get_parameter('child_frame').value)
        x, y, z, roll, pitch, yaw = self._read_transform_parameters(
            parameter_prefix, overrides
        )

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
        state_name = self._state_name(parameter_prefix)
        self.get_logger().info(
            f'Real camera static TF [{state_name}]: '
            f'{parent_frame} -> {child_frame}, '
            f'xyz=({x:.4f}, {y:.4f}, {z:.4f}), '
            f'rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})'
        )

    def _read_transform_parameters(self, parameter_prefix, overrides=None):
        """读取默认或指定姿态下的真实相机 TF 参数."""
        if parameter_prefix in self._preset_parameter_prefixes():
            names = [
                f'{parameter_prefix}.{field}'
                for field in CAMERA_TF_FIELDS
            ]
        else:
            names = list(CAMERA_TF_FIELDS)
        return tuple(
            float(overrides[name]) if overrides and name in overrides
            else float(self.get_parameter(name).value)
            for name in names
        )

    def _state_name(self, parameter_prefix):
        for action_type, preset in PRESET_CAMERA_TF.items():
            if parameter_prefix == preset['parameter_prefix']:
                return preset_name(action_type)
        return 'initial'

    def _preset_parameter_prefixes(self):
        return {
            preset['parameter_prefix']
            for preset in PRESET_CAMERA_TF.values()
        }


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
