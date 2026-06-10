#!/usr/bin/env python3
"""RealSense D435 三路可视化：色彩 / 深度 / 点云（含俯视障碍物视图）。"""

import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ──────────────────────────── 常量 ────────────────────────────
W, H = 640, 480          # 图像尺寸
MAX_Z = 3.0              # 最大显示距离（米）
BEV_SIZE = 400           # 俯视图边长（像素）
BEV_RANGE = 3.0          # 俯视图范围（米）


class TripleViewer(Node):
    """同时订阅 color / aligned_depth / pointcloud2 / camera_info 并显示。"""

    def __init__(self):
        super().__init__('triple_viewer')
        self.bridge = CvBridge()

        # ── 图像缓冲区 ──
        self._color = None
        self._depth_color = None
        self._pc_points = None      # (N, 3) ndarray
        self._K = None              # 3×3 相机内参

        # ── 订阅 ──
        self.create_subscription(Image,       '/camera/camera/color/image_raw',
                                 self._cb_color, 1)
        self.create_subscription(Image,       '/camera/camera/aligned_depth_to_color/image_raw',
                                 self._cb_depth, 1)
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points',
                                 self._cb_pc, 1)
        self.create_subscription(CameraInfo,  '/camera/camera/aligned_depth_to_color/camera_info',
                                 self._cb_info, 1)

        # ── 定时器：~20 Hz 刷新界面 ──
        self.create_timer(0.05, self._display)
        self.get_logger().info(
            'TripleViewer 已启动 —— 三窗口：  色彩  |  深度  |  点云'
        )

    # ──────────────────────── 回调 ────────────────────────

    def _cb_color(self, msg):
        try:
            self._color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'色彩转换失败: {exc}')

    def _cb_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            depth = np.asarray(depth, dtype=np.float32)
            # ROS2 RealSense 深度单位为米，但保险判断一下
            if depth.max() > 10.0:
                depth *= 0.001  # mm → m
            depth = np.clip(depth, 0, MAX_Z)
            norm = cv2.normalize(depth, None, 0, 255,
                                 cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self._depth_color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        except Exception as exc:
            self.get_logger().error(f'深度转换失败: {exc}')

    def _cb_info(self, msg):
        self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _cb_pc(self, msg):
        try:
            points = np.array(list(
                pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            ))
            if len(points) == 0:
                return
            # 滤除无效 / 过远点
            mask = (points[:, 2] > 0.1) & (points[:, 2] < MAX_Z)
            points = points[mask]
            if len(points) == 0:
                return
            self._pc_points = points
        except Exception as exc:
            self.get_logger().error(f'点云转换失败: {exc}')

    # ──────────────────────── 显示 ────────────────────────

    def _display(self):
        if self._color is not None:
            cv2.imshow('1 — 色彩 (Color)', self._color)

        if self._depth_color is not None:
            cv2.imshow('2 — 深度 (Depth)', self._depth_color)

        if self._pc_points is not None and self._K is not None:
            pc_img = self._render_pointcloud()
            cv2.imshow('3 — 点云 (PointCloud)', pc_img)

        cv2.waitKey(1)

    def _render_pointcloud(self):
        """将点云渲染为两张图左右拼接：距离图 + 俯视障碍物图。"""
        pts = self._pc_points
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]

        # ── 左半：投影到像平面 → 距离图 ──
        u = (pts[:, 0] * fx / pts[:, 2] + cx).astype(np.int32)
        v = (pts[:, 1] * fy / pts[:, 2] + cy).astype(np.int32)
        mask_uv = (u >= 0) & (u < W) & (v >= 0) & (v < H)

        depth_map = np.full((H, W), np.inf, dtype=np.float32)
        np.minimum.at(depth_map, (v[mask_uv], u[mask_uv]), pts[mask_uv, 2])
        depth_map = np.clip(depth_map, 0, MAX_Z)
        depth_map[depth_map == np.inf] = 0.0

        norm = cv2.normalize(depth_map, None, 0, 255,
                             cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        left = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        cv2.putText(left, f'Distance   pts={len(pts)}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # ── 右半：俯视障碍物图（BEV, x vs z） ──
        # 相机坐标系：x→右,  y→下,  z→前（远离相机）
        # 俯视图：水平=x, 垂直=z（前向距离）
        size = BEV_SIZE
        half = size // 2
        u_bev = (pts[:, 0] / BEV_RANGE * half + half).astype(np.int32)
        v_bev = (pts[:, 2] / BEV_RANGE * size).astype(np.int32)
        mask_bev = (u_bev >= 0) & (u_bev < size) & (v_bev >= 0) & (v_bev < size)

        bev = np.zeros((size, size), dtype=np.float32)
        np.add.at(bev, (v_bev[mask_bev], u_bev[mask_bev]), 1.0)
        # 对数压缩使稀缺点也可见
        bev = np.log1p(bev)
        bev = cv2.normalize(bev, None, 0, 255,
                            cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        right = cv2.applyColorMap(bev, cv2.COLORMAP_VIRIDIS)
        cv2.putText(right, 'BEV  x-z', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # 标注方向
        cv2.putText(right, 'z (forward)', (10, size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(right, 'x (right)', (size - 80, size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # ── 拼接 ──
        # 把 left 缩放到与 right 等高等宽，方便并排
        left_resized = cv2.resize(left, (size, size))
        return np.hstack((left_resized, right))


def main(args=None):
    rclpy.init(args=args)
    node = TripleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
