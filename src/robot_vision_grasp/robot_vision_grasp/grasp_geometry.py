"""Object-specific grasp geometry helpers."""

import numpy as np


def _is_upright(points_cam, cam_pitch=0.0, seg_mask=None, depth_img=None):
    """Return True when the object is likely standing upright."""
    if seg_mask is not None and depth_img is not None and np.any(seg_mask):
        ys, xs = np.where(seg_mask)
        h_px = int(ys.max() - ys.min())
        w_px = int(xs.max() - xs.min())
        if w_px > h_px * 1.2:
            return False

        y_hi = ys.min() + h_px // 4
        y_lo = ys.min() + 3 * h_px // 4
        top = ys <= y_hi
        bottom = ys >= y_lo
        d_top = depth_img[ys[top], xs[top]]
        d_bottom = depth_img[ys[bottom], xs[bottom]]
        d_top = d_top[(d_top > 0) & (d_top <= 2.0)]
        d_bottom = d_bottom[(d_bottom > 0) & (d_bottom <= 2.0)]

        if len(d_top) >= 5 and len(d_bottom) >= 5:
            diff = float(np.median(d_bottom) - np.median(d_top))
            if diff > 0.03:
                return True
            if diff < -0.03:
                return False

    if points_cam is None or len(points_cam) < 20:
        return True

    pitch_r = np.radians(cam_pitch)
    c, s = np.cos(pitch_r), np.sin(pitch_r)
    r_level = np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])
    pts_level = points_cam @ r_level.T
    extent = pts_level.max(axis=0) - pts_level.min(axis=0)
    return extent[2] > extent[0] and extent[2] > extent[1]


def _get_axis_angle(points_cam, cam_pitch=0.0):
    """Estimate the horizontal principal-axis angle in degrees."""
    if points_cam is None or len(points_cam) < 10:
        return 0.0

    pitch_r = np.radians(cam_pitch)
    c, s = np.cos(pitch_r), np.sin(pitch_r)
    r_level = np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])
    pts_level = points_cam @ r_level.T
    xz = pts_level[:, [0, 2]]
    if len(xz) < 10:
        return 0.0

    centered = xz - xz.mean(axis=0)
    cov = np.cov(centered.T)
    _evals, evecs = np.linalg.eigh(cov)
    principal = evecs[:, 1]
    angle = float(np.degrees(np.arctan2(principal[0], principal[1])))
    if angle > 90.0:
        angle -= 180.0
    elif angle < -90.0:
        angle += 180.0
    return angle


def get_ik_pitch(class_name, points_cam=None, cam_pitch=0.0,
                 seg_mask=None, depth_img=None):
    """Return (end_pitch_deg, axis_angle_deg, is_upright)."""
    pitch_table = {
        'pen': 90.0,
        'paper': 65.0,
        'charger': 90.0,
        'usb': 90.0,
        'yan': 90.0,

    }

    if class_name == 'paper':
        return pitch_table[class_name], 0.0, False

    if class_name == 'bottle':
        is_up = _is_upright(points_cam, cam_pitch, seg_mask, depth_img)
        angle_3d = 0.0 if is_up else _get_axis_angle(points_cam, cam_pitch)
        return (0.0 if is_up else 90.0), angle_3d, is_up

    else:
        angle_3d = _get_axis_angle(points_cam, cam_pitch)
        return pitch_table[class_name], angle_3d, False

    return 90.0, 0.0, False


def get_gripper_yaw(class_name, angle_3d=None, is_upright=True):
    """Return tool-rotate joint yaw in degrees."""
    if class_name  == 'paper':
        return 0.0
    if angle_3d is None or is_upright:
        return 0.0
    return float(angle_3d)


def gripper_close_open_ratio(class_name, open_raw=0.0, close_raw=1000.0):
    width_cm_by_class = {
        'bottle': 3.0,
        'charger': 1.0,
    }
    width_cm = width_cm_by_class.get(class_name, 0.0)
    old_raw = int(1000 - 500.0 / 3.0 * width_cm)
    old_raw = max(0, min(1000, old_raw))

    denominator = float(open_raw - close_raw)
    if abs(denominator) < 1e-9:
        return 0.0

    ratio = (old_raw - close_raw) / denominator
    return max(0.0, min(1.0, float(ratio)))
