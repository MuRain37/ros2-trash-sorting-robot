import numpy as np
import matplotlib.pyplot as plt


# 关节运动范围配置（度）
JOINT_LIMITS = {
    'theta1': (-180, 180),  # 底座旋转
    'theta2': (-120, 120),   # 肩关节
    'theta3': (-120, 120),     # 肘关节
    'theta4': (-120, 120),     # 末端关节
}

# 机械臂连杆长度配置
LINK_LENGTHS = {
    'l1': 3.38648,    # 基座高度
    'l2': 12.9416,  # 大臂长度
    'l3': 12.9444,   # 小臂长度
    'l4': 11.38648,  # 末端执行器长度
}

LINK_RADIUS = 2.0  # 连杆碰撞半径


def _within_limits(theta1, theta2, theta3, theta4):
    """检查关节角度是否在运动范围内"""
    return (
        JOINT_LIMITS['theta1'][0] <= theta1 <= JOINT_LIMITS['theta1'][1]
        and JOINT_LIMITS['theta2'][0] <= theta2 <= JOINT_LIMITS['theta2'][1]
        and JOINT_LIMITS['theta3'][0] <= theta3 <= JOINT_LIMITS['theta3'][1]
        and JOINT_LIMITS['theta4'][0] <= theta4 <= JOINT_LIMITS['theta4'][1]
    )


def _seg_dist(a, b):
    """点到线段ab的最短距离（平方）"""
    ab = b - a
    t = np.dot(-a, ab) / np.dot(ab, ab) if np.dot(ab, ab) > 1e-12 else 0.0
    if t < 0:
        return np.dot(a, a)
    elif t > 1:
        return np.dot(a + ab, a + ab)
    else:
        p = a + t * ab
        return np.dot(p, p)


def _seg_seg_min_dist(a1, a2, b1, b2):
    """两线段之间的最短距离"""
    return np.sqrt(min(
        _seg_dist(b1 - a1, b2 - a1),
        _seg_dist(b1 - a2, b2 - a2),
        _seg_dist(a1 - b1, a2 - b1),
        _seg_dist(a1 - b2, a2 - b2),
    ))


def _check_collision(joints):
    """检查连杆之间是否自碰撞，非相邻连杆间距 < 2*LINK_RADIUS 视为碰撞"""
    # 连杆：l1=0-1, l2=1-2, l3=2-3, l4=3-4
    non_adjacent = [
        (0, 1, 2, 3),  # l1 vs l3
        (0, 1, 3, 4),  # l1 vs l4
        (1, 2, 3, 4),  # l2 vs l4
    ]
    threshold = 2 * LINK_RADIUS
    for i1, j1, i2, j2 in non_adjacent:
        if _seg_seg_min_dist(joints[i1], joints[j1], joints[i2], joints[j2]) < threshold:
            return True
    return False


def forward_kinematics(l1, l2, l3, l4, theta1, theta2, theta3, theta4):
    """
    正运动学：根据关节角度计算末端位置

    坐标系：X轴=前方，Y轴=右方，Z轴=上方

    角度定义：
    - theta1: 底座旋转角（度），theta1=0指向前方(X+)，右正左负
    - theta2: 肩关节角度（度），0°时大臂垂直向上，正值向前倾斜
    - theta3: 肘关节角度（度），0°时小臂与大臂平行（同向），正值向前弯曲
    - theta4: 末端关节角度（度），0°时末端垂直向上，正值向前倾斜

    零位状态（所有角度为0）：机械臂完全垂直于地板向上

    返回:
        各关节位置列表: [基座底, 基座顶, 肩关节, 肘关节, 末端]
    """
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    theta3_rad = np.radians(theta3)
    theta4_rad = np.radians(theta4)

    # 基座顶（基座是垂直的，不受theta1影响）
    joint1 = np.array([0, 0, l1])

    # 肩关节：大臂从基座顶出发
    # 坐标系：X轴=前方，Y轴=右方，Z轴=上方
    # theta1=0时指向前方(X正方向)，theta1>0时指向右方(Y正方向)
    # theta2=0时垂直向上，theta2>0时向前倾斜
    pitch2 = np.pi/2 - theta2_rad
    joint2 = np.array([
        l2 * np.cos(pitch2) * np.cos(theta1_rad),
        l2 * np.cos(pitch2) * np.sin(theta1_rad),
        l1 + l2 * np.sin(pitch2)
    ])

    # 肘关节：小臂从大臂末端出发
    pitch3 = np.pi/2 - (theta2_rad + theta3_rad)
    joint3 = np.array([
        joint2[0] + l3 * np.cos(pitch3) * np.cos(theta1_rad),
        joint2[1] + l3 * np.cos(pitch3) * np.sin(theta1_rad),
        joint2[2] + l3 * np.sin(pitch3)
    ])

    # 末端：从肘关节出发
    pitch4 = np.pi/2 - (theta2_rad + theta3_rad + theta4_rad)
    joint4 = np.array([
        joint3[0] + l4 * np.cos(pitch4) * np.cos(theta1_rad),
        joint3[1] + l4 * np.cos(pitch4) * np.sin(theta1_rad),
        joint3[2] + l4 * np.sin(pitch4)
    ])

    return [np.array([0, 0, 0]), joint1, joint2, joint3, joint4]


def inverse_kinematics(l1, l2, l3, l4, angle, target):
    """
    完整的4-DOF逆运动学求解，返回肘上+肘下两个解

    参数:
        l1, l2, l3, l4: 机械臂各段长度
        angle: 末端执行器俯仰角度（度）
               正值 = 向下倾斜, 负值 = 向上倾斜, 0° = 水平
        target: 目标位置 [x, y, z]
               坐标系：X轴=前方，Y轴=右方，Z轴=上方

    返回:
        解列表 [(theta1, theta2, theta3, theta4), ...]，空列表表示不可达
        第一个解为肘向上，第二个解（如有）为肘向下
    """
    target = np.array(target, dtype=float)
    x, y, _ = target

    # 第一步：计算theta1（底座旋转角）
    theta1 = np.degrees(np.arctan2(y, x))
    theta1_rad = np.radians(theta1)

    # 末端方向：angle是末端与水平的夹角，正值向下
    pitch_end = np.radians(-angle)
    end_dir = np.array([
        np.cos(pitch_end) * np.cos(theta1_rad),
        np.cos(pitch_end) * np.sin(theta1_rad),
        np.sin(pitch_end)
    ])

    # 腕部位置 = 末端位置 - l4 * 末端方向
    wrist = target - l4 * end_dir

    # 第二步：在手臂平面内求解theta2和theta3
    r = wrist[0] * np.cos(theta1_rad) + wrist[1] * np.sin(theta1_rad)
    z_wrist = wrist[2] - l1

    d = np.sqrt(r**2 + z_wrist**2)

    # 可达性检查
    if d > (l2 + l3) or d < abs(l2 - l3):
        return []

    # 余弦定理求肘关节内角
    cos_theta3 = (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    theta3_magnitude = np.degrees(np.arccos(cos_theta3))

    # alpha: 腕部的仰角; beta: l2与连线d的夹角
    alpha = np.arctan2(z_wrist, r)
    cos_beta = (l2**2 + d**2 - l3**2) / (2 * l2 * d)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)

    solutions = []

    # 肘向上解
    theta2_up = np.degrees(np.pi / 2 - (alpha + beta))
    theta3_up = 180 - theta3_magnitude
    theta4_up = 90 + angle - theta2_up - theta3_up
    solutions.append((theta1, theta2_up, theta3_up, theta4_up))

    # 肘向下解（当beta非零时与肘上解不同）
    if beta > 1e-9:
        theta2_down = np.degrees(np.pi / 2 - (alpha - beta))
        theta3_down = theta3_magnitude - 180
        theta4_down = 90 + angle - theta2_down - theta3_down
        solutions.append((theta1, theta2_down, theta3_down, theta4_down))

    return solutions


def plot_arm(l1, l2, l3, l4, theta1, theta2, theta3, theta4, target):
    """
    绘制机械臂3D图
    """
    joints = forward_kinematics(l1, l2, l3, l4, theta1, theta2, theta3, theta4)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = ['black', 'red', 'green', 'blue', 'purple']

    # 绘制连杆
    for i in range(len(joints) - 1):
        ax.plot([joints[i][0], joints[i+1][0]],
                [joints[i][1], joints[i+1][1]],
                [joints[i][2], joints[i+1][2]],
                color=colors[i], linewidth=3, label=f'Link {i+1}')

    # 绘制关节点
    for i, joint in enumerate(joints):
        ax.scatter(*joint, color=colors[i], s=100, zorder=5)

    # 绘制目标点
    ax.scatter(*target, color='orange', s=150, marker='*', label='Target', zorder=6)

    # 绘制垂直参考线
    ax.plot([0, 0], [0, 0], [0, l1+l2+l3+l4], 'k--', alpha=0.3, label='Vertical ref')

    # 设置坐标轴
    total_len = l1 + l2 + l3 + l4
    ax.set_xlim([-total_len, total_len])
    ax.set_ylim([-total_len, total_len])
    ax.set_zlim([0, total_len * 1.2])
    ax.set_xlabel('X (Forward)')
    ax.set_ylabel('Y (Right +)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'4-DOF Robot Arm\n'
                 f'θ1={theta1:.1f}° θ2={theta2:.1f}° θ3={theta3:.1f}° θ4={theta4:.1f}°')
    ax.legend(loc='upper left', fontsize=8)

    # 设置相等的比例
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


def solve(target, angle=None, verify=False, plot=False, collision_check=True):
    """
    求解逆运动学，返回关节角度

    参数:
        target: 目标位置 [x, y, z]
        angle: 末端执行器俯仰角度（度），默认None自动选择最优角度
        verify: 是否进行正解验证（默认False）
        plot: 是否绘制机械臂（默认False）
        collision_check: 是否检测连杆碰撞（默认True）

    连杆长度和关节范围从 LINK_LENGTHS / JOINT_LIMITS 配置读取。

    返回:
        (theta1, theta2, theta3, theta4, angle) 角度（度），或 False 表示不可达
    """
    l1, l2, l3, l4 = LINK_LENGTHS['l1'], LINK_LENGTHS['l2'], LINK_LENGTHS['l3'], LINK_LENGTHS['l4']

    best_angle = angle if angle is not None else 90  # best 目标角度
    cur_angle = 90
    best = None  # (theta1, theta2, theta3, theta4, angle)

    for a in range(90, -20, -1):  # 90° 向下搜索到 -90°
        solutions = inverse_kinematics(l1, l2, l3, l4, a, target)
        for theta1, theta2, theta3, theta4 in solutions:
            if not _within_limits(theta1, theta2, theta3, theta4):
                continue
            if collision_check:
                joints = forward_kinematics(l1, l2, l3, l4, theta1, theta2, theta3, theta4)
                if _check_collision(joints):
                    continue
            # 找到有效解，替换 best（越往下越接近 best_angle）
            cur = abs(a - best_angle) 
            if abs(a - best_angle) < cur_angle:
                best = (theta1, theta2, theta3, theta4, a)
                cur_angle = cur
            if a == best_angle:
                return best  # 到达 best 角度，完美，直接返回
            break  # 一个角度只取一个解，继续往下搜

    if best is None:
        return False

    theta1, theta2, theta3, theta4, a = best
    print(f"θ1={theta1:.2f}°, θ2={theta2:.2f}°, θ3={theta3:.2f}°, θ4={theta4:.2f}°, 俯仰角={a}°")

    if verify:
        joints = forward_kinematics(l1, l2, l3, l4, theta1, theta2, theta3, theta4)
        actual_pos = joints[-1]
        error = np.linalg.norm(actual_pos - np.array(target))
        print(f"正解验证 - 目标: {target}, 实际: [{actual_pos[0]:.2f}, {actual_pos[1]:.2f}, {actual_pos[2]:.2f}], 误差: {error:.6f}mm")

    if plot:
        plot_arm(l1, l2, l3, l4, theta1, theta2, theta3, theta4, target)

    return (theta1, theta2, theta3, theta4, a)


# ==================== 主程序 ====================

if __name__ == "__main__":
    target = [10, 0, 15]
    result = solve(target, angle=None, verify=True, plot=True, collision_check=True)
    if result is False:
        print("目标位置不可达")