"""E6 正运动学（从 practice/p2_fk.cpp 移植，用 numpy 批量计算）。

这里它有两个身份：
1. 数据生成器：随机关节角 → 精确的末端位置，当作训练数据的"标准答案"
2. 裁判：训练完后拿来对比神经网络算得准不准
"""
import numpy as np

# 直接抄自 p2_fk.cpp 的 kJoints（来自 me6_robot.urdf）：x, y, z, roll, pitch, yaw
JOINTS = np.array([
    [0.0,    0.0,     0.1268, 0.0,     0.0,    0.0],
    [-0.046, 0.0,     0.04,   1.5708,  0.0,    -1.5708],
    [0.0,    0.18906, 0.003,  0.0,     0.0,    0.0],
    [0.0,    0.16,    0.005,  0.0,     0.0,    0.0],
    [0.0,    0.067,   0.032,  -1.5708, 1.5708, 0.0],
    [-0.047, 0.0,     0.034,  1.5708,  0.0,    -1.5708],
])


def _rot(axis, a):
    """绕 x/y/z 轴旋转的 4×4 矩阵；a 可以是一批角度，返回 (N,4,4)。"""
    a = np.asarray(a, dtype=float)
    c, s = np.cos(a), np.sin(a)
    m = np.zeros(a.shape + (4, 4))
    m[..., 3, 3] = 1
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    k = 3 - i - j
    m[..., k, k] = 1
    m[..., i, i], m[..., i, j] = c, -s
    m[..., j, i], m[..., j, j] = s, c
    return m


def _origin(x, y, z, roll, pitch, yaw):
    m = _rot("z", yaw) @ _rot("y", pitch) @ _rot("x", roll)
    m[:3, 3] = x, y, z
    return m


_ORIGINS = [_origin(*j) for j in JOINTS]


def forward_kinematics(q):
    """q: (N,6) 关节角（弧度）→ (N,3) 末端 Link6 位置（米）。"""
    q = np.atleast_2d(q)
    t = np.broadcast_to(np.eye(4), (len(q), 4, 4))
    for i in range(6):
        t = t @ _ORIGINS[i] @ _rot("z", q[:, i])
    return t[:, :3, 3]


if __name__ == "__main__":
    # 和 p2_fk.cpp 一样的测试数据（来自 tf2_echo / MoveIt 实测）
    cases = {
        "全部归零":   ([0, 0, 0, 0, 0, 0], [-0.1330, 0.0000, 0.6169]),
        "home 姿态":  ([0, 0.5337, 0.7276, 0.3548, -1.5848, 0], [-0.0853, 0.3473, 0.3268]),
        "任意姿态 A": ([0.5, -0.3, 0.8, 0.2, 1.0, -0.7], [-0.1244, -0.0046, 0.5906]),
        "任意姿态 B": ([-1.2, 0.9, -1.1, 0.6, -0.5, 2.0], [0.1183, 0.1825, 0.5254]),
        "任意姿态 C": ([2.0, -1.0, 1.5, -0.8, 0.3, 0.4], [0.1686, -0.0668, 0.5017]),
    }
    for name, (q, want) in cases.items():
        err = np.linalg.norm(forward_kinematics(q)[0] - want) * 1000
        print(f"{'✅' if err < 1 else '❌'} {name:10s} 误差 {err:.2f} mm")
