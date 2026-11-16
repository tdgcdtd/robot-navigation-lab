"""M5 的仿真世界：E6 机械臂 + 桌子 + 方块 + 相机，都在同一个 MuJoCo 里。

它扮演"真机器人和真桌面"：MoveIt 规划的轨迹交给这里执行，相机拍到的就是执行的真实结果。

坐标系和 ROS 那边完全一致（arm_ws/src/e6_llm/config/world.yaml）：
- 原点 = 机械臂底座 dummy_link，x 朝前，y 朝左，z 朝上，单位米
- 桌面在 z ≈ 0（和 task_executor 里的 table 一样：0.8 × 0.8 m，顶面 z = -0.001）
- 方块边长 4 cm，放在桌上时中心 z = 0.02

和 M3 / M4 的场景比：那时原点在桌面中心、没有机械臂；这里原点在机械臂底座。
"""
import os
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
E6_XML = os.path.join(HERE, "assets", "e6.xml")      # build_model.py 生成
JOINTS = [f"joint{i}" for i in range(1, 7)]
HOME = [0, 0.5337, 0.7276, 0.3548, -1.5848, 0]       # 和 e6_motion 里的 home 姿态一样
CUBE = 0.04
COLORS = {"red": (0.85, 0.1, 0.1, 1), "blue": (0.1, 0.25, 0.85, 1), "green": (0.1, 0.65, 0.2, 1),
          "yellow": (0.95, 0.85, 0.1, 1), "orange": (0.95, 0.45, 0.1, 1), "purple": (0.45, 0.15, 0.75, 1)}
W, H = 640, 480
# 相机在机械臂对面、斜着往下看工作区中心：机械臂在画面远处，伸手抓方块时不容易挡住方块
CAM_POS, CAM_TARGET = (1.05, 0.0, 0.75), (0.15, 0.0, 0.10)


def build(cubes):
    """cubes: {"red": (x, y), ...}（放在桌上）或 {"red": (x, y, z)}。返回 (model, data)。"""
    spec = mujoco.MjSpec.from_file(E6_XML)
    spec.option.timestep = 0.002
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    wb = spec.worldbody
    wb.add_light(pos=[0.5, -0.3, 1.5], dir=[-0.2, 0.2, -1], diffuse=[0.7, 0.7, 0.7])
    wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, -0.011], size=[0.4, 0.4, 0.01],
                rgba=[0.62, 0.47, 0.33, 1])
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[0, 0, -0.75], size=[3, 3, 0.1],
                rgba=[0.3, 0.3, 0.32, 1])
    for color, p in cubes.items():
        z = p[2] if len(p) > 2 else CUBE / 2
        b = wb.add_body(name=f"{color}_cube", pos=[p[0], p[1], z])
        b.add_freejoint()                               # 自由关节：方块可以被推、被抓、掉下去
        b.add_geom(name=f"{color}_cube", type=mujoco.mjtGeom.mjGEOM_BOX, size=[CUBE / 2] * 3,
                   rgba=COLORS[color], mass=0.03, friction=[1.0, 0.005, 0.0001])
    # 相机：xyaxes = 相机 x 轴（朝右）、y 轴（朝上）在世界里的方向
    fwd = np.subtract(CAM_TARGET, CAM_POS); fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    wb.add_camera(name="cam", pos=CAM_POS, xyaxes=[*right, *up], fovy=50)
    # 每个关节一个位置伺服：给目标角度，它出力把关节拉过去（真机械臂的驱动器也是这样）
    for j in JOINTS:
        spec.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[2000] + [0] * 9, biasprm=[0, -2000, -60] + [0] * 7)
    for name in ["base_link"] + [f"Link{i}" for i in range(1, 7)]:
        # 重力补偿：驱动器额外出一份力抵消连杆自重，否则纯位置伺服会被重力拉得下垂（实测 joint2 下垂 0.23°）。
        # 必须在 compile 之前设：MuJoCo 编译时决定要不要算这一项
        spec.body(name).gravcomp = 1.0
    model = spec.compile()
    _stabilize(model)
    data = mujoco.MjData(model)
    set_joints(model, data, HOME)
    return model, data


def _stabilize(model):
    """让机械臂在物理仿真里稳定（直接用 URDF 做物理会"炸"，原因见下）。"""
    arm_bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                  for n in ["base_link"] + [f"Link{i}" for i in range(1, 7)]}
    for g in range(model.ngeom):
        if model.geom_bodyid[g] in arm_bodies:
            # 碰撞分组：机械臂只和方块碰撞，不和自己、不和桌子碰撞。
            # 相邻连杆的 STL 在关节处本来就重叠，底座也微微陷进桌面，不关掉的话物理引擎会拼命把它们推开。
            # （机械臂撞自己 / 撞桌子由 MoveIt 在规划时负责避免）
            model.geom_contype[g], model.geom_conaffinity[g] = 2, 0
        elif model.geom_bodyid[g] != 0:
            model.geom_contype[g], model.geom_conaffinity[g] = 1, 3      # 方块：和桌子、别的方块、机械臂都碰撞
    # armature：电机转子经过减速器折算到关节上的惯量。URDF 里连杆总共才 0.94 kg（真机约 7 kg），
    # 不加的话"很轻的连杆 + 很硬的伺服"会越振越大
    model.dof_armature[:6] = 0.1
    model.dof_damping[:6] = 2.0
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST   # 对"硬"的伺服更稳的积分方法


def joint_addr(model):
    return [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS]


def set_joints(model, data, q):
    """直接把机械臂摆到某个姿态（瞬移，不走物理），同时把伺服目标设成同一个姿态。"""
    data.qpos[joint_addr(model)] = q
    data.ctrl[:6] = q
    mujoco.mj_forward(model, data)


def render(model, data):
    r = mujoco.Renderer(model, H, W)
    r.update_scene(data, camera="cam")
    rgb = r.render()
    r.close()
    return rgb


if __name__ == "__main__":
    import imageio.v2 as imageio
    model, data = build({"red": (0.30, 0.06), "blue": (0.30, -0.06), "green": (0.22, 0.20)})
    imageio.imwrite(os.path.join(HERE, "world_home.png"), render(model, data))
    # 让物理跑 2 秒：机械臂应该稳稳停在 home，方块应该稳稳停在桌上
    q0 = data.qpos[joint_addr(model)].copy()
    cube0 = data.body("red_cube").xpos.copy()
    for _ in range(1000):
        mujoco.mj_step(model, data)
    drift = np.abs(data.qpos[joint_addr(model)] - q0).max()
    print(f"物理跑 2 秒后：关节最大漂移 {np.rad2deg(drift):.3f}°，红色方块移动 "
          f"{np.linalg.norm(data.body('red_cube').xpos - cube0) * 1000:.2f} mm")
    print(f"末端 Link6 位置 {np.round(data.body('Link6').xpos, 4)}（home 应为 [-0.0853 0.3473 0.3268]）")
    imageio.imwrite(os.path.join(HERE, "world_after_2s.png"), render(model, data))
