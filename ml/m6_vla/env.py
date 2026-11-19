"""M6 VLA 仿真环境：复用 M5 的 E6 + 吸盘 + eye 相机，再在末端 Link6 上加一台腕部相机（wrist）。

为什么加 wrist 相机：
  SmolVLA / π0 等主流 VLA 模型通常同时看“全局第三视角（eye）+ 随手移动的腕部视角（wrist）”。
  eye 负责看清方块和目标区域（left_zone / right_zone）的全局位置；
  wrist 在末端靠近方块时能从正上方把方块放大看清，抓取对准精度远高于只用固定相机。
"""
import os
import sys
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "m5_robot"))
import world  # noqa: E402

FPS = 10
STEPS_PER_FRAME = 50  # 50 * 0.002 s = 0.1 s -> 10 Hz
IMG_SIZE = 256
ZONES = {"left_zone": (0.22, 0.20), "right_zone": (0.22, -0.20)}
TASKS = [
    ("red", "left_zone", "Pick up the red cube and place it in the left zone."),
    ("red", "right_zone", "Pick up the red cube and place it in the right zone."),
    ("blue", "left_zone", "Pick up the blue cube and place it in the left zone."),
    ("blue", "right_zone", "Pick up the blue cube and place it in the right zone."),
]


def build_env_world(cubes, light=1.0):
    """按 M5 world.py 完全一致的物理与相机搭建世界，支持灯光亮度调节并挂载腕部相机 wrist。"""
    spec = mujoco.MjSpec.from_file(world.E6_XML)
    mesh_dir = os.path.join(os.path.dirname(world.E6_XML), "dobot_rviz", "meshes", "me6")
    if os.path.isdir(mesh_dir):
        spec.meshdir = mesh_dir
    spec.option.timestep = 0.002
    spec.visual.global_.offwidth, spec.visual.global_.offheight = world.W, world.H
    wb = spec.worldbody
    spec.visual.headlight.ambient = [0.35 * light] * 3
    spec.visual.headlight.diffuse = [0.5 * light] * 3
    wb.add_light(pos=[0.5, -0.3, 1.5], dir=[-0.2, 0.2, -1], diffuse=[0.6 * light] * 3)
    wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, -0.011], size=[0.4, 0.4, 0.01],
                rgba=[0.62, 0.47, 0.33, 1])
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[0, 0, -0.75], size=[3, 3, 0.1],
                rgba=[0.3, 0.3, 0.32, 1])
    for color, p in cubes.items():
        z = p[2] if len(p) > 2 else world.CUBE / 2
        b = wb.add_body(name=f"{color}_cube", pos=[p[0], p[1], z])
        b.add_freejoint()
        b.add_geom(name=f"{color}_cube", type=mujoco.mjtGeom.mjGEOM_BOX, size=[world.CUBE / 2] * 3,
                   rgba=world.COLORS[color], mass=0.03, friction=[1.0, 0.005, 0.0001])

    # 吸盘杆 + 吸盘面 + 腕部相机（装在 Link6 旁，沿着 Link6 +z 方向朝正下方看吸盘和方块）
    tool = spec.body("Link6")
    tool.add_geom(name="tool_rod", type=mujoco.mjtGeom.mjGEOM_CYLINDER, fromto=[0, 0, 0, 0, 0, world.TOOL_LEN - 0.005],
                  size=[0.006, 0, 0], rgba=[0.2, 0.2, 0.22, 1], mass=0.02)
    tool.add_geom(name="tool_pad", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  fromto=[0, 0, world.TOOL_LEN - 0.005, 0, 0, world.TOOL_LEN],
                  size=[0.012, 0, 0], rgba=[0.9, 0.5, 0.1, 1], mass=0.005)
    tool.add_site(name="suction", pos=[0, 0, world.TOOL_LEN], size=[0.003, 0.003, 0.003])
    # 腕部相机：装在 Link6 +Y 侧法兰外沿 (0, 0.055, 0.022)，避开 Link5/Link6 外壳，斜向内瞄准吸盘与方块 (0, 0, 0.060)
    w_pos, w_tgt = np.array([0.0, 0.055, 0.022]), np.array([0.0, 0.0, 0.060])
    w_fwd = w_tgt - w_pos; w_fwd /= np.linalg.norm(w_fwd)
    w_x = np.array([1.0, 0.0, 0.0])
    w_y = np.cross(-w_fwd, w_x); w_y /= np.linalg.norm(w_y)
    tool.add_camera(name="wrist", pos=w_pos.tolist(), xyaxes=[*w_x, *w_y], fovy=80)

    for color in cubes:
        spec.add_equality(name=f"hold_{color}", type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                          name1="Link6", name2=f"{color}_cube", active=False)

    world._add_camera(wb, "cam", world.CAM_POS, world.CAM_TARGET, 50)
    world._add_camera(wb, "eye", world.EYE_POS, world.EYE_TARGET, world.EYE_FOVY)

    for j in world.JOINTS:
        spec.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[2000] + [0] * 9, biasprm=[0, -2000, -60] + [0] * 7)
    for name in ["base_link"] + [f"Link{i}" for i in range(1, 7)]:
        spec.body(name).gravcomp = 1.0

    model = spec.compile()
    world._stabilize(model)
    data = mujoco.MjData(model)
    world.set_joints(model, data, world.HOME)
    return model, data


class E6PickEnv:
    """10 Hz 控制步长的 E6 抓取环境，供数据采集与本地 VLA 闭环评估共用。"""

    def __init__(self, img_size=IMG_SIZE):
        self.img_size = img_size
        self.model = None
        self.data = None
        self.adr = None
        self.suction_active = False
        self.held = None
        self.target_color = "red"
        self.target_zone = "left_zone"
        self.light = 1.0
        self._renderer = None

    def reset(self, cubes, target_color, target_zone, light=1.0):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self.light = float(light)
        self.model, self.data = build_env_world(cubes, light=self.light)
        self.adr = world.joint_addr(self.model)
        self.target_color = target_color
        self.target_zone = target_zone
        self.suction_active = False
        self.held = None
        for _ in range(int(0.3 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self._renderer = mujoco.Renderer(self.model, self.img_size, self.img_size)
        return self.get_state()

    def get_state(self):
        q = self.data.qpos[self.adr].astype(np.float32)
        s = np.float32(1.0 if self.suction_active else 0.0)
        return np.append(q, s)

    def render_camera(self, name):
        """渲染指定名称的相机画面，返回 (H, W, 3) uint8 RGB 数组。"""
        self._renderer.update_scene(self.data, camera=name)
        return self._renderer.render().copy()

    def render_cameras(self):
        return self.render_camera("eye"), self.render_camera("wrist")

    def step(self, action_7d):
        action = np.asarray(action_7d, dtype=float)
        q_start = self.data.ctrl[:6].copy()
        q_goal = action[:6]
        for k in range(1, STEPS_PER_FRAME + 1):
            a = k / STEPS_PER_FRAME
            self.data.ctrl[:6] = q_start + a * (q_goal - q_start)
            mujoco.mj_step(self.model, self.data)

        want_suction = bool(action[6] > 0.5)
        if want_suction and not self.suction_active:
            self.held, _ = world.suction_on(self.model, self.data)
            self.suction_active = True
        elif not want_suction and self.suction_active:
            world.suction_off(self.model, self.data)
            self.suction_active = False
            self.held = None
        return self.get_state()

    def is_success(self):
        pos = self.data.body(f"{self.target_color}_cube").xpos
        tx, ty = ZONES[self.target_zone]
        err = float(np.hypot(pos[0] - tx, pos[1] - ty))
        ok = (not self.suction_active) and (pos[2] < 0.032) and (err < 0.020)
        return ok, err * 1000.0

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
