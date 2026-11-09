"""MuJoCo 桌面场景：随机摆几个彩色方块，渲染 彩色图 / 深度图 / 分割图，并给出真值。

为什么用仿真：方块的真实位置、每个像素属于谁，仿真器全都知道
→ 1) 能精确算出感知误差（毫米）  2) 微调时的标注是免费的。

坐标约定：世界坐标系原点在桌面中心，z 轴朝上，桌面高度 z = 0，单位米。
"""
from dataclasses import dataclass
import mujoco
import numpy as np

COLORS = {  # 名字 → RGBA
    "red": (0.85, 0.1, 0.1, 1), "blue": (0.1, 0.25, 0.85, 1),
    "green": (0.1, 0.65, 0.2, 1), "yellow": (0.95, 0.85, 0.1, 1),
    # 相近颜色：用来测模型分不分得清
    "orange": (0.95, 0.45, 0.1, 1), "purple": (0.45, 0.15, 0.75, 1),
}
CUBE = 0.04          # 方块边长 4 cm
W, H = 640, 480      # 图像分辨率
CAM_POS = (0.0, -0.55, 0.55)   # 相机在桌子前上方，斜着往下看桌面中心
CAM_TARGET = (0.0, 0.0, 0.0)


@dataclass
class Cube:
    color: str
    pos: np.ndarray  # 方块中心（世界坐标，米）


def _camera_axes(pos, target):
    """MuJoCo 相机看向自己的 -z 方向；返回 xyaxes（相机 x 轴、y 轴在世界坐标里的方向）。"""
    forward = np.subtract(target, pos); forward /= np.linalg.norm(forward)
    x = np.cross(forward, [0, 0, 1]); x /= np.linalg.norm(x)   # 相机 x 朝右
    y = np.cross(x, forward)                                     # 相机 y 朝上
    return x, y


def build_xml(cubes, light=1.0, occluders=()):
    """light：亮度系数（1 = 正常，0.25 = 很暗）；occluders：灰色挡板的 (中心, 半尺寸) 列表。"""
    x, y = _camera_axes(CAM_POS, CAM_TARGET)
    bodies = "\n".join(
        f'<body name="cube{i}" pos="{c.pos[0]:.4f} {c.pos[1]:.4f} {c.pos[2]:.4f}">'
        f'<geom name="cube{i}" type="box" size="{CUBE/2} {CUBE/2} {CUBE/2}" rgba="{" ".join(map(str, COLORS[c.color]))}"/></body>'
        for i, c in enumerate(cubes))
    bodies += "\n".join(
        f'<geom name="occ{i}" type="box" pos="{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" size="{s[0]} {s[1]} {s[2]}" rgba="0.55 0.55 0.58 1"/>'
        for i, (p, s) in enumerate(occluders))
    a, d = 0.35 * light, 0.5 * light
    return f"""
<mujoco>
  <visual><global offwidth="{W}" offheight="{H}"/><headlight ambient="{a} {a} {a}" diffuse="{d} {d} {d}"/></visual>
  <asset>
    <texture name="wood" type="2d" builtin="flat" rgb1="0.62 0.47 0.33" width="64" height="64" mark="random" markrgb="0.55 0.4 0.28" random="0.3"/>
    <material name="wood" texture="wood" texrepeat="4 4"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 1.2" dir="-0.2 0.2 -1" diffuse="{0.6*light} {0.6*light} {0.6*light}"/>
    <geom name="table" type="box" pos="0 0 -0.02" size="0.4 0.3 0.02" material="wood"/>
    <geom name="floor" type="plane" pos="0 0 -0.75" size="2 2 0.1" rgba="0.3 0.3 0.32 1"/>
    {bodies}
    <camera name="cam" pos="{' '.join(map(str, CAM_POS))}" xyaxes="{x[0]:.5f} {x[1]:.5f} {x[2]:.5f} {y[0]:.5f} {y[1]:.5f} {y[2]:.5f}" fovy="50"/>
  </worldbody>
</mujoco>"""


def random_cubes(rng, colors):
    """在桌面中间区域随机摆放，方块之间至少隔 6 cm（不重叠）。"""
    cubes = []
    while len(cubes) < len(colors):
        p = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.15, 0.15), CUBE / 2])
        if all(np.linalg.norm(p[:2] - c.pos[:2]) > 0.06 for c in cubes):
            cubes.append(Cube(colors[len(cubes)], p))
    return cubes


def render(cubes, light=1.0, occluders=()):
    """返回 rgb (H,W,3) uint8、depth (H,W) 米、seg (H,W) 每像素属于第几个方块（-1=背景）、相机参数。"""
    model = mujoco.MjModel.from_xml_string(build_xml(cubes, light, occluders))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, H, W)
    r.update_scene(data, camera="cam")
    rgb = r.render()
    r.enable_depth_rendering(); depth = r.render(); r.disable_depth_rendering()
    r.enable_segmentation_rendering(); seg_raw = r.render(); r.disable_segmentation_rendering()
    r.close()

    # 分割图里存的是 geom 编号 → 换成"第几个方块"
    seg = np.full((H, W), -1)
    for i in range(len(cubes)):
        seg[seg_raw[..., 0] == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"cube{i}")] = i

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")
    f = (H / 2) / np.tan(np.deg2rad(model.cam_fovy[cam_id]) / 2)   # 焦距（像素）
    camera = {"f": f, "cx": W / 2, "cy": H / 2,
              "pos": data.cam_xpos[cam_id].copy(), "R": data.cam_xmat[cam_id].reshape(3, 3).copy()}
    return rgb, depth, seg, camera


def pixel_to_world(u, v, z, camera):
    """反投影：像素 (u,v) + 深度 z（沿相机视线方向的距离）→ 世界坐标。

    1. 像素 → 相机坐标：针孔模型 u = cx + f·x/z  ⇒  x = (u-cx)·z/f
       图像 v 朝下、相机 y 朝上，所以 y 取负；相机看向 -z，所以 z 取负
    2. 相机坐标 → 世界坐标：p_world = R · p_cam + 相机位置（和 P2 的齐次变换是同一回事）
    """
    p_cam = np.stack([(u - camera["cx"]) * z / camera["f"],
                      -(v - camera["cy"]) * z / camera["f"],
                      -z], axis=-1)
    return p_cam @ camera["R"].T + camera["pos"]


def gt_box(seg, i):
    """真值框 (x0, y0, x1, y1)：第 i 个方块所有像素的外接矩形。微调时的标注就是它。"""
    ys, xs = np.nonzero(seg == i)
    return (xs.min(), ys.min(), xs.max() + 1, ys.max() + 1) if len(xs) else None


if __name__ == "__main__":
    import imageio.v2 as imageio
    rng = np.random.default_rng(0)
    cubes = random_cubes(rng, ["red", "blue", "green", "yellow"])
    rgb, depth, seg, cam = render(cubes)
    imageio.imwrite("scene_rgb.png", rgb)
    # 自检：用真值分割 + 深度反投影，看能不能还原方块位置（验证相机模型写对了）
    for i, c in enumerate(cubes):
        vs, us = np.nonzero(seg == i)
        pts = pixel_to_world(us, vs, depth[vs, us], cam)
        top = pts[pts[:, 2] > pts[:, 2].max() - 0.003]           # 顶面的点
        print(f"{c.color:6s} 真值中心 {np.round(c.pos, 3)}  顶面中心 {np.round(top.mean(0), 3)}"
              f"  框 {gt_box(seg, i)}")
