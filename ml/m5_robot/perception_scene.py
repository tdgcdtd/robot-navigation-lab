"""新世界里的随机场景 + 真值，用来测（以及之后重新训练）M3、M4 的感知模型。

和 M3 / M4 的场景比，变了什么：
- 画面里多了机械臂（home 姿态，占画面下方）
- 相机换成"眼睛"相机，坐标原点从桌面中心换成机械臂底座
- position 的定义：按机械臂的左右（y > 7 cm 是 left，y < -7 cm 是 right），和 world.yaml 的约定一致

场景内容和 M4 一样：2~5 个不同颜色的方块，约 1/3 叠在别的方块上，1/3 偏暗。
方块只放在机械臂够得到的范围：离底座 18~40 cm。
"""
import json, os, sys
from dataclasses import dataclass
import numpy as np
import world

MIN_PIXELS = 150
W, H, CUBE = world.W, world.H, world.CUBE


@dataclass
class Cube:
    color: str
    pos: np.ndarray


def random_layout(rng):
    n = int(rng.integers(2, 6))
    colors = list(rng.permutation(list(world.COLORS)))[:n]
    cubes, below = [], []
    for c in colors:
        bases = [i for i in range(len(cubes)) if i not in below and cubes[i].pos[2] < 2 * CUBE]   # 最多叠 3 层（和 M4 的场景规则一样）
        if bases and rng.random() < 0.35:
            b = int(rng.choice(bases))
            off = rng.uniform(-0.004, 0.004, 2)
            cubes.append(Cube(c, np.array([*(cubes[b].pos[:2] + off), cubes[b].pos[2] + CUBE]))); below.append(b)
            continue
        while True:
            p = np.array([rng.uniform(0.14, 0.38), rng.uniform(-0.24, 0.24), CUBE / 2 - 0.001])
            if 0.18 <= np.hypot(p[0], p[1]) <= 0.40 and all(np.linalg.norm(p[:2] - q.pos[:2]) > 0.06 for q in cubes):
                break
        cubes.append(Cube(c, p)); below.append(-1)
    light = float(rng.choice([1.0, 1.0, rng.uniform(0.3, 0.6)]))
    return cubes, below, light


def answer(cubes, below, seg):
    objs = []
    for i, c in enumerate(cubes):
        vs, us = np.nonzero(seg == i)
        objs.append(dict(color=c.color, shape="cube",
                         position="left" if c.pos[1] > 0.07 else "right" if c.pos[1] < -0.07 else "center",
                         point=[int(round(1000 * us.mean() / W)), int(round(1000 * vs.mean() / H))],
                         on="table" if below[i] < 0 else cubes[below[i]].color))
    objs.sort(key=lambda o: o["point"][0])
    return {"objects": objs}


def make_scene(seed):
    """同一个种子 → 同一个场景。有方块被挡得看不清（比如被机械臂挡住）就重新生成。"""
    rng = np.random.default_rng(seed)
    while True:
        cubes, below, light = random_layout(rng)
        model, data = world.build({c.color: tuple(c.pos) for c in cubes}, light=light)
        rgb, depth, seg, cam, names = world.render_all(model, data)
        assert names == [c.color + "_cube" for c in cubes]
        if all((seg == i).sum() >= MIN_PIXELS for i in range(len(cubes))):
            return dict(rgb=rgb, depth=depth, seg=seg, cam=cam, cubes=cubes, below=below, light=light,
                        answer=answer(cubes, below, seg))


if __name__ == "__main__":
    import imageio.v2 as imageio
    tiles = []
    for s in range(10_000, 10_006):
        sc = make_scene(s)
        print(s, f"亮度 {sc['light']:.2f}", json.dumps(sc["answer"], ensure_ascii=False))
        tiles.append(sc["rgb"])
    imageio.imwrite(os.path.join(world.HERE, "perception_scenes.png"),
                    np.concatenate([np.concatenate(tiles[:3], 1), np.concatenate(tiles[3:], 1)], 0))
