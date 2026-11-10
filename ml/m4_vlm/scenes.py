"""M4 场景：桌上 2~5 个不同颜色的方块，有的叠在别的方块上；自动生成"标准答案" JSON。

和 M3 共用渲染（../m3_perception/scene.py），区别只是：方块可以叠起来（z 抬高一个方块的高度）。
标准答案里每个物体：
- color / shape：颜色、形状（目前全是 cube，留着字段方便以后加球和圆柱）
- position：在桌面的 left / center / right（方块放在 x ∈ ±20 cm，按 ±7 cm 分三段；相机正对桌面，桌面左 = 画面左）——给 LLM 规划时看的粗略位置
- point：[x, y]，0~1000 归一化的图像坐标（Qwen3-VL 自己的坐标习惯）——只用来和 DINO 的框对号入座，不用来抓取
- on："table" 或下面那个方块的颜色
物体按 point 的 x 从左到右排好，答案顺序固定，模型才好学、我们也好打分。
"""
import json, os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "m3_perception"))
from scene import COLORS, CUBE, W, H, Cube, render   # noqa: E402

MIN_PIXELS = 150   # 看得见的像素少于这个数的方块，没法要求模型认出来 → 整个场景重新生成


def random_scene(rng):
    """返回 (cubes, below, light)。below[i] = 第 i 个方块下面压着的方块编号（-1 = 直接放桌上）。"""
    n = int(rng.integers(2, 6))
    colors = list(rng.permutation(list(COLORS)))[:n]
    cubes, below = [], []
    for c in colors:
        bases = [i for i in range(len(cubes)) if i not in below and cubes[i].pos[2] < 2 * CUBE]  # 最多叠 2 层
        if bases and rng.random() < 0.35:                  # 约 1/3 的方块叠到别人上面
            b = int(rng.choice(bases))
            off = rng.uniform(-0.006, 0.006, 2)            # 叠得不是严丝合缝，更像真的
            pos = np.array([*(cubes[b].pos[:2] + off), cubes[b].pos[2] + CUBE])
            cubes.append(Cube(c, pos)); below.append(b)
            continue
        while True:                                       # 放桌上：和桌上已有的方块至少隔 6 cm
            p = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.15, 0.15), CUBE / 2])
            if all(np.linalg.norm(p[:2] - q.pos[:2]) > 0.06 for q in cubes):
                break
        cubes.append(Cube(c, p)); below.append(-1)
    light = float(rng.choice([1.0, 1.0, rng.uniform(0.3, 0.6)]))   # 1/3 的场景偏暗
    return cubes, below, light


def answer(cubes, below, seg):
    """标准答案：每个方块一条，按从左到右排序。"""
    objs = []
    for i, c in enumerate(cubes):
        vs, us = np.nonzero(seg == i)
        u, v = us.mean(), vs.mean()                        # 看得见的像素的中心
        objs.append(dict(color=c.color, shape="cube",
                         position="left" if c.pos[0] < -0.07 else "right" if c.pos[0] > 0.07 else "center",
                         point=[int(round(1000 * u / W)), int(round(1000 * v / H))],
                         on="table" if below[i] < 0 else cubes[below[i]].color))
    objs.sort(key=lambda o: o["point"][0])
    return {"objects": objs}


def make_scene(seed):
    """同一个种子 → 同一个场景。看不清的方块太多就换下一个随机数接着生成。"""
    rng = np.random.default_rng(seed)
    while True:
        cubes, below, light = random_scene(rng)
        rgb, depth, seg, cam = render(cubes, light)
        if all((seg == i).sum() >= MIN_PIXELS for i in range(len(cubes))):
            return dict(rgb=rgb, answer=answer(cubes, below, seg), cubes=cubes, light=light)


if __name__ == "__main__":
    import imageio.v2 as imageio
    tiles = []
    for s in range(6):
        sc = make_scene(s)
        print(s, json.dumps(sc["answer"], ensure_ascii=False))
        tiles.append(sc["rgb"])
    grid = np.concatenate([np.concatenate(tiles[:3], 1), np.concatenate(tiles[3:], 1)], 0)
    imageio.imwrite("scenes_grid.png", grid)
    print("已存 scenes_grid.png")
