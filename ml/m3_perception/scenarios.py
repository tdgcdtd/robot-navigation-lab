"""5 类场景生成器：零样本测量和微调造数据共用同一套。

每个生成器：输入随机数发生器 rng → 返回 dict(cubes, light, occluders)。
**测试集固定用种子 10000 起，训练数据用 0 起，两者永不重叠**——否则就是"拿考题当练习题"。
"""
import numpy as np
from scene import CUBE, Cube, random_cubes

BASE = ["red", "blue", "green", "yellow"]


def separated(rng):
    """普通：4 个基础颜色，分开摆放。"""
    return dict(cubes=random_cubes(rng, list(rng.permutation(BASE))), light=1.0, occluders=())


def touching(rng):
    """挨在一起：3~4 个方块排成 2×2 的小堆，缝隙 2~10 mm。"""
    n = int(rng.integers(3, 5))
    colors = list(rng.permutation(BASE))[:n]
    gap = CUBE + rng.uniform(0.002, 0.01)
    anchor = np.array([rng.uniform(-0.15, 0.1), rng.uniform(-0.1, 0.05)])
    slots = [anchor + gap * np.array(o) for o in [(0, 0), (1, 0), (0, 1), (1, 1)]]
    order = rng.permutation(4)[:n]
    cubes = [Cube(c, np.array([*slots[k], CUBE / 2])) for c, k in zip(colors, order)]
    return dict(cubes=cubes, light=1.0, occluders=())


def similar(rng):
    """相近颜色：红和橙、蓝和紫同时出现，再加一个其他颜色。"""
    colors = ["red", "orange", "blue", "purple", str(rng.choice(["green", "yellow"]))]
    return dict(cubes=random_cubes(rng, list(rng.permutation(colors))), light=1.0, occluders=())


def dim(rng):
    """暗光：亮度只有正常的 15%~30%。"""
    return dict(cubes=random_cubes(rng, list(rng.permutation(BASE))),
                light=float(rng.uniform(0.15, 0.3)), occluders=())


def occluded(rng):
    """遮挡：每个方块前面（靠相机一侧）立一块灰色挡板，挡住一部分（约 20%~70%）。"""
    cubes = random_cubes(rng, list(rng.permutation(BASE))[:3])
    occ = []
    for c in cubes:
        h = rng.uniform(0.045, 0.06)                  # 挡板高 4.5~6 cm（方块 4 cm），斜着看过去能挡住一部分
        center = c.pos + np.array([rng.uniform(-0.015, 0.015), -0.032, 0])
        center[2] = h / 2
        occ.append((center, (0.03, 0.004, h / 2)))
    return dict(cubes=cubes, light=1.0, occluders=occ)


SCENARIOS = {"分开": separated, "挨在一起": touching, "相近颜色": similar, "暗光": dim, "遮挡": occluded}
TEST_SEED0 = 10_000
