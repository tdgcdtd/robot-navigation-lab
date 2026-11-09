"""微调数据：仿真自动标注。

每个样本 = 一张图 + 一句话（"yellow cube"）+ 这句话对应方块的真值框。
标注来自仿真器的分割图（scene.gt_box），一张都不用人工画。
种子从 0 开始；测试集从 10000 开始，互不重叠。
"""
import numpy as np
from scene import render, gt_box, W, H
from scenarios import SCENARIOS

NAMES = list(SCENARIOS)


def make_sample(seed, scenario=None, target=None):
    """scenario / target 不给就随机；给了就固定场景类型 / 要找的颜色（用来给少见颜色补样本）。"""
    rng = np.random.default_rng(seed)
    name = scenario or NAMES[seed % len(NAMES)]     # 5 类场景轮流，数量均衡
    sc = SCENARIOS[name](rng)
    rgb, _, seg, _ = render(**sc)
    visible = [i for i in range(len(sc["cubes"])) if (seg == i).sum() >= 50]
    i = int(rng.choice(visible))                    # 随机挑一个看得见的方块当"要找的目标"
    if target is not None and any(sc["cubes"][j].color == target for j in visible):
        i = next(j for j in visible if sc["cubes"][j].color == target)
    x0, y0, x1, y1 = gt_box(seg, i)
    # DETR 系列模型的框格式：(中心 x, 中心 y, 宽, 高)，都除以图像宽高归一化到 0~1
    box = np.array([(x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H], dtype=np.float32)
    return dict(rgb=rgb, text=f"{sc['cubes'][i].color} cube.", box=box, scenario=name)


# ── 负样本："问的东西不在画面里"，正确答案是"没有框" ──
# 只有正样本的话，模型从没见过"答案是空"的题，就学会了"永远交一个框"。
COLORS = ["red", "blue", "green", "yellow", "orange", "purple"]
TRAIN_OBJECTS = ["banana", "mug", "bottle", "phone", "book", "apple", "scissors", "toy car"]
TEST_OBJECTS = ["tennis ball", "pen", "keyboard", "spoon", "remote control", "hammer"]   # 训练里从没出现过


def absent_query(rng, cubes, objects, p_color=0.7):
    """挑一个画面里不存在的东西来问：多数是"不存在的颜色 + cube"，少数是无关物体。"""
    absent = [c for c in COLORS if c not in {cb.color for cb in cubes}]
    if absent and rng.random() < p_color:
        return f"{rng.choice(absent)} cube."
    return f"{rng.choice(objects)}."


def make_negative(seed):
    rng = np.random.default_rng(seed)
    name = NAMES[seed % len(NAMES)]
    sc = SCENARIOS[name](rng)
    rgb, _, _, _ = render(**sc)
    return dict(rgb=rgb, text=absent_query(rng, sc["cubes"], TRAIN_OBJECTS), box=None, scenario=name)
