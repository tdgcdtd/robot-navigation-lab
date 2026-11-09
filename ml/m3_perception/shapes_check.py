"""遗忘检查：训练里只有方块；把场景里的方块换成球和圆柱，看模型还认不认得"没训练过的形状和说法"。

尤其要防的是：加了负样本（"banana." → 没有框）之后，模型会不会学成"凡是不带 cube 的问题都说没有"。
每个场景：0 号 = 红球，1 号 = 蓝圆柱，2、3 号 = 绿/黄方块。

用法：../.venv/bin/python shapes_check.py ckpt/final ckpt/ft_neg
"""
import sys
import numpy as np
import scene
from perceive import Perceiver
from scenarios import TEST_SEED0
from benchmark_utils import iou

_orig = scene.build_xml


def shapes_xml(cubes, light=1.0, occluders=()):
    s = _orig(cubes, light, occluders)
    h = scene.CUBE / 2
    s = s.replace(f'name="cube0" type="box" size="{h} {h} {h}"', f'name="cube0" type="sphere" size="{h}"')
    s = s.replace(f'name="cube1" type="box" size="{h} {h} {h}"', f'name="cube1" type="cylinder" size="{h} {h}"')
    return s


scene.build_xml = shapes_xml
QUERIES = [("red ball", 0), ("red sphere", 0), ("the round object", 0), ("blue cylinder", 1), ("a can", 1),
           ("green cube", 2), ("red cube", None), ("blue cube", None)]   # None：画面里没有（红的是球、蓝的是圆柱）
N = 10
for path in sys.argv[1:]:
    p = Perceiver(dino_path=path, edge=480)
    hits = {q: [] for q, _ in QUERIES}
    for k in range(N):
        cubes = scene.random_cubes(np.random.default_rng(TEST_SEED0 + 500 + k), ["red", "blue", "green", "yellow"])
        rgb, _, seg, _ = scene.render(cubes)
        for q, i in QUERIES:
            d = p.detect(rgb, q, threshold=0.05)
            score, box = d[0] if d else (0.0, None)
            ok = iou(box, scene.gt_box(seg, i)) >= 0.5 if (i is not None and box) else False
            hits[q].append((score, ok))
    print(f"\n== {path}")
    for q, i in QUERIES:
        sc = np.mean([s for s, _ in hits[q]])
        extra = f"找对 {sum(o for _, o in hits[q])}/{N}" if i is not None else "（画面里没有）"
        print(f"  {q:18s} 平均最高分 {sc:.2f}  {extra}")
    del p
