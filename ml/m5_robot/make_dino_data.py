"""造 DINO 的微调数据：M5 新画面（有机械臂、"眼睛"相机）+ 一部分 M3 的旧数据（防止忘掉原来会的）。

格式和 M3 一样（../m3_perception/finetune.py 直接能读）：
    dino_data/{train,val}/labels.json：[{"image": "xxx.png", "text": "red cube.", "box": [cx, cy, w, h] 或 null}, ...]
    box 是 0~1 归一化的（中心 x, 中心 y, 宽, 高）；null = 负样本（问的东西画面里没有）

重测发现的问题（eval_perception.py）决定了数据怎么配：
- 红 / 橙分不清：橙色存在时 9/41 没认出来，红色不存在时 13/21 给了框
  → 正样本里一半的题专门问红或橙；负样本里优先问"有橙没红时问红""有红没橙时问橙"
- 被压在下面的方块：也出题（框 = 露出来的那部分），让它学会认出只露侧面的方块
种子：训练 0 起、验证 5000 起，测试是 10000 起（eval_perception.py），互不重叠。

用法：../.venv/bin/python make_dino_data.py [新画面训练数=1500] [验证数=100] [M3 旧数据混入数=300]
"""
import json, os, random, shutil, sys, time
import imageio.v2 as imageio
import numpy as np
from perception_scene import make_scene, W, H

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "dino_data")
OLD = os.path.join(HERE, "..", "m3_perception", "data", "train")
COLORS = ["red", "blue", "green", "yellow", "orange", "purple"]
RIVAL = {"red": "orange", "orange": "red"}
P_NEG = 0.25


def box_of(seg, i):
    ys, xs = np.nonzero(seg == i)
    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max() + 1, ys.max() + 1
    return [(x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H]


def make_item(seed):
    sc = make_scene(seed)
    rng = np.random.default_rng(seed + 777)
    present = [c.color for c in sc["cubes"]]
    absent = [c for c in COLORS if c not in present]
    if absent and rng.random() < P_NEG:
        tricky = [c for c in absent if RIVAL.get(c) in present]          # 有它的"像的颜色"，最容易认错
        q = str(rng.choice(tricky)) if tricky and rng.random() < 0.7 else str(rng.choice(absent))
        return sc["rgb"], dict(text=f"{q} cube.", box=None)
    redor = [i for i, c in enumerate(present) if c in RIVAL]
    i = int(rng.choice(redor)) if redor and rng.random() < 0.5 else int(rng.integers(len(present)))
    return sc["rgb"], dict(text=f"{present[i]} cube.", box=box_of(sc["seg"], i))


def dump(split, seeds, n_old=0):
    d = os.path.join(OUT, split)
    shutil.rmtree(d, ignore_errors=True); os.makedirs(d)
    labels = []
    for k, s in enumerate(seeds):
        rgb, item = make_item(s)
        name = f"m5_{s:05d}.png"
        imageio.imwrite(os.path.join(d, name), rgb)
        labels.append(dict(image=name, scenario="m5", **item))
        if (k + 1) % 250 == 0:
            print(f"{split}: {k + 1}/{len(seeds)}", flush=True)
    if n_old:
        old = json.load(open(os.path.join(OLD, "labels.json")))
        for it in random.Random(0).sample(old, n_old):
            name = "m3_" + it["image"]
            os.symlink(os.path.abspath(os.path.join(OLD, it["image"])), os.path.join(d, name))
            labels.append(dict(it, image=name))
    random.Random(1).shuffle(labels)
    json.dump(labels, open(os.path.join(d, "labels.json"), "w"), ensure_ascii=False)
    texts = [it["text"] for it in labels if it["scenario"] == "m5"]
    neg = sum(it["box"] is None for it in labels)
    print(f"{split}: 共 {len(labels)} 个（新画面 {len(texts)}，M3 旧数据 {n_old}；负样本 {neg}）"
          f"  新画面里问红 {texts.count('red cube.')}、问橙 {texts.count('orange cube.')}")


a = [int(x) for x in sys.argv[1:]] + [1500, 100, 300][len(sys.argv) - 1:]
t0 = time.time()
dump("train", range(a[0]), n_old=a[2])
dump("val", range(5000, 5000 + a[1]))
print(f"完成，用时 {time.time() - t0:.0f} 秒")
