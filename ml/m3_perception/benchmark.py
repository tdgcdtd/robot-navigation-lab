"""测量：在固定测试集（5 类场景 × 每类 20 个）上，检测 + 3D 定位的成功率。

用法：
    ../.venv/bin/python benchmark.py                          # 零样本（官方原版 DINO）
    ../.venv/bin/python benchmark.py --edge 480 --tag zeroshot480        # 480 分辨率零样本
    ../.venv/bin/python benchmark.py --edge 480 --dino ckpt/final --tag ft # 微调后的模型

判定标准（每一次"找某个颜色的方块"算一次查询）：
- 检测对：置信度最高的框和真值框的重合度（IoU）≥ 0.5
- 3D 对：算出的方块中心和真值相差 < 10 mm（分别用"框内像素"和"SAM 轮廓"各算一次）
结果存到 results/<tag>.json，微调后用同一个脚本、同一批场景再测，直接对比。
"""
import argparse, json, os, time
import numpy as np
from perceive import Perceiver, locate, DINO_ID
from scene import render, gt_box
from scenarios import SCENARIOS, TEST_SEED0
from benchmark_utils import iou

parser = argparse.ArgumentParser()
parser.add_argument("--dino", default=DINO_ID)
parser.add_argument("--tag", default="zeroshot")
parser.add_argument("--n", type=int, default=20, help="每类场景测几个")
parser.add_argument("--edge", type=int, default=800, help="DINO 输入短边像素数，要和训练时一致")
args = parser.parse_args()


p = Perceiver(dino_path=args.dino, edge=args.edge)
records, t_start = [], time.time()
for name, gen in SCENARIOS.items():
    for k in range(args.n):
        sc = gen(np.random.default_rng(TEST_SEED0 + k))
        rgb, depth, seg, cam = render(**sc)
        for i, c in enumerate(sc["cubes"]):
            g = gt_box(seg, i)
            if g is None or (seg == i).sum() < 50:     # 几乎看不见的不考
                continue
            dets = p.detect(rgb, f"{c.color} cube")
            rec = dict(scenario=name, seed=TEST_SEED0 + k, color=c.color, found=bool(dets),
                       iou=0.0, score=0.0, err_box=None, err_mask=None)
            if dets:
                score, box = dets[0]
                rec.update(score=score, iou=iou(box, g))
                x0, y0, x1, y1 = [int(round(v)) for v in box]
                bm = np.zeros(seg.shape, bool); bm[y0:y1, x0:x1] = True
                for key, m in (("err_box", bm), ("err_mask", p.segment(rgb, box))):
                    est = locate(m, depth, cam)
                    rec[key] = None if est is None else float(np.linalg.norm(est - c.pos) * 1000)
            records.append(rec)
        print(f"\r{name} {k + 1}/{args.n}  已用 {time.time() - t_start:.0f} 秒", end="", flush=True)
    print()

os.makedirs("results", exist_ok=True)
with open(f"results/{args.tag}.json", "w") as f:
    json.dump(records, f, ensure_ascii=False, indent=1)


def ok3d(r, key):
    return r[key] is not None and r[key] < 10


print(f"\n{'场景':6s} {'查询数':>5s} {'检测对':>7s} {'3D对(用框)':>10s} {'3D对(用轮廓)':>11s}")
for name in list(SCENARIOS) + ["全部"]:
    rs = [r for r in records if name == "全部" or r["scenario"] == name]
    n = len(rs)
    det = sum(r["iou"] >= 0.5 for r in rs) / n
    b = sum(ok3d(r, "err_box") for r in rs) / n
    m = sum(ok3d(r, "err_mask") for r in rs) / n
    print(f"{name:6s} {n:5d} {det:7.0%} {b:10.0%} {m:11.0%}")
print(f"\n结果已存 results/{args.tag}.json，总用时 {time.time() - t_start:.0f} 秒")
