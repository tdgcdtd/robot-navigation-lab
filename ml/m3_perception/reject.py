"""测量"拒绝不存在的物体"：问画面里没有的东西，模型应该一个框都不给。

和 benchmark.py 用同一批测试场景（种子 10000 起），每个场景问两次：
- 不存在的颜色："purple cube"（画面里没有紫色）——最难，因为有长得像的（蓝色）
- 无关物体："keyboard"——用的词训练里一个都没出现过，考的是"会说没有"而不是"背过这个词"

阈值是关键：模型对每个框都给置信度，我们只接受 ≥ 阈值的框。
- 阈值高 → 更容易拒绝不存在的，但也可能把答对的挡掉
- 阈值低 → 答对的都留下，但不存在的也会乱给框
所以把 benchmark 的正样本结果（results/<pos>.json）拿来，同一个阈值下一起看两边。

用法：
    ../.venv/bin/python reject.py --edge 480 --tag zeroshot480 --pos zeroshot480
    ../.venv/bin/python reject.py --edge 480 --dino ckpt/final --tag ft480 --pos ft480
"""
import argparse, json, time
import numpy as np
from perceive import Perceiver, DINO_ID
from scene import render
from scenarios import SCENARIOS, TEST_SEED0
from datagen import absent_query, TEST_OBJECTS

parser = argparse.ArgumentParser()
parser.add_argument("--dino", default=DINO_ID)
parser.add_argument("--tag", required=True, help="结果存到 results/<tag>_neg.json")
parser.add_argument("--pos", help="对应的正样本结果 results/<pos>.json，用来一起看阈值")
parser.add_argument("--n", type=int, default=20)
parser.add_argument("--edge", type=int, default=800)
args = parser.parse_args()

p = Perceiver(dino_path=args.dino, edge=args.edge)
records, t0 = [], time.time()
for s_idx, (name, gen) in enumerate(SCENARIOS.items()):
    for k in range(args.n):
        sc = gen(np.random.default_rng(TEST_SEED0 + k))       # 和 benchmark 一模一样的场景
        rgb, _, _, _ = render(**sc)
        q_rng = np.random.default_rng(20_000 + 100 * s_idx + k)  # 问什么也固定下来，每个模型考同一套题
        for kind, p_color in (("不存在的颜色", 1.0), ("无关物体", 0.0)):
            text = absent_query(q_rng, sc["cubes"], TEST_OBJECTS, p_color)
            dets = p.detect(rgb, text, threshold=0.05)           # 阈值放很低：把最高分记下来，事后再选阈值
            records.append(dict(scenario=name, seed=TEST_SEED0 + k, kind=kind, text=text,
                                score=dets[0][0] if dets else 0.0))
        print(f"\r{name} {k + 1}/{args.n}  已用 {time.time() - t0:.0f} 秒", end="", flush=True)
    print()

with open(f"results/{args.tag}_neg.json", "w") as f:
    json.dump(records, f, ensure_ascii=False, indent=1)

print(f"\n拒绝率（最高分 < 阈值 就算拒绝）")
print(f"{'阈值':>5s} {'不存在的颜色':>8s} {'无关物体':>6s}" + (f" {'正样本仍答对':>8s}" if args.pos else ""))
pos = json.load(open(f"results/{args.pos}.json")) if args.pos else None
for t in (0.3, 0.5, 0.6, 0.7, 0.8, 0.9):
    row = f"{t:5.1f}"
    for kind in ("不存在的颜色", "无关物体"):
        rs = [r for r in records if r["kind"] == kind]
        row += f" {sum(r['score'] < t for r in rs) / len(rs):12.0%}"
    if pos:
        row += f" {sum(r['iou'] >= 0.5 and r['score'] >= t for r in pos) / len(pos):12.0%}"
    print(row)

print("\n不存在的颜色，按问的颜色看平均最高分（越低越好）：")
for c in ("red", "blue", "green", "yellow", "orange", "purple"):
    rs = [r["score"] for r in records if r["text"] == f"{c} cube."]
    if rs:
        print(f"  {c:7s} {len(rs):3d} 次  平均 {np.mean(rs):.2f}")
print(f"\n结果已存 results/{args.tag}_neg.json，总用时 {time.time() - t0:.0f} 秒")
