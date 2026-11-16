"""感知重测：M3（DINO + SAM + 定位）和 M4（VLM 场景描述）在 M5 的新画面上还剩多少准确率。

新画面和训练时的区别：多了机械臂、换了相机位置、position 的定义换成"机械臂的左右"。
模型一个字都没改，直接拿来考——这叫"分布外测试"：考题和练习题不是一个出题人。

用法（两个模型分开跑，内存装不下一起）：
    ../.venv/bin/python eval_perception.py --model dino [--dino ckpt路径] [--tag 名字]
    ../.venv/bin/python eval_perception.py --model vlm  [--adapter adapter路径] [--tag 名字]
测试场景：种子 10000 起，100 个（和训练用的种子不重叠）。
"""
import argparse, json, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
M3, M4 = os.path.join(HERE, "..", "m3_perception"), os.path.join(HERE, "..", "m4_vlm")
sys.path[:0] = [M3, M4]
from perception_scene import make_scene, W, H, CUBE   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--model", choices=["dino", "vlm"], required=True)
ap.add_argument("--dino", default=os.path.join(M3, "ckpt", "ft_neg"))
ap.add_argument("--adapter", default=os.path.join(M4, "adapters", "lora"))
ap.add_argument("--tag", default=None)
ap.add_argument("--n", type=int, default=100)
args = ap.parse_args()
tag = args.tag or f"{args.model}_old"
TEST_SEED0 = 10_000
COLORS = ["red", "blue", "green", "yellow", "orange", "purple"]


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def gt_box(seg, i):
    ys, xs = np.nonzero(seg == i)
    return (xs.min(), ys.min(), xs.max() + 1, ys.max() + 1)


def run_dino():
    from perceive import Perceiver, locate
    p = Perceiver(dino_path=args.dino, edge=480)
    recs, t0 = [], time.time()
    for k in range(args.n):
        sc = make_scene(TEST_SEED0 + k)
        covered = set(sc["below"]) - {-1}                       # 上面压着别的方块的（顶面看不见）
        for i, c in enumerate(sc["cubes"]):
            dets = p.detect(sc["rgb"], f"{c.color} cube")
            r = dict(kind="present", seed=TEST_SEED0 + k, color=c.color, dim=sc["light"] < 1, covered=i in covered,
                     stacked=sc["below"][i] >= 0, score=0.0, iou=0.0, err_mm=None)
            if dets:
                score, box = dets[0]
                est = locate(p.segment(sc["rgb"], box), sc["depth"], sc["cam"])
                r.update(score=score, iou=float(iou(box, gt_box(sc["seg"], i))),
                         err_mm=None if est is None else float(np.linalg.norm(est - c.pos) * 1000))
            recs.append(r)
        absent = [c for c in COLORS if c not in {cb.color for cb in sc["cubes"]}]
        if absent:                                              # 每个场景问一个不存在的颜色：应该不给框
            q = absent[k % len(absent)]
            dets = p.detect(sc["rgb"], f"{q} cube", threshold=0.05)
            recs.append(dict(kind="absent", seed=TEST_SEED0 + k, color=q, score=dets[0][0] if dets else 0.0))
        print(f"\r{k + 1}/{args.n}  已用 {time.time() - t0:.0f} 秒", end="", flush=True)
    print()
    pres = [r for r in recs if r["kind"] == "present"]
    ab = [r for r in recs if r["kind"] == "absent"]

    def show(title, rs):
        if rs:
            det = sum(r["iou"] >= 0.5 for r in rs) / len(rs)
            loc = sum(r["err_mm"] is not None and r["err_mm"] < 10 for r in rs) / len(rs)
            print(f"  {title:14s} {len(rs):4d} 次   检测对 {det:5.0%}   3D 误差 < 10 mm {loc:5.0%}")
    print(f"\n【M3：DINO + SAM + 定位】{args.n} 个场景")
    show("全部", pres)
    show("顶面露着的", [r for r in pres if not r["covered"]])
    show("被压在下面的", [r for r in pres if r["covered"]])
    show("偏暗的场景", [r for r in pres if r["dim"]])
    print(f"  拒绝不存在的颜色（阈值 0.3）：{sum(r['score'] < 0.3 for r in ab)}/{len(ab)}")
    return recs


def run_vlm():
    from vlm import VLM
    v = VLM(adapter=args.adapter)
    recs, t0 = [], time.time()
    for k in range(args.n):
        sc = make_scene(TEST_SEED0 + k)
        text, pred = v.describe(sc["rgb"])
        g = {o["color"]: o for o in sc["answer"]["objects"]}
        p = {o["color"]: o for o in (pred or {}).get("objects", []) if isinstance(o, dict) and o.get("color")}
        r = dict(seed=TEST_SEED0 + k, parsed=pred is not None, dim=sc["light"] < 1, n_gt=len(g),
                 missed=[c for c in g if c not in p], extra=[c for c in p if c not in g],
                 pos_ok=0, on_ok=0, n_stacked=sum(o["on"] != "table" for o in g.values()), stacked_ok=0,
                 point_err=[], output=text, answer=sc["answer"])
        for c in set(g) & set(p):
            r["pos_ok"] += p[c].get("position") == g[c]["position"]
            ok = p[c].get("on") == g[c]["on"]
            r["on_ok"] += ok
            r["stacked_ok"] += ok and g[c]["on"] != "table"
            pt = p[c].get("point")
            if isinstance(pt, list) and len(pt) == 2 and all(isinstance(x, (int, float)) for x in pt):
                r["point_err"].append(float(np.hypot((pt[0] - g[c]["point"][0]) * W / 1000, (pt[1] - g[c]["point"][1]) * H / 1000)))
        n = len(set(g) & set(p))
        r["all_ok"] = bool(pred) and not r["missed"] and not r["extra"] and r["pos_ok"] == n and r["on_ok"] == n
        r["content_ok"] = bool(pred) and not r["missed"] and not r["extra"] and r["on_ok"] == n     # 不算 position
        recs.append(r)
        print(f"\r{k + 1}/{args.n}  已用 {time.time() - t0:.0f} 秒", end="", flush=True)
    print()
    n_obj = sum(r["n_gt"] for r in recs); n_match = n_obj - sum(len(r["missed"]) for r in recs)
    errs = [e for r in recs for e in r["point_err"]]; n_st = sum(r["n_stacked"] for r in recs)
    print(f"\n【M4：VLM 场景描述】{args.n} 个场景，{n_obj} 个方块（叠在别人上面的 {n_st} 个）")
    print(f"  能解析                 {sum(r['parsed'] for r in recs) / len(recs):5.0%}")
    print(f"  场景全对               {sum(r['all_ok'] for r in recs) / len(recs):5.0%}")
    print(f"  场景全对（不算 position）{sum(r['content_ok'] for r in recs) / len(recs):5.0%}")
    print(f"  漏报 {1 - n_match / n_obj:.0%}，多报 {sum(len(r['extra']) for r in recs)} 个")
    if n_match:
        print(f"  position 对            {sum(r['pos_ok'] for r in recs) / n_match:5.0%}")
        print(f"  on 对                  {sum(r['on_ok'] for r in recs) / n_match:5.0%}   叠放的那些 {sum(r['stacked_ok'] for r in recs)}/{n_st}")
    if errs:
        print(f"  point 误差             中位数 {np.median(errs):.0f} 像素，90% 分位 {np.percentile(errs, 90):.0f} 像素")
    return recs


records = run_dino() if args.model == "dino" else run_vlm()
os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
with open(os.path.join(HERE, "results", f"{tag}.json"), "w") as f:
    json.dump(records, f, ensure_ascii=False, indent=1)
print(f"结果已存 results/{tag}.json")
