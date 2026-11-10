"""测量：VLM 在固定测试集（种子 10000 起，100 个场景）上描述场景的准确率。

和 M3 一样，先测零样本、再微调、再用同一套题测一次，直接对比。

打分（按颜色把模型说的物体和标准答案配对，每个场景颜色不重复）：
- 能解析：输出是不是合法 JSON、格式对不对（程序能不能直接用）
- 漏报 / 多报：标准答案里有、模型没说 / 模型说了、标准答案里没有（"幻觉"）
- position、on：配对上的物体里，粗略位置、叠放关系对不对
- point：和真值差多少像素；能不能"对号入座"（离它最近的真值点是同一个方块）
- 全对：整个场景每一项都对

用法：
    ../.venv/bin/python evaluate.py --tag zeroshot
    ../.venv/bin/python evaluate.py --tag zeroshot8bit --quantize-like adapters/lora   # 8 bit 零样本（公平对比用）
    ../.venv/bin/python evaluate.py --tag lora --adapter adapters/lora
"""
import argparse, json, os, time
import numpy as np
from scenes import make_scene, W, H
from vlm import VLM

TEST_SEED0 = 10_000

parser = argparse.ArgumentParser()
parser.add_argument("--tag", required=True)
parser.add_argument("--adapter", default=None, help="LoRA 小矩阵的文件夹；不给 = 原版模型")
parser.add_argument("--quantize-like", default=None, help="不加 LoRA，只按这个 adapter 训练时的方式把原模型压成 8 bit")
parser.add_argument("--n", type=int, default=100)
args = parser.parse_args()


def score(pred, gt):
    """一个场景的打分。pred 是模型输出解析出的 dict（解析失败为 None）。"""
    g = {o["color"]: o for o in gt["objects"]}
    r = dict(parsed=pred is not None, n_gt=len(g), missed=list(g), extra=[], pos_ok=0, on_ok=0,
             point_err=[], point_match=0, n_stacked=sum(o["on"] != "table" for o in g.values()), stacked_ok=0)
    if pred is None:
        return r | dict(all_ok=False)
    p = {}
    for o in pred["objects"]:
        if isinstance(o, dict) and o.get("color"):
            p[o["color"]] = o
    r["missed"] = [c for c in g if c not in p]
    r["extra"] = [c for c in p if c not in g]
    for c in set(g) & set(p):
        go, po = g[c], p[c]
        r["pos_ok"] += po.get("position") == go["position"]
        on_ok = po.get("on") == go["on"]
        r["on_ok"] += on_ok
        if go["on"] != "table":
            r["stacked_ok"] += on_ok
        pt = po.get("point")
        if isinstance(pt, list) and len(pt) == 2 and all(isinstance(v, (int, float)) for v in pt):
            px = np.array([pt[0] * W / 1000, pt[1] * H / 1000])
            err = {cc: np.linalg.norm(px - np.array([o["point"][0] * W / 1000, o["point"][1] * H / 1000]))
                   for cc, o in g.items()}
            r["point_err"].append(float(err[c]))
            r["point_match"] += min(err, key=err.get) == c       # 离它最近的真值点是不是同一个方块
    n_match = len(set(g) & set(p))
    r["all_ok"] = (not r["missed"] and not r["extra"] and r["pos_ok"] == n_match
                   and r["on_ok"] == n_match and r["point_match"] == n_match)
    return r


v = VLM(adapter=args.adapter, quantize_like=args.quantize_like)
records, t0 = [], time.time()
for k in range(args.n):
    sc = make_scene(TEST_SEED0 + k)
    text, pred = v.describe(sc["rgb"])
    rec = score(pred, sc["answer"]) | dict(seed=TEST_SEED0 + k, dim=sc["light"] < 1, output=text, answer=sc["answer"])
    records.append(rec)
    print(f"\r{k + 1}/{args.n}  已用 {time.time() - t0:.0f} 秒", end="", flush=True)
print()

os.makedirs("results", exist_ok=True)
with open(f"results/{args.tag}.json", "w") as f:
    json.dump(records, f, ensure_ascii=False, indent=1)


def report(rs, title):
    if not rs:
        return
    n_obj = sum(r["n_gt"] for r in rs)
    n_match = n_obj - sum(len(r["missed"]) for r in rs)
    errs = [e for r in rs for e in r["point_err"]]
    n_st = sum(r["n_stacked"] for r in rs)
    print(f"\n【{title}】{len(rs)} 个场景，{n_obj} 个方块（其中叠在别人上面的 {n_st} 个）")
    print(f"  能解析        {sum(r['parsed'] for r in rs) / len(rs):6.0%}")
    print(f"  场景全对      {sum(r['all_ok'] for r in rs) / len(rs):6.0%}")
    print(f"  漏报          {1 - n_match / n_obj:6.0%}   多报（幻觉）{sum(len(r['extra']) for r in rs)} 个")
    if n_match:
        print(f"  position 对   {sum(r['pos_ok'] for r in rs) / n_match:6.0%}")
        print(f"  on 对         {sum(r['on_ok'] for r in rs) / n_match:6.0%}   叠放的那些：{sum(r['stacked_ok'] for r in rs)}/{n_st}")
    if errs:
        print(f"  point 误差    中位数 {np.median(errs):.0f} 像素，90% 分位 {np.percentile(errs, 90):.0f} 像素；"
              f"对号入座 {sum(r['point_match'] for r in rs) / len(errs):.0%}")


report(records, "全部")
report([r for r in records if r["dim"]], "偏暗的场景")
report([r for r in records if r["n_stacked"]], "有叠放的场景")
print(f"\n结果已存 results/{args.tag}.json，总用时 {time.time() - t0:.0f} 秒")
