"""把仿真自动标注的样本存到硬盘：data/train、data/val。

三份数据各管一件事（都不重叠）：
- 训练集 train：拿来改参数。正样本种子 0 起，负样本种子 2000 起，
  另补 200 个紫/橙正样本（种子 3000 起）：负样本里"purple/orange cube"特别多，不补的话模型可能学成"问紫色就说没有"
- 验证集 val：训练中途看 loss，判断是否过拟合、该不该停。正样本 5000 起，负样本 5100 起
- 测试集 test：benchmark.py / reject.py 里种子 10000 起，只在训练前后各考一次

负样本（box 为 null）：问的东西画面里没有，正确答案是"一个框都不给"。
已经存在的图片不会重新渲染（同一个种子渲染结果完全一样），所以加负样本时只多渲染新的那部分。

用法：../.venv/bin/python make_dataset.py [训练正样本=2000] [训练负样本=700] [验证正=100] [验证负=40]
"""
import json, os, sys, time
import imageio.v2 as imageio
from datagen import make_sample, make_negative


def dump(split, pos_seeds, neg_seeds):
    os.makedirs(f"data/{split}", exist_ok=True)
    old = {}
    if os.path.exists(f"data/{split}/labels.json"):
        old = {it["image"]: it for it in json.load(open(f"data/{split}/labels.json"))}
    labels = []
    jobs = [(s, make_sample) for s in pos_seeds] + [(s, make_negative) for s in neg_seeds]
    if split == "train":
        jobs += [(s, lambda s: make_sample(s, "相近颜色", ["purple", "orange"][s % 2])) for s in range(3000, 3200)]
    for k, (seed, fn) in enumerate(jobs):
        name = f"{seed:05d}.png"
        if name in old and os.path.exists(f"data/{split}/{name}"):
            labels.append(old[name])                  # 之前生成过，直接复用
            continue
        s = fn(seed)
        imageio.imwrite(f"data/{split}/{name}", s["rgb"])
        box = None if s["box"] is None else s["box"].tolist()
        labels.append(dict(image=name, text=s["text"], box=box, scenario=s["scenario"]))
        if (k + 1) % 100 == 0:
            print(f"{split}: {k + 1}/{len(jobs)}", flush=True)
    with open(f"data/{split}/labels.json", "w") as f:
        json.dump(labels, f, ensure_ascii=False)
    n_neg = sum(it["box"] is None for it in labels)
    print(f"{split}: 共 {len(labels)} 个（负样本 {n_neg} 个）")


a = [int(x) for x in sys.argv[1:]] + [2000, 700, 100, 40][len(sys.argv) - 1:]
t0 = time.time()
dump("train", range(a[0]), range(2000, 2000 + a[1]))
dump("val", range(5000, 5000 + a[2]), range(5100, 5100 + a[3]))
print(f"完成，用时 {time.time() - t0:.0f} 秒")
