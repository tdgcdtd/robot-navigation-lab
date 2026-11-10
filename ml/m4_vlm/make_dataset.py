"""造微调数据：每个样本 = 一张图 + 一段"对话"（用户：提示词；助手：标准答案 JSON）。

mlx-vlm 要的格式：一个文件夹里放 train.jsonl / valid.jsonl，每行一个样本：
    {"images": ["/绝对路径/00000.png"],
     "messages": [{"role": "user", "content": "<提示词>"},
                  {"role": "assistant", "content": "<标准答案 JSON>"}]}
提示词必须和推理时（vlm.py 的 PROMPT）一字不差——训练时见什么题，考试就出什么题。

种子不重叠：训练 0 起、验证 5000 起、测试 10000 起（evaluate.py）。

用法：../.venv/bin/python make_dataset.py [训练数=1000] [验证数=50]
"""
import json, os, sys, time
import imageio.v2 as imageio
from scenes import make_scene
from vlm import PROMPT

HERE = os.path.dirname(os.path.abspath(__file__))


def dump(split, seeds):
    img_dir = os.path.join(HERE, "data", "images")
    os.makedirs(img_dir, exist_ok=True)
    n_stack = 0
    with open(os.path.join(HERE, "data", f"{split}.jsonl"), "w") as f:
        for k, seed in enumerate(seeds):
            sc = make_scene(seed)
            path = os.path.join(img_dir, f"{seed:05d}.png")
            imageio.imwrite(path, sc["rgb"])
            answer = json.dumps(sc["answer"], ensure_ascii=False)
            f.write(json.dumps({"images": [path],
                                "messages": [{"role": "user", "content": PROMPT},
                                             {"role": "assistant", "content": answer}]},
                               ensure_ascii=False) + "\n")
            n_stack += any(o["on"] != "table" for o in sc["answer"]["objects"])
            if (k + 1) % 200 == 0:
                print(f"{split}: {k + 1}/{len(seeds)}", flush=True)
    print(f"{split}: {len(seeds)} 个样本（有叠放的 {n_stack} 个）")


n_train = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
n_val = int(sys.argv[2]) if len(sys.argv) > 2 else 50
t0 = time.time()
dump("train", range(n_train))
dump("valid", range(5000, 5000 + n_val))
print(f"完成，用时 {time.time() - t0:.0f} 秒")
