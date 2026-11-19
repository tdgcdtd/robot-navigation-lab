"""VLM 看图 → JSON 场景描述（用 MLX 在 Mac 上跑 Qwen3-VL-4B-Instruct）。

VLM 和 M3 的 DINO 的分工：
- DINO：给一个词 → 精确的框（像素级）。负责"在哪"
- VLM：给一张图 + 一个问题 → 一段文字（一个 token 一个 token 往外生成）。负责"有什么、什么关系"
"""
import json, os, re, tempfile, time
import imageio.v2 as imageio
from mlx_vlm import generate, load
from mlx_vlm.prompt_utils import apply_chat_template

MODEL_ID = "Qwen/Qwen3-VL-4B-Instruct"

PROMPT = """List every cube on the table. Answer with JSON only, no other text, in exactly this format:
{"objects": [{"color": "red", "shape": "cube", "position": "left", "point": [x, y], "on": "table"}]}
Rules:
- color: one of red, blue, green, yellow, orange, purple
- position: "left", "center" or "right" part of the table
- point: the cube's center in the image, x and y from 0 to 1000
- on: "table", or the color of the cube it is stacked on
- sort objects from left to right by point x"""


class VLM:
    def __init__(self, path=MODEL_ID, adapter=None, quantize_like=None):
        """path：模型（Hugging Face 名字或本地文件夹）；adapter：LoRA 微调出来的小矩阵（文件夹），不给就是原版。
        quantize_like：某个 adapter 文件夹。不加 LoRA，只把原模型按它训练时的方式压成 8 bit
        ——用来公平对比：分清进步来自 LoRA 还是来自量化。"""
        self.model, self.processor = load(path, adapter_path=adapter)
        if quantize_like:
            import mlx.nn as nn
            q = json.load(open(os.path.join(quantize_like, "adapter_config.json")))["base_quantization"]
            spec = next(iter(q.values()))
            nn.quantize(self.model, group_size=spec["group_size"], bits=spec["bits"],
                        class_predicate=lambda p, m: p in q and hasattr(m, "to_quantized"))
        self.prompt = apply_chat_template(self.processor, self.model.config, PROMPT, num_images=1)

    def describe(self, rgb, max_tokens=400):
        """返回 (原始输出文字, 解析出的 dict 或 None)。"""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "scene.png")
            imageio.imwrite(path, rgb)
            out = generate(self.model, self.processor, self.prompt, image=path,
                           max_tokens=max_tokens, temperature=0.0, verbose=False)
        text = out.text if hasattr(out, "text") else str(out)
        return text, parse(text)


    def chat(self, system, user, max_tokens=400):
        """纯文字对话（不看图），用来做任务规划。

        LoRA 的效果是"原权重 + scale × 小矩阵"。规划时把 scale 临时改成 0，模型就变回原版 Qwen3-VL
        （LoRA 只学过"看图写场景 JSON"，带着它做规划反而可能变差）；用完改回去。
        这样内存里始终只有一个 4B 模型，不用再加载一个规划模型。
        """
        loras = [m for _, m in self.model.named_modules() if hasattr(m, "lora_a") and hasattr(m, "scale")]
        saved = [m.scale for m in loras]
        for m in loras:
            m.scale = 0.0
        try:
            prompt = apply_chat_template(self.processor, self.model.config,
                                         [{"role": "system", "content": system}, {"role": "user", "content": user}],
                                         num_images=0)
            out = generate(self.model, self.processor, prompt, max_tokens=max_tokens, temperature=0.0, verbose=False)
        finally:
            for m, s in zip(loras, saved):
                m.scale = s
        return out.text if hasattr(out, "text") else str(out)


def parse(text):
    """从输出里抠出 JSON。程序要的是"每次都能解析"，解析不了就算这次失败（返回 None）。"""
    m = re.search(r"\{.*\}", text, re.S)        # 模型常在 JSON 外面包 ```json ... ``` 或加一句话
    if not m:
        return None
    try:
        d = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    return d if isinstance(d, dict) and isinstance(d.get("objects"), list) else None


if __name__ == "__main__":
    from scenes import make_scene
    t0 = time.time(); v = VLM(); print(f"加载用了 {time.time() - t0:.0f} 秒")
    sc = make_scene(10_000)
    t0 = time.time(); text, d = v.describe(sc["rgb"]); print(f"生成用了 {time.time() - t0:.1f} 秒\n")
    print("模型输出：\n" + text)
    print("\n标准答案：\n" + json.dumps(sc["answer"], ensure_ascii=False))
    imageio.imwrite("test_scene.png", sc["rgb"])
