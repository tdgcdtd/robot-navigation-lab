# 里程碑 4：LoRA 微调 VLM（Qwen3-VL-4B-Instruct，MLX）

看桌面图片 → 输出固定格式 JSON（有哪些方块、粗略位置、图像坐标点、叠在谁上面），给里程碑 5 的 LLM 规划用。
毫米级抓取坐标仍由里程碑 3（DINO + SAM + 深度）负责；`point` 只用来把 VLM 说的物体和 DINO 的框对号入座。

| 文件 | 作用 |
|---|---|
| `scenes.py` | 场景：2~5 个不同颜色方块，约 1/3 叠放、1/3 偏暗；自动生成标准答案 JSON |
| `vlm.py` | 推理：提示词、加载模型（可加 LoRA，或只做 8 bit 量化作对照）、解析 JSON |
| `make_dataset.py` | 造训练数据（mlx-vlm 的 JSONL 对话格式）：训练种子 0 起、验证 5000 起 |
| `make_config.py` | 生成 `lora_config.json`：LoRA 挂在语言部分全部线性层 + 视觉部分 104 层 |
| `evaluate.py` | 测试：固定 100 个场景（种子 10000 起），解析率、漏报/幻觉、position、on、point |
| `watch_train.py` | 终端里实时看训练进度、train / val loss |

```bash
../.venv/bin/python make_dataset.py 1000 50
../.venv/bin/python make_config.py
PYTHONUNBUFFERED=1 ../.venv/bin/python -m mlx_vlm.train -c lora_config.json --model Qwen/Qwen3-VL-4B-Instruct \
  --data data --iters 1000 --learning-rate 1e-4 --lora-rank 32 --train-on-completions \
  --quantization-bits 8 --grad-checkpoint --steps-per-report 20 --steps-per-eval 200 \
  --val-batches 10 --steps-per-save 100 --output-path adapters/lora > results/log_train_lora.txt 2>&1
../.venv/bin/python watch_train.py                                        # 另开终端看进度
../.venv/bin/python evaluate.py --tag zeroshot                            # 原版（bf16）
../.venv/bin/python evaluate.py --tag zeroshot8bit --quantize-like adapters/lora
../.venv/bin/python evaluate.py --tag lora --adapter adapters/lora
```

## 结果（同 100 个测试场景，333 个方块，其中 94 个叠在别的方块上）

| | 零样本 bf16 | 零样本 8 bit | **LoRA（8 bit）** |
|---|---|---|---|
| JSON 能解析 | 100% | 100% | 100% |
| 场景全对 | 13% | 13% | **93%** |
| 漏报 / 幻觉 | 7% / 5 个 | 6% / 5 个 | **0% / 0 个** |
| position 对 | 77% | 78% | **98%** |
| 叠放关系对 | 24/94 | 18/94 | **93/94** |
| point 中位误差 | 12 像素 | 12 像素 | **1 像素** |

训练：Air M5 16 GB，2.4 小时，可训练参数 8050 万（1.8%），val loss 0.120 → 0.085 没有过拟合。
剩下的错误：6 处 position 都在分界线 ±0.4 cm 以内，2 处叠放。只在仿真里验证过，场景里全是方块、颜色不重复。
