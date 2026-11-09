# 里程碑 3：开放词汇感知 + 微调

| 文件 | 作用 |
|---|---|
| `scene.py` | MuJoCo 桌面场景：随机彩色方块，渲染彩色图 / 深度图 / 分割图（真值），反投影工具 |
| `perceive.py` | 零样本流程：文字 → Grounding DINO base 出框 → SAM 2.1 large 抠轮廓 → 深度算 3D |
| `scenarios.py` | 5 类场景：分开、挨在一起、相近颜色、暗光、遮挡（测试集种子 10000 起） |
| `datagen.py` | 仿真自动标注：正样本（"yellow cube." + 真值框）、负样本（问不存在的东西，答案为空） |
| `make_dataset.py` | 存数据到 `data/train`、`data/val`（正样本、负样本、补充的紫/橙正样本，种子互不重叠） |
| `finetune.py` | 全量微调 DINO base：梯度累积、预热 + 余弦学习率、梯度裁剪、断点续训 |
| `benchmark.py` | 测试：每类场景 20 个，检测对（IoU ≥ 0.5）+ 3D 误差 < 10 mm |
| `reject.py` | 测试：问不存在的颜色 / 没见过的无关物体，在不同阈值下看拒绝率和正样本正确率 |
| `shapes_check.py` | 遗忘检查：把方块换成球和圆柱，看没训练过的形状和说法还认不认得 |

模型（首次运行自动从 Hugging Face 下载，约 1.8 GB，存在 `~/.cache/huggingface`）：
`IDEA-Research/grounding-dino-base`、`facebook/sam2.1-hiera-large`。另需 `transformers`、`torchvision`。

```bash
../.venv/bin/python scene.py                      # 渲染场景 + 自检反投影 → scene_rgb.png
../.venv/bin/python perceive.py                   # 零样本感知 → 误差表 + perceive_result.png
../.venv/bin/python make_dataset.py               # 造训练 / 验证数据（约 30 秒）
../.venv/bin/python finetune.py --out ft_neg      # 微调（Air M5 约 73 分钟）→ ckpt/ft_neg
../.venv/bin/python benchmark.py --edge 480 --dino ckpt/ft_neg --tag ft_neg480
../.venv/bin/python reject.py --edge 480 --dino ckpt/ft_neg --tag ft_neg480 --pos ft_neg480
../.venv/bin/python shapes_check.py ckpt/final ckpt/ft_neg
```

## 结果（480 分辨率，同一测试集）

| | 零样本 | 微调（只有正样本） | 微调（加负样本） |
|---|---|---|---|
| 检测正确率 | 64% | 99% | 100% |
| 3D < 10 mm（SAM 轮廓） | 62% | 98% | 97% |
| 拒绝不存在的颜色（阈值 0.3） | 0% | 3% | 100% |
| 拒绝没见过的无关物体（阈值 0.3） | 33% | 78% | 100% |

遗留问题：只会按颜色拒绝，不会按形状拒绝（只有红球时问 "red cube" 仍给 0.88）；"a can" 指圆柱的能力变差（0.82 → 0.12）。
只在和训练同分布的仿真里测过，没验证真实相机。
