# 里程碑 2：最小 PyTorch 训练循环 —— 神经网络学 E6 正运动学

| 文件 | 作用 |
|---|---|
| `fk.py` | 从 `practice/p2_fk.cpp` 移植的解析 FK：生成训练数据，也当裁判。`python fk.py` 自测 |
| `train.py` | 完整训练循环（② 快速搭建：AI 写、你运行观察）。目标：验证集平均误差 < 10 mm |

```bash
../.venv/bin/python train.py                                            # 约 10 秒
../.venv/bin/python train.py --n-train 1000 --epochs 300 --name small   # 过拟合实验
../.venv/bin/tensorboard --logdir runs
```
