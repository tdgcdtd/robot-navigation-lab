"""⑤ 核心手写：从零写出训练循环的两段（限时 5 分钟，不看 train.py）。

运行：../.venv/bin/python handwrite.py
写对的标准：最后打印 ✅（验证集平均误差 < 10 mm）。卡住了就问，每次只给一小步提示。
"""

import argparse
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from torch.utils.tensorboard import SummaryWriter
from fk import forward_kinematics

parser = argparse.ArgumentParser()
parser.add_argument("--n-train", type=int, default=50_000, help="训练样本数")
parser.add_argument("--epochs", type=int, default=40, help="把训练集完整过几遍")
parser.add_argument("--lr", type=float, default=1e-3, help="学习率：每次更新迈多大步")
parser.add_argument("--name", default="default", help="TensorBoard 里这次实验的名字")
args = parser.parse_args()

torch.manual_seed(0)
rng = np.random.default_rng(0)

# ---------------- 数据（已写好）----------------
# 关节角在 ±90° 内均匀随机采样；位置单位用厘米，让数字落在几到几十之间，网络更好学
ANGLE_RANGE = np.pi / 2


def make_data(n):
    q = rng.uniform(-ANGLE_RANGE, ANGLE_RANGE, size=(n, 6))
    pos_cm = forward_kinematics(q) * 100
    return torch.tensor(q, dtype=torch.float32), torch.tensor(pos_cm, dtype=torch.float32)


q_train, p_train = make_data(args.n_train)
q_val, p_val = make_data(5_000)  # 验证集：训练时网络从没见过，用来检查是"学会了"还是"背答案"

# DataLoader：每轮把训练集打乱，切成每份 256 条的小批（batch）依次喂给网络
train_loader = DataLoader(TensorDataset(q_train, p_train), batch_size=256, shuffle=True)

# ---------------- 模型 / loss / 优化器（已写好）----------------
model = nn.Sequential(          # 6 → 256 → 256 → 256 → 3 的全连接网络
    nn.Linear(6, 256), nn.ReLU(),
    nn.Linear(256, 256), nn.ReLU(),
    nn.Linear(256, 256), nn.ReLU(),
    nn.Linear(256, 3),
)
loss_fn = nn.MSELoss()          # 均方误差：(预测 - 真值)² 的平均，单位 cm²
optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
writer = SummaryWriter(f"runs/handwrite")

# ---------------- 训练循环 ----------------
for epoch in range(args.epochs):
    model.train()               # 切到训练模式
    train_loss_sum = 0.0
    for q_batch, p_batch in train_loader:
        # ========== ① 训练一步：在这里写你的代码（5 行）==========
        # 要得到变量 loss（下面要用）
        pred = model(q_batch)
        loss = loss_fn(pred, p_batch)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        # ==========================================================
        train_loss_sum += loss.item() * len(q_batch)
    train_loss = train_loss_sum / len(q_train)

    model.eval()                # 切到评估模式
    # ========== ② 验证：在这里写你的代码（3 行）==========
    # 要得到变量 val_pred 和 val_loss（下面要用）
    with torch.no_grad():
        val_pred = model(q_val)
        val_loss = loss_fn(val_pred, p_val)

    # =====================================================

    mean_err_mm = (val_pred - p_val).norm(dim=1).mean().item() * 10
    writer.add_scalars("loss", {"train": train_loss, "val": val_loss.item()}, epoch)
    writer.add_scalar("val_mean_error_mm", mean_err_mm, epoch)
    print(f"第 {epoch + 1:3d} 轮  train loss {train_loss:8.3f}  val loss {val_loss.item():8.3f}"
          f"  验证集平均误差 {mean_err_mm:5.1f} mm")
writer.close()

# ---------------- 验收（不用改）----------------
print("\n用 p2_fk.cpp 里的测试姿态考一考（神经网络 vs 解析 FK）：")
poses = {
    "全部归零": [0, 0, 0, 0, 0, 0],
    "home 姿态": [0, 0.5337, 0.7276, 0.3548, -1.5848, 0],
    "任意姿态 A": [0.5, -0.3, 0.8, 0.2, 1.0, -0.7],
}
with torch.no_grad():
    for name, q in poses.items():
        nn_pos = model(torch.tensor([q], dtype=torch.float32))[0].numpy() / 100
        fk_pos = forward_kinematics(q)[0]
        err = np.linalg.norm(nn_pos - fk_pos) * 1000
        print(f"  {name:8s} 网络 ({nn_pos[0]:+.4f}, {nn_pos[1]:+.4f}, {nn_pos[2]:+.4f})"
              f"  解析 ({fk_pos[0]:+.4f}, {fk_pos[1]:+.4f}, {fk_pos[2]:+.4f})  误差 {err:5.1f} mm")
print(f"\n{'✅ 通过' if mean_err_mm < 10 else '❌ 未通过'}：验证集平均误差 {mean_err_mm:.1f} mm（目标 < 10 mm）")
