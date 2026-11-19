# 里程碑 6：端到端 VLA（视觉-语言-动作模型）实战与对比

> 目标：在 M5 同一套 E6 机械臂 + 吸盘物理世界里，不依赖 DINO、SAM、深度图与 MoveIt 2，微调开源连续动作块 VLA（`SmolVLA` 与 `π0.5`），跑通端到端抓取闭环并与 M5 分层架构做实测对比。

## 文件结构

| 文件 | 作用 |
|---|---|
| `env.py` | 10 Hz E6 抓取环境：复用 M5 物理与全局 `eye` 相机，在 `Link6` 外沿 `(0, 0.055, 0.022)` 增配避障腕部相机 `wrist`（双视角 `256×256`），支持灯光域随机化与跨平台 `meshdir` 覆盖 |
| `expert.py` | 基于 MuJoCo 雅可比阻尼最小二乘数值 IK 的确定性专家轨迹生成器（60 帧 = 6.0 s/条），避免 RRT 随机采样制造多峰噪声 |
| `collect_data.py` | 自动采集 4 类语言任务（`red/blue × left_zone/right_zone`）的双视角视频 + 7 维状态/动作，含灯光域随机化（`0.85~1.15`）与物理成功验收 |
| `convert_to_lerobot.py` | 将原始 `.npz` + `.mp4` 校验并转换为标准 HuggingFace `LeRobotDataset v3.0`，同时向 `meta/stats.json` 补全 `q01~q99` 分位点统计（兼容 `SmolVLA` 与 `π0.5`） |
| `eval_vla.py` | E6 闭环评测脚本：同时支持 `expert` 基线、`SmolVLA`（Mac 16GB MPS `mjpython` 实时 3D 窗口）与 `π0.5`（Colab 无头 `EGL`），默认每 `5` 步（`0.5 s`）重看双画面闭环纠偏，并导出三视角对比视频 |
| `colab_train_vla.ipynb` | 最佳效果 Google Colab Notebook：按显存自动调配 `batch_size`、`use_amp` 与 `num_workers=4`，跑 `SmolVLA`（`20,000` 步）或 `π0.5` 高秩 LoRA（`r=32`，`6,000` 步），并在 MuJoCo 中逐个评测末尾 3 个检查点选出真冠军备份回 Drive 后自动释放 GPU |

## 相机设计（为什么保留背后 `eye` + 新增随手 `wrist`）

- **`eye`（眼在手外，Eye-to-Hand）**：固定在底座后上方，负责看清整张桌面、两个方块与 `left_zone / right_zone` 目标区的全局空间关系；无论手臂怎么转，桌面坐标基准不乱。
- **`wrist`（眼在手上，Eye-in-Hand）**：固定在 `Link6` 法兰外沿随手移动，靠近抓取时从近处放大看清吸盘头和目标方块的相对偏移，把毫米级对准交还给局部视觉闭环。

## 云端训练方案与预算（Google AI Pro 200 计算单元）

统一使用 HuggingFace **`LeRobot`** 数据集格式，一份 `320` 条 E6 数据跑两套模型对比：
1. **`SmolVLA`（~450M，Flow Matching，最佳效果档）**：在 **Colab A100 (40GB)**（`batch_size=64`、`20,000` 步，约 **13–17 CU**）或 **L4 (24GB)**（`batch_size=32`、`20,000` 步，约 **8–10 CU**）微调 + 多检查点物理筛选，冠军权重下载回 **Mac 16GB** 实时 3D 窗口跑闭环评测。
2. **`π0.5`（`lerobot/pi05_base`，高秩 LoRA `r=32`）**：同一份数据集在 **Colab A100（40GB）** 跑 `6,000` 步 + 多检查点物理筛选（约 **20–26 CU**），用于简历与架构对比。
- **合计预算**：约 **28–43 CU**（占每月 `200` CU 额度的 ~14%–22%）；Colab Notebook 末尾自动将 `best_pretrained_model.zip` 存入 Google Drive 并等待 15 秒调用 `runtime.unassign()` 释放 GPU。

## 当前进度

- [x] ① 预习通关与双相机冒烟测试（4 条全部吸取放置成功，平均误差 `0.015 mm`）
- [x] ② 加入灯光域随机化（`0.85~1.15`）并采齐 **320 条**完整数据集（`19,200` 帧，`52 MB`，平均误差 `0.016 mm`，最大 `0.020 mm`，无跳过回合）
- [x] ③ 打包便携 Colab 上传包 `ml/m6_vla/data/e6_colab_bundle.zip`（`55.30 MB`，`970` 个文件，含脱敏相对路径 `e6.xml` + 7 个 STL + 320 条数据）
- [x] ④ 完成 `convert_to_lerobot.py`（`--dry-run` 全部 `19,200` 帧校验通过）、`eval_vla.py`（默认 `n_action_steps=5`，`expert` 未见种子 `seed=1000` 评测 `8/8 = 100%`，平均误差 `0.015 mm`）与 `colab_train_vla.ipynb`（最佳效果配置 + 多检查点物理选优）
- [ ] ⑤ 下一步：把 `e6_colab_bundle.zip` 传上 Google Drive → 在 Colab 依次跑通 `SmolVLA` 与 `π0.5` LoRA → 下载冠军 `SmolVLA` 权重回 Mac 跑实时 3D 闭环 & Q&A 结项
