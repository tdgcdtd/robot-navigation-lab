# 里程碑 6：端到端 VLA（视觉-语言-动作模型）实战与对比

> 目标：在 M5 同一套 E6 机械臂 + 吸盘物理世界里，不依赖 DINO、SAM、深度图与 MoveIt 2，微调开源连续动作块 VLA（`SmolVLA` 与 `π0.5`），跑通端到端抓取闭环并与 M5 分层架构做实测对比。

## 文件结构

| 文件 | 作用 |
|---|---|
| `env.py` | 10 Hz E6 抓取环境：复用 M5 物理与全局 `eye` 相机，在 `Link6` 外沿 `(0, 0.055, 0.022)` 增配避障腕部相机 `wrist`（双视角 `256×256`），支持灯光域随机化与跨平台 `meshdir` 覆盖 |
| `expert.py` | 基于 MuJoCo 雅可比阻尼最小二乘数值 IK 的确定性专家轨迹生成器（60 帧 = 6.0 s/条），避免 RRT 随机采样制造多峰噪声 |
| `collect_data.py` | 自动采集 4 类语言任务（`red/blue × left_zone/right_zone`）的双视角视频 + 7 维状态/动作，含灯光域随机化（`0.85~1.15`）与物理成功验收 |
| `convert_to_lerobot.py` | 将原始 `.npz` + `.mp4` 校验并转换为标准 HuggingFace `LeRobotDataset v3.0`，同时向 `meta/stats.json` 补全 `q01~q99` 分位点统计（兼容 `SmolVLA` 与 `π0.5`） |
| `eval_vla.py` | E6 闭环评测脚本：同时支持 `expert` 基线、`SmolVLA`（Mac 16GB MPS `mjpython` 实时 3D 窗口）与 `π0.5`（Colab无头 `EGL`）与三视角对比视频导出 |
| `colab_train_vla.ipynb` | 一键式 Google Colab Notebook：自动解压数据包、转换 LeRobot 格式、动态对齐相机键名、微调 `SmolVLA`（L4）或 `π0.5` LoRA（A100），备份回 Google Drive 并自动释放 GPU |

## 相机设计（为什么保留背后 `eye` + 新增随手 `wrist`）

- **`eye`（眼在手外，Eye-to-Hand）**：固定在底座后上方，负责看清整张桌面、两个方块与 `left_zone / right_zone` 目标区的全局空间关系；无论手臂怎么转，桌面坐标基准不乱。
- **`wrist`（眼在手上，Eye-in-Hand）**：固定在 `Link6` 法兰外沿随手移动，靠近抓取时从近处放大看清吸盘头和目标方块的相对偏移，把毫米级对准交还给局部视觉闭环。

## 云端训练方案与预算（Google AI Pro 200 计算单元）

统一使用 HuggingFace **`LeRobot`** 数据集格式，一份 E6 数据跑两套模型对比：
1. **`SmolVLA`（~450M，Flow Matching）**：在 **Colab L4（24GB，~4.8 CU/h）** 微调 1–1.5 小时（约 **5–8 CU**），权重下载回 **Mac 16GB** 实时 3D 窗口跑闭环评测。
2. **`π0.5`（`lerobot/pi05_base`，LoRA）**：同一份数据集换参数在 **Colab A100（40GB，~13.0 CU/h）** 跑 LoRA 微调与云端闭环评测（约 **13–20 CU**），用于简历与架构对比。
- **合计预算**：约 **18–28 CU**（占每月 200 CU 额度的 ~10%–14%）；Colab Notebook 末尾自动存 Google Drive 并调用 `runtime.unassign()` 释放 GPU。

## 当前进度

- [x] ① 预习通关与双相机冒烟测试（4 条全部吸取放置成功，平均误差 `0.015 mm`）
- [x] ② 加入灯光域随机化（`0.85~1.15`）并采齐 **160 条**完整数据集（9600 帧，`26 MB`，平均误差 `0.016 mm`，最大 `0.020 mm`，无跳过回合）
- [x] ③ 打包便携 Colab 上传包 `ml/m6_vla/data/e6_colab_bundle.zip`（`31 MB`，含脱敏相对路径 `e6.xml` + 7 个 STL + 160 条数据）
- [x] ④ 完成 `convert_to_lerobot.py`（`--dry-run` 全量校验通过）、`eval_vla.py`（`expert` 未见种子 `seed=1000` 评测 `8/8 = 100%`，平均误差 `0.015 mm`）与 `colab_train_vla.ipynb`
- [ ] ⑤ 下一步：把 `e6_colab_bundle.zip` 传上 Google Drive → 在 Colab 依次跑通 `SmolVLA`（L4）与 `π0.5` LoRA（A100）→ 下载 `SmolVLA` 权重回 Mac 跑实时 3D 闭环 & Q&A 结项
