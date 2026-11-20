# 里程碑 6：端到端 VLA（视觉-语言-动作模型）实战与对比

> 目标：在 M5 同一套 E6 机械臂 + 吸盘物理世界里，不依赖 DINO、SAM、深度图与 MoveIt 2，微调开源连续动作块 VLA（`SmolVLA` 与 `π0.5`），跑通端到端抓取闭环并与 M5 分层架构做实测对比。

## 文件结构

| 文件 | 作用 |
|---|---|
| `env.py` | 10 Hz E6 抓取环境：复用 M5 物理与全局 `eye` 相机，在 `Link6` 外沿 `(0, 0.055, 0.022)` 增配避障腕部相机 `wrist`（双视角 `256×256`），采用持续负压吸盘物理（严格保持 `10 mm / -4~+6 mm / 15°` 门槛），支持灯光域随机化与跨平台 `meshdir` 覆盖 |
| `expert.py` | 基于 MuJoCo 雅可比阻尼最小二乘数值 IK 的确定性专家轨迹生成器（60 帧 = 6.0 s/条），避免 RRT 随机采样制造多峰噪声 |
| `collect_data.py` | 自动采集 4 类语言任务（`red/blue × left_zone/right_zone`）的双视角视频 + 7 维状态/动作，含灯光域随机化（`0.85~1.15`）、物理成功验收与 RNG 校验 `--resume` 断点续采 |
| `convert_to_lerobot.py` | 将原始 `.npz` + `.mp4` 校验并用 H.264 流式编码转换为标准 HuggingFace `LeRobotDataset v3.0`，同时向 `meta/stats.json` 补全 `q01~q99` 分位点统计（兼容 `SmolVLA` 与 `π0.5`） |
| `eval_vla.py` | E6 闭环评测脚本：同时支持 `expert` 基线、`SmolVLA`（Mac 16GB MPS `mjpython` 实时 3D 窗口）与 `π0.5`（Colab 无头 `EGL`），自动在 CUDA 启用 `bfloat16` 并启用动作块队列快取（Mac MPS `float32` 约 `55.7 ms/步`），默认每 `5` 步（`0.5 s`）重看双画面闭环纠偏，支持导出三视角对比视频 |
| `colab_train_vla.ipynb` | 完备版 Google Colab Notebook：支持 `smolvla_s2`（二阶段热启动解冻 VLM，A100 `bs=64`、`12,000` 步）、`smolvla` 基线与 `pi05` 高秩 LoRA（`r=32`），内置防断网双份同步 `train.log`、5 秒数据集 `.tar` 缓存加载、多检查点 MuJoCo 赛马选优及自动断开释放 GPU |

## 相机设计（为什么保留背后 `eye` + 新增随手 `wrist`）

- **`eye`（眼在手外，Eye-to-Hand）**：固定在底座后上方，负责看清整张桌面、两个方块与 `left_zone / right_zone` 目标区的全局空间关系；无论手臂怎么转，桌面坐标基准不乱。
- **`wrist`（眼在手上，Eye-in-Hand）**：固定在 `Link6` 法兰外沿随手移动，靠近抓取时从近处放大看清吸盘头和目标方块的相对偏移，把毫米级对准交还给局部视觉闭环。

## 云端训练方案与预算（Google AI Pro 200 计算单元）

统一使用 Mac 上已 0 CU 预转换好的 HuggingFace **`LeRobot`** 数据集缓存（**`600` 条** = `36,000` 帧，`lerobot_e6_pick_v600.tar` 仅 `106 MB`，Colab 开机 5 秒加载）：
1. **`SmolVLA` 消融演进（~450M，Flow Matching，Mac 本地主力演示）**：
   - 第一阶段基线（`smolvla_e6`，L4 `bs=32` 冻结 VLM 跑 `10,000` 步）已完成实测与 Mac 0 CU 诊断（`10%` 成功率，定位出冻结 VLM 对 `red/blue` 语言定位绑定偏弱与 `env.py` 上升沿吸盘问题）；
   - 第二阶段热启动（`smolvla_s2`，**当前默认**）：从 `outputs/smolvla_e6/best_pretrained_model` 热启动并**解冻 16 层 VLM 主干 + 动作专家（`306.5M` 可训练，仅锁 SigLIP）**，在 **Colab A100** 上跑 `batch_size=64`、`12,000` 步（`lr=5e-5` 精准衰减）并对 `003000/006000/009000/012000` 全部 4 个检查点物理挑冠军，约 **60–80 分钟（`~13–18 CU`）**。
2. **`π0.5`（`lerobot/pi05_base`，高秩 LoRA `r=32`）**：同一份 `600` 条数据集缓存直接在 **Colab A100（40GB）** 跑 `6,000` 步 + 多检查点物理筛选（约 **`20–26 CU`**），用于三方横评。
- **保护机制**：独立写入 `outputs/smolvla_e6_s2` 保留第一阶段对比数据，实时写入 `train.log` 同步备份 Drive，`AUTO_DISCONNECT=True` 赛马存盘完毕 15 秒后自动 `runtime.unassign()` 释放 GPU。

## 当前进度

- [x] ① 预习通关与双相机冒烟测试（4 条全部吸取放置成功，平均误差 `0.015 mm`）
- [x] ② 首轮 320 条采集 + 第一阶段 `SmolVLA` 基线训练（`010000` 实测 `10%`）+ Mac 16GB `mjpython`/MPS 0 CU 深度诊断与环境隔离（`ml/m6_vla/.venv`）
- [x] ③ 修复 `env.py` 持续负压物理、`eval_vla.py` `bf16`+队列快取加速、`collect_data.py --resume` 与 `colab_train_vla.ipynb` 二阶段防断网架构
- [x] ④ Mac 上 0 CU 扩采齐 **600 条**完整数据集（`36,000` 帧，平均误差 `0.016 mm`，最大 `0.02 mm`，零跳过）并直接转码打包出 **`ml/m6_vla/data/lerobot_e6_pick_v600.tar`（`106 MB`）**
- [ ] ⑤ 下一步：上传 `lerobot_e6_pick_v600.tar` 至 Drive → Colab A100 运行 `smolvla_s2` 抢救并回 Mac 跑 3D 实时闭环 → 继续跑通 `π0.5` LoRA 横评与 M6 Q&A 结项
