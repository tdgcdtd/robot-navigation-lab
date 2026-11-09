# 里程碑 1：MuJoCo 倒立摆 + PPO

## 环境（只需一次）
```bash
cd ml
python3 -m venv .bootstrap && .bootstrap/bin/pip install uv
.bootstrap/bin/uv venv --python 3.12 .venv
.bootstrap/bin/uv pip install --python .venv/bin/python "gymnasium[mujoco]" stable-baselines3 tensorboard imageio imageio-ffmpeg
```

## 运行（在 ml/m1_pendulum 目录）
| 脚本 | 做什么 |
|---|---|
| `../.venv/bin/python 01_random.py` | 随机策略，约 3 步就倒 → `videos/random.mp4` |
| `../.venv/bin/python 02_train_ppo.py 100000` | PPO 训练 10 万步（M5 Air 约 18 秒），约 7 万步达到满分 1000 |
| `../.venv/bin/python 03_record.py` | 加载模型录平衡视频 → `videos/ppo.mp4` |
| `../.venv/bin/tensorboard --logdir runs` | 看训练曲线（ep_len_mean、train/std、value_loss） |
| `../.venv/bin/python 04_reward_experiment.py` | 奖励实验：推小车 + 不同扣分权重，约 70 秒，输出对比表和 `videos/center_w*.mp4` |
