"""第二步：用 PPO 训练一个会平衡倒立摆的策略。

谁负责什么：
- Gymnasium 环境（MuJoCo 物理）：给观测、执行动作、算奖励
- SB3 的 PPO：收集经验 → 估计"哪些动作比平均好" → 更新策略网络
- 我们：只负责选环境、选算法、定训练步数、存模型

Air 无风扇：训练分段跑，每 10k 步存一次 checkpoint，中断了也能接着用。
"""
import sys
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from record import ENV_ID

TOTAL_STEPS = int(sys.argv[1]) if len(sys.argv) > 1 else 100_000

# 4 个环境并行收集经验：同样时间里数据更多、更多样
env = make_vec_env(ENV_ID, n_envs=4, seed=0)
eval_env = make_vec_env(ENV_ID, n_envs=1, seed=123)

model = PPO(
    "MlpPolicy",      # 策略网络：两层 64 神经元的全连接网络（小网络，CPU 就够）
    env,
    device="cpu",     # 网络太小，MPS 搬数据的开销比计算还大
    verbose=0,
    tensorboard_log="runs",
)

callbacks = [
    CheckpointCallback(save_freq=10_000 // 4, save_path="checkpoints", name_prefix="ppo"),
    # 每 5000 步拿当前策略考一次试（不带随机性），打印平均得分
    EvalCallback(eval_env, eval_freq=5_000 // 4, n_eval_episodes=5, verbose=1,
                 best_model_save_path="checkpoints/best"),
]

model.learn(total_timesteps=TOTAL_STEPS, callback=callbacks, progress_bar=False)
model.save("ppo_pendulum")
print("训练完成，模型已存为 ppo_pendulum.zip")
