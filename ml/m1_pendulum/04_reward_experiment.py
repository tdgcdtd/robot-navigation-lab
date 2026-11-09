"""第四步：奖励设计实验——"离中心越远扣分越多"，扣轻和扣重会学出什么？

原始策略本来就很靠中心（只偏约 0.07），直接加扣分看不出区别。
所以每 100 步随机推小车一把（推力 1.0），逼它必须自己回中。
三组只差一个变量：扣分权重 w。

    新奖励 = 原奖励(杆没倒 +1) - w × 小车位置²

    w=0  ：不扣分（对照组）
    w=1  ：轻扣。导轨边缘 x=±1 时扣 1，奖励刚好变 0，所以活着永远不亏
    w=20 ：重扣。|x|>0.22 时每步奖励就变负了 —— 被推远后，"活着"反而亏分

用法：../.venv/bin/python 04_reward_experiment.py [训练步数]
"""
import sys
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from record import ENV_ID, run_episode

PUSH = 1.0           # 推力：直接给小车加的速度
PUSH_EVERY = 100     # 每多少步推一次
WEIGHTS = [0, 1, 20]
TOTAL_STEPS = int(sys.argv[1]) if len(sys.argv) > 1 else 200_000


class CenterTask(gym.Wrapper):
    """包装器：不改 MuJoCo 物理模型本身，只在外面加"随机推一把"和"离中心扣分"。"""

    def __init__(self, env, w=0.0):
        super().__init__(env)
        self.w = w

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.t = 0
        # 推的方向随机；用 reset 的 seed 初始化，保证评估可复现
        self.rng = np.random.default_rng(kwargs.get("seed"))
        return obs, info

    def step(self, action):
        if self.t % PUSH_EVERY == PUSH_EVERY // 2:
            self.env.unwrapped.data.qvel[0] += self.rng.choice([-1, 1]) * PUSH
        self.t += 1
        obs, reward, terminated, truncated, info = self.env.step(action)
        x = obs[0]  # 小车位置
        return obs, reward - self.w * x**2, terminated, truncated, info


def make_env(w, render_mode=None):
    return CenterTask(gym.make(ENV_ID, render_mode=render_mode), w=w)


print(f"{'w':>4} | {'平均撑住步数':>10} | {'平均|x|':>8} | {'最大|x|':>8}")
for w in WEIGHTS:
    env = make_vec_env(lambda: make_env(w), n_envs=4, seed=0)
    model = PPO("MlpPolicy", env, device="cpu", verbose=0, tensorboard_log="runs", seed=0)
    model.learn(total_timesteps=TOTAL_STEPS, tb_log_name=f"center_w{w}")
    model.save(f"ppo_center_w{w}")

    policy = lambda obs: model.predict(obs, deterministic=True)[0]
    steps_list, xs_all = [], []
    for seed in range(5):  # 用 5 局不同的推力序列来考试
        xs = []
        steps, _ = run_episode(policy, make_env=lambda rm: make_env(w, rm), seed=seed, xs=xs,
                               video_path=f"videos/center_w{w}.mp4" if seed == 0 else None)
        steps_list.append(steps)
        xs_all += xs
    xs_all = np.abs(xs_all)
    print(f"{w:>4} | {np.mean(steps_list):>10.0f} | {xs_all.mean():>8.3f} | {xs_all.max():>8.3f}"
          f"   每局步数={steps_list}", flush=True)
