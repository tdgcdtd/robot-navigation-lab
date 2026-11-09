"""第一步：不训练，随机乱推小车，看倒立摆多快就倒。

目的：先认识环境——观测是什么、动作是什么、奖励怎么给。
"""
import gymnasium as gym
from record import ENV_ID, run_episode

env = gym.make(ENV_ID)
print("观测空间:", env.observation_space)  # 4 个数：小车位置、杆角度、小车速度、杆角速度
print("动作空间:", env.action_space)       # 1 个数：推小车的力，范围 [-3, 3]
env.close()

random_policy = lambda obs: env.action_space.sample()
steps, reward = run_episode(random_policy, video_path="videos/random.mp4")
print(f"随机策略：撑了 {steps} 步就倒了，总奖励 {reward}（满分 1000）")
