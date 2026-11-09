"""第三步：加载训练好的模型，录一段平衡视频。"""
import sys
from stable_baselines3 import PPO
from record import run_episode

path = sys.argv[1] if len(sys.argv) > 1 else "ppo_pendulum.zip"
model = PPO.load(path, device="cpu")

# deterministic=True：考试时不再随机探索，直接用网络认为最好的动作
policy = lambda obs: model.predict(obs, deterministic=True)[0]
steps, reward = run_episode(policy, video_path="videos/ppo.mp4")
print(f"PPO 策略：撑了 {steps} 步，总奖励 {reward}（满分 1000）")
