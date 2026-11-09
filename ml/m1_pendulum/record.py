"""公共工具：让一个策略在倒立摆环境里跑一局，把画面存成 mp4。

policy 是一个函数：输入观测 obs，输出动作 action。
随机策略、训练好的 PPO 都可以塞进来，这样两者的录像方式完全一样。
"""
import gymnasium as gym
import imageio.v2 as imageio

ENV_ID = "InvertedPendulum-v5"


def run_episode(policy, video_path=None, max_steps=1000, make_env=None, seed=0, xs=None):
    """make_env：自定义建环境的函数（比如套上推力/扣分包装）；xs：传个列表进来就记录小车位置。"""
    render_mode = "rgb_array" if video_path else None
    env = make_env(render_mode) if make_env else gym.make(ENV_ID, render_mode=render_mode)
    obs, _ = env.reset(seed=seed)
    frames, total_reward = [], 0.0
    for step in range(max_steps):
        action = policy(obs)
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward
        if xs is not None:
            xs.append(float(obs[0]))
        if video_path:
            frames.append(env.render())
        if terminated or truncated:  # terminated=杆倒了；truncated=到时间上限
            break
    env.close()
    if video_path:
        imageio.mimsave(video_path, frames, fps=env.metadata["render_fps"])
    return step + 1, total_reward
