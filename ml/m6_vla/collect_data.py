"""自动采集 M6 VLA 专家演示数据集（eye + wrist 双视角视频 + 7 维状态/动作 + 语言指令）。

用法：
  ../.venv/bin/python collect_data.py --episodes 4 --out data/smoke_test
  ../.venv/bin/python collect_data.py --episodes 160 --out data/e6_pick
"""
import argparse
import json
import os
import time
import imageio.v2 as imageio
import numpy as np

from env import E6PickEnv, FPS, IMG_SIZE, TASKS, ZONES
from expert import generate_expert_actions


def sample_cubes(rng):
    """随机摆放 red / blue 两个方块，彼此及放置区之间保持 >= 9 cm 间距。"""
    zone_pts = list(ZONES.values())
    for _ in range(500):
        pts = [(float(rng.uniform(0.24, 0.34)), float(rng.uniform(-0.12, 0.12))) for _ in range(2)]
        if np.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1]) < 0.09:
            continue
        if any(np.hypot(px - zx, py - zy) < 0.09 for px, py in pts for zx, zy in zone_pts):
            continue
        return {"red": pts[0], "blue": pts[1]}
    raise RuntimeError("未能采样到合法的方块摆放")


def save_video(path, frames):
    with imageio.get_writer(path, fps=FPS, codec="libx264", pixelformat="yuv420p", macro_block_size=1) as w:
        for f in frames:
            w.append_data(f)


def save_preview(out_dir, eye_frames, wrist_frames):
    idx = min(17, len(eye_frames) - 1)
    top = np.hstack([eye_frames[0], wrist_frames[0]])
    bot = np.hstack([eye_frames[idx], wrist_frames[idx]])
    imageio.imwrite(os.path.join(out_dir, "preview_cameras.png"), np.vstack([top, bot]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=160)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "e6_pick"))
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--light-range", nargs=2, type=float, default=[0.85, 1.15],
                    metavar=("MIN", "MAX"), help="灯光亮度域随机化区间")
    args = ap.parse_args()
    light_lo, light_hi = float(args.light_range[0]), float(args.light_range[1])
    if light_lo > light_hi:
        raise ValueError(f"light-range MIN ({light_lo}) 不能大于 MAX ({light_hi})")

    os.makedirs(os.path.join(args.out, "videos"), exist_ok=True)
    rng = np.random.default_rng(args.seed)
    env = E6PickEnv(img_size=IMG_SIZE)
    t0 = time.time()
    errors, saved = [], 0

    try:
        while saved < args.episodes:
            task_idx = saved % len(TASKS)
            color, zone, prompt = TASKS[task_idx]
            cubes = sample_cubes(rng)
            light = float(rng.uniform(light_lo, light_hi))
            env.reset(cubes, color, zone, light=light)
            actions = generate_expert_actions(env, color, zone)
            if actions is None:
                continue

            states, eye_frames, wrist_frames = [], [], []
            for act in actions:
                states.append(env.get_state())
                eye_rgb, wrist_rgb = env.render_cameras()
                eye_frames.append(eye_rgb)
                wrist_frames.append(wrist_rgb)
                env.step(act)

            ok, err_mm = env.is_success()
            if not ok:
                print(f"[跳过] 任务 {color}->{zone} 未放稳（偏差 {err_mm:.1f} mm）")
                continue

            states_arr = np.asarray(states, dtype=np.float32)
            ep_tag = f"{saved:04d}"
            np.savez(
                os.path.join(args.out, f"ep_{ep_tag}.npz"),
                states=states_arr,
                actions=actions,
                task_index=np.int64(task_idx),
                prompt=prompt,
                target_color=color,
                target_zone=zone,
                light=np.float32(light),
                cubes=json.dumps(cubes),
            )
            save_video(os.path.join(args.out, "videos", f"eye_{ep_tag}.mp4"), eye_frames)
            save_video(os.path.join(args.out, "videos", f"wrist_{ep_tag}.mp4"), wrist_frames)
            if saved == 0:
                save_preview(args.out, eye_frames, wrist_frames)

            errors.append(err_mm)
            saved += 1
            if saved <= 4 or saved % 20 == 0 or saved == args.episodes:
                print(f"[{saved}/{args.episodes}] {color}->{zone} 成功（偏差 {err_mm:.2f} mm，灯光 {light:.2f}，耗时 {time.time() - t0:.1f} s）",
                      flush=True)
    finally:
        env.close()

    meta = {
        "fps": FPS,
        "img_size": IMG_SIZE,
        "episodes": saved,
        "frames_per_episode": int(actions.shape[0]) if saved else 0,
        "light_range": [light_lo, light_hi],
        "camera_keys": ["eye", "wrist"],
        "state_dim": 7,
        "action_dim": 7,
        "tasks": [{"task_index": i, "color": c, "zone": z, "prompt": p} for i, (c, z, p) in enumerate(TASKS)],
        "mean_err_mm": round(float(np.mean(errors)), 3) if errors else None,
        "max_err_mm": round(float(np.max(errors)), 3) if errors else None,
    }
    with open(os.path.join(args.out, "meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"✅ 采集完成：{saved} 条，平均误差 {meta['mean_err_mm']} mm，最大 {meta['max_err_mm']} mm，保存于 {args.out}")


if __name__ == "__main__":
    main()
