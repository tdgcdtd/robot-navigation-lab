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


def _matches_existing_episode(out_dir, idx, color, zone, cubes, light):
    """确认磁盘已有轨迹与当前 RNG 状态完全一致，避免 --resume 拼接错位。"""
    tag = f"{idx:04d}"
    npz_path = os.path.join(out_dir, f"ep_{tag}.npz")
    eye_path = os.path.join(out_dir, "videos", f"eye_{tag}.mp4")
    wrist_path = os.path.join(out_dir, "videos", f"wrist_{tag}.mp4")
    if not (os.path.isfile(npz_path) and os.path.isfile(eye_path) and os.path.isfile(wrist_path)):
        return False
    with np.load(npz_path, allow_pickle=False) as ep:
        ep_light = float(ep["light"].item() if getattr(ep["light"], "ndim", 0) == 0 else ep["light"])
        if abs(ep_light - float(np.float32(light))) > 1e-5:
            raise RuntimeError(f"RNG 不同步：{npz_path} 灯光为 {ep_light}，期望 {light}")
        ep_color = str(ep["target_color"].item() if getattr(ep["target_color"], "ndim", 0) == 0 else ep["target_color"])
        ep_zone = str(ep["target_zone"].item() if getattr(ep["target_zone"], "ndim", 0) == 0 else ep["target_zone"])
        if ep_color != color or ep_zone != zone:
            raise RuntimeError(f"RNG 不同步：{npz_path} 任务为 {ep_color}->{ep_zone}，期望 {color}->{zone}")
        ep_cubes = json.loads(str(ep["cubes"].item() if getattr(ep["cubes"], "ndim", 0) == 0 else ep["cubes"]))
        for k in ("red", "blue"):
            if not np.allclose(ep_cubes[k], cubes[k], atol=1e-5):
                raise RuntimeError(f"RNG 不同步：{npz_path} 方块 {k}={ep_cubes[k]}，期望 {cubes[k]}")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=160)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "e6_pick"))
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--light-range", nargs=2, type=float, default=[0.85, 1.15],
                    metavar=("MIN", "MAX"), help="灯光亮度域随机化区间")
    ap.add_argument("--resume", action="store_true", help="校验已有前缀轨迹 RNG 一致后跳过重采，仅补齐到目标总数")
    args = ap.parse_args()
    light_lo, light_hi = float(args.light_range[0]), float(args.light_range[1])
    if light_lo > light_hi:
        raise ValueError(f"light-range MIN ({light_lo}) 不能大于 MAX ({light_hi})")

    os.makedirs(os.path.join(args.out, "videos"), exist_ok=True)
    meta_path = os.path.join(args.out, "meta.json")
    prev_meta = json.loads(open(meta_path, encoding="utf-8").read()) if args.resume and os.path.isfile(meta_path) else None

    rng = np.random.default_rng(args.seed)
    env = E6PickEnv(img_size=IMG_SIZE)
    t0 = time.time()
    t_new = None
    errors, saved, reused = [], 0, 0
    frames_per_ep = int(prev_meta.get("frames_per_episode", 0)) if prev_meta else 0

    try:
        while saved < args.episodes:
            task_idx = saved % len(TASKS)
            color, zone, prompt = TASKS[task_idx]
            cubes = sample_cubes(rng)
            light = float(rng.uniform(light_lo, light_hi))

            if args.resume and _matches_existing_episode(args.out, saved, color, zone, cubes, light):
                reused += 1
                saved += 1
                if saved == args.episodes or reused % 100 == 0:
                    print(f"[复用校验 {saved}/{args.episodes}] 已核对前 {reused} 条轨迹 RNG 与文件完整一致", flush=True)
                continue

            if t_new is None:
                t_new = time.time()
                if reused > 0:
                    print(f"▶️  从第 {saved} 条起继续补采至 {args.episodes} 条（需新采 {args.episodes - saved} 条）...", flush=True)

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

            frames_per_ep = int(actions.shape[0])
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
                err_mm=np.float32(err_mm),
            )
            save_video(os.path.join(args.out, "videos", f"eye_{ep_tag}.mp4"), eye_frames)
            save_video(os.path.join(args.out, "videos", f"wrist_{ep_tag}.mp4"), wrist_frames)
            if saved == 0:
                save_preview(args.out, eye_frames, wrist_frames)

            errors.append(err_mm)
            saved += 1
            if saved <= 4 or saved % 20 == 0 or saved == args.episodes:
                elapsed = time.time() - (t_new or t0)
                done_new = len(errors)
                rem = args.episodes - saved
                eta = (elapsed / done_new * rem) if done_new > 0 else 0.0
                print(
                    f"[{saved}/{args.episodes}] {color}->{zone} 成功（偏差 {err_mm:.2f} mm，灯光 {light:.2f}，"
                    f"已用 {elapsed:.1f}s，预计剩余 {eta:.1f}s）",
                    flush=True,
                )
    finally:
        env.close()

    prev_n = int(prev_meta.get("episodes", 0)) if prev_meta and reused >= int(prev_meta.get("episodes", 0)) else 0
    if prev_n > 0 and prev_meta.get("mean_err_mm") is not None and errors:
        mean_err = (float(prev_meta["mean_err_mm"]) * prev_n + float(np.sum(errors))) / (prev_n + len(errors))
        max_err = max(float(prev_meta.get("max_err_mm") or 0.0), float(np.max(errors)))
    elif errors:
        mean_err = float(np.mean(errors))
        max_err = float(np.max(errors))
    elif prev_meta:
        mean_err = prev_meta.get("mean_err_mm")
        max_err = prev_meta.get("max_err_mm")
    else:
        mean_err, max_err = None, None

    meta = {
        "fps": FPS,
        "img_size": IMG_SIZE,
        "episodes": saved,
        "frames_per_episode": frames_per_ep,
        "light_range": [light_lo, light_hi],
        "camera_keys": ["eye", "wrist"],
        "state_dim": 7,
        "action_dim": 7,
        "tasks": [{"task_index": i, "color": c, "zone": z, "prompt": p} for i, (c, z, p) in enumerate(TASKS)],
        "mean_err_mm": round(mean_err, 3) if mean_err is not None else None,
        "max_err_mm": round(max_err, 3) if max_err is not None else None,
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"✅ 采集完成：共 {saved} 条（复用 {reused} 条，新增 {len(errors)} 条），平均误差 {meta['mean_err_mm']} mm，最大 {meta['max_err_mm']} mm，保存于 {args.out}")


if __name__ == "__main__":
    main()
