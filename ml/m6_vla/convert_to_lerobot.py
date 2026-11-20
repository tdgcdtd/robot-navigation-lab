"""将 M6 采集的 E6 双视角原始数据集（npz + mp4）转换为标准 HuggingFace LeRobotDataset 格式。

特点：
1. 校验每条轨迹的 eye/wrist 视频帧数、状态维度（7D）与动作维度（7D）及数值有效性。
2. 兼容 LeRobot 0.3 / 0.4（v3.0 分片格式）的 `add_frame` 与 `finalize` API。
3. 自动向 `meta/stats.json` 补全 `observation.state` 与 `action` 的精确分位点统计量（`q01`~`q99`），
   确保 π0.5 等采用 QUANTILES 归一化的模型可直接训练。

用法：
  # 本地快速校验（不依赖 lerobot）
  ../.venv/bin/python convert_to_lerobot.py --src data/e6_pick --dry-run

  # 在 Colab 上转为标准 LeRobotDataset
  python convert_to_lerobot.py --src /content/work/e6_pick --dst /content/lerobot_e6_pick --repo-id local/e6_pick --overwrite
"""
import argparse
import inspect
import json
import os
import shutil
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

JOINT_AND_SUCTION_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "suction"]
QUANTILES = {"q01": 0.01, "q10": 0.10, "q50": 0.50, "q90": 0.90, "q99": 0.99}


def _read_video_frames(path):
    """读取 mp4 为 uint8 numpy 帧列表，每帧形状 (H, W, 3)。"""
    frames = []
    with imageio.get_reader(path) as reader:
        for frame in reader:
            frames.append(np.asarray(frame, dtype=np.uint8))
    return frames


def _load_and_validate_episode(src_dir, idx, expected_img_size):
    """加载并校验单条轨迹，返回 (eye_frames, wrist_frames, states, actions, prompt)。"""
    tag = f"{idx:04d}"
    npz_path = src_dir / f"ep_{tag}.npz"
    eye_path = src_dir / "videos" / f"eye_{tag}.mp4"
    wrist_path = src_dir / "videos" / f"wrist_{tag}.mp4"
    for p in (npz_path, eye_path, wrist_path):
        if not p.is_file():
            raise FileNotFoundError(f"缺少数据文件: {p}")

    with np.load(npz_path, allow_pickle=False) as ep:
        states = np.asarray(ep["states"], dtype=np.float32)
        actions = np.asarray(ep["actions"], dtype=np.float32)
        prompt = str(ep["prompt"].item() if getattr(ep["prompt"], "ndim", 0) == 0 else ep["prompt"])

    if states.ndim != 2 or states.shape[1] != 7:
        raise ValueError(f"{npz_path} states 形状异常: {states.shape}")
    if actions.shape != states.shape:
        raise ValueError(f"{npz_path} actions 形状 {actions.shape} 与 states {states.shape} 不一致")
    if not (np.isfinite(states).all() and np.isfinite(actions).all()):
        raise ValueError(f"{npz_path} 存在 NaN 或 Inf")

    eye_frames = _read_video_frames(eye_path)
    wrist_frames = _read_video_frames(wrist_path)
    n = len(states)
    if len(eye_frames) != n or len(wrist_frames) != n:
        raise ValueError(f"ep_{tag} 帧数不一致: state={n}, eye={len(eye_frames)}, wrist={len(wrist_frames)}")
    expected_shape = (expected_img_size, expected_img_size, 3)
    if eye_frames[0].shape != expected_shape or wrist_frames[0].shape != expected_shape:
        raise ValueError(f"ep_{tag} 图像尺寸异常: eye={eye_frames[0].shape}, wrist={wrist_frames[0].shape}")

    return eye_frames, wrist_frames, states, actions, prompt


def _compute_stats(arr):
    """计算 2D 数组 (N, D) 的基础统计量与分位点。"""
    out = {
        "mean": np.mean(arr, axis=0).tolist(),
        "std": np.std(arr, axis=0).tolist(),
        "min": np.min(arr, axis=0).tolist(),
        "max": np.max(arr, axis=0).tolist(),
        "count": [int(arr.shape[0])],
    }
    for name, q in QUANTILES.items():
        out[name] = np.quantile(arr, q, axis=0).tolist()
    return out


def _ensure_quantile_stats(dst_dir, state_stats, action_stats):
    """确保 meta/stats.json 包含 π0.5 所需的 q01~q99 分位点统计。"""
    meta_dir = dst_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    stats_path = meta_dir / "stats.json"
    stats = json.loads(stats_path.read_text(encoding="utf-8")) if stats_path.is_file() else {}
    for key, computed in (("observation.state", state_stats), ("action", action_stats)):
        entry = stats.setdefault(key, {})
        for stat_name, val in computed.items():
            if stat_name in QUANTILES or stat_name not in entry:
                entry[stat_name] = val
    stats_path.write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
    return stats_path


def _import_lerobot_dataset():
    for mod in ("lerobot.datasets", "lerobot.datasets.lerobot_dataset", "lerobot.common.datasets.lerobot_dataset"):
        try:
            return getattr(__import__(mod, fromlist=["LeRobotDataset"]), "LeRobotDataset")
        except (ImportError, AttributeError):
            continue
    raise ImportError("无法导入 LeRobotDataset，请确认已安装 lerobot[dataset]")


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="转换 E6 原始数据集为 LeRobotDataset")
    ap.add_argument("--src", type=Path, default=here / "data" / "e6_pick", help="原始数据集目录")
    ap.add_argument("--dst", type=Path, default=here / "data" / "lerobot_e6_pick", help="输出 LeRobot 数据集目录")
    ap.add_argument("--repo-id", default="local/e6_pick", help="LeRobot 数据集标识名")
    ap.add_argument("--dry-run", action="store_true", help="仅校验文件并打印分位点统计，不调用 lerobot")
    ap.add_argument("--overwrite", action="store_true", help="若目标目录已存在则先清空重建")
    args = ap.parse_args()

    meta_path = args.src / "meta.json"
    if not meta_path.is_file():
        raise FileNotFoundError(f"未找到 meta.json: {meta_path}")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    n_episodes = int(meta["episodes"])
    fps = int(meta["fps"])
    img_size = int(meta["img_size"])

    all_states, all_actions = [], []
    total_frames = 0
    dataset = None
    add_frame_takes_task = False

    if not args.dry_run:
        if args.dst.exists():
            if not args.overwrite:
                raise FileExistsError(f"目标目录已存在（如需重建请加 --overwrite）: {args.dst}")
            shutil.rmtree(args.dst)

        LeRobotDataset = _import_lerobot_dataset()
        features = {
            "observation.images.eye": {
                "dtype": "video",
                "shape": (img_size, img_size, 3),
                "names": ["height", "width", "channel"],
            },
            "observation.images.wrist": {
                "dtype": "video",
                "shape": (img_size, img_size, 3),
                "names": ["height", "width", "channel"],
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (7,),
                "names": JOINT_AND_SUCTION_NAMES,
            },
            "action": {
                "dtype": "float32",
                "shape": (7,),
                "names": JOINT_AND_SUCTION_NAMES,
            },
        }
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=fps,
            robot_type="dobot_e6",
            features=features,
            root=args.dst,
            use_videos=True,
        )
        add_frame_takes_task = "task" in inspect.signature(dataset.add_frame).parameters

    for idx in range(n_episodes):
        eye_frames, wrist_frames, states, actions, prompt = _load_and_validate_episode(args.src, idx, img_size)
        all_states.append(states)
        all_actions.append(actions)
        total_frames += len(states)

        if dataset is not None:
            for t in range(len(states)):
                frame = {
                    "observation.images.eye": eye_frames[t],
                    "observation.images.wrist": wrist_frames[t],
                    "observation.state": states[t],
                    "action": actions[t],
                }
                if add_frame_takes_task:
                    dataset.add_frame(frame, task=prompt)
                else:
                    frame["task"] = prompt
                    dataset.add_frame(frame)
            dataset.save_episode()

        if (idx + 1) % 40 == 0 or idx + 1 == n_episodes:
            mode = "校验" if args.dry_run else "写入"
            print(f"[{mode} {idx + 1}/{n_episodes}] 累计 {total_frames} 帧", flush=True)

    states_cat = np.concatenate(all_states, axis=0)
    actions_cat = np.concatenate(all_actions, axis=0)
    state_stats = _compute_stats(states_cat)
    action_stats = _compute_stats(actions_cat)

    print(f"✅ 数据完整：{n_episodes} 条轨迹，共 {total_frames} 帧（{fps} Hz，{img_size}x{img_size}）")
    print("state  q01:", [round(x, 4) for x in state_stats["q01"]])
    print("state  q99:", [round(x, 4) for x in state_stats["q99"]])
    print("action q01:", [round(x, 4) for x in action_stats["q01"]])
    print("action q99:", [round(x, 4) for x in action_stats["q99"]])

    if dataset is not None:
        if hasattr(dataset, "finalize"):
            dataset.finalize()
        stats_path = _ensure_quantile_stats(args.dst, state_stats, action_stats)
        print(f"✅ LeRobotDataset 已保存至 {args.dst}（分位点已写入 {stats_path}）")


if __name__ == "__main__":
    main()
