"""闭环评测 E6 VLA 策略（同时支持确定性专家基线与微调后的 SmolVLA / π0.5 检查点）。

用法：
  # 1. 用专家策略跑评测闭环，验证评测环境与视频录制无误（不依赖 lerobot）
  ../.venv/bin/python eval_vla.py --checkpoint expert --episodes 8 --seed 1000 --light-range 0.85 1.15

  # 2. 在 Mac 上实时 3D 窗口评测从 Colab 训好下载的 SmolVLA 模型
  ../.venv/bin/mjpython eval_vla.py --checkpoint ckpts/smolvla_e6/pretrained_model --episodes 20 --gui

  # 3. 在 Colab 无头环境评测 π0.5 LoRA 模型并导出三视角对比视频
  MUJOCO_GL=egl python eval_vla.py --checkpoint outputs/pi05_lora_e6/pretrained_model --episodes 20 --device cuda --save-video rollout.mp4
"""
import argparse
import contextlib
import json
import time
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

from collect_data import sample_cubes
from env import E6PickEnv, FPS, IMG_SIZE, TASKS, world
from expert import generate_expert_actions

SETTLE_SUCCESS_FRAMES = 5


def _resolve_device(device_arg):
    import torch

    if device_arg != "auto":
        return torch.device(device_arg)
    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def _load_json_from_ckpt(checkpoint, filename):
    ckpt_path = Path(checkpoint)
    if ckpt_path.is_dir():
        target = ckpt_path / filename
        return json.loads(target.read_text(encoding="utf-8")) if target.is_file() else None
    try:
        from huggingface_hub import hf_hub_download

        downloaded = Path(hf_hub_download(checkpoint, filename))
        return json.loads(downloaded.read_text(encoding="utf-8"))
    except Exception:
        return None


def _import_factory():
    for mod in ("lerobot.policies", "lerobot.policies.factory", "lerobot.common.policies.factory"):
        try:
            m = __import__(mod, fromlist=["get_policy_class", "make_pre_post_processors"])
            return getattr(m, "get_policy_class"), getattr(m, "make_pre_post_processors")
        except (ImportError, AttributeError):
            continue
    raise ImportError("无法导入 lerobot 策略工厂，请确认已安装 lerobot[training]")


def _fallback_policy_class(policy_type):
    candidates = {
        "smolvla": [("lerobot.policies.smolvla", "SmolVLAPolicy"), ("lerobot.policies.smolvla.modeling_smolvla", "SmolVLAPolicy")],
        "pi05": [("lerobot.policies.pi05", "PI05Policy"), ("lerobot.policies.pi05.modeling_pi05", "PI05Policy")],
        "pi0": [("lerobot.policies.pi0", "PI0Policy"), ("lerobot.policies.pi0.modeling_pi0", "PI0Policy")],
    }.get(policy_type)
    if not candidates:
        raise ValueError(f"未知的 policy_type: {policy_type}")
    for mod, cls_name in candidates:
        try:
            return getattr(__import__(mod, fromlist=[cls_name]), cls_name)
        except (ImportError, AttributeError):
            continue
    raise ImportError(f"无法加载策略类 {policy_type}，请确认已安装对应依赖")


def _detect_camera_mapping(checkpoint, policy_config):
    """自动匹配模型的相机输入键名 -> ('eye', 'wrist')。"""
    preproc_cfg = _load_json_from_ckpt(checkpoint, "policy_preprocessor.json")
    raw_keys = []
    if preproc_cfg:
        for step in preproc_cfg.get("steps", []):
            name = step.get("registry_name", "")
            if "rename" in name:
                rename_map = step.get("config", {}).get("rename_map", {})
                img_keys = [k for k in rename_map if "observation.images." in k]
                if img_keys:
                    raw_keys = img_keys
                    break

    if not raw_keys and hasattr(policy_config, "input_features"):
        for k, feat in policy_config.input_features.items():
            feat_type = str(getattr(feat, "type", "")).upper()
            if "VISUAL" in feat_type or "observation.images." in k:
                raw_keys.append(k)

    if "observation.images.eye" in raw_keys and "observation.images.wrist" in raw_keys:
        return {"eye": "observation.images.eye", "wrist": "observation.images.wrist"}

    if not raw_keys:
        return {"eye": "observation.images.eye", "wrist": "observation.images.wrist"}
    if len(raw_keys) == 1:
        return {"eye": raw_keys[0]}

    wrist_key = next(
        (k for k in raw_keys if any(tag in k.lower() for tag in ("wrist", "camera2", "image2"))),
        raw_keys[1],
    )
    eye_key = next((k for k in raw_keys if k != wrist_key), raw_keys[0])
    return {"eye": eye_key, "wrist": wrist_key}


def _load_lerobot_runner(checkpoint, device_arg, n_action_steps):
    import torch

    device = _resolve_device(device_arg)
    cfg_json = _load_json_from_ckpt(checkpoint, "config.json")
    if not cfg_json or "type" not in cfg_json:
        raise RuntimeError(f"无法从 {checkpoint} 读取 config.json 中的模型类型")
    policy_type = cfg_json["type"]

    get_policy_class, make_pre_post_processors = _import_factory()
    try:
        PolicyCls = get_policy_class(policy_type)
    except Exception:
        PolicyCls = _fallback_policy_class(policy_type)

    policy = PolicyCls.from_pretrained(checkpoint).to(device)
    policy.eval()

    if hasattr(policy.config, "n_action_steps"):
        chunk_size = int(getattr(policy.config, "chunk_size", n_action_steps))
        eff_steps = min(int(n_action_steps), chunk_size)
        policy.config.n_action_steps = eff_steps
        if hasattr(policy, "config") and hasattr(policy, "_queues"):
            policy.reset()
        print(f"[配置] n_action_steps 设为 {eff_steps}（chunk_size={chunk_size}）", flush=True)

    try:
        preprocess, postprocess = make_pre_post_processors(
            policy.config,
            pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
    except TypeError:
        preprocess, postprocess = make_pre_post_processors(
            policy.config,
            checkpoint,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )

    cam_map = _detect_camera_mapping(checkpoint, policy.config)
    use_bf16 = device.type == "cuda"
    amp_ctx = (
        (lambda: torch.autocast("cuda", dtype=torch.bfloat16))
        if use_bf16
        else contextlib.nullcontext
    )
    dtype_label = "bfloat16" if use_bf16 else "float32"
    print(f"[模型] 已加载 {policy_type} 于 {device}（{dtype_label}），相机映射: {cam_map}", flush=True)

    class _Runner:
        def reset(self, env, color, zone):
            if hasattr(policy, "reset"):
                policy.reset()

        def predict(self, env, state, prompt, step_idx):
            use_fast_queue = (
                getattr(policy.config, "n_obs_steps", 1) == 1
                and hasattr(policy, "_queues")
                and len(policy._queues.get("action", ())) > 0
            )
            if use_fast_queue:
                with torch.inference_mode():
                    raw_act = policy._queues["action"].popleft()
                    act = postprocess(raw_act.float())
                return act.detach().float().cpu().numpy().reshape(-1)[:7]

            eye_rgb, wrist_rgb = env.render_cameras()
            views = {"eye": eye_rgb, "wrist": wrist_rgb}
            obs = {
                "observation.state": torch.from_numpy(state).unsqueeze(0).float().to(device),
                "task": [prompt],
            }
            for role, target_key in cam_map.items():
                img_t = torch.from_numpy(views[role]).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                obs[target_key] = img_t.to(device)
            with torch.inference_mode():
                batch = preprocess(obs)
                with amp_ctx():
                    raw_act = policy.select_action(batch)
                act = postprocess(raw_act.float())
            return act.detach().float().cpu().numpy().reshape(-1)[:7]

    return _Runner(), policy_type


def _make_expert_runner():
    class _ExpertRunner:
        def __init__(self):
            self.actions = None

        def reset(self, env, color, zone):
            self.actions = generate_expert_actions(env, color, zone)
            if self.actions is None:
                raise RuntimeError(f"专家 IK 在场景 {color}->{zone} 无解")

        def predict(self, env, state, prompt, step_idx):
            idx = min(step_idx, len(self.actions) - 1)
            return self.actions[idx]

    return _ExpertRunner(), "expert"


def _init_viewer(env):
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(env.model, env.data)
    viewer.cam.lookat[:] = world.CAM_TARGET
    viewer.cam.distance = 1.15
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -35
    viewer.sync()
    return viewer


def _reload_viewer(viewer, env):
    if viewer is not None and viewer.is_running():
        viewer._get_sim().load(env.model, env.data, "")
        viewer._m, viewer._d = env.model, env.data
        viewer.sync()


def main():
    ap = argparse.ArgumentParser(description="E6 VLA 闭环仿真评测")
    ap.add_argument("--checkpoint", required=True, help="'expert' 或训练好的模型目录 / HF 仓库 ID")
    ap.add_argument("--episodes", type=int, default=20, help="评测回合数")
    ap.add_argument("--seed", type=int, default=1000, help="与训练集错开的随机种子")
    ap.add_argument("--light-range", nargs=2, type=float, default=[1.0, 1.0],
                    metavar=("MIN", "MAX"), help="评测光照强度范围")
    ap.add_argument("--device", default="auto", help="auto / cuda / mps / cpu")
    ap.add_argument("--n-action-steps", type=int, default=5,
                    help="每次重看画面前连续执行的动作步数（10 Hz 下 5 步 = 0.5 秒重看一次）")
    ap.add_argument("--max-steps", type=int, default=80, help="单回合最大控制步数")
    ap.add_argument("--gui", action="store_true", help="打开 MuJoCo 3D 实时窗口（需用 mjpython 运行）")
    ap.add_argument("--save-video", default=None, help="可选：保存前若干回合的 cam|eye|wrist 对比 mp4")
    ap.add_argument("--video-episodes", type=int, default=4, help="视频里最多录制几个回合")
    ap.add_argument("--out-json", default=None, help="可选：输出评测统计 JSON 文件路径")
    args = ap.parse_args()

    light_lo, light_hi = float(args.light_range[0]), float(args.light_range[1])
    if light_lo > light_hi:
        raise ValueError(f"light-range MIN ({light_lo}) 不能大于 MAX ({light_hi})")

    if args.checkpoint == "expert":
        runner, model_type = _make_expert_runner()
    else:
        runner, model_type = _load_lerobot_runner(args.checkpoint, args.device, args.n_action_steps)

    rng = np.random.default_rng(args.seed)
    env = E6PickEnv(img_size=IMG_SIZE)
    viewer = None
    video_frames = []
    results = []
    infer_times_ms = []

    try:
        for ep in range(args.episodes):
            task_idx = ep % len(TASKS)
            color, zone, prompt = TASKS[task_idx]
            cubes = sample_cubes(rng)
            light = float(rng.uniform(light_lo, light_hi))
            state = env.reset(cubes, color, zone, light=light)
            runner.reset(env, color, zone)

            if args.gui:
                if viewer is None:
                    viewer = _init_viewer(env)
                else:
                    _reload_viewer(viewer, env)

            settle_ok = 0
            ep_ok, err_mm = False, 999.0
            for step in range(args.max_steps):
                t_step = time.monotonic()
                if args.save_video and ep < args.video_episodes:
                    eye_rgb, wrist_rgb = env.render_cameras()
                    cam_rgb = env.render_camera("cam")
                    video_frames.append(np.hstack([cam_rgb, eye_rgb, wrist_rgb]))

                t_inf = time.monotonic()
                action = runner.predict(env, state, prompt, step)
                infer_times_ms.append((time.monotonic() - t_inf) * 1000.0)

                state = env.step(action)
                if viewer is not None and viewer.is_running():
                    viewer.sync()
                    gap = (1.0 / FPS) - (time.monotonic() - t_step)
                    if gap > 0:
                        time.sleep(gap)

                ok_now, err_mm = env.is_success()
                settle_ok = settle_ok + 1 if ok_now else 0
                if settle_ok >= SETTLE_SUCCESS_FRAMES:
                    ep_ok = True
                    break

            if not ep_ok:
                ep_ok, err_mm = env.is_success()

            results.append({
                "episode": ep,
                "task": f"{color}->{zone}",
                "light": round(light, 3),
                "steps": step + 1,
                "success": bool(ep_ok),
                "err_mm": round(float(err_mm), 2),
            })
            mark = "✅" if ep_ok else "❌"
            print(f"[{ep + 1}/{args.episodes}] {mark} {color}->{zone} | 步数 {step + 1} | 偏差 {err_mm:.2f} mm | 灯光 {light:.2f}",
                  flush=True)
    finally:
        if viewer is not None:
            viewer.close()
        env.close()

    if args.save_video and video_frames:
        out_vid = Path(args.save_video)
        out_vid.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(out_vid, fps=FPS, codec="libx264", pixelformat="yuv420p", macro_block_size=1) as w:
            for frame in video_frames:
                w.append_data(frame)
        print(f"🎬 对比视频已保存至 {out_vid}（共 {len(video_frames)} 帧）", flush=True)

    succ = sum(r["success"] for r in results)
    per_task = {}
    for color, zone, _ in TASKS:
        key = f"{color}->{zone}"
        sub = [r for r in results if r["task"] == key]
        per_task[key] = {
            "success": sum(r["success"] for r in sub),
            "total": len(sub),
            "rate": round(sum(r["success"] for r in sub) / max(len(sub), 1), 3),
        }

    summary = {
        "checkpoint": args.checkpoint,
        "model_type": model_type,
        "episodes": len(results),
        "success": succ,
        "success_rate": round(succ / max(len(results), 1), 3),
        "mean_err_mm": round(float(np.mean([r["err_mm"] for r in results])), 3) if results else None,
        "mean_infer_ms": round(float(np.mean(infer_times_ms)), 2) if infer_times_ms else None,
        "per_task": per_task,
        "trials": results,
    }
    print(f"\n📊 评测总结 ({model_type}): 成功 {succ}/{len(results)} ({summary['success_rate'] * 100:.1f}%)"
          f" | 平均误差 {summary['mean_err_mm']} mm | 平均推理耗时 {summary['mean_infer_ms']} ms/步")
    for k, v in per_task.items():
        print(f"   - {k}: {v['success']}/{v['total']}")

    if args.out_json:
        out_p = Path(args.out_json)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        out_p.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"📝 详细结果已写入 {out_p}")


if __name__ == "__main__":
    main()
