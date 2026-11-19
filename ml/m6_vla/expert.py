"""用 MuJoCo 雅可比阻尼最小二乘数值 IK 生成 E6 专家抓取轨迹（10 Hz 关节角 + 吸盘信号）。"""
import mujoco
import numpy as np

from env import ZONES, world


def solve_ik(model, adr, target_xyz, q_seed):
    """让吸盘面到达 target_xyz 且垂直朝下，返回 6 维关节角；不收敛返回 None。"""
    d = mujoco.MjData(model)
    q = np.array(q_seed, dtype=float)
    site = model.site("suction").id
    err = np.zeros(6)
    for _ in range(300):
        d.qpos[adr] = q
        mujoco.mj_forward(model, d)
        axis = d.site_xmat[site].reshape(3, 3)[:, 2]
        err = np.concatenate([target_xyz - d.site_xpos[site], np.cross(axis, [0, 0, -1])])
        if np.linalg.norm(err) < 1e-5:
            return q
        jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, d, jp, jr, site)
        J = np.concatenate([jp, jr])[:, :6]
        q += 0.5 * J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
    return q if np.linalg.norm(err) < 1e-3 else None


def _cubic_segment(q_from, q_to, n_frames, suction):
    s = np.linspace(0.0, 1.0, n_frames + 1)[1:]
    s = 3 * s**2 - 2 * s**3
    qs = q_from + s[:, None] * (q_to - q_from)
    return np.column_stack([qs, np.full((n_frames, 1), float(suction))])


def _hold_segment(q, n_frames, suction):
    return np.column_stack([np.tile(q, (n_frames, 1)), np.full((n_frames, 1), float(suction))])


def generate_expert_actions(env, target_color, target_zone):
    """规划一条光滑的 10 Hz 专家抓放序列，返回 (T, 7) float32 数组，不可达返回 None。"""
    cube_xyz = env.data.body(f"{target_color}_cube").xpos.copy()
    tx, ty = ZONES[target_zone]
    pick_top = np.array([cube_xyz[0], cube_xyz[1], cube_xyz[2] + world.CUBE / 2 + 0.002])
    place_top = np.array([tx, ty, world.CUBE + 0.002])

    q0 = np.array(world.HOME, dtype=float)
    seed = [-1.2, 0.3, 1.2, 0.0, -1.57, 0.0]
    q_pick_above = solve_ik(env.model, env.adr, pick_top + [0.0, 0.0, 0.10], seed)
    if q_pick_above is None:
        return None
    q_pick_down = solve_ik(env.model, env.adr, pick_top, q_pick_above)
    q_place_above = solve_ik(env.model, env.adr, place_top + [0.0, 0.0, 0.10], q_pick_above)
    if q_pick_down is None or q_place_above is None:
        return None
    q_place_down = solve_ik(env.model, env.adr, place_top, q_place_above)
    if q_place_down is None:
        return None

    segments = [
        _cubic_segment(q0, q_pick_above, 10, 0),
        _cubic_segment(q_pick_above, q_pick_down, 8, 0),
        _hold_segment(q_pick_down, 3, 1),
        _cubic_segment(q_pick_down, q_pick_above, 8, 1),
        _cubic_segment(q_pick_above, q_place_above, 12, 1),
        _cubic_segment(q_place_above, q_place_down, 8, 1),
        _hold_segment(q_place_down, 3, 0),
        _cubic_segment(q_place_down, q_place_above, 8, 0),
    ]
    return np.vstack(segments).astype(np.float32)
