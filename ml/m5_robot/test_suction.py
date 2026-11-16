"""吸盘的本地测试（不需要 MoveIt）：对准了吸得住并能搬走；偏了、离远了吸不住。

没有 MoveIt 就没有人帮我们算逆运动学，这里用一个最简单的数值 IK：
反复"看吸盘离目标差多少 → 用雅可比算关节该怎么微调"，直到对上。只用于测试。

用法：../.venv/bin/python test_suction.py
"""
import mujoco
import numpy as np
import robot_server, world


def ik(robot, target, q0):
    """让吸盘面到 target（世界坐标）并且朝正下方。返回关节角。"""
    m = robot.model
    d = mujoco.MjData(m)
    q = np.array(q0, float)
    site = m.site("suction").id
    for _ in range(300):
        d.qpos[robot.adr] = q
        mujoco.mj_forward(m, d)
        axis = d.site_xmat[site].reshape(3, 3)[:, 2]
        err = np.concatenate([target - d.site_xpos[site], np.cross(axis, [0, 0, -1])])   # 位置差 + 朝向差
        if np.linalg.norm(err) < 1e-5:
            break
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jp, jr, site)
        J = np.concatenate([jp, jr])[:, :6]
        q += 0.5 * J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)                  # 阻尼最小二乘
    return q


def move(robot, q_to, seconds=2.0):
    q_from = robot.q()
    s = np.linspace(0, 1, 30); s = 3 * s**2 - 2 * s**3
    return robot.handle(dict(cmd="traj", t=(s * 0 + np.linspace(0, seconds, 30)).tolist(),
                             q=(q_from + s[:, None] * (q_to - q_from)).tolist()))


def cube_pos(robot, name):
    return np.array(robot.handle(dict(cmd="state"))["cubes"][name])


def trial(title, dx=0.0, dz=0.0):
    """去抓红色方块，吸盘相对"正确位置"偏 dx（水平）/ dz（抬高）。然后抬起 10 cm 看方块跟不跟。"""
    robot = robot_server.Robot({"red": (0.30, 0.06), "blue": (0.30, -0.06)})
    top = np.array([0.30 + dx, 0.06, world.CUBE + 0.002 + dz])           # 吸盘面的目标：方块顶面上方 2 mm
    q_above = ik(robot, top + [0, 0, 0.10], [-1.2, 0.3, 1.2, 0.0, -1.57, 0.0])
    q_down = ik(robot, top, q_above)
    move(robot, q_above); move(robot, q_down, 1.5)
    rep = robot.handle(dict(cmd="suction", on=True))
    z0 = cube_pos(robot, "red_cube")[2]
    move(robot, q_above, 1.5)
    lifted = cube_pos(robot, "red_cube")[2] - z0
    print(f"{title}：{rep['report']['red_cube']} → 吸住 {rep['held']}，抬起后方块升高 {lifted * 1000:.1f} mm")
    return robot, q_above, rep["held"], lifted


if __name__ == "__main__":
    robot, q_above, held, lifted = trial("① 对准")
    assert held == "red_cube" and abs(lifted - 0.10) < 0.003
    # 接着搬到 (0.22, 0.20) 放下
    place = np.array([0.22, 0.20, world.CUBE + 0.002])
    q2_above = ik(robot, place + [0, 0, 0.10], q_above); q2_down = ik(robot, place, q2_above)
    move(robot, q2_above, 2.5); move(robot, q2_down, 1.5)
    released = robot.handle(dict(cmd="suction", on=False))["held"]
    move(robot, q2_above, 1.5)
    p = cube_pos(robot, "red_cube")
    print(f"   搬到 (0.22, 0.20) 放下：松开了 {released}，方块最后在 ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})，"
          f"离目标 {np.linalg.norm(p[:2] - place[:2]) * 1000:.1f} mm")
    robot.handle(dict(cmd="render", path="suction_placed.png"))
    assert np.linalg.norm(p[:2] - place[:2]) < 0.003 and abs(p[2] - 0.02) < 0.002

    _, _, held, lifted = trial("② 水平偏 15 mm", dx=0.015)
    assert held is None and abs(lifted) < 0.002
    _, _, held, lifted = trial("③ 离顶面 20 mm", dz=0.018)
    assert held is None and abs(lifted) < 0.002
    _, _, held, lifted = trial("④ 水平偏 6 mm（在容差内）", dx=0.006)
    assert held == "red_cube"
    print("✅ 全部通过")
