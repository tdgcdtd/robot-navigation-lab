"""把厂家的 E6 URDF 转成 MuJoCo 模型（assets/e6.xml），并核对几何有没有转错。

为什么要转：URDF 是 ROS 的机器人描述格式（MoveIt、RViz 用），MuJoCo 用自己的 MJCF。
MuJoCo 能直接读 URDF，只是要告诉它外形文件（STL）在哪。

怎么核对：M2 里的 fk.py 是从同一份 URDF 手抄的正运动学，而且用 MoveIt 的实测数据验证过。
随机取一批关节角，MuJoCo 算出的末端位置应该和 fk.py 完全一致（误差只剩浮点舍入）。

用法：../.venv/bin/python build_model.py
"""
import os, sys
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
URDF = os.path.join(ASSETS, "dobot_rviz", "urdf", "me6_robot.urdf")
MESHDIR = os.path.join(ASSETS, "dobot_rviz", "meshes", "me6")
OUT = os.path.join(ASSETS, "e6.xml")
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def urdf_to_mjcf():
    urdf = open(URDF, encoding="utf-8").read()
    # URDF 里可以塞一段 <mujoco> 设置：meshdir = STL 所在目录；
    # fusestatic=false：不要把固定连接的 dummy_link / base_link 合并掉，名字保持和 ROS 里一致；
    # discardvisual=false：保留外观；balanceinertia：SolidWorks 导出的惯量偶尔不满足物理约束，自动修正
    opts = (f'<mujoco><compiler meshdir="{MESHDIR}" fusestatic="false" discardvisual="false" '
            f'balanceinertia="true" strippath="true"/></mujoco>')
    urdf = urdf.replace("</robot>", opts + "\n</robot>")
    model = mujoco.MjModel.from_xml_string(urdf)
    mujoco.mj_saveLastXML(OUT, model)
    return model


def check_fk(model, n=1000):
    sys.path.insert(0, os.path.join(HERE, "..", "m2_training_loop"))
    from fk import forward_kinematics
    data = mujoco.MjData(model)
    link6 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link6")
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINT_NAMES]
    rng = np.random.default_rng(0)
    lo, hi = model.jnt_range[:6, 0], model.jnt_range[:6, 1]
    q = rng.uniform(lo, hi, size=(n, 6))
    q[0] = 0                                           # 第一组：全部归零
    q[1] = [0, 0.5337, 0.7276, 0.3548, -1.5848, 0]     # 第二组：home 姿态
    got = np.zeros((n, 3))
    for k in range(n):
        data.qpos[qadr] = q[k]
        mujoco.mj_kinematics(model, data)
        got[k] = data.xpos[link6]
    want = forward_kinematics(q)
    err = np.linalg.norm(got - want, axis=1) * 1000
    print(f"全部归零：MuJoCo {np.round(got[0], 4)}  fk.py {np.round(want[0], 4)}")
    print(f"home 姿态：MuJoCo {np.round(got[1], 4)}  fk.py {np.round(want[1], 4)}")
    print(f"{n} 组随机关节角：最大误差 {err.max():.4f} mm，平均 {err.mean():.5f} mm")
    return err.max()


if __name__ == "__main__":
    m = urdf_to_mjcf()
    print(f"已生成 {os.path.relpath(OUT, HERE)}：{m.nbody} 个 body，{m.njnt} 个关节，{m.nmesh} 个外形，"
          f"总质量 {m.body_mass.sum():.2f} kg")
    for j in JOINT_NAMES:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        print(f"  {j}: 范围 {np.rad2deg(m.jnt_range[i, 0]):7.1f}° ~ {np.rad2deg(m.jnt_range[i, 1]):6.1f}°")
    worst = check_fk(m)
    print("✅ 几何和 M2 的正运动学一致" if worst < 0.1 else "❌ 几何对不上，先别往下做")
