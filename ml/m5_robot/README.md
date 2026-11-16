# 里程碑 5：机械臂 + 桌面 + 相机都在 MuJoCo 里（真正的闭环）

MuJoCo 扮演"真机器人和真桌面"：MoveIt（在 Windows 的 WSL 里）规划轨迹 → 这里执行 → 相机拍到执行的真实结果 → Mac 上的模型再看、再规划。

| 文件 | 作用 |
|---|---|
| `build_model.py` | 厂家的 E6 URDF → MuJoCo 模型 `assets/e6.xml`，并用 M2 的 `fk.py` 核对几何（1000 组随机关节角，误差 0.0000 mm） |
| `world.py` | 仿真世界：E6 + 桌子 + 方块（自由物体）+ 相机；坐标系和 ROS 的 `dummy_link` 一致 |
| `robot_server.py` | MuJoCo "机器人"服务（扮演控制柜）：收整条轨迹、按仿真时间执行、回报结果；也能查状态、拍照 |
| `arm_ws/src/e6_mujoco/` | ROS2 这边的桥接：`bridge_node` 接 MoveIt 的 `FollowJointTrajectory` 并转交给 MuJoCo；`moveit_mujoco.launch.py` 启动不带假硬件的 MoveIt |

`assets/`（URDF 和 STL，18 MB，来自 Dobot 的 MIT 许可仓库 `arm_ws/src/DOBOT_6Axis_ROS2_V4`）不进 git。
从 Windows 取：见 `docs/remote-windows.md`，把 `dobot_rviz/urdf/me6_robot.urdf` 和 `dobot_rviz/meshes/me6/` 拷到 `assets/dobot_rviz/` 下。

```bash
../.venv/bin/python build_model.py   # 生成 assets/e6.xml + 核对正运动学
../.venv/bin/python world.py         # 渲染 world_home.png，物理跑 2 秒检查稳定性
```

## 让 MoveIt 指挥 MuJoCo（两台电脑）

```bash
# Mac
../.venv/bin/python robot_server.py
# Windows 的容器里（sim_host = Mac 的 Tailscale 地址）
ros2 launch e6_mujoco moveit_mujoco.launch.py sim_host:=<地址>
ros2 launch e6_motion move_demo.launch.py use_sim_time:=false
```

实测（11-16）：MoveIt 规划 72 个路点、7.1 秒的轨迹 → MuJoCo 执行，途中跟踪误差最大 0.78°，停稳后末端离目标 0.10 mm。

**为什么整条轨迹一次发过去**：两台电脑之间的网络往返约 230 ms 还丢包（校园网 + Tailscale），做不了每秒几十次的实时控制。
所以网络只在动作开始和结束各用一次，执行全在 Mac 本地按仿真时间完成；`/joint_states` 在执行途中由桥接节点按轨迹推算，执行完用真实值校正。
MoveIt 的执行超时放宽到"3 倍 + 10 秒"，否则网络一慢就被误判成超时。

## 直接用 URDF 做物理会"炸"，做了三处处理（world.py 的 `_stabilize`）

1. **碰撞分组**：机械臂只和方块碰撞。相邻连杆的 STL 在关节处重叠、底座陷进桌面，不关掉会被物理引擎互相推开。
2. **armature + 阻尼**：URDF 里连杆总共 0.94 kg（真机约 7 kg），很轻的连杆配很硬的伺服会越振越大；armature 表示电机转子经减速器折算到关节的惯量。
3. **重力补偿**：纯位置伺服顶不住重力，joint2 会下垂 0.23°（末端偏 2 mm）；要在 compile 之前设。
