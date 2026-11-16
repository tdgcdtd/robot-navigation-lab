# 里程碑 5：机械臂 + 桌面 + 相机都在 MuJoCo 里（真正的闭环）

MuJoCo 扮演"真机器人和真桌面"：MoveIt（在 Windows 的 WSL 里）规划轨迹 → 这里执行 → 相机拍到执行的真实结果 → Mac 上的模型再看、再规划。

| 文件 | 作用 |
|---|---|
| `build_model.py` | 厂家的 E6 URDF → MuJoCo 模型 `assets/e6.xml`，并用 M2 的 `fk.py` 核对几何（1000 组随机关节角，误差 0.0000 mm） |
| `world.py` | 仿真世界：E6 + 桌子 + 方块（自由物体）+ 相机；坐标系和 ROS 的 `dummy_link` 一致 |
| `robot_server.py` | MuJoCo "机器人"服务（扮演控制柜）：收整条轨迹、按仿真时间执行、回报结果；也能查状态、拍照 |
| `test_suction.py` | 吸盘的本地测试（不需要 MoveIt，自带一个简单的数值 IK）：对准吸得住并能搬走，偏了 / 离远了吸不住 |
| `perception_scene.py` | 新世界里的随机场景 + 真值（"眼睛"相机、带机械臂、叠放、偏暗），测试和以后重新训练感知模型都用它 |
| `eval_perception.py` | 感知重测：M3、M4 的模型原封不动拿到新画面上考 |
| `make_dino_data.py` | 新画面的 DINO 微调数据（1500 个，重点红 / 橙）+ 混入 300 个 M3 旧数据；用 `../m3_perception/finetune.py --data ... --init ckpt/ft_neg` 接着训 |
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

## 吸盘

末端（Link6）上一根 3.8 cm 的杆，杆头是吸盘面。开吸盘时检查每个方块正对吸盘的那个面：
离吸盘面 −4 ~ +6 mm、中心偏差 ≤ 10 mm、歪 ≤ 15° 才吸得住；吸住 = 打开一个"焊接"约束，按当时的相对位置把方块固定在吸盘上。

ROS 这边：`bridge_node` 提供 `/suction` 服务（std_srvs/SetBool）；`e6_motion` 的 `attach` 先调它，
**没吸住就如实失败**，不在 MoveIt 里假装抓到了。没有这个服务时（假硬件、Gazebo）行为和以前一样。

实测（11-16）：`pick_place red_cube left_zone` 全程 7 段轨迹，红色方块最后离目标 0.0 mm、平放在桌上（在 Mac 上直接读 MuJoCo 状态核对）。
第一次联调时吸盘杆做长了 2 cm，把方块压歪，吸盘报"离吸盘面 −7.6 mm，偏 3.6 mm，歪 5.1°"并失败——这就是闭环要的真实反馈。

## 感知重测（11-16）：旧模型在新画面上还剩多少

新画面 = 多了机械臂 + 换了相机（"眼睛"相机，左右和机械臂一致）。模型没有改，100 个测试场景，333 个方块。

| M3：DINO + SAM + 定位 | 旧场景（M3 测试集） | 新画面 |
|---|---|---|
| 顶面露着的方块：检测对 | 100% | 96%（错的 9 次全是橙色没认出来） |
| 顶面露着的方块：3D 误差 < 10 mm | 97% | 96%（检测对的中位误差 1.1 mm） |
| 被压在下面的方块：检测对 / 3D | 训练和测试里都没有 | 57% / 0% |
| 拒绝不存在的颜色 | 100% | 84%（错的 13 次是把橙色当成红色） |

| M4：VLM 场景描述 | 旧场景（M4 测试集） | 新画面 |
|---|---|---|
| 场景全对 | 93% | 91% |
| 叠放关系对 | 93/94 | 86/90 |
| position 对 | 98% | 98% |
| point 中位误差 | 1 像素 | 1 像素 |

**用新画面数据接着微调 DINO 之后**（`ckpt/ft_m5`，49 分钟，验证 loss 0.472 → 0.177）：

| | 微调前 | 微调后 |
|---|---|---|
| 顶面露着：检测对 / 3D < 10 mm | 96% / 96% | **100% / 100%**（中位误差 1.1 mm） |
| 红 / 橙存在时认出 | 37/37、32/41 | 37/37、**41/41** |
| 拒绝不存在的颜色 | 84/100 | **97/100** |
| 被压在下面：检测对 / 3D | 57% / 0% | 89% / 0% |
| M3 旧测试集（遗忘检查）：检测对 | 100% | 100%（3D 轮廓 97%，没变） |
| M3 旧拒绝测试：不存在的颜色 / 无关物体 | 100% / 100% | 98% / 100% |

结论：多了机械臂、换了相机，两个模型大体扛得住。真正的问题是 M3 的红 / 橙分界在新光照下偏了，以及被压住的方块（M3 的训练数据里没有叠放；定位算法假设顶面看得见）。

## 直接用 URDF 做物理会"炸"，做了三处处理（world.py 的 `_stabilize`）

1. **碰撞分组**：机械臂只和方块碰撞。相邻连杆的 STL 在关节处重叠、底座陷进桌面，不关掉会被物理引擎互相推开。
2. **armature + 阻尼**：URDF 里连杆总共 0.94 kg（真机约 7 kg），很轻的连杆配很硬的伺服会越振越大；armature 表示电机转子经减速器折算到关节的惯量。
3. **重力补偿**：纯位置伺服顶不住重力，joint2 会下垂 0.23°（末端偏 2 mm）；要在 compile 之前设。
