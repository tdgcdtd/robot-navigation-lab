"""桥接节点：把 MoveIt 发出的轨迹转交给另一台电脑上的 MuJoCo "机器人"（ml/m5_robot/robot_server.py）。

对 MoveIt 来说，这个节点就是机械臂的控制器，和原来的假硬件（mock_components）用的是同一个接口：

提供动作  /me6_group_controller/follow_joint_trajectory  (control_msgs/FollowJointTrajectory)
          MoveIt 把整条轨迹交过来 → 这里一次性发给 MuJoCo → 等它执行完 → 告诉 MoveIt 成功 / 失败
发布      /joint_states  (sensor_msgs/JointState)  50 Hz
          空闲时：MuJoCo 回报的真实关节角
          执行中：按轨迹和时间算出"此刻应该在哪"（网络往返 230 ms，做不到实时回传；执行完再用真实值校正）

参数  sim_host / sim_port：MuJoCo 服务的地址（那台电脑的 Tailscale 地址）

为什么不实时回传：见 robot_server.py 开头。真机械臂也是控制柜收下整条轨迹自己执行。
"""

import json
import socket
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 7)]
FINAL_TOLERANCE = 0.02  # 弧度（约 1.1°）：执行完和目标差得比这多，就算失败


def interpolate(ts, qs, now):
    """轨迹上 now 时刻（秒）应该在的关节角：相邻两个路点之间线性插值。"""
    if now <= ts[0]:
        return list(qs[0])
    for k in range(1, len(ts)):
        if now <= ts[k]:
            a = (now - ts[k - 1]) / max(ts[k] - ts[k - 1], 1e-9)
            return [p + a * (n - p) for p, n in zip(qs[k - 1], qs[k])]
    return list(qs[-1])


class SimLink:
    """和 MuJoCo 服务的连接：一问一答，一行一个 JSON。断了就重连。"""

    def __init__(self, host, port, logger):
        self.host, self.port, self.logger = host, port, logger
        self.lock = threading.Lock()  # 同一时刻只允许一个请求在路上
        self.file = None

    def _connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=10.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = sock
        self.file = sock.makefile("rwb")

    def request(self, payload, timeout):
        with self.lock:
            for attempt in (1, 2):  # 连接断了：重连后再试一次
                try:
                    if self.file is None:
                        self._connect()
                    self.sock.settimeout(timeout)
                    self.file.write((json.dumps(payload) + "\n").encode())
                    self.file.flush()
                    line = self.file.readline()
                    if not line:
                        raise ConnectionError("对方关闭了连接")
                    return json.loads(line)
                except (OSError, ConnectionError) as error:
                    self.file = None
                    if attempt == 2:
                        raise
                    self.logger.warn(f"和 MuJoCo 的连接出错（{error}），重连")


class Bridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        host = self.declare_parameter("sim_host", "").value
        port = self.declare_parameter("sim_port", 5600).value
        if not host:
            raise RuntimeError("没有设置参数 sim_host（MuJoCo 那台电脑的地址）")
        self.sim = SimLink(host, port, self.get_logger())

        self.q = None        # 最近一次 MuJoCo 回报的真实关节角
        self.active = None   # 正在执行的轨迹：(开始时刻, 时间列表, 关节角列表)
        group = ReentrantCallbackGroup()  # 执行轨迹要等好几秒，期间定时器还得照常发 /joint_states
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.02, self.publish_state, callback_group=group)
        self.create_timer(2.0, self.refresh_state, callback_group=group)
        self.server = ActionServer(
            self, FollowJointTrajectory, "/me6_group_controller/follow_joint_trajectory",
            execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info(f"桥接节点启动，MuJoCo 在 {host}:{port}")

    def refresh_state(self):
        """空闲时每 2 秒问一次真实关节角（也顺便确认连接还活着）。"""
        if self.active is not None:
            return
        try:
            reply = self.sim.request({"cmd": "state"}, timeout=10.0)
            if self.q is None:
                self.get_logger().info("已连上 MuJoCo，开始发布 /joint_states")
            self.q = reply["q"]
        except (OSError, ConnectionError, ValueError) as error:
            self.get_logger().warn(f"问不到 MuJoCo 的状态：{error}", throttle_duration_sec=10.0)

    def publish_state(self):
        if self.q is None:
            return  # 还没连上：不知道机械臂在哪，宁可不发也不乱发
        active = self.active
        q = self.q if active is None else interpolate(active[1], active[2], time.monotonic() - active[0])
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(v) for v in q]
        self.state_pub.publish(msg)

    def execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        order = [traj.joint_names.index(j) for j in JOINTS]  # MoveIt 给的关节顺序不一定是 1~6，按名字对齐
        ts = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points]
        qs = [[p.positions[i] for i in order] for p in traj.points]
        result = FollowJointTrajectory.Result()
        self.get_logger().info(f"收到轨迹：{len(ts)} 个路点，时长 {ts[-1]:.2f} s，转交给 MuJoCo")

        start = time.monotonic()
        self.active = (start, ts, qs)
        try:
            reply = self.sim.request({"cmd": "traj", "t": ts, "q": qs}, timeout=ts[-1] + 60.0)
        except (OSError, ConnectionError, ValueError) as error:
            self.active = None
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"MuJoCo 没有回应：{error}"
            self.get_logger().error(result.error_string)
            goal_handle.abort()
            return result
        # MuJoCo 算得比真实时间快：等到轨迹本来该花的时间过完再报告，RViz 里的动画才是正常速度
        time.sleep(max(0.0, ts[-1] - (time.monotonic() - start)))
        if reply.get("ok"):
            self.q = reply["q"]  # 用真实结果校正
        self.active = None

        if not reply.get("ok"):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(reply.get("error"))
            goal_handle.abort()
        elif reply["final_err"] > FINAL_TOLERANCE:
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f"没走到目标：差 {reply['final_err']:.3f} 弧度"
            goal_handle.abort()
        else:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        self.get_logger().info(
            f"执行完：跟踪误差最大 {reply.get('track_err', 0):.3f} 弧度，停稳后误差 {reply.get('final_err', 0):.4f} 弧度，"
            f"总耗时 {time.monotonic() - start:.2f} s（轨迹本身 {ts[-1]:.2f} s）{' ' + result.error_string if result.error_string else ''}")
        return result


def main():
    rclpy.init()
    node = Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
