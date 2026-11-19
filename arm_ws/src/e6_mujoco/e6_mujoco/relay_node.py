"""传话节点：让 Mac 上的"大脑"（ml/m5_robot/brain.py）能给 task_executor 下指令、听它汇报。

Mac 上没有 ROS，没法直接发 / 收话题。这个节点主动连到 Mac 上的大脑（TCP，一行一个 JSON），两头传话：
  大脑 → 这里：{"cmd": "pick_place red_cube left_zone"}  → 发到 /task_command
  这里 → 大脑：/task_status 的每一条 → {"status": "[执行] 完成：..."}
              /world_state 的每一条 → {"world": "red_cube left_zone\\n..."}
为什么由这边主动连 Mac：和 bridge_node 一样，容器 → Mac 这个方向在 Tailscale 上是通的。

参数  brain_host / brain_port：Mac 的地址和端口（默认 5601）
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String


class Relay(Node):
    def __init__(self):
        super().__init__("brain_relay")
        self.host = self.declare_parameter("brain_host", "").value
        self.port = self.declare_parameter("brain_port", 5601).value
        if not self.host:
            raise RuntimeError("没有设置参数 brain_host（Mac 的地址）")
        self.lock = threading.Lock()
        self.sock = None
        self.command_pub = self.create_publisher(String, "/task_command", 10)
        self.create_subscription(String, "/task_status", lambda m: self.send({"status": m.data}), 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/world_state", lambda m: self.send({"world": m.data}), latched)
        self.last_world = None
        threading.Thread(target=self.read_loop, daemon=True).start()

    def send(self, msg):
        if "world" in msg:
            self.last_world = msg       # 大脑重连后要先知道执行端认识哪些物体
        with self.lock:
            if self.sock is None:
                return
            try:
                self.sock.sendall((json.dumps(msg, ensure_ascii=False) + "\n").encode())
            except OSError:
                self.sock = None

    def read_loop(self):
        while rclpy.ok():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=10.0)
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.sock = sock
                self.get_logger().info(f"已连上 Mac 上的大脑 {self.host}:{self.port}")
                if self.last_world:
                    self.send(self.last_world)
                for line in sock.makefile("rb"):
                    cmd = json.loads(line).get("cmd")
                    if cmd:
                        self.command_pub.publish(String(data=cmd))
                        self.get_logger().info(f"转发指令：{cmd}")
                raise ConnectionError("大脑断开了")
            except (OSError, ConnectionError, ValueError) as error:
                with self.lock:
                    self.sock = None
                self.get_logger().warn(f"和大脑的连接：{error}，5 秒后重连", throttle_duration_sec=30.0)
                time.sleep(5.0)


def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
