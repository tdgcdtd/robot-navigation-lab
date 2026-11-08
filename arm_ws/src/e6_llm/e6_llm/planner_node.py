"""planner_node：一句话 → 大模型 → 校验 → 一条条指令。

订阅  /voice_text   (std_msgs/String)  一句话（现在由 text_input 从键盘输入，以后换成语音识别）
订阅  /world_state  (std_msgs/String)  各物体现在在哪（task_executor 发的），每次调用大模型都写进提示词
发布  /task_command (std_msgs/String)  每一步一条，例如 "pick_place red_cube left_zone"
发布  /task_status  (std_msgs/String)  给人看的进度（"[理解] ..."）

参数：
    backend   "auto"（默认：有 LLM_API_KEY 就用大模型，否则用规则版）/ "llm" / "rule"
    objects.<id>.label、targets.<id>.label   来自 config/world.yaml，和 task_executor 用同一份
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from e6_llm import llm_client
from e6_llm.command_parser import (
    CommandError,
    build_system_prompt,
    parse_world_state,
    parse_llm_reply,
    rule_based_parse,
)


def read_labels(node, prefix):
    """把参数 objects.red_cube.label = "红色方块" 读成 {"red_cube": "红色方块"}。"""
    labels = {}
    for name, param in node.get_parameters_by_prefix(prefix).items():  # name = "red_cube.label"
        oid, _, key = name.partition(".")
        if key == "label":
            labels[oid] = param.value
    return labels


class PlannerNode(Node):
    def __init__(self):
        # 自动声明 YAML 里出现的所有参数（objects.* / targets.* 的个数不固定，没法一个个手动声明）
        super().__init__("planner_node", automatically_declare_parameters_from_overrides=True)
        self.objects = read_labels(self, "objects")
        self.targets = read_labels(self, "targets")

        backend = self.get_parameter_or("backend", rclpy.Parameter("backend", value="auto")).value
        if backend == "auto":
            backend = "llm" if llm_client.has_api_key() else "rule"
        self.backend = backend
        self.world_state = None  # 各物体现在在哪，来自 task_executor 的 /world_state；None = 还没收到

        self.command_pub = self.create_publisher(String, "/task_command", 10)
        self.status_pub = self.create_publisher(String, "/task_status", 10)
        self.create_subscription(String, "/voice_text", self.on_text, 10)
        # 和发布端一样用 transient_local：晚启动也能收到最后一次的状态
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/world_state", self.on_world_state, latched)

        if not self.objects or not self.targets:
            self.get_logger().error("参数里没有 objects / targets（launch 时加载 world.yaml 了吗？）")
        mode = "大模型" if self.backend == "llm" else "规则版（没有 LLM_API_KEY，只能听懂最简单的句子）"
        self.get_logger().info(
            f"准备好了：{mode}；物体 {list(self.objects)}，区域 {list(self.targets)}")

    def on_text(self, msg):
        text = msg.data.strip()
        if not text:
            return
        self.report(f"收到：{text}")
        start = time.monotonic()
        try:
            if self.backend == "llm":
                # 每次都用最新状态重新生成提示词：大模型只知道你告诉它的东西
                prompt = build_system_prompt(self.objects, self.targets, self.world_state)
                raw = llm_client.chat_json(prompt, text)
                self.get_logger().info(f"大模型原始回答：{raw}")
                commands, reply = parse_llm_reply(raw, self.objects, self.targets)
            else:
                commands, reply = rule_based_parse(text, self.objects, self.targets)
        except llm_client.LlmError as e:
            self.report(f"大模型调用失败：{e}")
            return
        except CommandError as e:
            # 校验没通过：一条指令都不发（要么全做，要么全不做）
            self.report(f"大模型的回答不合格，已拒绝：{e}")
            return

        elapsed = time.monotonic() - start
        if not commands:
            self.report(f"不执行：{reply}（{elapsed:.1f} s）")
            return
        self.report(f"理解为：{reply}（{len(commands)} 步，用时 {elapsed:.1f} s）")
        for command in commands:
            self.command_pub.publish(String(data=command))

    def on_world_state(self, msg):
        self.world_state = parse_world_state(msg.data)
        self.get_logger().info(f"当前状态：{self.world_state}")

    def report(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=f"[理解] {text}"))


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
