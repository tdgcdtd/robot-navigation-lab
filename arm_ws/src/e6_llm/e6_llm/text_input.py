"""text_input：在终端里打字代替说话。

输入一句话 → 发布到 /voice_text；同时把 /task_status 上的进度打印出来。
以后加了语音识别，只是把"键盘"换成"麦克风"，planner_node 一行都不用改 —— 这就是用话题解耦的好处。

用法：ros2 run e6_llm text_input   （要在自己的终端里直接运行，不能放进 launch：launch 拿不到键盘输入）
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TextInput(Node):
    def __init__(self):
        super().__init__("text_input")
        self.pub = self.create_publisher(String, "/voice_text", 10)
        self.create_subscription(String, "/task_status", lambda msg: print(f"  {msg.data}", flush=True), 10)


def main():
    rclpy.init()
    node = TextInput()
    # input() 会一直等键盘，所以 spin（收 /task_status）放到另一个线程
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    print("输入指令后回车（例如：把红色方块放到左边），输入 q 退出", flush=True)
    try:
        while rclpy.ok():
            text = input("> ").strip()
            if text.lower() in ("q", "quit", "exit"):
                break
            if text:
                node.pub.publish(String(data=text))
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
