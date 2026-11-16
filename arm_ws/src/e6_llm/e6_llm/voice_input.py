"""voice_input：按回车说话 → 本地语音识别 → 发到 /voice_text。

和 text_input 的唯一区别是"键盘"换成了"麦克风"，发的是同一个话题，所以 planner_node 一行都不用改。
同时把 /task_status 上的进度打印出来。

链路：Windows 麦克风 → WSLg 的 PulseAudio → 容器里的 parecord（录成 16 kHz 单声道原始数据）
      → SenseVoice 模型（sherpa-onnx，CPU）→ 文字 → /voice_text

用法：ros2 run e6_llm voice_input
  回车 = 开始录音，说完再按回车 = 停止并识别；也可以直接打字再回车（当作文字指令）；q 退出
"""

import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from e6_llm.asr_utils import SAMPLE_RATE, clean_text, duration_seconds, pcm16_to_float

MIN_SECONDS = 0.5  # 太短的录音（误按两下回车）不识别


def load_recognizer(model_dir, num_threads):
    import sherpa_onnx  # 放在这里导入：没装 sherpa-onnx 时，给出清楚的报错而不是一启动就崩

    model_dir = Path(model_dir)
    return sherpa_onnx.OfflineRecognizer.from_sense_voice(
        model=str(model_dir / "model.int8.onnx"),
        tokens=str(model_dir / "tokens.txt"),
        num_threads=num_threads,
        language="zh",
        use_itn=True,  # 逆文本规范化：把"三"写成"3"、加标点
    )


class Recorder:
    """用 parecord 录音。必须有一个线程一直读：管道只能存 64 KB（约 2 秒），不读走 parecord 就会卡住。"""

    def __init__(self):
        self.proc = None
        self.chunks = []
        self.reader = None

    def start(self):
        self.chunks = []
        self.proc = subprocess.Popen(
            ["parecord", "--raw", "--format=s16le", f"--rate={SAMPLE_RATE}", "--channels=1"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        while True:
            chunk = self.proc.stdout.read(4096)
            if not chunk:
                break
            self.chunks.append(chunk)

    def stop(self):
        self.proc.terminate()
        self.proc.wait(timeout=2)
        self.reader.join(timeout=2)
        error = self.proc.stderr.read().decode(errors="replace").strip()
        return b"".join(self.chunks), error


class VoiceInput(Node):
    def __init__(self):
        super().__init__("voice_input")
        self.declare_parameter("model_dir", "/opt/asr/sense-voice")
        self.declare_parameter("num_threads", 4)
        self.pub = self.create_publisher(String, "/voice_text", 10)
        self.create_subscription(String, "/task_status", lambda msg: print(f"  {msg.data}", flush=True), 10)

        start = time.monotonic()
        self.recognizer = load_recognizer(
            self.get_parameter("model_dir").value, self.get_parameter("num_threads").value)
        print(f"语音识别模型已加载（{time.monotonic() - start:.1f} s）", flush=True)

    def recognize(self, data):
        stream = self.recognizer.create_stream()
        stream.accept_waveform(SAMPLE_RATE, pcm16_to_float(data))
        self.recognizer.decode_stream(stream)
        return clean_text(stream.result.text)

    def publish(self, text):
        self.pub.publish(String(data=text))


def main():
    rclpy.init()
    node = VoiceInput()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    recorder = Recorder()
    print("回车 = 开始说话，说完再按回车；也可以直接打字再回车；q 退出", flush=True)
    try:
        while rclpy.ok():
            typed = input("> ").strip()
            if typed.lower() in ("q", "quit", "exit"):
                break
            if typed:  # 直接打字：和 text_input 一样
                node.publish(typed)
                continue

            recorder.start()
            input("  🎤 正在录音……说完按回车")
            data, error = recorder.stop()
            seconds = duration_seconds(data)
            if seconds < MIN_SECONDS:
                print(f"  录音太短（{seconds:.1f} s）{('：' + error) if error else ''}", flush=True)
                continue

            start = time.monotonic()
            text = node.recognize(data)
            print(f"  识别：「{text}」（录音 {seconds:.1f} s，识别用时 {time.monotonic() - start:.2f} s）",
                  flush=True)
            if text:
                node.publish(text)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
