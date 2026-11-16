"""语音识别用到的纯函数（不碰麦克风、不加载模型，可以直接用 pytest 测）。"""

import numpy as np

SAMPLE_RATE = 16000  # SenseVoice 要求 16 kHz 单声道：每秒 16000 个采样点
BYTES_PER_SAMPLE = 2  # s16le：每个采样点是 16 位整数 = 2 字节


def pcm16_to_float(data):
    """录音得到的原始字节 → 模型要的浮点数组（范围 -1 到 1）。

    例：3 秒录音 = 16000 × 3 = 48000 个采样点 = 96000 字节。
    16 位整数范围是 -32768 到 32767，除以 32768 就落到 [-1, 1)。
    最后多出半个采样点（奇数字节）就丢掉。
    """
    usable = len(data) - len(data) % BYTES_PER_SAMPLE
    samples = np.frombuffer(data[:usable], dtype="<i2")
    return samples.astype(np.float32) / 32768.0


def duration_seconds(data):
    return len(data) / (SAMPLE_RATE * BYTES_PER_SAMPLE)


def clean_text(text):
    """去掉首尾空白和句末标点："把红色方块放到左边。" → "把红色方块放到左边"。"""
    return text.strip().rstrip("。，、！？.,!? ")
