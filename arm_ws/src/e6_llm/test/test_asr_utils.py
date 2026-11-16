"""asr_utils 的测试：字节 → 浮点、时长、文字清理。"""

import numpy as np

from e6_llm.asr_utils import clean_text, duration_seconds, pcm16_to_float


def test_pcm16_to_float_range():
    # 小端 16 位：0、最大 32767、最小 -32768
    data = np.array([0, 32767, -32768], dtype="<i2").tobytes()
    samples = pcm16_to_float(data)
    assert samples.dtype == np.float32
    assert samples[0] == 0.0
    assert abs(samples[1] - 1.0) < 1e-4
    assert samples[2] == -1.0


def test_pcm16_drops_half_sample():
    data = np.array([100, 200], dtype="<i2").tobytes() + b"\x01"  # 多出 1 个字节
    assert len(pcm16_to_float(data)) == 2


def test_duration():
    assert duration_seconds(b"\x00" * 96000) == 3.0  # 16000 × 2 字节 × 3 秒


def test_clean_text():
    assert clean_text(" 把红色方块放到左边。") == "把红色方块放到左边"
    assert clean_text("回到初始位置！") == "回到初始位置"
    assert clean_text("") == ""
