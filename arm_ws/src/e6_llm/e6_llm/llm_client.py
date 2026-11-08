"""调用大模型：OpenAI 兼容的 /chat/completions 接口（DeepSeek、通义千问、OpenAI 等都支持这个格式）。

只用 Python 标准库（urllib），不用额外装包。

配置靠环境变量（密钥不能写进代码，仓库是公开的）：
    LLM_API_KEY    必填，你的 API 密钥
    LLM_BASE_URL   默认 https://api.deepseek.com
                   通义千问：https://dashscope.aliyuncs.com/compatible-mode/v1
    LLM_MODEL      默认 deepseek-chat；通义千问可用 qwen-plus
"""

import json
import os
import urllib.error
import urllib.request

DEFAULT_BASE_URL = "https://api.deepseek.com"
DEFAULT_MODEL = "deepseek-chat"


class LlmError(RuntimeError):
    """联网失败、密钥错误、超时……"""


def has_api_key():
    return bool(os.environ.get("LLM_API_KEY"))


def chat_json(system_prompt, user_text, timeout=20.0):
    """发一轮对话，要求模型只回 JSON。返回模型回答的原始文本（由 command_parser 再校验）。"""
    api_key = os.environ.get("LLM_API_KEY")
    if not api_key:
        raise LlmError("没有设置环境变量 LLM_API_KEY")
    base_url = os.environ.get("LLM_BASE_URL", DEFAULT_BASE_URL).rstrip("/")
    model = os.environ.get("LLM_MODEL", DEFAULT_MODEL)

    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text},
        ],
        "temperature": 0,  # 同一句话尽量每次给出同样的结果
        "response_format": {"type": "json_object"},  # JSON 模式：要求只输出 JSON
    }
    request = urllib.request.Request(
        f"{base_url}/chat/completions",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {api_key}"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            data = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        # 401 = 密钥不对；402 = 余额不足；429 = 请求太频繁
        detail = e.read().decode("utf-8", errors="replace")[:200]
        raise LlmError(f"HTTP {e.code}：{detail}") from e
    except (urllib.error.URLError, TimeoutError) as e:
        raise LlmError(f"连不上 {base_url}：{e}") from e

    try:
        return data["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as e:
        raise LlmError(f"回答格式不对：{str(data)[:200]}") from e
