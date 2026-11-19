"""本地规划：一句话 + 当前场景 → 步骤清单。用 Qwen3-VL（关掉 LoRA）当大模型，不联网、不花钱。

复用 C 项目（arm_ws/src/e6_llm）的提示词和校验：大模型只能用清单里的物体 / 区域 id、只能说两种动作。
在那之上多了两样：
1. 叠放：告诉大模型谁压着谁（来自 VLM 看图），被压住的不能直接抓
2. 先在 Mac 上"模拟走一遍"：小模型（4B）常犯"目标区域还被占着就直接放过去"这类错，
   走一遍就能查出来；查出错就把错误原话告诉大模型，让它重想（最多 3 次）。
   错误计划根本不会发给机械臂——这是"想"这一层的闭环。
"""
import os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "arm_ws", "src", "e6_llm"))
from e6_llm.command_parser import ASIDE, CommandError, build_system_prompt, parse_llm_reply   # noqa: E402

LABELS = {"red": "红色方块", "blue": "蓝色方块", "green": "绿色方块", "yellow": "黄色方块",
          "orange": "橙色方块", "purple": "紫色方块"}
TARGETS = {"left_zone": "左边区域", "right_zone": "右边区域"}
MAX_TRIES = 3


def stack_lines(on):
    """on：{物体id: 它下面的物体id 或 "table"}。"""
    lines = [f"- {oid} 叠在 {below} 上面：要动 {below}，必须先把 {oid} 挪开"
             for oid, below in on.items() if below != "table"]
    return ("\n叠放情况：\n" + "\n".join(lines)) if lines else ""


def simulate(commands, state, on):
    """按顺序在"纸面上"走一遍，返回第一个错误（没错返回 None）。state：{物体id: 区域id 或 "table"}。"""
    state, on = dict(state), dict(on)
    for i, cmd in enumerate(commands, start=1):
        if cmd == "home":
            continue
        _, obj, tgt = cmd.split()
        above = [o for o, b in on.items() if b == obj]
        if above:
            return f"第 {i} 步：{obj} 上面压着 {above[0]}，要先把 {above[0]} 挪开"
        if tgt != ASIDE:
            occupant = [o for o, z in state.items() if z == tgt and o != obj]
            if occupant:
                return f"第 {i} 步：{tgt} 已经被 {occupant[0]} 占着，要先把 {occupant[0]} 挪到 {ASIDE}"
        state[obj] = tgt if tgt != ASIDE else "table"
        on[obj] = "table"
    return None


class Planner:
    def __init__(self, vlm):
        self.vlm = vlm

    def plan(self, text, state, on, log=print):
        """返回 (指令列表, 给用户的回复)。指令格式和 task_executor 一样："pick_place red_cube left_zone"。"""
        objects = {oid: LABELS.get(oid.removesuffix("_cube"), oid) for oid in state}
        system = build_system_prompt(objects, TARGETS, state) + stack_lines(on)
        user, last_error = text, None
        for attempt in range(1, MAX_TRIES + 1):
            raw = self.vlm.chat(system, user)
            try:
                commands, reply = parse_llm_reply(raw[raw.find("{"):raw.rfind("}") + 1], objects, TARGETS)
                error = simulate(commands, state, on)
            except CommandError as e:
                commands, reply, error = [], "", f"格式不对：{e}"
            log(f"  规划第 {attempt} 次：{commands} {'✅' if not error else '❌ ' + error}")
            if not error:
                return commands, reply
            last_error = error
            user = f"{text}\n\n（你上一次的计划不行：{error}。请重新规划。）"
        return [], f"规划失败：{last_error}"
