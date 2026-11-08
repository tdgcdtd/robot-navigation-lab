"""一句话 → 指令：给大模型的提示词、校验大模型的回答、不用大模型的简单规则版。

这里全是纯函数（不碰 ROS、不联网），所以可以直接用 pytest 测试（见 test/test_command_parser.py）。

大模型被要求只回答这样的 JSON：
    {"steps": [{"action": "pick_place", "object": "red_cube", "target": "left_zone"},
               {"action": "home"}],
     "reply": "好的，把红色方块放到左边区域，然后回到初始位置"}
校验通过后，每一步变成一行文本指令发给 task_executor：
    "pick_place red_cube left_zone"、"home"
"""

import json

MAX_STEPS = 5  # 一句话最多拆成几步：防止大模型"发挥"出一长串动作

ACTIONS = ("pick_place", "home")

# 特殊目标"旁边的空地"：大模型只说"挪开"，具体放哪由 task_executor 找（几何交给代码，不让大模型报坐标）
ASIDE = "aside"
TABLE = "table"  # /world_state 里表示"在桌上，但不在任何区域"


class CommandError(ValueError):
    """大模型的回答不合格（格式不对、用了清单外的名字……）。"""


def parse_world_state(text):
    """把 task_executor 发来的 /world_state 读成 {"red_cube": "right_zone", "blue_cube": "table"}。

    格式：每个物体一行 "物体id 位置"，位置是区域 id 或 "table"。格式不对的行跳过。
    """
    state = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 2:
            state[parts[0]] = parts[1]
    return state


def describe_state(objects, targets, state):
    """当前状态写成给大模型看的几行中文。state 为 None（还没收到 /world_state）时如实说不知道。"""
    if state is None:
        return "（暂时不知道各物体在哪）"
    lines = []
    for oid in objects:
        where = state.get(oid)
        if where in targets:
            lines.append(f"- {oid} 在 {where}")
        else:
            lines.append(f"- {oid} 在桌上，不在任何区域")
    occupied = {state.get(oid) for oid in objects}
    free = [tid for tid in targets if tid not in occupied]
    lines.append(f"- 空着的区域：{', '.join(free) if free else '没有'}")
    return "\n".join(lines)


def build_system_prompt(objects, targets, state=None):
    """objects / targets：{id: 中文名}，例如 {"red_cube": "红色方块"}；state：parse_world_state 的结果。"""
    object_lines = "\n".join(f"- {oid}：{label}" for oid, label in objects.items())
    target_lines = "\n".join(f"- {tid}：{label}" for tid, label in targets.items())
    return f"""你是一个桌面机械臂的任务规划器。把用户的中文指令翻译成动作步骤。

桌上的物体（只能用这些 id）：
{object_lines}

可以放置的区域（只能用这些 id；"左""右"都是从机械臂的角度看）：
{target_lines}
- {ASIDE}：旁边的空地（由机械臂自己找一块空地放下，用于临时挪开物体）

现在的状态：
{describe_state(objects, targets, state)}

只有两种动作：
- {{"action": "pick_place", "object": 物体id, "target": 区域id 或 "{ASIDE}"}}：把物体放到区域
- {{"action": "home"}}：回到初始位置

只输出一个 JSON 对象，格式：
{{"steps": [动作, ...], "reply": "用一句中文复述你要做什么"}}

规则：
- 最多 {MAX_STEPS} 步，按执行顺序排列。
- 一个区域同一时间只能放一个物体。目标区域被别的物体占着时，要先把那个物体挪走（可以挪到 {ASIDE}）。
- 指令听不懂、提到了不存在的物体或区域、或者要求做以上两种动作以外的事：steps 为空列表 []，reply 说明原因。
- 不要编造 id，不要输出 JSON 以外的任何文字。"""


def parse_llm_reply(text, objects, targets):
    """校验大模型的回答。返回 (指令列表, 回复)；不合格就抛 CommandError。

    objects / targets 只用到它们的 id（字典的键）。
    """
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise CommandError(f"不是合法的 JSON：{e}") from e
    if not isinstance(data, dict):
        raise CommandError("最外层应该是 JSON 对象")

    steps = data.get("steps")
    reply = data.get("reply", "")
    if not isinstance(steps, list):
        raise CommandError("缺少 steps 列表")
    if not isinstance(reply, str):
        raise CommandError("reply 应该是字符串")
    if len(steps) > MAX_STEPS:
        raise CommandError(f"步骤太多（{len(steps)} 步，最多 {MAX_STEPS} 步）")

    commands = []
    for i, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            raise CommandError(f"第 {i} 步不是 JSON 对象")
        action = step.get("action")
        if action not in ACTIONS:
            raise CommandError(f"第 {i} 步：不支持的动作 {action!r}")
        if action == "home":
            commands.append("home")
            continue
        obj, tgt = step.get("object"), step.get("target")
        if obj not in objects:
            raise CommandError(f"第 {i} 步：不存在的物体 {obj!r}")
        if tgt not in targets and tgt != ASIDE:
            raise CommandError(f"第 {i} 步：不存在的区域 {tgt!r}")
        commands.append(f"pick_place {obj} {tgt}")
    return commands, reply


def rule_based_parse(text, objects, targets):
    """不用大模型的"简单规则版"：只为没有 API 密钥时也能把整条链路跑通、做测试。

    规则：中文名的第一个字出现在句子里就算提到了（红色方块 → "红"，左边区域 → "左"）。
    只能理解"一个物体 + 一个区域"或"回家"这种最简单的句子 —— 这正是需要大模型的原因。
    返回值格式和 parse_llm_reply 一样。
    """
    if "回" in text and ("初始" in text or "home" in text.lower() or "原位" in text):
        return ["home"], "回到初始位置"
    found_objects = [oid for oid, label in objects.items() if label[:1] in text]
    found_targets = [tid for tid, label in targets.items() if label[:1] in text]
    if len(found_objects) == 1 and len(found_targets) == 1:
        obj, tgt = found_objects[0], found_targets[0]
        return [f"pick_place {obj} {tgt}"], f"把{objects[obj]}放到{targets[tgt]}"
    return [], "规则版听不懂这句话（只支持“把 X 放到 Y”和“回初始位置”）"
