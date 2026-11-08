"""command_parser 的测试：重点是"大模型说错话时，坏指令一条也发不出去"。

运行：cd ~/arm_ws/src/e6_llm && python3 -m pytest test -v
"""

import json

import pytest

from e6_llm.command_parser import (
    ASIDE,
    MAX_STEPS,
    CommandError,
    build_system_prompt,
    describe_state,
    parse_llm_reply,
    parse_world_state,
    rule_based_parse,
)

OBJECTS = {"red_cube": "红色方块", "blue_cube": "蓝色方块"}
TARGETS = {"left_zone": "左边区域", "right_zone": "右边区域"}


def reply(steps, text="好的"):
    return json.dumps({"steps": steps, "reply": text}, ensure_ascii=False)


# ---------- 正常情况 ----------

def test_single_pick_place():
    commands, text = parse_llm_reply(
        reply([{"action": "pick_place", "object": "red_cube", "target": "left_zone"}], "把红色放左边"),
        OBJECTS, TARGETS)
    assert commands == ["pick_place red_cube left_zone"]
    assert text == "把红色放左边"


def test_multiple_steps_keep_order():
    commands, _ = parse_llm_reply(reply([
        {"action": "pick_place", "object": "blue_cube", "target": "right_zone"},
        {"action": "pick_place", "object": "red_cube", "target": "left_zone"},
        {"action": "home"},
    ]), OBJECTS, TARGETS)
    assert commands == [
        "pick_place blue_cube right_zone",
        "pick_place red_cube left_zone",
        "home",
    ]


def test_empty_steps_means_refused():
    commands, text = parse_llm_reply(reply([], "桌上没有绿色方块"), OBJECTS, TARGETS)
    assert commands == []
    assert text == "桌上没有绿色方块"


# ---------- 大模型说错话：一律拒绝 ----------

@pytest.mark.parametrize("bad_text", [
    "好的，我来把红色方块放到左边",          # 不是 JSON
    '```json\n{"steps": []}\n```',           # 包了 markdown 代码块
    '["home"]',                               # 最外层不是对象
    '{"reply": "好的"}',                      # 缺 steps
    '{"steps": "home", "reply": "好的"}',     # steps 不是列表
])
def test_rejects_malformed(bad_text):
    with pytest.raises(CommandError):
        parse_llm_reply(bad_text, OBJECTS, TARGETS)


@pytest.mark.parametrize("bad_step", [
    {"action": "throw", "object": "red_cube"},                                   # 清单外的动作
    {"action": "pick_place", "object": "green_cube", "target": "left_zone"},      # 编造的物体
    {"action": "pick_place", "object": "red_cube", "target": "floor"},            # 编造的区域
    {"action": "pick_place", "object": "red_cube"},                               # 少了 target
    "home",                                                                       # 不是对象
])
def test_rejects_bad_step(bad_step):
    with pytest.raises(CommandError):
        parse_llm_reply(reply([bad_step]), OBJECTS, TARGETS)


def test_one_bad_step_rejects_whole_plan():
    # 第 2 步错了，第 1 步也不能发出去：要么全做，要么全不做
    with pytest.raises(CommandError):
        parse_llm_reply(reply([
            {"action": "pick_place", "object": "red_cube", "target": "left_zone"},
            {"action": "pick_place", "object": "red_cube", "target": "moon"},
        ]), OBJECTS, TARGETS)


def test_rejects_too_many_steps():
    with pytest.raises(CommandError):
        parse_llm_reply(reply([{"action": "home"}] * (MAX_STEPS + 1)), OBJECTS, TARGETS)


def test_rejects_injected_text_in_names():
    # 名字里夹带空格和别的指令，拼成文本后会变成多余的单词 —— 必须在这一层就挡住
    with pytest.raises(CommandError):
        parse_llm_reply(reply([
            {"action": "pick_place", "object": "red_cube left_zone home", "target": "left_zone"},
        ]), OBJECTS, TARGETS)


# ---------- 提示词 ----------

def test_prompt_lists_every_id():
    prompt = build_system_prompt(OBJECTS, TARGETS)
    for name in list(OBJECTS) + list(TARGETS):
        assert name in prompt


# ---------- 规则版 ----------

def test_rule_simple_sentence():
    commands, _ = rule_based_parse("把红色方块放到左边", OBJECTS, TARGETS)
    assert commands == ["pick_place red_cube left_zone"]


def test_rule_home():
    commands, _ = rule_based_parse("回到初始位置", OBJECTS, TARGETS)
    assert commands == ["home"]


def test_rule_gives_up_on_two_objects():
    commands, _ = rule_based_parse("红的放左边，蓝的放右边", OBJECTS, TARGETS)
    assert commands == []


# ---------- 状态 + aside（交换位置要用到） ----------

def test_aside_is_allowed_target():
    commands, _ = parse_llm_reply(reply([
        {"action": "pick_place", "object": "red_cube", "target": ASIDE},
        {"action": "pick_place", "object": "blue_cube", "target": "right_zone"},
        {"action": "pick_place", "object": "red_cube", "target": "left_zone"},
    ]), OBJECTS, TARGETS)
    assert commands == [
        "pick_place red_cube aside",
        "pick_place blue_cube right_zone",
        "pick_place red_cube left_zone",
    ]


def test_parse_world_state():
    state = parse_world_state("red_cube right_zone\nblue_cube table\n坏的一行\n")
    assert state == {"red_cube": "right_zone", "blue_cube": "table"}


def test_describe_state_lists_positions_and_free_zones():
    text = describe_state(OBJECTS, TARGETS, {"red_cube": "right_zone", "blue_cube": "table"})
    assert "red_cube 在 right_zone" in text
    assert "blue_cube 在桌上，不在任何区域" in text
    assert "空着的区域：left_zone" in text


def test_describe_state_no_free_zone():
    text = describe_state(OBJECTS, TARGETS, {"red_cube": "right_zone", "blue_cube": "left_zone"})
    assert "空着的区域：没有" in text


def test_describe_state_unknown():
    assert "不知道" in describe_state(OBJECTS, TARGETS, None)


def test_prompt_contains_current_state_and_aside():
    prompt = build_system_prompt(OBJECTS, TARGETS, {"red_cube": "right_zone", "blue_cube": "left_zone"})
    assert "red_cube 在 right_zone" in prompt
    assert ASIDE in prompt
