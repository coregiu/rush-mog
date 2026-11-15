"""语音找东西场景（L3）纯逻辑单元测试，不依赖 ROS/浏览器。"""

from edge_server.core.instruction_parser import InstructionParser
from edge_server.core.scenarios import (
    FindObjectConfig,
    FindObjectScenario,
    VoiceIntentResolver,
    format_failure_text,
    format_found_text,
)

PARSER_CFG = {
    "object_labels": {
        "cup": ["杯子", "水杯", "cup", "mug"],
        "slipper": ["拖鞋", "鞋子", "slipper", "slippers", "shoe", "shoes"],
        "remote": ["遥控器", "遥控", "remote"],
        "person": ["人", "人员", "person"],
    }
}


def make_scenario(**overrides):
    parser = InstructionParser(PARSER_CFG)
    resolver = VoiceIntentResolver(parser)
    cfg = FindObjectConfig(**overrides)
    return FindObjectScenario(cfg, resolver)


# ------------------------------------------------------------------ intent
def test_voice_intent_find_cancel_ignore():
    resolver = VoiceIntentResolver(InstructionParser(PARSER_CFG))

    cmd = resolver.resolve("帮我找一下拖鞋")
    assert cmd.kind == "find"
    assert cmd.target_label == "slipper"
    assert cmd.display_name == "拖鞋"

    cmd = resolver.resolve("拖鞋在哪")
    assert cmd.kind == "find"
    assert cmd.target_label == "slipper"

    cmd = resolver.resolve("find my slippers")
    assert cmd.kind == "find"
    assert cmd.target_label == "slipper"

    cmd = resolver.resolve("找遥控器")
    assert cmd.kind == "find"
    assert cmd.target_label == "remote"

    assert resolver.resolve("别找了").kind == "cancel"
    assert resolver.resolve("停止寻找").kind == "cancel"
    assert resolver.resolve("停止").kind == "cancel"

    assert resolver.resolve("前进").kind == "ignore"
    assert resolver.resolve("").kind == "ignore"


def test_voice_intent_missing_target_and_free_label():
    resolver = VoiceIntentResolver(InstructionParser(PARSER_CFG))

    cmd = resolver.resolve("帮我找一下")
    assert cmd.kind == "find"
    assert cmd.target_label == ""

    # 未配置别名时保留自由文本，交给支持该 label 的检测器。
    cmd = resolver.resolve("找充电器")
    assert cmd.kind == "find"
    assert cmd.target_label == "充电器"


# ---------------------------------------------------------------- scenario
def test_find_object_success_flow():
    scenario = make_scenario(announce_progress_sec=10.0)
    effects = scenario.handle_utterance("帮我找拖鞋", now=100.0)
    assert [e.kind for e in effects] == ["speak", "start_search"]
    assert effects[0].text == "好的，我去找拖鞋。"
    assert effects[1].target_label == "slipper"
    assert scenario.active
    assert scenario.status(101.0)["state"] == "searching"

    effects = scenario.handle_search_feedback(
        "confirming", "candidate seen", 0.2, 0.7, now=102.0
    )
    assert [e.text for e in effects] == ["好像看到拖鞋了，我确认一下。"]

    effects = scenario.handle_search_feedback(
        "approaching", "approach target", 0.2, 0.8, now=103.0
    )
    assert [e.text for e in effects] == ["确认是拖鞋，正在靠近。"]

    effects = scenario.handle_search_result(
        True, "target reached", confidence=0.9, distance_m=0.62, now=104.0
    )
    assert [e.text for e in effects] == ["找到拖鞋了，就在前面大约60厘米。"]
    status = scenario.status(105.0)
    assert status["state"] == "succeeded"
    assert status["active"] is False
    assert status["elapsed_sec"] == 4.0

    # 迟到的播报/结果不能改变终态。
    assert scenario.handle_search_result(False, "late", now=106.0) == []


def test_find_object_failure_and_cancel():
    scenario = make_scenario()
    scenario.handle_utterance("找杯子", now=1.0)
    effects = scenario.handle_search_result(
        False, "search timeout after 300s", now=2.0
    )
    assert effects[0].text == "没有找到杯子，找了好久也没看到。"
    assert scenario.status(3.0)["state"] == "failed"

    scenario = make_scenario()
    scenario.handle_utterance("找拖鞋", now=1.0)
    effects = scenario.cancel("canceled by voice", now=2.0)
    assert [e.kind for e in effects] == ["cancel_search", "speak"]
    assert effects[1].text == "好的，停止寻找。"
    assert scenario.status(3.0)["state"] == "canceled"
    assert scenario.handle_search_result(True, "late", now=4.0) == []

    # 没有正在找时，取消不播报。
    assert scenario.cancel("again", now=5.0) == []


def test_find_object_switch_target():
    scenario = make_scenario()
    scenario.handle_utterance("找杯子", now=1.0)
    effects = scenario.handle_utterance("找拖鞋", now=2.0)
    assert [e.kind for e in effects] == ["cancel_search", "speak", "start_search"]
    assert "先不找杯子了" in effects[1].text
    assert effects[2].target_label == "slipper"


def test_find_object_missing_target_and_start_failure():
    scenario = make_scenario()
    effects = scenario.handle_utterance("帮我找一下", now=1.0)
    assert [e.kind for e in effects] == ["speak"]
    assert "你要找什么" in effects[0].text
    assert not scenario.active

    scenario.handle_utterance("找拖鞋", now=2.0)
    effects = scenario.handle_search_start_failed("action unavailable", now=3.0)
    assert "寻找功能还没准备好" in effects[0].text
    assert scenario.status(4.0)["state"] == "failed"


def test_find_object_progress_announcement():
    scenario = make_scenario(announce_progress_sec=5.0)
    scenario.handle_utterance("找拖鞋", now=0.0)
    assert scenario.handle_search_feedback("exploring", "frontier", 0.1, 0.0, now=3.0) == []
    effects = scenario.handle_search_feedback(
        "exploring", "frontier", 0.2, 0.0, now=6.0
    )
    assert [e.text for e in effects] == ["还在找拖鞋。"]


def test_find_object_emergency_cancel_speech():
    scenario = make_scenario()
    scenario.handle_utterance("找拖鞋", now=0.0)
    effects = scenario.cancel("emergency stop", now=1.0, speech="已急停，停止寻找。")
    assert effects[1].text == "已急停，停止寻找。"


def test_format_phrases():
    assert format_found_text("拖鞋", None) == "找到拖鞋了。"
    assert format_found_text("拖鞋", 0.42) == "找到拖鞋了，就在前面大约40厘米。"
    assert format_found_text("拖鞋", 1.24) == "找到拖鞋了，就在前面大约1.2米。"
    assert format_failure_text("拖鞋", "timeout") == "没有找到拖鞋，找了好久也没看到。"
    assert format_failure_text("拖鞋", "no reachable frontier") == "没有找到拖鞋。"
