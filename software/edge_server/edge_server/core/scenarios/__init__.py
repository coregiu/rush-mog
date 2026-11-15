"""场景编排层（L3）：把基础能力组合成用户可用的场景。

当前提供“语音找东西”：
- :mod:`voice_intent`：文本 -> find/cancel/ignore；
- :mod:`find_object`：FindObjectScenario 状态机与播报文案。

这里不 import rclpy，所有模块都可以直接单测。
"""

from .find_object import (
    FindObjectConfig,
    FindObjectScenario,
    ScenarioEffect,
    ScenarioState,
    format_failure_text,
    format_found_text,
)
from .voice_intent import VoiceCommand, VoiceIntentResolver

__all__ = [
    "FindObjectConfig",
    "FindObjectScenario",
    "ScenarioEffect",
    "ScenarioState",
    "VoiceCommand",
    "VoiceIntentResolver",
    "format_failure_text",
    "format_found_text",
]
