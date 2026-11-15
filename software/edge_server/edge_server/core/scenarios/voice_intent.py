"""语音文本 -> 场景意图（纯逻辑，不依赖 ROS/浏览器）。

只回答三件事：

- ``find``：用户要机器人找某个东西；
- ``cancel``：用户要停止当前寻找；
- ``ignore``：不是找东西场景的话（交给原有按键/导航逻辑）。

文本解析复用 :class:`InstructionParser`，因此“找杯子”“去厨房”“建图”等
mission 关键词与 `instruction.object_labels` 的别名配置保持一致。场景层只
关心 ``mission_type == "search"`` 的结果。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from ..instruction_parser import InstructionParser

# 停止寻找：不要用裸“停”匹配“停车场”这类词；下面按整句/明确短语判断。
_CANCEL_PHRASES = (
    "停止寻找",
    "停止搜索",
    "别找了",
    "不要找了",
    "不用找了",
    "不找了",
    "取消寻找",
    "取消搜索",
    "stop searching",
    "stop looking",
    "cancel search",
)
_CANCEL_EXACT = {
    "停",
    "停下",
    "停止",
    "别找",
    "取消",
    "stop",
    "cancel",
}


@dataclass(frozen=True)
class VoiceCommand:
    """解析后的语音意图。"""

    kind: str  # find / cancel / ignore
    raw_text: str = ""
    target_label: str = ""
    display_name: str = ""
    reason: str = ""

    @property
    def is_find(self) -> bool:
        return self.kind == "find"

    @property
    def is_cancel(self) -> bool:
        return self.kind == "cancel"


class VoiceIntentResolver:
    """把一段语音文本归一化成场景意图。"""

    def __init__(
        self,
        parser: Optional[InstructionParser] = None,
        *,
        cancel_phrases: Optional[Sequence[str]] = None,
    ):
        self.parser = parser or InstructionParser()
        self.cancel_phrases = tuple(
            str(item).lower().strip()
            for item in (cancel_phrases or _CANCEL_PHRASES)
            if str(item).strip()
        )

    def resolve(self, text: str) -> VoiceCommand:
        raw = str(text or "").strip()
        normalized = self._normalize(raw)
        if not normalized:
            return VoiceCommand("ignore", raw_text=raw, reason="empty")

        if self._is_cancel(normalized):
            return VoiceCommand("cancel", raw_text=raw, reason="cancel keyword")

        spec = self.parser.parse(raw)
        if spec.mission_type == "search":
            label = str(spec.target_label or "").strip()
            if label:
                return VoiceCommand(
                    "find",
                    raw_text=raw,
                    target_label=label,
                    display_name=self.parser.display_name(label) or label,
                    reason=spec.reason or "search intent",
                )
            # “帮我找一下”没有宾语：不要静默忽略，让场景追问。
            return VoiceCommand(
                "find",
                raw_text=raw,
                target_label="",
                display_name="",
                reason="search intent without target",
            )

        return VoiceCommand("ignore", raw_text=raw, reason=spec.reason or "not a find intent")

    def _is_cancel(self, normalized: str) -> bool:
        if normalized in _CANCEL_EXACT:
            return True
        return any(phrase in normalized for phrase in self.cancel_phrases)

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(str(text or "").strip().lower().split())
