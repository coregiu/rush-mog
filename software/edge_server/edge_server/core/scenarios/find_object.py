"""“语音找东西”场景状态机（纯逻辑，不依赖 ROS/浏览器）。

场景层只做三件事：

1. 听懂“找 X / 别找了”；
2. 通过 ``start_search`` / ``cancel_search`` 效果调用基础搜索能力；
3. 把搜索结果翻译成要播报的话。

真正的探索、导航、检测、确认、接近都在基础能力层
（``object_search_node`` / ``SearchCoordinator``）完成，本模块不重复实现。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from .voice_intent import VoiceCommand, VoiceIntentResolver


class ScenarioState(str, Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"

    @property
    def terminal(self) -> bool:
        return self in (
            ScenarioState.SUCCEEDED,
            ScenarioState.FAILED,
            ScenarioState.CANCELED,
        )


@dataclass(frozen=True)
class ScenarioEffect:
    """场景输出的副作用；由 ROS 适配节点翻译成 Topic/Action 调用。"""

    kind: str  # speak / start_search / cancel_search
    text: str = ""
    target_label: str = ""
    display_name: str = ""
    reason: str = ""


@dataclass
class FindObjectConfig:
    max_duration_sec: float = 300.0
    min_confidence: float = 0.5
    standoff_distance: float = 0.6
    explore_if_not_found: bool = True
    save_map: bool = False
    announce_start: bool = True
    announce_milestones: bool = True
    announce_progress_sec: float = 30.0


class FindObjectScenario:
    """语音找东西的状态机。

    典型调用顺序::

        effects = scenario.handle_utterance("帮我找一下拖鞋", now)
        # -> cancel_search? + speak("好的，我去找拖鞋。") + start_search(slipper)
        ...
        effects = scenario.handle_search_feedback("exploring", detail, ...)
        effects = scenario.handle_search_result(found=True, distance_m=0.7, ...)
        # -> speak("找到拖鞋了，就在前面大约0.7米。")
    """

    def __init__(
        self,
        cfg: Optional[FindObjectConfig] = None,
        resolver: Optional[VoiceIntentResolver] = None,
    ):
        self.cfg = cfg or FindObjectConfig()
        self.resolver = resolver or VoiceIntentResolver()
        self.state = ScenarioState.IDLE
        self.target_label = ""
        self.display_name = ""
        self.detail = "idle"
        self.search_state = ""
        self.last_utterance = ""
        self.confidence = 0.0
        self.explored_ratio = 0.0
        self.started_at = 0.0
        self.finished_at = 0.0
        self.updated_at = 0.0
        self.last_progress_at = 0.0
        self._milestones: set[str] = set()

    # ------------------------------------------------------------------ state
    @property
    def active(self) -> bool:
        return self.state == ScenarioState.SEARCHING

    def status(self, now: float = 0.0) -> dict:
        now = float(now)
        end = self.finished_at if self.finished_at > 0.0 else now
        elapsed = max(0.0, end - self.started_at) if self.started_at > 0.0 else 0.0
        return {
            "state": self.state.value,
            "active": self.active,
            "target_label": self.target_label,
            "display_name": self.display_name,
            "detail": self.detail,
            "search_state": self.search_state,
            "elapsed_sec": elapsed,
            "confidence": float(self.confidence),
            "explored_ratio": float(self.explored_ratio),
            "last_utterance": self.last_utterance,
        }

    # ------------------------------------------------------------------ input
    def handle_utterance(self, text: str, now: float = 0.0) -> List[ScenarioEffect]:
        now = float(now)
        command: VoiceCommand = self.resolver.resolve(text)
        self.last_utterance = str(text or "").strip()
        self.updated_at = now

        if command.is_cancel:
            return self.cancel("canceled by voice", now)

        if not command.is_find:
            return []

        if not command.target_label:
            self.detail = "missing target"
            return [
                ScenarioEffect(
                    "speak",
                    text="你要找什么？请说，找加物品名称。",
                    reason="missing target",
                )
            ]

        previous_display = self.display_name
        was_active = self.active
        effects: List[ScenarioEffect] = []
        if was_active:
            effects.append(
                ScenarioEffect(
                    "cancel_search",
                    target_label=self.target_label,
                    display_name=previous_display,
                    reason="switch target",
                )
            )

        self.state = ScenarioState.SEARCHING
        self.target_label = command.target_label
        self.display_name = command.display_name or command.target_label
        self.detail = "starting search"
        self.search_state = "starting"
        self.confidence = 0.0
        self.explored_ratio = 0.0
        self.started_at = now
        self.finished_at = 0.0
        self.updated_at = now
        self.last_progress_at = now
        self._milestones.clear()

        if self.cfg.announce_start:
            if was_active and previous_display:
                message = f"好的，先不找{previous_display}了，去找{self.display_name}。"
            else:
                message = f"好的，我去找{self.display_name}。"
            effects.append(
                ScenarioEffect("speak", text=message, target_label=self.target_label)
            )

        effects.append(
            ScenarioEffect(
                "start_search",
                target_label=self.target_label,
                display_name=self.display_name,
                reason=command.reason,
            )
        )
        return effects

    # ---------------------------------------------------------------- events
    def handle_search_started(self, now: float = 0.0) -> List[ScenarioEffect]:
        if not self.active:
            return []
        self.detail = "search started"
        self.search_state = "starting"
        self.updated_at = float(now)
        return []

    def handle_search_start_failed(
        self, reason: str, now: float = 0.0
    ) -> List[ScenarioEffect]:
        if not self.active:
            return []
        self.state = ScenarioState.FAILED
        self.detail = f"search unavailable: {reason}"
        self.search_state = "failed"
        self.finished_at = float(now)
        self.updated_at = float(now)
        return [
            ScenarioEffect(
                "speak",
                text="寻找功能还没准备好，请检查导航和目标检测。",
                target_label=self.target_label,
                reason=str(reason),
            )
        ]

    def handle_search_feedback(
        self,
        state: str,
        detail: str = "",
        explored_ratio: float = 0.0,
        confidence: float = 0.0,
        now: float = 0.0,
    ) -> List[ScenarioEffect]:
        if not self.active:
            return []
        now = float(now)
        self.search_state = str(state or "")
        if detail:
            self.detail = str(detail)
        self.explored_ratio = max(0.0, float(explored_ratio))
        self.confidence = max(0.0, float(confidence))
        self.updated_at = now

        effects: List[ScenarioEffect] = []
        if self.cfg.announce_milestones:
            if state == "confirming" and "confirming" not in self._milestones:
                self._milestones.add("confirming")
                effects.append(
                    ScenarioEffect(
                        "speak",
                        text=f"好像看到{self.display_name}了，我确认一下。",
                        target_label=self.target_label,
                    )
                )
            elif state == "approaching" and "approaching" not in self._milestones:
                self._milestones.add("approaching")
                effects.append(
                    ScenarioEffect(
                        "speak",
                        text=f"确认是{self.display_name}，正在靠近。",
                        target_label=self.target_label,
                    )
                )

        if (
            not effects
            and self.cfg.announce_progress_sec > 0.0
            and state in ("exploring", "confirming", "approaching")
            and now - self.last_progress_at >= self.cfg.announce_progress_sec
        ):
            self.last_progress_at = now
            effects.append(
                ScenarioEffect(
                    "speak",
                    text=f"还在找{self.display_name}。",
                    target_label=self.target_label,
                )
            )
        return effects

    def handle_search_result(
        self,
        found: bool,
        info: str = "",
        confidence: float = 0.0,
        distance_m: Optional[float] = None,
        now: float = 0.0,
    ) -> List[ScenarioEffect]:
        if not self.active:
            # 已被“别找了”/急停取消，迟到的 action result 不再播报。
            return []
        now = float(now)
        self.confidence = float(confidence or self.confidence)
        self.detail = str(info or ("found" if found else "failed"))
        self.finished_at = now
        self.updated_at = now
        if found:
            self.state = ScenarioState.SUCCEEDED
            self.search_state = "found"
            text = format_found_text(self.display_name, distance_m)
        else:
            self.state = ScenarioState.FAILED
            self.search_state = "failed"
            text = format_failure_text(self.display_name, info)
        return [
            ScenarioEffect(
                "speak",
                text=text,
                target_label=self.target_label,
                reason=str(info or ""),
            )
        ]

    def cancel(
        self,
        reason: str = "canceled",
        now: float = 0.0,
        speech: str = "",
    ) -> List[ScenarioEffect]:
        if not self.active:
            return []
        self.state = ScenarioState.CANCELED
        self.detail = f"canceled: {reason}"
        self.search_state = "canceled"
        self.finished_at = float(now)
        self.updated_at = float(now)
        return [
            ScenarioEffect(
                "cancel_search",
                target_label=self.target_label,
                display_name=self.display_name,
                reason=str(reason),
            ),
            ScenarioEffect(
                "speak",
                text=speech or "好的，停止寻找。",
                target_label=self.target_label,
                reason=str(reason),
            ),
        ]


# ------------------------------------------------------------------ phrases
def format_found_text(display_name: str, distance_m: Optional[float]) -> str:
    name = str(display_name or "东西")
    if distance_m is None or float(distance_m) <= 0.0:
        return f"找到{name}了。"
    distance = float(distance_m)
    if distance < 1.0:
        centimeters = max(10, int(round(distance * 100.0 / 10.0)) * 10)
        return f"找到{name}了，就在前面大约{centimeters}厘米。"
    return f"找到{name}了，就在前面大约{distance:.1f}米。"


def format_failure_text(display_name: str, info: str = "") -> str:
    name = str(display_name or "东西")
    reason = str(info or "").lower()
    if "timeout" in reason or "超时" in reason:
        return f"没有找到{name}，找了好久也没看到。"
    return f"没有找到{name}。"
