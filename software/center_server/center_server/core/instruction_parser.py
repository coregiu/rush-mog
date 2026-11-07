"""自然语言/文本指令 -> 任务意图（VLA 单步动作或 SLAM/导航/搜索任务）。

规则桩，不依赖大模型。配置示例：

```yaml
instruction:
  object_labels:
    cup: ["杯子", "水杯", "cup", "mug"]
    bottle: ["瓶子", "水瓶", "bottle"]
  waypoints:
    kitchen: [1.2, 0.8, 0.0]
    charging: [0.0, 0.0, 0.0]
  home_pose: [0.0, 0.0, 0.0]
```
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

DEFAULT_OBJECT_LABELS: Dict[str, List[str]] = {
    "cup": ["杯子", "水杯", "马克杯", "cup", "mug"],
    "bottle": ["瓶子", "水瓶", "bottle"],
    "person": ["人", "人员", "person"],
    "box": ["盒子", "箱子", "box"],
    "ball": ["球", "小球", "ball"],
}

DEFAULT_WAYPOINTS: Dict[str, Tuple[float, float, float]] = {}

_MAP_WORDS = ("建图", "地图", "slam", "mapping", "map")
_EXPLORE_WORDS = ("探索", "巡检", "explore", "patrol")
_SEARCH_WORDS = ("找", "寻找", "搜索", "find", "search", "look for")
_RETURN_WORDS = ("回家", "返回", "回来", "return home", "go home", "home")
_NAV_WORDS = ("导航", "前往", "去", "navigate", "goto", "go to")

_NUM = r"(-?\d+(?:\.\d+)?)"


@dataclass
class MissionSpec:
    """解析结果。mission_type == "vla" 表示交给 VLA 单步动作。"""

    mission_type: str = "vla"
    target_label: str = ""
    waypoint: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    confidence: float = 0.0
    reason: str = ""
    # 显式任务参数（ExecuteTask/ExecuteMission 可直接填）
    max_duration: float = 0.0
    save_map: bool = False
    target_coverage: float = 0.0

    @property
    def is_mission(self) -> bool:
        return self.mission_type != "vla"


class InstructionParser:
    def __init__(self, cfg: Optional[dict] = None):
        cfg = cfg or {}
        self.object_labels = {
            str(k): [str(a).lower() for a in v]
            for k, v in (cfg.get("object_labels") or DEFAULT_OBJECT_LABELS).items()
        }
        waypoints = cfg.get("waypoints") or DEFAULT_WAYPOINTS
        self.waypoints: Dict[str, Tuple[float, float, float]] = {}
        self.waypoint_aliases: Dict[str, str] = {}
        for name, value in waypoints.items():
            aliases = []
            if isinstance(value, dict):
                coords = (
                    float(value.get("x", 0.0)),
                    float(value.get("y", 0.0)),
                    float(value.get("yaw", 0.0)),
                )
                aliases = value.get("aliases") or []
            else:
                vals = list(value)
                while len(vals) < 3:
                    vals.append(0.0)
                coords = (float(vals[0]), float(vals[1]), float(vals[2]))
            key = str(name)
            self.waypoints[key] = coords
            self.waypoint_aliases[key.lower()] = key
            for alias in aliases:
                if alias:
                    self.waypoint_aliases[str(alias).lower()] = key
        home = cfg.get("home_pose") or [0.0, 0.0, 0.0]
        self.home_pose = (
            float(home[0]),
            float(home[1]),
            float(home[2]) if len(home) > 2 else 0.0,
        )

    # ------------------------------------------------------------------ public
    def parse(self, text: str) -> MissionSpec:
        raw = (text or "").strip()
        normalized = raw.lower()
        if not normalized:
            return MissionSpec()

        # 1) 建图（“建图并找杯子”优先搜索，见下）
        has_search = self._contains_any(normalized, _SEARCH_WORDS)
        if self._contains_any(normalized, _MAP_WORDS) and not has_search:
            return MissionSpec("map", confidence=0.9, reason="map keyword")

        # 2) 回家/返回
        if self._contains_any(normalized, _RETURN_WORDS):
            x, y, yaw = self.home_pose
            return MissionSpec(
                "return_home", x=x, y=y, yaw=yaw, confidence=0.9, reason="home keyword"
            )

        # 3) 搜索/寻找物体
        if has_search:
            label = self.extract_label(normalized)
            return MissionSpec(
                "search",
                target_label=label,
                confidence=0.9 if label else 0.7,
                reason=f"search keyword, label={label or 'unknown'}",
            )

        # 4) 探索/巡检
        if self._contains_any(normalized, _EXPLORE_WORDS):
            return MissionSpec("explore", confidence=0.9, reason="explore keyword")

        # 5) 导航到命名航点/坐标
        coords = self._parse_coords(normalized)
        waypoint = self._extract_waypoint(normalized)
        if waypoint is not None:
            x, y, yaw = self.waypoints[waypoint]
            return MissionSpec(
                "navigate",
                waypoint=waypoint,
                x=x,
                y=y,
                yaw=yaw,
                confidence=0.9,
                reason=f"waypoint={waypoint}",
            )
        if coords is not None:
            x, y, yaw = coords
            return MissionSpec(
                "navigate", x=x, y=y, yaw=yaw, confidence=0.9, reason="coordinates"
            )
        if self._contains_any(normalized, _NAV_WORDS):
            return MissionSpec(
                "navigate",
                confidence=0.6,
                reason="navigate keyword but no waypoint/coordinates",
            )

        return MissionSpec()

    def extract_label(self, text: str) -> str:
        for label, aliases in self.object_labels.items():
            if any(alias and alias in text for alias in aliases):
                return label
        return ""

    def _extract_waypoint(self, text: str) -> Optional[str]:
        # 先匹配更长的别名，避免 "living" 被 "live" 抢先
        for alias in sorted(self.waypoint_aliases, key=len, reverse=True):
            if alias and alias in text:
                return self.waypoint_aliases[alias]
        return None

    def _parse_coords(self, text: str) -> Optional[Tuple[float, float, float]]:
        mx = re.search(r"(?:^|[\s,;，；])x\s*[=:]\s*" + _NUM, text)
        my = re.search(r"(?:^|[\s,;，；])y\s*[=:]\s*" + _NUM, text)
        myaw = re.search(r"(?:^|[\s,;，；])yaw\s*[=:]\s*" + _NUM, text)
        if mx is None or my is None:
            return None
        x, y = float(mx.group(1)), float(my.group(1))
        yaw = float(myaw.group(1)) if myaw is not None else 0.0
        # 大于 2π 的 yaw 视为角度制
        if abs(yaw) > 2.0 * math.pi:
            yaw = math.radians(yaw)
        return x, y, yaw

    @staticmethod
    def _contains_any(text: str, words: Iterable[str]) -> bool:
        return any(word in text for word in words)
