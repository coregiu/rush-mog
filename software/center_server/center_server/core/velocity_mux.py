"""多源速度仲裁（velocity mux）。

VLA、Nav2、反向操作、安全模块可能同时向底盘发速度。直接共用 /cmd_vel 会出现
“谁最后发谁赢”的竞态。本模块定义确定性的优先级：

    0 安全覆盖（急停/障碍物/低电量）
    1 反向操作/手动接管
    2 遥控/遥操作
    3 Nav2 或内置直航
    4 VLA 单步动作

同一时刻只有最高优先级的活跃源生效；每个源都有超时，超时自动失效并回零。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class SourceState:
    name: str
    priority: int
    timeout: float
    twist: object = None
    stamp: float = 0.0
    ever_set: bool = False


class VelocityMux:
    def __init__(
        self,
        default_timeout: float = 0.5,
        max_linear: float = 1.0,
        max_angular: float = 1.5,
        priorities: Optional[Dict[str, int]] = None,
    ):
        self.default_timeout = float(default_timeout)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.priorities = dict(priorities or {})
        self._sources: Dict[str, SourceState] = {}
        self.emergency = False
        self.emergency_reason = ""

    # ------------------------------------------------------------------ update
    def set(
        self,
        name: str,
        twist,
        now: float,
        priority: Optional[int] = None,
        timeout: Optional[float] = None,
    ):
        src = self._sources.get(name)
        if src is None:
            src = SourceState(
                name=name,
                priority=int(
                    self.priorities.get(name, priority if priority is not None else 9)
                ),
                timeout=float(timeout if timeout is not None else self.default_timeout),
            )
            self._sources[name] = src
        src.twist = twist
        src.stamp = float(now)
        src.ever_set = True

    def clear(self, name: str):
        self._sources.pop(name, None)

    def set_emergency(self, active: bool, reason: str = ""):
        self.emergency = bool(active)
        self.emergency_reason = reason if active else ""

    # ----------------------------------------------------------------- compute
    def compute(self, now: float) -> Tuple[object, str, List[str], bool]:
        """返回 (Twist, active_source, active_sources, emergency)。"""
        from geometry_msgs.msg import Twist

        if self.emergency:
            return Twist(), "emergency", ["emergency"], True

        active = [
            src
            for src in self._sources.values()
            if src.ever_set and src.twist is not None
            and (now - src.stamp) <= src.timeout
        ]
        active.sort(key=lambda s: (s.priority, s.name))
        names = [s.name for s in active]
        if not active:
            return Twist(), "none", names, False

        chosen = active[0]
        out = _copy_twist(chosen.twist)
        # 防御性限幅：即使上游绕过 SafetyChecker 也不允许超限
        out.linear.x = _clamp(out.linear.x, -self.max_linear, self.max_linear)
        out.linear.y = _clamp(out.linear.y, -self.max_linear, self.max_linear)
        out.angular.z = _clamp(out.angular.z, -self.max_angular, self.max_angular)
        return out, chosen.name, names, False

    def active_sources(self, now: float) -> List[str]:
        _, _, names, _ = self.compute(now)
        return names


def _clamp(value: float, low: float, high: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(low, min(high, value))


def _copy_twist(twist):
    from geometry_msgs.msg import Twist

    out = Twist()
    out.linear.x = float(twist.linear.x)
    out.linear.y = float(twist.linear.y)
    out.linear.z = float(twist.linear.z)
    out.angular.x = float(twist.angular.x)
    out.angular.y = float(twist.angular.y)
    out.angular.z = float(twist.angular.z)
    return out
