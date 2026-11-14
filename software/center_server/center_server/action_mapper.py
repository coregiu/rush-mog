"""SmolVLA 动作向量 -> Rush-MOG 控制量。

SmolVLA 输出的是动作向量（chunk），不同微调任务的语义不同：

- ``base_twist``：前 3 维假设为 ``[vx, vy, wz]``，适用于在 Rush-MOG 上微调的
  速度策略；
- ``hybrid``：前 3 维底盘速度 + 其余维机械臂关节；
- ``joint``：全部按关节处理；
- ``raw``：只返回模型原始动作，不映射。

``lerobot/smolvla_base`` 是通用 checkpoint，动作空间取决于训练机器人，
不是本车策略；直接使用时应选 ``raw``/``joint`` 并先做动作空间微调。详见 README。
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping

from .errors import UnprocessableEntity

_DEFAULTS: dict[str, Any] = {
    "mode": "base_twist",
    "base_indices": [0, 1, 2],
    "normalized": True,
    "max_linear_x": 0.5,
    "max_linear_y": 0.4,
    "max_angular_z": 1.0,
    "deadband": 0.0,
    "invert": [False, False, False],
    "arm_start": 3,
    "joint_names": ["j1", "j2", "j3", "j4", "j5"],
    "joint_limits": {},
    "round_digits": 6,
}


def _to_float(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise UnprocessableEntity(
            f"动作向量包含非数字值: {value!r}", code="invalid_action"
        )
    result = float(value)
    if not math.isfinite(result):
        raise UnprocessableEntity(
            f"动作向量包含非有限值: {value!r}", code="invalid_action"
        )
    return result


class ActionMapper:
    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        cfg = dict(_DEFAULTS)
        cfg.update(dict(config or {}))
        self.cfg = cfg
        self.mode = str(cfg["mode"]).lower()
        self.base_indices = [int(item) for item in cfg["base_indices"]]
        self.normalized = bool(cfg["normalized"])
        self.max_values = [
            float(cfg["max_linear_x"]),
            float(cfg["max_linear_y"]),
            float(cfg["max_angular_z"]),
        ]
        self.deadband = float(cfg["deadband"])
        self.invert = [bool(item) for item in cfg["invert"]]
        self.arm_start = int(cfg["arm_start"])
        self.joint_names = [str(item) for item in cfg["joint_names"]]
        self.joint_limits = dict(cfg["joint_limits"] or {})
        self.round_digits = int(cfg["round_digits"])

    # ------------------------------------------------------------------ base
    def _base(self, raw: list[float]) -> tuple[dict, bool]:
        clipped = False
        values = []
        for axis, index in enumerate(self.base_indices):
            value = raw[index] if 0 <= index < len(raw) else 0.0
            limit = self.max_values[axis]
            if self.normalized:
                if abs(value) > 1.0:
                    clipped = True
                value = max(-1.0, min(1.0, value)) * limit
            else:
                if abs(value) > limit:
                    clipped = True
                value = max(-limit, min(limit, value))
            if self.invert[axis]:
                value = -value
            if self.deadband and abs(value) < self.deadband:
                value = 0.0
            values.append(self._round(value))

        cmd = {
            "linear": {"x": values[0], "y": values[1], "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": values[2]},
        }
        return cmd, clipped

    # ------------------------------------------------------------------- arm
    def _arm(self, raw: list[float], start: int) -> tuple[dict, bool]:
        values = raw[start:]
        clipped = False
        result: dict[str, float] = {}
        for offset, value in enumerate(values):
            name = (
                self.joint_names[offset]
                if offset < len(self.joint_names)
                else f"j{offset + 1}"
            )
            limit = self.joint_limits.get(name)
            if limit:
                lower, upper = float(limit[0]), float(limit[1])
                if value < lower or value > upper:
                    clipped = True
                value = max(lower, min(upper, value))
            result[name] = self._round(value)
        return result, clipped

    # ------------------------------------------------------------------ main
    def map(self, raw: Iterable[Any]) -> dict:
        values = [_to_float(item, "action") for item in raw]
        if not values:
            raise UnprocessableEntity("模型返回了空动作", code="invalid_action")

        result: dict[str, Any] = {
            "mode": self.mode,
            "raw": [self._round(value) for value in values],
            "base_cmd": None,
            "arm_cmd": None,
            "clipped": False,
        }

        if self.mode == "raw":
            return result

        if self.mode in {"base_twist", "hybrid"}:
            base_cmd, clipped = self._base(values)
            result["base_cmd"] = base_cmd
            result["clipped"] = clipped

        if self.mode == "hybrid":
            arm_cmd, arm_clipped = self._arm(values, self.arm_start)
            result["arm_cmd"] = arm_cmd
            result["clipped"] = bool(result["clipped"] or arm_clipped)
        elif self.mode == "joint":
            arm_cmd, arm_clipped = self._arm(values, 0)
            result["arm_cmd"] = arm_cmd
            result["clipped"] = arm_clipped

        return result

    def _round(self, value: float) -> float:
        return round(float(value), self.round_digits)
