"""把 ROS2 Twist 量化为 Body Control 的离散方向 + PWM 档位命令。

Body Control 当前只支持 8 个平移方向、左/右原地转向和 1~9 档 PWM，
不支持连续的左右轮差速。因此 ROS2 的 (vx, vy, wz) 需要量化：

- 平移：按速度方向选择 1/2/3/4/5/6/7/8，按速度大小选择 1~9 档；
- 原地转向：wz > 0 用 9（左转），wz < 0 用 A（右转）；
- 同时有平移和转向时：
  - ``twist_mode="priority"``（默认）选择归一化后更大的分量；
  - ``twist_mode="time_share"`` 按 ``arc_period_sec`` 交替输出平移/转向，
    用时间片近似弧线，抖动会比 priority 明显。

输出字符串直接对应 STM32 Body Control 的 ASCII 命令，例如 ``"15"``
表示前进且 PWM 档位 5；Edge Server 的 ``raw:`` 接口会原样转发并保持第二字节。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

@dataclass(frozen=True)
class BodyDriveConfig:
    """ROS Twist -> Body Control 命令的量化参数。"""

    max_linear: float = 0.35
    max_angular: float = 1.0
    deadband_linear: float = 0.02
    deadband_angular: float = 0.05
    max_level: int = 9
    twist_mode: str = "priority"  # priority | time_share
    arc_period_sec: float = 0.5


def _level(value: float, maximum: float, max_level: int) -> int:
    max_level = max(1, int(max_level))
    ratio = min(1.0, abs(float(value)) / max(1e-6, float(maximum)))
    return max(1, min(max_level, int(round(ratio * max_level))))


def _translation_command(
    vx: float,
    vy: float,
    cfg: BodyDriveConfig,
) -> str:
    speed = math.hypot(vx, vy)
    if speed <= cfg.deadband_linear:
        return "0"

    angle = math.degrees(math.atan2(vy, vx))
    if angle >= -22.5 and angle < 22.5:
        direction = "1"
    elif angle >= 22.5 and angle < 67.5:
        direction = "5"
    elif angle >= 67.5 and angle < 112.5:
        direction = "3"
    elif angle >= 112.5 and angle < 157.5:
        direction = "7"
    elif angle >= 157.5 or angle < -157.5:
        direction = "2"
    elif angle >= -157.5 and angle < -112.5:
        direction = "8"
    elif angle >= -112.5 and angle < -67.5:
        direction = "4"
    else:
        direction = "6"

    return f"{direction}{_level(speed, cfg.max_linear, cfg.max_level)}"


def _rotation_command(wz: float, cfg: BodyDriveConfig) -> str:
    if abs(wz) <= cfg.deadband_angular:
        return "0"
    direction = "9" if wz > 0.0 else "A"
    return f"{direction}{_level(abs(wz), cfg.max_angular, cfg.max_level)}"


def twist_to_body_command(
    vx: float,
    vy: float,
    wz: float,
    cfg: BodyDriveConfig | None = None,
    now: float = 0.0,
) -> str:
    """返回 Body Control 命令，如 ``"0"`` / ``"15"`` / ``"95"``。"""
    cfg = cfg or BodyDriveConfig()
    vx = float(vx)
    vy = float(vy)
    wz = float(wz)

    translation_speed = math.hypot(vx, vy)
    has_translation = translation_speed > cfg.deadband_linear
    has_rotation = abs(wz) > cfg.deadband_angular

    if not has_translation and not has_rotation:
        return "0"
    if not has_rotation:
        return _translation_command(vx, vy, cfg)
    if not has_translation:
        return _rotation_command(wz, cfg)

    if cfg.twist_mode == "time_share":
        period = max(0.05, float(cfg.arc_period_sec))
        if (float(now) % period) < (period * 0.5):
            return _translation_command(vx, vy, cfg)
        return _rotation_command(wz, cfg)

    # priority：比较归一化后的平移/转向强度，选择更大的一项。
    linear_ratio = translation_speed / max(1e-6, cfg.max_linear)
    angular_ratio = abs(wz) / max(1e-6, cfg.max_angular)
    if angular_ratio > linear_ratio:
        return _rotation_command(wz, cfg)
    return _translation_command(vx, vy, cfg)
