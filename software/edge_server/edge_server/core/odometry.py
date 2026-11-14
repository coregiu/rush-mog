"""麦克纳姆轮运动学与轮式里程计。

JGB37-520 自带霍尔编码器，四轮独立测速后积分得到 /odom。参数必须实测标定：
轮半径、轮距/轴距、编码器每输出轴一圈计数（含四倍频与减速比）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Sequence

from .pose_utils import Pose2D, wrap_angle


@dataclass
class MecanumGeometry:
    wheel_radius: float = 0.048
    wheelbase_x: float = 0.20
    track_y: float = 0.18
    encoder_cpr: float = 1320.0  # 输出轴每圈计数（11PPR * 30 * 4 的示例）
    sign_fl: float = 1.0
    sign_fr: float = 1.0
    sign_rl: float = 1.0
    sign_rr: float = 1.0

    @property
    def rotation_radius(self) -> float:
        return max(1e-6, (float(self.wheelbase_x) + float(self.track_y)) / 2.0)

    @property
    def meters_per_count(self) -> float:
        return 2.0 * math.pi * float(self.wheel_radius) / max(1.0, float(self.encoder_cpr))

    def twist_to_wheel_speeds(self, vx: float, vy: float, wz: float) -> Dict[str, float]:
        k = self.rotation_radius * float(wz)
        return {
            "front_left": self.sign_fl * (vx - vy - k),
            "front_right": self.sign_fr * (vx + vy + k),
            "rear_left": self.sign_rl * (vx + vy - k),
            "rear_right": self.sign_rr * (vx - vy + k),
        }

    def wheel_speeds_to_twist(self, speeds: Dict[str, float]):
        fl = float(speeds.get("front_left", 0.0)) * self.sign_fl
        fr = float(speeds.get("front_right", 0.0)) * self.sign_fr
        rl = float(speeds.get("rear_left", 0.0)) * self.sign_rl
        rr = float(speeds.get("rear_right", 0.0)) * self.sign_rr
        vx = (fl + fr + rl + rr) / 4.0
        vy = (-fl + fr + rl - rr) / 4.0
        wz = (-fl + fr - rl + rr) / (4.0 * self.rotation_radius)
        return vx, vy, wz

    def encoder_deltas_to_speeds(
        self, deltas: Sequence[int], dt: float, order: Sequence[str] = ("front_left", "front_right", "rear_left", "rear_right")
    ) -> Dict[str, float]:
        dt = max(1e-6, float(dt))
        return {
            name: float(delta) * self.meters_per_count / dt
            for name, delta in zip(order, deltas)
        }


@dataclass
class WheelOdometry:
    geometry: MecanumGeometry = field(default_factory=MecanumGeometry)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame_id: str = "odom"
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.linear_x = self.linear_y = self.angular_z = 0.0

    def update_from_wheel_speeds(self, speeds: Dict[str, float], dt: float):
        vx, vy, wz = self.geometry.wheel_speeds_to_twist(speeds)
        return self.update_twist(vx, vy, wz, dt)

    def update_from_encoders(self, deltas: Sequence[int], dt: float):
        speeds = self.geometry.encoder_deltas_to_speeds(deltas, dt)
        return self.update_from_wheel_speeds(speeds, dt)

    def update_twist(self, vx: float, vy: float, wz: float, dt: float):
        dt = max(0.0, min(float(dt), 0.5))
        self.linear_x, self.linear_y, self.angular_z = float(vx), float(vy), float(wz)
        # 中点积分，降低大 dt 时的航向误差
        mid_yaw = self.yaw + 0.5 * wz * dt
        c, s = math.cos(mid_yaw), math.sin(mid_yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = wrap_angle(self.yaw + wz * dt)
        return self.pose

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw, self.frame_id)

    @property
    def twist(self):
        return self.linear_x, self.linear_y, self.angular_z


def wheel_names() -> List[str]:
    return ["front_left", "front_right", "rear_left", "rear_right"]


def normalize_speeds(values: Iterable[float]) -> Dict[str, float]:
    return {name: float(v) for name, v in zip(wheel_names(), values)}
