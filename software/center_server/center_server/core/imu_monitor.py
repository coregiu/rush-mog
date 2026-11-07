"""ICM-20948 数据健康度与航向估计。

ICM-20948 是 9 轴 IMU（3 轴加速度 + 3 轴陀螺 + AK09916 三轴磁力计）。本模块
不负责 I2C 读取，而是消费 /imu/data 并回答三个问题：

1. IMU 是否在线、频率是否足够、是否有跳变/饱和；
2. 磁力计是否可用（小车电机/电池的磁场干扰会让磁航向不可信）；
3. 融合后的平面航向（陀螺积分 + 磁航向慢速校正），供定位健康度与调试使用。

真正的 EKF 融合由 robot_localization 完成；这里的结果用于门控和降级策略，
不替代 EKF 输出。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional, Tuple

from .pose_utils import wrap_angle


@dataclass
class ImuStatus:
    online: bool = False
    rate_hz: float = 0.0
    gap_sec: float = 0.0
    accel_norm: float = 0.0
    gyro_norm: float = 0.0
    mag_norm: float = 0.0
    mag_ok: bool = False
    saturated: bool = False
    quality: float = 0.0
    yaw_rad: float = 0.0
    yaw_variance: float = 1.0
    gyro_bias_z: float = 0.0
    detail: str = ""


class HeadingEstimator:
    """平面航向：陀螺积分 + 可选磁航向互补校正。"""

    def __init__(self, mag_blend: float = 0.02, initial_yaw: float = 0.0):
        self.mag_blend = float(mag_blend)
        self.yaw = wrap_angle(initial_yaw)
        self.yaw_variance = 1.0
        self.bias = 0.0

    def reset(self, yaw: float = 0.0):
        self.yaw = wrap_angle(yaw)
        self.yaw_variance = 1.0

    def update(
        self,
        gyro_z: float,
        dt: float,
        mag_yaw: Optional[float] = None,
        mag_ok: bool = False,
        bias: float = 0.0,
    ) -> float:
        dt = max(0.0, min(float(dt), 0.5))
        self.bias = float(bias)
        self.yaw = wrap_angle(self.yaw + (float(gyro_z) - self.bias) * dt)
        self.yaw_variance = min(1.0e6, self.yaw_variance + dt * 0.01)
        if mag_ok and mag_yaw is not None and math.isfinite(mag_yaw):
            err = wrap_angle(float(mag_yaw) - self.yaw)
            self.yaw = wrap_angle(self.yaw + self.mag_blend * err)
            self.yaw_variance = max(0.002, self.yaw_variance * 0.9)
        return self.yaw

    @staticmethod
    def mag_yaw(mx: float, my: float, declination_rad: float = 0.0) -> float:
        """水平安装时的磁航向；安装偏航通过 declination_rad 补偿。"""
        return wrap_angle(math.atan2(-float(my), float(mx)) + float(declination_rad))


class ImuMonitor:
    def __init__(self, cfg: Optional[dict] = None):
        cfg = cfg or {}
        self.expected_rate = float(cfg.get("expected_rate_hz", 100.0))
        self.max_gap = float(cfg.get("max_gap_sec", 0.2))
        self.accel_min = float(cfg.get("accel_norm_min", 7.0))
        self.accel_max = float(cfg.get("accel_norm_max", 13.0))
        self.gyro_saturation = float(cfg.get("gyro_saturation_rad_s", 34.0))
        self.mag_min = float(cfg.get("mag_norm_min_ut", 10.0))
        self.mag_max = float(cfg.get("mag_norm_max_ut", 120.0))
        self.mag_disturb_ratio = float(cfg.get("mag_disturb_ratio", 0.35))
        self.stationary_gyro = float(cfg.get("stationary_gyro_rad_s", 0.05))
        self.bias_alpha = float(cfg.get("bias_alpha", 0.01))
        self.mag_declination = float(cfg.get("mag_declination_rad", 0.0))
        self.mount_yaw = float(cfg.get("mount_yaw_rad", 0.0))
        self.heading = HeadingEstimator(
            mag_blend=float(cfg.get("mag_blend", 0.02)),
            initial_yaw=float(cfg.get("initial_yaw_rad", 0.0)),
        )
        self._stamps: Deque[float] = deque(maxlen=100)
        self._last_time: Optional[float] = None
        self._mag_baseline: Optional[float] = None
        self._bias_z = 0.0
        self._status = ImuStatus()
        self._online = False

    # ------------------------------------------------------------------ update
    def update(
        self,
        accel: Tuple[float, float, float],
        gyro: Tuple[float, float, float],
        mag: Optional[Tuple[float, float, float]],
        now: float,
    ) -> ImuStatus:
        now = float(now)
        raw_dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        # 长间隔不参与积分，避免一次跳变把航向打飞；诊断仍保留真实 gap
        dt = raw_dt if raw_dt <= self.max_gap else 0.0
        self._last_time = now
        self._stamps.append(now)

        ax, ay, az = (float(accel[0]), float(accel[1]), float(accel[2]))
        gx, gy, gz = (float(gyro[0]), float(gyro[1]), float(gyro[2]))
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        saturated = max(abs(gx), abs(gy), abs(gz)) >= self.gyro_saturation

        # 静止时估计 z 轴零偏
        stationary = (
            gyro_norm <= self.stationary_gyro
            and self.accel_min <= accel_norm <= self.accel_max
        )
        if stationary:
            self._bias_z += self.bias_alpha * (gz - self._bias_z)

        mag_norm = 0.0
        mag_ok = False
        mag_yaw = None
        if mag is not None:
            mx, my, mz = (float(mag[0]), float(mag[1]), float(mag[2]))
            mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
            valid = self.mag_min <= mag_norm <= self.mag_max
            if valid:
                if self._mag_baseline is None:
                    self._mag_baseline = mag_norm
                else:
                    deviation = abs(mag_norm - self._mag_baseline) / max(
                        self._mag_baseline, 1e-6
                    )
                    mag_ok = deviation <= self.mag_disturb_ratio and not saturated
                    if deviation <= self.mag_disturb_ratio:
                        self._mag_baseline += 0.01 * (mag_norm - self._mag_baseline)
                mag_yaw = HeadingEstimator.mag_yaw(
                    mx, my, self.mag_declination + self.mount_yaw
                )

        if not saturated:
            self.heading.update(
                gz, dt, mag_yaw=mag_yaw, mag_ok=mag_ok, bias=self._bias_z
            )

        rate = self._rate()
        gap = raw_dt

        quality = 1.0
        reasons = []
        if rate < self.expected_rate * 0.5:
            quality -= 0.35
            reasons.append(f"low_rate={rate:.1f}Hz")
        if raw_dt > self.max_gap:
            quality -= 0.25
            reasons.append("timestamp_gap")
        if not (self.accel_min <= accel_norm <= self.accel_max):
            quality -= 0.2
            reasons.append("accel_abnormal")
        if saturated:
            quality -= 0.2
            reasons.append("gyro_saturation")
        if mag is not None and not mag_ok:
            quality -= 0.1
            reasons.append("mag_disturbed")
        quality = max(0.0, min(1.0, quality))

        self._online = True
        self._status = ImuStatus(
            online=True,
            rate_hz=rate,
            gap_sec=gap,
            accel_norm=accel_norm,
            gyro_norm=gyro_norm,
            mag_norm=mag_norm,
            mag_ok=mag_ok,
            saturated=saturated,
            quality=quality,
            yaw_rad=self.heading.yaw,
            yaw_variance=self.heading.yaw_variance,
            gyro_bias_z=self._bias_z,
            detail=",".join(reasons) if reasons else "ok",
        )
        return self._status

    def mark_stale(self, now: float):
        """长时间没有 IMU 数据时由节点调用。"""
        self._online = False
        self._status.online = False
        self._status.quality = 0.0
        self._status.detail = "imu_stale"

    def reset_heading(self, yaw: float):
        self.heading.reset(yaw)
        self._mag_baseline = None

    # ------------------------------------------------------------------- query
    @property
    def status(self) -> ImuStatus:
        return self._status

    @property
    def online(self) -> bool:
        return self._online

    def _rate(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 1e-6:
            return 0.0
        return float((len(self._stamps) - 1) / span)
