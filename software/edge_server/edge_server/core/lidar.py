"""LD19/LD06/D6 2D 激光雷达协议解析与扫描累积（不依赖 ROS）。

数据帧（47 字节，230400/921600 8N1 常见）：

```text
0x54 | 0x2C | speed(2) | start_angle(2) | 12 × (distance(2) + confidence(1))
     | end_angle(2) | timestamp(2) | crc8(1)
```

角度单位 0.01°，距离单位 mm，CRC-8 多项式 0x4D（覆盖前 46 字节）。
本模块只做纯解析/分箱，串口读取和 ROS `LaserScan` 发布在
``ros_nodes/lidar_node.py``。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

PACKET_SIZE = 47
PACKET_HEADER = 0x54
PACKET_LENGTH = 0x2C
POINTS_PER_PACKET = 12
CRC_POLY = 0x4D


def crc8(data: bytes) -> int:
    """LD19 CRC-8：多项式 0x4D，初值 0。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@dataclass
class LidarPoint:
    angle_deg: float
    distance_m: float
    confidence: int


@dataclass
class LidarPacket:
    speed_dps: int
    start_angle_deg: float
    end_angle_deg: float
    timestamp_ms: int
    points: List[LidarPoint] = field(default_factory=list)


@dataclass
class ScanFrame:
    angle_min: float
    angle_max: float
    angle_increment: float
    ranges: List[float]
    intensities: List[float]
    points: int


def parse_packet(data: bytes) -> Optional[LidarPacket]:
    """解析一个完整 47 字节帧；长度/头/CRC 非法返回 None。"""
    if len(data) < PACKET_SIZE:
        return None
    frame = bytes(data[:PACKET_SIZE])
    if frame[0] != PACKET_HEADER or frame[1] != PACKET_LENGTH:
        return None
    if crc8(frame[: PACKET_SIZE - 1]) != frame[-1]:
        return None
    speed = int.from_bytes(frame[2:4], "little")
    start = int.from_bytes(frame[4:6], "little") * 0.01
    end = int.from_bytes(frame[42:44], "little") * 0.01
    timestamp = int.from_bytes(frame[44:46], "little")
    end_unwrapped = end if end >= start else end + 360.0
    points: List[LidarPoint] = []
    for index in range(POINTS_PER_PACKET):
        offset = 6 + index * 3
        distance_mm = int.from_bytes(frame[offset : offset + 2], "little")
        confidence = int(frame[offset + 2])
        angle = start + (end_unwrapped - start) * index / (POINTS_PER_PACKET - 1)
        points.append(
            LidarPoint(
                angle_deg=angle % 360.0,
                distance_m=float(distance_mm) / 1000.0,
                confidence=confidence,
            )
        )
    return LidarPacket(
        speed_dps=speed,
        start_angle_deg=start,
        end_angle_deg=end,
        timestamp_ms=timestamp,
        points=points,
    )


class ScanAccumulator:
    """把连续角度点累积成固定 bin 的 360° 扫描。"""

    def __init__(
        self,
        *,
        bins: int = 360,
        angle_min_deg: float = -180.0,
        angle_max_deg: float = 180.0,
        range_min_m: float = 0.02,
        range_max_m: float = 12.0,
        min_points: int = 30,
    ) -> None:
        self.bins = max(36, int(bins))
        self.angle_min_deg = float(angle_min_deg)
        self.angle_max_deg = float(angle_max_deg)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self.min_points = max(1, int(min_points))
        self.angle_increment_rad = math.radians(360.0 / self.bins)
        self._ranges = [math.inf] * self.bins
        self._intensities = [0.0] * self.bins
        self._count = 0
        self._last_angle: Optional[float] = None

    def add_packet(self, packet: LidarPacket) -> Optional[ScanFrame]:
        """加入一帧；检测到角度回绕且点数足够时返回上一圈扫描。"""
        completed: Optional[ScanFrame] = None
        for point in packet.points:
            angle = float(point.angle_deg) % 360.0
            if (
                self._last_angle is not None
                and angle < (self._last_angle - 180.0)
                and self._count >= self.min_points
            ):
                if completed is None:
                    completed = self._finalize()
            self._add_point(angle, point)
            self._last_angle = angle
        return completed

    def _add_point(self, angle: float, point: LidarPoint) -> None:
        self._count += 1
        distance = float(point.distance_m)
        if not math.isfinite(distance) or distance <= 0.0:
            return
        if distance < self.range_min_m or distance > self.range_max_m:
            return
        relative = (angle - self.angle_min_deg) % 360.0
        index = int(relative / (360.0 / self.bins)) % self.bins
        if distance < self._ranges[index]:
            self._ranges[index] = distance
            self._intensities[index] = max(
                0.0, min(1.0, float(point.confidence) / 255.0)
            )

    def _finalize(self) -> ScanFrame:
        frame = ScanFrame(
            angle_min=math.radians(self.angle_min_deg),
            angle_max=math.radians(self.angle_max_deg),
            angle_increment=self.angle_increment_rad,
            ranges=list(self._ranges),
            intensities=list(self._intensities),
            points=self._count,
        )
        self._ranges = [math.inf] * self.bins
        self._intensities = [0.0] * self.bins
        self._count = 0
        self._last_angle = None
        return frame
