"""目标检测结果 -> 距离/方位 -> map 系坐标 -> 接近点。

不依赖深度相机：优先用 2D 雷达在目标方位附近的距离；雷达不可用时用目标
类别先验高度 + bbox 像素高度估距。两者都不可用时仍输出方位，但位置不可信。
后续接深度相机时，只需在 estimate() 里增加 depth 分支。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from .pose_utils import Pose2D, transform_point, wrap_angle


@dataclass
class CameraModel:
    width: int = 640
    height: int = 480
    hfov_deg: float = 62.0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0

    def __post_init__(self):
        self.width = int(self.width)
        self.height = int(self.height)
        if self.fx <= 0.0 and self.hfov_deg > 0.0:
            self.fx = (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        if self.fy <= 0.0:
            self.fy = self.fx
        if self.cx <= 0.0:
            self.cx = self.width / 2.0
        if self.cy <= 0.0:
            self.cy = self.height / 2.0

    @classmethod
    def from_config(cls, cfg: Optional[dict]) -> "CameraModel":
        cfg = cfg or {}
        return cls(
            width=int(cfg.get("width", 640)),
            height=int(cfg.get("height", 480)),
            hfov_deg=float(cfg.get("hfov_deg", 62.0)),
            fx=float(cfg.get("fx", 0.0)),
            fy=float(cfg.get("fy", 0.0)),
            cx=float(cfg.get("cx", 0.0)),
            cy=float(cfg.get("cy", 0.0)),
        )

    def bbox_pixels(self, bbox_norm: Sequence[float]) -> Tuple[float, float, float, float]:
        x0, y0, x1, y1 = (float(v) for v in bbox_norm)
        x0, x1 = sorted((max(0.0, min(1.0, x0)), max(0.0, min(1.0, x1))))
        y0, y1 = sorted((max(0.0, min(1.0, y0)), max(0.0, min(1.0, y1))))
        return (
            x0 * self.width,
            y0 * self.height,
            x1 * self.width,
            y1 * self.height,
        )

    def bearing_from_u(self, u_px: float) -> float:
        return math.atan((float(u_px) - self.cx) / max(1e-6, self.fx))

    def bearing_from_bbox(self, bbox_norm: Sequence[float]) -> float:
        x0, _, x1, _ = self.bbox_pixels(bbox_norm)
        return self.bearing_from_u((x0 + x1) / 2.0)

    def distance_from_height(self, bbox_norm: Sequence[float], real_height_m: float) -> float:
        if real_height_m <= 0.0:
            return 0.0
        _, y0, _, y1 = self.bbox_pixels(bbox_norm)
        pixel_height = max(0.0, y1 - y0)
        if pixel_height <= 1.0:
            return 0.0
        return float(self.fy * float(real_height_m) / pixel_height)


@dataclass
class TargetEstimate:
    label: str
    confidence: float
    bearing_rad: float
    distance_m: float
    distance_source: str
    x: float = 0.0
    y: float = 0.0
    position_frame: str = "base_link"
    var_x: float = 1.0
    var_y: float = 1.0
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    @property
    def has_position(self) -> bool:
        return self.distance_m > 0.0 and self.distance_source != "none"


def lidar_distance(
    ranges,
    angle_min: float,
    angle_increment: float,
    bearing: float,
    window_deg: float = 6.0,
    range_min: float = 0.05,
    range_max: float = 12.0,
    statistic: str = "min",
) -> Tuple[float, int]:
    """返回目标方位附近的有效雷达距离与有效光束数。"""
    arr = np.asarray(ranges, dtype=np.float64).reshape(-1)
    if arr.size == 0 or abs(float(angle_increment)) < 1e-9:
        return 0.0, 0
    angles = float(angle_min) + np.arange(arr.size) * float(angle_increment)
    diff = np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing))
    mask = (
        np.isfinite(arr)
        & (arr > float(range_min))
        & (arr < float(range_max))
        & (np.abs(diff) <= math.radians(float(window_deg) / 2.0))
    )
    valid = arr[mask]
    if valid.size == 0:
        return 0.0, 0
    if statistic == "min" or valid.size < 3:
        return float(np.min(valid)), int(valid.size)
    return float(np.median(valid)), int(valid.size)


class TargetLocalizer:
    def __init__(
        self,
        camera_cfg: Optional[dict] = None,
        targets_cfg: Optional[dict] = None,
        lidar_window_deg: float = 6.0,
        prefer_lidar: bool = True,
    ):
        self.camera = CameraModel.from_config(camera_cfg)
        self.targets = dict(targets_cfg or {})
        self.lidar_window_deg = float(lidar_window_deg)
        self.prefer_lidar = bool(prefer_lidar)

    def real_height(self, label: str) -> float:
        cfg = self.targets.get(label) or {}
        return float(cfg.get("real_height_m", 0.0) or 0.0)

    def estimate(
        self,
        label: str,
        confidence: float,
        bbox_norm: Sequence[float],
        scan: Optional[dict] = None,
        robot_pose: Optional[Pose2D] = None,
    ) -> TargetEstimate:
        bearing = self.camera.bearing_from_bbox(bbox_norm)
        lidar_d = 0.0
        lidar_n = 0
        if scan is not None:
            lidar_d, lidar_n = lidar_distance(
                scan.get("ranges", []),
                scan.get("angle_min", 0.0),
                scan.get("angle_increment", 0.0),
                bearing,
                window_deg=self.lidar_window_deg,
                range_min=float(scan.get("range_min", 0.05)),
                range_max=float(scan.get("range_max", 12.0)),
                statistic="min",
            )
        height_d = self.camera.distance_from_height(bbox_norm, self.real_height(label))

        distance = 0.0
        source = "none"
        if self.prefer_lidar and lidar_d > 0.0 and lidar_n >= 2:
            distance, source = lidar_d, "lidar"
        elif height_d > 0.0:
            distance, source = height_d, "bbox_height"
        elif lidar_d > 0.0:
            distance, source = lidar_d, "lidar"

        if source == "lidar":
            source_factor, var = 1.0, 0.05 ** 2
        elif source == "bbox_height":
            source_factor, var = 0.75, 0.20 ** 2
        else:
            source_factor, var = 0.35, 1.0 ** 2

        est = TargetEstimate(
            label=label,
            confidence=max(0.0, min(1.0, float(confidence) * source_factor)),
            bearing_rad=bearing,
            distance_m=distance,
            distance_source=source,
            bbox=tuple(float(v) for v in bbox_norm),
            var_x=var,
            var_y=var,
        )
        if distance <= 0.0:
            return est

        if robot_pose is not None and robot_pose.frame_id in ("map", "odom"):
            x, y = transform_point(robot_pose, distance * math.cos(bearing), distance * math.sin(bearing))
            est.x, est.y = x, y
            est.position_frame = robot_pose.frame_id
        else:
            est.x = distance * math.cos(bearing)
            est.y = distance * math.sin(bearing)
            est.position_frame = "base_link"
        return est

    def approach_pose(
        self,
        target: Tuple[float, float],
        robot: Pose2D,
        standoff: float = 0.6,
    ) -> Pose2D:
        return approach_pose(target, robot, standoff)


def approach_pose(
    target: Tuple[float, float],
    robot: Pose2D,
    standoff: float = 0.6,
) -> Pose2D:
    """从目标沿“目标->机器人”方向退开 standoff 米，朝向目标。"""
    tx, ty = float(target[0]), float(target[1])
    dx, dy = robot.x - tx, robot.y - ty
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        dx, dy = -math.cos(robot.yaw), -math.sin(robot.yaw)
        norm = 1.0
    ux, uy = dx / norm, dy / norm
    ax = tx + ux * float(standoff)
    ay = ty + uy * float(standoff)
    yaw = math.atan2(ty - ay, tx - ax)
    return Pose2D(ax, ay, wrap_angle(yaw), robot.frame_id)


def target_distance(robot: Optional[Pose2D], target: Tuple[float, float]) -> float:
    if robot is None:
        return math.inf
    return math.hypot(float(target[0]) - robot.x, float(target[1]) - robot.y)
