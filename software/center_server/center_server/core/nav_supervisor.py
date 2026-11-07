"""导航目标校验、进度监控与内置直航控制器。

Nav2 可用时优先用 Nav2；不可用时（轻量部署、无 Nav2 的 Humble 基础环境）
navigation_node 使用本模块的 DirectController 直接下发 /nav/cmd_vel，至少能
完成“到点导航”和搜索任务的移动闭环。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .pose_utils import Pose2D, angle_diff, to_body_frame, wrap_angle


@dataclass
class Goal:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    tolerance_xy: float = 0.20
    tolerance_yaw: float = 0.30
    timeout_sec: float = 120.0
    goal_id: str = ""

    def is_finite(self) -> bool:
        return all(math.isfinite(float(v)) for v in (self.x, self.y, self.yaw))


@dataclass
class DirectCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    done: bool = False
    reason: str = "ok"


class GoalValidator:
    def __init__(self, cfg: Optional[dict] = None):
        cfg = cfg or {}
        self.min_quality = float(cfg.get("min_localization_quality", 0.35))
        self.max_tolerance_xy = float(cfg.get("max_tolerance_xy", 1.0))
        self.max_tolerance_yaw = float(cfg.get("max_tolerance_yaw", 1.57))

    def validate(
        self,
        goal: Goal,
        map_ready: bool = True,
        localization_quality: float = 1.0,
        occupancy=None,
    ) -> Tuple[bool, str]:
        if not goal.is_finite():
            return False, "goal contains non-finite values"
        if goal.tolerance_xy <= 0.0 or goal.tolerance_xy > self.max_tolerance_xy:
            return False, f"invalid tolerance_xy={goal.tolerance_xy:.2f}"
        if goal.tolerance_yaw <= 0.0 or goal.tolerance_yaw > self.max_tolerance_yaw:
            return False, f"invalid tolerance_yaw={goal.tolerance_yaw:.2f}"
        if goal.frame_id in ("", "map"):
            if not map_ready:
                return False, "no map available for map-frame goal"
            if localization_quality < self.min_quality:
                return False, (
                    f"localization quality {localization_quality:.2f} "
                    f"< {self.min_quality:.2f}"
                )
            if occupancy is not None and not getattr(occupancy, "empty", True):
                if not occupancy.is_inside(goal.x, goal.y):
                    return False, "goal outside map bounds"
                if not occupancy.is_free(
                    goal.x, goal.y, unknown_is_free=True
                ):
                    return False, "goal cell is occupied"
        return True, "ok"

    def snap_goal(self, goal: Goal, occupancy, radius_m: Optional[float] = None) -> Optional[Goal]:
        """把落在障碍/地图外的目标吸附到最近自由栅格。"""
        if occupancy is None or getattr(occupancy, "empty", True):
            return goal
        radius = float(radius_m if radius_m is not None else max(0.4, goal.tolerance_xy * 2.0))
        free = occupancy.find_free_near(
            goal.x, goal.y, radius_m=radius, unknown_is_free=True
        )
        if free is None:
            return None
        return Goal(
            x=free.x,
            y=free.y,
            yaw=goal.yaw,
            frame_id=goal.frame_id,
            tolerance_xy=goal.tolerance_xy,
            tolerance_yaw=goal.tolerance_yaw,
            timeout_sec=goal.timeout_sec,
            goal_id=goal.goal_id,
        )


class ProgressMonitor:
    def __init__(
        self,
        stuck_timeout: float = 6.0,
        min_progress: float = 0.05,
        progress_window: Optional[float] = None,
    ):
        self.stuck_timeout = float(stuck_timeout)
        self.min_progress = float(min_progress)
        self.progress_window = float(progress_window or stuck_timeout)
        self.best_distance = math.inf
        self.last_progress_time = 0.0
        self.started = False

    def reset(self, distance: float, now: float):
        self.best_distance = float(distance)
        self.last_progress_time = float(now)
        self.started = True

    def update(self, distance: float, now: float) -> bool:
        """返回 True 表示停滞。"""
        if not self.started:
            self.reset(distance, now)
            return False
        if distance < self.best_distance - self.min_progress:
            self.best_distance = float(distance)
            self.last_progress_time = float(now)
        return (float(now) - self.last_progress_time) > self.stuck_timeout


class DirectController:
    def __init__(self, cfg: Optional[dict] = None):
        cfg = cfg or {}
        self.max_linear = float(cfg.get("max_linear_speed", 0.35))
        self.max_angular = float(cfg.get("max_angular_speed", 1.0))
        self.k_linear = float(cfg.get("k_linear", 1.2))
        self.k_angular = float(cfg.get("k_angular", 1.8))
        self.slow_radius = float(cfg.get("slow_radius", 0.6))
        self.min_linear = float(cfg.get("min_linear", 0.04))
        self.holonomic = bool(cfg.get("holonomic", True))
        self.turn_in_place = float(cfg.get("turn_in_place_rad", 0.8))
        self.stop_distance = float(cfg.get("obstacle_stop_distance", 0.35))
        self.slow_distance = float(cfg.get("obstacle_slow_distance", 0.8))

    def compute(
        self,
        robot: Pose2D,
        goal: Goal,
        front_distance: Optional[float] = None,
    ) -> DirectCommand:
        dx = float(goal.x) - robot.x
        dy = float(goal.y) - robot.y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx) if distance > 1e-6 else robot.yaw
        yaw_err = angle_diff(desired_yaw, robot.yaw)

        if distance <= goal.tolerance_xy and abs(yaw_err) <= goal.tolerance_yaw:
            return DirectCommand(0.0, 0.0, 0.0, True, "goal reached")

        body_x, body_y = to_body_frame(robot, goal.x, goal.y)

        if abs(yaw_err) > self.turn_in_place:
            wz = _clamp(self.k_angular * yaw_err, -self.max_angular, self.max_angular)
            return DirectCommand(0.0, 0.0, wz, False, "turn in place")

        scale = 1.0
        if self.slow_radius > 0.0:
            scale = min(1.0, max(0.0, distance / self.slow_radius))
        vx = self.k_linear * body_x * scale
        vy = self.k_linear * body_y * scale if self.holonomic else 0.0
        wz = _clamp(self.k_angular * yaw_err, -self.max_angular, self.max_angular)

        if distance <= goal.tolerance_xy:
            vx = vy = 0.0
            wz = _clamp(self.k_angular * yaw_err, -self.max_angular, self.max_angular)

        reason = "ok"
        if front_distance is not None and math.isfinite(front_distance):
            if front_distance <= self.stop_distance:
                vx = vy = 0.0
                reason = f"obstacle ahead ({front_distance:.2f} m)"
            elif front_distance < self.slow_distance:
                factor = max(
                    0.0,
                    (front_distance - self.stop_distance)
                    / max(1e-6, self.slow_distance - self.stop_distance),
                )
                vx *= factor
                vy *= factor
                reason = f"obstacle slow ({front_distance:.2f} m)"

        vx = _clamp(vx, -self.max_linear, self.max_linear)
        vy = _clamp(vy, -self.max_linear, self.max_linear)
        if 0.0 < abs(vx) < self.min_linear and distance > goal.tolerance_xy:
            vx = math.copysign(self.min_linear, vx)
        if 0.0 < abs(vy) < self.min_linear and distance > goal.tolerance_xy:
            vy = math.copysign(self.min_linear, vy)
        return DirectCommand(vx, vy, wz, False, reason)


def _clamp(value: float, low: float, high: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(low, min(high, value))
