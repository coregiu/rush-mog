"""目标搜索/自主探索决策器（不依赖 ROS2，便于单测）。

搜索流程：

    WAITING_MAP -> EXPLORING -> (发现候选) CONFIRMING -> APPROACHING -> FOUND
                        |                               |
                        +--------- 失败/超时 ------------+-> FAILED

- EXPLORING：按 frontier 的 size/(distance+0.5) 选择探索点；
- CONFIRMING：同一 track 连续命中且位置稳定，才认为“真的看到了目标”；
- APPROACHING：导航到目标前 standoff 米处，并朝向目标；
- 导航失败的目标进入黑名单，冷却后重试其他 frontier。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from .occupancy_map import Frontier
from .pose_utils import Pose2D, wrap_angle
from .target_localizer import approach_pose, target_distance
from .target_tracker import Track


class SearchState(str, Enum):
    WAITING_MAP = "waiting_map"
    EXPLORING = "exploring"
    CONFIRMING = "confirming"
    APPROACHING = "approaching"
    FOUND = "found"
    FAILED = "failed"
    CANCELED = "canceled"

    @property
    def terminal(self) -> bool:
        return self in (SearchState.FOUND, SearchState.FAILED, SearchState.CANCELED)


@dataclass
class SearchConfig:
    confirm_hits: int = 3
    confirm_time: float = 0.4
    standoff_distance: float = 0.6
    approach_tolerance: float = 0.30
    map_wait_timeout: float = 20.0
    target_lost_timeout: float = 3.0
    frontier_min_size: int = 5
    max_frontier_distance: float = 15.0
    blacklist_radius: float = 0.8
    blacklist_sec: float = 30.0
    replan_delay: float = 0.5
    min_confidence: float = 0.5
    explore_coverage: float = 0.9
    target_min_distance: float = 0.25
    max_approach_failures: int = 3


@dataclass
class SearchDecision:
    action: str  # wait / navigate / approach / cancel / found / failed
    goal: Optional[Pose2D] = None
    reason: str = ""


class SearchCoordinator:
    def __init__(
        self,
        cfg: Optional[SearchConfig] = None,
        target_label: str = "",
        now: float = 0.0,
        max_duration: float = 300.0,
        explore_only: bool = False,
    ):
        self.cfg = cfg or SearchConfig()
        self.target_label = str(target_label or "")
        self.max_duration = float(max_duration)
        self.explore_only = bool(explore_only)
        self.start_time = float(now)
        self.state = SearchState.WAITING_MAP
        self.map_ready = False
        self.known_ratio = 0.0
        self.frontiers: List[Frontier] = []
        self.pose: Optional[Pose2D] = None
        self.candidate: Optional[Track] = None
        self.confirmed: Optional[Track] = None
        self.best: Optional[Track] = None
        self.nav_active = False
        self.nav_goal: Optional[Pose2D] = None
        self.approach_failures = 0
        self.blacklist: List[Tuple[float, float, float]] = []
        self.last_nav_failure = -1.0e9
        self.cancel_nav_once = False
        self.detail = "waiting for map"

    # ------------------------------------------------------------------ update
    def update_map(self, known_ratio: float, frontiers: Sequence[Frontier], now: float = 0.0):
        self.known_ratio = float(known_ratio)
        self.frontiers = list(frontiers or [])
        if self.known_ratio > 0.005 or self.frontiers:
            self.map_ready = True
            if self.state == SearchState.WAITING_MAP:
                self.state = SearchState.EXPLORING
                self.detail = "map ready, start exploring"

    def update_pose(self, pose: Optional[Pose2D]):
        self.pose = pose

    def update_track(self, track: Optional[Track]):
        if (
            track is not None
            and (not self.target_label or track.label == self.target_label)
            and track.confidence >= self.cfg.min_confidence
        ):
            self.candidate = track
            if self.best is None or track.confidence >= self.best.confidence:
                self.best = track
        else:
            self.candidate = None

    def update_nav(
        self,
        active: Optional[bool] = None,
        result: Optional[str] = None,
        goal: Optional[Pose2D] = None,
        now: float = 0.0,
    ):
        if active is not None:
            self.nav_active = bool(active)
        if result is None:
            return
        self.nav_active = False
        self.nav_goal = None
        if result in ("failed", "blocked"):
            self.last_nav_failure = float(now)
            bad = goal
            if bad is not None:
                self.blacklist.append(
                    (bad.x, bad.y, float(now) + self.cfg.blacklist_sec)
                )
            if self.state == SearchState.APPROACHING:
                self.approach_failures += 1
                if self.approach_failures >= self.cfg.max_approach_failures:
                    self.state = SearchState.FAILED
                    self.detail = "cannot reach confirmed target"
            elif self.state == SearchState.EXPLORING:
                self.detail = "frontier unreachable, replanning"
        elif result == "canceled":
            # 取消通常来自“发现目标后主动停止探索”，不应黑名单
            self.detail = "navigation canceled"
        elif result == "succeeded":
            self.approach_failures = 0
            self.detail = "navigation goal reached"

    def cancel(self, reason: str = "canceled"):
        if not self.state.terminal:
            self.state = SearchState.CANCELED
            self.detail = reason
        self.nav_active = False

    # ------------------------------------------------------------------ decide
    def decide(self, now: float) -> SearchDecision:
        now = float(now)
        if self.state.terminal:
            return SearchDecision("wait", None, self.detail)
        if self.max_duration > 0.0 and now - self.start_time > self.max_duration:
            return self._fail(f"search timeout after {self.max_duration:.0f}s")

        if not self.map_ready:
            if now - self.start_time > self.cfg.map_wait_timeout:
                return self._fail("no map available")
            self.state = SearchState.WAITING_MAP
            return SearchDecision("wait", None, "waiting for map")

        if self.explore_only:
            return self._decide_explore(now)

        # 1) 确认目标
        if self.candidate is not None and self._valid_target(self.candidate, now):
            if self.confirmed is None or self.confirmed.track_id != self.candidate.track_id:
                self.confirmed = self.candidate
                self.approach_failures = 0
                self.state = SearchState.APPROACHING
                self.detail = f"target confirmed: {self.confirmed.label} {self.confirmed.track_id}"
                if self.nav_active:
                    self.cancel_nav_once = True
                    return SearchDecision("cancel", None, "target confirmed, cancel exploration")

        # 2) 接近已确认目标
        if self.confirmed is not None:
            lost = now - self.confirmed.last_seen
            if lost > self.cfg.target_lost_timeout:
                self.detail = "confirmed target lost, resume exploring"
                self.confirmed = None
                self.approach_failures = 0
                self.state = SearchState.EXPLORING
                if self.nav_active:
                    self.cancel_nav_once = True
                    return SearchDecision("cancel", None, "target lost")
            else:
                self.state = SearchState.APPROACHING
                if self.pose is not None and self.confirmed.position is not None:
                    dist = target_distance(self.pose, self.confirmed.position)
                    if dist <= self.cfg.standoff_distance + self.cfg.approach_tolerance:
                        self.state = SearchState.FOUND
                        self.detail = f"target reached at {dist:.2f} m"
                        return SearchDecision("found", None, self.detail)
                if self.nav_active:
                    return SearchDecision("wait", None, "approaching confirmed target")
                if now - self.last_nav_failure < self.cfg.replan_delay:
                    return SearchDecision("wait", None, "approach cooldown")
                goal = self._approach_goal()
                if goal is None:
                    return self._fail("no robot pose for approach")
                self.nav_goal = goal
                return SearchDecision("approach", goal, "approach confirmed target")

        # 3) 候选目标 -> 停下确认（避免开过去错过）
        if self.candidate is not None:
            self.state = SearchState.CONFIRMING
            if self.nav_active:
                self.cancel_nav_once = True
                return SearchDecision("cancel", None, "candidate detected, stop to confirm")
            return SearchDecision("wait", None, "confirming candidate")

        # 4) 探索
        self.state = SearchState.EXPLORING
        if self.nav_active:
            return SearchDecision("wait", None, "exploring: navigation active")
        if self.pose is None:
            return SearchDecision("wait", None, "waiting for robot pose")
        if now - self.last_nav_failure < self.cfg.replan_delay:
            return SearchDecision("wait", None, "replan cooldown")
        frontier = self._select_frontier(now)
        if frontier is None:
            if self.known_ratio >= self.cfg.explore_coverage:
                self.state = SearchState.FOUND
                return SearchDecision("found", None, "exploration coverage reached")
            if now - self.start_time < self.cfg.map_wait_timeout:
                return SearchDecision("wait", None, "waiting for frontiers")
            return self._fail("no reachable frontier")
        yaw = math.atan2(frontier.y - self.pose.y, frontier.x - self.pose.x)
        goal = Pose2D(frontier.x, frontier.y, wrap_angle(yaw), self.pose.frame_id)
        self.nav_goal = goal
        self.detail = f"explore frontier ({frontier.x:.2f}, {frontier.y:.2f})"
        return SearchDecision("navigate", goal, self.detail)

    # ---------------------------------------------------------------- helpers
    def status(self, now: float = 0.0) -> dict:
        best = self.confirmed or self.candidate or self.best
        return {
            "state": self.state.value,
            "detail": self.detail,
            "explored_ratio": self.known_ratio,
            "elapsed": max(0.0, float(now) - self.start_time),
            "best_track_id": best.track_id if best is not None else "",
            "best_label": best.label if best is not None else "",
            "best_confidence": best.confidence if best is not None else 0.0,
            "best_position": best.position if best is not None else None,
            "best_frame": best.position_frame if best is not None else "",
        }

    def _decide_explore(self, now: float) -> SearchDecision:
        if self.known_ratio >= self.cfg.explore_coverage:
            self.state = SearchState.FOUND
            self.detail = f"coverage {self.known_ratio:.2f} reached"
            return SearchDecision("found", None, self.detail)
        if self.nav_active:
            return SearchDecision("wait", None, "exploring: navigation active")
        if self.pose is None:
            return SearchDecision("wait", None, "waiting for robot pose")
        if now - self.last_nav_failure < self.cfg.replan_delay:
            return SearchDecision("wait", None, "replan cooldown")
        frontier = self._select_frontier(now)
        if frontier is None:
            if self.known_ratio >= self.cfg.explore_coverage * 0.95:
                self.state = SearchState.FOUND
                return SearchDecision("found", None, "no frontier, coverage acceptable")
            if now - self.start_time < self.cfg.map_wait_timeout:
                return SearchDecision("wait", None, "waiting for frontiers")
            return self._fail("no reachable frontier")
        yaw = math.atan2(frontier.y - self.pose.y, frontier.x - self.pose.x)
        goal = Pose2D(frontier.x, frontier.y, wrap_angle(yaw), self.pose.frame_id)
        self.nav_goal = goal
        self.state = SearchState.EXPLORING
        self.detail = f"explore frontier ({frontier.x:.2f}, {frontier.y:.2f})"
        return SearchDecision("navigate", goal, self.detail)

    def _valid_target(self, track: Track, now: float) -> bool:
        return (
            track.hits >= self.cfg.confirm_hits
            and (now - track.first_seen) >= self.cfg.confirm_time
            and track.confidence >= self.cfg.min_confidence
            and track.position is not None
            and track.position_frame in ("map", "odom")
            and track.stable(0.35)
        )

    def _approach_goal(self) -> Optional[Pose2D]:
        if self.pose is None or self.confirmed is None or self.confirmed.position is None:
            return None
        return approach_pose(
            self.confirmed.position,
            self.pose,
            standoff=self.cfg.standoff_distance,
        )

    def approach_goal(self) -> Optional[Pose2D]:
        """公开的接近点，供 action result 使用。"""
        return self._approach_goal()

    def _select_frontier(self, now: float) -> Optional[Frontier]:
        if self.pose is None:
            return None
        active_blacklist = [
            (bx, by) for bx, by, until in self.blacklist if until > now
        ]
        best: Optional[Frontier] = None
        best_score = -1.0
        for frontier in self.frontiers:
            if frontier.size < self.cfg.frontier_min_size:
                continue
            dist = math.hypot(frontier.x - self.pose.x, frontier.y - self.pose.y)
            if dist > self.cfg.max_frontier_distance:
                continue
            if any(
                math.hypot(frontier.x - bx, frontier.y - by)
                <= self.cfg.blacklist_radius
                for bx, by in active_blacklist
            ):
                continue
            score = float(frontier.size) / (dist + 0.5)
            if score > best_score:
                best_score = score
                best = frontier
        return best

    def _fail(self, reason: str) -> SearchDecision:
        self.state = SearchState.FAILED
        self.detail = reason
        return SearchDecision("failed", None, reason)
