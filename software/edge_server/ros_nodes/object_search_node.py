"""object_search_node：自主探索 + 目标搜索 + 接近。

服务两个 Action：
- /center/explore        Explore：按 frontier 探索到覆盖率阈值；
- /center/search_object  SearchObject：搜索指定标签，确认后导航到目标前
                          standoff 米处，返回目标位姿/置信度。

搜索期间持续消费 /center/target_detections；导航失败的前沿进入黑名单。
"""

from __future__ import annotations

import dataclasses
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from edge_server.core.object_search import SearchConfig, SearchCoordinator, SearchState
from edge_server.core.occupancy_map import Frontier
from edge_server.core.pose_utils import Pose2D, make_pose_stamped, pose_from_msg
from edge_server.core.target_tracker import Track
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.action import Explore, NavigateTo, SearchObject
from custom_msgs.msg import (
    FrontierList,
    LocalizationStatus,
    MapInfo,
    NavStatus,
    SafetyState,
    SearchStatus,
    TargetList,
)
from custom_msgs.srv import MapControl


class ObjectSearchNode(Node):
    def __init__(self):
        super().__init__("object_search_node")
        self.cfg = load_config(self)
        cfg = self.cfg.get("search", {})
        self.default_max_duration = float(cfg.get("default_max_duration_sec", 300.0))
        self.default_target_coverage = float(cfg.get("default_target_coverage", 0.90))
        self.track_memory_sec = float(cfg.get("track_memory_sec", 2.0))
        self.feedback_interval = float(cfg.get("feedback_interval_sec", 0.5))
        self.cfg_search = SearchConfig(
            confirm_hits=int(cfg.get("confirm_hits", 3)),
            confirm_time=float(cfg.get("confirm_time_sec", 0.4)),
            standoff_distance=float(cfg.get("standoff_distance", 0.6)),
            approach_tolerance=float(cfg.get("approach_tolerance", 0.30)),
            map_wait_timeout=float(cfg.get("map_wait_timeout_sec", 20.0)),
            target_lost_timeout=float(cfg.get("target_lost_timeout_sec", 3.0)),
            frontier_min_size=int(cfg.get("frontier_min_size", 5)),
            max_frontier_distance=float(cfg.get("max_frontier_distance", 15.0)),
            blacklist_radius=float(cfg.get("blacklist_radius", 0.8)),
            blacklist_sec=float(cfg.get("blacklist_sec", 30.0)),
            replan_delay=float(cfg.get("replan_delay_sec", 0.5)),
            min_confidence=float(cfg.get("min_confidence", 0.5)),
            explore_coverage=self.default_target_coverage,
            max_approach_failures=int(cfg.get("max_approach_failures", 3)),
        )
        self.rate = max(1.0, float(cfg.get("rate_hz", 5.0)))

        self.pub_status = self.create_publisher(SearchStatus, "/center/search_status", 10)
        self.pub_nav_zero = self.create_publisher(Twist, "/nav/cmd_vel", 10)
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(MapInfo, "/center/map_info", self.on_map_info, 10)
        self.create_subscription(
            FrontierList, "/center/frontiers", self.on_frontiers, 10
        )
        self.create_subscription(
            TargetList, "/center/target_detections", self.on_targets, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/center/pose", self.on_pose, 10
        )
        self.create_subscription(
            LocalizationStatus, "/center/localization_status", self.on_loc, 10
        )
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.create_subscription(NavStatus, "/center/nav_status", self.on_nav_status, 10)

        self.nav_client = ActionClient(
            self,
            NavigateTo,
            "/center/navigate_to",
            callback_group=self._cb_group,
        )
        self.map_client = self.create_client(
            MapControl, "/center/map_control", callback_group=self._cb_group
        )
        self.search_server = ActionServer(
            self,
            SearchObject,
            "/center/search_object",
            execute_callback=self.execute_search,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self._cb_group,
        )
        self.explore_server = ActionServer(
            self,
            Explore,
            "/center/explore",
            execute_callback=self.execute_explore,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self._cb_group,
        )

        self._lock = threading.Lock()
        self._active = False
        self._map_info: MapInfo | None = None
        self._frontiers: list[Frontier] = []
        self._pose: Pose2D | None = None
        self._pose_time = 0.0
        self._loc_quality = 0.0
        self._emergency = False
        self._nav_status: NavStatus | None = None
        self._tracks: dict[str, Track] = {}
        self._nav_future = None
        self._nav_result_future = None
        self._nav_goal_handle = None
        self._nav_active = False
        self._nav_goal: Pose2D | None = None
        self._nav_result: str | None = None
        self._nav_result_goal: Pose2D | None = None
        self._nav_deadline = 0.0
        self._last_feedback = 0.0
        self._steady_clock = create_steady_clock()
        self.get_logger().info("object_search_node started")

    # ------------------------------------------------------------- callbacks
    def on_map_info(self, msg: MapInfo):
        self._map_info = msg

    def on_frontiers(self, msg: FrontierList):
        self._frontiers = [
            Frontier(
                x=float(f.position.x),
                y=float(f.position.y),
                size=int(f.size),
                distance=float(f.distance),
            )
            for f in msg.frontiers
        ]

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self._pose = pose_from_msg(msg)
        self._pose_time = time.monotonic()

    def on_loc(self, msg: LocalizationStatus):
        self._loc_quality = float(msg.quality)

    def on_safety(self, msg: SafetyState):
        self._emergency = bool(msg.emergency_stop)

    def on_nav_status(self, msg: NavStatus):
        self._nav_status = msg

    def on_targets(self, msg: TargetList):
        now = time.monotonic()
        for det in msg.detections:
            if det.confidence < self.cfg_search.min_confidence:
                continue
            position = None
            frame = ""
            if det.position_frame in ("map", "odom"):
                position = (float(det.position.x), float(det.position.y))
                frame = det.position_frame
            prev = self._tracks.get(det.track_id)
            if prev is None:
                track = Track(
                    track_id=det.track_id,
                    label=det.label,
                    bbox=(det.x_min, det.y_min, det.x_max, det.y_max),
                    confidence=float(det.confidence),
                    position=position,
                    position_frame=frame,
                    hits=1,
                    first_seen=now,
                    last_seen=now,
                    distance_m=float(det.distance_m),
                    bearing_rad=float(det.bearing_rad),
                    distance_source=det.distance_source,
                )
                if position is not None:
                    track.history.append(position)
                self._tracks[det.track_id] = track
            else:
                prev.label = det.label
                prev.bbox = (det.x_min, det.y_min, det.x_max, det.y_max)
                prev.confidence = float(det.confidence)
                prev.last_seen = now
                prev.hits += 1
                prev.distance_m = float(det.distance_m)
                prev.bearing_rad = float(det.bearing_rad)
                prev.distance_source = det.distance_source
                if position is not None:
                    prev.position = position
                    prev.position_frame = frame
                    prev.history.append(position)
        for track_id, track in list(self._tracks.items()):
            if now - track.last_seen > self.track_memory_sec:
                self._tracks.pop(track_id, None)

    def _best_track(self, label: str) -> Track | None:
        candidates = [
            t
            for t in self._tracks.values()
            if (not label or t.label == label)
            and t.position is not None
            and t.confidence >= self.cfg_search.min_confidence
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda t: (t.confidence, t.hits, t.last_seen))

    # ---------------------------------------------------------------- actions
    def goal_cb(self, goal_request):
        if self._emergency:
            return GoalResponse.REJECT
        with self._lock:
            if self._active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_search(self, goal_handle):
        request = goal_handle.request
        target = (request.target_label or "").strip()
        if not target:
            return self._abort_search(goal_handle, "target_label is required")
        duration = float(request.max_duration or self.default_max_duration)
        cfg = dataclasses.replace(
            self.cfg_search,
            min_confidence=float(request.min_confidence or self.cfg_search.min_confidence),
            standoff_distance=float(
                request.standoff_distance or self.cfg_search.standoff_distance
            ),
        )
        coordinator = SearchCoordinator(
            cfg,
            target_label=target,
            now=time.monotonic(),
            max_duration=duration,
            explore_only=False,
        )
        if not self._map_ready() and request.explore_if_not_found:
            self._start_mapping()
        return self._run(
            goal_handle,
            coordinator,
            explore_only=False,
            save_map=bool(request.save_map),
        )

    def execute_explore(self, goal_handle):
        request = goal_handle.request
        duration = float(request.max_duration or self.default_max_duration)
        coverage = float(request.target_coverage or self.default_target_coverage)
        cfg = dataclasses.replace(self.cfg_search, explore_coverage=coverage)
        coordinator = SearchCoordinator(
            cfg,
            target_label="",
            now=time.monotonic(),
            max_duration=duration,
            explore_only=True,
        )
        if not self._map_ready():
            self._start_mapping()
        return self._run(
            goal_handle, coordinator, explore_only=True, save_map=False
        )

    def _run(
        self,
        goal_handle,
        coordinator: SearchCoordinator,
        explore_only: bool,
        save_map: bool,
    ):
        with self._lock:
            if self._active:
                return self._abort_common(goal_handle, explore_only, "another search is active")
            self._active = True
        self._tracks.clear()
        self._nav_result = None
        self._nav_active = False
        started = time.monotonic()
        success = False
        info = "canceled"
        try:
            while rclpy.ok():
                now = time.monotonic()
                if goal_handle.is_cancel_requested:
                    coordinator.cancel("canceled by client")
                    info = "canceled"
                    break
                if self._emergency:
                    coordinator.cancel("emergency stop")
                    info = "emergency stop"
                    break

                self._poll_nav(now)
                if self._nav_result is not None:
                    result = self._nav_result
                    goal = self._nav_result_goal
                    self._nav_result = None
                    self._nav_result_goal = None
                    coordinator.update_nav(active=False, result=result, goal=goal, now=now)

                pose = self._pose if (now - self._pose_time) <= 1.0 else None
                coordinator.update_pose(pose)
                coordinator.update_map(self._known_ratio(), self._frontiers, now=now)
                coordinator.update_track(self._best_track(coordinator.target_label))

                decision = coordinator.decide(now)
                if decision.action in ("navigate", "approach"):
                    if not self._nav_active and decision.goal is not None:
                        remaining = max(10.0, coordinator.max_duration - (now - started))
                        self._send_nav_goal(decision.goal, remaining)
                elif decision.action == "cancel":
                    self._cancel_nav()
                    coordinator.update_nav(active=False, result="canceled", now=now)
                elif decision.action == "found":
                    success = True
                    info = decision.reason
                    break
                elif decision.action == "failed":
                    success = False
                    info = decision.reason
                    break

                self._publish_status(coordinator, now)
                self._publish_feedback(goal_handle, coordinator, explore_only, now)
                time.sleep(1.0 / self.rate)

            if save_map and success:
                self._save_map()
        finally:
            self._cancel_nav()
            self._publish_status(coordinator, time.monotonic())
            with self._lock:
                self._active = False

        elapsed = time.monotonic() - started
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        if explore_only:
            result = Explore.Result()
            result.success = bool(success)
            result.info = info
            result.coverage = float(coordinator.known_ratio)
            result.elapsed_sec = elapsed
            return result
        result = SearchObject.Result()
        result.found = bool(success)
        result.info = info
        result.confidence = self._best_confidence(coordinator)
        result.elapsed_sec = elapsed
        target_pose = self._confirmed_pose(coordinator)
        if target_pose is not None:
            result.target_pose = make_pose_stamped(
                "map", target_pose, self.get_clock().now().to_msg()
            )
        approach = coordinator.approach_goal()
        if approach is not None:
            result.approach_pose = make_pose_stamped(
                approach.frame_id or "map", approach, self.get_clock().now().to_msg()
            )
        return result

    # -------------------------------------------------------------- navigation
    def _send_nav_goal(self, pose: Pose2D, timeout: float):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to action server unavailable")
            self._nav_result = "failed"
            self._nav_result_goal = pose
            return
        goal = NavigateTo.Goal()
        goal.target_pose = make_pose_stamped(
            pose.frame_id or "map", pose, self.get_clock().now().to_msg()
        )
        goal.goal_id = f"search_{int(time.time() * 1000)}"
        goal.timeout_sec = float(timeout)
        goal.tolerance_xy = 0.20
        goal.tolerance_yaw = 0.35
        goal.avoid_obstacles = True
        self._nav_goal = pose
        self._nav_active = True
        self._nav_deadline = time.monotonic() + float(timeout)
        self._nav_future = self.nav_client.send_goal_async(goal)
        self.get_logger().info(
            f"navigate to ({pose.x:.2f}, {pose.y:.2f}, {pose.yaw:.2f})"
        )

    def _poll_nav(self, now: float):
        if self._nav_future is not None and self._nav_future.done():
            try:
                handle = self._nav_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"navigate goal failed: {exc}")
                handle = None
            self._nav_future = None
            if handle is None or not handle.accepted:
                self._nav_active = False
                self._nav_result = "failed"
                self._nav_result_goal = self._nav_goal
                return
            self._nav_goal_handle = handle
            self._nav_result_future = handle.get_result_async()
        if self._nav_result_future is not None and self._nav_result_future.done():
            try:
                wrapped = self._nav_result_future.result()
                success = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            except Exception:  # noqa: BLE001
                success = False
            self._nav_active = False
            self._nav_result = "succeeded" if success else "failed"
            self._nav_result_goal = self._nav_goal
            self._nav_goal_handle = None
            self._nav_result_future = None
            return
        if self._nav_active and now > self._nav_deadline:
            self.get_logger().warn("search navigation timed out")
            self._cancel_nav()
            self._nav_result = "failed"
            self._nav_result_goal = self._nav_goal

    def _cancel_nav(self):
        handle = self._nav_goal_handle
        self._nav_goal_handle = None
        self._nav_future = None
        self._nav_result_future = None
        self._nav_active = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self.pub_nav_zero.publish(Twist())

    # ---------------------------------------------------------------- helpers
    def _map_ready(self) -> bool:
        info = self._map_info
        return bool(info is not None and info.width > 1 and info.known_ratio > 0.005)

    def _known_ratio(self) -> float:
        return float(self._map_info.known_ratio) if self._map_info is not None else 0.0

    def _start_mapping(self):
        if not self.map_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("map_control service unavailable")
            return
        req = MapControl.Request()
        req.command = "start"
        req.map_name = ""
        self.map_client.call_async(req)

    def _save_map(self):
        if not self.map_client.wait_for_service(timeout_sec=0.5):
            return
        req = MapControl.Request()
        req.command = "save"
        req.map_name = ""
        self.map_client.call_async(req)

    def _confirmed_pose(self, coordinator: SearchCoordinator) -> Pose2D | None:
        track = coordinator.confirmed or coordinator.best
        if track is None or track.position is None:
            return None
        return Pose2D(track.position[0], track.position[1], 0.0, "map")

    @staticmethod
    def _best_confidence(coordinator: SearchCoordinator) -> float:
        track = coordinator.confirmed or coordinator.best
        return float(track.confidence) if track is not None else 0.0

    def _publish_status(self, coordinator: SearchCoordinator, now: float):
        status = coordinator.status(now)
        msg = SearchStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = ""
        msg.state = status["state"]
        msg.target_label = coordinator.target_label
        msg.best_track_id = status["best_track_id"]
        msg.best_confidence = float(status["best_confidence"])
        msg.explored_ratio = float(status["explored_ratio"])
        msg.elapsed_sec = float(status["elapsed"])
        msg.detail = status["detail"]
        position = status.get("best_position")
        if position is not None and status.get("best_frame") in ("map", "odom"):
            msg.best_pose = make_pose_stamped(
                status["best_frame"],
                Pose2D(position[0], position[1], 0.0, status["best_frame"]),
                self.get_clock().now().to_msg(),
            )
        self.pub_status.publish(msg)

    def _publish_feedback(self, goal_handle, coordinator, explore_only: bool, now: float):
        if now - self._last_feedback < self.feedback_interval:
            return
        self._last_feedback = now
        status = coordinator.status(now)
        best_pose = make_pose_stamped(
            "map", Pose2D(0.0, 0.0, 0.0, "map"), self.get_clock().now().to_msg()
        )
        if status.get("best_position") is not None:
            best_pose = make_pose_stamped(
                status.get("best_frame") or "map",
                Pose2D(
                    status["best_position"][0],
                    status["best_position"][1],
                    0.0,
                    status.get("best_frame") or "map",
                ),
                self.get_clock().now().to_msg(),
            )
        if explore_only:
            fb = Explore.Feedback()
            fb.state = status["state"]
            fb.coverage = float(status["explored_ratio"])
            fb.detail = status["detail"]
        else:
            fb = SearchObject.Feedback()
            fb.state = status["state"]
            fb.target_label = coordinator.target_label
            fb.explored_ratio = float(status["explored_ratio"])
            fb.best_confidence = float(status["best_confidence"])
            fb.best_pose = best_pose
            fb.detail = status["detail"]
        goal_handle.publish_feedback(fb)

    def _abort_common(self, goal_handle, explore_only: bool, info: str):
        goal_handle.abort()
        if explore_only:
            result = Explore.Result()
            result.success = False
            result.info = info
            return result
        result = SearchObject.Result()
        result.found = False
        result.info = info
        return result

    def _abort_search(self, goal_handle, info: str):
        goal_handle.abort()
        result = SearchObject.Result()
        result.found = False
        result.info = info
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSearchNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
