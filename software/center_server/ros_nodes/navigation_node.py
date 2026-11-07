"""navigation_node：/center/navigate_to 目标导航。

后端：
- nav2   —— 转发到 Nav2 /navigate_to_pose（需要安装 nav2_msgs + Nav2）；
- direct —— 内置直航控制器，直接发布 /nav/cmd_vel（无需 Nav2，适合轻量部署）；
- auto   —— Nav2 可用时用 Nav2，否则回退 direct。

安全：
- map 系目标要求定位质量 >= min_localization_quality；
- 目标落在障碍/地图外时吸附到最近自由栅格或拒绝；
- 急停、传感器超时、导航停滞、超时都会取消并回零。
"""

from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from center_server.core.nav_supervisor import (
    DirectController,
    Goal,
    GoalValidator,
    ProgressMonitor,
)
from center_server.core.occupancy_map import OccupancyMap
from center_server.core.pose_utils import (
    Pose2D,
    make_pose_stamped,
    pose_from_msg,
    yaw_from_quaternion,
)
from center_server.core.safety_checker import SafetyChecker
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.action import NavigateTo
from custom_msgs.msg import LocalizationStatus, MapInfo, NavStatus, SafetyState

try:  # Nav2 为可选依赖
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.cfg = load_config(self)
        nav_cfg = self.cfg.get("navigation", {})
        self.backend = str(nav_cfg.get("backend", "auto")).lower()
        self.nav2_action = str(nav_cfg.get("nav2_action", "/navigate_to_pose"))
        self.default_timeout = float(nav_cfg.get("default_timeout_sec", 120.0))
        self.default_tol_xy = float(nav_cfg.get("tolerance_xy", 0.20))
        self.default_tol_yaw = float(nav_cfg.get("tolerance_yaw", 0.30))
        self.min_quality = float(nav_cfg.get("min_localization_quality", 0.35))
        self.require_scan = bool(nav_cfg.get("require_scan", True))
        self.stuck_timeout = float(nav_cfg.get("stuck_timeout_sec", 6.0))
        self.progress_min = float(nav_cfg.get("progress_min_m", 0.05))
        self.pose_timeout = float(nav_cfg.get("pose_timeout_sec", 1.0))
        self.blocked_timeout = float(nav_cfg.get("blocked_timeout_sec", 5.0))
        self.cmd_topic = str(nav_cfg.get("cmd_topic", "/nav/cmd_vel"))
        self.status_rate = max(0.5, float(nav_cfg.get("status_rate", 2.0)))

        self.goal_validator = GoalValidator(
            {
                "min_localization_quality": self.min_quality,
                "max_tolerance_xy": nav_cfg.get("max_tolerance_xy", 1.0),
                "max_tolerance_yaw": nav_cfg.get("max_tolerance_yaw", 1.57),
            }
        )
        self.controller = DirectController(nav_cfg.get("direct", {}))
        self.safety = SafetyChecker(self.cfg.get("safety", {}))
        self.progress = ProgressMonitor(
            stuck_timeout=self.stuck_timeout, min_progress=self.progress_min
        )

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_status = self.create_publisher(NavStatus, "/center/nav_status", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/center/pose", self.on_pose, 10
        )
        self.create_subscription(
            LocalizationStatus, "/center/localization_status", self.on_loc, 10
        )
        self.create_subscription(MapInfo, "/center/map_info", self.on_map_info, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/edge/radar", self.on_scan, qos_profile_sensor_data
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        self.occupancy = OccupancyMap()
        self.latest_pose: Pose2D | None = None
        self.latest_pose_time = 0.0
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time = 0.0
        self.loc_quality = 0.0
        self.map_ready = False
        self.emergency_stop = False
        self.active = False
        self._last_status = 0.0

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._cb_group = ReentrantCallbackGroup()
        self.nav2_client = None
        if NavigateToPose is not None and self.backend in ("auto", "nav2"):
            try:
                self.nav2_client = ActionClient(
                    self,
                    NavigateToPose,
                    self.nav2_action,
                    callback_group=self._cb_group,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Nav2 action client unavailable: {exc}")
        self.action_server = ActionServer(
            self,
            NavigateTo,
            "/center/navigate_to",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self._cb_group,
        )
        self._nav2_feedback = None
        self._steady_clock = create_steady_clock()
        self.create_timer(
            1.0 / self.status_rate,
            self.on_status_timer,
            clock=self._steady_clock,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"navigation_node started (backend={self.backend}, "
            f"nav2={'yes' if self.nav2_client is not None else 'no'})"
        )

    # ------------------------------------------------------------- callbacks
    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = pose_from_msg(msg)
        self.latest_pose_time = time.monotonic()

    def on_loc(self, msg: LocalizationStatus):
        self.loc_quality = float(msg.quality)

    def on_map_info(self, msg: MapInfo):
        self.map_ready = bool(
            msg.width > 1 and msg.height > 1 and msg.known_ratio > 0.005
        )

    def on_map(self, msg: OccupancyGrid):
        try:
            self.occupancy = OccupancyMap.from_msg(msg)
        except Exception:  # noqa: BLE001
            pass

    def on_safety(self, msg: SafetyState):
        was = self.emergency_stop
        self.emergency_stop = bool(msg.emergency_stop)
        if self.emergency_stop and not was and self.active:
            self.get_logger().warn("emergency stop during navigation")

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def goal_cb(self, goal_request):
        if self.emergency_stop:
            return GoalResponse.REJECT
        if not hasattr(goal_request, "target_pose"):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    # ---------------------------------------------------------------- action
    def execute_cb(self, goal_handle):
        started = time.monotonic()
        request = goal_handle.request
        goal = self._goal_from_request(request)
        backend = self._choose_backend()
        self.active = True
        try:
            if backend == "nav2":
                success, info, final_pose = self._run_nav2(goal_handle, goal)
            else:
                success, info, final_pose = self._run_direct(goal_handle, goal)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"navigation failed: {exc}")
            success, info, final_pose = False, f"navigation exception: {exc}", self._current_pose()
        finally:
            self.active = False
            self._publish_zero()
            self._publish_status(
                "succeeded" if success else "failed",
                info,
                goal,
                backend=backend,
                current_pose=final_pose,
            )
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = NavigateTo.Result()
        result.success = bool(success)
        result.info = info
        result.elapsed_sec = time.monotonic() - started
        final = final_pose or self._current_pose()
        if final is not None:
            result.final_pose = make_pose_stamped(
                final.frame_id, final, self.get_clock().now().to_msg()
            )
        return result

    def _goal_from_request(self, request) -> Goal:
        pose = request.target_pose
        yaw = yaw_from_quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return Goal(
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=yaw,
            frame_id=pose.header.frame_id or "map",
            tolerance_xy=float(request.tolerance_xy or self.default_tol_xy),
            tolerance_yaw=float(request.tolerance_yaw or self.default_tol_yaw),
            timeout_sec=float(request.timeout_sec or self.default_timeout),
            goal_id=str(request.goal_id or f"goal_{int(time.time() * 1000)}"),
        )

    def _choose_backend(self) -> str:
        if self.backend in ("nav2", "direct"):
            if self.backend == "nav2" and self.nav2_client is None:
                return "direct"
            return self.backend
        if self.nav2_client is not None and self.nav2_client.wait_for_server(
            timeout_sec=0.5
        ):
            return "nav2"
        return "direct"

    # -------------------------------------------------------------- direct
    def _run_direct(self, goal_handle, goal: Goal):
        if goal.frame_id in ("", "map") and not self.map_ready:
            return False, "no map available for map-frame goal", None
        ok, reason = self.goal_validator.validate(
            goal,
            map_ready=self.map_ready,
            localization_quality=self.loc_quality,
            occupancy=self.occupancy,
        )
        if not ok:
            return False, reason, None
        if not self.occupancy.empty:
            snapped = self.goal_validator.snap_goal(goal, self.occupancy)
            if snapped is None:
                return False, "goal has no free cell nearby", None
            goal = snapped

        started = time.monotonic()
        last_pose = self._current_pose()
        distance = _distance(last_pose, goal) if last_pose is not None else math.inf
        self.progress.reset(distance, started)
        blocked_since = None
        self._publish_status("navigating", "direct backend", goal, distance=distance)

        while rclpy.ok():
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                self._publish_zero()
                return False, "canceled", self._current_pose()
            if self.emergency_stop:
                self._publish_zero()
                return False, "emergency stop", self._current_pose()
            if (now - started) > goal.timeout_sec:
                self._publish_zero()
                return False, f"timeout after {goal.timeout_sec:.0f}s", self._current_pose()

            pose = self._current_pose()
            if pose is None:
                if now - started > max(2.0, self.pose_timeout * 3):
                    self._publish_zero()
                    return False, "no robot pose (localization/TF unavailable)", None
                time.sleep(0.1)
                continue
            if not _frames_compatible(pose.frame_id, goal.frame_id):
                self._publish_zero()
                return False, (
                    f"pose frame {pose.frame_id!r} incompatible with goal frame "
                    f"{goal.frame_id!r}"
                ), pose

            distance = _distance(pose, goal)
            if self.progress.update(distance, now):
                self._publish_zero()
                return False, f"navigation stuck at {distance:.2f} m", pose

            front = self._front_distance()
            cmd = self.controller.compute(pose, goal, front)
            twist = Twist()
            twist.linear.x = cmd.vx
            twist.linear.y = cmd.vy
            twist.angular.z = cmd.wz

            if cmd.done:
                self._publish_zero()
                self._publish_feedback(goal_handle, "succeeded", 0.0, pose, "goal reached")
                return True, f"goal reached ({distance:.2f} m)", pose

            if self.require_scan and self._scan_age() > 1.0:
                self._publish_zero()
                return False, "no laser data for obstacle avoidance", pose

            scan = self.latest_scan
            if scan is not None:
                ok, reason = self.safety.check_twist(
                    twist, scan.ranges, scan.angle_min, scan.angle_increment
                )
                if not ok:
                    self._publish_zero()
                    blocked_since = blocked_since or now
                    if now - blocked_since > self.blocked_timeout:
                        return False, f"blocked: {reason}", pose
                    time.sleep(0.1)
                    continue
            blocked_since = None
            self.pub_cmd.publish(twist)
            self._publish_feedback(goal_handle, "navigating", distance, pose, cmd.reason)
            time.sleep(0.1)
        self._publish_zero()
        return False, "node shutting down", self._current_pose()

    # ---------------------------------------------------------------- nav2
    def _run_nav2(self, goal_handle, goal: Goal):
        if self.nav2_client is None:
            return False, "nav2_msgs/Nav2 unavailable", None
        if not self.nav2_client.wait_for_server(timeout_sec=3.0):
            return False, "Nav2 /navigate_to_pose server unavailable", None
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = make_pose_stamped(
            goal.frame_id or "map",
            Pose2D(goal.x, goal.y, goal.yaw, goal.frame_id or "map"),
            self.get_clock().now().to_msg(),
        )
        self._nav2_feedback = None
        send_future = self.nav2_client.send_goal_async(
            nav_goal, feedback_callback=self._on_nav2_feedback
        )
        handle = self._wait_future(send_future, 10.0, goal_handle)
        if handle is None:
            return False, "send Nav2 goal timeout", self._current_pose()
        if not handle.accepted:
            return False, "Nav2 goal rejected", self._current_pose()

        result_future = handle.get_result_async()
        started = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested or self.emergency_stop:
                handle.cancel_goal_async()
                return False, "canceled/emergency stop", self._current_pose()
            if result_future.done():
                break
            if time.monotonic() - started > goal.timeout_sec:
                handle.cancel_goal_async()
                return False, f"Nav2 timeout after {goal.timeout_sec:.0f}s", self._current_pose()
            fb = self._nav2_feedback or {}
            self._publish_feedback(
                goal_handle,
                "navigating",
                float(fb.get("distance_remaining", 0.0)),
                self._current_pose(),
                "nav2",
            )
            time.sleep(0.1)
        if not result_future.done():
            return False, "node shutting down", self._current_pose()
        wrapped = result_future.result()
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        info = "nav2 succeeded" if success else f"nav2 failed (status={wrapped.status})"
        return success, info, self._current_pose()

    def _on_nav2_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._nav2_feedback = {
            "distance_remaining": float(getattr(fb, "distance_remaining", 0.0) or 0.0),
            "recoveries": int(getattr(fb, "number_of_recoveries", 0) or 0),
        }

    def _wait_future(self, future, timeout: float, goal_handle):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested or self.emergency_stop:
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    # -------------------------------------------------------------- helpers
    def _current_pose(self) -> Pose2D | None:
        now = time.monotonic()
        if self.latest_pose is not None and (now - self.latest_pose_time) <= 0.5:
            return self.latest_pose
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001
            return self.latest_pose
        t = transform.transform.translation
        q = transform.transform.rotation
        return Pose2D(t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), "map")

    def _front_distance(self):
        scan = self.latest_scan
        if scan is None:
            return None
        return self.safety.nearest_obstacle(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            fov_center=0.0,
            fov_deg=self.safety.front_fov_deg,
        )

    def _scan_age(self) -> float:
        if self.latest_scan_time <= 0.0:
            return math.inf
        return time.monotonic() - self.latest_scan_time

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _publish_feedback(self, goal_handle, state: str, distance: float, pose, detail: str):
        fb = NavigateTo.Feedback()
        fb.state = state
        fb.distance_remaining = float(distance) if math.isfinite(distance) else 0.0
        fb.eta_sec = 0.0
        if pose is not None:
            fb.current_pose = make_pose_stamped(
                pose.frame_id, pose, self.get_clock().now().to_msg()
            )
        fb.detail = detail
        goal_handle.publish_feedback(fb)
        self._publish_status(state, detail, None, distance=fb.distance_remaining, current_pose=pose)

    def _publish_status(
        self,
        state: str,
        detail: str,
        goal: Goal | None,
        distance: float = 0.0,
        eta: float = 0.0,
        current_pose: Pose2D | None = None,
        backend: str = "",
    ):
        msg = NavStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.goal_id = goal.goal_id if goal is not None else ""
        msg.backend = backend or self.backend
        if goal is not None:
            msg.goal = make_pose_stamped(
                goal.frame_id or "map",
                Pose2D(goal.x, goal.y, goal.yaw, goal.frame_id or "map"),
                self.get_clock().now().to_msg(),
            )
        if current_pose is not None:
            msg.current_pose = make_pose_stamped(
                current_pose.frame_id,
                current_pose,
                self.get_clock().now().to_msg(),
            )
        msg.distance_remaining = float(distance) if math.isfinite(distance) else 0.0
        msg.eta_sec = float(eta)
        msg.detail = detail
        self.pub_status.publish(msg)

    def on_status_timer(self):
        if self.active:
            return
        if time.monotonic() - self._last_status < 5.0:
            return
        self._last_status = time.monotonic()
        msg = NavStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = "idle"
        msg.backend = self.backend
        msg.detail = "waiting for goal"
        self.pub_status.publish(msg)


def _distance(pose: Pose2D | None, goal: Goal | None) -> float:
    if pose is None or goal is None:
        return math.inf
    return math.hypot(goal.x - pose.x, goal.y - pose.y)


def _frames_compatible(a: str, b: str) -> bool:
    return (a or "map") == (b or "map")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
