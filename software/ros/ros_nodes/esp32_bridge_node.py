"""esp32_bridge_node：ROS2 <-> ESP32 车身遥测/运动桥。

数据路径：
    STM32 编码器 -> USART3 -> ESP32 /api/body -> HTTP -> 本节点
    /cmd_vel -> 本节点量化 -> HTTP POST /cmd (raw:...) -> ESP32 -> STM32

发布：
    /odom            nav_msgs/Odometry（四轮麦轮正运动学，20Hz）
    /joint_states    sensor_msgs/JointState（四轮位置/速度）
    /base/status     std_msgs/String（JSON，供 health_monitor_node 使用）
    TF odom->base_link（默认关闭；由 robot_localization EKF 发布，避免冲突）

订阅：
    /cmd_vel         geometry_msgs/Twist
    /center/safety_state  custom_msgs/SafetyState

安全：
    - /cmd_vel 超过 cmd_watchdog_sec 未更新 -> 停车；
    - 急停 -> 停车；
    - ESP32 车身遥测超过 stale_timeout_sec 未更新 -> 默认停车；
    - ESP32 侧对 raw: 命令有 1s 租约，ROS 进程异常退出后自动停车。
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from center_server.core.body_drive import BodyDriveConfig, twist_to_body_command
from center_server.core.odometry import MecanumGeometry, WheelOdometry, wheel_names
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import SafetyState


class Esp32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp32_bridge_node")
        self.cfg = load_config(self)
        base_cfg = self.cfg.get("base", {}) or {}
        esp_cfg = base_cfg.get("esp32", {}) or {}

        self.enabled = bool(base_cfg.get("enabled", True))
        host = str(esp_cfg.get("host", "192.168.4.1")).strip()
        if host.startswith("http://") or host.startswith("https://"):
            self.base_url = host.rstrip("/")
        else:
            self.base_url = f"http://{host}"
        body_path = str(esp_cfg.get("body_path", "/api/body"))
        cmd_path = str(esp_cfg.get("cmd_path", "/cmd"))
        self.body_url = self.base_url + (body_path if body_path.startswith("/") else "/" + body_path)
        self.cmd_url = self.base_url + (cmd_path if cmd_path.startswith("/") else "/" + cmd_path)

        self.body_poll_hz = max(1.0, float(esp_cfg.get("poll_hz", 10.0)))
        self.odom_rate_hz = max(5.0, float(esp_cfg.get("odom_rate_hz", 20.0)))
        self.status_rate_hz = max(0.2, float(esp_cfg.get("status_rate_hz", 1.0)))
        self.cmd_rate_hz = max(1.0, float(esp_cfg.get("cmd_rate_hz", 10.0)))
        self.http_timeout = max(0.1, float(esp_cfg.get("http_timeout_sec", 0.4)))
        self.stale_timeout = max(0.2, float(esp_cfg.get("stale_timeout_sec", 1.0)))
        self.cmd_watchdog = max(0.1, float(base_cfg.get("watchdog_sec", 0.5)))
        self.cmd_resend_sec = max(0.1, float(esp_cfg.get("cmd_resend_sec", 0.4)))
        self.stop_on_body_stale = bool(esp_cfg.get("stop_on_body_stale", True))
        self.publish_tf = bool(esp_cfg.get("publish_tf", False))

        self.frame_id = str(base_cfg.get("odom_frame", "odom"))
        self.base_frame = str(base_cfg.get("base_frame", "base_link"))
        self.wheel_joint_names = list(
            base_cfg.get(
                "wheel_joint_names",
                [
                    "wheel_front_left_joint",
                    "wheel_front_right_joint",
                    "wheel_rear_left_joint",
                    "wheel_rear_right_joint",
                ],
            )
        )
        self.wheel_names = wheel_names()
        self.wheel_radius = float(base_cfg.get("wheel_radius", 0.048))

        geometry = MecanumGeometry(
            wheel_radius=self.wheel_radius,
            wheelbase_x=float(base_cfg.get("wheelbase_x", 0.20)),
            track_y=float(base_cfg.get("track_y", 0.18)),
            encoder_cpr=float(base_cfg.get("encoder_cpr", 1320.0)),
            sign_fl=float(base_cfg.get("sign_fl", 1.0)),
            sign_fr=float(base_cfg.get("sign_fr", 1.0)),
            sign_rl=float(base_cfg.get("sign_rl", 1.0)),
            sign_rr=float(base_cfg.get("sign_rr", 1.0)),
        )
        self.odom_state = WheelOdometry(geometry=geometry, frame_id=self.frame_id)

        self.drive_cfg = BodyDriveConfig(
            max_linear=float(esp_cfg.get("max_linear", 0.35)),
            max_angular=float(esp_cfg.get("max_angular", 1.0)),
            deadband_linear=float(esp_cfg.get("deadband_linear", 0.02)),
            deadband_angular=float(esp_cfg.get("deadband_angular", 0.05)),
            max_level=int(esp_cfg.get("max_level", 9)),
            twist_mode=str(esp_cfg.get("twist_mode", "priority")),
            arc_period_sec=float(esp_cfg.get("arc_period_sec", 0.5)),
        )

        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 20)
        self.pub_status = self.create_publisher(String, "/base/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._lock = threading.Lock()
        self._cmd = None
        self._cmd_time = 0.0
        self._estop = False
        self._wheel_speeds = {name: 0.0 for name in self.wheel_names}
        self._wheel_mm_s = [0, 0, 0, 0]
        self._wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self._body_valid = False
        self._body_last_seen = 0.0
        self._body_seq = 0
        self._body_age_ms = -1
        self._body_distance_mm = -1
        self._body_uptime_ms = 0

        self._last_odom_time = time.monotonic()
        self._last_cmd_raw = None
        self._last_cmd_sent = 0.0
        self._next_cmd_attempt = 0.0
        self._last_cmd_error_log = 0.0
        self._last_body_error_log = 0.0
        self._closing = threading.Event()

        self._body_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._cmd_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

        self._steady_clock = create_steady_clock()
        self._body_thread = threading.Thread(target=self._body_loop, daemon=True)
        self._cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self._body_thread.start()
        self._cmd_thread.start()

        self._odom_timer = self.create_timer(
            1.0 / self.odom_rate_hz, self.on_odom_timer, clock=self._steady_clock
        )
        self._status_timer = self.create_timer(
            1.0 / self.status_rate_hz, self.on_status_timer, clock=self._steady_clock
        )

        self.get_logger().info(
            f"esp32_bridge_node started: body={self.body_url}, cmd={self.cmd_url}, "
            f"poll={self.body_poll_hz:.0f}Hz, odom={self.odom_rate_hz:.0f}Hz, "
            f"watchdog={self.cmd_watchdog:.2f}s, twist_mode={self.drive_cfg.twist_mode}"
        )

    # ------------------------------------------------------------- callbacks
    def on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = msg
            self._cmd_time = time.monotonic()

    def on_safety(self, msg: SafetyState) -> None:
        with self._lock:
            self._estop = bool(msg.emergency_stop)

    # ------------------------------------------------------------- HTTP helpers
    def _fetch_body(self) -> dict:
        request = urllib.request.Request(
            self.body_url, headers={"Accept": "application/json"}
        )
        with self._body_opener.open(request, timeout=self.http_timeout) as response:
            payload = response.read().decode("utf-8")
        data = json.loads(payload)
        if isinstance(data, dict) and isinstance(data.get("body"), dict):
            return data["body"]
        return data if isinstance(data, dict) else {}

    def _post_raw_command(self, command: str) -> None:
        message = f"raw:{command}".encode("utf-8")
        request = urllib.request.Request(
            self.cmd_url,
            data=message,
            headers={
                "Content-Type": "text/plain;charset=UTF-8",
                "Accept": "text/plain",
            },
            method="POST",
        )
        with self._cmd_opener.open(request, timeout=self.http_timeout) as response:
            response.read()

    # ------------------------------------------------------------- body polling
    def _body_loop(self) -> None:
        interval = 1.0 / self.body_poll_hz
        while not self._closing.is_set():
            started = time.monotonic()
            try:
                body = self._fetch_body()
                self._apply_body(body)
            except Exception as exc:  # noqa: BLE001
                now = time.monotonic()
                if now - self._last_body_error_log > 3.0:
                    self.get_logger().warn(f"ESP32 body poll failed: {exc}")
                    self._last_body_error_log = now
            elapsed = time.monotonic() - started
            self._closing.wait(max(0.0, interval - elapsed))

    def _apply_body(self, body: dict) -> None:
        if not isinstance(body, dict):
            return
        wheels = body.get("wheel_mm_s")
        valid = bool(body.get("valid", False)) and not bool(body.get("stale", True))
        if not valid or not isinstance(wheels, list) or len(wheels) < 4:
            with self._lock:
                self._body_valid = False
            return

        try:
            wheel_mm_s = [int(round(float(value))) for value in wheels[:4]]
        except (TypeError, ValueError):
            return
        if any(abs(value) > 5000 for value in wheel_mm_s):
            return

        wheel_speeds = {
            name: wheel_mm_s[index] / 1000.0 for index, name in enumerate(self.wheel_names)
        }
        now = time.monotonic()
        with self._lock:
            self._wheel_speeds = wheel_speeds
            self._wheel_mm_s = wheel_mm_s
            self._body_valid = True
            self._body_last_seen = now
            self._body_seq = int(body.get("seq", 0) or 0)
            self._body_age_ms = int(body.get("age_ms", -1) or -1)
            self._body_distance_mm = int(body.get("distance_mm", -1) or -1)
            self._body_uptime_ms = int(body.get("uptime_ms", 0) or 0)

    def _body_fresh(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        with self._lock:
            return bool(
                self._body_valid
                and self._body_last_seen > 0.0
                and (now - self._body_last_seen) <= self.stale_timeout
            )

    # ------------------------------------------------------------- ROS timers
    def on_odom_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            fresh = bool(
                self._body_valid
                and self._body_last_seen > 0.0
                and (now - self._body_last_seen) <= self.stale_timeout
            )
            speeds = (
                dict(self._wheel_speeds)
                if fresh
                else {name: 0.0 for name in self.wheel_names}
            )

        dt = now - self._last_odom_time
        self._last_odom_time = now
        dt = min(0.5, max(0.0, dt))
        if dt > 0.0:
            if fresh:
                self.odom_state.update_from_wheel_speeds(speeds, dt)
            else:
                self.odom_state.update_twist(0.0, 0.0, 0.0, dt)
            for index, name in enumerate(self.wheel_names):
                self._wheel_positions[index] += (
                    speeds[name] * dt / max(1e-6, self.wheel_radius)
                )

        stamp = self.get_clock().now().to_msg()
        pose = self.odom_state.pose
        vx, vy, wz = self.odom_state.twist

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pose.x)
        odom.pose.pose.position.y = float(pose.y)
        odom.pose.pose.orientation.z = float(math.sin(pose.yaw / 2.0))
        odom.pose.pose.orientation.w = float(math.cos(pose.yaw / 2.0))
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 0.02 if index != 35 else 0.05
            odom.twist.covariance[index] = 0.02 if index != 35 else 0.05
        self.pub_odom.publish(odom)

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(self.wheel_joint_names)
        joint.position = list(self._wheel_positions)
        joint.velocity = [float(speeds[name]) for name in self.wheel_names]
        self.pub_joint.publish(joint)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(pose.x)
            transform.transform.translation.y = float(pose.y)
            transform.transform.rotation.z = float(math.sin(pose.yaw / 2.0))
            transform.transform.rotation.w = float(math.cos(pose.yaw / 2.0))
            self.tf_broadcaster.sendTransform(transform)

    def on_status_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            fresh = bool(
                self._body_valid
                and self._body_last_seen > 0.0
                and (now - self._body_last_seen) <= self.stale_timeout
            )
            wheel_mm_s = list(self._wheel_mm_s) if fresh else [0, 0, 0, 0]
            age_ms = (
                int((now - self._body_last_seen) * 1000.0)
                if self._body_last_seen > 0.0
                else -1
            )
            payload = {
                "source": "esp32",
                "seq": int(self._body_seq),
                "age_ms": age_ms,
                "encoder_ok": fresh,
                "estop": bool(self._estop),
                "distance_mm": int(self._body_distance_mm) if fresh else -1,
                "wheel_mm_s": wheel_mm_s,
                "uptime_ms": int(self._body_uptime_ms),
            }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------- command loop
    def _cmd_loop(self) -> None:
        interval = 1.0 / self.cmd_rate_hz
        while not self._closing.is_set():
            started = time.monotonic()
            now = time.monotonic()
            with self._lock:
                cmd = self._cmd
                cmd_time = self._cmd_time
                estop = self._estop
                body_fresh = bool(
                    self._body_valid
                    and self._body_last_seen > 0.0
                    and (now - self._body_last_seen) <= self.stale_timeout
                )

            stale = cmd is None or (now - cmd_time) > self.cmd_watchdog
            if estop or stale or (self.stop_on_body_stale and not body_fresh):
                desired = "0"
            else:
                desired = twist_to_body_command(
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                    self.drive_cfg,
                    now=now,
                )

            should_send = (
                desired != self._last_cmd_raw
                or (now - self._last_cmd_sent) >= self.cmd_resend_sec
            )
            if should_send and now >= self._next_cmd_attempt:
                try:
                    self._post_raw_command(desired)
                except Exception as exc:  # noqa: BLE001
                    self._next_cmd_attempt = now + 0.2
                    if now - self._last_cmd_error_log > 3.0:
                        self.get_logger().warn(f"ESP32 cmd POST failed: {exc}")
                        self._last_cmd_error_log = now
                else:
                    self._last_cmd_raw = desired
                    self._last_cmd_sent = now

            elapsed = time.monotonic() - started
            self._closing.wait(max(0.0, interval - elapsed))

    # ------------------------------------------------------------- lifecycle
    def destroy_node(self) -> None:
        self._closing.set()
        for thread in (self._body_thread, self._cmd_thread):
            if thread.is_alive():
                thread.join(timeout=0.8)
        try:
            self._post_raw_command("0")
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
