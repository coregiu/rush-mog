"""edge_server_node：树莓派上的 Edge Server（网页 + 底盘串口 + ROS 桥）。

合并原 ESP32 Edge Server 与树莓派 ROS 模块后，本节点同时承担：

- HTTP 80：手柄页、数字孪生 dashboard、``/cmd``、``/status``、``/api/*``；
- HTTP 8080：ROS 相机 ``/camera/image_raw`` 的 MJPEG 图传；
- UART：树莓派直连 STM32 USART3（115200 8N1，ASCII 命令 + ``$B`` 遥测）；
- ROS：``/cmd_vel`` -> STM32，STM32 遥测 -> ``/odom``、``/joint_states``、
  ``/base/status``。

和原 ESP32 固件保持一致：
- 网页按钮/摇杆按下时下发一次，松开/停止时再下发一次；默认不周期性重发
  （``edge.motion.resend_manual=true`` 可恢复旧行为）；
- ROS/API ``raw:`` 命令保留 PWM 第二字节，按 ``keepalive_ms`` 刷新并带 1s 租约；
- 网页手动控制优先于 ROS ``/cmd_vel``，避免导航和手柄互相抢方向。
"""

from __future__ import annotations

import json
import math
import os
import platform
import socket
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from typing import Any, Dict, List, Mapping, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from edge_server.core.body_drive import BodyDriveConfig, twist_to_body_command
from edge_server.core.odometry import MecanumGeometry, WheelOdometry, wheel_names
from edge_server.edge_control import (
    BodyTelemetry,
    MotionState,
    dispatch_control_message,
    parse_body_line,
)
from edge_server.edge_web import EdgeHttpApp, EdgeHttpServer, MjpegServer
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.msg import SafetyState

try:  # 相机/图传是可降级能力，缺失时网页仍可用
    import cv2  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
except Exception:  # noqa: BLE001
    cv2 = None
    CvBridge = None


def _local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def _free_memory_bytes() -> int:
    try:
        with open("/proc/meminfo", "r", encoding="ascii") as handle:
            for line in handle:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _int_or(value: Any, default: int) -> int:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return default


class SerialLink:
    """STM32 串口读写线程；断线自动重连。"""

    def __init__(
        self,
        port: str,
        baud: int,
        on_line,
        on_state,
        *,
        reconnect_sec: float = 2.0,
    ) -> None:
        self.port = str(port)
        self.baud = int(baud)
        self.reconnect_sec = max(0.2, float(reconnect_sec))
        self._on_line = on_line
        self._on_state = on_state
        self._serial = None
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._thread = threading.Thread(
            target=self._loop, name="edge-serial", daemon=True
        )
        self._connected = False

    @property
    def connected(self) -> bool:
        with self._lock:
            return bool(self._connected)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._closing.set()
        with self._lock:
            serial_port = self._serial
            self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def write(self, command: str) -> bool:
        text = str(command or "").strip()
        if not text:
            return False
        with self._lock:
            serial_port = self._serial
            if serial_port is None:
                return False
            try:
                serial_port.write((text + "\n").encode("ascii"))
                serial_port.flush()
                return True
            except Exception:  # noqa: BLE001
                return False

    def _loop(self) -> None:
        while not self._closing.is_set():
            try:
                import serial  # type: ignore
            except Exception:  # noqa: BLE001
                self._set_state(False)
                self._closing.wait(2.0)
                continue
            serial_port = None
            try:
                serial_port = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.05,
                    write_timeout=0.05,
                )
                with self._lock:
                    self._serial = serial_port
                self._set_state(True)
                while not self._closing.is_set():
                    line = serial_port.readline()
                    if not line:
                        continue
                    text = line.decode("utf-8", "replace").strip()
                    if text:
                        try:
                            self._on_line(text)
                        except Exception:  # noqa: BLE001
                            pass
            except Exception:  # noqa: BLE001
                self._set_state(False)
                self._closing.wait(self.reconnect_sec)
            finally:
                with self._lock:
                    if self._serial is serial_port:
                        self._serial = None
                if serial_port is not None:
                    try:
                        serial_port.close()
                    except Exception:  # noqa: BLE001
                        pass

    def _set_state(self, connected: bool) -> None:
        with self._lock:
            if self._connected == connected:
                return
            self._connected = connected
        try:
            self._on_state(connected)
        except Exception:  # noqa: BLE001
            pass


class EdgeServerNode(Node):
    def __init__(self) -> None:
        super().__init__("edge_server_node")
        self.cfg = load_config(self)
        base_cfg = dict(self.cfg.get("base") or {})
        edge_cfg = dict(self.cfg.get("edge") or {})
        web_cfg = dict(edge_cfg.get("web") or {})
        camera_cfg = dict(edge_cfg.get("camera") or {})
        motion_cfg = dict(edge_cfg.get("motion") or {})

        self.started_monotonic = time.monotonic()
        self.hostname = socket.gethostname()
        self.ip_address = _local_ip()
        self.platform_name = platform.platform()
        self._state_lock = threading.Lock()
        self._odom_lock = threading.Lock()
        self._health_lock = threading.Lock()
        self._jpeg_lock = threading.Lock()
        self._params_lock = threading.Lock()

        self.telemetry = BodyTelemetry()
        self._last_uptime_ms: Optional[int] = None
        self._telemetry_mono = 0.0
        self._serial_connected = False
        self._estop = False
        self._last_command = ""
        self._last_command_at = 0.0
        self._last_ros_command = ""
        self._ros_command_at = 0.0
        self._health_cache: Optional[dict] = None
        self._health_cache_at = 0.0
        self._jpeg_seq = 0
        self._jpeg_bytes: Optional[bytes] = None
        self._jpeg_at = 0.0
        self._jpeg_warned = False

        state_file = str(
            edge_cfg.get("state_file", "~/.config/rush-mog/edge_params.json")
        )
        self._state_file = os.path.expanduser(state_file)
        saved = self._load_state()
        self._stream_fps = max(
            1,
            min(
                30,
                _int_or(
                    saved.get("stream_fps"),
                    _int_or(camera_cfg.get("fps"), 10),
                ),
            ),
        )
        keepalive_ms = _int_or(
            saved.get("motion_keepalive_ms"),
            _int_or(motion_cfg.get("keepalive_ms"), 200),
        )
        self.motion = MotionState(
            keepalive_ms=keepalive_ms,
            raw_lease_sec=float(motion_cfg.get("raw_lease_sec", 1.0)),
            resend_manual=bool(motion_cfg.get("resend_manual", False)),
        )
        # 运行事件环形缓冲：最新在前，最多 120 条；服务重启后清空。
        self._events = deque(maxlen=120)
        self._events_lock = threading.Lock()
        self._event_seq = 0
        self._event_last_at = 0.0
        self._event_last_key = ""
        self._last_ros_event_command = ""

        self.odom_frame = str(base_cfg.get("odom_frame", "odom"))
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
        self.wheel_radius = float(base_cfg.get("wheel_radius", 0.048))
        self.wheel_names = wheel_names()
        self.geometry = MecanumGeometry(
            wheel_radius=self.wheel_radius,
            wheelbase_x=float(base_cfg.get("wheelbase_x", 0.20)),
            track_y=float(base_cfg.get("track_y", 0.18)),
            encoder_cpr=float(base_cfg.get("encoder_cpr", 1320.0)),
            sign_fl=float(base_cfg.get("sign_fl", 1.0)),
            sign_fr=float(base_cfg.get("sign_fr", 1.0)),
            sign_rl=float(base_cfg.get("sign_rl", 1.0)),
            sign_rr=float(base_cfg.get("sign_rr", 1.0)),
        )
        self.odom_state = WheelOdometry(geometry=self.geometry, frame_id=self.odom_frame)
        self._wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self._last_odom_mono = time.monotonic()
        self.publish_tf = bool(base_cfg.get("publish_tf", False))

        body_cfg = dict(base_cfg.get("body_command") or {})
        self.drive_cfg = BodyDriveConfig(
            max_linear=float(body_cfg.get("max_linear", 0.35)),
            max_angular=float(body_cfg.get("max_angular", 1.0)),
            deadband_linear=float(body_cfg.get("deadband_linear", 0.02)),
            deadband_angular=float(body_cfg.get("deadband_angular", 0.05)),
            max_level=int(body_cfg.get("max_level", 9)),
            twist_mode=str(body_cfg.get("twist_mode", "priority")),
            arc_period_sec=float(body_cfg.get("arc_period_sec", 0.5)),
        )
        self.cmd_watchdog = max(0.1, float(base_cfg.get("watchdog_sec", 0.5)))

        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 20)
        self.pub_status = self.create_publisher(String, "/base/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # 串口：base.enabled + edge_serial/serial 时启动；仿真可只跑网页。
        self.serial_enabled = bool(base_cfg.get("enabled", True))
        self.serial = SerialLink(
            str(base_cfg.get("port", "/dev/ttyUSB0")),
            int(base_cfg.get("baud", 115200)),
            self._on_serial_line,
            self._on_serial_state,
            reconnect_sec=float((base_cfg.get("edge_serial") or {}).get("reconnect_sec", 2.0)),
        )
        self._record_event("system", "Edge Server 启动", source="system", status="ok")
        if self.serial_enabled:
            self.serial.start()

        # 网页/API
        self._web_root = self._resolve_web_root(web_cfg)
        self._sdcard_root = os.path.expanduser(
            str(web_cfg.get("sdcard_dir", "") or "")
        )
        self.app = EdgeHttpApp(
            self,
            static_root=self._web_root,
            sdcard_root=self._sdcard_root,
            stream_port=int(web_cfg.get("stream_port", 8080)),
        )
        self.http = EdgeHttpServer(
            str(web_cfg.get("host", "0.0.0.0")),
            int(web_cfg.get("port", 80)),
            self.app,
        )
        self.http.start()

        # MJPEG：来自 ROS 相机；无 cv_bridge/相机时 503
        self.camera_enabled = bool(camera_cfg.get("enabled", True))
        self.camera_topic = str(camera_cfg.get("topic", "/camera/image_raw"))
        self.jpeg_quality = max(10, min(95, _int_or(camera_cfg.get("quality"), 70)))
        self.mjpeg = MjpegServer(
            str(web_cfg.get("host", "0.0.0.0")),
            int(web_cfg.get("stream_port", 8080)),
            self.camera_snapshot,
            lambda: self.stream_fps,
            self.camera_available,
        )
        self.mjpeg.start()
        self._camera_sub = None
        self._cv_bridge = None
        if self.camera_enabled:
            if cv2 is None or CvBridge is None:
                self.get_logger().warn(
                    "cv_bridge/opencv 不可用，/stream 将返回 503；网页/控制不受影响"
                )
            else:
                self._cv_bridge = CvBridge()
                self._camera_sub = self.create_subscription(
                    Image,
                    self.camera_topic,
                    self.on_image,
                    qos_profile_sensor_data,
                )
                self.get_logger().info(f"camera stream topic: {self.camera_topic}")

        self._steady_clock = create_steady_clock()
        self._odom_timer = self.create_timer(
            1.0 / max(5.0, float(base_cfg.get("odom_rate_hz", 20.0))),
            self.on_odom_timer,
            clock=self._steady_clock,
        )
        self._status_timer = self.create_timer(
            1.0,
            self.on_status_timer,
            clock=self._steady_clock,
        )
        self._keepalive_timer = self.create_timer(
            0.1,
            self.on_keepalive_timer,
            clock=self._steady_clock,
        )

        self.get_logger().info(
            f"edge_server_node started: web={self.http.info()}, "
            f"mjpeg={self.mjpeg.info()}, serial={self.serial.port}@{self.serial.baud}"
        )
        self.get_logger().info(
            f"edge state: ip={self.ip_address}, web_root={self._web_root}, "
            f"stream_fps={self._stream_fps}"
        )
        if self.serial_enabled:
            self.serial.write("0")

    # ------------------------------------------------------------- config/state
    def _resolve_web_root(self, web_cfg: Mapping[str, Any]) -> str:
        configured = str(web_cfg.get("root", "") or "").strip()
        if configured:
            return os.path.expanduser(configured)
        try:
            from ament_index_python.packages import get_package_share_directory

            return os.path.join(get_package_share_directory("edge_server"), "web")
        except Exception:  # noqa: BLE001
            return os.path.join(os.path.dirname(__file__), "..", "web")

    def _load_state(self) -> dict:
        try:
            with open(self._state_file, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            return payload if isinstance(payload, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_state(self) -> None:
        payload = {
            "motion_keepalive_ms": self.motion.keepalive_ms,
            "stream_fps": self.stream_fps,
        }
        try:
            directory = os.path.dirname(self._state_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            temporary = self._state_file + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(temporary, self._state_file)
        except OSError as exc:
            self.get_logger().warn(f"save edge params failed: {exc}")

    @property
    def stream_fps(self) -> int:
        with self._params_lock:
            return int(self._stream_fps)

    # ------------------------------------------------------------- events
    def _record_event(
        self,
        kind: str,
        text: str,
        *,
        command: str = "",
        source: str = "",
        status: str = "info",
    ) -> None:
        """写入运行事件环形缓冲（最新在前）。"""
        with self._events_lock:
            self._event_seq += 1
            self._events.appendleft(
                {
                    "id": int(self._event_seq),
                    "ts_ms": int(time.time() * 1000.0),
                    "kind": str(kind)[:16],
                    "text": str(text)[:160],
                    "command": str(command)[:8],
                    "source": str(source)[:16],
                    "status": str(status)[:16],
                }
            )

    def _record_command_event(
        self, command: str, source: str, *, delivered: bool = True
    ) -> None:
        text = str(command or "").strip()
        if not text:
            return
        now = time.monotonic()
        key = f"{source}:{text}:{1 if delivered else 0}"
        if key == self._event_last_key and (now - self._event_last_at) < 0.8:
            return
        # 摇杆/ROS 高频连续指令限流；停车命令始终记录。
        if text != "0" and (now - self._event_last_at) < 0.2:
            return
        self._event_last_key = key
        self._event_last_at = now
        self._record_event(
            "command",
            "指令已下发" if delivered else "指令已接受，串口未连接",
            command=text,
            source=source,
            status="ok" if delivered else "warn",
        )

    def motion_snapshot(self) -> dict:
        snap = self.motion.snapshot()
        return {
            "active": bool(snap["active"]),
            "command": str(snap["command"]),
            "manual": bool(snap["manual"]),
            "source": str(snap["source"]),
            "age_ms": int(snap["age_ms"]),
            "keepalive_ms": int(snap["keepalive_ms"]),
        }

    def events(self, limit: int = 40) -> dict:
        try:
            count = int(limit)
        except (TypeError, ValueError):
            count = 40
        count = max(1, min(120, count))
        with self._events_lock:
            items = list(self._events)[:count]
            total = len(self._events)
        return {
            "ok": True,
            "count": total,
            "events": items,
            "server_time_ms": int(time.time() * 1000.0),
        }

    # ------------------------------------------------------------- serial
    def _on_serial_state(self, connected: bool) -> None:
        self._serial_connected = bool(connected)
        if connected:
            self.get_logger().info(
                f"STM32 serial connected: {self.serial.port}@{self.serial.baud}"
            )
            self._record_event(
                "serial",
                f"STM32 串口已连接 {self.serial.port}@{self.serial.baud}",
                source="serial",
                status="ok",
            )
            self.serial.write("0")
        else:
            self.get_logger().warn("STM32 serial disconnected")
            self._record_event(
                "serial",
                f"STM32 串口断开 {self.serial.port}",
                source="serial",
                status="error",
            )

    def _on_serial_line(self, line: str) -> None:
        telemetry = parse_body_line(line)
        if telemetry is None:
            return
        now = time.monotonic()
        with self._state_lock:
            previous_uptime = self._last_uptime_ms
            self.telemetry = telemetry
            self._last_uptime_ms = telemetry.uptime_ms
            self._telemetry_mono = now
        dt = 0.0
        if previous_uptime is not None:
            delta_ms = int(telemetry.uptime_ms) - int(previous_uptime)
            if 0 < delta_ms < 1000:
                dt = delta_ms / 1000.0
        if dt <= 0.0:
            dt = min(0.5, max(0.0, now - self._last_odom_mono))
        self._update_odom(telemetry, now, dt)

    def _update_odom(self, telemetry: BodyTelemetry, now: float, dt: float) -> None:
        speeds = {
            name: telemetry.wheel_mm_s[index] / 1000.0
            for index, name in enumerate(self.wheel_names)
        }
        with self._odom_lock:
            if dt > 0.0:
                self.odom_state.update_from_wheel_speeds(speeds, dt)
                for index, name in enumerate(self.wheel_names):
                    self._wheel_positions[index] += (
                        speeds[name] * dt / max(1e-6, self.wheel_radius)
                    )
            self._last_odom_mono = now

    def _send_command(self, command: str, *, force: bool = False) -> bool:
        text = str(command or "").strip()
        if not text:
            return False
        now = time.monotonic()
        if (
            not force
            and text == self._last_command
            and (now - self._last_command_at) < 0.15
        ):
            return False
        self._last_command = text
        self._last_command_at = now
        return self.serial.write(text)

    # ------------------------------------------------------------- ROS callbacks
    def on_cmd_vel(self, msg: Twist) -> None:
        now = time.monotonic()
        with self._state_lock:
            estop = self._estop
        if estop:
            return
        command = twist_to_body_command(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            self.drive_cfg,
            now=now,
        )
        if command == "0" and (now - self._ros_command_at) > self.cmd_watchdog:
            return
        accepted = self.motion.apply_raw(command, now, source="ros")
        if not accepted:
            return
        self._ros_command_at = now
        self._last_ros_command = command
        self._send_command(command)
        if command != self._last_ros_event_command:
            self._last_ros_event_command = command
            self._record_command_event(
                command, "ros", delivered=bool(self._serial_connected)
            )

    def on_safety(self, msg: SafetyState) -> None:
        with self._state_lock:
            previous = self._estop
            self._estop = bool(msg.emergency_stop)
        if self._estop != previous:
            if self._estop:
                self._record_event(
                    "safety", "急停触发", source="safety", status="error"
                )
            else:
                self._record_event(
                    "safety", "急停解除", source="safety", status="ok"
                )
        if self._estop:
            self.motion.clear()
            self._send_command("0", force=True)

    # ------------------------------------------------------------- timers
    def on_odom_timer(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            fresh = bool(
                self.telemetry.valid
                and self._telemetry_mono > 0.0
                and (now - self._telemetry_mono) <= 2.0
            )
        with self._odom_lock:
            if not fresh:
                self.odom_state.update_twist(0.0, 0.0, 0.0, 0.0)
            pose = self.odom_state.pose
            vx, vy, wz = self.odom_state.twist
            positions = list(self._wheel_positions)
        with self._state_lock:
            wheel_mm_s = [int(value) for value in self.telemetry.wheel_mm_s]
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
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
        joint.position = positions
        joint.velocity = [
            float(wheel_mm_s[i] / 1000.0) if fresh else 0.0 for i in range(4)
        ]
        self.pub_joint.publish(joint)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(pose.x)
            transform.transform.translation.y = float(pose.y)
            transform.transform.rotation.z = float(math.sin(pose.yaw / 2.0))
            transform.transform.rotation.w = float(math.cos(pose.yaw / 2.0))
            self.tf_broadcaster.sendTransform(transform)

    def on_status_timer(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            telemetry = self.telemetry
            fresh = bool(
                telemetry.valid
                and self._telemetry_mono > 0.0
                and (now - self._telemetry_mono) <= 2.0
            )
            age_ms = (
                int(max(0.0, now - self._telemetry_mono) * 1000.0)
                if self._telemetry_mono > 0.0
                else -1
            )
            payload = {
                "source": "pi",
                "seq": int(telemetry.seq),
                "age_ms": age_ms,
                "encoder_ok": fresh,
                "estop": bool(self._estop),
                "distance_mm": int(telemetry.distance_mm) if fresh else -1,
                "wheel_mm_s": (
                    [int(value) for value in telemetry.wheel_mm_s]
                    if fresh
                    else [0, 0, 0, 0]
                ),
                "uptime_ms": int(telemetry.uptime_ms),
                "serial_connected": bool(self._serial_connected),
            }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def on_keepalive_timer(self) -> None:
        command = self.motion.tick()
        if command:
            self._send_command(command, force=True)
            if command == "0":
                self._record_event(
                    "safety",
                    "指令租约超时，自动停车",
                    command="0",
                    source="system",
                    status="warn",
                )

    # ------------------------------------------------------------- camera
    def on_image(self, msg: Image) -> None:
        if cv2 is None or self._cv_bridge is None:
            return
        now = time.monotonic()
        if (now - self._jpeg_at) < (1.0 / max(1, self.stream_fps)):
            return
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                return
            data = encoded.tobytes()
        except Exception as exc:  # noqa: BLE001
            if not self._jpeg_warned:
                self.get_logger().warn(f"camera frame encode failed: {exc}")
                self._jpeg_warned = True
            return
        with self._jpeg_lock:
            self._jpeg_seq += 1
            self._jpeg_bytes = data
            self._jpeg_at = now
        self._jpeg_warned = False

    def camera_snapshot(self) -> Tuple[int, Optional[bytes]]:
        with self._jpeg_lock:
            return self._jpeg_seq, self._jpeg_bytes

    def camera_available(self) -> bool:
        with self._jpeg_lock:
            return bool(
                self._jpeg_bytes
                and self._jpeg_at > 0.0
                and (time.monotonic() - self._jpeg_at) <= 2.0
            )

    # ------------------------------------------------------------- runtime API
    def status(self) -> dict:
        now = time.monotonic()
        with self._state_lock:
            fresh = bool(
                self.telemetry.valid
                and self._telemetry_mono > 0.0
                and (now - self._telemetry_mono) <= 2.0
            )
        health = self._ros_health()
        return {
            "uptime_ms": int((now - self.started_monotonic) * 1000.0),
            "free_heap": int(_free_memory_bytes()),
            "free_psram": 0,
            "camera": 1 if self.camera_available() else 0,
            "sdcard": 1 if (self._sdcard_root and os.path.isdir(self._sdcard_root)) else 0,
            "motion_active": 1 if self.motion.snapshot(now)["active"] else 0,
            "motion": self.motion_snapshot(),
            "serial_connected": bool(self._serial_connected),
            "encoder_ok": bool(fresh),
            "ros_status": str((health or {}).get("status", "unknown")),
            "ros_ready": bool((health or {}).get("ready", False)),
            "ip": self.ip_address,
            "hostname": self.hostname,
        }

    def body(self) -> dict:
        with self._state_lock:
            telemetry = self.telemetry
        return telemetry.to_dict()

    def _ros_health(self) -> Optional[dict]:
        now = time.monotonic()
        with self._health_lock:
            if self._health_cache is not None and (now - self._health_cache_at) < 1.5:
                return self._health_cache
        url = str(
            (self.cfg.get("edge") or {}).get(
                "ros_health_url", "http://127.0.0.1:8090/ros/health"
            )
        )
        parsed = None
        try:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(url, timeout=0.8) as response:
                payload = json.loads(response.read().decode("utf-8"))
            if isinstance(payload, dict):
                parsed = payload
        except Exception:  # noqa: BLE001
            parsed = None
        with self._health_lock:
            self._health_cache = parsed
            self._health_cache_at = now
        return parsed

    def local_modules(self) -> List[dict]:
        health = self._ros_health()
        now = time.monotonic()
        if health is None:
            module = {
                "id": "ros",
                "name": "ROS 2 (Edge)",
                "version": "",
                "status": "unknown",
                "summary": "ROS health 网关未响应（Edge 内 ROS 子系统）",
                "health_url": str(
                    (self.cfg.get("edge") or {}).get(
                        "ros_health_url", "http://127.0.0.1:8090/ros/health"
                    )
                ),
                "params_url": str(
                    (self.cfg.get("edge") or {}).get(
                        "ros_params_url", "http://127.0.0.1:8090/ros/params"
                    )
                ),
                "updated_ms": int(now * 1000.0),
                "age_ms": 0,
                "stale": True,
                "params": {},
            }
            return [module]
        summary = health.get("summary") or {}
        return [
            {
                "id": "ros",
                "name": "ROS 2 (Edge)",
                "version": str(health.get("version", "")),
                "status": str(health.get("status", "unknown")),
                "summary": str(health.get("reason", "")),
                "health_url": str(
                    (self.cfg.get("edge") or {}).get(
                        "ros_health_url", "http://127.0.0.1:8090/ros/health"
                    )
                ),
                "params_url": str(
                    (self.cfg.get("edge") or {}).get(
                        "ros_params_url", "http://127.0.0.1:8090/ros/params"
                    )
                ),
                "updated_ms": int(now * 1000.0),
                "age_ms": 0,
                "stale": False,
                "params": {
                    "devices_ok": int(summary.get("ok", 0)),
                    "devices_total": int(summary.get("total", 0)),
                    "ready": bool(health.get("ready", False)),
                },
            }
        ]

    def _edge_params_object(self) -> dict:
        now = time.monotonic()
        motion = self.motion.snapshot(now)
        health = self._ros_health() or {}
        return {
            "motion_keepalive_ms": int(motion["keepalive_ms"]),
            "stream_fps": self.stream_fps,
            "free_heap": int(_free_memory_bytes()),
            "free_psram": 0,
            "uptime_ms": int((now - self.started_monotonic) * 1000.0),
            "wifi_clients": 0,
            "camera": self.camera_available(),
            "sdcard": bool(self._sdcard_root and os.path.isdir(self._sdcard_root)),
            "motion_active": bool(motion["active"]),
            "ap_ssid": "",
            "ap_ip": self.ip_address,
            "ap_channel": 0,
            "max_clients": 0,
            "serial_connected": bool(self._serial_connected),
            "serial_port": self.serial.port,
            "serial_baud": self.serial.baud,
            "ros_status": str(health.get("status", "unknown")),
            "ros_ready": bool(health.get("ready", False)),
            "hostname": self.hostname,
            "ip": self.ip_address,
            "platform": self.platform_name,
        }

    def edge_module(self) -> dict:
        params = self._edge_params_object()
        health = self._ros_health()
        ros_status = str((health or {}).get("status", "unknown"))
        serial_ok = bool(self._serial_connected)
        if health is None:
            status = "degraded"
        elif not serial_ok:
            status = "degraded"
        elif ros_status == "ok":
            status = "ok"
        elif ros_status in {"degraded", "starting", "unknown"}:
            status = "degraded"
        else:
            status = "error"
        summary = (
            f"serial={'on' if serial_ok else 'off'}, "
            f"ros={ros_status}, "
            f"camera={'on' if params['camera'] else 'off'}, "
            f"base={'on' if params['motion_active'] else 'idle'}"
        )
        return {
            "id": "edge",
            "name": "Edge Server",
            "version": "0.3.0",
            "ip": self.ip_address,
            "status": status,
            "summary": summary,
            "health_url": f"http://{self.ip_address}/status",
            "params_url": f"http://{self.ip_address}/api/modules/edge/params",
            "updated_ms": params["uptime_ms"],
            "age_ms": 0,
            "stale": False,
            "params": params,
            "body": self.body(),
        }

    def edge_params(self) -> dict:
        values = self._edge_params_object()
        params = [
            _param("motion_keepalive_ms", "运动保活周期", values["motion_keepalive_ms"],
                   "int", "运动控制", True, "ms", "100", "600", "10"),
            _param("stream_fps", "视频流帧率", values["stream_fps"],
                   "int", "视频", True, "fps", "5", "20", "1"),
            _param("hostname", "主机名", json.dumps(values["hostname"]),
                   "string", "运行状态", False, "", "", "", ""),
            _param("ip", "IP 地址", json.dumps(values["ip"]),
                   "string", "网络", False, "", "", "", ""),
            _param("platform", "系统平台", json.dumps(values["platform"]),
                   "string", "运行状态", False, "", "", "", ""),
            _param("serial_port", "STM32 串口", json.dumps(values["serial_port"]),
                   "string", "底盘", False, "", "", "", ""),
            _param("serial_baud", "串口波特率", values["serial_baud"],
                   "int", "底盘", False, "bps", "", "", ""),
            _param("serial_connected", "串口连接", _bool_literal(values["serial_connected"]),
                   "bool", "底盘", False, "", "", "", ""),
            _param("ros_status", "ROS 状态", json.dumps(values["ros_status"]),
                   "string", "ROS", False, "", "", "", ""),
            _param("ros_ready", "ROS 就绪", _bool_literal(values["ros_ready"]),
                   "bool", "ROS", False, "", "", "", ""),
            _param("free_heap", "可用内存", values["free_heap"],
                   "int", "运行状态", False, "bytes", "", "", ""),
            _param("uptime_ms", "运行时长", values["uptime_ms"],
                   "int", "运行状态", False, "ms", "", "", ""),
            _param("camera", "摄像头", _bool_literal(values["camera"]),
                   "bool", "运行状态", False, "", "", "", ""),
            _param("sdcard", "SD 卡目录", _bool_literal(values["sdcard"]),
                   "bool", "运行状态", False, "", "", "", ""),
            _param("motion_active", "运动保活中", _bool_literal(values["motion_active"]),
                   "bool", "运行状态", False, "", "", "", ""),
        ]
        return {
            "ok": True,
            "module": "edge",
            "name": "Edge Server",
            "version": "0.3.0",
            "editable": True,
            "requires_restart": False,
            "params": params,
        }

    def update_params(self, updates: Mapping[str, Any]) -> dict:
        changed: Dict[str, Any] = {}
        for key, raw in updates.items():
            name = str(key).strip()
            if name == "motion_keepalive_ms":
                value = _int_or(raw, -1)
                if value < 100 or value > 600:
                    raise ValueError("motion_keepalive_ms 必须在 100..600")
                self.motion.keepalive_ms = value
                changed[name] = value
            elif name == "stream_fps":
                value = _int_or(raw, -1)
                if value < 5 or value > 20:
                    raise ValueError("stream_fps 必须在 5..20")
                with self._params_lock:
                    self._stream_fps = value
                changed[name] = value
            else:
                raise ValueError(f"参数不可修改: {name}")
        if not changed:
            raise ValueError("没有可更新的参数")
        self._save_state()
        return {
            "ok": True,
            "updated": changed,
            "message": "边缘参数已保存",
        }

    def handle_command(self, message: str) -> str:
        now = time.monotonic()
        with self._state_lock:
            estop = self._estop
        if estop:
            return ""
        command = dispatch_control_message(message, self.motion, now)
        if command:
            delivered = self._send_command(command, force=True)
            source = "api" if message.startswith("raw:") else "web"
            self._record_command_event(command, source, delivered=delivered)
        return command

    # ------------------------------------------------------------- lifecycle
    def destroy_node(self) -> None:
        try:
            self._send_command("0", force=True)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.mjpeg.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.http.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.serial.stop()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def _bool_literal(value: Any) -> str:
    return "true" if bool(value) else "false"


def _param(key, label, value_literal, value_type, group, editable, unit, minimum, maximum, step) -> dict:
    item = {
        "key": key,
        "label": label,
        "value": value_literal,
        "type": value_type,
        "group": group,
        "editable": bool(editable),
    }
    if unit:
        item["unit"] = unit
    if minimum != "":
        item["min"] = int(minimum)
    if maximum != "":
        item["max"] = int(maximum)
    if step != "":
        item["step"] = int(step)
    return item


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EdgeServerNode()
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
