"""lidar_node：D6/LD19（LD06 兼容）2D 激光雷达串口驱动。

- UART4 `/dev/rush_lidar`（或 `/dev/ttyAMA*`），默认 230400 8N1；
- 解析 47 字节 LD19 帧，累积为固定 bin 的 360° `LaserScan`；
- 发布 `/scan`（默认 frame_id `laser_link`），可选发布 `base_link -> laser_link` 静态 TF；
- 串口断开自动重连；1 Hz 发布 `/lidar/status` JSON 便于排障。

协议解析与分箱在 `edge_server/core/lidar.py`，可纯逻辑单测。
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from edge_server.core.lidar import (
    PACKET_HEADER,
    PACKET_LENGTH,
    PACKET_SIZE,
    ScanAccumulator,
    parse_packet,
)
from edge_server.utils.config import load_config


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_node")
        self.cfg = load_config(self)
        lidar_cfg = dict(self.cfg.get("lidar") or {})
        self.enabled = bool(lidar_cfg.get("enabled", True))
        self.port = str(lidar_cfg.get("port", "/dev/rush_lidar") or "/dev/rush_lidar")
        self.baud = int(lidar_cfg.get("baud", 230400))
        self.topic = str(lidar_cfg.get("topic", "/scan") or "/scan")
        self.status_topic = str(
            lidar_cfg.get("status_topic", "/lidar/status") or "/lidar/status"
        )
        self.frame_id = str(lidar_cfg.get("frame_id", "laser_link") or "laser_link")
        self.protocol = str(lidar_cfg.get("protocol", "ld19") or "ld19").lower()
        self.range_min = float(lidar_cfg.get("range_min", 0.02))
        self.range_max = float(lidar_cfg.get("range_max", 12.0))
        self.bins = int(lidar_cfg.get("bins", 360))
        self.angle_min_deg = float(lidar_cfg.get("angle_min_deg", -180.0))
        self.angle_max_deg = float(lidar_cfg.get("angle_max_deg", 180.0))
        self.min_points = int(lidar_cfg.get("min_points", 30))
        self.reconnect_sec = max(0.2, float(lidar_cfg.get("reconnect_sec", 2.0)))
        self.status_rate_hz = max(
            0.1, float(lidar_cfg.get("status_rate_hz", 1.0))
        )

        self.publish_tf = bool(lidar_cfg.get("publish_tf", True))
        self.tf_parent = str(lidar_cfg.get("tf_parent", "base_link") or "base_link")
        self.tf_x = float(lidar_cfg.get("tf_x", 0.0))
        self.tf_y = float(lidar_cfg.get("tf_y", 0.0))
        self.tf_z = float(lidar_cfg.get("tf_z", 0.18))
        self.tf_yaw_deg = float(lidar_cfg.get("tf_yaw_deg", 0.0))

        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial = None
        self._connected = False
        self._state = "disabled" if not self.enabled else "starting"
        self._packets = 0
        self._scans = 0
        self._crc_errors = 0
        self._header_errors = 0
        self._last_error = ""
        self._last_error_log_at = 0.0
        self._last_packet_at = 0.0
        self._last_scan_at = 0.0
        self._last_scan_points = 0
        self._last_speed_dps = 0
        self._started_monotonic = time.monotonic()

        self.accumulator = ScanAccumulator(
            bins=self.bins,
            angle_min_deg=self.angle_min_deg,
            angle_max_deg=self.angle_max_deg,
            range_min_m=self.range_min,
            range_max_m=self.range_max,
            min_points=self.min_points,
        )
        self.pub_scan = self.create_publisher(LaserScan, self.topic, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.create_timer(1.0 / self.status_rate_hz, self.on_status_timer)

        if self.publish_tf:
            self._publish_static_tf()
        if self.protocol not in {"ld19", "ld06", "d6"}:
            self._set_error(f"不支持的协议: {self.protocol}")
            self.enabled = False
        if self.enabled:
            self._thread = threading.Thread(
                target=self._read_loop, name="lidar-serial", daemon=True
            )
            self._thread.start()
        self.get_logger().info(
            f"lidar_node started: enabled={self.enabled}, port={self.port}, "
            f"baud={self.baud}, topic={self.topic}, frame={self.frame_id}"
        )

    # ------------------------------------------------------------- TF/status
    def _publish_static_tf(self) -> None:
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.tf_parent
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = self.tf_x
        transform.transform.translation.y = self.tf_y
        transform.transform.translation.z = self.tf_z
        yaw = math.radians(self.tf_yaw_deg)
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        broadcaster.sendTransform(transform)
        self._tf_broadcaster = broadcaster

    def on_status_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            payload = {
                "ok": True,
                "enabled": bool(self.enabled),
                "state": str(self._state),
                "connected": bool(self._connected),
                "port": self.port,
                "baud": int(self.baud),
                "protocol": self.protocol,
                "topic": self.topic,
                "frame_id": self.frame_id,
                "packets": int(self._packets),
                "scans": int(self._scans),
                "crc_errors": int(self._crc_errors),
                "header_errors": int(self._header_errors),
                "last_scan_points": int(self._last_scan_points),
                "last_speed_dps": int(self._last_speed_dps),
                "packet_age_ms": (
                    int(max(0.0, now - self._last_packet_at) * 1000.0)
                    if self._last_packet_at > 0
                    else -1
                ),
                "scan_age_ms": (
                    int(max(0.0, now - self._last_scan_at) * 1000.0)
                    if self._last_scan_at > 0
                    else -1
                ),
                "last_error": str(self._last_error),
                "uptime_ms": int((now - self._started_monotonic) * 1000.0),
            }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------- serial loop
    def _read_loop(self) -> None:
        while not self._closing.is_set():
            try:
                import serial  # type: ignore
            except Exception as exc:  # noqa: BLE001
                self._set_error(f"pyserial 不可用: {exc}")
                self._closing.wait(self.reconnect_sec)
                continue
            serial_port = None
            try:
                self._set_state("connecting")
                serial_port = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.05,
                    write_timeout=0.05,
                )
                with self._lock:
                    self._serial = serial_port
                    self._connected = True
                    self._state = "receiving"
                    self._last_error = ""
                self.get_logger().info(f"lidar serial connected: {self.port}@{self.baud}")
                self._read_serial(serial_port)
            except Exception as exc:  # noqa: BLE001
                self._set_error(f"{type(exc).__name__}: {exc}")
                self._closing.wait(self.reconnect_sec)
            finally:
                with self._lock:
                    if self._serial is serial_port:
                        self._serial = None
                    self._connected = False
                if serial_port is not None:
                    try:
                        serial_port.close()
                    except Exception:  # noqa: BLE001
                        pass
        self._set_state("disabled")

    def _read_serial(self, serial_port) -> None:
        buffer = bytearray()
        while not self._closing.is_set():
            chunk = serial_port.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)
            while len(buffer) >= PACKET_SIZE:
                index = buffer.find(bytes([PACKET_HEADER]))
                if index < 0:
                    buffer.clear()
                    self._count_header_error()
                    break
                if index > 0:
                    del buffer[:index]
                if len(buffer) < PACKET_SIZE:
                    break
                if buffer[1] != PACKET_LENGTH:
                    self._count_header_error()
                    del buffer[0]
                    continue
                packet = parse_packet(bytes(buffer[:PACKET_SIZE]))
                if packet is None:
                    with self._lock:
                        self._crc_errors += 1
                    del buffer[0]
                    continue
                del buffer[:PACKET_SIZE]
                with self._lock:
                    self._packets += 1
                    self._last_packet_at = time.monotonic()
                    self._last_speed_dps = int(packet.speed_dps)
                    self._state = "receiving"
                scan = self.accumulator.add_packet(packet)
                if scan is not None:
                    self._publish_scan(scan)

    def _publish_scan(self, scan) -> None:
        now_mono = time.monotonic()
        with self._lock:
            previous = self._last_scan_at
        scan_time = 0.1
        if previous > 0.0:
            scan_time = max(0.02, min(0.5, now_mono - previous))
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.angle_min = float(scan.angle_min)
        message.angle_max = float(scan.angle_max)
        message.angle_increment = float(scan.angle_increment)
        message.time_increment = float(scan_time / max(1, self.bins))
        message.scan_time = float(scan_time)
        message.range_min = float(self.range_min)
        message.range_max = float(self.range_max)
        message.ranges = list(scan.ranges)
        message.intensities = list(scan.intensities)
        self.pub_scan.publish(message)
        with self._lock:
            self._scans += 1
            self._last_scan_at = now_mono
            self._last_scan_points = int(scan.points)

    # ------------------------------------------------------------- state
    def _set_state(self, state: str) -> None:
        with self._lock:
            self._state = str(state)

    def _set_error(self, message: str) -> None:
        text = str(message)[:240]
        now = time.monotonic()
        with self._lock:
            changed = text != self._last_error
            self._state = "error"
            self._last_error = text
            should_log = changed or (now - self._last_error_log_at) > 10.0
            if should_log:
                self._last_error_log_at = now
        if should_log:
            self.get_logger().warn(f"lidar: {text}")

    def _count_header_error(self) -> None:
        with self._lock:
            self._header_errors += 1

    def destroy_node(self) -> None:
        self._closing.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        serial_port = self._serial
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
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
