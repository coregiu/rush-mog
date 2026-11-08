"""base_bridge_node：树莓派 <-> STM32 底盘串口桥 + 轮式里程计 + IMU 转发。

- 订阅 /cmd_vel（velocity_mux 的最终输出），按 base_protocol 二进制帧下发；
- 读取 STM32 回传的编码器/电池遥测，发布 /odom、/joint_states、/battery_state；
- 读取 STM32 回传的 ICM-20948 数据，发布 /imu/data；
- 0.5s 未收到新速度命令自动回零，急停时置 FLAG_ESTOP；
- 串口断开后每 2s 重连，不阻塞 ROS 回调。
"""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String

from center_server.core import base_protocol as proto
from center_server.core.odometry import MecanumGeometry, WheelOdometry, wheel_names
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import SafetyState


class BaseBridgeNode(Node):
    def __init__(self):
        super().__init__("base_bridge_node")
        self.cfg = load_config(self)
        base_cfg = self.cfg.get("base", {})
        self.enabled = bool(base_cfg.get("enabled", True))
        self.port = str(base_cfg.get("port", "/dev/ttyUSB0"))
        self.baud = int(base_cfg.get("baud", 921600))
        self.send_rate = max(5.0, float(base_cfg.get("send_rate_hz", 30.0)))
        self.watchdog = float(base_cfg.get("watchdog_sec", 0.5))
        self.frame_id = str(base_cfg.get("odom_frame", "odom"))
        self.base_frame = str(base_cfg.get("base_frame", "base_link"))
        self.imu_frame = str(base_cfg.get("imu_frame", "imu_link"))
        self.full_voltage = float(base_cfg.get("battery_full_v", 12.6))
        self.empty_voltage = float(base_cfg.get("battery_empty_v", 10.0))
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

        geometry = MecanumGeometry(
            wheel_radius=float(base_cfg.get("wheel_radius", 0.048)),
            wheelbase_x=float(base_cfg.get("wheelbase_x", 0.20)),
            track_y=float(base_cfg.get("track_y", 0.18)),
            encoder_cpr=float(base_cfg.get("encoder_cpr", 1320.0)),
            sign_fl=float(base_cfg.get("sign_fl", 1.0)),
            sign_fr=float(base_cfg.get("sign_fr", 1.0)),
            sign_rl=float(base_cfg.get("sign_rl", 1.0)),
            sign_rr=float(base_cfg.get("sign_rr", 1.0)),
        )
        self.odom_state = WheelOdometry(geometry=geometry, frame_id=self.frame_id)

        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 20)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 20)
        self.pub_battery = self.create_publisher(BatteryState, "/battery_state", 10)
        self.pub_status = self.create_publisher(String, "/base/status", 10)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )

        self._lock = threading.Lock()
        self._serial = None
        self._serial_lock = threading.Lock()
        self._buffer = bytearray()
        self._cmd = None
        self._cmd_time = 0.0
        self._estop = False
        self._seq = 0
        self._last_telemetry_mono = 0.0
        self._last_uptime_ms = None
        self._wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self._last_mode_sent = None
        self._last_mode_time = 0.0
        self._last_status_time = 0.0
        self._closing = False

        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / self.send_rate, self.on_timer, clock=self._steady_clock
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f"base_bridge_node started: {self.port} @ {self.baud}, "
            f"send_rate={self.send_rate:.0f}Hz, watchdog={self.watchdog:.1f}s"
        )

    # ------------------------------------------------------------- callbacks
    def on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._cmd = msg
            self._cmd_time = time.monotonic()

    def on_safety(self, msg: SafetyState):
        with self._lock:
            self._estop = bool(msg.emergency_stop)

    # --------------------------------------------------------------- serial io
    def _open_serial(self):
        try:
            import serial

            ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.05)
            with self._serial_lock:
                self._serial = ser
            self.get_logger().info(f"serial connected: {self.port}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"serial open failed ({self.port}): {exc}")
            return False

    def _close_serial(self):
        with self._serial_lock:
            ser = self._serial
            self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _read_loop(self):
        while not self._closing and rclpy.ok():
            with self._serial_lock:
                ser = self._serial
            if ser is None:
                self._open_serial()
                time.sleep(1.0)
                continue
            try:
                data = ser.read(max(1, getattr(ser, "in_waiting", 1)))  # type: ignore[arg-type]
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"serial read error: {exc}")
                self._close_serial()
                time.sleep(0.5)
                continue
            if not data:
                continue
            self._buffer.extend(data)
            try:
                frames = proto.decode_stream(self._buffer)
            except Exception as exc:  # noqa: BLE001
                self._buffer.clear()
                self.get_logger().warn(f"protocol decode error: {exc}")
                continue
            for frame in frames:
                self._handle_frame(frame)

    def _handle_frame(self, frame: proto.Frame):
        try:
            payload = proto.decode_frame(frame)
        except proto.ProtocolError as exc:
            self.get_logger().debug(f"bad frame: {exc}")
            return
        if isinstance(payload, proto.Telemetry):
            self._handle_telemetry(payload)
        elif isinstance(payload, proto.ImuSample):
            self._publish_imu(payload)

    def _handle_telemetry(self, msg: proto.Telemetry):
        now = time.monotonic()
        dt = 0.0
        if self._last_uptime_ms is not None:
            delta_ms = int(msg.uptime_ms) - int(self._last_uptime_ms)
            if 0 < delta_ms < 1000:
                dt = delta_ms / 1000.0
        if dt <= 0.0 and self._last_telemetry_mono > 0.0:
            dt = min(0.5, now - self._last_telemetry_mono)
        self._last_uptime_ms = int(msg.uptime_ms)
        self._last_telemetry_mono = now

        speeds = dict(zip(wheel_names(), msg.wheel_speeds))
        if dt > 0.0:
            self.odom_state.update_from_wheel_speeds(speeds, dt)

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame
        pose = self.odom_state.pose
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        vx, vy, wz = self.odom_state.twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        for i in (0, 7, 35):
            odom.pose.covariance[i] = 0.02 if i != 35 else 0.05
            odom.twist.covariance[i] = 0.02 if i != 35 else 0.05
        self.pub_odom.publish(odom)

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(self.wheel_joint_names)
        # 编码器计数 -> 轮子转角（rad）
        cpr = max(1.0, self.odom_state.geometry.encoder_cpr)
        for i, delta in enumerate(msg.encoders):
            self._wheel_positions[i] += 2.0 * math.pi * float(delta) / cpr
        joint.position = list(self._wheel_positions)
        joint.velocity = list(msg.wheel_speeds)
        self.pub_joint.publish(joint)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = float(msg.battery_mv) / 1000.0
        span = max(0.1, self.full_voltage - self.empty_voltage)
        battery.percentage = max(
            0.0, min(1.0, (battery.voltage - self.empty_voltage) / span)
        )
        battery.present = True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(battery)

        status = String()
        status.data = json.dumps(
            {
                "seq": int(msg.seq),
                "battery_mv": int(msg.battery_mv),
                "current_ma": int(msg.current_ma),
                "flags": int(msg.flags),
                "estop": bool(msg.estop),
                "encoder_ok": bool(msg.flags & proto.FLAG_ENCODER_OK),
                "imu_ok": bool(msg.flags & proto.FLAG_IMU_OK),
                "wheel_speeds": [round(float(value), 4) for value in msg.wheel_speeds],
                "encoders": [int(value) for value in msg.encoders],
                "uptime_ms": int(msg.uptime_ms),
            },
            ensure_ascii=False,
        )
        self.pub_status.publish(status)

    def _publish_imu(self, sample: proto.ImuSample):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation_covariance[0] = -1.0
        (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ) = (float(v) for v in sample.gyro)
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = (float(v) for v in sample.accel)
        (
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ) = (float(v) * 1e-6 for v in sample.mag)
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.01
            msg.linear_acceleration_covariance[i] = 0.05
            msg.magnetic_field_covariance[i] = 0.01
        self.pub_imu.publish(msg)

    # ---------------------------------------------------------------- sending
    def _send(self, payload: bytes) -> bool:
        with self._serial_lock:
            ser = self._serial
        if ser is None:
            return False
        try:
            ser.write(payload)
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"serial write error: {exc}")
            self._close_serial()
            return False

    def on_timer(self):
        now = time.monotonic()
        with self._lock:
            cmd = self._cmd
            cmd_time = self._cmd_time
            estop = self._estop
        stale = cmd is None or (now - cmd_time) > self.watchdog
        vx = 0.0 if stale else float(cmd.linear.x)
        wz = 0.0 if stale else float(cmd.angular.z)
        self._seq = (self._seq + 1) & 0xFF
        self._send(proto.encode_cmd_vel(vx, wz, estop=estop, seq=self._seq))
        # 模式帧 1Hz 重发，防止 STM32 复位后丢失自动模式
        mode = proto.MODE_ESTOP if estop else proto.MODE_AUTO
        if mode != self._last_mode_sent or (now - self._last_mode_time) > 1.0:
            self._send(proto.encode_mode(mode, estop=estop, seq=self._seq))
            self._last_mode_sent = mode
            self._last_mode_time = now

    def destroy_node(self):
        self._closing = True
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridgeNode()
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
