"""localization_node：定位健康度、地图坐标系位姿与初始位姿服务。

定位融合本身由 robot_localization（EKF：ICM-20948 + 轮式里程计）和 AMCL
（激光 + 地图）完成。本节点负责：

- 消费 /imu/data、/odom、/amcl_pose、TF(map->base_link)；
- 发布 /center/pose（map 系，供导航/搜索/目标定位使用）；
- 发布 /center/localization_status，导航节点据此门控 map 系目标；
- 提供 /center/set_initial_pose，转发到 AMCL 的 /initialpose。

ICM-20948 的磁力计只在“可信”时参与航向校正；电机电流/电池磁场会让磁航向
漂移，因此默认不用磁力计硬切航向，只做慢速互补。
"""

from __future__ import annotations

import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

from edge_server.core.imu_monitor import ImuMonitor
from edge_server.core.pose_utils import (
    Pose2D,
    make_pose_with_covariance,
    pose_from_msg,
    yaw_from_quaternion,
)
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.msg import LocalizationStatus
from custom_msgs.srv import SetInitialPose


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        self.cfg = load_config(self)
        loc_cfg = self.cfg.get("localization", {})
        self.pose_rate = max(1.0, float(loc_cfg.get("pose_rate", 10.0)))
        self.status_rate = max(0.5, float(loc_cfg.get("status_rate", 2.0)))
        self.odom_timeout = float(loc_cfg.get("odom_timeout_sec", 0.5))
        self.imu_timeout = float(loc_cfg.get("imu_timeout_sec", 0.3))
        self.amcl_timeout = float(loc_cfg.get("amcl_timeout_sec", 1.0))
        self.min_quality = float(loc_cfg.get("min_quality", 0.35))
        self.base_frame = str(loc_cfg.get("base_frame", "base_link"))
        self.map_frame = str(loc_cfg.get("map_frame", "map"))
        self.imu_monitor = ImuMonitor(
            self.cfg.get("imu", {}) if isinstance(self.cfg.get("imu"), dict) else {}
        )

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/center/pose", 10
        )
        self.pub_status = self.create_publisher(
            LocalizationStatus, "/center/localization_status", 10
        )
        self.pub_initial = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10
        )
        self.create_service(
            SetInitialPose, "/center/set_initial_pose", self.on_set_initial_pose
        )

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_imu = None
        self.last_odom = None
        self.last_amcl = None
        self.last_imu_time = 0.0
        self.last_odom_time = 0.0
        self.last_amcl_time = 0.0
        self.last_tf_time = 0.0
        self.tf_pose: Pose2D | None = None
        self._odom_stamps = deque(maxlen=50)
        self._source = "none"
        self._quality = 0.0
        self._status = None
        self._last_status_time = 0.0

        self._steady_clock = create_steady_clock()
        self.create_timer(1.0 / self.pose_rate, self.on_pose_timer, clock=self._steady_clock)
        self.create_timer(
            1.0 / self.status_rate, self.on_status_timer, clock=self._steady_clock
        )
        self.get_logger().info(
            f"localization_node started (map={self.map_frame}, base={self.base_frame})"
        )

    # ------------------------------------------------------------- callbacks
    def on_imu(self, msg: Imu):
        self.last_imu = msg
        self.last_imu_time = time.monotonic()
        mag = msg.magnetic_field
        self.imu_monitor.update(
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (mag.x * 1e6, mag.y * 1e6, mag.z * 1e6),
            self.last_imu_time,
        )

    def on_odom(self, msg: Odometry):
        self.last_odom = msg
        self.last_odom_time = time.monotonic()
        self._odom_stamps.append(self.last_odom_time)

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        self.last_amcl = msg
        self.last_amcl_time = time.monotonic()

    def on_set_initial_pose(self, request, response):
        msg = request.pose
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_initial.publish(msg)
        yaw = _yaw_from_msg(msg)
        self.imu_monitor.reset_heading(yaw)
        response.success = True
        response.msg = f"initial pose set: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {yaw:.2f})"
        self.get_logger().info(response.msg)
        return response

    # ---------------------------------------------------------------- timers
    def on_pose_timer(self):
        now = time.monotonic()
        pose, source, tf_ok = self._select_pose(now)
        if pose is None:
            return
        msg = make_pose_with_covariance(
            self.map_frame,
            pose,
            stamp=self.get_clock().now().to_msg(),
            var_xy=0.05 if source in ("amcl", "tf") else 0.5,
        )
        self.pub_pose.publish(msg)
        self._source = source
        self._quality = self._compute_quality(source, now)
        if source in ("amcl", "tf"):
            self.last_tf_time = now
        self._tf_ok = tf_ok or source in ("amcl", "tf")

    def on_status_timer(self):
        now = time.monotonic()
        imu_status = self.imu_monitor.status
        quality = getattr(self, "_quality", 0.0)
        if self._source == "none":
            quality = 0.0
        healthy = quality >= self.min_quality and self._source != "none"
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.healthy = bool(healthy)
        msg.quality = float(quality)
        msg.yaw_rad = float(imu_status.yaw_rad)
        msg.yaw_variance = float(imu_status.yaw_variance)
        msg.imu_rate_hz = float(imu_status.rate_hz)
        msg.odom_rate_hz = self._odom_rate()
        msg.imu_online = bool(imu_status.online and (now - self.last_imu_time) <= self.imu_timeout)
        msg.mag_ok = bool(imu_status.mag_ok)
        msg.tf_ok = bool(getattr(self, "_tf_ok", False))
        msg.source = self._source
        msg.detail = self._detail(now, imu_status)
        self.pub_status.publish(msg)
        self._status = msg

    # ---------------------------------------------------------------- helpers
    def _select_pose(self, now: float):
        if self.last_amcl is not None and (now - self.last_amcl_time) <= self.amcl_timeout:
            pose = pose_from_msg(self.last_amcl)
            return Pose2D(pose.x, pose.y, pose.yaw, self.map_frame), "amcl", True

        tf_pose = self._lookup_tf()
        if tf_pose is not None:
            return tf_pose, "tf", True

        if self.last_odom is not None and (now - self.last_odom_time) <= self.odom_timeout:
            pose = pose_from_msg(self.last_odom)
            return Pose2D(pose.x, pose.y, pose.yaw, "odom"), "odom", False
        return None, "none", False

    def _lookup_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001 - TF 未就绪是常态
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return Pose2D(t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), self.map_frame)

    def _odom_rate(self) -> float:
        if len(self._odom_stamps) < 2:
            return 0.0
        span = self._odom_stamps[-1] - self._odom_stamps[0]
        if span <= 1e-6:
            return 0.0
        return float((len(self._odom_stamps) - 1) / span)

    def _compute_quality(self, source: str, now: float) -> float:
        if source == "amcl":
            base = 0.95
        elif source == "tf":
            base = 0.75
        elif source == "odom":
            base = 0.35
        else:
            return 0.0
        imu_status = self.imu_monitor.status
        if imu_status.online and (now - self.last_imu_time) <= self.imu_timeout:
            base = min(1.0, base + 0.05 * imu_status.quality)
        elif source in ("amcl", "tf"):
            base -= 0.10
        if self._odom_rate() < 5.0 and source in ("tf", "odom"):
            base -= 0.15
        return max(0.0, min(1.0, base))

    def _detail(self, now: float, imu_status) -> str:
        parts = [f"source={self._source}", f"quality={getattr(self, '_quality', 0.0):.2f}"]
        if now - self.last_imu_time > self.imu_timeout:
            parts.append("imu_stale")
        if imu_status.mag_ok:
            parts.append("mag_ok")
        elif imu_status.online:
            parts.append("mag_untrusted")
        if not getattr(self, "_tf_ok", False):
            parts.append("tf_unavailable")
        return ",".join(parts)


def _yaw_from_msg(msg: PoseWithCovarianceStamped) -> float:
    q = msg.pose.pose.orientation
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
