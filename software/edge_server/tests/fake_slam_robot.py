#!/usr/bin/env python3
"""模拟“已建图 + 已定位”的机器人，用于无硬件联调 SLAM/导航/搜索链路。

发布：
    /map                          OccupancyGrid（free 区域 + 未知边界）
    /scan                         LaserScan
    /center/pose                  随 /nav/cmd_vel 积分的位姿
    /center/localization_status   高质量定位
    /center/safety_state          无急停
    /center/target_detections     延迟 target_delay 秒后发布 cup 目标

需要同时运行 map_manager_node（提供 /center/map_info、/center/frontiers）。
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from edge_server.core.pose_utils import Pose2D, quaternion_from_yaw
from edge_server.utils.clock import create_steady_clock
from custom_msgs.msg import (
    LocalizationStatus,
    SafetyState,
    TargetDetection,
    TargetList,
)


class FakeSlamRobot(Node):
    def __init__(self):
        super().__init__("fake_slam_robot")
        self.declare_parameter("target_delay", 3.0)
        self.declare_parameter("target_x", 1.5)
        self.declare_parameter("target_y", 0.0)
        self.target_delay = float(self.get_parameter("target_delay").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.start = time.monotonic()
        self.pose = Pose2D(0.0, 0.0, 0.0, "map")
        self.twist = Twist()
        self._last_update = time.monotonic()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/center/pose", 10
        )
        self.pub_scan = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.pub_loc = self.create_publisher(
            LocalizationStatus, "/center/localization_status", 10
        )
        self.pub_safety = self.create_publisher(
            SafetyState, "/center/safety_state", 10
        )
        self.pub_targets = self.create_publisher(
            TargetList, "/center/target_detections", 10
        )
        self.create_subscription(
            Twist, "/nav/cmd_vel", self.on_cmd, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        self._map = self._build_map()
        self._steady_clock = create_steady_clock()
        self.create_timer(0.05, self.on_pose_timer, clock=self._steady_clock)
        self.create_timer(0.1, self.on_scan_timer, clock=self._steady_clock)
        self.create_timer(1.0, self.on_map_timer, clock=self._steady_clock)
        self.create_timer(0.5, self.on_status_timer, clock=self._steady_clock)
        self.create_timer(0.1, self.on_target_timer, clock=self._steady_clock)
        self.get_logger().info(
            f"fake_slam_robot started; target cup at "
            f"({self.target_x:.2f}, {self.target_y:.2f}) after "
            f"{self.target_delay:.1f}s"
        )

    def _build_map(self) -> OccupancyGrid:
        width = height = 120
        resolution = 0.05
        data = np.full((height, width), -1, dtype=np.int8)
        data[20:100, 20:100] = 0
        data[60, 20:100] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.width = width
        msg.info.height = height
        msg.info.resolution = resolution
        msg.info.origin.position.x = -3.0
        msg.info.origin.position.y = -3.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.reshape(-1).tolist()
        return msg

    def on_cmd(self, msg: Twist):
        self.twist = msg

    def on_pose_timer(self):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self._last_update))
        self._last_update = now
        vx, vy, wz = self.twist.linear.x, self.twist.linear.y, self.twist.angular.z
        mid = self.pose.yaw + 0.5 * wz * dt
        self.pose = Pose2D(
            self.pose.x + (vx * math.cos(mid) - vy * math.sin(mid)) * dt,
            self.pose.y + (vx * math.sin(mid) + vy * math.cos(mid)) * dt,
            self.pose.yaw + wz * dt,
            "map",
        )
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.pose.x
        msg.pose.pose.position.y = self.pose.y
        qx, qy, qz, qw = quaternion_from_yaw(self.pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.pub_pose.publish(msg)

    def on_scan_timer(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        num = 360
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - 2.0 * math.pi / num
        msg.angle_increment = 2.0 * math.pi / num
        msg.range_min = 0.05
        msg.range_max = 12.0
        msg.scan_time = 0.1
        msg.time_increment = 0.0
        msg.ranges = [5.0] * num
        self.pub_scan.publish(msg)

    def on_map_timer(self):
        self._map.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self._map)

    def on_status_timer(self):
        loc = LocalizationStatus()
        loc.header.stamp = self.get_clock().now().to_msg()
        loc.healthy = True
        loc.quality = 0.9
        loc.yaw_rad = self.pose.yaw
        loc.yaw_variance = 0.01
        loc.imu_rate_hz = 100.0
        loc.odom_rate_hz = 30.0
        loc.imu_online = True
        loc.mag_ok = True
        loc.tf_ok = True
        loc.source = "tf"
        loc.detail = "fake"
        self.pub_loc.publish(loc)

        safety = SafetyState()
        safety.header.stamp = self.get_clock().now().to_msg()
        safety.emergency_stop = False
        safety.data_ok = True
        safety.min_obstacle_distance = 5.0
        safety.reason = "ok"
        safety.localization_ok = True
        safety.battery_ok = True
        safety.battery_percent = 80.0
        self.pub_safety.publish(safety)

    def on_target_timer(self):
        if time.monotonic() - self.start < self.target_delay:
            return
        detection = TargetDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = "map"
        detection.task_id = "search_demo"
        detection.label = "cup"
        detection.track_id = "trk_cup"
        detection.confidence = 0.9
        detection.x_min = 0.45
        detection.y_min = 0.4
        detection.x_max = 0.55
        detection.y_max = 0.6
        detection.bearing_rad = 0.0
        detection.distance_m = math.hypot(
            self.target_x - self.pose.x, self.target_y - self.pose.y
        )
        detection.position.x = self.target_x
        detection.position.y = self.target_y
        detection.position_variance.x = 0.0025
        detection.position_variance.y = 0.0025
        detection.position_frame = "map"
        detection.distance_source = "lidar"
        msg = TargetList()
        msg.header = detection.header
        msg.task_id = "search_demo"
        msg.detections.append(detection)
        self.pub_targets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSlamRobot()
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
