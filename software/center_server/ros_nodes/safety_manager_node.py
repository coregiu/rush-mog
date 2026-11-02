"""safety_manager_node：障碍物监控、急停服务与安全状态发布。

服务：/center/emergency_stop (custom_msgs/srv/EmergencyStop)
发布：/center/safety_state、/cmd_vel（急停覆盖指令）
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from center_server.core.safety_checker import SafetyChecker
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import SafetyState, SyncedSensorData
from custom_msgs.srv import EmergencyStop


class SafetyManagerNode(Node):
    def __init__(self):
        super().__init__("safety_manager_node")
        self.cfg = load_config(self)
        self.safety_cfg = self.cfg.get("safety", {})
        self.checker = SafetyChecker(self.safety_cfg)
        self.data_timeout = float(self.safety_cfg.get("data_timeout_sec", 2.0))
        self.clear_dist = float(
            self.safety_cfg.get("obstacle_clear_distance", self.checker.min_dist + 0.2)
        )

        self.pub_state = self.create_publisher(
            SafetyState,
            "/center/safety_state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            LaserScan, "/edge/radar", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            SyncedSensorData,
            "/center/synced_data",
            self.on_synced,
            qos_profile_sensor_data,
        )
        self.create_service(
            EmergencyStop, "/center/emergency_stop", self.on_emergency_service
        )

        self.last_data_time = time.monotonic()
        self.last_zero_time = 0.0
        self.min_distance = None
        self.manual_stop = False
        self.obstacle_stop = False
        self.emergency_stop = False
        self.data_ok = True
        self.reason = "initializing"
        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            0.1, self.on_tick, clock=self._steady_clock
        )
        self.get_logger().info(
            f"safety_manager_node started (stop_distance="
            f"{self.checker.min_dist:.2f} m, data_timeout={self.data_timeout:.1f} s)"
        )

    # ------------------------------------------------------------- callbacks
    def on_scan(self, msg: LaserScan):
        self.min_distance = self.checker.nearest_obstacle(
            msg.ranges, msg.angle_min, msg.angle_increment
        )

    def on_synced(self, msg: SyncedSensorData):
        self.last_data_time = time.monotonic()

    def on_emergency_service(self, request, response):
        if request.trigger:
            self.manual_stop = True
            self.get_logger().warn("emergency stop requested by service")
        else:
            self.manual_stop = False
            self.get_logger().info("emergency stop release requested by service")
        self._refresh_state()
        # 立即广播状态，不等待下一次定时器（避免墙钟跳变导致解除急停延迟）
        self._publish_state()
        response.ack = True
        return response

    # ---------------------------------------------------------------- internal
    def _publish_state(self):
        state = SafetyState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.emergency_stop = bool(self.emergency_stop)
        state.data_ok = bool(self.data_ok)
        state.min_obstacle_distance = (
            float(self.min_distance) if self.min_distance is not None else -1.0
        )
        state.reason = self.reason
        self.pub_state.publish(state)

    def _refresh_state(self):
        elapsed = time.monotonic() - self.last_data_time
        self.data_ok = elapsed <= self.data_timeout

        if self.min_distance is not None:
            if self.min_distance < self.checker.min_dist:
                self.obstacle_stop = True
            elif self.min_distance > self.clear_dist:
                self.obstacle_stop = False

        was_stop = self.emergency_stop
        self.emergency_stop = self.manual_stop or self.obstacle_stop

        reasons = []
        if self.manual_stop:
            reasons.append("manual_emergency_stop")
        if self.obstacle_stop:
            reasons.append(
                f"obstacle_too_close({self.min_distance:.2f}m)"
                if self.min_distance is not None
                else "obstacle_too_close"
            )
        if not self.data_ok:
            reasons.append("sensor_data_timeout")
        self.reason = ",".join(reasons) if reasons else "ok"

        if self.emergency_stop and not was_stop:
            self.get_logger().error(f"EMERGENCY STOP engaged: {self.reason}")
            self.pub_cmd_vel.publish(Twist())
        elif was_stop and not self.emergency_stop:
            self.get_logger().info("emergency stop cleared")

    def on_tick(self):
        self._refresh_state()
        self._publish_state()

        # 急停期间持续覆盖 /cmd_vel，确保底盘执行器不会因丢失单条消息继续运动
        if self.emergency_stop:
            now = time.monotonic()
            if now - self.last_zero_time >= 0.2:
                self.last_zero_time = now
                self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManagerNode()
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
