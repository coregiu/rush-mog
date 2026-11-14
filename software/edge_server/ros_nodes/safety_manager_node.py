"""safety_manager_node：障碍物、数据超时、低电量与定位失效监控。

发布：
- /center/safety_state             安全状态（供 task_manager/mux/导航门控）
- /safety/cmd_vel_override         急停覆盖速度（优先级最高的速度源）
服务：
- /center/emergency_stop           手动急停/解除

v2 新增：
- 分方向障碍物检查（前进/后退/横移）；
- 低电量保护与定位失效保护（仅在导航/接近阶段生效，避免建图时误停）；
- 急停覆盖改发 /safety/cmd_vel_override，由 velocity_mux 统一仲裁。
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
from sensor_msgs.msg import BatteryState, LaserScan

from edge_server.core.safety_checker import SafetyChecker
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.msg import LocalizationStatus, MissionStatus, SafetyState, SyncedSensorData
from custom_msgs.srv import EmergencyStop

_NAV_ACTIVE_STATES = {
    "NAVIGATING",
    "SEARCHING",
    "navigating",
    "exploring",
    "confirming",
    "approaching",
}


class SafetyManagerNode(Node):
    def __init__(self):
        super().__init__("safety_manager_node")
        self.cfg = load_config(self)
        self.safety_cfg = self.cfg.get("safety", {})
        self.topics = self.cfg.get("sensor_topics", {})
        self.checker = SafetyChecker(self.safety_cfg)
        self.data_timeout = float(self.safety_cfg.get("data_timeout_sec", 2.0))
        self.scan_timeout = float(self.safety_cfg.get("scan_timeout_sec", 1.0))
        self.clear_dist = float(
            self.safety_cfg.get("obstacle_clear_distance", self.checker.min_dist + 0.2)
        )
        self.low_battery = float(self.safety_cfg.get("battery_low_percent", 15.0))
        self.stop_on_low_battery = bool(
            self.safety_cfg.get("stop_on_low_battery", True)
        )
        self.require_localization = bool(
            self.safety_cfg.get("require_localization_for_nav", True)
        )
        self.localization_timeout = float(
            self.safety_cfg.get("localization_timeout_sec", 2.0)
        )
        self.localization_grace = float(
            self.safety_cfg.get("localization_grace_sec", 1.5)
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
        self.pub_override = self.create_publisher(
            Twist, "/safety/cmd_vel_override", 10
        )

        laser_topic = str(self.topics.get("laser", "/scan"))
        self.create_subscription(
            LaserScan, laser_topic, self.on_scan, qos_profile_sensor_data
        )
        if laser_topic != "/edge/radar":
            self.create_subscription(
                LaserScan, "/edge/radar", self.on_scan, qos_profile_sensor_data
            )
        self.create_subscription(
            SyncedSensorData,
            "/center/synced_data",
            self.on_synced,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BatteryState, "/battery_state", self.on_battery, 10
        )
        self.create_subscription(
            LocalizationStatus, "/center/localization_status", self.on_localization, 10
        )
        self.create_subscription(
            MissionStatus, "/center/mission_status", self.on_mission, 10
        )
        self.create_service(
            EmergencyStop, "/center/emergency_stop", self.on_emergency_service
        )

        now = time.monotonic()
        self.last_data_time = now
        self.last_scan_time = 0.0
        self.last_loc_time = 0.0
        self.last_zero_time = 0.0
        self.min_distance = None
        self.manual_stop = False
        self.obstacle_stop = False
        self.battery_stop = False
        self.localization_stop = False
        self.emergency_stop = False
        self.data_ok = True
        self.localization_ok = True
        self.battery_ok = True
        self.battery_percent = -1.0
        self.mission_state = ""
        self._loc_bad_since = None
        self.reason = "initializing"
        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(0.1, self.on_tick, clock=self._steady_clock)
        self.get_logger().info(
            f"safety_manager_node started (stop_distance={self.checker.min_dist:.2f} m, "
            f"data_timeout={self.data_timeout:.1f}s, laser={laser_topic})"
        )

    # ------------------------------------------------------------- callbacks
    def on_scan(self, msg: LaserScan):
        self.min_distance = self.checker.nearest_obstacle(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            fov_center=0.0,
            fov_deg=self.checker.front_fov_deg,
        )
        self.last_scan_time = time.monotonic()

    def on_synced(self, msg: SyncedSensorData):
        self.last_data_time = time.monotonic()

    def on_battery(self, msg: BatteryState):
        if msg.voltage > 1.0 and 0.0 <= msg.percentage <= 1.0:
            self.battery_percent = float(msg.percentage) * 100.0
        elif 0.0 <= msg.percentage <= 1.0:
            self.battery_percent = float(msg.percentage) * 100.0
        else:
            self.battery_percent = -1.0
        if self.battery_percent < 0.0:
            self.battery_ok = True
        else:
            self.battery_ok = self.battery_percent >= self.low_battery
        if not self.battery_ok and self.stop_on_low_battery:
            self.get_logger().error(
                f"battery low: {self.battery_percent:.1f}% < {self.low_battery:.1f}%"
            )

    def on_localization(self, msg: LocalizationStatus):
        self.localization_ok = bool(msg.healthy)
        self.last_loc_time = time.monotonic()

    def on_mission(self, msg: MissionStatus):
        self.mission_state = msg.state

    def on_emergency_service(self, request, response):
        if request.trigger:
            self.manual_stop = True
            self.get_logger().warn("emergency stop requested by service")
        else:
            self.manual_stop = False
            self.get_logger().info("emergency stop release requested by service")
        self._refresh_state()
        self._publish_state()
        self.pub_override.publish(Twist())
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
        state.localization_ok = bool(self.localization_ok)
        state.battery_ok = bool(self.battery_ok)
        state.battery_percent = float(self.battery_percent)
        self.pub_state.publish(state)

    def _refresh_state(self):
        now = time.monotonic()
        synced_ok = (now - self.last_data_time) <= self.data_timeout
        scan_ok = (now - self.last_scan_time) <= self.scan_timeout
        self.data_ok = bool(synced_ok or scan_ok)

        if self.min_distance is not None:
            if self.min_distance < self.checker.min_dist:
                self.obstacle_stop = True
            elif self.min_distance > self.clear_dist:
                self.obstacle_stop = False

        self.battery_stop = bool(
            self.stop_on_low_battery
            and self.battery_percent >= 0.0
            and self.battery_percent < self.low_battery
        )

        # 定位保护只在导航/搜索阶段生效
        nav_active = self.mission_state in _NAV_ACTIVE_STATES
        loc_fresh = (now - self.last_loc_time) <= self.localization_timeout
        loc_bad = bool(nav_active and self.require_localization and not (self.localization_ok and loc_fresh))
        if loc_bad:
            if self._loc_bad_since is None:
                self._loc_bad_since = now
            self.localization_stop = (now - self._loc_bad_since) >= self.localization_grace
        else:
            self._loc_bad_since = None
            self.localization_stop = False

        was_stop = self.emergency_stop
        self.emergency_stop = bool(
            self.manual_stop
            or self.obstacle_stop
            or self.battery_stop
            or self.localization_stop
        )

        reasons = []
        if self.manual_stop:
            reasons.append("manual_emergency_stop")
        if self.obstacle_stop:
            reasons.append(
                f"obstacle_too_close({self.min_distance:.2f}m)"
                if self.min_distance is not None
                else "obstacle_too_close"
            )
        if self.battery_stop:
            reasons.append(f"battery_low({self.battery_percent:.1f}%)")
        if self.localization_stop:
            reasons.append("localization_lost")
        if not self.data_ok:
            reasons.append("sensor_data_timeout")
        self.reason = ",".join(reasons) if reasons else "ok"

        if self.emergency_stop and not was_stop:
            self.get_logger().error(f"EMERGENCY STOP engaged: {self.reason}")
            self.pub_override.publish(Twist())
        elif was_stop and not self.emergency_stop:
            self.get_logger().info("emergency stop cleared")

    def on_tick(self):
        self._refresh_state()
        self._publish_state()
        if self.emergency_stop:
            now = time.monotonic()
            if now - self.last_zero_time >= 0.2:
                self.last_zero_time = now
                self.pub_override.publish(Twist())


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
