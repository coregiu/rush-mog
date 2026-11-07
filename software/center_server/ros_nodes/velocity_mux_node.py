"""velocity_mux_node：把 VLA / Nav2 / 反向操作 / 安全覆盖合并为唯一 /cmd_vel。

上游话题：
    /safety/cmd_vel_override   安全模块（最高优先级）
    /reverse/cmd_vel           反向操作
    /teleop/cmd_vel            遥控
    /nav/cmd_vel               Nav2 或内置直航
    /cmd_vel_raw               VLA 单步动作

输出：
    /cmd_vel                   最终底盘速度
    /center/velocity_status    当前生效源（调试）
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from center_server.core.velocity_mux import VelocityMux
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import SafetyState, VelocityMuxStatus


class VelocityMuxNode(Node):
    def __init__(self):
        super().__init__("velocity_mux_node")
        self.cfg = load_config(self)
        mux_cfg = self.cfg.get("velocity_mux", {})
        self.rate = max(2.0, float(mux_cfg.get("publish_rate", 20.0)))
        self.status_rate = max(0.5, float(mux_cfg.get("status_rate", 5.0)))
        self.mux = VelocityMux(
            default_timeout=float(mux_cfg.get("default_timeout", 0.5)),
            max_linear=float(mux_cfg.get("max_linear", 1.0)),
            max_angular=float(mux_cfg.get("max_angular", 1.5)),
            priorities=mux_cfg.get("priorities")
            or {"safety": 0, "reverse": 1, "teleop": 2, "nav": 3, "vla": 4},
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(
            VelocityMuxStatus, "/center/velocity_status", 10
        )

        topics = mux_cfg.get("topics") or {}
        self._bind(str(topics.get("safety", "/safety/cmd_vel_override")), "safety")
        self._bind(str(topics.get("reverse", "/reverse/cmd_vel")), "reverse")
        self._bind(str(topics.get("teleop", "/teleop/cmd_vel")), "teleop")
        self._bind(str(topics.get("nav", "/nav/cmd_vel")), "nav")
        self._bind(str(topics.get("vla", "/cmd_vel_raw")), "vla")

        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self._last_source = None
        self._last_status = 0.0
        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / self.rate, self.on_timer, clock=self._steady_clock
        )
        self.get_logger().info(
            f"velocity_mux_node started ({self.rate:.0f} Hz, "
            f"priorities=safety>reverse>teleop>nav>vla)"
        )

    def _bind(self, topic: str, name: str):
        prio = self.mux.priorities.get(name, 9)
        timeout = float(
            (self.cfg.get("velocity_mux", {}).get("timeouts") or {}).get(name, 0.5)
        )
        self.create_subscription(
            Twist,
            topic,
            lambda msg, source=name: self.mux.set(
                source, msg, time.monotonic(), priority=prio, timeout=timeout
            ),
            10,
        )

    def on_safety(self, msg: SafetyState):
        self.mux.set_emergency(
            bool(msg.emergency_stop),
            msg.reason if msg.emergency_stop else "",
        )

    def on_timer(self):
        now = time.monotonic()
        twist, source, active, emergency = self.mux.compute(now)
        self.pub_cmd.publish(twist)
        if source != self._last_source:
            self.get_logger().info(
                f"velocity source -> {source} (active={active}, estop={emergency})"
            )
            self._last_source = source
        if now - self._last_status >= 1.0 / self.status_rate:
            self._last_status = now
            status = VelocityMuxStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.active_source = source
            status.active_sources = list(active)
            status.emergency_stop = bool(emergency)
            status.output = twist
            status.detail = ",".join(active) if active else "no active source"
            self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMuxNode()
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
