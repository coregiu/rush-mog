"""action_generator_node：动作意图 -> /cmd_vel 与 /arm/trajectory。

在发布前执行 SafetyChecker 校验（DESIGN.md 5.4/5.6）。任何急停、手动模式、
数据超时或安全校验失败都会输出零速度，防止继续运动。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from trajectory_msgs.msg import JointTrajectory

from edge_server.core.action_planner import ActionPlanner
from edge_server.core.safety_checker import SafetyChecker
from edge_server.utils.config import load_config
from custom_msgs.msg import (
    ActionIntent,
    ReverseOperation,
    SafetyState,
    SystemStatus,
)


class ActionGeneratorNode(Node):
    def __init__(self):
        super().__init__("action_generator_node")
        self.cfg = load_config(self)
        self.planner = ActionPlanner(self.cfg.get("action_generator", {}))
        self.safety = SafetyChecker(self.cfg.get("safety", {}))
        # v2：VLA 单步速度走 /cmd_vel_raw，由 velocity_mux 与其他来源仲裁后
        # 再发布到 /cmd_vel。单独运行本节点时可用参数改回 /cmd_vel。
        self.cmd_topic = str(
            self.cfg.get("action_generator", {}).get("cmd_vel_topic", "/cmd_vel_raw")
        )

        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_arm_traj = self.create_publisher(
            JointTrajectory, "/arm/trajectory", 10
        )

        self.create_subscription(
            ActionIntent, "/center/action_intent", self.on_intent, 10
        )
        joint_topic = str(
            self.cfg.get("sensor_topics", {}).get(
                "joint_states", "/robot/joint_states"
            )
        )
        self.create_subscription(
            JointState,
            joint_topic,
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        laser_topic = str(
            self.cfg.get("sensor_topics", {}).get("laser", "/scan")
        )
        self.create_subscription(
            LaserScan, laser_topic, self.on_scan, qos_profile_sensor_data
        )
        if laser_topic != "/edge/radar":
            # 兼容仿真/旧版 edge_server
            self.create_subscription(
                LaserScan, "/edge/radar", self.on_scan, qos_profile_sensor_data
            )
        self.create_subscription(
            SystemStatus, "/center/status", self.on_status, 10
        )
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.create_subscription(
            ReverseOperation, "/reverse/operation", self.on_reverse, 10
        )

        self.latest_joint = {}
        self.latest_scan = None
        self.emergency_stop = False
        self.data_ok = True
        self.mode = "auto"
        self.task_state = "IDLE"
        self.reverse_until = 0.0
        self.reverse_hold = float(
            self.cfg.get("reverse_operation", {}).get("operation_timeout", 5.0)
        )
        self.last_twist = None
        self.last_cmd_time = None
        self._last_warn_time = 0.0
        self._intent_count = 0
        self.get_logger().info("action_generator_node started")

    # ------------------------------------------------------------- callbacks
    def on_joint_state(self, msg: JointState):
        self.latest_joint = dict(zip(msg.name, msg.position))

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_status(self, msg: SystemStatus):
        was_gated = self._is_gated()
        self.mode = msg.mode
        self.task_state = msg.state
        # REVERSING 时由 reverse_operation_node 独占 /cmd_vel；
        # 这里若再发零速度会覆盖反向指令。
        if (
            self._is_gated()
            and not was_gated
            and msg.state != "REVERSING"
        ):
            self._publish_stop()

    def on_safety(self, msg: SafetyState):
        was_gated = self._is_gated()
        was_estop = self.emergency_stop
        self.emergency_stop = bool(msg.emergency_stop)
        self.data_ok = bool(msg.data_ok)
        if self.emergency_stop and not was_estop:
            self.get_logger().warn(f"safety stop active: {msg.reason}")
        elif was_estop and not self.emergency_stop:
            self.get_logger().info("safety stop cleared")
        if self._is_gated() and not was_gated:
            self._publish_stop()

    def on_reverse(self, op: ReverseOperation):
        # 反向操作优先级最高：立即门控普通动作，但不发布零速度，
        # 避免覆盖 reverse_operation_node 刚发出的回退指令。
        self.reverse_until = time.monotonic() + self.reverse_hold
        self.get_logger().warn(
            f"reverse operation received (type={op.operation_type}); "
            "auto motion suppressed"
        )

    def _is_gated(self):
        if self.emergency_stop:
            return True
        if not self.data_ok:
            return True
        if self.mode != "auto":
            return True
        if time.monotonic() < self.reverse_until:
            return True
        return self.task_state in (
            "MAPPING",
            "NAVIGATING",
            "SEARCHING",
            "REVERSING",
            "PAUSED",
            "ERROR",
            "TASK_SUCCESS",
            "TASK_FAILED",
        )

    def _publish_stop(self):
        self.pub_cmd_vel.publish(Twist())
        self.last_twist = Twist()
        self.last_cmd_time = time.monotonic()
        traj = self.planner.make_hold_traj(self.latest_joint)
        if traj is not None:
            self.pub_arm_traj.publish(traj)

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._last_warn_time >= 1.0:
            self._last_warn_time = now
            self.get_logger().warn(message)

    # ---------------------------------------------------------------- intents
    def on_intent(self, intent: ActionIntent):
        self._intent_count += 1
        if self._intent_count <= 5:
            self.get_logger().info(
                f"intent #{self._intent_count}: {intent.intent} "
                f"task_type={intent.task_type or 'vla'} "
                f"task_state={self.task_state} mode={self.mode} "
                f"estop={self.emergency_stop} data_ok={self.data_ok} "
                f"scan={self.latest_scan is not None} "
                f"gated={self._is_gated()}"
            )
        if self._is_gated() or intent.reverse_request:
            self._publish_stop()
            return
        if intent.task_type not in ("", "vla"):
            # 建图/导航/搜索任务由导航节点控制底盘，VLA 单步通道让出
            self._publish_stop()
            return
        if self.latest_scan is None:
            # 无雷达数据时按 DESIGN.md 11 的异常策略拒绝运动
            self._warn_throttled("no laser data, motion rejected")
            self._publish_stop()
            return

        twist = self.planner.intent_to_twist(intent)
        laser = self.latest_scan.ranges
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ok, reason = self.safety.check_twist(
            twist, laser, angle_min, angle_increment
        )
        if not ok:
            self._warn_throttled(f"twist rejected by safety: {reason}")
            self._publish_stop()
            return

        now = time.monotonic()
        dt = now - self.last_cmd_time if self.last_cmd_time is not None else 0.0
        twist = self.planner.limit_acceleration(twist, self.last_twist, dt)
        self.pub_cmd_vel.publish(twist)
        self.last_twist = twist
        self.last_cmd_time = now

        if intent.target == "arm":
            traj = self.planner.intent_to_arm_traj(intent, self.latest_joint)
            if traj is not None:
                arm_ok, arm_reason = self.safety.check_arm_traj(
                    traj, self.latest_joint
                )
                if arm_ok:
                    self.pub_arm_traj.publish(traj)
                else:
                    self._warn_throttled(
                        f"arm trajectory rejected by safety: {arm_reason}"
                    )

        self.get_logger().debug(
            f"intent={intent.intent} -> linear.x={twist.linear.x:.2f}, "
            f"angular.z={twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActionGeneratorNode()
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
