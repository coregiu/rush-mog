"""高层动作意图 -> 底层 ROS2 控制指令。"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ActionPlanner:
    def __init__(self, cfg):
        cfg = cfg or {}
        self.max_lin = float(cfg.get("max_linear_speed", 0.8))
        self.max_ang = float(cfg.get("max_angular_speed", 1.0))
        self.accel_limit = float(cfg.get("acceleration_limit", 0.3))

    def intent_to_twist(self, intent) -> Twist:
        twist = Twist()
        if getattr(intent, "reverse_request", False):
            return twist

        value = float(getattr(intent, "value", 0.0) or 0.0)
        if not math.isfinite(value):
            value = 0.0
        value = abs(value)
        name = getattr(intent, "intent", "stop")

        if name == "forward":
            twist.linear.x = min(value, self.max_lin)
        elif name == "backward":
            twist.linear.x = -min(value, self.max_lin)
        elif name == "turn_left":
            twist.angular.z = min(value, self.max_ang)
        elif name == "turn_right":
            twist.angular.z = -min(value, self.max_ang)
        return twist

    def limit_acceleration(self, target: Twist, previous: Twist, dt: float) -> Twist:
        """按加速度上限平滑线速度/角速度，避免阶跃指令。"""
        if previous is None or dt <= 0.0:
            return target
        max_delta = max(self.accel_limit * dt, 0.0)
        target.linear.x = previous.linear.x + max(
            -max_delta, min(max_delta, target.linear.x - previous.linear.x)
        )
        target.angular.z = previous.angular.z + max(
            -max_delta, min(max_delta, target.angular.z - previous.angular.z)
        )
        return target

    def make_hold_traj(self, current_joint: dict):
        """用当前关节位置生成保持轨迹（速度为零），用于急停/暂停机械臂。"""
        if not current_joint:
            return None
        traj = JointTrajectory()
        traj.joint_names = list(current_joint.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in current_joint.values()]
        point.velocities = [0.0] * len(point.positions)
        point.time_from_start.sec = 1
        traj.points.append(point)
        return traj

    def intent_to_arm_traj(self, intent, current_joint: dict):
        """DESIGN.md 5.4：机械臂意图 -> JointTrajectory。

        当前 VLA 桩只输出底盘意图；机械臂真实运动学/轨迹规划待模型接入后实现，
        这里对非机械臂目标返回 None，避免向机械臂发布误导性轨迹。
        """
        if getattr(intent, "target", "base") != "arm":
            return None
        return self.make_hold_traj(current_joint)
