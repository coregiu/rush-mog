"""安全校验：速度限制、分方向障碍物约束、机械臂关节限位。

与 v1 的区别：v1 只检查前进方向；Nav2/麦轮会横向移动甚至后退，因此按
linear.x / linear.y 的方向分别检查前、后、左、右 FOV。
"""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Twist


class SafetyChecker:
    def __init__(self, cfg):
        cfg = cfg or {}
        self.min_dist = float(cfg.get("obstacle_stop_distance", 0.5))
        self.clear_dist = float(
            cfg.get("obstacle_clear_distance", self.min_dist + 0.2)
        )
        self.max_lin = float(cfg.get("max_twist_x", 1.0))
        self.max_ang = float(cfg.get("max_twist_z", 1.5))
        self.front_fov_deg = float(cfg.get("laser_front_fov_deg", 60.0))
        self.rear_fov_deg = float(cfg.get("laser_rear_fov_deg", 60.0))
        self.side_fov_deg = float(cfg.get("laser_side_fov_deg", 45.0))
        self.joint_limits = {
            str(name): (float(bounds[0]), float(bounds[1]))
            for name, bounds in (cfg.get("joint_limits") or {}).items()
        }

    def nearest_obstacle(
        self,
        laser,
        angle_min=None,
        angle_increment=None,
        fov_center: float = 0.0,
        fov_deg: float = None,
    ):
        """返回指定 FOV 内最近的有效障碍距离；无有效数据返回 None。"""
        if laser is None:
            return None
        ranges = np.asarray(laser, dtype=np.float64).reshape(-1)
        if ranges.size == 0:
            return None
        mask = np.isfinite(ranges) & (ranges > 0.0)

        if (
            angle_min is not None
            and angle_increment is not None
            and abs(float(angle_increment)) > 1e-9
        ):
            angles = float(angle_min) + np.arange(ranges.size) * float(angle_increment)
            # 归一化到 [-pi, pi]，只保留指定方向 FOV
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            diff = np.arctan2(
                np.sin(angles - float(fov_center)),
                np.cos(angles - float(fov_center)),
            )
            half = math.radians(float(fov_deg if fov_deg is not None else self.front_fov_deg) / 2.0)
            mask &= np.abs(diff) <= half

        if not np.any(mask):
            return None
        return float(np.min(ranges[mask]))

    def check_twist(self, twist: Twist, laser=None, angle_min=None, angle_increment=None):
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if any(not math.isfinite(float(v)) for v in values):
            return False, "non-finite twist command"
        if abs(twist.linear.x) > self.max_lin:
            return False, f"linear speed out of limit ({twist.linear.x:.2f} m/s)"
        if abs(twist.linear.y) > self.max_lin:
            return False, f"lateral speed out of limit ({twist.linear.y:.2f} m/s)"
        if abs(twist.angular.z) > self.max_ang:
            return False, f"angular speed out of limit ({twist.angular.z:.2f} rad/s)"

        checks = []
        if twist.linear.x > 1e-6:
            checks.append((0.0, self.front_fov_deg, "front"))
        elif twist.linear.x < -1e-6:
            checks.append((math.pi, self.rear_fov_deg, "rear"))
        if twist.linear.y > 1e-6:
            checks.append((math.pi / 2.0, self.side_fov_deg, "left"))
        elif twist.linear.y < -1e-6:
            checks.append((-math.pi / 2.0, self.side_fov_deg, "right"))

        for center, fov, name in checks:
            distance = self.nearest_obstacle(
                laser,
                angle_min,
                angle_increment,
                fov_center=center,
                fov_deg=fov,
            )
            if distance is not None and distance < self.min_dist:
                return False, f"obstacle too close on {name} ({distance:.2f} m)"
        return True, "ok"

    def check_arm_traj(self, traj, current_joint=None):
        if traj is None:
            return True, "ok"
        for point in traj.points:
            if len(point.positions) != len(traj.joint_names):
                return False, "joint name/position length mismatch"
            for name, position in zip(traj.joint_names, point.positions):
                position = float(position)
                if not math.isfinite(position):
                    return False, f"non-finite joint position for {name}"
                bounds = self.joint_limits.get(name)
                if bounds and not (bounds[0] <= position <= bounds[1]):
                    return (
                        False,
                        f"joint {name} out of range ({position:.2f} not in "
                        f"[{bounds[0]:.2f}, {bounds[1]:.2f}])",
                    )
        return True, "ok"
