"""反向操作（急停/回退/机械臂复位/任务撤销/手动接管）处理。"""

from __future__ import annotations

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ReverseHandler:
    def __init__(self, cfg):
        cfg = cfg or {}
        self.rev_speed = float(cfg.get("reverse_speed", 0.3))
        self.rev_dist = float(cfg.get("reverse_distance", 0.5))
        self.operation_timeout = float(cfg.get("operation_timeout", 5.0))
        self.arm_joint_names = list(
            cfg.get("arm_joint_names", ["j1", "j2", "j3", "j4", "j5"])
        )
        self.arm_home_positions = [
            float(v)
            for v in cfg.get("arm_home_positions", [0.0, 1.57, 0.0, 1.0, 0.5])
        ]

    def _hold_traj(self, current_joint):
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

    def handle_stop(self, current_joint=None):
        return Twist(), self._hold_traj(current_joint)

    def handle_backward(self, current_joint=None, distance=None):
        twist = Twist()
        twist.linear.x = -abs(self.rev_speed)
        return twist, self._hold_traj(current_joint)

    def handle_arm_reset(self):
        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(self.arm_home_positions)
        point.velocities = [0.0] * len(point.positions)
        point.time_from_start.sec = 2
        traj.points.append(point)
        return Twist(), traj

    def handle_task_cancel(self, current_joint=None):
        return self.handle_stop(current_joint)

    def handle_manual_override(self, current_joint=None):
        return self.handle_stop(current_joint)

    def reverse_duration(self, distance=None) -> float:
        """回退 distance 米所需时间，限制在 (0.1, operation_timeout] 秒。"""
        distance = abs(float(distance if distance is not None else self.rev_dist))
        if distance <= 0.0:
            distance = abs(self.rev_dist)
        if self.rev_speed <= 0.0:
            return 0.1
        duration = distance / self.rev_speed
        return max(0.1, min(duration, self.operation_timeout))
