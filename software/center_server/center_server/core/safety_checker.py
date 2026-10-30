import numpy as np
from geometry_msgs.msg import Twist

class SafetyChecker:
    def __init__(self, cfg):
        self.min_dist = cfg["obstacle_stop_distance"]
        self.max_lin = cfg["max_twist_x"]
        self.max_ang = cfg["max_twist_z"]

    def check_twist(self, twist: Twist, laser: np.ndarray) -> tuple[bool, str]:
        if abs(twist.linear.x) > self.max_lin or abs(twist.angular.z) > self.max_ang:
            return False, "speed out of limit"
        if len(laser) > 0 and np.min(laser) < self.min_dist and twist.linear.x > 0:
            return False, "obstacle too close"
        return True, "ok"

    def check_arm_traj(self, traj, current_joint) -> tuple[bool, str]:
        # 简单限位占位，可扩展
        return True, "ok"
