from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ActionPlanner:
    def __init__(self, cfg):
        self.max_lin = cfg["max_linear_speed"]
        self.max_ang = cfg["max_angular_speed"]

    def intent_to_twist(self, intent):
        twist = Twist()
        if intent.intent == "forward":
            twist.linear.x = min(intent.value, self.max_lin)
        elif intent.intent == "backward":
            twist.linear.x = -min(intent.value, self.max_lin)
        elif intent.intent == "turn_left":
            twist.angular.z = min(intent.value, self.max_ang)
        elif intent.intent == "turn_right":
            twist.angular.z = -min(intent.value, self.max_ang)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        return twist

    def intent_to_arm_traj(self, intent, current_joint):
        traj = JointTrajectory()
        traj.joint_names = list(current_joint.keys())
        pt = JointTrajectoryPoint()
        pt.positions = list(current_joint.values())
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        return traj
