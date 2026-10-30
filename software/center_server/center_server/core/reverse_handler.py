from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ReverseHandler:
    def __init__(self, cfg):
        self.rev_speed = cfg["reverse_speed"]
        self.rev_dist = cfg["reverse_distance"]

    def handle_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        traj = JointTrajectory()
        return twist, traj

    def handle_backward(self):
        twist = Twist()
        twist.linear.x = -self.rev_speed
        traj = JointTrajectory()
        return twist, traj

    def handle_arm_reset(self):
        traj = JointTrajectory()
        traj.joint_names = ["j1","j2","j3","j4","j5"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 1.57, 0.0, 1.0, 0.5]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        twist = Twist()
        return twist, traj
