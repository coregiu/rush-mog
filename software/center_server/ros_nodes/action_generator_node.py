import rclpy
from rclpy.node import Node
from custom_msgs.msg import ActionIntent
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from center_server.core.action_planner import ActionPlanner
import yaml

class ActionGeneratorNode(Node):
    def __init__(self):
        super().__init__("action_generator_node")
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel",10)
        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/arm/trajectory",10)
        self.sub_intent = self.create_subscription(ActionIntent, "/center/action_intent", self.cb,10)
        with open("config/config.yaml","r") as f:
            cfg = yaml.safe_load(f)
        self.planner = ActionPlanner(cfg["action_generator"])

    def cb(self, intent):
        twist = self.planner.intent_to_twist(intent)
        # 简单占位joint
        dummy_joint = {"j1":0.0,"j2":1.0}
        traj = self.planner.intent_to_arm_traj(intent, dummy_joint)
        self.pub_cmd_vel.publish(twist)
        self.pub_arm_traj.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ActionGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
