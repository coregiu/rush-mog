import rclpy
from rclpy.node import Node
from custom_msgs.msg import ReverseOperation
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from center_server.core.reverse_handler import ReverseHandler
import yaml

class ReverseNode(Node):
    def __init__(self):
        super().__init__("reverse_operation_node")
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel",20)
        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/arm/trajectory",20)
        self.sub_rev = self.create_subscription(ReverseOperation, "/reverse/operation", self.cb,10)
        with open("config/config.yaml","r") as f:
            cfg = yaml.safe_load(f)
        self.handler = ReverseHandler(cfg["reverse_operation"])

    def cb(self, op: ReverseOperation):
        if op.op_type == 0:
            twist, traj = self.handler.handle_stop()
        elif op.op_type == 1:
            twist, traj = self.handler.handle_backward()
        elif op.op_type == 2:
            twist, traj = self.handler.handle_arm_reset()
        else:
            twist, traj = self.handler.handle_stop()
        self.pub_cmd_vel.publish(twist)
        self.pub_arm_traj.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ReverseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
