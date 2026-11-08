"""reverse_operation_node：处理急停、回退、机械臂复位、任务撤销、手动接管。

反向操作优先级高于 VLA 普通推理：本节点直接向 /cmd_vel 与 /arm/trajectory
发布覆盖指令；task_manager_node 同时把系统状态置为 REVERSING，动作生成节点
会因此停止输出普通动作。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from center_server.core.reverse_handler import ReverseHandler
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import ReverseOperation


class ReverseOperationNode(Node):
    def __init__(self):
        super().__init__("reverse_operation_node")
        self.cfg = load_config(self)
        self.handler = ReverseHandler(self.cfg.get("reverse_operation", {}))

        self.pub_cmd_vel = self.create_publisher(Twist, "/reverse/cmd_vel", 10)
        self.pub_arm_traj = self.create_publisher(
            JointTrajectory, "/arm/trajectory", 10
        )
        self.create_subscription(
            ReverseOperation, "/reverse/operation", self.on_reverse, 10
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

        self.latest_joint = {}
        self._reverse_deadline = 0.0
        self._reverse_twist = None
        # 周期检查单调时钟，避免单次定时器被延迟/ROS 时钟跳变影响回退停止；
        # 回退期间按 10 Hz 重复发布，确保反向指令覆盖普通控制指令。
        self._steady_clock = create_steady_clock()
        self.create_timer(0.1, self._reverse_tick, clock=self._steady_clock)
        self.get_logger().info("reverse_operation_node started")

    def on_joint_state(self, msg: JointState):
        self.latest_joint = dict(zip(msg.name, msg.position))

    def _publish(self, twist, traj):
        self.pub_cmd_vel.publish(twist)
        if traj is not None and len(traj.points) > 0:
            self.pub_arm_traj.publish(traj)

    def _reverse_tick(self):
        if self._reverse_deadline <= 0.0:
            return
        now = time.monotonic()
        if now < self._reverse_deadline:
            if self._reverse_twist is not None:
                self.pub_cmd_vel.publish(self._reverse_twist)
            return
        self._reverse_deadline = 0.0
        self._reverse_twist = None
        twist, traj = self.handler.handle_stop(self.latest_joint)
        self._publish(twist, traj)
        self.get_logger().info("reverse move finished, stop command sent")

    def on_reverse(self, op: ReverseOperation):
        op_type = int(op.operation_type)
        reason = op.reason or "unspecified"
        self._reverse_deadline = 0.0
        self._reverse_twist = None

        if op_type == ReverseOperation.EMERGENCY_STOP:
            twist, traj = self.handler.handle_stop(self.latest_joint)
            self._publish(twist, traj)
            self.get_logger().warn(f"EMERGENCY_STOP ({reason})")
        elif op_type == ReverseOperation.REVERSE_MOVE:
            twist, traj = self.handler.handle_backward(
                self.latest_joint, op.distance
            )
            self._publish(twist, traj)
            self._reverse_twist = twist
            duration = self.handler.reverse_duration(op.distance)
            self._reverse_deadline = time.monotonic() + duration
            self.get_logger().warn(
                f"REVERSE_MOVE speed={-twist.linear.x:.2f} m/s duration="
                f"{duration:.2f} s ({reason})"
            )
        elif op_type == ReverseOperation.ARM_RESET:
            twist, traj = self.handler.handle_arm_reset()
            self._publish(twist, traj)
            self.get_logger().warn(f"ARM_RESET ({reason})")
        elif op_type == ReverseOperation.TASK_CANCEL:
            twist, traj = self.handler.handle_task_cancel(self.latest_joint)
            self._publish(twist, traj)
            self.get_logger().warn(f"TASK_CANCEL ({reason})")
        elif op_type == ReverseOperation.MANUAL_OVERRIDE:
            twist, traj = self.handler.handle_manual_override(self.latest_joint)
            self._publish(twist, traj)
            self.get_logger().warn(f"MANUAL_OVERRIDE ({reason})")
        else:
            twist, traj = self.handler.handle_stop(self.latest_joint)
            self._publish(twist, traj)
            self.get_logger().warn(
                f"unknown reverse operation_type={op_type}, fallback to stop"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ReverseOperationNode()
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
