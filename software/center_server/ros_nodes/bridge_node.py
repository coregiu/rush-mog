"""center_bridge_node：接收 edge_server 多模态数据并做时间同步。

订阅：/edge/video /edge/radar /edge/audio /robot/odometry /robot/joint_states
发布：/center/synced_data /center/robot_state
"""

import rclpy
from message_filters import Subscriber
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan

from center_server.utils.config import load_config
from center_server.utils.time_sync import make_approximate_synchronizer
from custom_msgs.msg import AudioData, RobotState, SyncedSensorData


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")
        self.cfg = load_config(self)
        sync_cfg = self.cfg.get("time_sync", {})
        slop = float(sync_cfg.get("sync_window_ms", 100)) / 1000.0
        queue_size = int(sync_cfg.get("queue_size", 10))

        self.pub_sync = self.create_publisher(
            SyncedSensorData, "/center/synced_data", qos_profile_sensor_data
        )
        self.pub_state = self.create_publisher(
            RobotState, "/center/robot_state", 10
        )

        # 传感器流使用 SensorDataQoS，避免与真实驱动（通常 best effort）QoS 不兼容
        sub_img = Subscriber(
            self, Image, "/edge/video", qos_profile=qos_profile_sensor_data
        )
        sub_laser = Subscriber(
            self, LaserScan, "/edge/radar", qos_profile=qos_profile_sensor_data
        )
        sub_audio = Subscriber(
            self, AudioData, "/edge/audio", qos_profile=qos_profile_sensor_data
        )
        sub_odom = Subscriber(
            self, Odometry, "/robot/odometry", qos_profile=qos_profile_sensor_data
        )
        sub_joint = Subscriber(
            self, JointState, "/robot/joint_states", qos_profile=qos_profile_sensor_data
        )

        self.sync = make_approximate_synchronizer(
            [sub_img, sub_laser, sub_audio, sub_odom, sub_joint],
            queue_size=queue_size,
            slop=slop,
        )
        self.sync.registerCallback(self.on_synced)
        self._got_first = False
        self.get_logger().info(
            f"bridge_node started (slop={slop * 1000:.0f} ms, queue={queue_size})"
        )

    def on_synced(self, image, laser, audio, odom, joint):
        stamp = self.get_clock().now().to_msg()

        synced = SyncedSensorData()
        synced.header.stamp = stamp
        synced.image = image
        synced.laser = laser
        synced.audio = audio
        synced.odom = odom
        synced.joint_state = joint
        self.pub_sync.publish(synced)

        state = RobotState()
        state.header.stamp = stamp
        state.odom = odom
        state.joint_state = joint
        self.pub_state.publish(state)

        if not self._got_first:
            self._got_first = True
            self.get_logger().info("first synchronized sensor frame received")


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
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
