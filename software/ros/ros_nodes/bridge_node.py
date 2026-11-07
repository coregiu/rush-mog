"""center_bridge_node：接收多模态数据并按图像帧做软同步。

v2 调整：
- 只对 image / laser / odom 三路做 ApproximateTimeSynchronizer；
- audio / joint_states / imu 采用“最近值缓存”，缺失时填空消息；
  这样真实机器人没有麦克风时不会阻塞整条感知链路；
- 话题可通过 sensor_topics 配置切换（仿真 /edge/radar，实车 /scan）。
"""

import rclpy
from message_filters import Subscriber
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, LaserScan

from center_server.utils.config import load_config
from center_server.utils.time_sync import make_approximate_synchronizer
from custom_msgs.msg import AudioData, RobotState, SyncedSensorData


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")
        self.cfg = load_config(self)
        sync_cfg = self.cfg.get("time_sync", {})
        topics = self.cfg.get("sensor_topics", {})
        slop = float(sync_cfg.get("sync_window_ms", 100)) / 1000.0
        queue_size = int(sync_cfg.get("queue_size", 10))

        self.image_topic = str(topics.get("image", "/edge/video"))
        self.laser_topic = str(topics.get("laser", "/edge/radar"))
        self.audio_topic = str(topics.get("audio", "/edge/audio"))
        self.odom_topic = str(topics.get("odom", "/robot/odometry"))
        self.joint_topic = str(topics.get("joint_states", "/robot/joint_states"))
        self.imu_topic = str(topics.get("imu", "/imu/data"))

        self.pub_sync = self.create_publisher(
            SyncedSensorData, "/center/synced_data", qos_profile_sensor_data
        )
        self.pub_state = self.create_publisher(RobotState, "/center/robot_state", 10)

        sub_img = Subscriber(
            self, Image, self.image_topic, qos_profile=qos_profile_sensor_data
        )
        sub_laser = Subscriber(
            self, LaserScan, self.laser_topic, qos_profile=qos_profile_sensor_data
        )
        sub_odom = Subscriber(
            self, Odometry, self.odom_topic, qos_profile=qos_profile_sensor_data
        )
        self.create_subscription(
            AudioData, self.audio_topic, self.on_audio, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, self.joint_topic, self.on_joint, qos_profile_sensor_data
        )
        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)

        self.sync = make_approximate_synchronizer(
            [sub_img, sub_laser, sub_odom],
            queue_size=queue_size,
            slop=slop,
        )
        self.sync.registerCallback(self.on_synced)
        self.latest_audio = None
        self.latest_joint = None
        self.latest_imu = None
        self._got_first = False
        self.get_logger().info(
            f"bridge_node started (slop={slop * 1000:.0f} ms, queue={queue_size}, "
            f"image={self.image_topic}, laser={self.laser_topic}, odom={self.odom_topic})"
        )

    # ------------------------------------------------------------- callbacks
    def on_audio(self, msg: AudioData):
        self.latest_audio = msg

    def on_joint(self, msg: JointState):
        self.latest_joint = msg

    def on_imu(self, msg: Imu):
        self.latest_imu = msg

    def on_synced(self, image, laser, odom):
        stamp = image.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        synced = SyncedSensorData()
        synced.header.stamp = stamp
        synced.header.frame_id = image.header.frame_id or "camera_link"
        synced.image = image
        synced.laser = laser
        synced.odom = odom
        synced.audio = self.latest_audio if self.latest_audio is not None else AudioData()
        synced.joint_state = (
            self.latest_joint if self.latest_joint is not None else JointState()
        )
        synced.imu = self.latest_imu if self.latest_imu is not None else Imu()
        if not synced.audio.header.stamp.sec and not synced.audio.header.stamp.nanosec:
            synced.audio.header.stamp = stamp
        if not synced.joint_state.header.stamp.sec and not synced.joint_state.header.stamp.nanosec:
            synced.joint_state.header.stamp = stamp
        if not synced.imu.header.stamp.sec and not synced.imu.header.stamp.nanosec:
            synced.imu.header.stamp = stamp
        self.pub_sync.publish(synced)

        state = RobotState()
        state.header.stamp = stamp
        state.odom = odom
        state.joint_state = synced.joint_state
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
