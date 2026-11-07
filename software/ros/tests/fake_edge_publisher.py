#!/usr/bin/env python3
"""模拟 edge_server 发布多模态传感器数据，用于 center_server 本地联调。

用法（先 source 工作空间 install/setup.bash）：
    python3 software/center_server/tests/fake_edge_publisher.py

运行中动态调整前方障碍距离：
    ros2 topic pub --once /sim/obstacle_distance std_msgs/msg/Float32 "{data: 0.2}"

发布话题：
    /edge/video  /edge/radar  /edge/audio  /robot/odometry  /robot/joint_states
"""

import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Float32

from center_server.utils.clock import create_steady_clock
from custom_msgs.msg import AudioData


class FakeEdgePublisher(Node):
    def __init__(self):
        super().__init__("fake_edge_publisher")
        self.obstacle_distance = 3.0
        self.joint_names = ["j1", "j2", "j3", "j4", "j5"]
        self.joint_positions = [0.0, 1.57, 0.0, 1.0, 0.5]
        self.start_time = time.monotonic()
        self.rate = 10.0
        # 用单调时钟生成递增时间戳，避免 WSL/虚拟机墙钟回拨影响 message_filters
        self._stamp_base_ns = self.get_clock().now().nanoseconds
        self._steady_base = time.monotonic()

        self.pub_image = self.create_publisher(
            Image, "/edge/video", qos_profile_sensor_data
        )
        self.pub_laser = self.create_publisher(
            LaserScan, "/edge/radar", qos_profile_sensor_data
        )
        self.pub_audio = self.create_publisher(
            AudioData, "/edge/audio", qos_profile_sensor_data
        )
        self.pub_odom = self.create_publisher(
            Odometry, "/robot/odometry", qos_profile_sensor_data
        )
        self.pub_joint = self.create_publisher(
            JointState, "/robot/joint_states", qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, "/sim/obstacle_distance", self.on_obstacle, 10
        )
        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / self.rate, self.publish_all, clock=self._steady_clock
        )
        self.get_logger().info(
            "fake_edge_publisher started; publish Float32 to "
            "/sim/obstacle_distance to simulate an obstacle"
        )

    def on_obstacle(self, msg: Float32):
        distance = float(msg.data)
        if not math.isfinite(distance) or distance <= 0.0:
            distance = 3.0
        self.obstacle_distance = distance
        self.get_logger().info(f"simulated front obstacle distance={distance:.2f} m")

    def _stamp(self):
        elapsed_ns = int((time.monotonic() - self._steady_base) * 1e9)
        total_ns = self._stamp_base_ns + elapsed_ns
        stamp = Time()
        stamp.sec = total_ns // 1_000_000_000
        stamp.nanosec = total_ns % 1_000_000_000
        return stamp

    def publish_all(self):
        stamp = self._stamp()

        # 图像：64x64 bgr8 渐变图，保证 cv_bridge 可解码
        width = height = 64
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :, 0] = np.arange(width, dtype=np.uint8)
        image[:, :, 1] = np.arange(height, dtype=np.uint8)[:, None]
        image[:, :, 2] = 128
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = width * 3
        img_msg.data = image.tobytes()
        self.pub_image.publish(img_msg)

        # 雷达：360 线，正前方 60 度扇区放障碍
        num_beams = 360
        ranges = np.full(num_beams, 3.0, dtype=np.float32)
        if self.obstacle_distance < 2.5:
            ranges[150:210] = self.obstacle_distance
        laser = LaserScan()
        laser.header.stamp = stamp
        laser.angle_min = -math.pi
        laser.angle_max = math.pi - 2.0 * math.pi / num_beams
        laser.angle_increment = 2.0 * math.pi / num_beams
        laser.time_increment = 0.0
        laser.scan_time = 1.0 / self.rate
        laser.range_min = 0.05
        laser.range_max = 10.0
        laser.ranges = ranges.tolist()
        laser.intensities = []
        self.pub_laser.publish(laser)

        # 音频：0.2 秒 440Hz PCM s16le
        sample_rate = 16000
        t = np.arange(int(sample_rate * 0.2)) / sample_rate
        samples = (np.sin(2.0 * math.pi * 440.0 * t) * 0.3 * 32767).astype(np.int16)
        audio = AudioData()
        audio.header.stamp = stamp
        audio.format = "pcm_s16le"
        audio.sample_rate = sample_rate
        audio.data = samples.tobytes()
        self.pub_audio.publish(audio)

        # 里程计：匀速前进的假数据
        elapsed = time.monotonic() - self.start_time
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.3 * elapsed
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.3
        self.pub_odom.publish(odom)

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(self.joint_names)
        joint.position = list(self.joint_positions)
        joint.velocity = [0.0] * len(self.joint_names)
        joint.effort = [0.0] * len(self.joint_names)
        self.pub_joint.publish(joint)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEdgePublisher()
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
