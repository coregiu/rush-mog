import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from audio_common_msgs.msg import AudioData
from custom_msgs.msg import SyncedSensorData

class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")
        self.pub_sync = self.create_publisher(SyncedSensorData, "/center/synced_data", 10)
        sub_img = Subscriber(self, Image, "/edge/video")
        sub_laser = Subscriber(self, LaserScan, "/edge/radar")
        sub_audio = Subscriber(self, AudioData, "/edge/audio")
        sub_odom = Subscriber(self, Odometry, "/robot/odometry")
        sub_joint = Subscriber(self, JointState, "/robot/joint_states")
        self.ts = TimeSynchronizer([sub_img, sub_laser, sub_audio, sub_odom, sub_joint], 5)
        self.ts.registerCallback(self.cb)

    def cb(self, img, laser, audio, odom, joint):
        msg = SyncedSensorData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.image = img
        msg.laser = laser
        msg.audio = audio
        msg.odom = odom
        msg.joint_state = joint
        self.pub_sync.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
