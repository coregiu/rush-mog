import rclpy
from rclpy.node import Node
from custom_msgs.msg import SyncedSensorData, ActionIntent, InferenceResult
from center_server.core.vla_engine import VLAEngine
from center_server.core.data_preprocessor import MultiModalPreprocessor
import cv_bridge
import yaml
import os

class VLAInferenceNode(Node):
    def __init__(self):
        super().__init__("vla_inference_node")
        self.pub_intent = self.create_publisher(ActionIntent, "/center/action_intent", 10)
        self.pub_result = self.create_publisher(InferenceResult, "/center/inference_result",10)
        self.sub_sync = self.create_subscription(SyncedSensorData, "/center/synced_data", self.cb,10)
        self.bridge = cv_bridge.CvBridge()
        with open(os.path.join("config","config.yaml"),"r") as f:
            cfg = yaml.safe_load(f)
        self.preprocessor = MultiModalPreprocessor()
        self.vla = VLAEngine(cfg["vla_engine"])

    def cb(self, msg: SyncedSensorData):
        cv_img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
        img_tensor = self.preprocessor.process_image(cv_img)
        laser_arr = self.preprocessor.process_laser(msg.laser.ranges)
        audio_tensor = self.preprocessor.process_audio(msg.audio.data)
        odom_data = {"x":msg.odom.pose.pose.position.x}
        joint_data = dict(zip(msg.joint_state.name, msg.joint_state.position))
        dto = self.preprocessor.pack(img_tensor, laser_arr, audio_tensor, odom_data, joint_data)
        intent_dto = self.vla.predict(dto)
        out = ActionIntent()
        out.task_id = intent_dto.task_id
        out.target = intent_dto.target
        out.intent = intent_dto.intent
        out.value = intent_dto.value
        out.confidence = intent_dto.confidence
        self.pub_intent.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = VLAInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
