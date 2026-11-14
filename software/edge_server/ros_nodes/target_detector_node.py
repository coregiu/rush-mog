"""target_detector_node：相机目标检测 -> 方位/距离 -> map 系目标列表。

- 默认颜色块检测（HSV），无模型依赖；可切换 YOLO（需 ultralytics）；
- 距离优先取目标方位附近的 2D 雷达距离，其次用类别先验高度 + bbox 高度；
- 用轻量 IoU 跟踪器给稳定 track_id，并投影到 map 系（依赖 /center/pose）；
- 发布 /center/target_detections（custom_msgs/TargetList）。
"""

from __future__ import annotations

import time

import rclpy
import cv_bridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from edge_server.core.object_detector import make_detector
from edge_server.core.pose_utils import pose_from_msg
from edge_server.core.target_localizer import TargetLocalizer
from edge_server.core.target_tracker import DetectionInput, TargetTracker
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.msg import TargetDetection, TargetList, TaskRequest


class TargetDetectorNode(Node):
    def __init__(self):
        super().__init__("target_detector_node")
        self.cfg = load_config(self)
        self.perception_cfg = self.cfg.get("perception", {})
        self.enabled = bool(self.perception_cfg.get("enabled", True))
        self.frame_id = str(self.perception_cfg.get("position_frame", "map"))
        self.publish_rate = max(1.0, float(self.perception_cfg.get("publish_rate", 10.0)))
        self.min_confidence = float(self.perception_cfg.get("min_confidence", 0.5))

        self.pub_targets = self.create_publisher(
            TargetList, "/center/target_detections", 10
        )
        self.detector = None
        self.localizer = None
        self.tracker = TargetTracker(
            iou_threshold=float(self.perception_cfg.get("iou_threshold", 0.25)),
            max_age_sec=float(self.perception_cfg.get("track_max_age_sec", 1.0)),
        )
        self.latest_scan: LaserScan | None = None
        self.latest_pose = None
        self.latest_pose_time = 0.0
        self.latest_task_id = ""
        self._latest_tracks = []
        self._last_stamp = None
        self._last_log = 0.0

        if not self.enabled:
            self.get_logger().warn("target_detector_node disabled by config")
            return

        self.detector = make_detector(self.perception_cfg, self.get_logger())
        if self.detector is None:
            self.get_logger().warn("no detector configured; target detection idle")
            return
        self.localizer = TargetLocalizer(
            camera_cfg=self.perception_cfg.get("camera", {}),
            targets_cfg=self.perception_cfg.get("targets", {}),
            lidar_window_deg=float(self.perception_cfg.get("lidar_window_deg", 6.0)),
            prefer_lidar=bool(self.perception_cfg.get("prefer_lidar", True)),
        )
        self.cv_bridge = cv_bridge.CvBridge()

        image_topic = str(self.perception_cfg.get("image_topic", "/camera/image_raw"))
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/edge/radar", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/center/pose", self.on_pose, 10
        )
        self.create_subscription(TaskRequest, "/task/request", self.on_task, 10)
        self._steady_clock = create_steady_clock()
        self.create_timer(1.0 / self.publish_rate, self.on_timer, clock=self._steady_clock)
        self.get_logger().info(
            f"target_detector_node started (image={image_topic}, "
            f"detector={self.perception_cfg.get('detector', 'color')})"
        )

    # ------------------------------------------------------------- callbacks
    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = pose_from_msg(msg)
        self.latest_pose_time = time.monotonic()

    def on_task(self, msg: TaskRequest):
        self.latest_task_id = msg.task_id

    def on_image(self, msg: Image):
        if self.detector is None or self.localizer is None:
            return
        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion failed: {exc}")
            return
        try:
            detections = self.detector.detect(image)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"detector failed: {exc}")
            return

        scan = None
        if self.latest_scan is not None:
            scan = {
                "ranges": self.latest_scan.ranges,
                "angle_min": self.latest_scan.angle_min,
                "angle_increment": self.latest_scan.angle_increment,
                "range_min": self.latest_scan.range_min,
                "range_max": self.latest_scan.range_max,
            }
        pose = (
            self.latest_pose
            if (time.monotonic() - self.latest_pose_time) <= 1.0
            else None
        )

        inputs = []
        for det in detections:
            if det.confidence < self.min_confidence:
                continue
            est = self.localizer.estimate(
                det.label, det.confidence, det.bbox, scan=scan, robot_pose=pose
            )
            inputs.append(
                DetectionInput(
                    label=det.label,
                    confidence=est.confidence,
                    bbox=det.bbox,
                    position=(est.x, est.y) if est.has_position else None,
                    position_frame=est.position_frame,
                    distance_m=est.distance_m,
                    bearing_rad=est.bearing_rad,
                    distance_source=est.distance_source,
                )
            )
        self._latest_tracks = self.tracker.update(inputs, time.monotonic())
        self._last_stamp = msg.header.stamp
        if inputs and time.monotonic() - self._last_log > 2.0:
            self._last_log = time.monotonic()
            best = max(inputs, key=lambda d: d.confidence)
            self.get_logger().info(
                f"target {best.label} conf={best.confidence:.2f} "
                f"d={best.distance_m:.2f}m ({best.distance_source}) "
                f"frame={best.position_frame}"
            )

    # ---------------------------------------------------------------- publish
    def on_timer(self):
        if self.detector is None:
            return
        msg = TargetList()
        msg.header.stamp = (
            self._last_stamp if self._last_stamp is not None else self.get_clock().now().to_msg()
        )
        msg.header.frame_id = self.frame_id
        msg.task_id = self.latest_task_id
        for track in self._latest_tracks:
            if track.confidence < self.min_confidence:
                continue
            item = TargetDetection()
            item.header = msg.header
            item.task_id = self.latest_task_id
            item.label = track.label
            item.track_id = track.track_id
            item.confidence = float(track.confidence)
            item.x_min, item.y_min, item.x_max, item.y_max = (float(v) for v in track.bbox)
            item.bearing_rad = float(track.bearing_rad)
            item.distance_m = float(track.distance_m)
            item.distance_source = track.distance_source
            if track.position is not None:
                item.position.x = float(track.position[0])
                item.position.y = float(track.position[1])
                item.position_frame = track.position_frame or self.frame_id
                var = 0.05 ** 2 if track.distance_source == "lidar" else 0.20 ** 2
                item.position_variance.x = var
                item.position_variance.y = var
            else:
                item.position_frame = ""
            msg.detections.append(item)
        self.pub_targets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectorNode()
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
