"""vla_inference_node：多模态推理并发布动作意图。

输入：/center/synced_data /center/robot_state /task/request
输出：/center/action_intent /center/inference_result
"""

import math

import cv_bridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from center_server.core.data_preprocessor import MultiModalPreprocessor
from center_server.core.vla_engine import ActionIntentDTO, VLAEngine
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import (
    ActionIntent,
    InferenceResult,
    RobotState,
    SafetyState,
    SyncedSensorData,
    SystemStatus,
    TaskRequest,
)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class VLAInferenceNode(Node):
    def __init__(self):
        super().__init__("vla_inference_node")
        self.cfg = load_config(self)
        vla_cfg = self.cfg.get("vla_engine", {})

        self.confidence_threshold = float(
            vla_cfg.get("confidence_threshold", 0.6)
        )
        self.preprocessor = MultiModalPreprocessor()
        # 任务关键词/航点配置在 instruction 段；VLA 规则桩也需要它来识别长任务
        vla_cfg = dict(vla_cfg)
        vla_cfg.setdefault("instruction", self.cfg.get("instruction", {}))
        self.vla = VLAEngine(vla_cfg)
        self.cv_bridge = cv_bridge.CvBridge()

        self.pub_intent = self.create_publisher(
            ActionIntent, "/center/action_intent", 10
        )
        self.pub_result = self.create_publisher(
            InferenceResult, "/center/inference_result", 10
        )

        self.create_subscription(
            SyncedSensorData,
            "/center/synced_data",
            self.on_synced,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RobotState, "/center/robot_state", self.on_robot_state, 10
        )
        self.create_subscription(TaskRequest, "/task/request", self.on_task, 10)
        self.create_subscription(SystemStatus, "/center/status", self.on_status, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )

        self.latest_data = None
        self.latest_robot_state = None
        self.latest_task = None
        self.mode = "auto"
        self.task_state = "IDLE"
        self.emergency_stop = False
        self.data_ok = True
        self._last_gate = None

        rate = max(1.0, float(vla_cfg.get("inference_rate", 10)))
        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / rate, self.on_timer, clock=self._steady_clock
        )
        self.get_logger().info(f"vla_inference_node started ({rate:.1f} Hz)")

    # ------------------------------------------------------------- callbacks
    def on_synced(self, msg: SyncedSensorData):
        self.latest_data = msg

    def on_robot_state(self, msg: RobotState):
        self.latest_robot_state = msg

    def on_task(self, msg: TaskRequest):
        self.latest_task = msg
        self.get_logger().info(
            f"task received: id={msg.task_id!r}, "
            f"instruction={msg.language_instruction!r}"
        )

    def on_status(self, msg: SystemStatus):
        self.mode = msg.mode
        self.task_state = msg.state
        self.emergency_stop = bool(msg.emergency_stop)

    def on_safety(self, msg: SafetyState):
        self.emergency_stop = bool(msg.emergency_stop)
        self.data_ok = bool(msg.data_ok)

    def _gate_reason(self):
        if self.emergency_stop:
            return "emergency_stop"
        if self.mode != "auto":
            return f"mode={self.mode}"
        if not self.data_ok:
            return "sensor_data_timeout"
        if self.task_state not in ("RECEIVING_DATA", "INFERENCING", "EXECUTING"):
            return f"task_state={self.task_state}"
        if self.latest_task is None:
            return "waiting_for_task"
        return None

    def _publish(self, intent_dto: ActionIntentDTO, status: str, description: str):
        intent = ActionIntent()
        intent.task_id = intent_dto.task_id
        intent.target = intent_dto.target
        intent.intent = intent_dto.intent
        intent.value = float(intent_dto.value)
        intent.confidence = float(intent_dto.confidence)
        intent.reverse_request = bool(intent_dto.reverse_request)
        intent.task_type = intent_dto.task_type
        intent.object_label = intent_dto.object_label
        intent.goal_x = float(intent_dto.goal_x)
        intent.goal_y = float(intent_dto.goal_y)
        intent.goal_yaw = float(intent_dto.goal_yaw)
        intent.waypoint = intent_dto.waypoint
        self.pub_intent.publish(intent)

        result = InferenceResult()
        result.task_id = intent_dto.task_id
        result.action_type = intent_dto.intent
        result.confidence = float(intent_dto.confidence)
        result.status = status
        result.description = description
        self.pub_result.publish(result)

    # ---------------------------------------------------------------- main loop
    def on_timer(self):
        reason = self._gate_reason()
        if reason is not None:
            if reason != self._last_gate:
                self._last_gate = reason
                self.get_logger().info(f"inference paused: {reason}")
                self._publish(
                    ActionIntentDTO(
                        task_id=self.latest_task.task_id if self.latest_task else "",
                        intent="stop",
                        confidence=0.0,
                    ),
                    "paused",
                    reason,
                )
            return
        if self._last_gate is not None:
            self.get_logger().info("inference resumed")
            self._last_gate = None

        # 定时器与传感器频率可能交错触发，没有新帧时直接等待，不应视为故障
        if self.latest_data is None:
            return

        msg = self.latest_data
        # 只处理新帧，避免同一帧被重复推理
        self.latest_data = None
        if msg is None:
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg.image, desired_encoding="bgr8"
            )
            image = self.preprocessor.process_image(cv_image)
            laser = self.preprocessor.process_laser(msg.laser.ranges)
            audio = self.preprocessor.process_audio(
                msg.audio.data, sample_rate=msg.audio.sample_rate
            )
        except Exception as exc:  # noqa: BLE001 - 单帧异常不应导致节点退出
            self.get_logger().error(f"failed to preprocess sensor data: {exc}")
            self._publish(
                ActionIntentDTO(
                    task_id=self.latest_task.task_id, intent="stop", confidence=0.0
                ),
                "error",
                f"preprocess_error: {exc}",
            )
            return

        odom = msg.odom
        odom_data = {
            "x": odom.pose.pose.position.x,
            "y": odom.pose.pose.position.y,
            "yaw": _yaw_from_quaternion(odom.pose.pose.orientation),
        }
        joint_state = msg.joint_state
        if not joint_state.name and self.latest_robot_state is not None:
            joint_state = self.latest_robot_state.joint_state
        joint_data = dict(zip(joint_state.name, joint_state.position))
        task = self.latest_task
        instruction = (
            task.language_instruction.strip()
            or task.voice_instruction.strip()
            or ""
        )

        dto = self.preprocessor.pack(
            image,
            laser,
            audio,
            odom_data,
            joint_data,
            task_id=task.task_id,
            instruction=instruction,
        )
        intent_dto = self.vla.predict(dto)

        if intent_dto.confidence < self.confidence_threshold:
            # DESIGN.md 11：置信度低时停止执行并请求确认
            stop_dto = ActionIntentDTO(
                task_id=intent_dto.task_id,
                target=intent_dto.target,
                intent="stop",
                value=0.0,
                confidence=intent_dto.confidence,
            )
            self._publish(
                stop_dto,
                "needs_confirmation",
                f"low confidence {intent_dto.confidence:.2f} for "
                f"instruction {instruction!r}",
            )
            return

        self._publish(
            intent_dto,
            "running",
            f"intent={intent_dto.intent} instruction={instruction!r}",
        )
        self.get_logger().debug(
            f"intent={intent_dto.intent} value={intent_dto.value:.2f} "
            f"confidence={intent_dto.confidence:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VLAInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.vla.unload()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
