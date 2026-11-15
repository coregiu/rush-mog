"""voice_scenario_node：场景编排层（L3）的 ROS 适配节点。

职责很薄：把纯逻辑 :class:`FindObjectScenario` 产生的效果翻译成 ROS 调用：

- ``speak``         -> 发布 ``/body/say``；
- ``start_search``  -> 调用 ``/center/search_object`` Action；
- ``cancel_search`` -> 取消当前 Action goal；
- 状态变化          -> 发布 ``/center/scenario_status``（JSON）。

业务的“听懂什么、什么时候播报、失败说什么”全部在
``edge_server/core/scenarios/find_object.py``，本节点不写业务判断。
"""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from edge_server.core.instruction_parser import InstructionParser
from edge_server.core.pose_utils import Pose2D, pose_from_msg
from edge_server.core.scenarios import (
    FindObjectConfig,
    FindObjectScenario,
    ScenarioEffect,
    VoiceIntentResolver,
)
from edge_server.utils.config import load_config
from custom_msgs.action import SearchObject
from custom_msgs.msg import SafetyState, SearchStatus


class VoiceScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_scenario_node")
        self.cfg = load_config(self)
        scenario_cfg = dict(self.cfg.get("scenario") or {})

        self.voice_topic = str(
            scenario_cfg.get("voice_topic", "/edge/voice_command")
        )
        self.search_action = str(
            scenario_cfg.get("search_action", "/center/search_object")
        )
        self.say_topic = str(scenario_cfg.get("say_topic", "/body/say"))
        self.status_topic = str(
            scenario_cfg.get("status_topic", "/center/scenario_status")
        )
        self.status_rate = max(0.2, float(scenario_cfg.get("status_rate_hz", 1.0)))
        self.max_duration = max(
            10.0, float(scenario_cfg.get("max_duration_sec", 300.0))
        )

        parser = InstructionParser(self.cfg.get("instruction") or {})
        resolver = VoiceIntentResolver(parser)
        scenario_conf = FindObjectConfig(
            max_duration_sec=self.max_duration,
            min_confidence=float(scenario_cfg.get("min_confidence", 0.5)),
            standoff_distance=float(scenario_cfg.get("standoff_distance", 0.6)),
            explore_if_not_found=bool(
                scenario_cfg.get("explore_if_not_found", True)
            ),
            save_map=bool(scenario_cfg.get("save_map", False)),
            announce_start=bool(scenario_cfg.get("announce_start", True)),
            announce_milestones=bool(
                scenario_cfg.get("announce_milestones", True)
            ),
            announce_progress_sec=max(
                0.0, float(scenario_cfg.get("announce_progress_sec", 30.0))
            ),
        )
        self.scenario = FindObjectScenario(scenario_conf, resolver)

        self._lock = threading.RLock()
        self._generation = 0
        self._goal_handle = None
        self._result_future = None
        self._search_active = False
        self._emergency = False
        self._last_pose: Pose2D | None = None
        self._last_pose_at = 0.0
        self._last_status_at = 0.0
        self._last_published_state = ""
        self._shutdown = threading.Event()

        self._cb_group = ReentrantCallbackGroup()
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_say = self.create_publisher(String, self.say_topic, 10)
        self.pub_status = self.create_publisher(
            String, self.status_topic, status_qos
        )
        self.create_subscription(
            String, self.voice_topic, self.on_voice_command, 10
        )
        self.create_subscription(
            SearchStatus, "/center/search_status", self.on_search_status, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/center/pose", self.on_pose, 10
        )
        # 用默认 QoS 订阅 safety：真实 safety_manager 是 TRANSIENT_LOCAL，
        # fake/第三方可能是 VOLATILE；VOLATILE 订阅与两者都兼容。
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.search_client = ActionClient(
            self,
            SearchObject,
            self.search_action,
            callback_group=self._cb_group,
        )
        self.create_timer(1.0 / self.status_rate, self.on_status_timer)
        self.get_logger().info(
            f"voice_scenario_node started (voice={self.voice_topic}, "
            f"search={self.search_action}, say={self.say_topic})"
        )

    # ------------------------------------------------------------- callbacks
    def on_voice_command(self, msg: String) -> None:
        text = str(msg.data or "").strip()
        if not text:
            return
        now = time.monotonic()
        with self._lock:
            if self._emergency:
                self._speak("急停中，暂时不能寻找。")
                return
            effects = self.scenario.handle_utterance(text, now)
        self._apply_effects(effects)
        self._publish_status(force=True)

    def on_search_status(self, msg: SearchStatus) -> None:
        with self._lock:
            if not self.scenario.active:
                return
            if (
                msg.target_label
                and self.scenario.target_label
                and msg.target_label != self.scenario.target_label
            ):
                return
            effects = self.scenario.handle_search_feedback(
                str(msg.state or ""),
                detail=str(msg.detail or ""),
                explored_ratio=float(msg.explored_ratio),
                confidence=float(msg.best_confidence),
                now=time.monotonic(),
            )
        self._apply_effects(effects)
        self._publish_status()

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        try:
            pose = pose_from_msg(msg)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._last_pose = pose
            self._last_pose_at = time.monotonic()

    def on_safety(self, msg: SafetyState) -> None:
        emergency = bool(msg.emergency_stop)
        effects: list[ScenarioEffect] = []
        with self._lock:
            was_emergency = self._emergency
            self._emergency = emergency
            if emergency and not was_emergency:
                effects = self.scenario.cancel(
                    "emergency stop",
                    time.monotonic(),
                    speech="已急停，停止寻找。",
                )
                self._generation += 1
                self._cancel_active_locked()
        if effects:
            self._apply_effects(effects)
            self._publish_status(force=True)
        elif emergency and not was_emergency:
            self._cancel_search("emergency stop")

    def on_status_timer(self) -> None:
        self._publish_status()
        effects: list[ScenarioEffect] = []
        with self._lock:
            if (
                self.scenario.active
                and self.scenario.started_at > 0.0
                and time.monotonic() - self.scenario.started_at > self.max_duration + 5.0
            ):
                effects = self.scenario.handle_search_result(
                    False,
                    "search timeout",
                    now=time.monotonic(),
                )
                self._generation += 1
                self._cancel_active_locked()
        if effects:
            self._apply_effects(effects)
            self._publish_status(force=True)

    # -------------------------------------------------------------- effects
    def _apply_effects(self, effects: list[ScenarioEffect]) -> None:
        for effect in effects:
            if effect.kind == "speak":
                self._speak(effect.text)
            elif effect.kind == "start_search":
                self._start_search(effect.target_label)
            elif effect.kind == "cancel_search":
                self._cancel_search(effect.reason)

    def _speak(self, text: str) -> None:
        message = str(text or "").strip()
        if not message:
            return
        try:
            self.pub_say.publish(String(data=message))
            self.get_logger().info(f"say: {message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"publish say failed: {exc}")

    def _start_search(self, target_label: str) -> None:
        label = str(target_label or "").strip()
        if not label:
            return
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._cancel_active_locked()
            self._search_active = True
        worker = threading.Thread(
            target=self._wait_and_send_goal,
            args=(generation, label),
            name="voice-search-goal",
            daemon=True,
        )
        worker.start()

    def _wait_and_send_goal(self, generation: int, target_label: str) -> None:
        try:
            if not self.search_client.wait_for_server(timeout_sec=2.0):
                self._on_start_failed(generation, "search action server unavailable")
                return
            with self._lock:
                if generation != self._generation or self._shutdown.is_set():
                    return
            goal = SearchObject.Goal()
            goal.task_id = f"voice_{int(time.time() * 1000)}"
            goal.target_label = target_label
            goal.min_confidence = float(self.scenario.cfg.min_confidence)
            goal.max_duration = float(self.max_duration)
            goal.standoff_distance = float(self.scenario.cfg.standoff_distance)
            goal.explore_if_not_found = bool(
                self.scenario.cfg.explore_if_not_found
            )
            goal.save_map = bool(self.scenario.cfg.save_map)
            future = self.search_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done: self._on_goal_response(done, generation)
            )
        except Exception as exc:  # noqa: BLE001
            self._on_start_failed(generation, str(exc))

    def _on_goal_response(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._on_start_failed(generation, str(exc))
            return
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self._on_start_failed(generation, "search goal rejected")
            return
        with self._lock:
            if generation != self._generation:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return
            self._goal_handle = goal_handle
            self._result_future = goal_handle.get_result_async()
            self._result_future.add_done_callback(
                lambda done: self._on_search_result(done, generation)
            )
            effects = self.scenario.handle_search_started(time.monotonic())
        self._apply_effects(effects)
        self._publish_status(force=True)

    def _on_start_failed(self, generation: int, reason: str) -> None:
        with self._lock:
            if generation != self._generation:
                return
            self._search_active = False
            self._goal_handle = None
            self._result_future = None
            effects = self.scenario.handle_search_start_failed(
                reason, time.monotonic()
            )
        self._apply_effects(effects)
        self._publish_status(force=True)

    def _on_search_result(self, future, generation: int) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            status = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"search result failed: {exc}")
            result = None
            status = GoalStatus.STATUS_UNKNOWN
        now = time.monotonic()
        with self._lock:
            if generation != self._generation:
                return
            self._search_active = False
            self._goal_handle = None
            self._result_future = None
            if result is not None and status == GoalStatus.STATUS_SUCCEEDED:
                found = bool(getattr(result, "found", False))
                distance = self._target_distance(result) if found else None
            else:
                found = False
                distance = None
            info = str(getattr(result, "info", "") or "")
            if not info:
                info = "search succeeded" if found else "search failed"
            effects = self.scenario.handle_search_result(
                found,
                info=info,
                confidence=float(getattr(result, "confidence", 0.0) or 0.0),
                distance_m=distance,
                now=now,
            )
        self._apply_effects(effects)
        self._publish_status(force=True)

    def _cancel_search(self, reason: str) -> None:
        with self._lock:
            self._generation += 1
            handle = self._goal_handle
            self._cancel_active_locked()
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        if reason:
            self.get_logger().info(f"search canceled: {reason}")

    def _cancel_active_locked(self) -> None:
        handle = self._goal_handle
        self._goal_handle = None
        self._result_future = None
        self._search_active = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # -------------------------------------------------------------- helpers
    def _target_distance(self, result) -> float | None:
        target_pose = getattr(result, "target_pose", None)
        if target_pose is None or not getattr(target_pose, "header", None):
            return None
        if not str(target_pose.header.frame_id or ""):
            return None
        try:
            target = pose_from_msg(target_pose)
        except Exception:  # noqa: BLE001
            return None
        with self._lock:
            pose = self._last_pose
            pose_at = self._last_pose_at
        if pose is None or (time.monotonic() - pose_at) > 3.0:
            return None
        distance = math.hypot(target.x - pose.x, target.y - pose.y)
        if not math.isfinite(distance) or distance > 20.0:
            return None
        return distance

    def _publish_status(self, force: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            if (
                not force
                and now - self._last_status_at < (1.0 / self.status_rate) * 0.9
                and self.scenario.state.value == self._last_published_state
            ):
                return
            self._last_status_at = now
            self._last_published_state = self.scenario.state.value
            payload = self.scenario.status(now)
            payload["ok"] = True
            payload["emergency_stop"] = bool(self._emergency)
            payload["search_active"] = bool(self._search_active)
            payload["updated_ms"] = int(time.time() * 1000)
        try:
            self.pub_status.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"publish scenario status failed: {exc}")

    def shutdown(self) -> None:
        self._shutdown.set()
        with self._lock:
            self._generation += 1
            self._cancel_active_locked()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceScenarioNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
