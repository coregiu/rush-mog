"""task_manager_node：任务状态机、VLA 单步任务与 SLAM/导航/搜索长任务。

服务：
    /center/set_mode              auto / manual / reverse
Action：
    /center/execute_task          VLA 单步任务（兼容 v1）；指令若命中建图/搜索/
                                  导航/回家关键词，会自动转成长任务
    /center/execute_mission       map / explore / search / navigate / return_home
状态：
    /center/status                系统状态机
    /center/mission_status        长任务进度（安全模块据此门控定位）
"""

from __future__ import annotations

import dataclasses
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from center_server.core.instruction_parser import InstructionParser, MissionSpec
from center_server.core.pose_utils import Pose2D, make_pose_stamped
from center_server.core.state_machine import State, TaskStateMachine, mission_entry_event
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.action import (
    ExecuteMission,
    ExecuteTask,
    Explore,
    NavigateTo,
    SearchObject,
)
from custom_msgs.msg import (
    InferenceResult,
    LocalizationStatus,
    MapInfo,
    MissionStatus,
    ReverseOperation,
    SafetyState,
    SystemStatus,
    TaskRequest,
)
from custom_msgs.srv import MapControl, SetMode

_MISSION_TYPES = {"map", "explore", "search", "navigate", "return_home", "vla"}


@dataclass
class MissionFeedback:
    state: str = ""
    detail: str = ""
    progress: float = 0.0
    explored_ratio: float = 0.0
    distance_remaining: float = 0.0
    best_pose: Optional[Pose2D] = None
    best_confidence: float = 0.0


@dataclass
class MissionOutcome:
    success: bool
    info: str
    target_pose: Optional[Pose2D] = None
    approach_pose: Optional[Pose2D] = None
    confidence: float = 0.0
    elapsed_sec: float = 0.0


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")
        self.cfg = load_config(self)
        tm_cfg = self.cfg.get("task_manager", {})
        mission_cfg = self.cfg.get("mission", {})
        self.task_timeout = float(tm_cfg.get("task_timeout_sec", 30.0))
        self.reverse_timeout = float(tm_cfg.get("reverse_timeout_sec", 5.0))
        self.status_rate = max(0.5, float(tm_cfg.get("status_rate", 2.0)))
        self.execution_hold = max(
            0.0, float(tm_cfg.get("execution_hold_sec", 1.0))
        )
        self.default_mission_timeout = float(
            mission_cfg.get("default_timeout_sec", 300.0)
        )
        self.map_timeout = float(mission_cfg.get("map_timeout_sec", 300.0))
        self.map_coverage = float(mission_cfg.get("map_target_coverage", 0.90))
        self.save_map_after_mapping = bool(
            mission_cfg.get("save_map_after_mapping", True)
        )
        self.instruction_parser = InstructionParser(self.cfg.get("instruction", {}))

        self.sm = TaskStateMachine()
        self.mode = "auto"
        self.emergency_stop = False
        self.data_ok = True
        self.localization_ok = True
        self.latest_task = None
        self.last_result = None
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.executed_cmd = None
        self.active_task_id = ""
        self.detail = "idle"
        self.reverse_until = 0.0
        self.map_info: MapInfo | None = None
        self.localization_status: LocalizationStatus | None = None

        self._executing = False
        self._exec_lock = threading.Lock()
        self._mission_active = False
        self._mission_lock = threading.Lock()
        self._cancel_requested = False
        self._active_subgoal = None
        self._active_client_name = ""
        self._last_feedback = MissionFeedback()
        self._last_emit = 0.0
        self._mission_id = ""
        self._mission_type = ""

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cb_group = ReentrantCallbackGroup()
        self.pub_task = self.create_publisher(TaskRequest, "/task/request", 10)
        self.pub_status = self.create_publisher(
            SystemStatus, "/center/status", status_qos
        )
        self.pub_mission = self.create_publisher(
            MissionStatus, "/center/mission_status", 10
        )

        self.create_subscription(
            TaskRequest, "/task/request", self.on_task_request, 10
        )
        self.create_subscription(
            TaskRequest, "/center/mission_request", self.on_mission_request, 10
        )
        self.create_subscription(
            InferenceResult, "/center/inference_result", self.on_inference, 10
        )
        self.create_subscription(
            ReverseOperation, "/reverse/operation", self.on_reverse, 10
        )
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, status_qos
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(MapInfo, "/center/map_info", self.on_map_info, 10)
        self.create_subscription(
            LocalizationStatus,
            "/center/localization_status",
            self.on_localization,
            10,
        )
        self.create_service(SetMode, "/center/set_mode", self.on_set_mode)

        self.nav_client = ActionClient(
            self,
            NavigateTo,
            "/center/navigate_to",
            callback_group=self.cb_group,
        )
        self.explore_client = ActionClient(
            self,
            Explore,
            "/center/explore",
            callback_group=self.cb_group,
        )
        self.search_client = ActionClient(
            self,
            SearchObject,
            "/center/search_object",
            callback_group=self.cb_group,
        )
        self.map_client = self.create_client(
            MapControl, "/center/map_control", callback_group=self.cb_group
        )

        self.action_server = ActionServer(
            self,
            ExecuteTask,
            "/center/execute_task",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )
        self.mission_server = ActionServer(
            self,
            ExecuteMission,
            "/center/execute_mission",
            execute_callback=self.execute_mission_cb,
            goal_callback=self.mission_goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )

        self._steady_clock = create_steady_clock()
        self.create_timer(
            1.0 / self.status_rate,
            self.on_status_timer,
            clock=self._steady_clock,
        )
        self.get_logger().info(
            f"task_manager_node started (task_timeout={self.task_timeout:.1f}s, "
            f"mission_timeout={self.default_mission_timeout:.0f}s)"
        )

    # ----------------------------------------------------------------- status
    def publish_status(self):
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.sm.state_name
        msg.mode = self.mode
        msg.emergency_stop = bool(self.emergency_stop)
        msg.detail = self.detail
        self.pub_status.publish(msg)

    def publish_mission_status(self, feedback: Optional[MissionFeedback] = None):
        fb = feedback or self._last_feedback
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self._mission_id
        msg.mission_type = self._mission_type
        msg.state = self.sm.state_name
        msg.detail = fb.detail or self.detail
        msg.progress = float(fb.progress)
        msg.explored_ratio = float(fb.explored_ratio)
        msg.distance_remaining = float(fb.distance_remaining)
        msg.best_confidence = float(fb.best_confidence)
        if fb.best_pose is not None:
            msg.best_pose = make_pose_stamped(
                fb.best_pose.frame_id or "map",
                fb.best_pose,
                self.get_clock().now().to_msg(),
            )
        self.pub_mission.publish(msg)

    def on_status_timer(self):
        if self.sm.state == State.REVERSING and time.monotonic() > self.reverse_until:
            if self.sm.transition("reverse_done"):
                self.detail = "reverse operation finished"
                self.get_logger().info("reverse operation finished")
        self.publish_status()

    # ------------------------------------------------------------- callbacks
    def on_task_request(self, msg: TaskRequest):
        if self.sm.state in (State.IDLE, State.TASK_SUCCESS, State.TASK_FAILED):
            self.latest_task = msg
            self.active_task_id = msg.task_id
            self.last_result = None
            if self.sm.transition("recv_data"):
                self.detail = f"task received: {msg.task_id}"
                self.publish_status()
        elif msg.task_id == self.active_task_id:
            self.latest_task = msg
        else:
            self.get_logger().warn(
                f"ignore new task {msg.task_id!r} while busy in "
                f"{self.sm.state_name}"
            )

    def on_mission_request(self, msg: TaskRequest):
        if self.mode != "auto" or self.emergency_stop or self._busy():
            self.get_logger().warn("ignore mission request while busy/not auto")
            return
        spec = self._spec_from_task_request(msg)
        if not spec.is_mission:
            self.get_logger().warn(
                f"mission request {msg.task_id!r} is not a mission: {msg.task_type!r}"
            )
            return
        thread = threading.Thread(
            target=self._background_mission, args=(spec,), daemon=True
        )
        thread.start()

    def on_map_info(self, msg: MapInfo):
        self.map_info = msg

    def on_localization(self, msg: LocalizationStatus):
        self.localization_status = msg
        self.localization_ok = bool(msg.healthy)

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_inference(self, msg: InferenceResult):
        if self.active_task_id and msg.task_id and msg.task_id != self.active_task_id:
            return
        if self.sm.state in (State.IDLE, State.TASK_SUCCESS, State.TASK_FAILED):
            return
        if self.sm.state.is_mission:
            return
        self.last_result = msg

        if msg.status == "running":
            if self.sm.state == State.RECEIVING_DATA:
                self.sm.transition("start_infer")
            if self.sm.state == State.INFERENCING:
                self.sm.transition("infer_ok")
            self.detail = f"executing intent={msg.action_type}"
        elif msg.status == "needs_confirmation":
            if self.sm.state == State.RECEIVING_DATA:
                self.sm.transition("start_infer")
            if self.sm.state == State.INFERENCING:
                self.sm.transition("infer_low_conf")
            self.detail = f"needs confirmation: {msg.description}"
        elif msg.status == "error":
            if not self.sm.transition("error"):
                self.sm.force(State.ERROR)
            self.detail = f"inference error: {msg.description}"
        elif msg.status == "paused":
            self.detail = f"inference paused: {msg.description}"
        self.publish_status()

    def on_reverse(self, op: ReverseOperation):
        op_type = int(op.operation_type)
        self.reverse_until = time.monotonic() + self.reverse_timeout
        self._cancel_requested = True
        self._cancel_active_subgoal()

        if op_type == ReverseOperation.TASK_CANCEL:
            if not self.sm.transition("task_cancel"):
                self.sm.force(State.IDLE)
            self.detail = f"task canceled: {op.reason}"
        elif op_type == ReverseOperation.MANUAL_OVERRIDE:
            self.mode = "manual"
            self.sm.transition("reverse_req")
            self.detail = f"manual override: {op.reason}"
        else:
            if not self.sm.transition("reverse_req"):
                self.sm.force(State.REVERSING)
            self.detail = f"reverse operation {op_type}: {op.reason}"

        self.get_logger().warn(f"reverse operation: {self.detail}")
        self.publish_status()

    def on_safety(self, msg: SafetyState):
        was_estop = self.emergency_stop
        self.emergency_stop = bool(msg.emergency_stop)
        self.data_ok = bool(msg.data_ok)
        self.localization_ok = bool(msg.localization_ok)

        if self.emergency_stop:
            self._cancel_requested = True
            self._cancel_active_subgoal()
        if self.emergency_stop and not was_estop:
            if self.sm.state != State.REVERSING:
                self.sm.force(State.ERROR)
            self.detail = f"emergency stop: {msg.reason}"
        elif was_estop and not self.emergency_stop:
            if self.sm.state == State.ERROR:
                self.sm.transition("reset")
            self.detail = "emergency stop cleared"

        if not self.data_ok and self.sm.state in (
            State.RECEIVING_DATA,
            State.INFERENCING,
            State.EXECUTING,
            State.MAPPING,
            State.NAVIGATING,
            State.SEARCHING,
        ):
            self.sm.force(State.IDLE)
            self.detail = "sensor data timeout"

        self.publish_status()

    def on_set_mode(self, request, response):
        mode = (request.mode or "").strip().lower()
        if mode not in ("auto", "manual", "reverse"):
            response.success = False
            response.msg = "mode must be one of: auto, manual, reverse"
            return response

        if mode != "auto":
            self._cancel_requested = True
            self._cancel_active_subgoal()
            self.sm.transition("pause")
        elif self.sm.state == State.PAUSED:
            self.sm.transition("resume")

        self.mode = mode
        self.detail = f"mode set to {mode}"
        self.get_logger().info(self.detail)
        self.publish_status()
        response.success = True
        response.msg = self.detail
        return response

    # ---------------------------------------------------------------- actions
    def _busy(self) -> bool:
        if self._mission_active:
            return True
        with self._exec_lock:
            return self._executing

    def goal_cb(self, goal_request):
        if self.mode != "auto":
            return GoalResponse.REJECT
        if self.emergency_stop or not self.data_ok:
            return GoalResponse.REJECT
        if not (goal_request.instruction or "").strip() and not goal_request.task_type:
            return GoalResponse.REJECT
        if self.sm.state not in (
            State.IDLE,
            State.TASK_SUCCESS,
            State.TASK_FAILED,
            State.PAUSED,
            State.ERROR,
        ):
            return GoalResponse.REJECT
        if self._busy():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def mission_goal_cb(self, goal_request):
        mission_type = self._normalize_mission_type(goal_request.mission_type)
        if mission_type == "vla":
            return GoalResponse.REJECT
        if self.mode != "auto" or self.emergency_stop or not self.data_ok:
            return GoalResponse.REJECT
        if self.sm.state not in (
            State.IDLE,
            State.TASK_SUCCESS,
            State.TASK_FAILED,
            State.PAUSED,
            State.ERROR,
        ):
            return GoalResponse.REJECT
        if self._busy():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self._cancel_requested = True
        self._cancel_active_subgoal()
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        with self._exec_lock:
            self._executing = True
        try:
            request = goal_handle.request
            spec = self._spec_from_execute_task(request)
            if spec.is_mission:
                outcome = self._run_mission(spec, goal_handle, kind="task")
                return self._task_result(outcome)
            return self._execute_vla(goal_handle)
        finally:
            with self._exec_lock:
                self._executing = False

    def execute_mission_cb(self, goal_handle):
        with self._exec_lock:
            self._executing = True
        try:
            spec = self._spec_from_execute_mission(goal_handle.request)
            outcome = self._run_mission(spec, goal_handle, kind="mission")
            return self._mission_result(outcome)
        finally:
            with self._exec_lock:
                self._executing = False

    # --------------------------------------------------------------- missions
    def _run_mission(self, spec: MissionSpec, goal_handle, kind: str) -> MissionOutcome:
        with self._mission_lock:
            if self._mission_active:
                if goal_handle is not None:
                    goal_handle.abort()
                return MissionOutcome(False, "another mission is active")
            self._mission_active = True
        self._cancel_requested = False
        self._last_feedback = MissionFeedback()
        self._mission_id = f"mission_{int(time.time() * 1000)}"
        self._mission_type = spec.mission_type
        self.active_task_id = self._mission_id
        self.detail = f"{spec.mission_type} mission started"
        started = time.monotonic()

        event = mission_entry_event(spec.mission_type)
        if not self.sm.transition(event):
            self.sm.force(
                {
                    "map": State.MAPPING,
                    "search": State.SEARCHING,
                    "explore": State.SEARCHING,
                }.get(spec.mission_type, State.NAVIGATING)
            )
        self.publish_status()
        self.publish_mission_status()

        try:
            if spec.mission_type == "map":
                outcome = self._call_map(spec, goal_handle, kind)
            elif spec.mission_type == "explore":
                outcome = self._call_explore(spec, goal_handle, kind)
            elif spec.mission_type == "search":
                outcome = self._call_search(spec, goal_handle, kind)
            elif spec.mission_type == "navigate":
                outcome = self._call_navigate(
                    spec.x, spec.y, spec.yaw, spec, goal_handle, kind
                )
            elif spec.mission_type == "return_home":
                x, y, yaw = self.instruction_parser.home_pose
                outcome = self._call_navigate(x, y, yaw, spec, goal_handle, kind)
            else:
                outcome = MissionOutcome(False, f"unknown mission: {spec.mission_type}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"mission failed: {exc}")
            outcome = MissionOutcome(False, f"mission exception: {exc}")

        outcome.elapsed_sec = time.monotonic() - started
        if self._cancel_requested and not outcome.success:
            if not self.sm.transition("task_cancel"):
                self.sm.force(State.IDLE)
            self.detail = f"{spec.mission_type} canceled: {outcome.info}"
            terminal = "canceled"
        elif outcome.success:
            if not self.sm.transition("task_done"):
                self.sm.force(State.TASK_SUCCESS)
            self.detail = f"{spec.mission_type} success: {outcome.info}"
            terminal = "succeeded"
        else:
            if not self.sm.transition("task_failed"):
                self.sm.force(State.TASK_FAILED)
            self.detail = f"{spec.mission_type} failed: {outcome.info}"
            terminal = "aborted"

        if goal_handle is not None:
            if terminal == "canceled":
                goal_handle.canceled()
            elif terminal == "succeeded":
                goal_handle.succeed()
            else:
                goal_handle.abort()

        with self._mission_lock:
            self._mission_active = False
        self._active_subgoal = None
        self.publish_status()
        self.publish_mission_status()
        self.get_logger().info(self.detail)
        return outcome

    def _background_mission(self, spec: MissionSpec):
        outcome = self._run_mission(spec, None, kind="mission")
        self.get_logger().info(
            f"background mission {spec.mission_type}: {outcome.success} {outcome.info}"
        )

    def _call_navigate(
        self,
        x: float,
        y: float,
        yaw: float,
        spec: MissionSpec,
        goal_handle,
        kind: str,
    ) -> MissionOutcome:
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            return MissionOutcome(False, "invalid navigation target")
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return MissionOutcome(False, "navigate_to action server unavailable")
        goal = NavigateTo.Goal()
        goal.target_pose = make_pose_stamped(
            "map", Pose2D(x, y, yaw, "map"), self.get_clock().now().to_msg()
        )
        goal.goal_id = self._mission_id
        goal.timeout_sec = float(spec.max_duration or self.default_mission_timeout)
        goal.tolerance_xy = 0.20
        goal.tolerance_yaw = 0.30
        goal.avoid_obstacles = True
        result, reason = self._send_sub_action(
            self.nav_client,
            goal,
            feedback_cb=self._on_nav_feedback,
            timeout=goal.timeout_sec,
            goal_handle=goal_handle,
            kind=kind,
        )
        if result is None:
            return MissionOutcome(False, reason)
        success = bool(getattr(result, "success", False))
        info = str(getattr(result, "info", "navigation finished"))
        final_pose = Pose2D(x, y, yaw, "map")
        return MissionOutcome(
            success,
            info,
            target_pose=final_pose if success else None,
            elapsed_sec=float(getattr(result, "elapsed_sec", 0.0)),
        )

    def _call_explore(
        self, spec: MissionSpec, goal_handle, kind: str
    ) -> MissionOutcome:
        if not self.explore_client.wait_for_server(timeout_sec=5.0):
            return MissionOutcome(False, "explore action server unavailable")
        goal = Explore.Goal()
        goal.mission_id = self._mission_id
        goal.max_duration = float(spec.max_duration or self.default_mission_timeout)
        goal.target_coverage = float(spec.target_coverage or 0.0)
        if goal.target_coverage <= 0.0 or goal.target_coverage > 1.0:
            goal.target_coverage = self.map_coverage
        result, reason = self._send_sub_action(
            self.explore_client,
            goal,
            feedback_cb=self._on_explore_feedback,
            timeout=goal.max_duration,
            goal_handle=goal_handle,
            kind=kind,
        )
        if result is None:
            return MissionOutcome(False, reason)
        success = bool(getattr(result, "success", False))
        info = str(getattr(result, "info", "exploration finished"))
        return MissionOutcome(
            success,
            info,
            elapsed_sec=float(getattr(result, "elapsed_sec", 0.0)),
        )

    def _call_search(
        self, spec: MissionSpec, goal_handle, kind: str
    ) -> MissionOutcome:
        if not spec.target_label:
            return MissionOutcome(False, "target_label is required for search")
        if not self.search_client.wait_for_server(timeout_sec=5.0):
            return MissionOutcome(False, "search_object action server unavailable")
        goal = SearchObject.Goal()
        goal.task_id = self._mission_id
        goal.target_label = spec.target_label
        goal.min_confidence = 0.5
        goal.max_duration = float(spec.max_duration or self.default_mission_timeout)
        goal.standoff_distance = 0.6
        goal.explore_if_not_found = True
        goal.save_map = bool(spec.save_map)
        result, reason = self._send_sub_action(
            self.search_client,
            goal,
            feedback_cb=self._on_search_feedback,
            timeout=goal.max_duration,
            goal_handle=goal_handle,
            kind=kind,
        )
        if result is None:
            return MissionOutcome(False, reason)
        success = bool(getattr(result, "found", False))
        info = str(getattr(result, "info", "search finished"))
        target_pose = None
        approach = None
        target_msg = getattr(result, "target_pose", None)
        if target_msg is not None and target_msg.header.frame_id:
            target_pose = self._pose_from_stamped(target_msg)
        approach_msg = getattr(result, "approach_pose", None)
        if approach_msg is not None and approach_msg.header.frame_id:
            approach = self._pose_from_stamped(approach_msg)
        return MissionOutcome(
            success,
            info,
            target_pose=target_pose,
            approach_pose=approach,
            confidence=float(getattr(result, "confidence", 0.0)),
            elapsed_sec=float(getattr(result, "elapsed_sec", 0.0)),
        )

    def _call_map(self, spec: MissionSpec, goal_handle, kind: str) -> MissionOutcome:
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            return MissionOutcome(False, "map_control service unavailable")
        start = MapControl.Request()
        start.command = "start"
        start.map_name = ""
        response = self._call_service(self.map_client, start, timeout=5.0)
        if response is None or not response.success:
            return MissionOutcome(
                False,
                f"failed to start mapping: {getattr(response, 'msg', 'no response')}",
            )

        explore_spec = dataclasses.replace(
            spec,
            max_duration=float(spec.max_duration or self.map_timeout),
        )
        outcome = self._call_explore(explore_spec, goal_handle, kind)
        if not outcome.success:
            self._stop_mapping()
            return outcome

        save_requested = bool(spec.save_map or self.save_map_after_mapping)
        if save_requested:
            save = MapControl.Request()
            save.command = "save"
            save.map_name = ""
            saved = self._call_service(self.map_client, save, timeout=10.0)
            if saved is None or not saved.success:
                self._stop_mapping()
                return MissionOutcome(
                    False,
                    f"map save failed: {getattr(saved, 'msg', 'no response')}",
                )
            outcome.info = f"{outcome.info}; map saved to {saved.map_file}"
        self._stop_mapping()
        return outcome

    def _stop_mapping(self):
        req = MapControl.Request()
        req.command = "stop"
        req.map_name = ""
        self._call_service(self.map_client, req, timeout=3.0)

    def _call_service(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            reason = self._abort_reason()
            if reason is not None:
                future.cancel()
                return None
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None
        return future.result()

    def _send_sub_action(
        self,
        client,
        goal,
        feedback_cb,
        timeout: float,
        goal_handle,
        kind: str,
    ):
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        handle = self._wait_future(send_future, 10.0)
        if handle is None:
            return None, "send goal timeout/canceled"
        if not handle.accepted:
            return None, "sub-action rejected"
        self._active_subgoal = handle
        result_future = handle.get_result_async()
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok():
            reason = self._abort_reason()
            if reason is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self._active_subgoal = None
                return None, reason
            if goal_handle is not None and goal_handle.is_cancel_requested:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self._active_subgoal = None
                return None, "canceled by client"
            if result_future.done():
                wrapped = result_future.result()
                self._active_subgoal = None
                if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    return None, f"sub-action finished with status {wrapped.status}"
                return wrapped.result, ""
            if time.monotonic() > deadline:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self._active_subgoal = None
                return None, f"sub-action timeout after {timeout:.0f}s"
            now = time.monotonic()
            if now - self._last_emit >= 0.5:
                self._last_emit = now
                self._emit_feedback(goal_handle, kind, self._last_feedback)
            time.sleep(0.05)
        return None, "node shutting down"

    def _wait_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._abort_reason() is not None:
                return None
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return None

    def _abort_reason(self) -> Optional[str]:
        if not rclpy.ok():
            return "node shutting down"
        if self._cancel_requested:
            return "canceled"
        if self.emergency_stop:
            return "emergency stop"
        if not self.data_ok:
            return "sensor data timeout"
        if self.mode != "auto":
            return f"mode changed to {self.mode}"
        if self.sm.state == State.REVERSING:
            return "reverse operation"
        return None

    def _cancel_active_subgoal(self):
        handle = self._active_subgoal
        self._active_subgoal = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # -------------------------------------------------------------- feedback
    def _on_nav_feedback(self, msg):
        fb = msg.feedback
        self._last_feedback = MissionFeedback(
            state=getattr(fb, "state", "navigating"),
            detail=getattr(fb, "detail", ""),
            distance_remaining=float(getattr(fb, "distance_remaining", 0.0) or 0.0),
            best_pose=self._pose_from_stamped(getattr(fb, "current_pose", None)),
        )

    def _on_explore_feedback(self, msg):
        fb = msg.feedback
        self._last_feedback = MissionFeedback(
            state=getattr(fb, "state", "exploring"),
            detail=getattr(fb, "detail", ""),
            explored_ratio=float(getattr(fb, "coverage", 0.0) or 0.0),
            progress=float(getattr(fb, "coverage", 0.0) or 0.0),
        )

    def _on_search_feedback(self, msg):
        fb = msg.feedback
        self._last_feedback = MissionFeedback(
            state=getattr(fb, "state", "searching"),
            detail=getattr(fb, "detail", ""),
            explored_ratio=float(getattr(fb, "explored_ratio", 0.0) or 0.0),
            best_pose=self._pose_from_stamped(getattr(fb, "best_pose", None)),
            best_confidence=float(getattr(fb, "best_confidence", 0.0) or 0.0),
            progress=float(getattr(fb, "explored_ratio", 0.0) or 0.0),
        )

    def _emit_feedback(self, goal_handle, kind: str, feedback: MissionFeedback):
        self.publish_mission_status(feedback)
        if goal_handle is None:
            return
        if kind == "mission":
            msg = ExecuteMission.Feedback()
            msg.state = feedback.state
            msg.detail = feedback.detail
            msg.progress = feedback.progress
            msg.explored_ratio = feedback.explored_ratio
            msg.distance_remaining = feedback.distance_remaining
            msg.best_confidence = feedback.best_confidence
            if feedback.best_pose is not None:
                msg.best_pose = make_pose_stamped(
                    feedback.best_pose.frame_id or "map",
                    feedback.best_pose,
                    self.get_clock().now().to_msg(),
                )
            goal_handle.publish_feedback(msg)
        else:
            msg = ExecuteTask.Feedback()
            msg.progress = feedback.state or self.sm.state_name
            msg.current_intent = self._mission_type
            msg.state = feedback.state
            msg.explored_ratio = feedback.explored_ratio
            msg.distance_remaining = feedback.distance_remaining
            goal_handle.publish_feedback(msg)

    # ---------------------------------------------------------------- results
    def _task_result(self, outcome: MissionOutcome) -> ExecuteTask.Result:
        result = ExecuteTask.Result()
        result.success = bool(outcome.success)
        result.info = outcome.info
        result.confidence = float(outcome.confidence)
        result.elapsed_sec = float(outcome.elapsed_sec)
        if outcome.target_pose is not None:
            result.target_pose = make_pose_stamped(
                outcome.target_pose.frame_id or "map",
                outcome.target_pose,
                self.get_clock().now().to_msg(),
            )
        if outcome.approach_pose is not None:
            result.approach_pose = make_pose_stamped(
                outcome.approach_pose.frame_id or "map",
                outcome.approach_pose,
                self.get_clock().now().to_msg(),
            )
        return result

    def _mission_result(self, outcome: MissionOutcome) -> ExecuteMission.Result:
        result = ExecuteMission.Result()
        result.success = bool(outcome.success)
        result.info = outcome.info
        result.confidence = float(outcome.confidence)
        result.elapsed_sec = float(outcome.elapsed_sec)
        if outcome.target_pose is not None:
            result.target_pose = make_pose_stamped(
                outcome.target_pose.frame_id or "map",
                outcome.target_pose,
                self.get_clock().now().to_msg(),
            )
        if outcome.approach_pose is not None:
            result.approach_pose = make_pose_stamped(
                outcome.approach_pose.frame_id or "map",
                outcome.approach_pose,
                self.get_clock().now().to_msg(),
            )
        return result

    # ---------------------------------------------------------------- spec parsing
    def _normalize_mission_type(self, value: str) -> str:
        value = (value or "").strip().lower()
        return value if value in _MISSION_TYPES else "vla"

    def _spec_from_execute_task(self, request) -> MissionSpec:
        mission_type = self._normalize_mission_type(request.task_type)
        if mission_type != "vla":
            return self._spec_from_fields(
                mission_type,
                target_label=request.target_label,
                x=request.target_x,
                y=request.target_y,
                yaw=request.target_yaw,
                max_duration=request.max_duration,
                save_map=request.save_map,
                target_coverage=request.target_coverage,
            )
        spec = self.instruction_parser.parse(request.instruction)
        if not spec.is_mission:
            return spec
        if request.max_duration > 0.0:
            spec.max_duration = float(request.max_duration)
        return spec

    def _spec_from_execute_mission(self, request) -> MissionSpec:
        mission_type = self._normalize_mission_type(request.mission_type)
        return self._spec_from_fields(
            mission_type,
            target_label=request.target_label,
            x=request.target_x,
            y=request.target_y,
            yaw=request.target_yaw,
            max_duration=request.max_duration,
            save_map=request.save_map,
            target_coverage=request.target_coverage,
        )

    def _spec_from_task_request(self, msg: TaskRequest) -> MissionSpec:
        mission_type = self._normalize_mission_type(msg.task_type)
        if mission_type == "vla" and msg.language_instruction:
            return self.instruction_parser.parse(msg.language_instruction)
        return self._spec_from_fields(
            mission_type,
            target_label=msg.target_label,
            x=msg.target_x,
            y=msg.target_y,
            yaw=msg.target_yaw,
            max_duration=msg.max_duration,
            save_map=msg.save_map,
            target_coverage=msg.target_coverage,
        )

    def _spec_from_fields(
        self,
        mission_type: str,
        target_label: str = "",
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        max_duration: float = 0.0,
        save_map: bool = False,
        target_coverage: float = 0.0,
    ) -> MissionSpec:
        spec = MissionSpec(
            mission_type=mission_type,
            target_label=str(target_label or ""),
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            confidence=0.9,
            reason="explicit mission fields",
            max_duration=float(max_duration or 0.0),
            save_map=bool(save_map),
            target_coverage=float(target_coverage or 0.0),
        )
        return spec

    def _pose_from_stamped(self, msg) -> Optional[Pose2D]:
        if msg is None or not getattr(msg, "header", None):
            return None
        if not msg.header.frame_id:
            return None
        p = msg.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return Pose2D(p.position.x, p.position.y, yaw, msg.header.frame_id)

    # ---------------------------------------------------------------- VLA path
    def _abort(self, goal_handle, info: str):
        self.detail = info
        self.publish_status()
        goal_handle.abort()
        return ExecuteTask.Result(success=False, info=info)

    def _execute_vla(self, goal_handle):
        if self.mode != "auto" or self.emergency_stop:
            return self._abort(goal_handle, "system is not in auto mode")

        state = self.sm.state
        if state in (State.TASK_SUCCESS, State.TASK_FAILED, State.ERROR):
            self.sm.transition("reset")
        elif state == State.PAUSED:
            self.sm.transition("resume")
        if self.sm.state != State.IDLE:
            return self._abort(goal_handle, f"task manager busy in {self.sm.state_name}")

        goal = goal_handle.request
        task_id = (goal.task_id or "").strip() or f"task_{int(time.time() * 1000)}"
        instruction = (goal.instruction or "").strip()
        if not instruction:
            return self._abort(goal_handle, "empty instruction")

        task = TaskRequest()
        task.task_id = task_id
        task.language_instruction = instruction
        task.voice_instruction = ""
        task.priority = 1
        task.enable_reverse = True
        task.task_type = "vla"

        self.active_task_id = task_id
        self.last_result = None
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.executed_cmd = None
        self.sm.transition("recv_data")
        self.sm.transition("start_infer")
        task_start = time.monotonic()
        self.detail = f"task {task_id} dispatched: {instruction}"
        self.get_logger().info(self.detail)
        self.pub_task.publish(task)
        self.publish_status()

        feedback = ExecuteTask.Feedback()
        last_feedback = 0.0
        deadline = time.monotonic() + self.task_timeout
        command_started = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.sm.transition("task_cancel")
                self.detail = f"task {task_id} canceled by client"
                self.publish_status()
                goal_handle.canceled()
                return ExecuteTask.Result(success=False, info="canceled")

            if self.emergency_stop:
                self.sm.force(State.ERROR)
                return self._abort(goal_handle, "emergency stop during task")
            if not self.data_ok:
                self.sm.force(State.IDLE)
                return self._abort(goal_handle, "sensor data timeout during task")
            if self.mode != "auto":
                self.sm.transition("pause")
                return self._abort(goal_handle, f"mode changed to {self.mode}")

            state = self.sm.state
            if state == State.EXECUTING:
                intent = self.last_result.action_type if self.last_result else "unknown"
                if command_started is None:
                    nonzero_command = (
                        self.last_cmd is not None
                        and self.last_cmd_time >= task_start
                        and (
                            abs(self.last_cmd.linear.x) > 1e-6
                            or abs(self.last_cmd.linear.y) > 1e-6
                            or abs(self.last_cmd.angular.z) > 1e-6
                        )
                    )
                    if intent == "stop" or nonzero_command:
                        command_started = time.monotonic()
                        if nonzero_command and self.executed_cmd is None:
                            self.executed_cmd = self.last_cmd
                        self.detail = f"task {task_id} executing: {intent}"
                        self.publish_status()
                if (
                    command_started is not None
                    and time.monotonic() - command_started >= self.execution_hold
                ):
                    self.sm.transition("task_done")
                    self.detail = f"task {task_id} success: {intent}"
                    self.publish_status()
                    self.get_logger().info(self.detail)
                    goal_handle.succeed()
                    info = f"action dispatched: {intent}"
                    if self.executed_cmd is not None:
                        info += (
                            f" (linear.x={self.executed_cmd.linear.x:.2f}, "
                            f"angular.z={self.executed_cmd.angular.z:.2f})"
                        )
                    return ExecuteTask.Result(success=True, info=info)
            if state == State.REVERSING:
                return self._abort(goal_handle, "reverse operation in progress")
            if state == State.PAUSED:
                return self._abort(
                    goal_handle,
                    "low confidence, confirmation required",
                )
            if state == State.ERROR:
                return self._abort(goal_handle, "task manager error")
            if state == State.IDLE and self.last_result is not None:
                return self._abort(goal_handle, "task reset unexpectedly")

            if time.monotonic() > deadline:
                if not self.sm.transition("error"):
                    self.sm.force(State.ERROR)
                return self._abort(
                    goal_handle,
                    f"timeout after {self.task_timeout:.1f}s waiting for "
                    "inference/control command",
                )

            now = time.monotonic()
            if now - last_feedback >= 0.5:
                feedback.progress = state.value
                feedback.current_intent = (
                    self.last_result.action_type if self.last_result else ""
                )
                goal_handle.publish_feedback(feedback)
                last_feedback = now
            time.sleep(0.05)

        return self._abort(goal_handle, "node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
