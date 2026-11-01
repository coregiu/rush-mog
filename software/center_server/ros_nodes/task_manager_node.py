"""task_manager_node：任务状态机、模式服务与 ExecuteTask Action。

服务：/center/set_mode
Action：/center/execute_task
状态发布：/center/status

说明：真实 VLA 长任务闭环需要模型给出任务完成信号。当前 VLA 为规则桩，
ExecuteTask 的实现是“单步任务”：把指令下发给 VLA，检测到 action_generator
真正发布非零 /cmd_vel 后，保持一个短暂执行窗口再判定成功；超时/低置信度/
急停会中止任务。
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from center_server.core.state_machine import State, TaskStateMachine
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.action import ExecuteTask
from custom_msgs.msg import (
    InferenceResult,
    ReverseOperation,
    SafetyState,
    SystemStatus,
    TaskRequest,
)
from custom_msgs.srv import SetMode


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")
        self.cfg = load_config(self)
        tm_cfg = self.cfg.get("task_manager", {})
        self.task_timeout = float(tm_cfg.get("task_timeout_sec", 30.0))
        self.reverse_timeout = float(tm_cfg.get("reverse_timeout_sec", 5.0))
        self.status_rate = max(0.5, float(tm_cfg.get("status_rate", 2.0)))
        self.execution_hold = max(
            0.0, float(tm_cfg.get("execution_hold_sec", 1.0))
        )

        self.sm = TaskStateMachine()
        self.mode = "auto"
        self.emergency_stop = False
        self.data_ok = True
        self.latest_task = None
        self.last_result = None
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.executed_cmd = None
        self.active_task_id = ""
        self.detail = "idle"
        self.reverse_until = 0.0
        self._executing = False
        self._exec_lock = threading.Lock()

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_task = self.create_publisher(TaskRequest, "/task/request", 10)
        self.pub_status = self.create_publisher(
            SystemStatus, "/center/status", status_qos
        )

        self.create_subscription(
            TaskRequest, "/task/request", self.on_task_request, 10
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
        self.create_service(SetMode, "/center/set_mode", self.on_set_mode)

        self.action_server = ActionServer(
            self,
            ExecuteTask,
            "/center/execute_task",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self._steady_clock = create_steady_clock()
        self.create_timer(
            1.0 / self.status_rate,
            self.on_status_timer,
            clock=self._steady_clock,
        )
        self.get_logger().info(
            f"task_manager_node started (task_timeout={self.task_timeout:.1f}s)"
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

    def on_status_timer(self):
        if self.sm.state == State.REVERSING and time.monotonic() > self.reverse_until:
            if self.sm.transition("reverse_done"):
                self.detail = "reverse operation finished"
                self.get_logger().info("reverse operation finished")
        self.publish_status()

    # ------------------------------------------------------------- callbacks
    def on_task_request(self, msg: TaskRequest):
        if self.sm.state in (State.IDLE, State.TASK_SUCCESS, State.TASK_FAILED):
            # 空闲时接受外部 HMI / Action 的任务
            self.latest_task = msg
            self.active_task_id = msg.task_id
            self.last_result = None
            if self.sm.transition("recv_data"):
                self.detail = f"task received: {msg.task_id}"
                self.publish_status()
        elif msg.task_id == self.active_task_id:
            # Action 内部发布的任务回声
            self.latest_task = msg
        else:
            self.get_logger().warn(
                f"ignore new task {msg.task_id!r} while busy in "
                f"{self.sm.state_name}"
            )

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_inference(self, msg: InferenceResult):
        if self.active_task_id and msg.task_id and msg.task_id != self.active_task_id:
            return  # 忽略过期任务的结果
        if self.sm.state in (State.IDLE, State.TASK_SUCCESS, State.TASK_FAILED):
            # 任务结束后 VLA 仍会持续推理，避免覆盖最终状态
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
        ):
            # DESIGN.md 11：传感器数据超时停止输出，回到 IDLE
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

    # ---------------------------------------------------------------- action
    def goal_cb(self, goal_request):
        if self.mode != "auto":
            return GoalResponse.REJECT
        if self.emergency_stop or not self.data_ok:
            return GoalResponse.REJECT
        if not (goal_request.instruction or "").strip():
            return GoalResponse.REJECT
        if self.sm.state not in (
            State.IDLE,
            State.TASK_SUCCESS,
            State.TASK_FAILED,
            State.PAUSED,
            State.ERROR,
        ):
            return GoalResponse.REJECT
        with self._exec_lock:
            if self._executing:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        with self._exec_lock:
            self._executing = True
        try:
            return self._execute(goal_handle)
        finally:
            with self._exec_lock:
                self._executing = False

    def _abort(self, goal_handle, info: str):
        self.detail = info
        self.publish_status()
        goal_handle.abort()
        return ExecuteTask.Result(success=False, info=info)

    def _execute(self, goal_handle):
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
                    # 等 action_generator 真正下发非零控制指令，避免“推理成功但车没动”
                    nonzero_command = (
                        self.last_cmd is not None
                        and self.last_cmd_time >= task_start
                        and (
                            abs(self.last_cmd.linear.x) > 1e-6
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
    executor = MultiThreadedExecutor(num_threads=4)
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
