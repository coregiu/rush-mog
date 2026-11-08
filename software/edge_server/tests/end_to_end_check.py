#!/usr/bin/env python3
"""edge_server 端到端冒烟测试（需要 launch + fake_edge_publisher 已在运行）。"""

import collections
import re
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from custom_msgs.action import ExecuteTask
from custom_msgs.msg import (
    ActionIntent,
    InferenceResult,
    ReverseOperation,
    SafetyState,
    SystemStatus,
)
from custom_msgs.srv import EmergencyStop, SetMode


class IntegrationTest(Node):
    def __init__(self):
        super().__init__("edge_server_integration_test")
        self.lock = threading.Lock()
        self.cmds = collections.deque(maxlen=500)
        self.statuses = collections.deque(maxlen=100)
        self.safeties = collections.deque(maxlen=100)
        self.intents = collections.deque(maxlen=100)
        self.results = collections.deque(maxlen=100)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(SystemStatus, "/center/status", self.on_status, 10)
        self.create_subscription(
            SafetyState, "/center/safety_state", self.on_safety, 10
        )
        self.create_subscription(
            ActionIntent, "/center/action_intent", self.on_intent, 10
        )
        self.create_subscription(
            InferenceResult, "/center/inference_result", self.on_result, 10
        )
        self.reverse_pub = self.create_publisher(
            ReverseOperation, "/reverse/operation", 10
        )
        self.obstacle_pub = self.create_publisher(
            Float32, "/sim/obstacle_distance", 10
        )
        self.action_client = ActionClient(
            self, ExecuteTask, "/center/execute_task"
        )
        self.mode_client = self.create_client(SetMode, "/center/set_mode")
        self.estop_client = self.create_client(
            EmergencyStop, "/center/emergency_stop"
        )

    # ------------------------------------------------------------- callbacks
    def on_cmd(self, msg):
        with self.lock:
            self.cmds.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def on_status(self, msg):
        with self.lock:
            self.statuses.append(
                (time.monotonic(), msg.state, msg.mode, msg.emergency_stop, msg.detail)
            )

    def on_safety(self, msg):
        with self.lock:
            self.safeties.append(
                (
                    time.monotonic(),
                    msg.emergency_stop,
                    msg.data_ok,
                    msg.min_obstacle_distance,
                    msg.reason,
                )
            )

    def on_intent(self, msg):
        with self.lock:
            self.intents.append((time.monotonic(), msg.intent, msg.value))

    def on_result(self, msg):
        with self.lock:
            self.results.append(
                (time.monotonic(), msg.task_id, msg.status, msg.action_type)
            )

    # --------------------------------------------------------------- helpers
    def latest_cmd(self):
        with self.lock:
            return self.cmds[-1] if self.cmds else None

    def latest_status(self):
        with self.lock:
            return self.statuses[-1] if self.statuses else None

    def latest_safety(self):
        with self.lock:
            return self.safeties[-1] if self.safeties else None

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        print(f"  TIMEOUT waiting for {description}; status={self.latest_status()}, "
              f"safety={self.latest_safety()}, cmd={self.latest_cmd()}", flush=True)
        return False

    def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def call_service(self, client, request, timeout=15):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        return self.wait_future(client.call_async(request), timeout)

    def run_action(self, task_id, instruction, timeout=40):
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            return None, "action server unavailable"
        goal = ExecuteTask.Goal()
        goal.task_id = task_id
        goal.instruction = instruction
        goal_handle = self.wait_future(
            self.action_client.send_goal_async(goal), timeout
        )
        if goal_handle is None:
            return None, "send_goal timeout"
        if not goal_handle.accepted:
            return None, "goal rejected"
        wrapped = self.wait_future(goal_handle.get_result_async(), timeout)
        if wrapped is None:
            return None, "result timeout"
        return wrapped.result, ""

    # ----------------------------------------------------------------- tests
    def test_forward(self):
        print("[1] forward task", flush=True)
        result, err = self.run_action("it_forward", "前进")
        assert result is not None, err
        assert result.success, f"action failed: {result.info}"
        # Action 成功条件：task_manager 观察到非零 /cmd_vel。
        # 部分沙箱环境 DDS 发现很慢，测试节点自身不一定收到 /cmd_vel，
        # 因此优先从 result.info 校验实际下发的速度。
        match = re.search(r"linear\.x=(-?\d+\.\d+)", result.info)
        from_result = match is not None and float(match.group(1)) > 0.02
        from_topic = self.latest_cmd() is not None and self.latest_cmd()[1] > 0.02
        assert from_result or from_topic, result.info
        assert self.wait_until(
            lambda: self.latest_status()[1] == "TASK_SUCCESS", 3, "TASK_SUCCESS"
        )
        print(f"  OK ({result.info})", flush=True)

    def test_turn(self):
        print("[2] turn-left task", flush=True)
        result, err = self.run_action("it_turn", "左转")
        assert result is not None and result.success, f"{err} {getattr(result, 'info', '')}"
        match = re.search(r"angular\.z=(-?\d+\.\d+)", result.info)
        from_result = match is not None and float(match.group(1)) > 0.02
        from_topic = self.latest_cmd() is not None and self.latest_cmd()[2] > 0.02
        assert from_result or from_topic, result.info
        print(f"  OK ({result.info})", flush=True)

    def test_low_confidence(self):
        print("[3] low-confidence task", flush=True)
        result, err = self.run_action("it_low", "帮我泡一杯咖啡")
        assert result is not None, err
        assert not result.success, result.info
        assert self.wait_until(
            lambda: self.latest_status()[1] == "PAUSED", 3, "PAUSED"
        ), self.latest_status()
        print("  OK", flush=True)

    def test_manual_mode(self):
        print("[4] manual mode", flush=True)
        req = SetMode.Request()
        req.mode = "manual"
        resp = self.call_service(self.mode_client, req)
        assert resp is not None and resp.success, resp
        assert self.wait_until(
            lambda: self.latest_status()[2] == "manual", 3, "mode=manual"
        )
        result, err = self.run_action("it_manual", "前进", timeout=10)
        assert result is None and err == "goal rejected", f"{result} {err}"
        req.mode = "auto"
        resp = self.call_service(self.mode_client, req)
        assert resp is not None and resp.success, resp
        assert self.wait_until(
            lambda: self.latest_status()[2] == "auto", 3, "mode=auto"
        )
        print("  OK", flush=True)

    def test_emergency_stop(self):
        print("[5] emergency stop service", flush=True)
        req = EmergencyStop.Request()
        req.trigger = True
        resp = self.call_service(self.estop_client, req)
        assert resp is not None and resp.ack, resp
        assert self.wait_until(
            lambda: self.latest_safety()[1] is True, 4, "safety estop"
        ), self.latest_safety()
        assert self.wait_until(
            lambda: abs(self.latest_cmd()[1]) < 1e-6, 3, "zero cmd on estop"
        ), self.latest_cmd()
        result, err = self.run_action("it_estop", "前进", timeout=10)
        assert result is None and err == "goal rejected", f"{result} {err}"
        req.trigger = False
        resp = self.call_service(self.estop_client, req)
        assert resp is not None and resp.ack, resp
        assert self.wait_until(
            lambda: self.latest_safety()[1] is False, 4, "safety estop cleared"
        ), self.latest_safety()
        assert self.wait_until(
            lambda: self.latest_status()[1] == "IDLE", 4, "task manager IDLE"
        ), self.latest_status()
        print("  OK", flush=True)

    def test_obstacle(self):
        print("[6] obstacle auto stop", flush=True)
        self.obstacle_pub.publish(Float32(data=0.2))
        assert self.wait_until(
            lambda: self.latest_safety()[1] is True, 5, "obstacle estop"
        ), self.latest_safety()
        assert "obstacle" in self.latest_safety()[4], self.latest_safety()
        assert self.wait_until(
            lambda: abs(self.latest_cmd()[1]) < 1e-6, 3, "zero cmd on obstacle"
        ), self.latest_cmd()
        self.obstacle_pub.publish(Float32(data=3.0))
        assert self.wait_until(
            lambda: self.latest_safety()[1] is False, 5, "obstacle estop cleared"
        ), self.latest_safety()
        print("  OK", flush=True)

    def test_reverse(self):
        print("[7] reverse move", flush=True)
        op = ReverseOperation()
        op.operation_type = ReverseOperation.REVERSE_MOVE
        op.distance = 0.3
        op.reason = "integration_test"
        started = time.monotonic()
        self.reverse_pub.publish(op)

        def saw_reverse():
            with self.lock:
                return any(t >= started and x < -0.25 for t, x, _ in self.cmds)

        assert self.wait_until(saw_reverse, 4, "negative reverse velocity")
        assert self.wait_until(
            lambda: self.latest_status()[1] == "REVERSING", 3, "REVERSING state"
        ), self.latest_status()
        assert self.wait_until(
            lambda: abs(self.latest_cmd()[1]) < 1e-6, 4, "reverse stop"
        ), self.latest_cmd()
        assert self.wait_until(
            lambda: self.latest_status()[1] == "IDLE", 8, "IDLE after reverse"
        ), self.latest_status()
        print(f"  OK ({time.monotonic() - started:.1f}s)", flush=True)


def main():
    rclpy.init()
    node = IntegrationTest()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    failures = 0
    tests = [
        "test_forward",
        "test_turn",
        "test_low_confidence",
        "test_manual_mode",
        "test_emergency_stop",
        "test_obstacle",
        "test_reverse",
    ]
    try:
        if not node.wait_until(
            lambda: node.latest_safety() is not None
            and node.latest_safety()[2] is True,
            30,
            "sensor data ready",
        ):
            raise RuntimeError("sensor data never became ready")
        for name in tests:
            try:
                getattr(node, name)()
            except AssertionError as exc:
                failures += 1
                print(f"  FAIL {name}: {exc}", flush=True)
            time.sleep(0.3)
    finally:
        print(f"RESULT: {len(tests) - failures}/{len(tests)} passed", flush=True)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
