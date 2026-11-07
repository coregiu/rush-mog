#!/usr/bin/env python3
"""SLAM/导航/搜索链路联调检查（需要 launch + fake_slam_robot 在运行）。

覆盖：
1. NavigateTo 到 (1.0, 0)；
2. Explore 到覆盖率阈值；
3. SearchObject 找到 cup 并返回目标位姿；
4. MapControl 保存/加载地图；
5. ExecuteTask "找杯子" 自动转 search 长任务；
6. ExecuteMission map 建图 + 探索 + 保存。

用法：
    ros2 launch center_server center_server.launch.py \
        config_file:=$(pwd)/tests/config_test.yaml \
        start_perception:=false start_localization:=false
    python3 tests/fake_slam_robot.py
    python3 tests/slam_nav_check.py
"""

import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from custom_msgs.action import (
    ExecuteMission,
    ExecuteTask,
    Explore,
    NavigateTo,
    SearchObject,
)
from custom_msgs.srv import MapControl


class SlamNavCheck(Node):
    def __init__(self):
        super().__init__("slam_nav_check")
        self.nav = ActionClient(self, NavigateTo, "/center/navigate_to")
        self.explore = ActionClient(self, Explore, "/center/explore")
        self.search = ActionClient(self, SearchObject, "/center/search_object")
        self.execute_task = ActionClient(self, ExecuteTask, "/center/execute_task")
        self.execute_mission = ActionClient(
            self, ExecuteMission, "/center/execute_mission"
        )
        self.map_client = self.create_client(MapControl, "/center/map_control")
        self._last_feedback = None

    def feedback(self, msg):
        self._last_feedback = msg.feedback

    def wait_action(self, client, name, timeout=20.0):
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"{name} action server unavailable")
        return client

    def run_action(self, client, goal, timeout=60.0):
        future = client.send_goal_async(goal, feedback_callback=self.feedback)
        handle = self._wait(future, 10.0)
        if handle is None or not handle.accepted:
            return None, "goal rejected/timeout"
        result_future = handle.get_result_async()
        wrapped = self._wait(result_future, timeout)
        if wrapped is None:
            return None, "result timeout"
        return wrapped.result, ""

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def call_map(self, command, name=""):
        if not self.map_client.wait_for_service(timeout_sec=10.0):
            return None
        req = MapControl.Request()
        req.command = command
        req.map_name = name
        future = self.map_client.call_async(req)
        return self._wait(future, 20.0)

    # ------------------------------------------------------------------ tests
    def test_navigate(self):
        print("[1] navigate to (1.0, 0)", flush=True)
        self.wait_action(self.nav, "navigate_to")
        goal = NavigateTo.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        goal.goal_id = "check_nav"
        goal.timeout_sec = 30.0
        goal.tolerance_xy = 0.2
        goal.tolerance_yaw = 0.3
        result, err = self.run_action(self.nav, goal, timeout=40.0)
        assert result is not None, err
        assert result.success, result.info
        print(f"  OK ({result.info})", flush=True)

    def test_explore(self):
        print("[2] explore coverage", flush=True)
        self.wait_action(self.explore, "explore")
        goal = Explore.Goal()
        goal.mission_id = "check_explore"
        goal.max_duration = 30.0
        goal.target_coverage = 0.30
        result, err = self.run_action(self.explore, goal, timeout=40.0)
        assert result is not None, err
        assert result.success, result.info
        print(f"  OK coverage={result.coverage:.2f}", flush=True)

    def test_search(self):
        print("[3] search cup", flush=True)
        self.wait_action(self.search, "search_object")
        goal = SearchObject.Goal()
        goal.task_id = "check_search"
        goal.target_label = "cup"
        goal.min_confidence = 0.5
        goal.max_duration = 60.0
        goal.standoff_distance = 0.6
        goal.explore_if_not_found = True
        result, err = self.run_action(self.search, goal, timeout=70.0)
        assert result is not None, err
        assert result.found, result.info
        assert result.target_pose.header.frame_id == "map"
        assert abs(result.target_pose.pose.position.x - 1.5) < 0.5
        print(
            f"  OK target=({result.target_pose.pose.position.x:.2f}, "
            f"{result.target_pose.pose.position.y:.2f}) conf={result.confidence:.2f}",
            flush=True,
        )

    def test_map_io(self):
        print("[4] map save/load", flush=True)
        if not self.map_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("map_control service unavailable")
        saved = self.call_map("save", "smoke_map")
        assert saved is not None and saved.success, getattr(saved, "msg", "no response")
        assert saved.map_file and os.path.exists(saved.map_file), saved.map_file
        loaded = self.call_map("load", "smoke_map")
        assert loaded is not None and loaded.success, getattr(loaded, "msg", "no response")
        print(f"  OK {saved.map_file}", flush=True)

    def test_execute_task_mission(self):
        print("[5] ExecuteTask '找杯子' -> mission dispatch", flush=True)
        if not self.execute_task.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("execute_task action server unavailable")
        goal = ExecuteTask.Goal()
        goal.task_id = "check_task_search"
        goal.instruction = "找杯子"
        result, err = self.run_action(self.execute_task, goal, timeout=80.0)
        assert result is not None, err
        assert result.success, result.info
        assert result.target_pose.header.frame_id == "map"
        print(
            f"  OK target=({result.target_pose.pose.position.x:.2f}, "
            f"{result.target_pose.pose.position.y:.2f})",
            flush=True,
        )

    def test_execute_mission_map(self):
        print("[6] ExecuteMission map -> explore + save", flush=True)
        if not self.execute_mission.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("execute_mission action server unavailable")
        goal = ExecuteMission.Goal()
        goal.mission_id = "check_map"
        goal.mission_type = "map"
        goal.max_duration = 60.0
        goal.target_coverage = 0.30
        goal.save_map = True
        result, err = self.run_action(self.execute_mission, goal, timeout=90.0)
        assert result is not None, err
        assert result.success, result.info
        assert "map saved" in result.info or "保存" in result.info, result.info
        print(f"  OK {result.info}", flush=True)


def main():
    rclpy.init()
    node = SlamNavCheck()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    tests = [
        ("navigate", node.test_navigate),
        ("explore", node.test_explore),
        ("search", node.test_search),
        ("map_io", node.test_map_io),
        ("execute_task_mission", node.test_execute_task_mission),
        ("execute_mission_map", node.test_execute_mission_map),
    ]
    failures = 0
    try:
        time.sleep(2.0)
        for name, fn in tests:
            try:
                fn()
            except AssertionError as exc:
                failures += 1
                print(f"  FAIL {name}: {exc}", flush=True)
            except Exception as exc:  # noqa: BLE001
                failures += 1
                print(f"  ERROR {name}: {exc}", flush=True)
            time.sleep(0.5)
    finally:
        print(f"RESULT: {len(tests) - failures}/{len(tests)} passed", flush=True)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
