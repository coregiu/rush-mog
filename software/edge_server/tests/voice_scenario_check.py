#!/usr/bin/env python3
"""语音找东西场景联调检查（需要完整栈 + fake_slam_robot）。

覆盖：
1. 语音“找杯子” → SearchObject → 找到 → /body/say “找到杯子了”；
2. 语音“别找了” → 取消当前搜索 → /body/say “好的，停止寻找”；
3. /center/scenario_status 状态从 searching 到 succeeded/canceled。

用法：
    # 终端 1：完整栈（fake_slam_robot 提供地图/位姿/目标检测）
    ros2 launch edge_server edge_server.launch.py \
        config_file:=$(pwd)/tests/config_test.yaml \
        start_perception:=false start_localization:=false
    python3 tests/fake_slam_robot.py

    # 终端 2：
    python3 tests/voice_scenario_check.py

可调参数：
    --find-instruction 默认“找杯子”
    --find-expect      默认“找到杯子了”
    --timeout          每阶段等待秒数，默认 60
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceScenarioCheck(Node):
    def __init__(self):
        super().__init__("voice_scenario_check")
        self.says = []
        self.statuses = []
        self.search_states = []
        self.create_subscription(String, "/body/say", self.on_say, 10)
        self.create_subscription(String, "/center/scenario_status", self.on_status, 10)
        self.create_subscription(String, "/center/search_status", self.on_search, 10)
        self.pub = self.create_publisher(String, "/edge/voice_command", 10)

    def on_say(self, msg):
        text = str(msg.data or "")
        self.says.append(text)
        print(f"SAY: {text}", flush=True)

    def on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.statuses.append(payload)
        print(
            "SCENARIO: "
            f"{payload.get('state')} target={payload.get('display_name')} "
            f"detail={payload.get('detail')}",
            flush=True,
        )

    def on_search(self, msg):
        self.search_states.append(str(msg.state or ""))
        print(f"SEARCH: {msg.state} {msg.detail}", flush=True)

    def say(self, text):
        print(f"VOICE: {text}", flush=True)
        self.pub.publish(String(data=str(text)))

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def last_state(self):
        return str(self.statuses[-1].get("state", "")) if self.statuses else ""


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--find-instruction", default="找杯子")
    parser.add_argument("--find-expect", default="找到杯子了")
    parser.add_argument("--timeout", type=float, default=60.0)
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = VoiceScenarioCheck()
    results = []

    try:
        print("waiting for subscriptions...", flush=True)
        time.sleep(1.5)

        # 1) 找到目标
        node.say(options.find_instruction)
        found = node.wait_for(
            lambda: any(options.find_expect in item for item in node.says),
            options.timeout,
        )
        results.append(("find+announce", found))
        print(f"CHECK find+announce: {'PASS' if found else 'FAIL'}", flush=True)

        # 2) 取消一个不会命中的搜索
        node.say("找不存在目标")
        node.wait_for(lambda: node.last_state() == "searching", 10.0)
        time.sleep(2.0)
        node.say("别找了")
        canceled_speech = node.wait_for(
            lambda: any("停止寻找" in item for item in node.says[-3:]),
            20.0,
        )
        canceled_state = node.wait_for(
            lambda: node.last_state() == "canceled", 10.0
        )
        canceled = canceled_speech and canceled_state
        results.append(("cancel", canceled))
        print(f"CHECK cancel: {'PASS' if canceled else 'FAIL'}", flush=True)

        # 3) 状态链可观测
        saw_searching = any(
            str(item.get("state")) == "searching" for item in node.statuses
        )
        results.append(("status chain", saw_searching))
        print(f"CHECK status chain: {'PASS' if saw_searching else 'FAIL'}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = sum(1 for _, ok in results if ok)
    print(f"RESULT: {passed}/{len(results)} passed", flush=True)
    raise SystemExit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
