"""Edge Server 控制面纯逻辑测试（不依赖 ROS/串口/网络）。"""

import pytest

from edge_server.edge_control import (
    MotionState,
    button_command,
    dispatch_control_message,
    is_sustained_motion_command,
    is_valid_raw_motion_command,
    parse_body_line,
    stick_command,
)


def test_parse_body_line_valid_and_to_dict():
    telemetry = parse_body_line("$B,7,1234,100,-200,300,-400,5678", now=10.0)
    assert telemetry is not None
    assert telemetry.seq == 7
    assert telemetry.uptime_ms == 1234
    assert telemetry.wheel_mm_s == [100, -200, 300, -400]
    assert telemetry.distance_mm == 5678
    payload = telemetry.to_dict(now=10.5)
    assert payload["valid"] is True
    assert payload["stale"] is False
    assert payload["age_ms"] == 500
    assert payload["wheel_mm_s"] == [100, -200, 300, -400]


@pytest.mark.parametrize(
    "line",
    [
        "",
        "B,1,2,3,4,5,6,7",
        "$B,1,2,3,4,5,6",
        "$B,-1,2,3,4,5,6,7",
        "$B,1,2,6000,4,5,6,7",
        "$B,1,2,3,4,5,6,x",
    ],
)
def test_parse_body_line_rejects_invalid(line):
    assert parse_body_line(line) is None


def test_stale_telemetry_is_flagged():
    telemetry = parse_body_line("$B,1,10,0,0,0,0,0", now=100.0)
    assert telemetry is not None
    payload = telemetry.to_dict(now=103.0)
    assert payload["valid"] is True
    assert payload["stale"] is True
    assert payload["age_ms"] == 3000


def test_button_mapping():
    assert button_command("up", "down") == "1"
    assert button_command("up", "up") == "0"
    assert button_command("l1", "down") == "9"
    assert button_command("l1", "up") == "0"
    assert button_command("square", "up") == "I"
    assert button_command("triangle", "up") == ""
    assert button_command("stop", "down") == "0"


def test_stick_mapping():
    assert stick_command("left-analog", "up", "5") == "15"
    assert stick_command("left-analog", "dnrt", "9") == "89"
    assert stick_command("left-analog", "up", "0") == "0"
    assert stick_command("left-analog", "stop", "5") == "0"
    assert stick_command("right-analog", "up", "5") == ""
    assert stick_command("left-analog", "up", "55") == ""


def test_raw_command_validation():
    for command in ["0", "1", "15", "95", "A5", "C0"]:
        assert is_valid_raw_motion_command(command)
    for command in ["", "00", "J", "Z", "155", "1A", "A"]:
        if command == "A":
            assert is_valid_raw_motion_command(command)
        else:
            assert not is_valid_raw_motion_command(command)


def test_motion_manual_keepalive_and_release():
    motion = MotionState(keepalive_ms=200)
    assert motion.apply_manual("15", now=1.0) == "15"
    assert motion.snapshot(1.1)["active"] is True
    # 网页摇杆保活只重发方向
    assert motion.tick(1.2) == "1"
    assert motion.apply_manual("0", now=1.3) == "0"
    assert motion.tick(1.4) == ""
    assert motion.snapshot(1.4)["active"] is False


def test_motion_raw_lease_expires():
    motion = MotionState(raw_lease_sec=1.0)
    assert motion.apply_raw("15", now=1.0) == "15"
    assert motion.tick(1.9) == "15"
    assert motion.tick(2.1) == "0"
    assert motion.snapshot(2.2)["active"] is False


def test_manual_control_has_priority_over_ros():
    motion = MotionState()
    assert motion.apply_manual("1", now=1.0) == "1"
    assert motion.apply_raw("2", now=1.1) == ""
    assert motion.apply_raw("0", now=1.2) == ""
    assert motion.tick(1.3) == "1"
    motion.clear()
    assert motion.apply_raw("2", now=1.4) == "2"


def test_dispatch_control_message():
    motion = MotionState()
    assert dispatch_control_message("button:up:down", motion, now=1.0) == "1"
    assert dispatch_control_message("button:up:up", motion, now=1.1) == "0"
    assert dispatch_control_message("stick:left-analog:left:4", motion, now=1.2) == "34"
    motion.clear()
    assert dispatch_control_message("raw:95", motion, now=1.3) == "95"
    assert dispatch_control_message("hold:1", motion, now=1.4) == ""
    assert dispatch_control_message("raw:9A", motion, now=1.5) == ""


def test_sustained_command_detection():
    for command in ["1", "9", "A", "B", "C", "15", "89"]:
        assert is_sustained_motion_command(command)
    for command in ["0", "D", "J", "E", "I", "G", "H", "95"]:
        assert not is_sustained_motion_command(command)
