"""树莓派电源监控核心逻辑测试（不依赖真实 vcgencmd/sysfs）。"""

import pytest

from edge_server.core.power import (
    PowerMonitor,
    parse_temp,
    parse_throttled,
    parse_volts,
)
from edge_server.core.speaker import CommandResult


class FakeRunner:
    def __init__(self, throttled="throttled=0x0", volts="volt=0.8500V", temp="temp=39.4'C"):
        self.throttled = throttled
        self.volts = volts
        self.temp = temp
        self.calls = []

    def run(self, args, timeout=10.0, **kwargs):
        self.calls.append(list(args))
        command = [str(item) for item in args]
        if command[-1] == "get_throttled":
            return CommandResult(0, self.throttled, "")
        if command[-2:] == ["measure_volts", "core"]:
            return CommandResult(0, self.volts, "")
        if command[-1] == "measure_temp":
            return CommandResult(0, self.temp, "")
        return CommandResult(0, "", "")


def make_monitor(tmp_path, runner=None, alarm="0", enabled=True):
    model = tmp_path / "model"
    model.write_text("Raspberry Pi 4 Model B Rev 1.4\x00", encoding="utf-8")
    alarm_path = tmp_path / "in0_lcrit_alarm"
    alarm_path.write_text(alarm, encoding="utf-8")
    return PowerMonitor(
        {
            "enabled": enabled,
            "sample_interval_sec": 10,
            "model_path": str(model),
            "alarm_path": str(alarm_path),
        },
        runner=runner or FakeRunner(),
        which=lambda name: "/usr/bin/vcgencmd" if name == "vcgencmd" else None,
    )


def test_parse_throttled_flags():
    now = parse_throttled("throttled=0x1")
    assert now["available"] is True
    assert now["undervoltage"] is True
    assert parse_throttled("throttled=0x4")["throttled"] is True
    assert parse_throttled("throttled=0x8")["soft_temp_limit"] is True

    history = parse_throttled("throttled=0x50000")
    assert history["undervoltage"] is False
    assert history["undervoltage_occurred"] is True
    assert history["throttled_occurred"] is True
    assert history["raw"] == "0x50000"

    assert parse_throttled("garbage")["available"] is False


def test_parse_volts_and_temp():
    assert parse_volts("volt=0.8500V") == 0.85
    assert parse_temp("temp=39.4'C") == 39.4
    assert parse_volts("bad") is None
    assert parse_temp("bad") is None


def test_history_only_is_ok_with_warning(tmp_path):
    monitor = make_monitor(tmp_path, FakeRunner("throttled=0x50000"))
    status = monitor.status()
    assert status["available"] is True
    assert status["status"] == "ok"
    assert status["healthy"] is True
    assert status["warning"] is True
    assert status["undervoltage"] is False
    assert status["undervoltage_occurred"] is True
    assert status["throttled_raw"] == "0x50000"
    assert status["model"].startswith("Raspberry Pi 4")
    assert status["core_voltage_v"] == 0.85
    assert status["temperature_c"] == 39.4
    assert any("此前发生过" in item for item in status["warnings"])


def test_current_undervoltage_is_error(tmp_path):
    monitor = make_monitor(tmp_path, FakeRunner("throttled=0x1"))
    status = monitor.status()
    assert status["status"] == "error"
    assert status["healthy"] is False
    assert status["undervoltage"] is True


def test_alarm_is_error(tmp_path):
    monitor = make_monitor(tmp_path, FakeRunner("throttled=0x0"), alarm="1")
    status = monitor.status()
    assert status["status"] == "error"
    assert status["alarm"] == 1


def test_throttled_is_degraded(tmp_path):
    monitor = make_monitor(tmp_path, FakeRunner("throttled=0x4"))
    status = monitor.status()
    assert status["status"] == "degraded"
    assert status["throttled"] is True


def test_missing_vcgencmd(tmp_path):
    monitor = make_monitor(tmp_path, FakeRunner(), enabled=True)
    monitor._which = lambda name: None
    status = monitor.status()
    assert status["status"] == "error"
    assert "vcgencmd" in status["error"]


def test_disabled(tmp_path):
    monitor = make_monitor(tmp_path, enabled=False)
    status = monitor.status()
    assert status["status"] == "disabled"
    assert status["available"] is False


def test_cache_avoids_repeated_commands(tmp_path):
    runner = FakeRunner()
    monitor = make_monitor(tmp_path, runner)
    monitor.status()
    monitor.status()
    assert len(runner.calls) == 3  # get_throttled + measure_volts + measure_temp
    monitor.status(force=True)
    assert len(runner.calls) == 6
