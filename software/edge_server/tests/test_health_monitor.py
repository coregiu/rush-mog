import math

import pytest

from edge_server.core.health_monitor import HealthMonitor

CAMERA = {
    "type": "camera",
    "topic": "/camera/image_raw",
    "message_type": "sensor_msgs/msg/Image",
    "expected_rate_hz": 10.0,
    "timeout_sec": 1.0,
    "required": True,
}
LIDAR = {
    "type": "lidar",
    "topic": "/scan",
    "message_type": "sensor_msgs/msg/LaserScan",
    "expected_rate_hz": 10.0,
    "timeout_sec": 1.0,
    "required": True,
}
SPEAKER = {
    "type": "speaker",
    "topic": "/body/audio_status",
    "message_type": "std_msgs/msg/String",
    "expected_rate_hz": 0.0,
    "timeout_sec": 2.0,
    "required": False,
}


def make_monitor(devices=None, **kwargs):
    return HealthMonitor(
        devices or {"camera": CAMERA, "lidar": LIDAR, "speaker": SPEAKER},
        startup_grace_sec=kwargs.pop("startup_grace_sec", 0.0),
        started_at=kwargs.pop("started_at", 0.0),
        safety_required=kwargs.pop("safety_required", False),
        **kwargs,
    )


def feed(monitor, device_id, *, start=1.0, rate=10.0, count=20, metrics=None):
    for index in range(count):
        monitor.on_message(
            device_id,
            start + index / rate,
            metrics=metrics if metrics is not None else {"index": index},
        )
    return start + (count - 1) / rate + 0.1


def device(report, device_id):
    for item in report.devices:
        if item.id == device_id:
            return item
    raise AssertionError(f"device {device_id} missing")


def test_all_required_devices_ok():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(now=3.0)

    assert report.status == "ok"
    assert report.healthy is True
    assert report.ready is True
    assert report.reason == "ok"
    assert report.summary == {
        "total": 3,
        "enabled": 3,
        "ok": 2,
        "degraded": 0,
        "error": 0,
        "unknown": 1,
        "disabled": 0,
    }
    camera = device(report, "camera")
    assert camera.status == "ok"
    assert camera.rate_hz == pytest.approx(10.0, abs=0.5)
    assert camera.metrics == {"index": 19}


def test_required_device_stale_is_error():
    monitor = make_monitor()
    monitor.on_message("camera", 1.0)
    monitor.on_message("lidar", 1.0)
    report = monitor.snapshot(now=3.0)

    assert report.status == "error"
    assert report.ready is False
    assert device(report, "camera").status == "error"
    assert "stale" in device(report, "camera").detail


def test_optional_device_missing_is_unknown_but_not_fatal():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(now=3.0)

    speaker = device(report, "speaker")
    assert speaker.status == "unknown"
    assert speaker.healthy is False
    assert report.status == "ok"
    assert report.ready is True

    strict = make_monitor(strict_unknown=True)
    feed(strict, "camera")
    feed(strict, "lidar")
    strict_report = strict.snapshot(now=3.0)
    assert strict_report.status == "degraded"
    assert "speaker" in strict_report.reason


def test_optional_device_description_explains_unknown():
    monitor = make_monitor(
        {
            "camera": CAMERA,
            "lidar": LIDAR,
            "speaker": {**SPEAKER, "description": "speaker feedback not implemented"},
        }
    )
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(now=3.0)

    assert device(report, "speaker").detail == "speaker feedback not implemented"


def test_low_rate_degrades():
    monitor = make_monitor()
    feed(monitor, "camera", rate=2.0, count=8)
    feed(monitor, "lidar", start=4.0, count=5)
    report = monitor.snapshot(now=4.6)

    camera = device(report, "camera")
    assert camera.status == "degraded"
    assert "low rate" in camera.detail
    assert report.status == "degraded"


def test_startup_grace_then_error():
    monitor = make_monitor(startup_grace_sec=5.0, started_at=0.0)

    early = monitor.snapshot(now=1.0)
    assert early.status == "starting"
    assert early.ready is False
    assert device(early, "camera").status == "unknown"

    late = monitor.snapshot(now=6.0)
    assert late.status == "error"
    assert device(late, "camera").status == "error"


def test_disabled_device_is_ignored():
    monitor = make_monitor(
        {
            "camera": {**CAMERA, "enabled": False},
            "lidar": LIDAR,
        }
    )
    feed(monitor, "lidar")
    report = monitor.snapshot(now=3.0)

    camera = device(report, "camera")
    assert camera.status == "disabled"
    assert camera.healthy is False
    assert report.status == "ok"
    assert report.summary["disabled"] == 1


def test_invalid_optional_payload_degrades():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    monitor.on_message(
        "speaker", 2.9, metrics={"online": False}, invalid=True, error="speaker offline"
    )
    report = monitor.snapshot(now=3.0)

    speaker = device(report, "speaker")
    assert speaker.status == "degraded"
    assert speaker.detail == "speaker offline"
    assert report.status == "degraded"


def test_invalid_required_payload_is_error():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    monitor.on_message("camera", 2.95, invalid=True, error="empty image")
    report = monitor.snapshot(now=3.0)

    assert device(report, "camera").status == "error"
    assert report.status == "error"


def test_config_error_is_error():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    monitor.set_config_error("camera", "topic not configured")
    report = monitor.snapshot(now=3.0)

    assert device(report, "camera").status == "error"
    assert "topic not configured" in report.reason
    assert report.status == "error"


def test_emergency_stop_degrades_and_not_ready():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(
        now=3.0,
        safety={
            "available": True,
            "emergency_stop": True,
            "reason": "manual_emergency_stop",
        },
    )

    assert report.status == "degraded"
    assert report.ready is False
    assert "emergency_stop" in report.reason
    assert report.safety["emergency_stop"] is True


def test_missing_expected_node_is_error_after_grace():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(
        now=3.0,
        ros_info={"ok": False, "missing_nodes": ["vla_inference_node"]},
    )

    assert report.status == "error"
    assert "missing nodes" in report.reason
    assert report.ros["missing_nodes"] == ["vla_inference_node"]


def test_missing_safety_is_starting_then_error():
    monitor = make_monitor(startup_grace_sec=5.0, started_at=0.0, safety_required=True)
    feed(monitor, "camera")
    feed(monitor, "lidar")

    early = monitor.snapshot(now=1.0, safety={"available": False})
    assert early.status == "starting"

    late = monitor.snapshot(now=6.0, safety={"available": False})
    assert late.status == "error"
    assert "safety state unavailable" in late.reason


def test_filtered_snapshot_keeps_global_status():
    monitor = make_monitor()
    feed(monitor, "camera")
    monitor.on_message("lidar", 1.0)  # 只有一帧，到 3.0 已超时
    report = monitor.snapshot(now=3.0, device_ids=["camera"])

    assert [item.id for item in report.devices] == ["camera"]
    assert report.summary["total"] == 1
    assert report.status == "error"

    with pytest.raises(KeyError):
        monitor.snapshot(now=3.0, device_ids=["nope"])


def test_report_json_and_metrics_toggle():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")
    report = monitor.snapshot(now=3.0)

    full = report.to_dict(include_metrics=True)
    lean = report.to_dict(include_metrics=False)

    assert full["ok"] is True
    assert full["timestamp"].endswith("Z")
    assert full["devices"][0]["metrics"]["index"] == 19
    assert "metrics" not in lean["devices"][0]
    assert math.isfinite(lean["devices"][0]["rate_hz"])


def test_monitor_disabled():
    monitor = HealthMonitor({}, enabled=False, started_at=0.0)
    report = monitor.snapshot(now=1.0)

    assert report.status == "disabled"
    assert report.healthy is False
    assert report.ready is False
    assert report.devices == []
    assert report.summary["total"] == 0


def test_configure_device_updates_runtime_thresholds():
    monitor = make_monitor()
    feed(monitor, "camera")
    feed(monitor, "lidar")

    spec = monitor.configure_device(
        "camera", expected_rate_hz=20.0, timeout_sec=0.5, required=False
    )
    assert spec.expected_rate_hz == 20.0
    assert spec.timeout_sec == 0.5
    assert spec.required is False

    report = monitor.snapshot(now=3.0)
    camera = device(report, "camera")
    assert camera.status == "ok"  # 10Hz >= 20*0.5

    monitor.configure_device("camera", min_rate_ratio=0.8)
    report = monitor.snapshot(now=3.0)
    camera = device(report, "camera")
    assert camera.status == "degraded"
    assert report.status == "degraded"

    with pytest.raises(ValueError):
        monitor.configure_device("camera", topic="/other")
    with pytest.raises(ValueError):
        monitor.configure_device("camera", expected_rate_hz=-1)
    with pytest.raises(KeyError):
        monitor.configure_device("nope", required=True)
