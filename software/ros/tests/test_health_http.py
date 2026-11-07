import json
import urllib.error
import urllib.request

import pytest

from center_server.http_health import HealthHttpServer


def sample_payload(*, ready=True, emergency_stop=False):
    return {
        "ok": True,
        "status": "ok" if ready else "degraded",
        "healthy": ready,
        "ready": ready,
        "service": "rush-mog-ros-health",
        "version": "0.2.0",
        "uptime_sec": 12.5,
        "timestamp": "2026-11-07T15:00:00.000Z",
        "reason": "ok" if ready else "speaker: optional device not present",
        "summary": {
            "total": 2,
            "enabled": 2,
            "ok": 1,
            "degraded": 0,
            "error": 0,
            "unknown": 1,
            "disabled": 0,
        },
        "emergency_stop": emergency_stop,
        "safety": {"available": True, "emergency_stop": emergency_stop},
        "ros": {"ok": True, "missing_nodes": []},
        "http": {"enabled": True, "bound": True},
        "devices": [
            {
                "id": "camera",
                "type": "camera",
                "status": "ok",
                "healthy": True,
                "ready": True,
                "enabled": True,
                "required": True,
                "connected": True,
                "data_fresh": True,
                "topic": "/camera/image_raw",
                "message_type": "sensor_msgs/msg/Image",
                "publisher_count": 1,
                "age_sec": 0.05,
                "rate_hz": 14.9,
                "expected_rate_hz": 15.0,
                "detail": "ok",
                "metrics": {"width": 640, "height": 480},
            },
            {
                "id": "speaker",
                "type": "speaker",
                "status": "unknown",
                "healthy": False,
                "ready": False,
                "enabled": True,
                "required": False,
                "connected": False,
                "data_fresh": False,
                "topic": "/body/audio_status",
                "message_type": "std_msgs/msg/String",
                "publisher_count": 0,
                "age_sec": -1.0,
                "rate_hz": -1.0,
                "expected_rate_hz": 0.0,
                "detail": "optional device not present",
                "metrics": {},
            },
        ],
    }


def start_server(provider, **kwargs):
    server = HealthHttpServer("127.0.0.1", 0, provider, **kwargs)
    assert server.start(), server.info()
    return server


def http_get(server, path, headers=None):
    url = f"http://127.0.0.1:{server.info()['port']}{path}"
    request = urllib.request.Request(url, headers=headers or {})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=2.0) as response:
            return response.status, json.loads(response.read()), dict(response.headers)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read()), dict(exc.headers)


def http_post(server, path):
    url = f"http://127.0.0.1:{server.info()['port']}{path}"
    request = urllib.request.Request(url, data=b"{}", method="POST")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=2.0) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


@pytest.fixture
def running_server():
    state = {"payload": sample_payload()}
    server = start_server(lambda: state["payload"])
    try:
        yield server, state
    finally:
        server.stop()


def test_liveness_and_root(running_server):
    server, _ = running_server

    status, payload, headers = http_get(server, "/ros/healthz")
    assert status == 200
    assert payload["ok"] is True
    assert payload["status"] == "ok"
    assert headers["X-Request-ID"]

    status, payload, _ = http_get(server, "/")
    assert status == 200
    assert payload["endpoints"]["health"] == "GET /ros/health"
    assert payload["endpoints"]["liveness"] == "GET /ros/healthz"


def test_ready_probe(running_server):
    server, state = running_server
    status, payload, _ = http_get(server, "/ros/readyz")
    assert status == 200
    assert payload["status"] == "ready"

    state["payload"] = sample_payload(ready=False)
    status, payload, _ = http_get(server, "/ros/readyz")
    assert status == 503
    assert payload["ok"] is False
    assert payload["status"] == "not_ready"
    assert "speaker" in payload["reason"]


def test_detail_and_alias(running_server):
    server, _ = running_server

    status, payload, headers = http_get(server, "/ros/health")
    assert status == 200
    assert payload["status"] == "ok"
    assert len(payload["devices"]) == 2
    assert payload["devices"][0]["metrics"]["width"] == 640
    assert headers["X-Request-ID"] == payload["request_id"]

    status, payload, _ = http_get(server, "/v1/health")
    assert status == 200
    assert payload["service"] == "rush-mog-ros-health"

    status, payload, _ = http_get(server, "/healthcheck")
    assert status == 200
    assert payload["status"] == "ok"


def test_device_endpoint(running_server):
    server, _ = running_server

    status, payload, _ = http_get(server, "/ros/health/devices/camera")
    assert status == 200
    assert payload["device"]["id"] == "camera"

    status, payload, _ = http_get(server, "/v1/health/speaker")
    assert status == 200
    assert payload["device"]["status"] == "unknown"

    status, payload, _ = http_get(server, "/ros/health/devices/nope")
    assert status == 404
    assert payload["error"]["code"] == "unknown_device"


def test_include_metrics_toggle(running_server):
    server, _ = running_server

    status, payload, _ = http_get(server, "/ros/health?include_metrics=false")
    assert status == 200
    assert "metrics" not in payload["devices"][0]

    status, payload, _ = http_get(server, "/ros/health/devices/camera?include_metrics=0")
    assert status == 200
    assert "metrics" not in payload["device"]


def test_optional_auth():
    server = start_server(lambda: sample_payload(), api_key="secret")
    try:
        status, payload, _ = http_get(server, "/ros/health")
        assert status == 401
        assert payload["error"]["code"] == "unauthorized"

        status, payload, _ = http_get(
            server, "/ros/health", {"Authorization": "Bearer secret"}
        )
        assert status == 200
        assert payload["status"] == "ok"

        status, payload, _ = http_get(server, "/ros/health", {"X-API-Key": "wrong"})
        assert status == 401

        status, payload, _ = http_get(server, "/ros/health", {"X-API-Key": "secret"})
        assert status == 200

        # 存活/就绪探针不要求鉴权
        status, payload, _ = http_get(server, "/ros/healthz")
        assert status == 200
        assert payload["ok"] is True
    finally:
        server.stop()


def test_unknown_path_and_method(running_server):
    server, _ = running_server

    status, payload, _ = http_get(server, "/nope")
    assert status == 404
    assert payload["error"]["code"] == "not_found"

    status, payload = http_post(server, "/ros/health")
    assert status == 405
    assert payload["error"]["code"] == "method_not_allowed"


def test_no_snapshot_is_not_ready():
    server = start_server(lambda: None)
    try:
        status, payload, _ = http_get(server, "/ros/healthz")
        assert status == 200

        status, payload, _ = http_get(server, "/ros/readyz")
        assert status == 503
        assert payload["status"] == "not_ready"

        status, payload, _ = http_get(server, "/ros/health")
        assert status == 503
        assert payload["error"]["code"] == "not_ready"

        status, payload, _ = http_get(server, "/ros/health/devices/camera")
        assert status == 503
        assert payload["error"]["code"] == "not_ready"
    finally:
        server.stop()
