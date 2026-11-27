"""CENTER -> Edge 握手注册表测试（stdlib fake runtime，不依赖 ROS）。"""

import json
import time
import urllib.error
import urllib.request

import pytest

from edge_server.edge_web import EdgeHttpApp, EdgeHttpServer


class FakeRuntime:
    started_monotonic = time.monotonic()
    ip_address = "127.0.0.1"

    def edge_module(self):
        return {
            "id": "edge",
            "name": "Edge Server",
            "version": "0.2.0",
            "status": "ok",
            "summary": "fake",
            "health_url": "http://127.0.0.1/health",
            "params_url": "http://127.0.0.1/api/modules/edge/params",
            "updated_ms": 0,
            "age_ms": 0,
            "stale": False,
            "params": {},
        }

    def local_modules(self):
        return []


def request(port, path, method="GET", data=None, headers=None):
    url = f"http://127.0.0.1:{port}{path}"
    body = data.encode("utf-8") if isinstance(data, str) else data
    req = urllib.request.Request(url, data=body, method=method)
    for key, value in (headers or {}).items():
        req.add_header(key, value)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=3.0) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


@pytest.fixture()
def web_server(tmp_path):
    app = EdgeHttpApp(FakeRuntime(), static_root=str(tmp_path))
    server = EdgeHttpServer("127.0.0.1", 0, app)
    assert server.start(), server.info()
    try:
        yield server.info()["port"], app
    finally:
        server.stop()


def handshake_payload(**overrides):
    payload = {
        "module": "center",
        "name": "SmolVLA Center Server",
        "version": "0.2.0",
        "status": "ok",
        "summary": "ready=True backend=smolvla device=cuda",
        "ip": "192.168.3.25",  # Edge 必须忽略它，以 TCP 对端地址为准
        "port": 8000,
        "scheme": "http",
        "health_port": 8000,
        "health_path": "/v1/health",
        "params_port": 8000,
        "params_path": "/v1/params",
        "infer_port": 8000,
        "infer_path": "/v1/infer",
        "params_json": '{"action.mode":"base_twist"}',
        "meta_json": '{"protocol":"http-json-v1","model":{"backend":"smolvla"}}',
        "handshake": "startup",
    }
    payload.update(overrides)
    return payload


def test_center_not_reported_returns_404(web_server):
    port, _ = web_server
    status, body = request(port, "/api/center")
    assert status == 404
    assert json.loads(body)["error"]["code"] == "center_not_reported"


def test_center_handshake_records_ip_port_and_infer_url(web_server):
    port, _ = web_server
    status, body = request(
        port,
        "/api/center/handshake",
        method="POST",
        data=json.dumps(handshake_payload()),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    result = json.loads(body)
    assert result["ok"] is True
    assert result["ip"] == "127.0.0.1"  # 以握手来源地址为准
    assert result["port"] == 8000
    assert result["infer_url"] == "http://127.0.0.1:8000/v1/infer"

    status, body = request(port, "/api/center")
    assert status == 200
    center = json.loads(body)
    assert center["ok"] is True
    assert center["reported"] is True
    assert center["ip"] == "127.0.0.1"
    assert center["port"] == 8000
    assert center["base_url"] == "http://127.0.0.1:8000"
    assert center["infer_url"].endswith("/v1/infer")
    assert center["health_url"].endswith("/v1/health")
    assert center["handshake"] == "startup"
    assert center["meta"]["protocol"] == "http-json-v1"
    assert center["meta"]["model"]["backend"] == "smolvla"
    assert center["stale"] is False


def test_center_handshake_accepts_form_encoding(web_server):
    port, _ = web_server
    import urllib.parse

    body = urllib.parse.urlencode(handshake_payload(handshake="manual"))
    status, raw = request(
        port,
        "/api/center/handshake",
        method="POST",
        data=body,
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    assert status == 200
    assert json.loads(raw)["module"] == "center"
    status, raw = request(port, "/api/center")
    assert json.loads(raw)["handshake"] == "manual"


def test_center_handshake_rejects_bad_port(web_server):
    port, _ = web_server
    status, raw = request(
        port,
        "/api/center/handshake",
        method="POST",
        data=json.dumps(handshake_payload(infer_port=70000)),
        headers={"Content-Type": "application/json"},
    )
    assert status == 400
    assert json.loads(raw)["error"]["code"] == "invalid_handshake"


def test_center_handshake_rejects_bad_path(web_server):
    port, _ = web_server
    status, raw = request(
        port,
        "/api/center/handshake",
        method="POST",
        data=json.dumps(handshake_payload(infer_path="../../etc/passwd")),
        headers={"Content-Type": "application/json"},
    )
    assert status == 400


def test_legacy_module_report_fills_center_infer_defaults(web_server):
    port, _ = web_server
    import urllib.parse

    body = urllib.parse.urlencode(
        {
            "module": "center",
            "name": "CENTER",
            "version": "0.1.0",
            "status": "ok",
            "health_port": 8000,
            "params_port": 8000,
        }
    )
    status, raw = request(
        port,
        "/api/modules/report",
        method="POST",
        data=body,
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    assert status == 200
    result = json.loads(raw)
    assert result["infer_url"] == "http://127.0.0.1:8000/v1/infer"

    status, raw = request(port, "/api/modules")
    assert status == 200
    modules = json.loads(raw)["modules"]
    center = next(item for item in modules if item["id"] == "center")
    assert center["ip"] == "127.0.0.1"
    assert center["port"] == 8000
    assert center["infer_url"].endswith("/v1/infer")


def test_report_module_rejects_public_ip():
    app = EdgeHttpApp(FakeRuntime())
    with pytest.raises(ValueError):
        app.report_module(
            {"module": "center", "health_port": 8000}, "8.8.8.8"
        )
