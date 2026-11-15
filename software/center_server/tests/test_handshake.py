import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from center_server.config import ConfigError, load_config
from center_server.handshake import HandshakeAnnouncer
from center_server.http_api import HttpApi
from center_server.service import InferenceService


def _config(**handshake_overrides):
    cfg = load_config(None)
    cfg["model"].update(
        {
            "backend": "stub",
            "stub_action": [0.2, 0.0, 0.0],
            "image_keys": ["camera1"],
            "require_images": False,
            "state_dim": 3,
        }
    )
    cfg["handshake"].update(handshake_overrides)
    return cfg


def _announcer(cfg=None, **kwargs):
    cfg = cfg or _config(
        enabled=True,
        edge_url="http://127.0.0.1:9",
        public_host="192.168.4.50",
    )
    service = kwargs.pop("service", None) or InferenceService(cfg)
    announcer = HandshakeAnnouncer(
        cfg,
        status_provider=service.status,
        params_provider=service.params,
        model_provider=service.model_info,
        port=8000,
        **kwargs,
    )
    return announcer, service


# --------------------------------------------------------------------- payload
def test_payload_reports_ip_port_and_endpoints():
    cfg = _config(
        enabled=True,
        edge_url="http://192.168.3.26",
        public_host="192.168.4.50",
    )
    service = InferenceService(cfg)
    announcer = HandshakeAnnouncer(
        cfg,
        status_provider=service.status,
        params_provider=service.params,
        model_provider=service.model_info,
        port=8123,
    )
    try:
        payload = announcer._payload("startup")
        assert payload["module"] == "center"
        assert payload["ip"] == "192.168.4.50"
        assert payload["port"] == 8123
        assert payload["health_path"] == "/v1/health"
        assert payload["params_path"] == "/v1/params"
        assert payload["infer_path"] == "/v1/infer"
        assert payload["infer_port"] == 8123
        assert payload["handshake"] == "startup"
        assert "ready=True" in payload["summary"]
        assert payload["status"] == "ok"

        params = json.loads(payload["params_json"])
        assert params["action.mode"] == "base_twist"
        assert "action.max_linear_x" in params

        meta = json.loads(payload["meta_json"])
        assert meta["protocol"] == "http-json-v1"
        assert "infer" in meta["capabilities"]
        assert meta["model"]["backend"] == "stub"
        assert "camera1" in meta["image_keys"]
    finally:
        service.close()


def test_payload_marks_degraded_without_model_status():
    cfg = _config(enabled=True, edge_url="http://192.168.3.26")
    announcer = HandshakeAnnouncer(cfg, port=8000)
    payload = announcer._payload("heartbeat")
    assert payload["status"] == "degraded"
    assert payload["summary"].startswith("ready=False")


# -------------------------------------------------------------------- announce
def test_announce_now_tracks_success_and_sends_reason():
    sent = []

    def sender(url, payload, timeout, api_key):
        sent.append((url, dict(payload)))
        return 200, {"ok": True, "infer_url": "http://192.168.4.50:8000/v1/infer"}

    announcer, service = _announcer(sender=sender)
    try:
        result = announcer.announce_now("manual")
        assert result["ok"] is True
        assert sent and sent[0][0] == "http://127.0.0.1:9/api/center/handshake"
        assert sent[0][1]["handshake"] == "manual"
        status = announcer.status()
        assert status["connected"] is True
        assert status["attempts"] == 1
        assert status["successes"] == 1
        assert status["last_error"] == ""
        assert status["last_response"]["infer_url"].endswith("/v1/infer")
    finally:
        service.close()


def test_announce_now_records_failure():
    def sender(url, payload, timeout, api_key):
        raise RuntimeError("connection refused")

    announcer, service = _announcer(sender=sender)
    try:
        result = announcer.announce_now()
        assert result["ok"] is False
        assert "connection refused" in result["error"]
        status = announcer.status()
        assert status["connected"] is False
        assert status["failures"] == 1
        assert "connection refused" in status["last_error"]
    finally:
        service.close()


def test_disabled_announcer_does_not_send():
    calls = []

    def sender(*args, **kwargs):
        calls.append(args)
        return 200, {"ok": True}

    announcer, service = _announcer(
        cfg=_config(enabled=True, edge_url=""),
        sender=sender,
    )
    try:
        assert announcer.enabled is False
        assert announcer.start() is False
        assert announcer.announce_now()["ok"] is False
        assert calls == []
    finally:
        service.close()


def test_background_loop_retries_until_edge_is_up():
    attempts = {"count": 0}

    def sender(url, payload, timeout, api_key):
        attempts["count"] += 1
        if attempts["count"] < 3:
            raise RuntimeError("edge offline")
        return 200, {"ok": True}

    cfg = _config(
        enabled=True,
        edge_url="http://127.0.0.1:9",
        interval_sec=0.1,
        retry_sec=0.05,
        max_retry_sec=0.1,
    )
    announcer, service = _announcer(cfg=cfg, sender=sender)
    try:
        assert announcer.start() is True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not announcer.status()["connected"]:
            time.sleep(0.02)
        status = announcer.status()
        assert status["connected"] is True
        assert status["attempts"] >= 3
        assert status["failures"] >= 2
    finally:
        announcer.stop()
        service.close()


# ------------------------------------------------------------------ real HTTP
class _EdgeHandler(BaseHTTPRequestHandler):
    received = []
    lock = threading.Lock()

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(length)
        with self.lock:
            self.received.append((self.path, json.loads(body.decode("utf-8"))))
        payload = json.dumps(
            {
                "ok": True,
                "module": "center",
                "ip": "127.0.0.1",
                "infer_url": "http://127.0.0.1:8000/v1/infer",
            }
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *args):
        pass


@pytest.fixture()
def edge_http():
    _EdgeHandler.received = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), _EdgeHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server, f"http://127.0.0.1:{server.server_address[1]}"
    finally:
        server.shutdown()
        server.server_close()


def test_handshake_posts_json_to_edge(edge_http):
    server, url = edge_http
    cfg = _config(enabled=True, edge_url=url, public_host="192.168.4.50")
    announcer, service = _announcer(cfg=cfg)
    try:
        assert announcer.announce_now("startup")["ok"] is True
        assert len(_EdgeHandler.received) == 1
        path, payload = _EdgeHandler.received[0]
        assert path == "/api/center/handshake"
        assert payload["module"] == "center"
        assert payload["port"] == 8000
        assert payload["infer_port"] == 8000
        assert payload["ip"] == "192.168.4.50"
        status = announcer.status()
        assert status["connected"] is True
        assert status["last_response"]["infer_url"].endswith("/v1/infer")
    finally:
        service.close()


# --------------------------------------------------------------------- config
def test_config_rejects_bad_edge_url(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text(
        "handshake:\n  edge_url: 'ftp://192.168.3.26'\n", encoding="utf-8"
    )
    with pytest.raises(ConfigError):
        load_config(path)


def test_config_rejects_bad_interval(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text(
        "handshake:\n  edge_url: 'http://192.168.3.26'\n  retry_sec: 0\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError):
        load_config(path)


def test_config_env_override(monkeypatch):
    monkeypatch.setenv("CENTER_SERVER_EDGE_URL", "http://10.0.0.2")
    monkeypatch.setenv("CENTER_SERVER_HANDSHAKE_ENABLED", "false")
    cfg = load_config(None)
    assert cfg["handshake"]["edge_url"] == "http://10.0.0.2"
    assert cfg["handshake"]["enabled"] is False


def test_config_defaults_keep_handshake_disabled_without_edge():
    cfg = load_config(None)
    announcer = HandshakeAnnouncer(cfg, port=8000)
    assert announcer.enabled is False


# ------------------------------------------------------------------- HTTP API
class _FakeAnnouncer:
    def __init__(self, ok=True):
        self.ok = ok
        self.reason = None

    def status(self):
        return {
            "enabled": True,
            "target": "http://192.168.3.26/api/center/handshake",
            "connected": self.ok,
            "attempts": 1,
            "last_error": "" if self.ok else "edge offline",
        }

    def announce_now(self, reason="manual"):
        self.reason = reason
        return {"ok": self.ok, "reason": reason, **({} if self.ok else {"error": "edge offline"})}


@pytest.fixture()
def api():
    cfg = _config()
    service = InferenceService(cfg)
    try:
        yield lambda handshake=None: HttpApi(service, cfg, handshake=handshake), service
    finally:
        service.close()


def test_api_get_handshake_status(api):
    factory, _ = api
    api_obj = factory(_FakeAnnouncer())
    status, payload, _ = api_obj.dispatch("GET", "/v1/handshake")
    assert status == 200
    assert payload["handshake"]["connected"] is True


def test_api_post_handshake_triggers_announce(api):
    factory, _ = api
    announcer = _FakeAnnouncer()
    api_obj = factory(announcer)
    status, payload, _ = api_obj.dispatch("POST", "/v1/handshake", body=b"")
    assert status == 200
    assert payload["ok"] is True
    assert payload["result"]["reason"] == "manual"
    assert announcer.reason == "manual"


def test_api_post_handshake_ignores_content_type(api):
    factory, _ = api
    api_obj = factory(_FakeAnnouncer())
    status, payload, _ = api_obj.dispatch(
        "POST",
        "/v1/handshake",
        headers={"content-type": "application/x-www-form-urlencoded"},
        body=b"",
    )
    assert status == 200
    assert payload["ok"] is True


def test_api_post_handshake_failure_is_502(api):
    factory, _ = api
    api_obj = factory(_FakeAnnouncer(ok=False))
    status, payload, _ = api_obj.dispatch("POST", "/v1/handshake", body=b"")
    assert status == 502
    assert payload["ok"] is False
    assert payload["error"]["code"] == "handshake_failed"


def test_api_health_includes_handshake(api):
    factory, _ = api
    api_obj = factory(_FakeAnnouncer())
    status, payload, _ = api_obj.dispatch("GET", "/v1/health")
    assert status == 200
    assert payload["handshake"]["enabled"] is True


def test_api_without_announcer_reports_disabled(api):
    factory, _ = api
    api_obj = factory(None)
    status, payload, _ = api_obj.dispatch("GET", "/v1/handshake")
    assert status == 200
    assert payload["handshake"]["enabled"] is False
