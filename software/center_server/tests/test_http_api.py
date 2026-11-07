import base64
import io
import json
import threading

import pytest
import urllib.request
from PIL import Image

from center_server.clients import CenterServerClient, CenterServerError
from center_server.config import load_config
from center_server.http_api import HttpApi
from center_server.service import InferenceService
from center_server.server import create_server


def _stub_config():
    cfg = load_config(None)
    cfg["model"].update(
        {
            "backend": "stub",
            "stub_action": [0.2, 0.0, 0.0],
            "image_keys": ["camera1"],
            "image_aliases": {"front": "camera1"},
            "require_images": False,
            "state_dim": 3,
            "chunk_size": 1,
        }
    )
    cfg["action"].update({"max_linear_x": 0.5, "max_linear_y": 0.4})
    return cfg


def _png_b64():
    image = Image.new("RGB", (8, 8), (0, 255, 0))
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


@pytest.fixture()
def service():
    svc = InferenceService(_stub_config())
    yield svc
    svc.close()


@pytest.fixture()
def api(service):
    return HttpApi(service, _stub_config())


def _json_body(payload):
    return json.dumps(payload).encode("utf-8")


def test_health_and_model(api):
    status, payload, headers = api.dispatch("GET", "/v1/health")
    assert status == 200
    assert payload["ready"] is True
    assert payload["model"]["backend"] == "stub"
    assert headers["X-Request-ID"]

    status, payload, _ = api.dispatch("GET", "/v1/model")
    assert status == 200
    assert payload["model"]["action_mode"] == "base_twist"


def test_infer_without_images(api):
    body = _json_body(
        {"instruction": "前进", "images": {}, "state": [0.0, 0.0, 0.0]}
    )
    status, payload, _ = api.dispatch(
        "POST",
        "/v1/infer",
        {"content-type": "application/json"},
        body,
        request_id="req-1",
    )
    assert status == 200
    assert payload["request_id"] == "req-1"
    assert payload["base_cmd"]["linear"]["x"] == pytest.approx(0.1)
    assert payload["warnings"] == []


def test_infer_with_alias_image(api):
    body = _json_body(
        {
            "instruction": "go",
            "images": {"front": _png_b64()},
            "state": [0.0, 0.0, 0.0],
        }
    )
    status, payload, _ = api.dispatch(
        "POST", "/v1/infer", {"content-type": "application/json"}, body
    )
    assert status == 200
    assert payload["ok"] is True


def test_missing_instruction(api):
    status, payload, _ = api.dispatch(
        "POST", "/v1/infer", {"content-type": "application/json"}, b"{}"
    )
    assert status == 422
    assert payload["error"]["code"] == "instruction_required"


def test_unknown_image_key(api):
    body = _json_body({"instruction": "go", "images": {"back": _png_b64()}})
    status, payload, _ = api.dispatch(
        "POST", "/v1/infer", {"content-type": "application/json"}, body
    )
    assert status == 422
    assert payload["error"]["code"] == "unknown_image_key"


def test_invalid_json(api):
    status, payload, _ = api.dispatch(
        "POST", "/v1/infer", {"content-type": "application/json"}, b"{bad"
    )
    assert status == 400
    assert payload["error"]["code"] == "invalid_json"


def test_method_not_allowed(api):
    status, payload, _ = api.dispatch("GET", "/v1/infer")
    assert status == 405
    assert payload["error"]["code"] == "method_not_allowed"


def test_options(api):
    status, _, headers = api.dispatch("OPTIONS", "/v1/infer")
    assert status == 204
    assert headers["Access-Control-Allow-Methods"] == "GET,POST,OPTIONS"


def test_payload_too_large():
    cfg = _stub_config()
    cfg["server"]["max_body_bytes"] = 1024
    svc = InferenceService(cfg)
    api = HttpApi(svc, cfg)
    try:
        status, payload, _ = api.dispatch(
            "POST",
            "/v1/infer",
            {"content-type": "application/json", "content-length": "2048"},
            b"{}",
        )
        assert status == 413
        assert payload["error"]["code"] == "payload_too_large"
    finally:
        svc.close()


def test_api_key_auth():
    cfg = _stub_config()
    cfg["server"]["api_key"] = "secret"
    svc = InferenceService(cfg)
    api = HttpApi(svc, cfg)
    body = _json_body({"instruction": "go", "images": {}})
    try:
        status, _, _ = api.dispatch("GET", "/v1/health")
        assert status == 200

        status, payload, _ = api.dispatch(
            "POST", "/v1/infer", {"content-type": "application/json"}, body
        )
        assert status == 401
        assert payload["error"]["code"] == "unauthorized"

        status, _, _ = api.dispatch(
            "POST",
            "/v1/infer",
            {
                "content-type": "application/json",
                "authorization": "Bearer secret",
            },
            body,
        )
        assert status == 200
    finally:
        svc.close()


def test_socket_server_and_client():
    cfg = _stub_config()
    svc = InferenceService(cfg)
    api = HttpApi(svc, cfg)
    httpd = create_server(api, "127.0.0.1", 0)
    port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/v1/health", timeout=3
        ) as response:
            assert response.status == 200

        client = CenterServerClient(f"http://127.0.0.1:{port}", timeout_sec=3)
        health = client.health()
        assert health["ready"] is True

        model = client.model()
        assert model["model"]["backend"] == "stub"

        result = client.infer(
            "前进",
            {"front": _png_b64()},
            state=[0.0, 0.0, 0.0],
            task_id="task-1",
            chunk_size=1,
        )
        assert result["ok"] is True
        assert result["task_id"] == "task-1"
        assert result["base_cmd"]["linear"]["x"] == pytest.approx(0.1)

        assert client.reset("session-1")["ok"] is True

        with pytest.raises(CenterServerError) as error:
            client.infer("前进", {"front": "not-base64"})
        assert error.value.status == 400
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=3)
        svc.close()
