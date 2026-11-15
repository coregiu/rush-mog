"""VLAEngine 远程 SmolVLA 链路测试（stdlib fake CENTER/Edge，不依赖 ROS）。"""

import base64
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest
from PIL import Image

from edge_server.core.center_client import (
    CenterClient,
    CenterClientError,
    base_url_from_registry,
    discover_center,
    encode_image_b64,
)
from edge_server.core.data_preprocessor import SyncedDTO
from edge_server.core.vla_engine import VLAEngine


def make_dto(instruction="前进", **overrides):
    dto = SyncedDTO(
        image=np.zeros((3, 8, 8), dtype=np.float32),
        laser=np.ones(10, dtype=np.float32),
        audio=np.zeros((64, 4), dtype=np.float32),
        odom={"x": 1.0, "y": 2.0, "yaw": 0.5},
        joint={"j1": 0.1, "j2": 0.2},
        task_id="task-1",
        instruction=instruction,
        image_raw=np.zeros((8, 8, 3), dtype=np.uint8),
    )
    for key, value in overrides.items():
        setattr(dto, key, value)
    return dto


class FakeCenter(BaseHTTPRequestHandler):
    infer_calls = []
    base_cmd = {
        "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    lock = threading.Lock()

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0") or 0)
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        with self.lock:
            FakeCenter.infer_calls.append((self.path, payload))
        body = json.dumps(
            {
                "ok": True,
                "base_cmd": FakeCenter.base_cmd,
                "action": [0.2, 0.0, 0.0],
                "actions": [[0.2, 0.0, 0.0]],
                "latency_ms": 1.0,
            }
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        body = json.dumps({"ok": True, "model": {}}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


class FakeRegistry(BaseHTTPRequestHandler):
    payload = {}

    def do_GET(self):  # noqa: N802
        if self.path.split("?")[0] != "/api/center":
            self.send_error(404)
            return
        body = json.dumps(FakeRegistry.payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


def _serve(handler):
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


@pytest.fixture()
def center():
    FakeCenter.infer_calls = []
    FakeCenter.base_cmd = {
        "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    server = _serve(FakeCenter)
    try:
        yield server, f"http://127.0.0.1:{server.server_address[1]}"
    finally:
        server.shutdown()
        server.server_close()


@pytest.fixture()
def registry():
    server = _serve(FakeRegistry)
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()


def _engine(remote):
    return VLAEngine(
        {
            "backend": "auto",
            "confidence_threshold": 0.6,
            "remote": remote,
        }
    )


# --------------------------------------------------------------------- mapping
@pytest.mark.parametrize(
    "base_cmd,intent",
    [
        ({"linear": {"x": 0.2, "y": 0.0}, "angular": {"z": 0.0}}, "forward"),
        ({"linear": {"x": -0.2, "y": 0.0}, "angular": {"z": 0.0}}, "backward"),
        ({"linear": {"x": 0.0, "y": 0.2}, "angular": {"z": 0.0}}, "strafe_left"),
        ({"linear": {"x": 0.0, "y": -0.2}, "angular": {"z": 0.0}}, "strafe_right"),
        ({"linear": {"x": 0.0, "y": 0.0}, "angular": {"z": 0.4}}, "turn_left"),
        ({"linear": {"x": 0.0, "y": 0.0}, "angular": {"z": -0.4}}, "turn_right"),
        ({"linear": {"x": 0.0, "y": 0.0}, "angular": {"z": 0.0}}, "stop"),
    ],
)
def test_remote_base_cmd_to_intent(center, base_cmd, intent):
    server, base_url = center
    FakeCenter.base_cmd = base_cmd
    engine = _engine(
        {
            "discovery_url": "",
            "host": base_url,
            "timeout_sec": 1.0,
            "state_joint_names": ["j1", "j2"],
        }
    )
    result = engine.predict(make_dto())
    assert result.source == "center"
    assert result.intent == intent
    assert result.confidence >= 0.6


def test_remote_builds_request_payload(center):
    server, base_url = center
    engine = _engine(
        {
            "discovery_url": "",
            "host": base_url,
            "timeout_sec": 1.0,
            "image_key": "front",
            "chunk_size": 1,
            "state_joint_names": ["j2", "j1"],
        }
    )
    engine.predict(make_dto("把红色杯子拿起来"))
    assert len(FakeCenter.infer_calls) == 1
    path, payload = FakeCenter.infer_calls[0]
    assert path == "/v1/infer"
    assert payload["instruction"] == "把红色杯子拿起来"
    assert payload["state"] == [1.0, 2.0, 0.5, 0.2, 0.1]
    assert payload["options"]["chunk_size"] == 1
    assert payload["task_id"] == "task-1"
    raw = base64.b64decode(payload["images"]["front"])
    assert raw[:2] == b"\xff\xd8"  # JPEG SOI


def test_remote_uses_discovery_registry(center, registry):
    center_server, base_url = center
    FakeRegistry.payload = {
        "ok": True,
        "reported": True,
        "module": "center",
        "base_url": base_url,
        "ip": "127.0.0.1",
        "port": 8000,
        "infer_url": f"{base_url}/v1/infer",
    }
    engine = _engine(
        {
            "discovery_url": f"http://127.0.0.1:{registry.server_address[1]}/api/center",
            "timeout_sec": 1.0,
        }
    )
    result = engine.predict(make_dto())
    assert result.source == "center"
    assert result.intent == "forward"
    assert engine.center_status()["base_url"] == base_url


def test_auto_falls_back_to_rules(registry):
    FakeRegistry.payload = {"ok": False}
    engine = _engine(
        {
            "discovery_url": f"http://127.0.0.1:{registry.server_address[1]}/api/center",
            "host": "127.0.0.1:9",  # 不可达
            "timeout_sec": 0.3,
            "discovery_timeout_sec": 0.2,
            "retry_after_sec": 5.0,
        }
    )
    result = engine.predict(make_dto("前进"))
    assert result.source == "rule"
    assert result.intent == "forward"


def test_remote_without_fallback_stops(registry):
    FakeRegistry.payload = {"ok": False}
    engine = VLAEngine(
        {
            "backend": "remote",
            "confidence_threshold": 0.6,
            "remote": {
                "discovery_url": "",
                "host": "127.0.0.1:9",
                "timeout_sec": 0.3,
                "fallback_to_rules": False,
            },
        }
    )
    result = engine.predict(make_dto("前进"))
    assert result.source == "center"
    assert result.intent == "stop"
    assert result.confidence == 0.0


def test_remote_uses_meta_image_key(center, registry):
    center_server, base_url = center
    FakeRegistry.payload = {
        "ok": True,
        "base_url": base_url,
        "meta": {
            "image_keys": ["camera1", "camera2"],
            "image_aliases": {"front": "camera1"},
        },
    }
    engine = _engine(
        {
            "discovery_url": f"http://127.0.0.1:{registry.server_address[1]}/api/center",
            "image_key": "front",
            "timeout_sec": 1.0,
        }
    )
    engine.predict(make_dto())
    _, payload = FakeCenter.infer_calls[0]
    assert list(payload["images"]) == ["camera1"]


def test_remote_falls_back_to_first_model_image_key(center, registry):
    center_server, base_url = center
    FakeRegistry.payload = {
        "ok": True,
        "base_url": base_url,
        "meta": {"image_keys": ["camera9"], "image_aliases": {}},
    }
    engine = _engine(
        {
            "discovery_url": f"http://127.0.0.1:{registry.server_address[1]}/api/center",
            "image_key": "front",
            "timeout_sec": 1.0,
        }
    )
    engine.predict(make_dto())
    _, payload = FakeCenter.infer_calls[0]
    assert list(payload["images"]) == ["camera9"]


def test_remote_failure_sets_degraded_window(center):
    server, base_url = center
    FakeCenter.base_cmd = {"linear": {}, "angular": {}}  # 合法但全 0
    engine = _engine(
        {"discovery_url": "", "host": base_url, "timeout_sec": 1.0}
    )
    result = engine.predict(make_dto())
    assert result.intent == "stop"
    assert engine.center_status()["degraded_until_ms"] == 0

    # 关闭 center 后再次推理：进入降级窗口并回退规则桩
    server.shutdown()
    server.server_close()
    result = engine.predict(make_dto("前进"))
    assert result.source == "rule"
    assert engine.center_status()["degraded_until_ms"] > 0


def test_stub_backend_never_calls_center(center, registry):
    server, base_url = center
    FakeRegistry.payload = {"ok": True, "base_url": base_url}
    engine = VLAEngine(
        {
            "backend": "stub",
            "confidence_threshold": 0.6,
            "remote": {
                "discovery_url": f"http://127.0.0.1:{registry.server_address[1]}/api/center"
            },
        }
    )
    engine.predict(make_dto("前进"))
    assert FakeCenter.infer_calls == []
    assert engine.center_status()["remote_enabled"] is False


# ---------------------------------------------------------------------- client
def test_encode_image_accepts_chw_and_rgb():
    bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    bgr[:, :, 0] = 255  # BGR 蓝
    encoded = encode_image_b64(bgr)
    image = Image.open(io.BytesIO(base64.b64decode(encoded)))
    assert image.size == (8, 8)

    chw = np.zeros((3, 8, 8), dtype=np.float32)
    chw[0, :, :] = 1.0  # R 通道
    encoded_rgb = encode_image_b64(chw, color_order="rgb")
    image = Image.open(io.BytesIO(base64.b64decode(encoded_rgb)))
    assert image.convert("RGB").getpixel((0, 0))[0] > 200


def test_center_client_rejects_empty_instruction(center):
    server, base_url = center
    client = CenterClient(base_url, timeout_sec=1.0)
    with pytest.raises(CenterClientError):
        client.infer("  ", {"front": np.zeros((4, 4, 3), dtype=np.uint8)})


def test_base_url_from_registry_variants():
    assert base_url_from_registry({"base_url": "http://10.0.0.1:8000/"}) == (
        "http://10.0.0.1:8000"
    )
    assert base_url_from_registry(
        {"scheme": "http", "ip": "10.0.0.2", "port": 9000}
    ) == "http://10.0.0.2:9000"
    assert base_url_from_registry(
        {"infer_url": "http://10.0.0.3:8000/v1/infer"}
    ) == "http://10.0.0.3:8000"
    assert base_url_from_registry({}) == ""


def test_discover_center_returns_none_on_404(registry):
    # 404/未握手时返回 None，而不是抛异常（VLA 节点继续用规则桩）
    assert discover_center(
        f"http://127.0.0.1:{registry.server_address[1]}/api/missing", timeout_sec=0.5
    ) is None
