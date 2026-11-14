"""Edge HTTP/API 与 MJPEG 测试（stdlib fake runtime，不依赖 ROS）。"""

import http.client
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from edge_server.edge_web import EdgeHttpApp, EdgeHttpServer, MjpegServer


class FakeRuntime:
    started_monotonic = time.monotonic()
    ip_address = "127.0.0.1"

    def __init__(self):
        self.commands = []
        self.params = {"stream_fps": 10, "motion_keepalive_ms": 200}
        self.speaker_actions = []
        self.speaker_payload = {
            "ok": True,
            "enabled": True,
            "state": "connected",
            "status": "ok",
            "connected": True,
            "audio_ready": True,
            "device": {
                "name": "newmine",
                "mac": "73:DB:39:F2:15:4B",
                "paired": True,
                "trusted": True,
            },
            "sink": {
                "name": "bluez_output.73_DB_39_F2_15_4B.1",
                "id": 42,
                "ready": True,
                "volume": 70,
            },
            "tts": {"backend": "espeak-ng", "voice": "cmn", "speed": 150},
            "playback": {
                "playing": False,
                "queue": 0,
                "last_text": "",
                "last_error": "",
                "last_played_ms": 0,
            },
            "last_error": "",
        }
        self.microphone_actions = []
        self.microphone_payload = {
            "ok": True,
            "enabled": True,
            "supported": True,
            "available": True,
            "online": True,
            "recording": False,
            "source": "bluez_input.73_DB_39_F2_15_4B.headset-head-unit",
            "sample_rate": 8000,
            "channels": 1,
            "duration_ms": 0,
            "file": "",
            "file_name": "",
            "last_file": "",
            "last_file_name": "",
            "last_duration_ms": 1200,
            "volume": 100,
            "error": "",
        }
        self.mic_file = ""
        self.power_payload = {
            "ok": True,
            "enabled": True,
            "available": True,
            "source": "rpi",
            "model": "Raspberry Pi 4 Model B Rev 1.4",
            "status": "ok",
            "healthy": True,
            "online": True,
            "undervoltage": False,
            "throttled_raw": "0x50000",
            "undervoltage_occurred": True,
            "warning": True,
            "core_voltage_v": 0.85,
            "temperature_c": 39.4,
        }
        self.body_payload = {
            "valid": True,
            "stale": False,
            "seq": 3,
            "uptime_ms": 1000,
            "age_ms": 20,
            "wheel_mm_s": [10, 20, 30, 40],
            "distance_mm": 500,
        }

    def status(self):
        return {
            "uptime_ms": 12345,
            "free_heap": 1000,
            "free_psram": 0,
            "camera": True,
            "sdcard": False,
            "motion_active": False,
            "serial_connected": True,
            "ros_status": "ok",
        }

    def edge_module(self):
        return {
            "id": "edge",
            "name": "Edge Server",
            "version": "0.3.0",
            "ip": self.ip_address,
            "status": "ok",
            "summary": "serial=on, ros=ok",
            "params": dict(self.params, camera=True, sdcard=False),
            "body": self.body_payload,
        }

    def body(self):
        return self.body_payload

    def motion_snapshot(self):
        return {
            "active": False,
            "command": "",
            "manual": False,
            "source": "idle",
            "age_ms": -1,
            "keepalive_ms": 200,
        }

    def events(self, limit=40):
        return {
            "ok": True,
            "count": 1,
            "events": [
                {
                    "id": 1,
                    "ts_ms": 1700000000000,
                    "kind": "command",
                    "text": "指令已下发",
                    "command": "1",
                    "source": "web",
                    "status": "ok",
                }
            ],
            "server_time_ms": 1700000000001,
        }

    def edge_params(self):
        return {
            "ok": True,
            "module": "edge",
            "name": "Edge Server",
            "params": [
                {
                    "key": "stream_fps",
                    "label": "视频流帧率",
                    "value": self.params["stream_fps"],
                    "type": "int",
                    "group": "视频",
                    "editable": True,
                }
            ],
        }

    def update_params(self, updates):
        updated = {}
        for key, value in updates.items():
            if key not in {"stream_fps", "motion_keepalive_ms"}:
                raise ValueError(f"参数不可修改: {key}")
            self.params[key] = int(value)
            updated[key] = self.params[key]
        if not updated:
            raise ValueError("没有可更新的参数")
        return {"ok": True, "updated": updated, "message": "边缘参数已保存"}

    def handle_command(self, message):
        self.commands.append(message)
        if message.startswith("button:stop") or message == "raw:0":
            return "0"
        return "1"

    def speaker_status(self):
        return dict(self.speaker_payload)

    def handle_speaker(self, action, fields):
        self.speaker_actions.append((action, dict(fields)))
        if action == "say":
            if not str(fields.get("text", "")).strip():
                raise ValueError("播报文本为空")
            return {
                "ok": True,
                "message": "语音已加入播放队列",
                "speaker": self.speaker_payload,
            }
        if action == "volume":
            value = int(str(fields.get("volume", "")))
            if value < 0 or value > 100:
                raise ValueError("volume 必须在 0..100")
            self.speaker_payload["sink"]["volume"] = value
            return {
                "ok": True,
                "message": "音量已更新",
                "speaker": self.speaker_payload,
            }
        return {
            "ok": True,
            "message": f"{action} ok",
            "speaker": self.speaker_payload,
        }

    def power_status(self):
        return dict(self.power_payload)

    def microphone_status(self):
        return dict(self.microphone_payload)

    def handle_microphone(self, action, fields):
        self.microphone_actions.append((action, dict(fields)))
        if action == "start":
            self.microphone_payload["recording"] = True
            return {
                "ok": True,
                "message": "录音已开始",
                "microphone": self.microphone_payload,
            }
        if action == "stop":
            self.microphone_payload["recording"] = False
            return {
                "ok": True,
                "message": "录音已停止",
                "microphone": self.microphone_payload,
            }
        if action == "list":
            return {
                "ok": True,
                "count": 1,
                "recordings": [
                    {
                        "name": "mic-test.wav",
                        "size": 4,
                        "duration_ms": 1200,
                        "modified_ms": 1700000000000,
                    }
                ],
            }
        if action == "volume":
            value = int(str(fields.get("volume", "")))
            if value < 0 or value > 100:
                raise ValueError("volume 必须在 0..100")
            self.microphone_payload["volume"] = value
            return {
                "ok": True,
                "message": "麦克风音量已更新",
                "microphone": self.microphone_payload,
            }
        return {
            "ok": True,
            "message": f"{action} ok",
            "microphone": self.microphone_payload,
        }

    def microphone_recording(self, name):
        if name != "mic-test.wav" or not self.mic_file:
            raise ValueError("录音文件不存在")
        return self.mic_file

    def local_modules(self):
        return [
            {
                "id": "ros",
                "name": "ROS 2 (Edge)",
                "version": "0.2.0",
                "status": "ok",
                "summary": "all devices ok",
                "health_url": "http://127.0.0.1:8090/ros/health",
                "params_url": "http://127.0.0.1:8090/ros/params",
                "updated_ms": 1,
                "age_ms": 0,
                "stale": False,
                "params": {},
            }
        ]


class FakeModuleHandler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        self._send({"ok": True, "module": "center", "path": self.path})

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(length).decode("utf-8") if length else ""
        self._send({"ok": True, "module": "center", "path": self.path, "body": body})

    def _send(self, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):  # noqa: A003
        pass


def start_fake_module():
    server = ThreadingHTTPServer(("127.0.0.1", 0), FakeModuleHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def request(port, path, method="GET", data=None, headers=None):
    url = f"http://127.0.0.1:{port}{path}"
    body = data
    if isinstance(body, str):
        body = body.encode("utf-8")
    req = urllib.request.Request(url, data=body, method=method)
    for key, value in (headers or {}).items():
        req.add_header(key, value)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=3.0) as response:
            return response.status, dict(response.headers), response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, dict(exc.headers), exc.read()


@pytest.fixture()
def web_server(tmp_path):
    (tmp_path / "index.html").write_text("<html>edge</html>", encoding="utf-8")
    (tmp_path / "dashboard.js").write_text("console.log(1);", encoding="utf-8")
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "app.css").write_text("body{}", encoding="utf-8")
    runtime = FakeRuntime()
    app = EdgeHttpApp(
        runtime,
        static_root=str(tmp_path),
        stream_port=18080,
    )
    server = EdgeHttpServer("127.0.0.1", 0, app)
    assert server.start(), server.info()
    try:
        yield server.info()["port"], runtime
    finally:
        server.stop()


def test_static_and_status(web_server):
    port, _ = web_server
    status, headers, body = request(port, "/")
    assert status == 200
    assert body == b"<html>edge</html>"
    assert "text/html" in headers["Content-Type"]

    status, headers, body = request(port, "/dashboard.js")
    assert status == 200
    assert headers["Cache-Control"] == "no-cache"

    status, _, body = request(port, "/sub/app.css")
    assert status == 200
    assert body == b"body{}"

    status, _, body = request(port, "/missing.html")
    assert status == 404

    status, _, body = request(port, "/status")
    payload = json.loads(body)
    assert status == 200
    assert payload["uptime_ms"] == 12345
    assert payload["serial_connected"] is True


def test_body_and_modules(web_server):
    port, _ = web_server
    status, _, body = request(port, "/api/body")
    assert status == 200
    payload = json.loads(body)
    assert payload["body"]["wheel_mm_s"] == [10, 20, 30, 40]
    assert payload["motion"]["source"] == "idle"

    status, _, body = request(port, "/api/events?limit=5")
    assert status == 200
    events_payload = json.loads(body)
    assert events_payload["count"] == 1
    assert events_payload["events"][0]["source"] == "web"

    status, _, body = request(port, "/api/modules")
    payload = json.loads(body)
    assert status == 200
    assert payload["edge"]["id"] == "edge"
    assert [item["id"] for item in payload["modules"]] == ["ros"]

    status, _, body = request(port, "/api/modules/edge")
    assert status == 200
    assert json.loads(body)["status"] == "ok"

    status, _, body = request(port, "/api/modules/edge/params")
    assert status == 200
    assert json.loads(body)["params"][0]["key"] == "stream_fps"


def test_commands_and_stop(web_server):
    port, runtime = web_server
    status, _, _ = request(
        port,
        "/cmd",
        method="POST",
        data="button:up:down",
        headers={"Content-Type": "text/plain"},
    )
    assert status == 200
    assert runtime.commands[-1] == "button:up:down"

    status, _, _ = request(
        port,
        "/cmd-button",
        method="POST",
        data="key=up&type=up",
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    assert status == 200
    assert runtime.commands[-1] == "button:up:up"

    status, _, _ = request(
        port,
        "/cmd-stick",
        method="POST",
        data="stickId=left-analog&direct=up&stepValue=5",
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    assert status == 200
    assert runtime.commands[-1] == "stick:left-analog:up:5"

    status, _, body = request(port, "/stop", method="POST")
    assert status == 200
    assert body == b"stopped"
    assert runtime.commands[-1] == "button:stop:down"


def test_legacy_get_controls(web_server):
    port, runtime = web_server
    status, _, _ = request(port, "/cmd-button?key=up&type=down")
    assert status == 200
    assert runtime.commands[-1] == "button:up:down"

    status, _, _ = request(port, "/cmd-stick?stickId=left-analog&direct=left&stepValue=4")
    assert status == 200
    assert runtime.commands[-1] == "stick:left-analog:left:4"

    status, _, body = request(port, "/stop")
    assert status == 200
    assert body == b"stopped"
    assert runtime.commands[-1] == "button:stop:down"

    status, _, _ = request(port, "/cmd-button?key=up")
    assert status == 400


def test_edge_params_update(web_server):
    port, runtime = web_server
    status, _, body = request(
        port,
        "/api/modules/edge/params",
        method="POST",
        data=json.dumps({"updates": {"stream_fps": 12}}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    assert json.loads(body)["updated"]["stream_fps"] == 12
    assert runtime.params["stream_fps"] == 12

    status, _, body = request(
        port,
        "/api/modules/edge/params",
        method="POST",
        data=json.dumps({"updates": {"unknown": 1}}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 400
    assert json.loads(body)["error"]["code"] == "invalid_param"


def test_report_and_proxy(web_server):
    port, _ = web_server
    fake = start_fake_module()
    try:
        fake_port = fake.server_address[1]
        form = urllib.parse.urlencode(
            {
                "module": "center",
                "name": "CENTER",
                "version": "0.1.0",
                "status": "ok",
                "summary": "inference ready",
                "health_port": fake_port,
                "health_path": "/v1/health",
                "params_port": fake_port,
                "params_path": "/v1/params",
                "params_json": '{"model":"smolvla"}',
            }
        )
        status, _, body = request(
            port,
            "/api/modules/report",
            method="POST",
            data=form,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )
        assert status == 200
        assert json.loads(body)["module"] == "center"

        status, _, body = request(port, "/api/modules")
        payload = json.loads(body)
        ids = [item["id"] for item in payload["modules"]]
        assert "center" in ids and "ros" in ids

        status, _, body = request(port, "/api/modules/center/health")
        assert status == 200
        assert json.loads(body)["module"] == "center"

        status, _, body = request(port, "/api/modules/center/params")
        assert status == 200
        assert json.loads(body)["path"] == "/v1/params"
    finally:
        fake.shutdown()
        fake.server_close()


def test_report_rejects_reserved_module(web_server):
    port, _ = web_server
    form = urllib.parse.urlencode({"module": "ros", "status": "ok"})
    status, _, body = request(
        port,
        "/api/modules/report",
        method="POST",
        data=form,
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    assert status == 400
    assert json.loads(body)["ok"] is False


def test_stream_redirect(web_server):
    port, _ = web_server
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3.0)
    try:
        connection.request("GET", "/stream")
        response = connection.getresponse()
        assert response.status == 302
        assert response.getheader("Location") == "http://127.0.0.1:18080/stream"
    finally:
        connection.close()


def test_power_api(web_server):
    port, _ = web_server
    status, _, body = request(port, "/api/power")
    payload = json.loads(body)
    assert status == 200
    assert payload["status"] == "ok"
    assert payload["model"].startswith("Raspberry Pi 4")
    assert payload["warning"] is True


def test_microphone_api(web_server, tmp_path):
    port, runtime = web_server
    status, _, body = request(port, "/api/microphone")
    payload = json.loads(body)
    assert status == 200
    assert payload["sample_rate"] == 8000
    assert payload["available"] is True

    status, _, body = request(
        port,
        "/api/microphone/start",
        method="POST",
        data=json.dumps({"duration": 5}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    assert json.loads(body)["microphone"]["recording"] is True
    assert runtime.microphone_actions[-1] == ("start", {"duration": "5"})

    status, _, body = request(port, "/api/microphone/stop", method="POST")
    assert status == 200
    assert json.loads(body)["microphone"]["recording"] is False

    status, _, body = request(
        port,
        "/api/microphone/volume",
        method="POST",
        data=json.dumps({"volume": 80}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    assert json.loads(body)["microphone"]["volume"] == 80

    status, _, body = request(port, "/api/microphone/recordings")
    payload = json.loads(body)
    assert status == 200
    assert payload["recordings"][0]["name"] == "mic-test.wav"

    recording = tmp_path / "mic-test.wav"
    recording.write_bytes(b"RIFFtest")
    runtime.mic_file = str(recording)
    status, headers, body = request(
        port, "/api/microphone/recordings/mic-test.wav"
    )
    assert status == 200
    assert body == b"RIFFtest"
    assert "audio/wav" in headers["Content-Type"]
    assert "attachment" in headers["Content-Disposition"]

    status, _, _ = request(port, "/api/microphone/recordings/missing.wav")
    assert status == 404

    status, _, _ = request(port, "/api/microphone/unknown", method="POST")
    assert status == 404


def test_speaker_api(web_server):
    port, runtime = web_server
    status, _, body = request(port, "/api/speaker")
    payload = json.loads(body)
    assert status == 200
    assert payload["state"] == "connected"
    assert payload["device"]["name"] == "newmine"
    assert payload["sink"]["ready"] is True

    status, _, body = request(
        port,
        "/api/speaker/say",
        method="POST",
        data=json.dumps({"text": "机器人已启动"}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    assert json.loads(body)["message"] == "语音已加入播放队列"
    assert runtime.speaker_actions[-1] == ("say", {"text": "机器人已启动"})

    # 也接受 text/plain，方便 curl
    status, _, _ = request(
        port,
        "/api/speaker/say",
        method="POST",
        data="请注意避让",
        headers={"Content-Type": "text/plain"},
    )
    assert status == 200
    assert runtime.speaker_actions[-1][1]["text"] == "请注意避让"

    status, _, body = request(
        port,
        "/api/speaker/volume",
        method="POST",
        data=json.dumps({"volume": 55}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 200
    assert json.loads(body)["speaker"]["sink"]["volume"] == 55

    status, _, body = request(
        port,
        "/api/speaker/say",
        method="POST",
        data=json.dumps({"text": ""}),
        headers={"Content-Type": "application/json"},
    )
    assert status == 400
    assert json.loads(body)["error"]["code"] == "invalid_speaker_request"

    status, _, _ = request(port, "/api/speaker/unknown", method="POST")
    assert status == 404


def test_speaker_api_unavailable():
    runtime = type(
        "NoSpeakerRuntime",
        (),
        {
            "started_monotonic": time.monotonic(),
            "ip_address": "127.0.0.1",
            "status": lambda self: {},
            "edge_module": lambda self: {},
            "body": lambda self: {},
            "local_modules": lambda self: [],
        },
    )()
    app = EdgeHttpApp(runtime, static_root="", stream_port=18080)
    server = EdgeHttpServer("127.0.0.1", 0, app)
    assert server.start(), server.info()
    try:
        port = server.info()["port"]
        status, _, body = request(port, "/api/speaker")
        assert status == 503
        assert json.loads(body)["error"]["code"] == "speaker_unavailable"
    finally:
        server.stop()


def test_mjpeg_available_and_unavailable():
    frame = b"\xff\xd8fakejpeg\xff\xd9"
    state = {"available": True, "seq": 1}
    server = MjpegServer(
        "127.0.0.1",
        0,
        lambda: (state["seq"], frame if state["available"] else None),
        lambda: 20,
        lambda: state["available"],
    )
    assert server.start(), server.info()
    port = server.info()["port"]
    try:
        # available=False -> 503
        state["available"] = False
        status, headers, _ = request(port, "/stream")
        assert status == 503

        state["available"] = True
        req = urllib.request.Request(f"http://127.0.0.1:{port}/stream")
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(req, timeout=2.0) as response:
            assert response.status == 200
            assert "multipart/x-mixed-replace" in response.headers["Content-Type"]
            chunk = response.read(64)
            assert b"--frame" in chunk
    finally:
        server.stop()
