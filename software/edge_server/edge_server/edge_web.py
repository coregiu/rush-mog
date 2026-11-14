"""Edge Server HTTP/API 与 MJPEG 服务（stdlib，不直接依赖 ROS）。

迁移自 ESP32 `edge_server/src/web.cpp`，保留原网页/API 契约：

- 静态页面、``/cmd``、``/cmd-button``、``/cmd-stick``、``/stop``、``/status``；
- ``/api/body``、``/api/modules``、``/api/modules/report``、
  ``/api/modules/{id}/health|params``；
- 80 端口 ``/stream`` 302 到 8080 MJPEG。

运行时（ROS 节点/测试 fake）通过 ``runtime`` 注入，HTTP 线程只读快照，
不直接调用 rclpy。接口契约见 ``software/api.yaml`` 和
``software/edge_server/README.md``。
"""

from __future__ import annotations

import copy
import ipaddress
import json
import logging
import mimetypes
import os
import posixpath
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

LOGGER = logging.getLogger("edge_server.edge_web")

_MODULE_ID = re.compile(r"^[A-Za-z0-9_-]{1,16}$")
_MODULE_STALE_SEC = 15.0
_MAX_FORM_BYTES = 8192
_MAX_PARAM_BYTES = 65536

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".wasm": "application/wasm",
    ".woff2": "font/woff2",
    ".bin": "application/octet-stream",
}


def _json_bytes(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(dict(payload), ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )


def _json_quote(value: Any) -> str:
    return json.dumps(str(value or ""), ensure_ascii=False)


def _now_ms() -> int:
    return int(time.monotonic() * 1000.0)


def _is_private_host(host: str) -> bool:
    name = str(host or "").strip().lower().strip("[]")
    if not name:
        return False
    if name in {"localhost", "localhost.localdomain"}:
        return True
    try:
        address = ipaddress.ip_address(name)
    except ValueError:
        return False
    return bool(
        address.is_loopback
        or address.is_private
        or address.is_link_local
        or address.is_unspecified
    )


def _valid_module_url(url: str) -> Optional[str]:
    """只允许 http(s) 且指向本机/内网模块，避免页面代理成 SSRF。"""
    text = str(url or "").strip()
    if not text or len(text) > 512:
        return None
    parsed = urllib.parse.urlsplit(text)
    if parsed.scheme not in {"http", "https"}:
        return None
    if not parsed.hostname or not _is_private_host(parsed.hostname):
        return None
    path = parsed.path or "/"
    if ".." in path or len(path) > 120:
        return None
    return text


@dataclass
class ModuleRecord:
    id: str
    name: str
    version: str
    status: str
    summary: str
    params_json: str
    health_url: str
    params_url: str
    updated_at: float = field(default_factory=time.monotonic)

    def to_dict(self, now: Optional[float] = None, include_params: bool = True) -> dict:
        moment = time.monotonic() if now is None else float(now)
        age = max(0.0, moment - self.updated_at)
        payload = {
            "id": self.id,
            "name": self.name,
            "version": self.version,
            "status": self.status,
            "summary": self.summary,
            "health_url": self.health_url,
            "params_url": self.params_url,
            "updated_ms": int(self.updated_at * 1000.0),
            "age_ms": int(age * 1000.0),
            "stale": age > _MODULE_STALE_SEC,
        }
        if include_params:
            payload["params"] = _parse_json_object(self.params_json)
        return payload


def _parse_json_object(text: str) -> dict:
    raw = str(text or "").strip()
    if len(raw) < 2 or raw[0] != "{":
        return {}
    try:
        parsed = json.loads(raw)
    except (TypeError, ValueError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


class EdgeHttpApp:
    """HTTP 路由/模块注册表；数据由 ``runtime`` 提供。"""

    def __init__(
        self,
        runtime: Any,
        *,
        static_root: str = "",
        sdcard_root: str = "",
        stream_port: int = 8080,
        proxy_timeout_sec: float = 2.5,
    ) -> None:
        self.runtime = runtime
        self.static_root = str(static_root or "")
        self.sdcard_root = str(sdcard_root or "")
        self.stream_port = int(stream_port)
        self.proxy_timeout_sec = max(0.2, float(proxy_timeout_sec))
        self._modules: Dict[str, ModuleRecord] = {}
        self._modules_lock = threading.Lock()

    # ------------------------------------------------------------- helpers
    @staticmethod
    def error_payload(code: str, message: str) -> dict:
        return {"ok": False, "error": {"code": str(code), "message": str(message)}}

    def server_time_ms(self) -> int:
        started = getattr(self.runtime, "started_monotonic", None)
        if started is None:
            return _now_ms()
        return int(max(0.0, time.monotonic() - float(started)) * 1000.0)

    # ------------------------------------------------------------- registry
    def report_module(self, fields: Mapping[str, Any], remote_ip: str) -> dict:
        module_id = str(fields.get("module", "")).strip()
        if not _MODULE_ID.match(module_id) or module_id in {"edge", "ros"}:
            raise ValueError("module 非法或为保留值 edge/ros")
        name = str(fields.get("name", "") or module_id).strip()[:48]
        version = str(fields.get("version", "") or "").strip()[:24]
        status = str(fields.get("status", "") or "unknown").strip()[:16] or "unknown"
        summary = str(fields.get("summary", "") or "").strip()[:240]
        params_json = str(fields.get("params_json", "") or "{}")
        if len(params_json) > 2048:
            params_json = "{}"
        params_json = json.dumps(_parse_json_object(params_json), ensure_ascii=False)
        health_port = _int_or(fields.get("health_port"), 0)
        params_port = _int_or(fields.get("params_port"), 0)
        if health_port <= 0:
            health_port = 8000 if module_id == "center" else 8090
        if params_port <= 0:
            params_port = health_port
        if not (1 <= health_port <= 65535 and 1 <= params_port <= 65535):
            raise ValueError("端口必须在 1..65535")
        health_path = str(fields.get("health_path", "") or "").strip()
        params_path = str(fields.get("params_path", "") or "").strip()
        if not health_path:
            health_path = "/v1/health" if module_id == "center" else "/ros/health"
        if not params_path:
            params_path = "/v1/params" if module_id == "center" else "/ros/params"
        for path in (health_path, params_path):
            if not path.startswith("/") or ".." in path or len(path) > 80:
                raise ValueError("health_path/params_path 非法")

        host = str(remote_ip or "").strip()
        if not host or host == "0.0.0.0":
            raise ValueError("无法获取模块 IP")
        url_host = f"[{host}]" if ":" in host and not host.startswith("[") else host
        health_url = _valid_module_url(
            f"http://{url_host}:{health_port}{health_path}"
        )
        params_url = _valid_module_url(
            f"http://{url_host}:{params_port}{params_path}"
        )
        if health_url is None or params_url is None:
            raise ValueError("模块地址必须是本机/内网地址")

        record = ModuleRecord(
            id=module_id,
            name=name,
            version=version,
            status=status,
            summary=summary,
            params_json=params_json,
            health_url=health_url,
            params_url=params_url,
        )
        with self._modules_lock:
            if len(self._modules) >= 8 and module_id not in self._modules:
                raise RuntimeError("模块注册表已满")
            self._modules[module_id] = record
        return {
            "ok": True,
            "module": module_id,
            "ip": host,
            "health_url": health_url,
            "params_url": params_url,
        }

    def local_modules(self) -> List[dict]:
        provider = getattr(self.runtime, "local_modules", None)
        if provider is None:
            return []
        try:
            modules = provider()
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("local module provider failed: %s", exc)
            return []
        result = []
        for item in modules or []:
            if isinstance(item, dict) and item.get("id"):
                result.append(copy.deepcopy(item))
        return result

    def find_module(self, module_id: str) -> Optional[dict]:
        for module in self.local_modules():
            if str(module.get("id")) == module_id:
                return module
        with self._modules_lock:
            record = self._modules.get(module_id)
            if record is None:
                return None
            return record.to_dict(include_params=True)

    def modules_payload(self) -> dict:
        return {
            "ok": True,
            "server_time_ms": self.server_time_ms(),
            "edge": self.runtime.edge_module(),
            "modules": self.local_modules(),
            "count": len(self.local_modules()),
        }

    def modules_payload_with_reported(self) -> dict:
        """``/api/modules``：本地 ROS 子系统 + 外部上报模块（CENTER）。"""
        local = self.local_modules()
        with self._modules_lock:
            reported = [
                record.to_dict(include_params=True)
                for record in self._modules.values()
            ]
        modules = reported + local
        return {
            "ok": True,
            "server_time_ms": self.server_time_ms(),
            "edge": self.runtime.edge_module(),
            "modules": modules,
            "count": len(modules),
        }

    # ------------------------------------------------------------- params
    def edge_params_payload(self) -> dict:
        return self.runtime.edge_params()

    def update_edge_params(self, updates: Mapping[str, Any]) -> dict:
        return self.runtime.update_params(updates)

    # ------------------------------------------------------------- proxy
    def proxy_module(
        self,
        module_id: str,
        kind: str,
        *,
        method: str = "GET",
        body: bytes = b"",
        content_type: str = "",
    ) -> Tuple[int, dict]:
        module = self.find_module(module_id)
        if module is None:
            return 404, self.error_payload(
                "module_not_reported", f"模块尚未上报: {module_id}"
            )
        url = module.get("health_url") if kind == "health" else module.get("params_url")
        valid_url = _valid_module_url(str(url or ""))
        if valid_url is None:
            return 400, self.error_payload(
                "invalid_module_url", "模块接口地址缺失或不在内网"
            )
        headers = {"Accept": "application/json"}
        data = None
        if method == "POST":
            if not body:
                return 400, self.error_payload("empty_body", "缺少参数请求体")
            data = bytes(body)
            headers["Content-Type"] = content_type or "application/json"
        request = urllib.request.Request(
            valid_url, data=data, headers=headers, method=method
        )
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(request, timeout=self.proxy_timeout_sec) as response:
                payload = response.read(1024 * 1024)
                status = int(response.status)
        except urllib.error.HTTPError as exc:
            payload = exc.read(1024 * 1024)
            status = int(exc.code)
        except Exception as exc:  # noqa: BLE001
            return 502, self.error_payload(
                "module_unreachable", f"模块未响应: {module_id} ({exc})"
            )
        try:
            parsed = json.loads(payload.decode("utf-8")) if payload else {}
        except (UnicodeDecodeError, ValueError):
            return 502, self.error_payload(
                "invalid_module_response", f"模块返回非 JSON: {module_id}"
            )
        return status, parsed if isinstance(parsed, dict) else {"ok": True}

    # ------------------------------------------------------------- control
    def control(self, message: str) -> Tuple[int, str]:
        text = str(message or "").strip()
        if not text:
            return 400, "empty command"
        if len(text) > 32:
            return 400, "command too long"
        handler = getattr(self.runtime, "handle_command", None)
        if handler is None:
            return 503, "control unavailable"
        try:
            command = handler(text)
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("control command failed")
            return 500, f"control failed: {exc}"
        if not command:
            return 200, "ignored"
        return 200, "ok"

    # ------------------------------------------------------------- speaker
    def speaker_payload(self) -> dict:
        handler = getattr(self.runtime, "speaker_status", None)
        if handler is None:
            raise RuntimeError("音箱功能未启用")
        return handler()

    def speaker_action(self, action: str, fields: Mapping[str, Any]) -> dict:
        handler = getattr(self.runtime, "handle_speaker", None)
        if handler is None:
            raise RuntimeError("音箱功能未启用")
        return handler(str(action or ""), fields)

    # ------------------------------------------------------------- microphone
    def microphone_payload(self) -> dict:
        handler = getattr(self.runtime, "microphone_status", None)
        if handler is None:
            raise RuntimeError("麦克风功能未启用")
        return handler()

    def microphone_action(self, action: str, fields: Mapping[str, Any]) -> dict:
        handler = getattr(self.runtime, "handle_microphone", None)
        if handler is None:
            raise RuntimeError("麦克风功能未启用")
        return handler(str(action or ""), fields)

    def microphone_recordings(self) -> dict:
        handler = getattr(self.runtime, "handle_microphone", None)
        if handler is None:
            raise RuntimeError("麦克风功能未启用")
        return handler("list", {})

    def microphone_recording(self, name: str) -> str:
        handler = getattr(self.runtime, "microphone_recording", None)
        if handler is None:
            raise RuntimeError("麦克风功能未启用")
        return str(handler(str(name or "")))

    # ------------------------------------------------------------- static
    def _safe_path(self, root: str, request_path: str) -> Optional[str]:
        if not root:
            return None
        cleaned = posixpath.normpath("/" + request_path.lstrip("/"))
        if cleaned.startswith("/..") or cleaned == "/..":
            return None
        relative = cleaned.lstrip("/")
        root_abs = os.path.abspath(root)
        candidate = os.path.abspath(os.path.join(root_abs, relative))
        try:
            if os.path.commonpath([candidate, root_abs]) != root_abs:
                return None
        except ValueError:
            return None
        # 保留符号链接：colcon --symlink-install 会把 share/.../web 下的
        # 文件指向源码树，realpath 会跑到 root 之外；词法路径已在 root 内。
        return candidate

    def static_file(self, request_path: str) -> Optional[Tuple[str, str]]:
        path = "/index.html" if request_path in {"", "/"} else request_path
        candidate = self._safe_path(self.static_root, path)
        if candidate and os.path.isfile(candidate):
            return candidate, _content_type(candidate)
        if self.sdcard_root and request_path.startswith("/sdcard/"):
            candidate = self._safe_path(self.sdcard_root, request_path[8:])
            if candidate and os.path.isfile(candidate):
                return candidate, _content_type(candidate)
        return None

    def stream_redirect(self, host_header: str) -> str:
        host = str(host_header or "").split(":", 1)[0].strip()
        if not host:
            host = str(getattr(self.runtime, "ip_address", "") or "127.0.0.1")
        return f"http://{host}:{self.stream_port}/stream"


def _int_or(value: Any, default: int) -> int:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return default


def _content_type(path: str) -> str:
    suffix = os.path.splitext(path)[1].lower()
    return _CONTENT_TYPES.get(
        suffix, mimetypes.guess_type(path)[0] or "application/octet-stream"
    )


class EdgeHttpServer:
    """把 ``EdgeHttpApp`` 暴露到 TCP 端口。"""

    def __init__(self, host: str, port: int, app: EdgeHttpApp) -> None:
        self.host = str(host or "0.0.0.0")
        self.port = int(port)
        self.app = app
        self._server: Optional[_EdgeHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._bound_port = 0
        self._error = ""

    def start(self) -> bool:
        if self._server is not None:
            return True
        server = None
        try:
            server = _EdgeHTTPServer((self.host, self.port), _EdgeRequestHandler)
            server.edge_app = self.app
            self._server = server
            self._bound_port = int(server.server_address[1])
            self._thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.2},
                name="edge-http",
                daemon=True,
            )
            self._thread.start()
            LOGGER.info("edge HTTP listening on http://%s:%s", self.host, self._bound_port)
            return True
        except Exception as exc:  # noqa: BLE001
            self._error = f"{type(exc).__name__}: {exc}"
            self._server = None
            if server is not None:
                try:
                    server.server_close()
                except Exception:  # noqa: BLE001
                    pass
            LOGGER.error("edge HTTP bind failed on %s:%s: %s", self.host, self.port, exc)
            return False

    def stop(self) -> None:
        server = self._server
        self._server = None
        if server is not None:
            try:
                server.shutdown()
            except Exception:  # noqa: BLE001
                pass
            try:
                server.server_close()
            except Exception:  # noqa: BLE001
                pass
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def info(self) -> dict:
        return {
            "enabled": True,
            "host": self.host,
            "port": self._bound_port or self.port,
            "configured_port": self.port,
            "bound": self._server is not None,
            "error": self._error,
        }


class _EdgeHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    edge_app: EdgeHttpApp


class _EdgeRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "RushMogEdge/1.0"

    # ------------------------------------------------------------- verbs
    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send(204, b"", "text/plain; charset=utf-8")

    def do_HEAD(self) -> None:  # noqa: N802
        self._handle_get(head_only=True)

    def do_GET(self) -> None:  # noqa: N802
        self._handle_get(head_only=False)

    def do_POST(self) -> None:  # noqa: N802
        app = self._app()
        path = self._path()
        if path in {"/cmd", "/cmd-button", "/cmd-stick"}:
            body = self._read_body()
            if body is None:
                return
            if path == "/cmd":
                text = self._command_text(body)
            else:
                fields = self._parse_fields(body)
                if path == "/cmd-button":
                    key = str(fields.get("key", "") or "")
                    event = str(fields.get("type", "") or "")
                    if not key or not event:
                        self._send_json(
                            400,
                            app.error_payload(
                                "bad_request", "Missing 'key' or 'type' parameter"
                            ),
                        )
                        return
                    text = f"button:{key}:{event}"
                else:
                    stick = str(fields.get("stickId", "") or "")
                    direct = str(fields.get("direct", "") or "")
                    step = str(fields.get("stepValue", "") or "")
                    if not stick or not direct or not step:
                        self._send_json(
                            400,
                            app.error_payload(
                                "bad_request",
                                "Missing 'stickId', 'direct' or 'stepValue' parameter",
                            ),
                        )
                        return
                    text = f"stick:{stick}:{direct}:{step}"
            status, message = app.control(text)
            self._send_text(status, message)
            return

        if path == "/stop":
            app.control("button:stop:down")
            self._send_text(200, "stopped")
            return

        if path.startswith("/api/speaker"):
            action = path[len("/api/speaker") :].strip("/")
            if action not in {"connect", "say", "play", "volume", "stop"}:
                self._send_json(
                    404,
                    app.error_payload("not_found", "未知音箱操作"),
                )
                return
            body = self._read_body()
            if body is None:
                return
            fields = self._parse_fields(body)
            if action == "say" and not fields.get("text"):
                raw_text = body.decode("utf-8", "replace").strip()
                if raw_text and not raw_text.startswith("{") and "=" not in raw_text:
                    fields["text"] = raw_text
            try:
                payload = app.speaker_action(action, fields)
            except ValueError as exc:
                self._send_json(
                    400, app.error_payload("invalid_speaker_request", str(exc))
                )
                return
            except RuntimeError as exc:
                self._send_json(
                    503, app.error_payload("speaker_unavailable", str(exc))
                )
                return
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("speaker action failed")
                self._send_json(
                    500, app.error_payload("speaker_failed", f"音箱操作失败: {exc}")
                )
                return
            self._send_json(200, payload)
            return

        if path.startswith("/api/microphone"):
            action = path[len("/api/microphone") :].strip("/")
            if action not in {"start", "stop", "play", "volume", "list"}:
                self._send_json(
                    404,
                    app.error_payload("not_found", "未知麦克风操作"),
                )
                return
            body = self._read_body()
            if body is None:
                return
            fields = self._parse_fields(body)
            try:
                payload = app.microphone_action(action, fields)
            except ValueError as exc:
                self._send_json(
                    400, app.error_payload("invalid_microphone_request", str(exc))
                )
                return
            except RuntimeError as exc:
                self._send_json(
                    503, app.error_payload("microphone_unavailable", str(exc))
                )
                return
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("microphone action failed")
                self._send_json(
                    500, app.error_payload("microphone_failed", f"麦克风操作失败: {exc}")
                )
                return
            self._send_json(200, payload)
            return

        if path == "/api/modules/report":
            body = self._read_body()
            if body is None:
                return
            fields = self._parse_fields(body)
            remote = self.client_address[0] if self.client_address else ""
            try:
                payload = app.report_module(fields, remote)
            except ValueError as exc:
                self._send_json(400, app.error_payload("invalid_module", str(exc)))
                return
            except RuntimeError as exc:
                self._send_json(507, app.error_payload("registry_full", str(exc)))
                return
            self._send_json(200, payload)
            return

        if path == "/api/modules/edge/params":
            body = self._read_body()
            if body is None:
                return
            updates = self._parse_updates(body)
            if updates is None:
                return
            try:
                result = app.update_edge_params(updates)
            except ValueError as exc:
                self._send_json(400, app.error_payload("invalid_param", str(exc)))
                return
            self._send_json(200, result)
            return

        if path.startswith("/api/modules/") and path.endswith("/params"):
            module_id = path[len("/api/modules/") : -len("/params")]
            body = self._read_body(_MAX_PARAM_BYTES)
            if body is None:
                return
            status, payload = app.proxy_module(
                module_id,
                "params",
                method="POST",
                body=body,
                content_type=str(self.headers.get("Content-Type", "") or ""),
            )
            self._send_json(status, payload)
            return

        self.close_connection = True
        self._send_json(
            405, app.error_payload("method_not_allowed", "不支持的 POST 路径")
        )

    # ------------------------------------------------------------- GET
    def _handle_get(self, *, head_only: bool) -> None:
        app = self._app()
        path = self._path()
        parsed = urllib.parse.urlsplit(self.path or "/")

        if path in {"/cmd-button", "/cmd-stick", "/stop"}:
            if path == "/stop":
                app.control("button:stop:down")
                self._send_text(200, "stopped", head_only=head_only)
                return
            query = urllib.parse.parse_qs(parsed.query, keep_blank_values=True)

            def field(name: str) -> str:
                values = query.get(name) or [""]
                return values[-1]

            if path == "/cmd-button":
                key = field("key")
                event = field("type")
                if not key or not event:
                    self._send_text(
                        400,
                        "400 Bad Request - Missing 'key' or 'type' parameter",
                        head_only=head_only,
                    )
                    return
                text = f"button:{key}:{event}"
            else:
                stick = field("stickId")
                direct = field("direct")
                step = field("stepValue")
                if not stick or not direct or not step:
                    self._send_text(
                        400,
                        "400 Bad Request - Missing 'stickId', 'direct' or 'stepValue' parameter",
                        head_only=head_only,
                    )
                    return
                text = f"stick:{stick}:{direct}:{step}"
            status, message = app.control(text)
            self._send_text(status, message, head_only=head_only)
            return

        if path == "/status":
            self._send_json(200, app.runtime.status(), head_only=head_only)
            return

        if path == "/api/body":
            payload = {"ok": True, "body": app.runtime.body()}
            motion = getattr(app.runtime, "motion_snapshot", None)
            if motion is not None:
                payload["motion"] = motion()
            self._send_json(200, payload, head_only=head_only)
            return

        if path == "/api/events":
            handler = getattr(app.runtime, "events", None)
            if handler is None:
                self._send_json(
                    404, app.error_payload("not_found", "事件记录不可用"), head_only
                )
                return
            query = urllib.parse.parse_qs(parsed.query, keep_blank_values=True)
            raw_limit = (query.get("limit") or ["40"])[-1]
            try:
                limit = int(raw_limit)
            except (TypeError, ValueError):
                limit = 40
            self._send_json(200, handler(limit), head_only=head_only)
            return

        if path == "/api/speaker":
            try:
                payload = app.speaker_payload()
            except RuntimeError as exc:
                self._send_json(
                    503,
                    app.error_payload("speaker_unavailable", str(exc)),
                    head_only=head_only,
                )
                return
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("speaker status failed")
                self._send_json(
                    500,
                    app.error_payload("speaker_failed", f"音箱状态不可用: {exc}"),
                    head_only=head_only,
                )
                return
            self._send_json(200, payload, head_only=head_only)
            return

        if path == "/api/microphone":
            try:
                payload = app.microphone_payload()
            except RuntimeError as exc:
                self._send_json(
                    503,
                    app.error_payload("microphone_unavailable", str(exc)),
                    head_only=head_only,
                )
                return
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("microphone status failed")
                self._send_json(
                    500,
                    app.error_payload(
                        "microphone_failed", f"麦克风状态不可用: {exc}"
                    ),
                    head_only=head_only,
                )
                return
            self._send_json(200, payload, head_only=head_only)
            return

        if path == "/api/microphone/recordings":
            try:
                payload = app.microphone_recordings()
            except RuntimeError as exc:
                self._send_json(
                    503,
                    app.error_payload("microphone_unavailable", str(exc)),
                    head_only=head_only,
                )
                return
            self._send_json(200, payload, head_only=head_only)
            return

        if path.startswith("/api/microphone/recordings/"):
            name = urllib.parse.unquote(path[len("/api/microphone/recordings/") :])
            try:
                filename = app.microphone_recording(name)
            except ValueError as exc:
                self._send_json(
                    404,
                    app.error_payload("recording_not_found", str(exc)),
                    head_only=head_only,
                )
                return
            except RuntimeError as exc:
                self._send_json(
                    503,
                    app.error_payload("microphone_unavailable", str(exc)),
                    head_only=head_only,
                )
                return
            self._send_file(
                filename,
                "audio/wav",
                filename=os.path.basename(filename),
                head_only=head_only,
            )
            return

        if path == "/api/modules":
            self._send_json(200, app.modules_payload_with_reported(), head_only=head_only)
            return

        if path in {"/api/modules/edge", "/api/modules/edge/health"}:
            self._send_json(200, app.runtime.edge_module(), head_only=head_only)
            return

        if path == "/api/modules/edge/params":
            self._send_json(200, app.edge_params_payload(), head_only=head_only)
            return

        if path.startswith("/api/modules/"):
            rest = path[len("/api/modules/") :]
            module_id = rest
            kind = ""
            if rest.endswith("/health"):
                module_id = rest[: -len("/health")]
                kind = "health"
            elif rest.endswith("/params"):
                module_id = rest[: -len("/params")]
                kind = "params"
            if not _MODULE_ID.match(module_id):
                self._send_json(
                    404, app.error_payload("unknown_module", "未知模块"), head_only
                )
                return
            if kind:
                status, payload = app.proxy_module(module_id, kind)
                self._send_json(status, payload, head_only=head_only)
                return
            module = app.find_module(module_id)
            if module is None:
                self._send_json(
                    404,
                    app.error_payload(
                        "module_not_reported", f"模块尚未上报: {module_id}"
                    ),
                    head_only,
                )
                return
            self._send_json(200, module, head_only=head_only)
            return

        if path == "/stream":
            location = app.stream_redirect(
                str(self.headers.get("Host", "") or "")
            )
            self.send_response(302)
            self.send_header("Location", location)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        if path.startswith("/api/"):
            self._send_json(
                404, app.error_payload("not_found", "未知 API 路径"), head_only
            )
            return

        found = app.static_file(path)
        if found is None:
            self._send_text(404, "File Not Found", head_only=head_only)
            return
        filename, content_type = found
        try:
            with open(filename, "rb") as handle:
                data = handle.read()
        except OSError:
            self._send_text(500, "Failed to open file", head_only=head_only)
            return
        no_cache = path.endswith(".html") or path in {
            "/dashboard.js",
            "/dashboard.css",
        }
        self._send_bytes(
            200,
            data,
            content_type,
            head_only=head_only,
            cache="no-cache" if no_cache else "public, max-age=36000",
        )

    # ------------------------------------------------------------- request
    def _read_body(self, max_bytes: int = _MAX_FORM_BYTES) -> Optional[bytes]:
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            length = 0
        if length < 0 or length > int(max_bytes):
            self.close_connection = True
            self._send_json(
                413, self._app().error_payload("payload_too_large", "请求体过大")
            )
            return None
        return self.rfile.read(length) if length > 0 else b""

    def _parse_fields(self, body: bytes) -> Dict[str, str]:
        content_type = str(self.headers.get("Content-Type", "") or "").lower()
        query = urllib.parse.parse_qs(
            urllib.parse.urlsplit(self.path or "/").query, keep_blank_values=True
        )
        fields = {
            key: values[-1] if values else ""
            for key, values in query.items()
        }
        if "application/json" in content_type:
            try:
                payload = json.loads(body.decode("utf-8")) if body else {}
            except (UnicodeDecodeError, ValueError):
                payload = {}
            if isinstance(payload, dict):
                fields.update({str(k): str(v) for k, v in payload.items()})
            return fields
        if body:
            form = urllib.parse.parse_qs(
                body.decode("utf-8", "replace"), keep_blank_values=True
            )
            for key, values in form.items():
                fields[str(key)] = values[-1] if values else ""
        return fields

    def _parse_updates(self, body: bytes) -> Optional[Dict[str, Any]]:
        content_type = str(self.headers.get("Content-Type", "") or "").lower()
        if "application/json" in content_type or not content_type:
            try:
                payload = json.loads(body.decode("utf-8")) if body else {}
            except (UnicodeDecodeError, ValueError):
                self._send_json(
                    400,
                    self._app().error_payload("invalid_body", "JSON 解析失败"),
                )
                return None
            candidate = payload.get("updates", payload) if isinstance(payload, dict) else None
            if not isinstance(candidate, dict):
                self._send_json(
                    400,
                    self._app().error_payload("invalid_body", "updates 必须是对象"),
                )
                return None
            return candidate
        fields = self._parse_fields(body)
        return fields or None

    @staticmethod
    def _command_text(body: bytes) -> str:
        text = body.decode("utf-8", "replace").strip()
        if not text:
            return ""
        if "=" in text and ":" not in text:
            form = urllib.parse.parse_qs(text, keep_blank_values=True)
            if len(form) == 1:
                key = next(iter(form))
                return urllib.parse.unquote_plus(key).strip()
        return text

    def _path(self) -> str:
        path = urllib.parse.urlsplit(self.path or "/").path or "/"
        if len(path) > 1:
            path = path.rstrip("/")
        return path

    def _app(self) -> EdgeHttpApp:
        return self.server.edge_app  # type: ignore[attr-defined]

    # ------------------------------------------------------------- response
    def _send_json(
        self, status: int, payload: Mapping[str, Any], head_only: bool = False
    ) -> None:
        self._send_bytes(
            status,
            _json_bytes(payload),
            "application/json; charset=utf-8",
            head_only=head_only,
            cache="no-store",
        )

    def _send_text(
        self, status: int, text: str, head_only: bool = False
    ) -> None:
        self._send_bytes(
            status,
            str(text).encode("utf-8"),
            "text/plain; charset=utf-8",
            head_only=head_only,
            cache="no-store",
        )

    def _send(
        self, status: int, body: bytes, content_type: str
    ) -> None:
        self._send_bytes(status, body, content_type)

    def _send_file(
        self,
        path: str,
        content_type: str,
        *,
        filename: str = "",
        head_only: bool = False,
    ) -> None:
        try:
            with open(path, "rb") as handle:
                body = handle.read()
        except OSError:
            self._send_text(500, "Failed to open file", head_only=head_only)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Access-Control-Allow-Origin", "*")
        if filename:
            quoted = urllib.parse.quote(filename)
            self.send_header(
                "Content-Disposition",
                f"attachment; filename={quoted}; filename*=UTF-8''{quoted}",
            )
        self.end_headers()
        if body and not head_only:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def _send_bytes(
        self,
        status: int,
        body: bytes,
        content_type: str,
        *,
        head_only: bool = False,
        cache: str = "no-store",
    ) -> None:
        self.send_response(int(status))
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,HEAD,POST,OPTIONS")
        self.send_header(
            "Access-Control-Allow-Headers",
            "Content-Type,Authorization,X-API-Key,X-Request-ID",
        )
        self.end_headers()
        if body and not head_only:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def log_message(self, fmt: str, *args: Any) -> None:
        LOGGER.debug("%s - %s", self.address_string(), fmt % args)


class MjpegServer:
    """把最新 ROS 相机 JPEG 帧暴露为 ``multipart/x-mixed-replace``。"""

    def __init__(
        self,
        host: str,
        port: int,
        snapshot_provider: Callable[[], Tuple[int, Optional[bytes]]],
        fps_provider: Callable[[], int],
        available_provider: Callable[[], bool],
    ) -> None:
        self.host = str(host or "0.0.0.0")
        self.port = int(port)
        self._snapshot = snapshot_provider
        self._fps = fps_provider
        self._available = available_provider
        self._server: Optional[_MjpegHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._bound_port = 0
        self._error = ""

    def start(self) -> bool:
        if self._server is not None:
            return True
        server = None
        try:
            server = _MjpegHTTPServer((self.host, self.port), _MjpegRequestHandler)
            server.mjpeg_app = self
            self._server = server
            self._bound_port = int(server.server_address[1])
            self._thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.2},
                name="edge-mjpeg",
                daemon=True,
            )
            self._thread.start()
            LOGGER.info(
                "edge MJPEG listening on http://%s:%s/stream",
                self.host,
                self._bound_port,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self._error = f"{type(exc).__name__}: {exc}"
            self._server = None
            if server is not None:
                try:
                    server.server_close()
                except Exception:  # noqa: BLE001
                    pass
            LOGGER.error(
                "edge MJPEG bind failed on %s:%s: %s", self.host, self.port, exc
            )
            return False

    def stop(self) -> None:
        server = self._server
        self._server = None
        if server is not None:
            try:
                server.shutdown()
            except Exception:  # noqa: BLE001
                pass
            try:
                server.server_close()
            except Exception:  # noqa: BLE001
                pass
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def info(self) -> dict:
        return {
            "enabled": True,
            "host": self.host,
            "port": self._bound_port or self.port,
            "configured_port": self.port,
            "bound": self._server is not None,
            "error": self._error,
        }

    def snapshot(self) -> Tuple[int, Optional[bytes]]:
        return self._snapshot()

    def available(self) -> bool:
        return bool(self._available())

    def fps(self) -> int:
        try:
            return max(1, min(30, int(self._fps())))
        except Exception:  # noqa: BLE001
            return 10


class _MjpegHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    mjpeg_app: MjpegServer


class _MjpegRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "RushMogEdgeMjpeg/1.0"

    def do_GET(self) -> None:  # noqa: N802
        app = self.server.mjpeg_app  # type: ignore[attr-defined]
        path = urllib.parse.urlsplit(self.path or "/").path or "/"
        if path != "/stream":
            self._simple(404, "Not Found")
            return
        if not app.available():
            self.send_response(503)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            return

        seq, frame = app.snapshot()
        if not frame:
            self.send_response(503)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            return

        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                seq, frame = app.snapshot()
                if not frame:
                    break
                if seq != last_seq:
                    header = (
                        "--frame\r\n"
                        "Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(frame)}\r\n\r\n"
                    ).encode("ascii")
                    self.wfile.write(header)
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_seq = seq
                time.sleep(1.0 / max(1, app.fps()))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        self.close_connection = True

    def _simple(self, status: int, text: str) -> None:
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt: str, *args: Any) -> None:
        LOGGER.debug("%s - %s", self.address_string(), fmt % args)
