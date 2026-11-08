"""Healthcheck HTTP 网关（stdlib，无 ROS 依赖）。

`health_monitor_node` 内嵌本模块，把最新的健康快照以 HTTP/JSON 暴露给
同机 Edge Server/dashboard 等非 ROS 调用方。接口契约见 `software/ros/HEALTHCHECK.md`。

设计要点：
- HTTP 线程只读取节点预计算好的快照，不直接调用 rclpy，避免线程安全问题；
- 存活/就绪探针与详细健康分离；
- 路径同时支持 `/ros/*` 规范路径和 `/v1/*`、`/health*` 兼容别名；
- 可选 Bearer/X-API-Key；默认只监听 127.0.0.1。
"""

from __future__ import annotations

import copy
import hmac
import json
import logging
import re
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Mapping, Optional
from urllib.parse import parse_qs, urlsplit

from .core.health_monitor import SERVICE_NAME, SERVICE_VERSION

LOGGER = logging.getLogger("center_server.health_http")

_DEVICE_PATH = re.compile(r"^/(?:ros/health/devices|v1/health|healthcheck)/([^/]+)$")
_DETAIL_PATHS = {"/health", "/healthcheck", "/v1/health", "/ros/health"}
_PARAMS_PATHS = {"/ros/params", "/params"}
_READY_PATHS = {"/readyz", "/ros/readyz"}
_LIVE_PATHS = {"/healthz", "/livez", "/ros/healthz"}


class HealthHttpServer:
    """把 `provider()` 返回的健康快照暴露为 HTTP/JSON。"""

    def __init__(
        self,
        host: str,
        port: int,
        provider: Callable[[], Optional[Mapping[str, Any]]],
        *,
        api_key: str = "",
        cors_allow_origin: str = "*",
        service: str = SERVICE_NAME,
        version: str = SERVICE_VERSION,
        params_provider: Optional[
            Callable[[], Optional[Mapping[str, Any]]]
        ] = None,
        params_updater: Optional[
            Callable[[Mapping[str, Any]], Mapping[str, Any]]
        ] = None,
    ) -> None:
        self.host = str(host or "127.0.0.1")
        self.port = int(port)
        self._provider = provider
        self._params_provider = params_provider
        self._params_updater = params_updater
        self.api_key = str(api_key or "")
        self.cors_allow_origin = str(cors_allow_origin or "*")
        self.service = str(service)
        self.version = str(version)
        self._server: Optional[_HealthHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._error = ""
        self._bound_port = 0

    # ------------------------------------------------------------------ life
    def start(self) -> bool:
        if self._server is not None:
            return True
        server: Optional[_HealthHTTPServer] = None
        try:
            server = _HealthHTTPServer((self.host, self.port), _HealthRequestHandler)
            server.health_app = self
            self._server = server
            self._bound_port = int(server.server_address[1])
            self._thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.2},
                name="health-http",
                daemon=True,
            )
            self._thread.start()
            LOGGER.info(
                "health HTTP listening on http://%s:%s", self.host, self._bound_port
            )
            return True
        except Exception as exc:  # noqa: BLE001 - 端口占用不应拖垮 ROS 节点
            self._error = f"{type(exc).__name__}: {exc}"
            self._server = None
            if server is not None:
                try:
                    server.server_close()
                except Exception:  # noqa: BLE001
                    pass
            LOGGER.error("health HTTP bind failed on %s:%s: %s", self.host, self.port, exc)
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

    def info(self) -> Dict[str, Any]:
        return {
            "enabled": True,
            "host": self.host,
            "port": self._bound_port or self.port,
            "configured_port": self.port,
            "bound": self._server is not None,
            "error": self._error,
            "auth": bool(self.api_key),
        }

    # -------------------------------------------------------------- responses
    def authorized(self, headers: Mapping[str, str]) -> bool:
        if not self.api_key:
            return True
        authorization = str(headers.get("Authorization", ""))
        token = ""
        if authorization.lower().startswith("bearer "):
            token = authorization[7:].strip()
        if not token:
            token = str(headers.get("X-API-Key", "")).strip()
        return bool(token) and hmac.compare_digest(
            token.encode("utf-8"), self.api_key.encode("utf-8")
        )

    def live_payload(self, request_id: str) -> Dict[str, Any]:
        snapshot = self._snapshot() or {}
        return {
            "ok": True,
            "status": "ok",
            "service": self.service,
            "version": self.version,
            "uptime_sec": snapshot.get("uptime_sec"),
            "timestamp": snapshot.get("timestamp"),
            "request_id": request_id,
        }

    def ready_payload(self, request_id: str) -> tuple[bool, Dict[str, Any]]:
        snapshot = self._snapshot()
        if snapshot is None:
            return False, {
                "ok": False,
                "status": "not_ready",
                "reason": "health snapshot not available",
                "request_id": request_id,
            }
        ready = bool(snapshot.get("ready", False))
        return ready, {
            "ok": ready,
            "status": "ready" if ready else "not_ready",
            "reason": snapshot.get("reason", ""),
            "status_detail": snapshot.get("status", ""),
            "request_id": request_id,
        }

    def detail_payload(
        self, request_id: str, *, include_metrics: bool = True
    ) -> Optional[Dict[str, Any]]:
        snapshot = self._snapshot()
        if snapshot is None:
            return None
        if not include_metrics:
            for device in snapshot.get("devices") or []:
                device.pop("metrics", None)
        snapshot["request_id"] = request_id
        return snapshot

    def device_payload(
        self,
        device_id: str,
        request_id: str,
        *,
        include_metrics: bool = True,
    ) -> Optional[Dict[str, Any]]:
        snapshot = self._snapshot()
        if snapshot is None:
            return None
        wanted = str(device_id)
        for device in snapshot.get("devices") or []:
            if str(device.get("id")) != wanted:
                continue
            if not include_metrics:
                device.pop("metrics", None)
            return {
                "ok": True,
                "status": snapshot.get("status", ""),
                "device": device,
                "request_id": request_id,
            }
        return None

    def root_payload(self, request_id: str) -> Dict[str, Any]:
        return {
            "ok": True,
            "service": self.service,
            "version": self.version,
            "endpoints": {
                "health": "GET /ros/health",
                "health_alias": "GET /v1/health",
                "healthcheck_alias": "GET /healthcheck",
                "device": "GET /ros/health/devices/{device_id}",
                "params": "GET/POST /ros/params",
                "liveness": "GET /ros/healthz",
                "readiness": "GET /ros/readyz",
            },
            "request_id": request_id,
        }

    def has_snapshot(self) -> bool:
        return self._snapshot() is not None

    def params_supported(self) -> bool:
        return self._params_provider is not None

    def params_payload(self, request_id: str) -> Optional[Dict[str, Any]]:
        if self._params_provider is None:
            return None
        try:
            payload = self._params_provider()
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("health params provider failed: %s", exc)
            return None
        if payload is None:
            return None
        result = copy.deepcopy(dict(payload))
        result["request_id"] = request_id
        return result

    def update_params_payload(
        self,
        request_id: str,
        body: bytes,
        content_type: str,
    ) -> tuple[int, Dict[str, Any]]:
        if self._params_updater is None:
            return 404, self.error_payload(
                "params_not_supported", "该模块不支持参数修改", request_id
            )

        updates: Optional[Dict[str, Any]] = None
        if not body:
            return 400, self.error_payload(
                "empty_body", "缺少参数请求体", request_id
            )

        try:
            text = body.decode("utf-8")
            if "application/json" in (content_type or "").lower() or not content_type:
                parsed = json.loads(text)
                if not isinstance(parsed, dict):
                    raise ValueError("JSON 根节点必须是对象")
                candidate = parsed.get("updates", parsed)
                if not isinstance(candidate, dict):
                    raise ValueError("updates 必须是对象")
                updates = {str(key): value for key, value in candidate.items()}
            else:
                form = parse_qs(text, keep_blank_values=True)
                updates = {
                    str(key): values[-1] if values else ""
                    for key, values in form.items()
                }
        except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
            return 400, self.error_payload(
                "invalid_body", f"参数请求体解析失败: {exc}", request_id
            )

        if not updates:
            return 400, self.error_payload(
                "no_updates", "没有可更新的参数", request_id
            )

        try:
            result = self._params_updater(updates)
        except ValueError as exc:
            return 400, self.error_payload(
                "invalid_param", str(exc), request_id
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("health params update failed")
            return 500, self.error_payload(
                "update_failed", f"参数更新失败: {type(exc).__name__}", request_id
            )

        payload: Dict[str, Any] = {
            "ok": True,
            "module": "ros",
            "requires_restart": False,
        }
        if isinstance(result, Mapping):
            payload.update(dict(result))
        payload["request_id"] = request_id
        return 200, payload

    def error_payload(self, code: str, message: str, request_id: str) -> Dict[str, Any]:
        return {
            "ok": False,
            "error": {"code": str(code), "message": str(message)},
            "request_id": request_id,
        }

    # ---------------------------------------------------------------- helpers
    def _snapshot(self) -> Optional[Dict[str, Any]]:
        try:
            snapshot = self._provider()
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("health snapshot provider failed: %s", exc)
            return None
        if snapshot is None:
            return None
        return copy.deepcopy(dict(snapshot))


class _HealthHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    health_app: HealthHttpServer


class _HealthRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "RushMogHealth/1.0"

    # ------------------------------------------------------------- HTTP verbs
    def do_OPTIONS(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        app = self._app()
        self._send(app, 204, {}, request_id=self._request_id())

    def do_GET(self) -> None:  # noqa: N802
        self._handle_get()

    def do_HEAD(self) -> None:  # noqa: N802
        self._handle_get(head_only=True)

    def do_POST(self) -> None:  # noqa: N802
        app = self._app()
        request_id = self._request_id()
        path = urlsplit(self.path or "/").path or "/"
        if len(path) > 1:
            path = path.rstrip("/")

        if path in _PARAMS_PATHS:
            if not app.params_supported():
                self._send(
                    app,
                    404,
                    app.error_payload(
                        "params_not_supported", "该模块不支持参数修改", request_id
                    ),
                    request_id,
                )
                return
            if not app.authorized(self.headers):
                self._send(
                    app,
                    401,
                    app.error_payload(
                        "unauthorized", "无效或缺失的 API Key", request_id
                    ),
                    request_id,
                )
                return
            try:
                length = int(self.headers.get("Content-Length", "0") or 0)
            except ValueError:
                length = 0
            if length > 65536:
                self.close_connection = True
                self._send(
                    app,
                    413,
                    app.error_payload(
                        "payload_too_large", "参数请求体超过 65536 字节", request_id
                    ),
                    request_id,
                )
                return
            body = self.rfile.read(length) if length > 0 else b""
            status, payload = app.update_params_payload(
                request_id, body, str(self.headers.get("Content-Type", ""))
            )
            self._send(app, status, payload, request_id)
            return

        self.close_connection = True
        self._send(
            app,
            405,
            app.error_payload("method_not_allowed", "只支持 GET", request_id),
            request_id=request_id,
        )

    do_PUT = do_POST  # noqa: N815
    do_DELETE = do_POST  # noqa: N815
    do_PATCH = do_POST  # noqa: N815

    # ------------------------------------------------------------------ routes
    def _handle_get(self, *, head_only: bool = False) -> None:
        app = self._app()
        request_id = self._request_id()
        parsed = urlsplit(self.path or "/")
        path = parsed.path or "/"
        if len(path) > 1:
            path = path.rstrip("/")
        query = parse_qs(parsed.query)
        include_metrics = self._include_metrics(query)

        if path in _LIVE_PATHS:
            self._send(app, 200, app.live_payload(request_id), request_id, head_only)
            return

        if path in _READY_PATHS:
            ready, payload = app.ready_payload(request_id)
            self._send(app, 200 if ready else 503, payload, request_id, head_only)
            return

        if path == "/":
            self._send(app, 200, app.root_payload(request_id), request_id, head_only)
            return

        protected = (
            path in _DETAIL_PATHS
            or path in _PARAMS_PATHS
            or bool(_DEVICE_PATH.match(path))
        )
        if protected and not app.authorized(self.headers):
            self._send(
                app,
                401,
                app.error_payload("unauthorized", "无效或缺失的 API Key", request_id),
                request_id,
                head_only,
            )
            return

        if path in _PARAMS_PATHS:
            if not app.params_supported():
                self._send(
                    app,
                    404,
                    app.error_payload(
                        "params_not_supported", "该模块不支持参数查询", request_id
                    ),
                    request_id,
                    head_only,
                )
                return
            payload = app.params_payload(request_id)
            if payload is None:
                self._send(
                    app,
                    503,
                    app.error_payload(
                        "not_ready", "参数快照尚未生成", request_id
                    ),
                    request_id,
                    head_only,
                )
                return
            self._send(app, 200, payload, request_id, head_only)
            return

        if path in _DETAIL_PATHS:
            payload = app.detail_payload(request_id, include_metrics=include_metrics)
            if payload is None:
                self._send(
                    app,
                    503,
                    app.error_payload(
                        "not_ready", "健康快照尚未生成", request_id
                    ),
                    request_id,
                    head_only,
                )
                return
            self._send(app, 200, payload, request_id, head_only)
            return

        match = _DEVICE_PATH.match(path)
        if match:
            device_id = match.group(1)
            payload = app.device_payload(
                device_id, request_id, include_metrics=include_metrics
            )
            if payload is None:
                if not app.has_snapshot():
                    self._send(
                        app,
                        503,
                        app.error_payload(
                            "not_ready", "健康快照尚未生成", request_id
                        ),
                        request_id,
                        head_only,
                    )
                    return
                self._send(
                    app,
                    404,
                    app.error_payload(
                        "unknown_device", f"未知设备: {device_id}", request_id
                    ),
                    request_id,
                    head_only,
                )
                return
            self._send(app, 200, payload, request_id, head_only)
            return

        self._send(
            app,
            404,
            app.error_payload("not_found", f"未知路径: {path}", request_id),
            request_id,
            head_only,
        )

    # ---------------------------------------------------------------- response
    def _send(
        self,
        app: HealthHttpServer,
        status: int,
        payload: Mapping[str, Any],
        request_id: str,
        head_only: bool = False,
    ) -> None:
        body = b""
        if status != 204:
            body = json.dumps(
                dict(payload), ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Request-ID", request_id)
        self.send_header("Access-Control-Allow-Origin", app.cors_allow_origin)
        self.send_header("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS")
        self.send_header(
            "Access-Control-Allow-Headers",
            "Content-Type,Authorization,X-API-Key,X-Request-ID",
        )
        if status != 204:
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
        else:
            self.send_header("Content-Length", "0")
        self.end_headers()
        if body and not head_only:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    # ----------------------------------------------------------------- helpers
    def _app(self) -> HealthHttpServer:
        return self.server.health_app  # type: ignore[attr-defined]

    def _request_id(self) -> str:
        value = str(self.headers.get("X-Request-ID", "") or "").strip()
        return value[:128] if value else uuid.uuid4().hex

    @staticmethod
    def _include_metrics(query: Mapping[str, Any]) -> bool:
        raw = (query.get("include_metrics") or ["true"])[0]
        return str(raw).strip().lower() not in {"0", "false", "no", "off"}

    def log_message(self, fmt: str, *args: Any) -> None:
        LOGGER.debug("%s - %s", self.address_string(), fmt % args)
