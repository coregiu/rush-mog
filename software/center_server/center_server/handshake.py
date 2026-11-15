"""启动握手：向 Edge Server 上报 Center Server 的地址与模型能力。

Edge Server（树莓派）是所有机载交互的入口：dashboard 通过它代理中心参数，
`vla_inference_node` 通过它发现中心推理地址。本模块在 Center Server 启动后
主动向 Edge Server 发送**握手消息**（HTTP POST），上报：

- Center Server 的 IP / 端口 / 各接口路径（health、params、infer）；
- 服务版本、就绪状态、模型 backend/device/dtype/维度、动作映射参数；
- 握手类型（``startup`` / ``heartbeat`` / ``manual``）。

Edge Server 以 TCP 对端地址为准记录 IP（不信任 payload 里的 ``ip``），再按
上报的端口拼出 ``infer_url`` 供后续 VLA 推理调用。

握手是**旁路能力**：Edge 未启动、Wi-Fi 未连接都不应影响推理服务本身。
因此本模块在后台线程运行，失败按指数退避重试，并在 Edge 重启后自动重连
（周期性 heartbeat）。
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from typing import Any, Callable, Mapping

LOGGER = logging.getLogger("center_server.handshake")

DEFAULT_PATH = "/api/center/handshake"
MAX_SUMMARY = 240
MAX_PARAMS_JSON = 2048
MAX_META_JSON = 2048
MAX_ERROR = 240

# 上报给 dashboard 的运行时参数（完整参数通过 Edge 代理按需读取）
_PARAM_KEYS = (
    "action.mode",
    "action.normalized",
    "action.max_linear_x",
    "action.max_linear_y",
    "action.max_angular_z",
    "action.deadband",
    "action.arm_start",
    "action.round_digits",
)

_HEADERS = {
    "Accept": "application/json",
    "Content-Type": "application/json; charset=utf-8",
    "User-Agent": "rush-mog-center-handshake/1.0",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _short(value: Any, limit: int = MAX_SUMMARY) -> str:
    text = str(value or "").strip().replace("\n", " ")
    return text[:limit]


def _compact_json(payload: Any, limit: int) -> str:
    """序列化并限制长度；超限时返回 ``{}``，避免把 Edge 表单撑爆。"""
    try:
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    except (TypeError, ValueError):
        return "{}"
    if len(raw) > limit:
        LOGGER.debug("handshake json 超过 %d 字节，已省略", limit)
        return "{}"
    return raw


def detect_public_host(edge_url: str, fallback: str = "") -> str:
    """选择到 Edge Server 的本地出口 IP。

    用 UDP ``connect`` 让内核选路（不会真的发包）；失败时退回主机名解析。
    该地址只用于日志/展示，Edge 端以 TCP 对端地址为准。
    """

    if fallback:
        return str(fallback).strip()
    parsed = urllib.parse.urlsplit(edge_url or "")
    host = parsed.hostname or "192.168.3.26"
    port = parsed.port or 80
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((host, port))
            local = sock.getsockname()[0]
            if local and not local.startswith("127."):
                return str(local)
    except OSError:
        pass
    try:
        resolved = socket.gethostbyname(socket.gethostname())
        if resolved:
            return str(resolved)
    except OSError:
        pass
    return "127.0.0.1"


def _default_sender(
    url: str,
    payload: Mapping[str, Any],
    timeout_sec: float,
    api_key: str,
) -> tuple[int, dict[str, Any]]:
    """POST JSON 到 Edge Server；非 2xx 抛 ``RuntimeError``。"""

    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    headers = dict(_HEADERS)
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"
    request = urllib.request.Request(url, data=data, headers=headers, method="POST")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=timeout_sec) as response:
            body = response.read(64 * 1024)
            status = int(getattr(response, "status", 200) or 200)
    except urllib.error.HTTPError as exc:
        raw = b""
        try:
            raw = exc.read(64 * 1024)
        except Exception:  # noqa: BLE001 - 读错误体失败不应掩盖状态码
            pass
        detail = ""
        try:
            parsed = json.loads(raw.decode("utf-8")) if raw else {}
            error = parsed.get("error") if isinstance(parsed, dict) else None
            if isinstance(error, dict):
                detail = str(error.get("message") or "")
        except (UnicodeDecodeError, ValueError):
            detail = ""
        message = f"HTTP {exc.code}" + (f": {detail}" if detail else "")
        raise RuntimeError(message) from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(f"无法连接 Edge Server: {exc}") from exc

    parsed: dict[str, Any] = {}
    if body:
        try:
            decoded = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            raise RuntimeError("Edge Server 返回了非法 JSON") from exc
        if isinstance(decoded, dict):
            parsed = decoded
    return status, parsed


class HandshakeAnnouncer:
    """后台握手线程 + 手动触发；线程安全的状态快照。"""

    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        status_provider: Callable[[], Mapping[str, Any]] | None = None,
        params_provider: Callable[[], Mapping[str, Any]] | None = None,
        model_provider: Callable[[], Mapping[str, Any]] | None = None,
        port: int | None = None,
        sender: Callable[
            [str, Mapping[str, Any], float, str], tuple[int, dict[str, Any]]
        ]
        | None = None,
    ) -> None:
        self.cfg = dict(config or {})
        hs = dict(self.cfg.get("handshake") or {})
        server_cfg = dict(self.cfg.get("server") or {})
        self.path = str(hs.get("path") or DEFAULT_PATH)
        self.edge_url = str(hs.get("edge_url") or "").strip().rstrip("/")
        self.url = f"{self.edge_url}{self.path}" if self.edge_url else ""
        self._enabled = bool(hs.get("enabled", True)) and bool(self.edge_url)
        self.interval_sec = max(0.5, float(hs.get("interval_sec", 10.0)))
        self.retry_sec = max(0.2, float(hs.get("retry_sec", 2.0)))
        self.max_retry_sec = max(self.retry_sec, float(hs.get("max_retry_sec", 30.0)))
        self.timeout_sec = max(0.2, float(hs.get("timeout_sec", 3.0)))
        self.api_key = str(hs.get("api_key") or "")
        self.service_name = str(
            hs.get("service_name") or "SmolVLA Center Server"
        )
        self.port = int(port if port is not None else server_cfg.get("port", 8000))
        self.public_host = detect_public_host(
            self.edge_url, str(hs.get("public_host") or "")
        )
        self.host = str(server_cfg.get("host") or "0.0.0.0")

        self._status_provider = status_provider
        self._params_provider = params_provider
        self._model_provider = model_provider
        self._sender = sender or _default_sender

        self._stop = threading.Event()
        self._wake = threading.Event()
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._first_success = False
        self._state: dict[str, Any] = {
            "attempts": 0,
            "successes": 0,
            "failures": 0,
            "connected": False,
            "last_error": "",
            "last_attempt_at": None,
            "last_ok_at": None,
            "last_reason": "",
            "last_response": {},
        }

    # ------------------------------------------------------------------ life
    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> bool:
        if not self._enabled:
            LOGGER.info("Edge 握手未启用（handshake.enabled/edge_url 为空）")
            return False
        if self._thread is not None:
            return True
        self._thread = threading.Thread(
            target=self._loop, name="center-handshake", daemon=True
        )
        self._thread.start()
        LOGGER.info(
            "Edge 握手已启动: target=%s, 本机=%s:%s, 间隔=%.1fs",
            self.url,
            self.public_host,
            self.port,
            self.interval_sec,
        )
        return True

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    # ---------------------------------------------------------------- announce
    def announce_now(self, reason: str = "manual") -> dict[str, Any]:
        """立即发送一次握手（供 ``POST /v1/handshake`` 使用）。"""

        if not self._enabled:
            return {
                "ok": False,
                "enabled": False,
                "error": "Edge 握手未启用（handshake.edge_url 为空）",
            }
        ok = self._send(reason)
        status = self.status()
        result: dict[str, Any] = {"ok": ok, "enabled": True, "reason": reason}
        if not ok:
            result["error"] = status.get("last_error", "")
        return result

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            state = dict(self._state)
        return {
            "enabled": self._enabled,
            "target": self.url,
            "path": self.path,
            "host": self.host,
            "port": self.port,
            "public_host": self.public_host,
            "interval_sec": self.interval_sec,
            "retry_sec": self.retry_sec,
            "max_retry_sec": self.max_retry_sec,
            "timeout_sec": self.timeout_sec,
            "auth": bool(self.api_key),
            **state,
        }

    # ------------------------------------------------------------------- loop
    def _loop(self) -> None:
        backoff = self.retry_sec
        while not self._stop.is_set():
            reason = "heartbeat" if self._first_success else "startup"
            ok = self._send(reason)
            if ok:
                self._first_success = True
                backoff = self.retry_sec
                wait = self.interval_sec
            else:
                wait = backoff
                backoff = min(self.max_retry_sec, backoff * 1.7)
            self._wake.wait(wait)
            self._wake.clear()
            if self._stop.is_set():
                break

    # ------------------------------------------------------------------- send
    def _send(self, reason: str) -> bool:
        with self._send_lock:
            payload = self._payload(reason)
            attempted_at = _utc_now()
            with self._state_lock:
                self._state["attempts"] += 1
                self._state["last_attempt_at"] = attempted_at
                self._state["last_reason"] = reason
            try:
                status, body = self._sender(
                    self.url, payload, self.timeout_sec, self.api_key
                )
                if not 200 <= int(status) < 300:
                    raise RuntimeError(f"HTTP {status}")
                ok = bool(body.get("ok", True))
                if not ok:
                    error = body.get("error") or {}
                    message = (
                        error.get("message")
                        if isinstance(error, Mapping)
                        else ""
                    )
                    raise RuntimeError(str(message or "Edge Server 拒绝握手"))
            except Exception as exc:  # noqa: BLE001 - 握手失败不影响推理
                message = _short(f"{type(exc).__name__}: {exc}", MAX_ERROR)
                with self._state_lock:
                    self._state["failures"] += 1
                    self._state["connected"] = False
                    self._state["last_error"] = message
                LOGGER.warning("Edge 握手失败（%s）: %s", reason, message)
                return False

            with self._state_lock:
                self._state["successes"] += 1
                successes = int(self._state["successes"])
                self._state["connected"] = True
                self._state["last_error"] = ""
                self._state["last_ok_at"] = _utc_now()
                self._state["last_response"] = _compact_response(body)
            if reason != "heartbeat" or successes == 1:
                LOGGER.info(
                    "Edge 握手成功: %s -> %s:%s (%s)",
                    self.url,
                    self.public_host,
                    self.port,
                    reason,
                )
            else:
                LOGGER.debug("Edge 心跳成功: %s", self.url)
            return True

    # ---------------------------------------------------------------- payload
    def _payload(self, reason: str) -> dict[str, Any]:
        status = self._service_status()
        model = self._model_info(status)
        ready = bool(status.get("ready", False))
        params = self._params_payload()
        action_mode = str(
            params.get("info", {}).get("action_mode")
            or (self.cfg.get("action") or {}).get("mode")
            or ""
        )
        summary = (
            f"ready={ready} backend={model.get('backend', '')} "
            f"device={model.get('device', '')} "
            f"model={_short(model.get('path', ''), 96)}"
        )
        meta = {
            "protocol": "http-json-v1",
            "capabilities": ["infer", "reset", "warmup", "params"],
            "model": {
                "backend": model.get("backend"),
                "path": model.get("path"),
                "device": model.get("device"),
                "dtype": model.get("dtype"),
                "loaded": bool(model.get("loaded")),
            },
            "action_mode": action_mode,
            "state_dim": model.get("state_dim"),
            "action_dim": model.get("action_dim"),
            "chunk_size": model.get("chunk_size"),
            "image_keys": list(model.get("configured_image_keys") or []),
            "image_aliases": dict(model.get("image_aliases") or {}),
        }
        port = int(self.port)
        return {
            "module": "center",
            "name": self.service_name,
            "version": _short(status.get("version", ""), 24),
            "status": "ok" if ready else "degraded",
            "summary": _short(summary, MAX_SUMMARY),
            "ip": self.public_host,
            "port": port,
            "scheme": "http",
            "health_port": port,
            "health_path": "/v1/health",
            "params_port": port,
            "params_path": "/v1/params",
            "infer_port": port,
            "infer_path": "/v1/infer",
            "params_json": _compact_json(_compact_params(params), MAX_PARAMS_JSON),
            "meta_json": _compact_json(meta, MAX_META_JSON),
            "handshake": reason,
            "timestamp": _utc_now(),
        }

    def _service_status(self) -> Mapping[str, Any]:
        provider = self._status_provider
        if provider is None:
            return {}
        try:
            payload = provider()
        except Exception as exc:  # noqa: BLE001 - 状态读取失败按未就绪处理
            LOGGER.debug("握手读取服务状态失败: %s", exc)
            return {}
        return payload if isinstance(payload, Mapping) else {}

    def _model_info(self, status: Mapping[str, Any]) -> dict[str, Any]:
        """优先用 model_info()：含 checkpoint 相机键/别名/动作维度。"""

        provider = self._model_provider
        if provider is not None:
            try:
                payload = provider()
            except Exception as exc:  # noqa: BLE001
                LOGGER.debug("握手读取模型信息失败: %s", exc)
            else:
                if isinstance(payload, Mapping):
                    return dict(payload)
        return dict(status.get("model") or {})

    def _params_payload(self) -> Mapping[str, Any]:
        provider = self._params_provider
        if provider is None:
            return {}
        try:
            payload = provider()
        except Exception as exc:  # noqa: BLE001
            LOGGER.debug("握手读取参数失败: %s", exc)
            return {}
        return payload if isinstance(payload, Mapping) else {}


def _compact_params(payload: Mapping[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for item in payload.get("params") or []:
        if not isinstance(item, Mapping):
            continue
        key = str(item.get("key") or "")
        if key in _PARAM_KEYS:
            values[key] = item.get("value")
    return values


def _compact_response(body: Mapping[str, Any]) -> dict[str, Any]:
    """只保留 Edge 返回的关键地址，避免状态快照无限增长。"""

    keys = (
        "ok",
        "module",
        "ip",
        "port",
        "health_url",
        "params_url",
        "infer_url",
        "message",
    )
    result: dict[str, Any] = {}
    for key in keys:
        if key in body:
            result[key] = body[key]
    return result


__all__ = [
    "DEFAULT_PATH",
    "HandshakeAnnouncer",
    "detect_public_host",
]
