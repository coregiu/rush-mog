"""标准库 HTTP 服务器与命令行入口。

选择 ``http.server.ThreadingHTTPServer`` 而不是 FastAPI/uvicorn，是为了让
中心推理机只依赖 Python 标准库 + numpy/Pillow/PyYAML 就能启动；真正的
GPU 依赖（torch/lerobot）只在加载 SmolVLA 时引入。
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Any

from . import SERVICE_NAME, __version__
from .config import ConfigError, default_config_path, load_config, validate_config
from .errors import ApiError
from .http_api import HttpApi
from .service import InferenceService

LOGGER = logging.getLogger("center_server.server")


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _AbortRequest(Exception):
    """请求已在 handler 内部回复，调用方直接返回。"""


class _RequestHandler(BaseHTTPRequestHandler):
    server_version = f"RushMogCenterServer/{__version__}"
    protocol_version = "HTTP/1.1"

    api: HttpApi
    max_body_bytes: int
    request_timeout_sec: float = 30.0

    def do_GET(self) -> None:  # noqa: N802
        self._handle("GET")

    def do_POST(self) -> None:  # noqa: N802
        self._handle("POST")

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._handle("OPTIONS")

    def do_PUT(self) -> None:  # noqa: N802
        self._handle("PUT")

    def do_PATCH(self) -> None:  # noqa: N802
        self._handle("PATCH")

    def do_DELETE(self) -> None:  # noqa: N802
        self._handle("DELETE")

    def _handle(self, method: str) -> None:
        started = time.perf_counter()
        status = 500
        try:
            try:
                self.connection.settimeout(float(self.request_timeout_sec))
            except OSError:
                pass
            try:
                length = self._content_length()
            except _AbortRequest:
                status = 400
                return
            if length is not None and length > self.max_body_bytes:
                self.close_connection = True
                status = 413
                self._send_json(
                    status,
                    {
                        "ok": False,
                        "error": {
                            "code": "payload_too_large",
                            "message": f"请求体超过 {self.max_body_bytes} 字节",
                        },
                    },
                    {"X-Request-ID": self.headers.get("X-Request-ID", "")},
                )
                return

            body = self.rfile.read(length) if length else b""
            status, payload, headers = self.api.dispatch(
                method,
                self.path,
                dict(self.headers.items()),
                body,
            )
            self._send_json(status, payload, headers)
        except (BrokenPipeError, ConnectionResetError):
            LOGGER.debug("client disconnected during %s %s", method, self.path)
        except (TimeoutError, socket.timeout):
            self.close_connection = True
            status = 408
            LOGGER.warning("request timeout on %s %s", method, self.path)
            try:
                self._send_json(
                    status,
                    {
                        "ok": False,
                        "error": {
                            "code": "request_timeout",
                            "message": "请求超时",
                        },
                    },
                    {},
                )
            except Exception:  # noqa: BLE001
                pass
        except Exception:  # noqa: BLE001
            LOGGER.exception("handler error on %s %s", method, self.path)
            try:
                self._send_json(
                    500,
                    {
                        "ok": False,
                        "error": {
                            "code": "internal_error",
                            "message": "HTTP handler error",
                        },
                    },
                    {},
                )
            except Exception:  # noqa: BLE001
                pass
        finally:
            LOGGER.info(
                "%s %s -> %s (%.1f ms)",
                method,
                self.path,
                status,
                (time.perf_counter() - started) * 1000.0,
            )

    def _content_length(self) -> int | None:
        raw = self.headers.get("Content-Length")
        if raw is None:
            return 0
        try:
            length = int(raw)
        except ValueError:
            self.close_connection = True
            self._send_json(
                400,
                {
                    "ok": False,
                    "error": {
                        "code": "invalid_length",
                        "message": "Content-Length 非法",
                    },
                },
                {},
            )
            raise _AbortRequest
        if length < 0:
            self.close_connection = True
            self._send_json(
                400,
                {
                    "ok": False,
                    "error": {
                        "code": "invalid_length",
                        "message": "Content-Length 不能为负数",
                    },
                },
                {},
            )
            raise _AbortRequest
        return length

    def _send_json(
        self, status: int, payload: dict[str, Any], headers: dict[str, str]
    ) -> None:
        data = (
            b""
            if status == 204
            else json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            )
        )
        self.send_response(status)
        for key, value in (headers or {}).items():
            if key.lower() in {"content-length", "connection"}:
                continue
            self.send_header(key, str(value))
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if data:
            self.wfile.write(data)
            self.wfile.flush()

    def log_message(self, fmt: str, *args: Any) -> None:
        # 默认实现直接写 stderr；统一交给 logging，避免重复时间戳。
        LOGGER.debug("%s - %s", self.address_string(), fmt % args)


def create_server(api: HttpApi, host: str, port: int) -> ThreadingHTTPServer:
    handler = type(
        "BoundCenterServerHandler",
        (_RequestHandler,),
        {
            "api": api,
            "max_body_bytes": api.max_body_bytes,
            "request_timeout_sec": float(
                api.server_cfg.get("request_timeout_sec", 30.0)
            ),
        },
    )
    return ThreadingHTTPServer((host, port), handler)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rush-mog-center-server",
        description="Rush-MOG Center Server：SmolVLA HTTP 推理服务",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="YAML 配置文件；默认读取 CENTER_SERVER_CONFIG 或仓库 config.yaml",
    )
    parser.add_argument("--host", default=None, help="监听地址（覆盖配置）")
    parser.add_argument("--port", type=int, default=None, help="监听端口（覆盖配置）")
    parser.add_argument(
        "--backend",
        choices=["smolvla", "stub"],
        default=None,
        help="推理后端（覆盖配置；stub 仅用于联调）",
    )
    parser.add_argument("--device", default=None, help="cpu/cuda/cuda:0/mps/auto")
    parser.add_argument(
        "--check",
        action="store_true",
        help="只加载配置并打印状态，不启动 HTTP 服务",
    )
    parser.add_argument(
        "--warmup",
        action="store_true",
        help="启动前执行一次空推理（会加载模型）",
    )
    parser.add_argument("--log-level", default=None, help="DEBUG/INFO/WARNING/ERROR")
    parser.add_argument("--version", action="version", version=__version__)
    return parser


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def _resolve_config_path(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    env_path = os.environ.get("CENTER_SERVER_CONFIG")
    if env_path:
        return env_path
    candidate = default_config_path()
    if candidate.exists():
        return str(candidate)
    return None


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    try:
        cfg = load_config(_resolve_config_path(args.config))
    except ConfigError as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    if args.host is not None:
        cfg["server"]["host"] = args.host
    if args.port is not None:
        cfg["server"]["port"] = int(args.port)
    if args.backend:
        cfg["model"]["backend"] = args.backend
    if args.device:
        cfg["model"]["device"] = args.device
    if args.log_level:
        cfg["logging"]["level"] = args.log_level.upper()

    try:
        validate_config(cfg)
    except ConfigError as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    _configure_logging(cfg["logging"]["level"])
    LOGGER.info(
        "%s %s starting (backend=%s, config=%s)",
        SERVICE_NAME,
        __version__,
        cfg["model"]["backend"],
        cfg.get("_config_path", "<defaults>"),
    )

    try:
        service = InferenceService(cfg)
    except (ApiError, ValueError, RuntimeError) as exc:
        LOGGER.error("初始化推理服务失败: %s", exc)
        return 1

    if args.check:
        print(json.dumps(service.status(), ensure_ascii=False, indent=2))
        service.close()
        return 0

    if args.warmup:
        try:
            result = service.warmup()
            LOGGER.info("warmup ok: %.1f ms", result["model_latency_ms"])
        except ApiError as exc:
            LOGGER.error("warmup 失败: %s", exc.message)
            service.close()
            return 1

    api = HttpApi(service, cfg)
    host = str(cfg["server"]["host"])
    port = int(cfg["server"]["port"])
    try:
        httpd = create_server(api, host, port)
    except OSError as exc:
        LOGGER.error("无法监听 %s:%s: %s", host, port, exc)
        service.close()
        return 1

    def _stop(_signum: int, _frame: Any) -> None:
        LOGGER.info("收到退出信号，正在停止服务")
        # shutdown() 不能在 serve_forever 所在线程调用
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    LOGGER.info("listening on http://%s:%s", host, port)
    LOGGER.info("health: http://%s:%s/v1/health", host, port)
    try:
        httpd.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        service.close()
        LOGGER.info("server stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
