"""HTTP 路由层（不依赖第三方 Web 框架）。

``HttpApi.dispatch`` 是纯函数式入口：输入 method/path/headers/body，输出
(status, payload, headers)。这样既方便单元测试，也能被任意 HTTP 适配器复用。
"""

from __future__ import annotations

import hmac
import json
import logging
import re
import uuid
from typing import Any, Mapping
from urllib.parse import urlsplit

from . import SERVICE_NAME, __version__
from .errors import (
    ApiError,
    BadRequest,
    MethodNotAllowed,
    NotFound,
    PayloadTooLarge,
    Unauthorized,
    UnsupportedMediaType,
)
from .service import InferenceService

LOGGER = logging.getLogger("center_server.http")

_REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")
_PROTECTED_PREFIXES = ("/v1/",)
_PUBLIC_PATHS = {"/v1/health", "/v1/healthz", "/v1/ready"}


class HttpApi:
    def __init__(self, service: InferenceService, config: Mapping[str, Any]) -> None:
        self.service = service
        self.server_cfg = dict(config.get("server") or {})
        self.api_key = str(self.server_cfg.get("api_key") or "")
        self.max_body_bytes = int(
            self.server_cfg.get("max_body_bytes", 16 * 1024 * 1024)
        )
        self.cors_origin = str(
            self.server_cfg.get("cors_allow_origin") or "*"
        )

    # ------------------------------------------------------------------ route
    def dispatch(
        self,
        method: str,
        path: str,
        headers: Mapping[str, str] | None = None,
        body: bytes = b"",
        *,
        request_id: str | None = None,
    ) -> tuple[int, dict[str, Any], dict[str, str]]:
        method = (method or "GET").upper()
        normalized_headers = {
            str(key).lower(): str(value) for key, value in (headers or {}).items()
        }
        request_id = self._request_id(request_id, normalized_headers)
        response_headers = {
            "X-Request-ID": request_id,
            "X-Content-Type-Options": "nosniff",
            "Cache-Control": "no-store",
            "Access-Control-Allow-Origin": self.cors_origin,
            "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
            "Access-Control-Allow-Headers": (
                "Content-Type,Authorization,X-API-Key,X-Request-ID"
            ),
            "Access-Control-Expose-Headers": "X-Request-ID",
        }

        try:
            route_path = urlsplit(path or "/").path or "/"
            if len(route_path) > 1:
                route_path = route_path.rstrip("/")

            if method == "OPTIONS":
                return 204, {}, self._common_headers(response_headers)

            if self._is_protected(route_path) and not self._authorized(
                normalized_headers
            ):
                raise Unauthorized("无效或缺失的 API Key")

            status, payload = self._route(
                method,
                route_path,
                normalized_headers,
                body,
                request_id,
            )
        except ApiError as exc:
            status = exc.status_code
            payload = {
                "ok": False,
                "error": exc.to_dict(),
                "request_id": request_id,
            }
            if status >= 500:
                LOGGER.error("%s %s -> %s %s", method, path, status, exc.message)
            else:
                LOGGER.info("%s %s -> %s %s", method, path, status, exc.message)
        except Exception as exc:  # noqa: BLE001 - 不让未捕获异常打断连接
            LOGGER.exception("unhandled error on %s %s", method, path)
            status = 500
            payload = {
                "ok": False,
                "error": {
                    "code": "internal_error",
                    "message": f"服务内部错误: {type(exc).__name__}",
                },
                "request_id": request_id,
            }

        if payload is not None and "request_id" not in payload:
            payload["request_id"] = request_id
        return status, payload, self._common_headers(response_headers)

    def _route(
        self,
        method: str,
        path: str,
        headers: Mapping[str, str],
        body: bytes,
        request_id: str,
    ) -> tuple[int, dict[str, Any]]:
        if path in {"", "/"}:
            self._require_method(method, {"GET"})
            status = self.service.status()
            return 200, {
                "ok": True,
                "service": SERVICE_NAME,
                "version": __version__,
                "status": status["status"],
                "endpoints": {
                    "health": "GET /v1/health",
                    "ready": "GET /v1/ready",
                    "model": "GET /v1/model",
                    "infer": "POST /v1/infer",
                    "reset": "POST /v1/reset",
                    "warmup": "POST /v1/warmup",
                },
            }

        if path in {"/healthz", "/livez"}:
            self._require_method(method, {"GET"})
            return 200, {"ok": True, "status": "ok"}

        if path in {"/readyz", "/v1/ready"}:
            self._require_method(method, {"GET"})
            ready = self.service.ready()
            return (
                200 if ready else 503,
                {
                    "ok": ready,
                    "status": "ready" if ready else "not_ready",
                    "model": self.service.engine.status(),
                },
            )

        if path in {"/health", "/v1/health", "/v1/healthz"}:
            self._require_method(method, {"GET"})
            status = self.service.status()
            return 200, status

        if path == "/v1/model":
            self._require_method(method, {"GET"})
            return 200, {"ok": True, "model": self.service.model_info()}

        if path == "/v1/infer":
            self._require_method(method, {"POST"})
            payload = self._json_body(headers, body)
            return 200, self.service.infer(payload, request_id=request_id)

        if path == "/v1/reset":
            self._require_method(method, {"POST"})
            payload = self._json_body(headers, body, allow_empty=True)
            return 200, self.service.reset(payload)

        if path == "/v1/warmup":
            self._require_method(method, {"POST"})
            self._json_body(headers, body, allow_empty=True)
            return 200, self.service.warmup()

        raise NotFound(f"未知路径: {path}")

    # ----------------------------------------------------------------- helpers
    def _is_protected(self, path: str) -> bool:
        if not self.api_key:
            return False
        if path in _PUBLIC_PATHS:
            return False
        return any(path.startswith(prefix) for prefix in _PROTECTED_PREFIXES)

    def _authorized(self, headers: Mapping[str, str]) -> bool:
        authorization = headers.get("authorization", "")
        token = ""
        if authorization.lower().startswith("bearer "):
            token = authorization[7:].strip()
        if not token:
            token = headers.get("x-api-key", "").strip()
        if not token:
            return False
        return hmac.compare_digest(
            token.encode("utf-8"), self.api_key.encode("utf-8")
        )

    def _json_body(
        self,
        headers: Mapping[str, str],
        body: bytes,
        *,
        allow_empty: bool = False,
    ) -> Any:
        length_header = headers.get("content-length")
        if length_header:
            try:
                declared = int(length_header)
            except ValueError as exc:
                raise BadRequest("Content-Length 非法", code="invalid_length") from exc
            if declared > self.max_body_bytes:
                raise PayloadTooLarge(
                    f"请求体超过 {self.max_body_bytes} 字节",
                    details={"declared": declared, "max": self.max_body_bytes},
                )
        if len(body) > self.max_body_bytes:
            raise PayloadTooLarge(
                f"请求体超过 {self.max_body_bytes} 字节",
                details={"received": len(body), "max": self.max_body_bytes},
            )

        content_type = headers.get("content-type", "").split(";", 1)[0].strip()
        if content_type and content_type.lower() != "application/json":
            raise UnsupportedMediaType(
                "Content-Type 必须是 application/json",
                details={"received": content_type},
            )

        if not body:
            if allow_empty:
                return {}
            raise BadRequest("请求体为空", code="empty_body")
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise BadRequest(
                f"JSON 解析失败: {exc}", code="invalid_json"
            ) from exc

    @staticmethod
    def _require_method(method: str, allowed: set[str]) -> None:
        if method not in allowed:
            raise MethodNotAllowed(
                f"该路径只支持 {sorted(allowed)}", details={"method": method}
            )

    @staticmethod
    def _request_id(
        provided: str | None, headers: Mapping[str, str]
    ) -> str:
        candidate = provided or headers.get("x-request-id") or ""
        if candidate and _REQUEST_ID_RE.match(candidate):
            return candidate
        return uuid.uuid4().hex

    @staticmethod
    def _common_headers(headers: dict[str, str]) -> dict[str, str]:
        # 对 204 也保留 CORS/request-id，方便浏览器预检
        result = dict(headers)
        result["Content-Type"] = "application/json; charset=utf-8"
        return result
