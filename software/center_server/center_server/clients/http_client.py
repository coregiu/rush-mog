"""轻量 HTTP 客户端（标准库 urllib，不依赖 Web 框架）。

ROS 节点可以直接 ``from center_server.clients import CenterServerClient``，
把相机帧/里程计/关节状态 POST 给中心推理机。客户端只依赖标准库 + numpy/Pillow
（仅当传 numpy 图像时需要）。
"""

from __future__ import annotations

import json
import socket
import urllib.error
import urllib.request
from typing import Any, Mapping, Sequence

from ..images import encode_image_b64


class CenterServerError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        status: int | None = None,
        code: str | None = None,
        payload: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.code = code
        self.payload = dict(payload or {})


class CenterServerClient:
    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8000",
        *,
        api_key: str = "",
        timeout_sec: float = 10.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.timeout_sec = float(timeout_sec)

    # ------------------------------------------------------------------ basic
    def health(self) -> dict[str, Any]:
        return self._request("GET", "/v1/health")

    def model(self) -> dict[str, Any]:
        return self._request("GET", "/v1/model")

    def warmup(self) -> dict[str, Any]:
        return self._request("POST", "/v1/warmup", {})

    def reset(self, session_id: str = "") -> dict[str, Any]:
        return self._request("POST", "/v1/reset", {"session_id": session_id})

    def infer(
        self,
        instruction: str,
        images: Mapping[str, Any],
        state: Sequence[float] | None = None,
        *,
        task_id: str = "",
        session_id: str = "",
        chunk_size: int | None = None,
        reset: bool = False,
        return_mapped: bool = False,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "instruction": instruction,
            "images": {
                name: (
                    value
                    if isinstance(value, str)
                    else encode_image_b64(value, image_format="JPEG")
                )
                for name, value in images.items()
            },
            "state": list(state or []),
            "task_id": task_id,
            "session_id": session_id,
            "reset": bool(reset),
            "options": {"return_mapped": bool(return_mapped)},
        }
        if chunk_size is not None:
            payload["options"]["chunk_size"] = int(chunk_size)
        return self._request("POST", "/v1/infer", payload)

    # ------------------------------------------------------------------ http
    def _request(
        self, method: str, path: str, payload: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                raw = response.read()
                body = json.loads(raw.decode("utf-8")) if raw else {}
                if not isinstance(body, dict):
                    raise CenterServerError(
                        "Center Server 返回了非 JSON 对象", payload={"body": body}
                    )
                return body
        except urllib.error.HTTPError as exc:
            body: dict[str, Any] = {}
            try:
                parsed = json.loads(exc.read().decode("utf-8"))
                if isinstance(parsed, dict):
                    body = parsed
            except Exception:  # noqa: BLE001
                pass
            error = body.get("error") or {}
            raise CenterServerError(
                str(error.get("message") or f"HTTP {exc.code}"),
                status=exc.code,
                code=str(error.get("code") or "http_error"),
                payload=body,
            ) from exc
        except (urllib.error.URLError, socket.timeout) as exc:
            raise CenterServerError(
                f"无法连接 Center Server {url}: {exc}", code="connection_error"
            ) from exc
        except json.JSONDecodeError as exc:
            raise CenterServerError(
                "Center Server 返回了非法 JSON", code="invalid_json"
            ) from exc

    def __enter__(self) -> "CenterServerClient":
        return self

    def __exit__(self, *_exc: Any) -> None:
        return None
