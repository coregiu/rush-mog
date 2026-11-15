"""Edge -> Center SmolVLA HTTP 客户端（标准库 + numpy，ROS 可直连）。

Center Server 启动后会向 Edge Server 握手，Edge 记录 ``base_url`` /
``infer_url``。本模块提供两件事：

1. ``discover_center``：从 Edge Server 注册表 ``GET /api/center`` 取回地址；
2. ``CenterClient``：把相机帧 + 本体状态 POST 到 ``/v1/infer``，返回动作块。

图像默认按 OpenCV 约定视为 **BGR**（``cv_bridge`` 的 ``bgr8`` 帧、``cv2.imread``
输出），编码成 JPEG 后由 Center 解码为 RGB；若上游是 RGB，请显式传
``color_order="rgb"``。

推理失败一律抛 ``CenterClientError``，由调用方决定降级（VLAEngine 会回退到
规则桩），不要在 ROS 定时器里重试。
"""

from __future__ import annotations

import base64
import io
import json
import socket
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Mapping, Optional, Sequence

import numpy as np

try:  # ROS Humble 环境默认有 python3-opencv；缺失时退回 Pillow
    import cv2
except ImportError:  # pragma: no cover - 兜底分支
    cv2 = None

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None

_MAX_IMAGE_BYTES = 6 * 1024 * 1024
_JPEG_QUALITY = 85


class CenterClientError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        status: Optional[int] = None,
        code: Optional[str] = None,
        payload: Optional[Mapping[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.code = code
        self.payload = dict(payload or {})


def _to_bgr8(image: Any, color_order: str = "bgr") -> np.ndarray:
    """HWC/CHW、uint8/float、RGB/BGR -> 连续 uint8 BGR 图像。"""

    array = np.asarray(image)
    if array.size == 0:
        raise CenterClientError("图像为空")
    if array.ndim == 2:
        array = np.repeat(array[:, :, None], 3, axis=2)
    elif array.ndim == 3:
        if (
            array.shape[0] in (1, 3, 4)
            and array.shape[0] < array.shape[1]
            and array.shape[0] < array.shape[2]
        ):
            array = np.transpose(array, (1, 2, 0))  # CHW -> HWC
        if array.shape[2] == 1:
            array = np.repeat(array, 3, axis=2)
        if array.shape[2] > 3:
            array = array[:, :, :3]
    if array.ndim != 3 or array.shape[2] != 3:
        raise CenterClientError(f"不支持的图像形状: {array.shape}")

    if np.issubdtype(array.dtype, np.floating):
        # 约定 float 图像范围 [0, 1]
        array = np.clip(array * 255.0, 0.0, 255.0)
    array = np.ascontiguousarray(array, dtype=np.uint8)
    if str(color_order).lower() == "rgb":
        array = np.ascontiguousarray(array[:, :, ::-1])
    return array


def encode_image_b64(
    image: Any,
    *,
    color_order: str = "bgr",
    quality: int = _JPEG_QUALITY,
) -> str:
    """numpy 图像 -> base64 JPEG；失败抛 ``CenterClientError``。"""

    bgr = _to_bgr8(image, color_order)
    data = b""
    if cv2 is not None:
        ok, encoded = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), max(10, min(95, int(quality)))],
        )
        if not ok:
            raise CenterClientError("JPEG 编码失败")
        data = encoded.tobytes()
    elif Image is not None:
        buffer = io.BytesIO()
        try:
            Image.fromarray(bgr[:, :, ::-1]).save(
                buffer, format="JPEG", quality=int(quality)
            )
        except Exception as exc:  # noqa: BLE001
            raise CenterClientError(f"JPEG 编码失败: {exc}") from exc
        data = buffer.getvalue()
    else:
        raise CenterClientError(
            "缺少 OpenCV/Pillow，无法编码图像；请安装 python3-opencv 或 Pillow"
        )
    if len(data) > _MAX_IMAGE_BYTES:
        raise CenterClientError(
            f"图像编码后 {len(data)} 字节，超过 {_MAX_IMAGE_BYTES} 上限"
        )
    return base64.b64encode(data).decode("ascii")


def discover_center(
    discovery_url: str,
    *,
    timeout_sec: float = 1.0,
) -> Optional[dict[str, Any]]:
    """从 Edge Server 读取最近一次 CENTER 握手记录。

    未握手/Edge 未启动时返回 ``None``（不是异常）——VLA 节点继续用规则桩。
    """

    url = str(discovery_url or "").strip()
    if not url:
        return None
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json", "User-Agent": "rush-mog-vla/1.0"},
        method="GET",
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=max(0.1, float(timeout_sec))) as response:
            raw = response.read(128 * 1024)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError):
        return None
    try:
        payload = json.loads(raw.decode("utf-8")) if raw else {}
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(payload, dict) or not payload.get("ok", False):
        return None
    return payload


def base_url_from_registry(payload: Mapping[str, Any]) -> str:
    """兼容 ``base_url``、``scheme/ip/port`` 和只有 ``infer_url`` 的注册表。"""

    base = str(payload.get("base_url") or "").strip().rstrip("/")
    if base:
        return base
    host = str(payload.get("ip") or "").strip()
    port = payload.get("port")
    if host and port:
        scheme = str(payload.get("scheme") or "http").strip() or "http"
        url_host = f"[{host}]" if ":" in host and not host.startswith("[") else host
        return f"{scheme}://{url_host}:{int(port)}"
    infer = str(payload.get("infer_url") or "").strip()
    if infer:
        parsed = urllib.parse.urlsplit(infer)
        if parsed.scheme and parsed.netloc:
            return f"{parsed.scheme}://{parsed.netloc}"
    return ""


class CenterClient:
    """``/v1/infer`` 轻量客户端；所有方法同步，超时由调用方配置。"""

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8000",
        *,
        api_key: str = "",
        timeout_sec: float = 1.5,
        color_order: str = "bgr",
    ) -> None:
        self.base_url = str(base_url or "").strip().rstrip("/")
        if not self.base_url:
            raise CenterClientError("Center Server 地址为空")
        self.api_key = str(api_key or "")
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.color_order = str(color_order or "bgr").lower()

    @classmethod
    def from_registry(
        cls,
        payload: Mapping[str, Any],
        *,
        api_key: str = "",
        timeout_sec: float = 1.5,
        color_order: str = "bgr",
    ) -> "CenterClient":
        base_url = base_url_from_registry(payload)
        if not base_url:
            raise CenterClientError("CENTER 握手记录里没有可用地址")
        return cls(
            base_url,
            api_key=api_key,
            timeout_sec=timeout_sec,
            color_order=color_order,
        )

    # ------------------------------------------------------------------ basic
    def health(self) -> dict[str, Any]:
        return self._request("GET", "/v1/health")

    def model(self) -> dict[str, Any]:
        return self._request("GET", "/v1/model")

    def reset(self, session_id: str = "") -> dict[str, Any]:
        return self._request(
            "POST", "/v1/reset", {"session_id": str(session_id or "")}
        )

    # ------------------------------------------------------------------ infer
    def infer(
        self,
        instruction: str,
        images: Mapping[str, Any],
        state: Optional[Sequence[float]] = None,
        *,
        task_id: str = "",
        session_id: str = "",
        chunk_size: Optional[int] = None,
        reset: bool = False,
        return_mapped: bool = False,
    ) -> dict[str, Any]:
        text = str(instruction or "").strip()
        if not text:
            raise CenterClientError("instruction 为空，SmolVLA 需要语言指令")
        if not images:
            raise CenterClientError("缺少图像输入")
        encoded = {
            str(name): (
                value
                if isinstance(value, str)
                else encode_image_b64(value, color_order=self.color_order)
            )
            for name, value in images.items()
        }
        payload: dict[str, Any] = {
            "instruction": text,
            "images": encoded,
            "state": [float(item) for item in (state or [])],
            "task_id": str(task_id or ""),
            "session_id": str(session_id or ""),
            "reset": bool(reset),
            "options": {"return_mapped": bool(return_mapped)},
        }
        if chunk_size is not None:
            payload["options"]["chunk_size"] = int(chunk_size)
        return self._request("POST", "/v1/infer", payload)

    # ------------------------------------------------------------------- http
    def _request(
        self, method: str, path: str, payload: Optional[Mapping[str, Any]] = None
    ) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        headers = {"Accept": "application/json"}
        data = None
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(request, timeout=self.timeout_sec) as response:
                raw = response.read(2 * 1024 * 1024)
                try:
                    body = json.loads(raw.decode("utf-8")) if raw else {}
                except (UnicodeDecodeError, ValueError) as exc:
                    raise CenterClientError(
                        f"Center Server 返回非法 JSON: {url}", code="invalid_json"
                    ) from exc
                if not isinstance(body, dict):
                    raise CenterClientError(
                        f"Center Server 返回非对象: {url}", code="invalid_json"
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
            raise CenterClientError(
                str(error.get("message") or f"HTTP {exc.code}"),
                status=int(exc.code),
                code=str(error.get("code") or "http_error"),
                payload=body,
            ) from exc
        except (urllib.error.URLError, socket.timeout, TimeoutError) as exc:
            raise CenterClientError(
                f"无法连接 Center Server {url}: {exc}", code="connection_error"
            ) from exc
        except CenterClientError:
            raise
        except OSError as exc:
            raise CenterClientError(
                f"请求 Center Server 失败 {url}: {exc}", code="connection_error"
            ) from exc


__all__ = [
    "CenterClient",
    "CenterClientError",
    "base_url_from_registry",
    "discover_center",
    "encode_image_b64",
]
