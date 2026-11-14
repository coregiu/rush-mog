"""图像编解码工具。

HTTP 请求使用 JSON，图像以 base64（可带 ``data:image/...;base64,`` 前缀）
传输。服务端统一解成 ``float32 CHW，范围 [0, 1]``，与 LeRobot 数据集的
图像约定一致；缩放/归一化的具体尺寸由 SmolVLA 的预处理器负责。
"""

from __future__ import annotations

import base64
import binascii
import io
from typing import Any

import numpy as np
from PIL import Image, ImageOps

from .errors import BadRequest, PayloadTooLarge

# 防止解压炸弹；Rush-MOG 相机远小于这个值。
Image.MAX_IMAGE_PIXELS = 40_000_000

_DEFAULT_MAX_BYTES = 6 * 1024 * 1024
_DEFAULT_MAX_PIXELS = 25_000_000


def _decode_base64(raw: str) -> bytes:
    compact = "".join(raw.split())
    if not compact:
        raise BadRequest("图像 base64 为空", code="invalid_image")
    try:
        return base64.b64decode(compact, validate=True)
    except (binascii.Error, ValueError):
        # 兼容 URL-safe base64 与缺失 padding 的客户端
        padded = compact + "=" * (-len(compact) % 4)
        try:
            return base64.urlsafe_b64decode(padded)
        except (binascii.Error, ValueError) as exc:
            raise BadRequest(
                "图像 base64 解码失败", code="invalid_image"
            ) from exc


def decode_image(
    value: Any,
    *,
    max_bytes: int = _DEFAULT_MAX_BYTES,
    max_pixels: int = _DEFAULT_MAX_PIXELS,
    field: str = "images",
) -> np.ndarray:
    """把 base64/data URI/``{"data": ...}`` 解成 float32 CHW。

    返回 shape ``(3, H, W)``，范围 ``[0, 1]``。
    """

    if isinstance(value, dict):
        raw = value.get("data") or value.get("base64") or value.get("image")
        if not isinstance(raw, str):
            raise BadRequest(
                f"{field} 必须是 base64 字符串或包含 data 字段的对象",
                code="invalid_image",
            )
    elif isinstance(value, str):
        raw = value
    else:
        raise BadRequest(
            f"{field} 必须是 base64 字符串或对象", code="invalid_image"
        )

    raw = raw.strip()
    if raw.startswith("data:"):
        header, separator, payload = raw.partition(",")
        if not separator or ";base64" not in header.lower():
            raise BadRequest(
                f"{field} 的 data URI 必须使用 base64 编码",
                code="invalid_image",
            )
        raw = payload

    data = _decode_base64(raw)
    if len(data) > max_bytes:
        raise PayloadTooLarge(
            f"单张图像超过上限 {max_bytes} 字节",
            details={"field": field, "bytes": len(data), "max_bytes": max_bytes},
        )

    try:
        with Image.open(io.BytesIO(data)) as image:
            image = ImageOps.exif_transpose(image)
            width, height = image.size
            if width <= 0 or height <= 0:
                raise ValueError("图像尺寸为 0")
            if width * height > max_pixels:
                raise PayloadTooLarge(
                    "图像像素数超过上限",
                    details={
                        "field": field,
                        "width": width,
                        "height": height,
                        "max_pixels": max_pixels,
                    },
                )
            rgb = image.convert("RGB")
            array = np.asarray(rgb, dtype=np.float32) / 255.0
    except PayloadTooLarge:
        raise
    except (OSError, ValueError) as exc:
        raise BadRequest(
            f"{field} 不是可解码的图像: {exc}", code="invalid_image"
        ) from exc

    if array.ndim != 3 or array.shape[2] != 3:
        raise BadRequest(f"{field} 图像通道数异常", code="invalid_image")
    array = np.ascontiguousarray(np.transpose(array, (2, 0, 1)))
    if not np.isfinite(array).all():
        raise BadRequest(f"{field} 图像包含非有限值", code="invalid_image")
    return array


def encode_image_b64(image: Any, *, image_format: str = "PNG") -> str:
    """把 CHW/HWC、float[0,1]/uint8 图像编码成 data URI。

    主要给 ROS 侧 HTTP 客户端和调试脚本复用。
    """

    array = np.asarray(image)
    if array.ndim == 3 and array.shape[0] in (1, 3, 4) and array.shape[-1] not in (
        1,
        3,
        4,
    ):
        array = np.transpose(array, (1, 2, 0))
    if array.ndim == 2:
        array = np.repeat(array[:, :, None], 3, axis=2)
    if array.ndim != 3 or array.shape[2] not in (1, 3, 4):
        raise ValueError(f"不支持的图像 shape: {array.shape}")

    if array.dtype != np.uint8:
        array = array.astype(np.float32)
        if array.size and float(np.nanmax(array)) <= 1.0:
            array = array * 255.0
        array = np.clip(np.nan_to_num(array), 0.0, 255.0).astype(np.uint8)

    if array.shape[2] == 1:
        array = array[:, :, 0]
    elif array.shape[2] == 3:
        # 约定输入为 RGB；若调用方传 BGR，请先自行转换。
        pass
    elif array.shape[2] == 4:
        array = array[:, :, :3]

    fmt = image_format.upper()
    if fmt not in {"PNG", "JPEG", "JPG", "WEBP"}:
        raise ValueError(f"不支持的图像格式: {image_format}")
    if fmt == "JPG":
        fmt = "JPEG"

    buffer = io.BytesIO()
    Image.fromarray(array, mode="RGB").save(buffer, format=fmt)
    encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
    mime = "image/jpeg" if fmt == "JPEG" else f"image/{fmt.lower()}"
    return f"data:{mime};base64,{encoded}"
