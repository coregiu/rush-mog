"""服务内部统一异常。

HTTP 层只依赖 ``ApiError``，核心层/模型层可以抛出同样的异常，
这样错误码、状态码和响应体才能保持一致。
"""

from __future__ import annotations


class ApiError(Exception):
    status_code = 500
    code = "internal_error"

    def __init__(
        self,
        message: str,
        *,
        status_code: int | None = None,
        code: str | None = None,
        details: dict | None = None,
    ) -> None:
        super().__init__(message)
        self.message = str(message)
        if status_code is not None:
            self.status_code = int(status_code)
        if code is not None:
            self.code = str(code)
        self.details = details or {}

    def to_dict(self) -> dict:
        payload = {"code": self.code, "message": self.message}
        if self.details:
            payload["details"] = self.details
        return payload


class BadRequest(ApiError):
    status_code = 400
    code = "bad_request"


class Unauthorized(ApiError):
    status_code = 401
    code = "unauthorized"


class NotFound(ApiError):
    status_code = 404
    code = "not_found"


class PayloadTooLarge(ApiError):
    status_code = 413
    code = "payload_too_large"


class UnsupportedMediaType(ApiError):
    status_code = 415
    code = "unsupported_media_type"


class MethodNotAllowed(ApiError):
    status_code = 405
    code = "method_not_allowed"


class UnprocessableEntity(ApiError):
    status_code = 422
    code = "invalid_request"


class ModelUnavailable(ApiError):
    status_code = 503
    code = "model_unavailable"


class InferenceError(ApiError):
    status_code = 500
    code = "inference_error"
