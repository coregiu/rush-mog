"""推理服务业务层：请求校验、模型调用、动作映射、响应组装。

HTTP 适配层只负责解析/返回 JSON，所有机器人相关语义集中在这里，
方便以后换成 FastAPI、gRPC 或在 ROS 节点内直接调用。
"""

from __future__ import annotations

import logging
import math
import time
import uuid
from datetime import datetime, timezone
from typing import Any, Mapping

import numpy as np

from . import SERVICE_NAME, __version__
from .action_mapper import ActionMapper
from .engine import BaseEngine, create_engine
from .errors import ApiError, BadRequest, ModelUnavailable, UnprocessableEntity
from .images import decode_image

LOGGER = logging.getLogger("center_server.service")

_MAX_INSTRUCTION_CHARS = 512
_MAX_IMAGES = 8
_MAX_CHUNK_SIZE = 1000


class InferenceService:
    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        engine: BaseEngine | None = None,
    ) -> None:
        self.cfg = dict(config)
        self.server_cfg = dict(self.cfg.get("server") or {})
        self.model_cfg = dict(self.cfg.get("model") or {})
        self.engine = engine or create_engine(self.model_cfg)
        self.mapper = ActionMapper(self.cfg.get("action") or {})
        self.started_at = time.time()
        if self.model_cfg.get("preload"):
            try:
                self.engine.ensure_loaded()
            except ApiError as exc:
                # 预加载失败不退出：/health 会报告 degraded，请求时再重试
                LOGGER.error("模型预加载失败: %s", exc.message)

        self._warn_if_base_checkpoint()

    def _warn_if_base_checkpoint(self) -> None:
        if str(self.model_cfg.get("backend", "")).lower() != "smolvla":
            return
        path = str(self.model_cfg.get("path") or "").rstrip("/").lower()
        if not path.endswith("smolvla_base"):
            return
        if self.mapper.mode in {"base_twist", "hybrid"}:
            LOGGER.warning(
                "检测到通用 checkpoint %s，但 action.mode=%s；"
                "该 checkpoint 的动作空间取决于训练机器人，不能直接作为本车"
                "速度策略。请先微调或改用 action.mode=raw 验证链路。",
                self.model_cfg.get("path"),
                self.mapper.mode,
            )

    # ----------------------------------------------------------------- health
    def status(self) -> dict[str, Any]:
        model = self.engine.status()
        ready = bool(model.get("loaded")) or model.get("backend") == "stub"
        return {
            "ok": True,
            "status": "ok" if ready else "degraded",
            "ready": ready,
            "service": SERVICE_NAME,
            "version": __version__,
            "uptime_sec": round(time.time() - self.started_at, 3),
            "model": model,
        }

    def ready(self) -> bool:
        model = self.engine.status()
        return bool(model.get("loaded")) or model.get("backend") == "stub"

    def model_info(self) -> dict[str, Any]:
        model = self.engine.status()
        model["configured_image_keys"] = list(self.model_cfg.get("image_keys") or [])
        model["image_aliases"] = dict(self.model_cfg.get("image_aliases") or {})
        model["action_mode"] = self.mapper.mode
        return model

    # ------------------------------------------------------------------ params
    @staticmethod
    def _param(
        key: str,
        label: str,
        value: Any,
        value_type: str,
        group: str,
        *,
        unit: str = "",
        min_value: float | None = None,
        max_value: float | None = None,
        step: float | None = None,
        options: list[str] | None = None,
        help_text: str = "",
    ) -> dict[str, Any]:
        item: dict[str, Any] = {
            "key": key,
            "label": label,
            "value": value,
            "type": value_type,
            "group": group,
            "editable": True,
            "requires_restart": False,
        }
        if unit:
            item["unit"] = unit
        if min_value is not None:
            item["min"] = min_value
        if max_value is not None:
            item["max"] = max_value
        if step is not None:
            item["step"] = step
        if options:
            item["options"] = list(options)
        if help_text:
            item["help"] = help_text
        return item

    def params(self) -> dict[str, Any]:
        mapper = self.mapper
        model = self.engine.status()
        params = [
            self._param(
                "action.mode", "动作映射模式", mapper.mode, "enum", "动作映射",
                options=["raw", "base_twist", "hybrid", "joint"],
                help_text="base_twist/hybrid 需要本车微调过的动作空间",
            ),
            self._param(
                "action.normalized", "模型输出归一化", mapper.normalized, "bool",
                "动作映射",
            ),
            self._param(
                "action.max_linear_x", "最大线速度 X", mapper.max_values[0], "float",
                "速度限幅", unit="m/s", min_value=0.0, max_value=5.0, step=0.05,
            ),
            self._param(
                "action.max_linear_y", "最大线速度 Y", mapper.max_values[1], "float",
                "速度限幅", unit="m/s", min_value=0.0, max_value=5.0, step=0.05,
            ),
            self._param(
                "action.max_angular_z", "最大角速度 Z", mapper.max_values[2], "float",
                "速度限幅", unit="rad/s", min_value=0.0, max_value=5.0, step=0.05,
            ),
            self._param(
                "action.deadband", "速度死区", mapper.deadband, "float",
                "速度限幅", min_value=0.0, max_value=1.0, step=0.01,
            ),
            self._param(
                "action.arm_start", "机械臂起始维度", mapper.arm_start, "int",
                "机械臂", min_value=0.0, max_value=32.0, step=1.0,
            ),
            self._param(
                "action.round_digits", "动作保留小数位", mapper.round_digits, "int",
                "动作映射", min_value=0.0, max_value=6.0, step=1.0,
            ),
        ]
        return {
            "ok": True,
            "module": "center",
            "name": "SmolVLA Center Server",
            "version": __version__,
            "editable": True,
            "requires_restart": False,
            "params": params,
            "info": {
                "backend": model.get("backend"),
                "loaded": bool(model.get("loaded")),
                "device": model.get("device"),
                "dtype": model.get("dtype"),
                "action_dim": model.get("action_dim"),
                "state_dim": model.get("state_dim"),
                "chunk_size": model.get("chunk_size"),
                "action_mode": mapper.mode,
            },
        }

    @staticmethod
    def _param_float(value: Any, key: str, low: float, high: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise UnprocessableEntity(f"{key} 必须是数字", code="invalid_param")
        number = float(value)
        if not math.isfinite(number) or number < low or number > high:
            raise UnprocessableEntity(
                f"{key} 必须在 {low}..{high}", code="invalid_param"
            )
        return number

    def update_params(self, payload: Any) -> dict[str, Any]:
        body = self._require_mapping(payload)
        updates = body.get("updates", body)
        if not isinstance(updates, Mapping) or not updates:
            raise BadRequest("updates 必须是非空对象", code="invalid_params")

        allowed = {
            "action.mode",
            "action.normalized",
            "action.max_linear_x",
            "action.max_linear_y",
            "action.max_angular_z",
            "action.deadband",
            "action.arm_start",
            "action.round_digits",
        }
        validated: dict[str, Any] = {}
        for raw_key, raw_value in updates.items():
            key = str(raw_key)
            if key not in allowed:
                raise UnprocessableEntity(
                    f"不支持的参数: {key}", code="invalid_param"
                )
            if key == "action.mode":
                mode = str(raw_value).strip().lower()
                if mode not in {"raw", "base_twist", "hybrid", "joint"}:
                    raise UnprocessableEntity(
                        "action.mode 必须是 raw/base_twist/hybrid/joint",
                        code="invalid_param",
                    )
                validated[key] = mode
            elif key == "action.normalized":
                if isinstance(raw_value, bool):
                    validated[key] = raw_value
                elif isinstance(raw_value, (int, float)):
                    validated[key] = bool(raw_value)
                else:
                    raise UnprocessableEntity(
                        "action.normalized 必须是布尔值", code="invalid_param"
                    )
            elif key == "action.arm_start":
                validated[key] = int(
                    self._param_float(raw_value, key, 0.0, 32.0)
                )
            elif key == "action.round_digits":
                validated[key] = int(
                    self._param_float(raw_value, key, 0.0, 6.0)
                )
            else:
                limits = {
                    "action.max_linear_x": 5.0,
                    "action.max_linear_y": 5.0,
                    "action.max_angular_z": 5.0,
                    "action.deadband": 1.0,
                }
                validated[key] = self._param_float(
                    raw_value, key, 0.0, limits[key]
                )

        action_cfg = dict(self.cfg.get("action") or {})
        for key, value in validated.items():
            action_cfg[key.split(".", 1)[1]] = value

        # 先构造新 mapper，校验失败不会破坏当前推理链路。
        mapper = ActionMapper(action_cfg)
        self.mapper = mapper
        self.cfg["action"] = action_cfg
        return {
            "ok": True,
            "module": "center",
            "updated": validated,
            "requires_restart": False,
            "message": "参数已更新",
        }

    # ------------------------------------------------------------------ infer
    def infer(
        self, payload: Any, *, request_id: str | None = None
    ) -> dict[str, Any]:
        started = time.perf_counter()
        request_id = request_id or uuid.uuid4().hex
        body = self._require_mapping(payload)

        instruction = self._instruction(body)
        task_id = str(body.get("task_id") or request_id)[:128]
        session_id = str(body.get("session_id") or "")[:128]
        warnings: list[str] = []

        if bool(body.get("reset")):
            self.engine.reset()

        # 先确保模型已加载：模型 checkpoint 可能覆盖配置里的相机键/状态维度，
        # 后面的图像校验必须使用最终 schema。
        self.engine.ensure_loaded()
        images, image_warnings = self._build_images(body.get("images"))
        warnings.extend(image_warnings)
        state, state_warnings = self._build_state(body.get("state"))
        warnings.extend(state_warnings)
        chunk_size = self._chunk_size(body)

        result = self.engine.infer(
            instruction,
            images,
            state,
            chunk_size=chunk_size,
        )
        mapped = [self.mapper.map(action) for action in result.actions]
        if not mapped:
            raise ModelUnavailable("模型没有返回任何动作")
        first = mapped[0]

        model_status = self.engine.status()
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        response: dict[str, Any] = {
            "ok": True,
            "request_id": request_id,
            "task_id": task_id,
            "session_id": session_id,
            "instruction": instruction,
            "actions": result.actions,
            "action": result.actions[0],
            "action_dim": len(result.actions[0]),
            "chunk_size": len(result.actions),
            "base_cmd": first["base_cmd"],
            "arm_cmd": first["arm_cmd"],
            "mapped": first,
            "model": {
                "backend": model_status.get("backend"),
                "path": model_status.get("path"),
                "device": model_status.get("device"),
                "dtype": model_status.get("dtype"),
            },
            "latency_ms": round(elapsed_ms, 3),
            "model_latency_ms": round(result.latency_ms, 3),
            "timestamp": _utc_now(),
            "warnings": warnings,
        }
        if bool((body.get("options") or {}).get("return_mapped")):
            response["mapped_actions"] = mapped
        return response

    # ------------------------------------------------------------------ reset
    def reset(self, payload: Any = None) -> dict[str, Any]:
        body = payload if isinstance(payload, Mapping) else {}
        self.engine.reset()
        return {
            "ok": True,
            "session_id": str(body.get("session_id") or "")[:128],
            "message": "SmolVLA action queue reset",
            "timestamp": _utc_now(),
        }

    def warmup(self) -> dict[str, Any]:
        started = time.perf_counter()
        result = self.engine.warmup()
        return {
            "ok": True,
            "latency_ms": round((time.perf_counter() - started) * 1000.0, 3),
            "model_latency_ms": round(result.latency_ms, 3),
            "model": self.engine.status(),
            "timestamp": _utc_now(),
        }

    def close(self) -> None:
        self.engine.close()

    # -------------------------------------------------------------- validation
    @staticmethod
    def _require_mapping(payload: Any) -> Mapping[str, Any]:
        if not isinstance(payload, Mapping):
            raise BadRequest("请求体必须是 JSON 对象", code="invalid_json")
        return payload

    @staticmethod
    def _instruction(body: Mapping[str, Any]) -> str:
        raw = body.get("instruction")
        if raw is None:
            raw = body.get("task") or body.get("language_instruction")
        if not isinstance(raw, str):
            raise UnprocessableEntity(
                "instruction 必须是字符串", code="instruction_required"
            )
        instruction = raw.strip()
        if not instruction:
            raise UnprocessableEntity(
                "instruction 不能为空；SmolVLA 需要语言指令",
                code="instruction_required",
            )
        if len(instruction) > _MAX_INSTRUCTION_CHARS:
            raise UnprocessableEntity(
                f"instruction 超过 {_MAX_INSTRUCTION_CHARS} 字符",
                code="instruction_too_long",
            )
        return instruction

    def _allowed_image_keys(self) -> list[str]:
        keys = list(getattr(self.engine, "image_keys", None) or [])
        if not keys:
            keys = list(self.model_cfg.get("image_keys") or [])
        return keys

    def _resolve_image_key(self, name: str) -> str | None:
        keys = self._allowed_image_keys()
        aliases = self.model_cfg.get("image_aliases") or {}
        prefix = "observation.images."

        def clean(value: str) -> str:
            return value[len(prefix) :] if value.startswith(prefix) else value

        cleaned_name = clean(name)
        target = aliases.get(name) or aliases.get(cleaned_name)
        if target is not None and clean(str(target)) in keys:
            return clean(str(target))
        if cleaned_name in keys:
            return cleaned_name
        lowered = {key.lower(): key for key in keys}
        return lowered.get(cleaned_name.lower())

    def _build_images(
        self, raw_images: Any
    ) -> tuple[dict[str, np.ndarray], list[str]]:
        warnings: list[str] = []
        if raw_images is None:
            raw_images = {}
        if not isinstance(raw_images, Mapping):
            raise UnprocessableEntity(
                "images 必须是 {相机名: base64} 的映射", code="invalid_images"
            )
        if len(raw_images) > _MAX_IMAGES:
            raise UnprocessableEntity(
                f"images 最多 {_MAX_IMAGES} 路", code="too_many_images"
            )

        model_keys = self._allowed_image_keys()
        decoded: dict[str, np.ndarray] = {}
        max_image_bytes = int(
            self.server_cfg.get("max_image_bytes", 6 * 1024 * 1024)
        )

        for name, value in raw_images.items():
            if not isinstance(name, str):
                raise UnprocessableEntity(
                    "images 的键必须是字符串", code="invalid_images"
                )
            model_key = self._resolve_image_key(name)
            if model_key is None:
                raise UnprocessableEntity(
                    f"未知相机名: {name}",
                    code="unknown_image_key",
                    details={
                        "allowed": model_keys,
                        "aliases": dict(self.model_cfg.get("image_aliases") or {}),
                    },
                )
            if model_key in decoded:
                raise UnprocessableEntity(
                    f"相机 {model_key} 被重复提供", code="duplicate_image"
                )
            decoded[model_key] = decode_image(
                value,
                max_bytes=max_image_bytes,
                field=f"images.{name}",
            )

        require_images = bool(self.model_cfg.get("require_images", True))
        if not decoded and require_images:
            raise UnprocessableEntity(
                "缺少图像输入", code="missing_image"
            )

        policy = str(self.model_cfg.get("missing_image_policy", "copy_first"))
        first_image = next(iter(decoded.values()), None)
        filled: dict[str, np.ndarray] = {}
        for key in model_keys:
            if key in decoded:
                filled[key] = decoded[key]
                continue
            if not require_images and first_image is None:
                continue
            if policy == "error":
                raise UnprocessableEntity(
                    f"缺少模型相机图像: {key}", code="missing_image"
                )
            if policy == "zeros":
                shape = first_image.shape if first_image is not None else (3, 224, 224)
                filled[key] = np.zeros(shape, dtype=np.float32)
                warnings.append(f"相机 {key} 缺失，已补零")
            else:  # copy_first
                if first_image is None:
                    raise UnprocessableEntity(
                        "缺少图像输入", code="missing_image"
                    )
                filled[key] = first_image
                warnings.append(f"相机 {key} 缺失，已复制第一路图像")

        return filled, warnings

    def _build_state(self, raw_state: Any) -> tuple[list[float], list[str]]:
        warnings: list[str] = []
        if raw_state is None:
            raw_state = []
        if not isinstance(raw_state, (list, tuple)):
            raise UnprocessableEntity(
                "state 必须是数字数组", code="invalid_state"
            )
        values: list[float] = []
        for item in raw_state:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise UnprocessableEntity(
                    f"state 包含非数字值: {item!r}", code="invalid_state"
                )
            value = float(item)
            if not math.isfinite(value):
                raise UnprocessableEntity(
                    f"state 包含非有限值: {item!r}", code="invalid_state"
                )
            values.append(value)

        if bool(self.model_cfg.get("require_state", False)) and not values:
            raise UnprocessableEntity("缺少 state 输入", code="missing_state")

        model_status = self.engine.status()
        state_dim = int(
            model_status.get("state_dim")
            or self.model_cfg.get("state_dim")
            or 0
        )
        if state_dim <= 0:
            return values, warnings

        if len(values) < state_dim:
            if not bool(self.model_cfg.get("pad_state", True)):
                raise UnprocessableEntity(
                    f"state 维度不足: {len(values)} < {state_dim}",
                    code="invalid_state",
                )
            original_dim = len(values)
            values.extend([0.0] * (state_dim - original_dim))
            warnings.append(f"state 已从 {original_dim} 补齐到 {state_dim} 维")
        elif len(values) > state_dim:
            warnings.append(
                f"state 从 {len(values)} 截断到模型要求的 {state_dim} 维"
            )
            values = values[:state_dim]
        return values, warnings

    def _chunk_size(self, body: Mapping[str, Any]) -> int | None:
        options = body.get("options") or {}
        if not isinstance(options, Mapping):
            raise UnprocessableEntity("options 必须是对象", code="invalid_options")
        raw = options.get("chunk_size", body.get("chunk_size"))
        if raw is None:
            return None
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise UnprocessableEntity(
                "chunk_size 必须是整数", code="invalid_options"
            )
        if not 1 <= raw <= _MAX_CHUNK_SIZE:
            raise UnprocessableEntity(
                f"chunk_size 必须在 1..{_MAX_CHUNK_SIZE}",
                code="invalid_options",
            )
        return raw


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
