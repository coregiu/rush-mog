"""SmolVLA 推理后端。

设计目标：

- 模型懒加载，首次请求时加载，避免服务启动即占用显存；
- 兼容 LeRobot 0.4+ 的 ``make_pre_post_processors`` 处理器管线；
- 单进程/单模型串行推理（``threading.RLock``），HTTP 层可以多线程收请求；
- 模型/依赖缺失时返回明确的 503，而不是伪造动作；
- ``stub`` 后端仅用于联调和单元测试。
"""

from __future__ import annotations

import importlib
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from .errors import InferenceError, ModelUnavailable, UnprocessableEntity

LOGGER = logging.getLogger("center_server.engine")


@dataclass
class EngineResult:
    actions: list[list[float]]
    latency_ms: float
    model: dict[str, Any] = field(default_factory=dict)


class BaseEngine:
    backend = "base"
    image_keys: list[str] = []

    def status(self) -> dict[str, Any]:
        raise NotImplementedError

    def ensure_loaded(self) -> None:
        raise NotImplementedError

    def infer(
        self,
        instruction: str,
        images: Mapping[str, np.ndarray],
        state: Sequence[float],
        *,
        chunk_size: int | None = None,
    ) -> EngineResult:
        raise NotImplementedError

    def warmup(self) -> EngineResult:
        image = np.zeros((3, 64, 64), dtype=np.float32)
        images = {key: image for key in self.image_keys}
        state = [0.0] * int(self.status().get("state_dim", 0))
        return self.infer("warmup", images, state, chunk_size=1)

    def reset(self) -> None:
        return None

    def close(self) -> None:
        return None


class StubEngine(BaseEngine):
    """固定动作的桩后端，只用于 HTTP/映射联调。"""

    backend = "stub"

    def __init__(self, model_cfg: Mapping[str, Any]) -> None:
        self.cfg = dict(model_cfg)
        self.image_keys = list(self.cfg.get("image_keys") or [])
        self.action = [float(item) for item in (self.cfg.get("stub_action") or [])]
        self._loaded = True

    def status(self) -> dict[str, Any]:
        return {
            "backend": self.backend,
            "path": "stub",
            "loaded": True,
            "loading": False,
            "device": "cpu",
            "dtype": "float32",
            "action_dim": len(self.action),
            "state_dim": int(self.cfg.get("state_dim", 0)),
            "chunk_size": 1,
            "image_keys": list(self.image_keys),
            "processors": False,
            "error": None,
        }

    def ensure_loaded(self) -> None:
        return None

    def infer(
        self,
        instruction: str,
        images: Mapping[str, np.ndarray],
        state: Sequence[float],
        *,
        chunk_size: int | None = None,
    ) -> EngineResult:
        started = time.perf_counter()
        actions = [list(self.action)]
        return EngineResult(
            actions=actions,
            latency_ms=(time.perf_counter() - started) * 1000.0,
            model={"backend": self.backend, "path": "stub", "device": "cpu"},
        )


class SmolVLAEngine(BaseEngine):
    backend = "smolvla"

    def __init__(self, model_cfg: Mapping[str, Any]) -> None:
        self.cfg = dict(model_cfg)
        self.path = str(self.cfg.get("path") or "lerobot/smolvla_base")
        self.requested_device = str(self.cfg.get("device") or "auto")
        self.requested_dtype = str(self.cfg.get("dtype") or "auto")
        self.reload_interval = float(self.cfg.get("reload_interval_sec", 10.0))
        self.require_processors = bool(self.cfg.get("require_processors", True))

        self.image_keys = list(self.cfg.get("image_keys") or [])
        self.state_dim = int(self.cfg.get("state_dim", 6))
        self.chunk_size = int(self.cfg.get("chunk_size", 50))
        self.n_action_steps = int(self.cfg.get("n_action_steps", 1))
        self.action_dim: int | None = None

        self._lock = threading.RLock()
        self._torch: Any = None
        self._policy: Any = None
        self._preprocessor: Any = None
        self._postprocessor: Any = None
        self._device = "cpu"
        self._dtype_name = "float32"
        self._loading = False
        self._error: str | None = None
        self._error_at = 0.0

    # ------------------------------------------------------------------ load
    def ensure_loaded(self) -> None:
        if self._policy is not None:
            return
        with self._lock:
            if self._policy is not None:
                return
            if (
                self._error
                and self._error_at
                and time.monotonic() - self._error_at < self.reload_interval
            ):
                raise ModelUnavailable(
                    f"SmolVLA 模型不可用: {self._error}",
                    details={"path": self.path, "retry_after_sec": self.reload_interval},
                )
            self._load_locked()

    def _load_locked(self) -> None:
        self._loading = True
        self._error = None
        try:
            self._torch = self._import_torch()
            self._policy = self._load_policy()
            self._device = self._resolve_device(self._torch)
            self._dtype_name, torch_dtype = self._resolve_dtype(self._torch)
            self._move_policy(self._policy, torch_dtype)
            self._preprocessor, self._postprocessor = self._build_processors()
            self._resolve_feature_schema()
            LOGGER.info(
                "SmolVLA loaded: path=%s device=%s dtype=%s processors=%s "
                "images=%s state_dim=%s action_dim=%s chunk_size=%s",
                self.path,
                self._device,
                self._dtype_name,
                self._preprocessor is not None,
                self.image_keys,
                self.state_dim,
                self.action_dim,
                self.chunk_size,
            )
        except Exception as exc:  # noqa: BLE001 - 统一转成 503
            self._policy = None
            self._preprocessor = None
            self._postprocessor = None
            self._error = f"{type(exc).__name__}: {exc}"
            self._error_at = time.monotonic()
            if isinstance(exc, ModelUnavailable):
                LOGGER.error(
                    "SmolVLA unavailable (%s): %s", self.path, exc.message
                )
                raise
            LOGGER.exception("failed to load SmolVLA model from %s", self.path)
            raise ModelUnavailable(
                f"SmolVLA 模型加载失败: {exc}",
                details={
                    "path": self.path,
                    "hint": (
                        "确认已安装 torch/lerobot（pip install -r requirements-vla.txt），"
                        "且模型路径/HF 仓库名可访问"
                    ),
                },
            ) from exc
        finally:
            self._loading = False

    @staticmethod
    def _import_torch() -> Any:
        try:
            import torch  # noqa: F401
        except ImportError as exc:
            raise ModelUnavailable(
                "未安装 PyTorch，无法加载 SmolVLA",
                details={"install": "pip install -r requirements-vla.txt"},
            ) from exc
        return torch

    def _load_policy(self) -> Any:
        # 兼容不同 LeRobot 版本的导入路径
        candidates = (
            "lerobot.policies.smolvla.modeling_smolvla",
            "lerobot.policies.smolvla",
        )
        policy_cls = None
        errors: list[str] = []
        for module_name in candidates:
            try:
                module = importlib.import_module(module_name)
                policy_cls = getattr(module, "SmolVLAPolicy", None)
                if policy_cls is not None:
                    break
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{module_name}: {type(exc).__name__}: {exc}")

        if policy_cls is None:
            raise ModelUnavailable(
                "未找到 lerobot 的 SmolVLAPolicy，请安装支持 SmolVLA 的 LeRobot",
                details={
                    "install": 'pip install "lerobot[smolvla]"',
                    "errors": errors,
                },
            )

        kwargs: dict[str, Any] = {}
        if self.cfg.get("local_files_only"):
            kwargs["local_files_only"] = True
        if self.cfg.get("trust_remote_code"):
            kwargs["trust_remote_code"] = True

        try:
            return policy_cls.from_pretrained(self.path, **kwargs)
        except TypeError:
            # 某些版本不接收 local_files_only/trust_remote_code
            return policy_cls.from_pretrained(self.path)

    def _resolve_device(self, torch: Any) -> str:
        requested = self.requested_device
        if requested == "auto":
            if torch.cuda.is_available():
                return "cuda"
            mps = getattr(getattr(torch, "backends", None), "mps", None)
            if mps is not None and getattr(mps, "is_available", lambda: False)():
                return "mps"
            return "cpu"

        if requested.startswith("cuda") and not torch.cuda.is_available():
            LOGGER.warning("CUDA 不可用，回退到 CPU（配置为 %s）", requested)
            return "cpu"
        if requested == "mps":
            mps = getattr(getattr(torch, "backends", None), "mps", None)
            if mps is None or not getattr(mps, "is_available", lambda: False)():
                LOGGER.warning("MPS 不可用，回退到 CPU")
                return "cpu"
        return requested

    def _resolve_dtype(self, torch: Any) -> tuple[str, Any]:
        requested = self.requested_dtype
        if requested == "auto":
            # 保持 checkpoint 原始 dtype（通常是 float32）；显式配置才转换
            return "auto", None
        aliases = {
            "fp32": "float32",
            "fp16": "float16",
            "bf16": "bfloat16",
        }
        requested = aliases.get(requested, requested)
        if requested == "float16" and not self._device.startswith("cuda"):
            LOGGER.warning("float16 仅建议在 CUDA 上使用，回退到 float32")
            requested = "float32"
        mapping = {
            "float32": torch.float32,
            "float16": torch.float16,
            "bfloat16": getattr(torch, "bfloat16", torch.float32),
        }
        return requested, mapping[requested]

    def _move_policy(self, policy: Any, torch_dtype: Any) -> None:
        policy.to(self._device)
        if torch_dtype is not None:
            try:
                policy.to(dtype=torch_dtype)
            except (TypeError, RuntimeError) as exc:
                LOGGER.warning("模型 dtype 转换失败，保持 checkpoint dtype: %s", exc)
        policy.eval()
        try:
            self._dtype_name = str(next(policy.parameters()).dtype).replace(
                "torch.", ""
            )
        except (StopIteration, AttributeError):
            if torch_dtype is not None:
                self._dtype_name = str(self.requested_dtype)

    def _build_processors(self) -> tuple[Any, Any]:
        make_pre_post_processors = None
        import_errors: list[str] = []
        for module_name in (
            "lerobot.policies.factory",
            "lerobot.processor",
            "lerobot.processor.factory",
        ):
            try:
                module = importlib.import_module(module_name)
                candidate = getattr(module, "make_pre_post_processors", None)
                if callable(candidate):
                    make_pre_post_processors = candidate
                    break
            except Exception as exc:  # noqa: BLE001
                import_errors.append(f"{module_name}: {type(exc).__name__}: {exc}")

        if make_pre_post_processors is None:
            if self.require_processors:
                raise ModelUnavailable(
                    "当前 LeRobot 版本缺少 make_pre_post_processors，无法完成 "
                    "SmolVLA 的图像/语言/归一化预处理",
                    details={
                        "install": 'pip install -U "lerobot[smolvla]"',
                        "errors": import_errors,
                    },
                )
            LOGGER.warning("make_pre_post_processors 不可用: %s", import_errors)
            return None, None

        overrides = {"device_processor": {"device": self._device}}
        attempts = (
            dict(
                policy_cfg=self._policy.config,
                pretrained_path=self.path,
                preprocessor_overrides=overrides,
            ),
            dict(policy_cfg=self._policy.config, pretrained_path=self.path),
            dict(policy_cfg=self._policy.config),
        )
        errors: list[str] = []
        for kwargs in attempts:
            try:
                preprocessor, postprocessor = make_pre_post_processors(**kwargs)
                if preprocessor is not None and postprocessor is not None:
                    return preprocessor, postprocessor
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{type(exc).__name__}: {exc}")

        message = "无法构建 SmolVLA 预处理器/后处理器"
        if self.require_processors:
            raise ModelUnavailable(
                message,
                details={"path": self.path, "errors": errors[-3:]},
            )
        LOGGER.warning("%s，将直接使用原始 batch: %s", message, errors[-1:])
        return None, None

    def _resolve_feature_schema(self) -> None:
        config = getattr(self._policy, "config", None)

        policy_images = self._policy_image_keys(config)
        if policy_images:
            if self.image_keys and policy_images != self.image_keys:
                LOGGER.warning(
                    "模型相机键 %s 与配置 %s 不同，以模型为准",
                    policy_images,
                    self.image_keys,
                )
            self.image_keys = policy_images
        if not self.image_keys:
            raise ModelUnavailable(
                "模型没有可用的图像输入，且配置未提供 model.image_keys"
            )

        policy_state_dim = self._feature_dim(
            self._feature(config, "input_features", "observation.state")
        )
        if policy_state_dim:
            if policy_state_dim != self.state_dim:
                LOGGER.info(
                    "模型 state_dim=%s，覆盖配置 %s", policy_state_dim, self.state_dim
                )
            self.state_dim = policy_state_dim

        action_feature = self._feature(config, "output_features", "action")
        action_dim = self._feature_dim(action_feature)
        if action_dim is None:
            action_dim = self._feature_dim(
                self._cfg_value(config, "action_feature")
            )
        if action_dim:
            self.action_dim = action_dim

        policy_chunk = self._cfg_value(config, "chunk_size")
        if policy_chunk:
            self.chunk_size = int(policy_chunk)
        policy_steps = self._cfg_value(config, "n_action_steps")
        if policy_steps:
            self.n_action_steps = int(policy_steps)

    @staticmethod
    def _cfg_value(config: Any, key: str, default: Any = None) -> Any:
        if isinstance(config, Mapping):
            return config.get(key, default)
        return getattr(config, key, default)

    @classmethod
    def _feature(cls, config: Any, group: str, name: str) -> Any:
        features = cls._cfg_value(config, group)
        if isinstance(features, Mapping):
            return features.get(name)
        return None

    @staticmethod
    def _feature_dim(feature: Any) -> int | None:
        if feature is None:
            return None
        shape = getattr(feature, "shape", None)
        if shape is None and isinstance(feature, Mapping):
            shape = feature.get("shape")
        if not shape:
            return None
        try:
            return int(shape[0])
        except (TypeError, ValueError, IndexError):
            return None

    @staticmethod
    def _policy_image_keys(config: Any) -> list[str]:
        keys: list[str] = []
        image_features = SmolVLAEngine._cfg_value(config, "image_features")
        if isinstance(image_features, Mapping):
            keys = list(image_features.keys())
        elif image_features:
            keys = list(image_features)

        if not keys:
            input_features = SmolVLAEngine._cfg_value(config, "input_features")
            if isinstance(input_features, Mapping):
                keys = [
                    key
                    for key in input_features
                    if str(key).startswith("observation.images.")
                ]

        cleaned: list[str] = []
        prefix = "observation.images."
        for key in keys:
            text = key if isinstance(key, str) else None
            if text is None:
                for attr in ("key", "name"):
                    candidate = getattr(key, attr, None)
                    if candidate:
                        text = str(candidate)
                        break
            if text is None:
                text = str(key)
            if text.startswith(prefix):
                text = text[len(prefix) :]
            if text and text not in cleaned:
                cleaned.append(text)
        return cleaned

    # ----------------------------------------------------------------- infer
    def infer(
        self,
        instruction: str,
        images: Mapping[str, np.ndarray],
        state: Sequence[float],
        *,
        chunk_size: int | None = None,
    ) -> EngineResult:
        self.ensure_loaded()
        with self._lock:
            if self._policy is None:
                raise ModelUnavailable(f"SmolVLA 模型不可用: {self._error}")
            started = time.perf_counter()
            try:
                batch = self._build_batch(instruction, images, state)
                if self._preprocessor is not None:
                    batch = self._preprocessor(batch)
                    # 某些 LeRobot 版本的 device_processor 覆盖参数不生效；
                    # 这里兜底把张量搬到目标设备。
                    batch = self._move_batch_to_device(batch)
                raw_actions = self._run_policy(batch)
                raw_actions = self._apply_postprocessor(raw_actions)
                actions = self._to_action_chunk(raw_actions, chunk_size)
            except (ModelUnavailable, UnprocessableEntity):
                raise
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("SmolVLA inference failed")
                raise InferenceError(
                    f"SmolVLA 推理失败: {exc}",
                    details={"path": self.path, "device": self._device},
                ) from exc

            latency = (time.perf_counter() - started) * 1000.0
            return EngineResult(
                actions=actions,
                latency_ms=latency,
                model={
                    "backend": self.backend,
                    "path": self.path,
                    "device": self._device,
                    "dtype": self._dtype_name,
                },
            )

    def _build_batch(
        self,
        instruction: str,
        images: Mapping[str, np.ndarray],
        state: Sequence[float],
    ) -> dict[str, Any]:
        torch = self._torch
        if torch is None:
            raise ModelUnavailable("PyTorch 未初始化")

        if len(state) != self.state_dim:
            raise UnprocessableEntity(
                f"state 维度 {len(state)} 与模型要求 {self.state_dim} 不一致",
                code="invalid_state",
            )

        batch: dict[str, Any] = {}
        for key in self.image_keys:
            image = images.get(key)
            if image is None:
                raise UnprocessableEntity(
                    f"缺少模型相机图像: {key}", code="missing_image"
                )
            array = np.asarray(image, dtype=np.float32)
            if array.ndim != 3:
                raise UnprocessableEntity(
                    f"相机 {key} 的图像必须是 CHW，当前 shape={array.shape}",
                    code="invalid_image",
                )
            if array.shape[0] != 3 and array.shape[-1] == 3:
                array = np.transpose(array, (2, 0, 1))
            if array.shape[0] != 3:
                raise UnprocessableEntity(
                    f"相机 {key} 的图像通道数必须为 3，当前 shape={array.shape}",
                    code="invalid_image",
                )
            if array.size and float(array.max()) > 1.0:
                array = array / 255.0
            array = np.ascontiguousarray(array, dtype=np.float32)
            tensor = torch.from_numpy(array).unsqueeze(0).to(dtype=torch.float32)
            batch[self._observation_image_key(key)] = tensor

        batch["observation.state"] = torch.tensor(
            [list(float(item) for item in state)], dtype=torch.float32
        )
        batch["task"] = instruction
        return batch

    def _move_batch_to_device(self, value: Any) -> Any:
        if isinstance(value, Mapping):
            return {key: self._move_batch_to_device(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return type(value)(self._move_batch_to_device(item) for item in value)
        move = getattr(value, "to", None)
        if callable(move):
            try:
                return move(self._device)
            except (TypeError, RuntimeError):
                return value
        return value

    @staticmethod
    def _observation_image_key(key: str) -> str:
        if key.startswith("observation.images."):
            return key
        return f"observation.images.{key}"

    def _run_policy(self, batch: Mapping[str, Any]) -> Any:
        torch = self._torch
        autocast = None
        if self._device.startswith("cuda") and self._dtype_name in {
            "float16",
            "bfloat16",
        }:
            factory = getattr(torch, "autocast", None)
            dtype = getattr(torch, self._dtype_name, None)
            if callable(factory) and dtype is not None:
                try:
                    autocast = factory(device_type="cuda", dtype=dtype)
                except TypeError:
                    autocast = factory("cuda", dtype=dtype)

        with torch.inference_mode():
            if autocast is None:
                return self._call_policy(batch)
            with autocast:
                return self._call_policy(batch)

    def _call_policy(self, batch: Mapping[str, Any]) -> Any:
        torch = self._torch
        predict_chunk = getattr(self._policy, "predict_action_chunk", None)
        if callable(predict_chunk):
            return predict_chunk(batch)

        # 老版本没有 chunk API：先清空策略内部动作队列，避免 HTTP 无状态
        # 请求拿到上一回合残留的动作。
        reset = getattr(self._policy, "reset", None)
        if callable(reset):
            try:
                reset()
            except Exception:  # noqa: BLE001 - 回退路径不应中断推理
                LOGGER.debug("policy.reset() failed in select_action fallback")
        steps = max(1, int(self.n_action_steps))
        outputs = []
        for _ in range(steps):
            outputs.append(self._policy.select_action(batch))
        if len(outputs) == 1:
            result = outputs[0]
            if hasattr(result, "unsqueeze") and getattr(result, "ndim", 0) == 2:
                return result.unsqueeze(1)
            return result
        return torch.stack(outputs, dim=1)

    def _apply_postprocessor(self, actions: Any) -> Any:
        if self._postprocessor is None or not self.cfg.get(
            "apply_postprocessor", True
        ):
            return actions
        try:
            processed = self._postprocessor(actions)
        except (TypeError, KeyError, AttributeError):
            processed = self._postprocessor({"action": actions})
        if isinstance(processed, Mapping):
            if "action" in processed:
                return processed["action"]
            if processed:
                return next(iter(processed.values()))
        return processed

    def _to_action_chunk(
        self, actions: Any, requested_chunk: int | None
    ) -> list[list[float]]:
        if isinstance(actions, Mapping):
            actions = actions.get("action", next(iter(actions.values()), None))
        if actions is None:
            raise InferenceError("SmolVLA 返回空动作")

        if hasattr(actions, "detach"):
            array = actions.detach().float().cpu().numpy()
        else:
            array = np.asarray(actions, dtype=np.float32)

        if array.dtype == object:
            array = array.astype(np.float32)
        if array.ndim == 0:
            array = array.reshape(1, 1)
        elif array.ndim == 1:
            array = array.reshape(1, -1)
        elif array.ndim == 3:
            if array.shape[0] < 1:
                raise InferenceError("SmolVLA 返回空 batch")
            array = array[0]
        elif array.ndim != 2:
            raise InferenceError(f"SmolVLA 返回了不支持的动作 shape: {array.shape}")

        if array.shape[0] < 1 or array.shape[1] < 1:
            raise InferenceError(f"SmolVLA 返回了空动作: {array.shape}")
        if not np.isfinite(array).all():
            raise InferenceError("SmolVLA 动作包含 NaN/Inf")

        limit = int(requested_chunk or self.chunk_size or 1)
        limit = max(1, min(limit, int(array.shape[0])))
        array = array[:limit]
        digits = 6
        return [
            [round(float(value), digits) for value in row]
            for row in array.tolist()
        ]

    # ----------------------------------------------------------------- misc
    def status(self) -> dict[str, Any]:
        return {
            "backend": self.backend,
            "path": self.path,
            "loaded": self._policy is not None,
            "loading": self._loading,
            "device": self._device,
            "dtype": self._dtype_name,
            "action_dim": self.action_dim,
            "state_dim": self.state_dim,
            "chunk_size": self.chunk_size,
            "image_keys": list(self.image_keys),
            "processors": self._preprocessor is not None,
            "error": self._error,
        }

    def reset(self) -> None:
        with self._lock:
            policy = self._policy
            if policy is not None and hasattr(policy, "reset"):
                try:
                    policy.reset()
                except Exception as exc:  # noqa: BLE001
                    LOGGER.warning("policy.reset() 失败: %s", exc)

    def close(self) -> None:
        with self._lock:
            self._policy = None
            self._preprocessor = None
            self._postprocessor = None
            torch = self._torch
            cuda = getattr(torch, "cuda", None) if torch is not None else None
            if cuda is not None and getattr(cuda, "is_available", lambda: False)():
                try:
                    cuda.empty_cache()
                except Exception:  # noqa: BLE001
                    pass
            self._torch = None
            LOGGER.info("SmolVLA engine closed")


def create_engine(model_cfg: Mapping[str, Any]) -> BaseEngine:
    backend = str(model_cfg.get("backend", "smolvla")).lower()
    if backend == "stub":
        return StubEngine(model_cfg)
    if backend == "smolvla":
        return SmolVLAEngine(model_cfg)
    raise ValueError(f"未知 model.backend: {backend}")
