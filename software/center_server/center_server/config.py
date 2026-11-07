"""YAML 配置加载与校验。

配置优先级：
1. 命令行 ``--config`` 指定的文件；
2. 环境变量 ``CENTER_SERVER_CONFIG``；
3. 内置默认值。

少量环境变量可覆盖关键字段（容器部署更方便），见 ``_apply_env_overrides``。
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Mapping

import yaml

DEFAULT_CONFIG: dict[str, Any] = {
    "server": {
        "host": "0.0.0.0",
        "port": 8000,
        "api_key": "",  # 非空时，写接口要求 Authorization: Bearer <key>
        "max_body_bytes": 16 * 1024 * 1024,
        "max_image_bytes": 6 * 1024 * 1024,
        "cors_allow_origin": "*",
        "request_timeout_sec": 30.0,
    },
    "model": {
        "backend": "smolvla",  # smolvla | stub
        "path": "lerobot/smolvla_base",
        "device": "auto",  # auto | cpu | cuda | cuda:0 | mps
        "dtype": "auto",  # auto | float32 | float16 | bfloat16
        "local_files_only": False,
        "trust_remote_code": False,
        "require_processors": True,
        "apply_postprocessor": True,
        "preload": False,
        "reload_interval_sec": 10.0,
        "image_keys": ["camera1", "camera2", "camera3"],
        # 请求里的相机名 -> 模型相机名，例如 {front: camera1, wrist: camera2}
        "image_aliases": {},
        "missing_image_policy": "copy_first",  # copy_first | zeros | error
        "require_images": True,
        "require_state": False,
        "state_dim": 6,
        "pad_state": True,
        "chunk_size": 50,
        "n_action_steps": 1,
        "stub_action": [0.0, 0.0, 0.0],
    },
    "action": {
        "mode": "base_twist",  # raw | base_twist | hybrid | joint
        # 动作向量中 [vx, vy, wz] 的下标
        "base_indices": [0, 1, 2],
        # True：模型输出是 [-1, 1] 归一化速度，按 max_* 缩放；
        # False：模型输出已经是 m/s、rad/s，直接限幅。
        "normalized": True,
        "max_linear_x": 0.5,
        "max_linear_y": 0.4,
        "max_angular_z": 1.0,
        "deadband": 0.0,
        "invert": [False, False, False],
        # hybrid 模式下机械臂动作从动作向量的第几位开始
        "arm_start": 3,
        "joint_names": ["j1", "j2", "j3", "j4", "j5"],
        "joint_limits": {},
        "round_digits": 6,
    },
    "logging": {"level": "INFO"},
}


class ConfigError(ValueError):
    """配置文件不合法。"""


def default_config_path() -> Path:
    """仓库内默认配置路径：``software/center_server/config.yaml``。"""

    return Path(__file__).resolve().parent.parent / "config.yaml"


def _deep_merge(base: dict, override: Mapping) -> dict:
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, Mapping)
        ):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _as_int(value: Any, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} 必须是整数，当前为 {value!r}") from exc


def _as_float(value: Any, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} 必须是数字，当前为 {value!r}") from exc


def _as_bool(value: Any, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    raise ConfigError(f"{name} 必须是布尔值，当前为 {value!r}")


def _apply_env_overrides(cfg: dict) -> None:
    env = os.environ
    server = cfg["server"]
    model = cfg["model"]

    if "CENTER_SERVER_HOST" in env:
        server["host"] = env["CENTER_SERVER_HOST"]
    if "CENTER_SERVER_PORT" in env:
        server["port"] = env["CENTER_SERVER_PORT"]
    if "CENTER_SERVER_API_KEY" in env:
        server["api_key"] = env["CENTER_SERVER_API_KEY"]
    if "CENTER_SERVER_BACKEND" in env:
        model["backend"] = env["CENTER_SERVER_BACKEND"]
    if "CENTER_SERVER_MODEL_PATH" in env:
        model["path"] = env["CENTER_SERVER_MODEL_PATH"]
    if "CENTER_SERVER_DEVICE" in env:
        model["device"] = env["CENTER_SERVER_DEVICE"]
    if "CENTER_SERVER_DTYPE" in env:
        model["dtype"] = env["CENTER_SERVER_DTYPE"]


def _validate(cfg: dict) -> None:
    for section in ("server", "model", "action", "logging"):
        if not isinstance(cfg.get(section), dict):
            raise ConfigError(f"{section} 必须是映射")

    server = cfg["server"]
    model = cfg["model"]
    action = cfg["action"]

    host = str(server.get("host", "")).strip()
    if not host:
        raise ConfigError("server.host 不能为空")
    server["host"] = host

    port = _as_int(server.get("port", 8000), "server.port")
    if not 1 <= port <= 65535:
        raise ConfigError(f"server.port 超出范围: {port}")
    server["port"] = port

    server["api_key"] = str(server.get("api_key") or "")
    max_body = _as_int(server.get("max_body_bytes", 0), "server.max_body_bytes")
    if max_body < 1024:
        raise ConfigError("server.max_body_bytes 至少为 1024")
    server["max_body_bytes"] = max_body

    max_image = _as_int(
        server.get("max_image_bytes", 6 * 1024 * 1024), "server.max_image_bytes"
    )
    if max_image < 1024:
        raise ConfigError("server.max_image_bytes 至少为 1024")
    if max_image > max_body:
        raise ConfigError("server.max_image_bytes 不能大于 server.max_body_bytes")
    server["max_image_bytes"] = max_image
    server["cors_allow_origin"] = str(server.get("cors_allow_origin") or "*")
    timeout = _as_float(
        server.get("request_timeout_sec", 30.0), "server.request_timeout_sec"
    )
    if timeout <= 0:
        raise ConfigError("server.request_timeout_sec 必须大于 0")
    server["request_timeout_sec"] = timeout

    backend = str(model.get("backend", "smolvla")).strip().lower()
    if backend not in {"smolvla", "stub"}:
        raise ConfigError("model.backend 只能是 smolvla 或 stub")
    model["backend"] = backend

    path = str(model.get("path", "")).strip()
    if backend == "smolvla" and not path:
        raise ConfigError("model.path 不能为空（smolvla 后端需要权重路径或 HF 仓库名）")
    model["path"] = path
    model["device"] = str(model.get("device", "auto")).strip().lower() or "auto"
    model["dtype"] = str(model.get("dtype", "auto")).strip().lower() or "auto"
    if model["dtype"] not in {"auto", "float32", "fp32", "float16", "fp16", "bfloat16", "bf16"}:
        raise ConfigError("model.dtype 只能是 auto/float32/float16/bfloat16")

    for key in (
        "local_files_only",
        "trust_remote_code",
        "require_processors",
        "apply_postprocessor",
        "preload",
        "require_images",
        "require_state",
        "pad_state",
    ):
        model[key] = _as_bool(model.get(key), f"model.{key}")

    reload_interval = _as_float(
        model.get("reload_interval_sec", 10.0), "model.reload_interval_sec"
    )
    if reload_interval < 0:
        raise ConfigError("model.reload_interval_sec 不能为负数")
    model["reload_interval_sec"] = reload_interval

    image_keys = model.get("image_keys")
    if not isinstance(image_keys, (list, tuple)) or not all(
        isinstance(item, str) and item.strip() for item in image_keys
    ):
        raise ConfigError("model.image_keys 必须是字符串列表")
    model["image_keys"] = [item.strip() for item in image_keys]

    aliases = model.get("image_aliases") or {}
    if not isinstance(aliases, Mapping):
        raise ConfigError("model.image_aliases 必须是映射（请求相机名 -> 模型相机名）")
    model["image_aliases"] = {
        str(key): str(value) for key, value in aliases.items()
    }

    missing_policy = str(model.get("missing_image_policy", "copy_first")).lower()
    if missing_policy not in {"copy_first", "zeros", "error"}:
        raise ConfigError("model.missing_image_policy 只能是 copy_first/zeros/error")
    model["missing_image_policy"] = missing_policy

    state_dim = _as_int(model.get("state_dim", 0), "model.state_dim")
    if state_dim < 0:
        raise ConfigError("model.state_dim 不能为负数")
    model["state_dim"] = state_dim

    chunk_size = _as_int(model.get("chunk_size", 1), "model.chunk_size")
    if chunk_size < 1:
        raise ConfigError("model.chunk_size 必须大于 0")
    model["chunk_size"] = chunk_size

    n_action_steps = _as_int(
        model.get("n_action_steps", 1), "model.n_action_steps"
    )
    if n_action_steps < 1:
        raise ConfigError("model.n_action_steps 必须大于 0")
    model["n_action_steps"] = n_action_steps

    stub_action = model.get("stub_action") or []
    if not isinstance(stub_action, (list, tuple)):
        raise ConfigError("model.stub_action 必须是数字列表")
    try:
        model["stub_action"] = [float(item) for item in stub_action]
    except (TypeError, ValueError) as exc:
        raise ConfigError("model.stub_action 必须是数字列表") from exc

    mode = str(action.get("mode", "base_twist")).strip().lower()
    if mode not in {"raw", "base_twist", "hybrid", "joint"}:
        raise ConfigError("action.mode 只能是 raw/base_twist/hybrid/joint")
    action["mode"] = mode

    base_indices = action.get("base_indices") or [0, 1, 2]
    if not isinstance(base_indices, (list, tuple)) or len(base_indices) != 3:
        raise ConfigError("action.base_indices 必须是 3 个整数的列表")
    action["base_indices"] = [
        _as_int(item, "action.base_indices") for item in base_indices
    ]
    if any(item < 0 for item in action["base_indices"]):
        raise ConfigError("action.base_indices 不能为负数")

    for key in ("max_linear_x", "max_linear_y", "max_angular_z", "deadband"):
        value = _as_float(action.get(key, 0.0), f"action.{key}")
        if value < 0:
            raise ConfigError(f"action.{key} 不能为负数")
        action[key] = value

    action["normalized"] = _as_bool(
        action.get("normalized", True), "action.normalized"
    )

    invert = action.get("invert") or [False, False, False]
    if not isinstance(invert, (list, tuple)) or len(invert) != 3:
        raise ConfigError("action.invert 必须是 3 个布尔值")
    action["invert"] = [
        _as_bool(item, "action.invert") for item in invert
    ]

    arm_start = _as_int(action.get("arm_start", 3), "action.arm_start")
    if arm_start < 0:
        raise ConfigError("action.arm_start 不能为负数")
    action["arm_start"] = arm_start

    joint_names = action.get("joint_names") or []
    if not isinstance(joint_names, (list, tuple)):
        raise ConfigError("action.joint_names 必须是字符串列表")
    action["joint_names"] = [str(item) for item in joint_names]

    limits = action.get("joint_limits") or {}
    if not isinstance(limits, Mapping):
        raise ConfigError("action.joint_limits 必须是映射")
    parsed_limits: dict[str, list[float]] = {}
    for name, pair in limits.items():
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ConfigError(f"action.joint_limits.{name} 必须是 [lower, upper]")
        lower = _as_float(pair[0], f"action.joint_limits.{name}")
        upper = _as_float(pair[1], f"action.joint_limits.{name}")
        if lower > upper:
            raise ConfigError(f"action.joint_limits.{name} 下限大于上限")
        parsed_limits[str(name)] = [lower, upper]
    action["joint_limits"] = parsed_limits

    digits = _as_int(action.get("round_digits", 6), "action.round_digits")
    if not 0 <= digits <= 12:
        raise ConfigError("action.round_digits 必须在 0..12")
    action["round_digits"] = digits

    logging_cfg = cfg["logging"]
    level = str(logging_cfg.get("level", "INFO")).upper()
    if level not in {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}:
        raise ConfigError("logging.level 不合法")
    logging_cfg["level"] = level


def validate_config(cfg: dict) -> dict:
    """校验并规范化已合并的配置（CLI 覆盖后再次调用）。"""

    _validate(cfg)
    return cfg


def load_config(path: str | os.PathLike | None = None) -> dict:
    """加载配置。

    ``path`` 为空时先读 ``CENTER_SERVER_CONFIG``；仍为空则只使用默认值
    （不会隐式加载仓库 config.yaml，便于测试和容器部署）。
    """

    selected = path or os.environ.get("CENTER_SERVER_CONFIG")
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if selected:
        config_path = Path(selected).expanduser()
        if not config_path.is_absolute():
            config_path = Path.cwd() / config_path
        if not config_path.exists():
            raise ConfigError(f"配置文件不存在: {config_path}")
        try:
            with config_path.open("r", encoding="utf-8") as handle:
                loaded = yaml.safe_load(handle) or {}
        except OSError as exc:
            raise ConfigError(f"无法读取配置文件 {config_path}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ConfigError(f"配置文件 YAML 解析失败: {exc}") from exc
        if not isinstance(loaded, Mapping):
            raise ConfigError("配置文件顶层必须是映射")
        # 兼容 ROS 风格：center_server: {...}
        if "center_server" in loaded and isinstance(loaded["center_server"], Mapping):
            loaded = loaded["center_server"]
        cfg = _deep_merge(cfg, loaded)
        cfg["_config_path"] = str(config_path)

    _apply_env_overrides(cfg)
    _validate(cfg)
    return cfg
