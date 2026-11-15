"""VLA 推理引擎：远程 SmolVLA（Center Server）优先，规则桩兜底。

Center Server 通过启动握手把 ``base_url`` 登记到 Edge Server；本引擎可以：

- ``backend: remote``：只调用 Center ``POST /v1/infer``，失败返回 stop；
- ``backend: auto``（推荐）：优先远程，发现不到/超时/报错时回退规则桩；
- ``backend: stub``：只用规则桩（无网络依赖，联调默认）。

规则桩保留的能力：

- 单步底盘动作：前进/后退/左转/右转/左移/右移/停止；
- 长任务意图：建图、探索、搜索、导航到航点/坐标、回家。

Center 返回的 ``base_cmd`` 是 **第一步** 底盘速度；本引擎按速度幅值映射成
单个 ``ActionIntentDTO``，仍交给上层的 action_generator/safety/velocity_mux
执行，不直接发 ``/cmd_vel``。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

from .center_client import (
    CenterClient,
    CenterClientError,
    base_url_from_registry,
    discover_center,
)
from .data_preprocessor import SyncedDTO
from .instruction_parser import InstructionParser
from ..utils.logger import get_logger

DEFAULT_REMOTE: Dict[str, Any] = {
    "enabled": True,
    # Edge Server 的 CENTER 注册表；留空则只用静态 host/port
    "discovery_url": "http://127.0.0.1/api/center",
    "discovery_interval_sec": 5.0,
    "discovery_timeout_sec": 1.0,
    "host": "",  # 静态回退地址；可写 IP 或完整 http://host:port
    "port": 8000,
    "api_key": "",
    "timeout_sec": 1.5,
    "chunk_size": 1,
    "image_key": "front",
    "color_order": "bgr",
    # state = [x, y, yaw] + 这些关节；空=按 JointState 顺序
    "state_joint_names": [],
    "confidence": 0.9,
    "turn_threshold": 0.15,  # rad/s
    "linear_threshold": 0.02,  # m/s
    "retry_after_sec": 5.0,
    "fallback_to_rules": True,
}

_BACKENDS = {"auto", "remote", "stub"}


@dataclass
class ActionIntentDTO:
    task_id: str = ""
    target: str = "base"
    intent: str = "stop"
    value: float = 0.0
    confidence: float = 0.0
    reverse_request: bool = False
    # --- SLAM/导航/搜索扩展 ---
    task_type: str = "vla"
    object_label: str = ""
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    waypoint: str = ""
    # --- 可观测性：center | rule ---
    source: str = "rule"


class VLAEngine:
    # 关键词 -> (意图, 目标)。按顺序匹配，停止类关键词优先。
    _RULES = (
        (("停止", "停下", "别走", "停", "stop", "halt"), "stop", "base"),
        (("左移", "向左平移", "strafe left", "strafe_left"), "strafe_left", "base"),
        (("右移", "向右平移", "strafe right", "strafe_right"), "strafe_right", "base"),
        (("后退", "向后", "倒车", "backward", "back"), "backward", "base"),
        (("左转", "向左", "left"), "turn_left", "base"),
        (("右转", "向右", "right"), "turn_right", "base"),
        (("前进", "向前", "前走", "forward", "go"), "forward", "base"),
    )

    def __init__(self, cfg):
        self.cfg = cfg or {}
        self.device = str(self.cfg.get("device", "cpu"))
        self.thresh = float(self.cfg.get("confidence_threshold", 0.6))
        self.default_linear = float(self.cfg.get("default_linear_speed", 0.3))
        self.default_angular = float(self.cfg.get("default_angular_speed", 0.5))
        self.parser = InstructionParser(
            self.cfg.get("instruction") or self.cfg.get("instruction_parser") or self.cfg
        )
        self.model = None
        self._logger = get_logger("edge_server.vla")

        backend = str(self.cfg.get("backend", "stub")).strip().lower()
        if backend not in _BACKENDS:
            self._logger.warning(
                "未知 vla_engine.backend=%r，回退 stub（可选 auto/remote/stub）",
                backend,
            )
            backend = "stub"
        self.backend = backend
        self.remote_cfg = dict(DEFAULT_REMOTE)
        remote_overrides = self.cfg.get("remote") or {}
        if isinstance(remote_overrides, Mapping):
            self.remote_cfg.update(dict(remote_overrides))
        self.remote_enabled = backend in {"auto", "remote"} and bool(
            self.remote_cfg.get("enabled", True)
        )
        self._center: Optional[CenterClient] = None
        self._center_at = 0.0
        self._center_base = ""
        self._remote_image_key = str(self.remote_cfg.get("image_key") or "front")
        self._discovery_interval = max(
            1.0, float(self.remote_cfg.get("discovery_interval_sec", 5.0))
        )
        self._remote_retry_at = 0.0
        self._warn_at = 0.0
        self._no_target_logged = 0.0
        self._load_model()
        if self.remote_enabled:
            self._logger.info(
                "VLA remote backend enabled (backend=%s, discovery=%s)",
                backend,
                self.remote_cfg.get("discovery_url") or self.remote_cfg.get("host"),
            )

    def _load_model(self):
        """本地模型加载钩子（规则桩不加载权重）。

        远程推理不在这里加载 torch；Center Server 负责模型生命周期。
        """
        model_name = str(self.cfg.get("model_name", "")).strip()
        if not model_name:
            return
        if self.remote_enabled:
            self._logger.info(
                "VLA model '%s' served by Center Server; edge keeps rule stub as fallback",
                model_name,
            )
            return
        self._logger.info(
            "VLA model '%s' is not bundled; running rule-based stub on %s",
            model_name,
            self.device,
        )

    # ------------------------------------------------------------------ predict
    def predict(self, data: SyncedDTO) -> ActionIntentDTO:
        strict_remote = self.backend == "remote" and not bool(
            self.remote_cfg.get("fallback_to_rules", True)
        )
        if self.remote_enabled:
            now = time.monotonic()
            if now >= self._remote_retry_at:
                intent = self._predict_remote(data, now)
                if intent is not None:
                    return intent
            elif strict_remote:
                return self._stop_intent(getattr(data, "task_id", "") or "")
        if strict_remote:
            return self._stop_intent(getattr(data, "task_id", "") or "")
        return self._predict_rules(data)

    def _predict_rules(self, data: SyncedDTO) -> ActionIntentDTO:
        instruction = (getattr(data, "instruction", "") or "").strip().lower()
        task_id = getattr(data, "task_id", "") or ""

        # 1) 停止/单步动作优先，避免“停止前进”被识别成前进
        for keywords, intent, target in self._RULES:
            if any(keyword in instruction for keyword in keywords):
                value = 0.0
                if intent in ("forward", "backward"):
                    value = self.default_linear
                elif intent in ("turn_left", "turn_right"):
                    value = self.default_angular
                elif intent in ("strafe_left", "strafe_right"):
                    value = self.default_linear
                return ActionIntentDTO(
                    task_id=task_id,
                    target=target,
                    intent=intent,
                    value=value,
                    # 规则桩的置信度固定为 0.9；真实模型应返回 softmax 概率
                    confidence=0.9,
                    source="rule",
                )

        # 2) 长任务意图（建图/搜索/导航/回家）
        mission = self.parser.parse(instruction)
        if mission.is_mission and mission.confidence >= 0.7:
            return ActionIntentDTO(
                task_id=task_id,
                target="mission",
                intent=mission.mission_type,
                value=0.0,
                confidence=mission.confidence,
                task_type=mission.mission_type,
                object_label=mission.target_label,
                goal_x=mission.x,
                goal_y=mission.y,
                goal_yaw=mission.yaw,
                waypoint=mission.waypoint,
                source="rule",
            )

        # 3) 无匹配指令：保持停止并给出低于阈值的置信度，交由上层请求确认
        return ActionIntentDTO(
            task_id=task_id,
            target="base",
            intent="stop",
            value=0.0,
            confidence=0.5,
            source="rule",
        )

    # ------------------------------------------------------------------- remote
    def _predict_remote(
        self, data: SyncedDTO, now: float
    ) -> Optional[ActionIntentDTO]:
        instruction = (getattr(data, "instruction", "") or "").strip()
        if not instruction:
            return None
        client = self._client(now)
        if client is None:
            if now - self._no_target_logged > max(5.0, self._discovery_interval):
                self._no_target_logged = now
                self._logger.info(
                    "未发现 CENTER（等待 Center Server 握手到 Edge Server）；"
                    "使用规则桩降级"
                )
            return None

        task_id = getattr(data, "task_id", "") or ""
        image = getattr(data, "image_raw", None)
        if image is None:
            image = getattr(data, "image", None)
        if image is None:
            self._on_remote_failure(now, "没有可用图像")
            return None

        try:
            response = client.infer(
                instruction,
                {self._remote_image_key: image},
                state=self._state_vector(data),
                task_id=task_id,
                session_id="edge-vla",
                chunk_size=int(self.remote_cfg.get("chunk_size") or 1),
            )
        except CenterClientError as exc:
            self._on_remote_failure(now, str(exc))
            return self._stop_after_failure(task_id)
        except Exception as exc:  # noqa: BLE001 - 远程异常不能拖垮控制循环
            self._on_remote_failure(now, f"{type(exc).__name__}: {exc}")
            return self._stop_after_failure(task_id)

        cmd = response.get("base_cmd")
        if not isinstance(cmd, Mapping):
            self._on_remote_failure(
                now, "Center 响应缺少 base_cmd（action.mode 需为 base_twist/hybrid）"
            )
            return self._stop_after_failure(task_id)

        self._remote_retry_at = 0.0
        return self._intent_from_cmd(cmd, task_id)

    def _client(self, now: float) -> Optional[CenterClient]:
        if self._center is not None and (now - self._center_at) < self._discovery_interval:
            return self._center
        client: Optional[CenterClient] = None
        discovery_url = str(self.remote_cfg.get("discovery_url") or "")
        endpoint = discover_center(
            discovery_url,
            timeout_sec=float(self.remote_cfg.get("discovery_timeout_sec", 1.0)),
        )
        kwargs = {
            "api_key": str(self.remote_cfg.get("api_key") or ""),
            "timeout_sec": float(self.remote_cfg.get("timeout_sec", 1.5)),
            "color_order": str(self.remote_cfg.get("color_order") or "bgr"),
        }
        try:
            if endpoint:
                client = CenterClient.from_registry(endpoint, **kwargs)
                self._remote_image_key = self._pick_image_key(endpoint)
            else:
                host = str(self.remote_cfg.get("host") or "").strip()
                if host:
                    base_url = (
                        host.rstrip("/") if host.startswith("http") else
                        f"http://{host}:{int(self.remote_cfg.get('port') or 8000)}"
                    )
                    client = CenterClient(base_url, **kwargs)
                    self._remote_image_key = str(
                        self.remote_cfg.get("image_key") or "front"
                    )
        except (CenterClientError, ValueError) as exc:
            self._logger.warning(f"解析 CENTER 地址失败: {exc}")
            client = None
        self._center = client
        self._center_at = now
        base_url = client.base_url if client is not None else base_url_from_registry(endpoint or {})
        if client is not None and base_url != self._center_base:
            self._logger.info(
                f"CENTER 推理地址已更新: {base_url} (image_key={self._remote_image_key})"
            )
        self._center_base = base_url
        return client

    def _pick_image_key(self, endpoint: Mapping[str, Any]) -> str:
        """从握手 meta 选择 Center 期望的相机键（含别名映射）。"""

        configured = str(self.remote_cfg.get("image_key") or "front")
        meta = endpoint.get("meta")
        if not isinstance(meta, Mapping):
            return configured
        aliases = meta.get("image_aliases")
        keys = [
            str(item)
            for item in (meta.get("image_keys") or [])
            if str(item).strip()
        ]
        if isinstance(aliases, Mapping):
            target = aliases.get(configured)
            if target and (not keys or str(target) in keys):
                return str(target)
        if configured in keys:
            return configured
        if keys:
            # 配置的相机名不被 checkpoints 接受时，退回第一路，避免整条链路 422
            return keys[0]
        return configured

    def _state_vector(self, data: SyncedDTO) -> list:
        odom = getattr(data, "odom", None) or {}
        state = [
            float(odom.get("x", 0.0) or 0.0),
            float(odom.get("y", 0.0) or 0.0),
            float(odom.get("yaw", 0.0) or 0.0),
        ]
        joint = getattr(data, "joint", None) or {}
        names = self.remote_cfg.get("state_joint_names") or list(joint.keys())
        for name in names:
            value = joint.get(str(name))
            if value is not None:
                state.append(float(value))
        return state

    def _intent_from_cmd(
        self, cmd: Mapping[str, Any], task_id: str
    ) -> ActionIntentDTO:
        linear = cmd.get("linear") if isinstance(cmd.get("linear"), Mapping) else {}
        angular = cmd.get("angular") if isinstance(cmd.get("angular"), Mapping) else {}
        try:
            vx = float(linear.get("x", 0.0) or 0.0)
            vy = float(linear.get("y", 0.0) or 0.0)
            wz = float(angular.get("z", 0.0) or 0.0)
        except (TypeError, ValueError):
            vx = vy = wz = 0.0

        turn_threshold = abs(float(self.remote_cfg.get("turn_threshold", 0.15)))
        linear_threshold = abs(float(self.remote_cfg.get("linear_threshold", 0.02)))
        confidence = float(self.remote_cfg.get("confidence", 0.9))

        intent = "stop"
        value = 0.0
        if abs(wz) >= turn_threshold and abs(wz) >= abs(vx):
            intent = "turn_left" if wz > 0.0 else "turn_right"
            value = abs(wz)
        elif abs(vx) >= linear_threshold:
            intent = "forward" if vx > 0.0 else "backward"
            value = abs(vx)
        elif abs(vy) >= linear_threshold:
            intent = "strafe_left" if vy > 0.0 else "strafe_right"
            value = abs(vy)

        return ActionIntentDTO(
            task_id=task_id,
            target="base",
            intent=intent,
            value=float(value),
            confidence=confidence,
            source="center",
        )

    def _on_remote_failure(self, now: float, message: str) -> None:
        self._remote_retry_at = now + max(
            1.0, float(self.remote_cfg.get("retry_after_sec", 5.0))
        )
        if now - self._warn_at > 2.0:
            self._warn_at = now
            self._logger.warning(
                f"CENTER 推理失败，{self._remote_retry_at - now:.1f}s 内降级规则桩: {message}"
            )

    def _stop_after_failure(self, task_id: str) -> Optional[ActionIntentDTO]:
        if self.backend == "remote" and not bool(
            self.remote_cfg.get("fallback_to_rules", True)
        ):
            return self._stop_intent(task_id)
        return None

    @staticmethod
    def _stop_intent(task_id: str) -> ActionIntentDTO:
        return ActionIntentDTO(
            task_id=task_id,
            intent="stop",
            value=0.0,
            confidence=0.0,
            source="center",
        )

    # ------------------------------------------------------------- observability
    def center_status(self) -> Dict[str, Any]:
        now = time.monotonic()
        return {
            "backend": self.backend,
            "remote_enabled": bool(self.remote_enabled),
            "discovery_url": str(self.remote_cfg.get("discovery_url") or ""),
            "base_url": self._center.base_url if self._center is not None else "",
            "degraded_until_ms": int(max(0.0, self._remote_retry_at - now) * 1000.0),
        }

    def unload(self):
        self.model = None
        self._center = None
        self._logger.info("VLA engine unloaded")
