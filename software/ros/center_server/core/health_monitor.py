"""设备健康检查核心逻辑（不依赖 ROS，便于单元测试）。

`health_monitor_node` 负责 ROS 订阅、节点发现和 HTTP 网关；本模块只做：

1. 设备注册（配置 -> DeviceSpec）；
2. 每帧消息的时间戳/频率/指标记录；
3. 按超时、频率、数据合法性、启动宽限期计算设备与整体状态；
4. 生成可 JSON 序列化的健康快照。

状态语义见 `software/ros/HEALTHCHECK.md`。
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Deque, Dict, List, Mapping, Optional, Sequence, Tuple

SERVICE_NAME = "rush-mog-ros-health"
SERVICE_VERSION = "0.2.0"

STATUS_OK = "ok"
STATUS_DEGRADED = "degraded"
STATUS_ERROR = "error"
STATUS_UNKNOWN = "unknown"
STATUS_DISABLED = "disabled"
STATUS_STARTING = "starting"

_HEALTHY_STATUSES = {STATUS_OK}
_READY_STATUSES = {STATUS_OK, STATUS_DEGRADED}

_DEFAULT_DEVICE: Dict[str, Any] = {
    "type": "other",
    "topic": "",
    "message_type": "",
    "expected_rate_hz": 0.0,
    "timeout_sec": 2.0,
    "required": True,
    "enabled": True,
    "min_rate_ratio": 0.5,
    "joint_names": (),
    "description": "",
    "qos": "sensor",
}


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _to_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    if value is None:
        return default
    return bool(value)


def _utc_now_iso() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class DeviceSpec:
    """一个受监控设备的静态配置。"""

    id: str
    kind: str = "other"
    topic: str = ""
    message_type: str = ""
    expected_rate_hz: float = 0.0
    timeout_sec: float = 2.0
    required: bool = True
    enabled: bool = True
    min_rate_ratio: float = 0.5
    joint_names: Tuple[str, ...] = ()
    description: str = ""
    qos: str = "sensor"

    @classmethod
    def from_config(
        cls,
        device_id: str,
        config: Optional[Mapping[str, Any]],
        defaults: Optional[Mapping[str, Any]] = None,
    ) -> "DeviceSpec":
        cfg: Dict[str, Any] = dict(_DEFAULT_DEVICE)
        if defaults:
            cfg.update(defaults)
        if config:
            cfg.update(config)

        joint_names = cfg.get("joint_names") or ()
        if isinstance(joint_names, str):
            joint_names = tuple(
                name.strip() for name in joint_names.split(",") if name.strip()
            )
        else:
            joint_names = tuple(str(name) for name in joint_names)

        return cls(
            id=str(device_id),
            kind=str(cfg.get("type") or "other"),
            topic=str(cfg.get("topic") or ""),
            message_type=str(cfg.get("message_type") or ""),
            expected_rate_hz=max(0.0, _to_float(cfg.get("expected_rate_hz"), 0.0)),
            timeout_sec=max(0.0, _to_float(cfg.get("timeout_sec"), 2.0)),
            required=_to_bool(cfg.get("required"), True),
            enabled=_to_bool(cfg.get("enabled"), True),
            min_rate_ratio=_clamp(
                _to_float(cfg.get("min_rate_ratio"), 0.5), 0.0, 1.0
            ),
            joint_names=joint_names,
            description=str(cfg.get("description") or ""),
            qos=str(cfg.get("qos") or "sensor"),
        )


@dataclass
class DeviceState:
    """一个设备的运行时观测值。"""

    spec: DeviceSpec
    first_seen: Optional[float] = None
    last_seen: Optional[float] = None
    count: int = 0
    timestamps: Deque[float] = field(
        default_factory=lambda: deque(maxlen=1024)
    )
    publisher_count: int = 0
    metrics: Dict[str, Any] = field(default_factory=dict)
    invalid: bool = False
    error: str = ""
    config_error: str = ""


@dataclass
class DeviceReport:
    id: str
    type: str
    status: str
    healthy: bool
    ready: bool
    enabled: bool
    required: bool
    connected: bool
    data_fresh: bool
    topic: str
    message_type: str
    publisher_count: int
    age_sec: float
    rate_hz: float
    expected_rate_hz: float
    detail: str
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self, include_metrics: bool = True) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "id": self.id,
            "type": self.type,
            "status": self.status,
            "healthy": self.healthy,
            "ready": self.ready,
            "enabled": self.enabled,
            "required": self.required,
            "connected": self.connected,
            "data_fresh": self.data_fresh,
            "topic": self.topic,
            "message_type": self.message_type,
            "publisher_count": self.publisher_count,
            "age_sec": self.age_sec,
            "rate_hz": self.rate_hz,
            "expected_rate_hz": self.expected_rate_hz,
            "detail": self.detail,
        }
        if include_metrics:
            payload["metrics"] = dict(self.metrics)
        return payload


@dataclass
class HealthReport:
    status: str
    healthy: bool
    ready: bool
    reason: str
    uptime_sec: float
    timestamp: str
    service: str
    version: str
    devices: List[DeviceReport]
    summary: Dict[str, int]
    ros: Dict[str, Any]
    safety: Dict[str, Any]
    http: Dict[str, Any]

    def to_dict(self, include_metrics: bool = True) -> Dict[str, Any]:
        return {
            "ok": True,
            "status": self.status,
            "healthy": self.healthy,
            "ready": self.ready,
            "service": self.service,
            "version": self.version,
            "uptime_sec": self.uptime_sec,
            "timestamp": self.timestamp,
            "reason": self.reason,
            "summary": dict(self.summary),
            "emergency_stop": bool(self.safety.get("emergency_stop", False)),
            "safety": dict(self.safety),
            "ros": dict(self.ros),
            "http": dict(self.http),
            "devices": [
                device.to_dict(include_metrics=include_metrics)
                for device in self.devices
            ],
        }


class HealthMonitor:
    """把消息时间戳转换成设备/系统健康状态。"""

    def __init__(
        self,
        devices: Optional[Mapping[str, Mapping[str, Any]]] = None,
        *,
        defaults: Optional[Mapping[str, Any]] = None,
        enabled: bool = True,
        startup_grace_sec: float = 5.0,
        rate_window_sec: float = 5.0,
        min_rate_ratio: float = 0.5,
        strict_unknown: bool = False,
        safety_required: bool = True,
        started_at: Optional[float] = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.startup_grace_sec = max(0.0, float(startup_grace_sec))
        self.rate_window_sec = max(0.5, float(rate_window_sec))
        self.min_rate_ratio = _clamp(float(min_rate_ratio), 0.0, 1.0)
        self.strict_unknown = bool(strict_unknown)
        self.safety_required = bool(safety_required)
        self.started_at = time.monotonic() if started_at is None else float(started_at)

        device_defaults: Dict[str, Any] = dict(defaults or {})
        device_defaults.setdefault("min_rate_ratio", self.min_rate_ratio)

        self.devices: Dict[str, DeviceSpec] = {}
        self.states: Dict[str, DeviceState] = {}
        for device_id, raw_cfg in (devices or {}).items():
            spec = DeviceSpec.from_config(device_id, raw_cfg, device_defaults)
            self.devices[spec.id] = spec
            self.states[spec.id] = DeviceState(spec=spec)

        self._ros: Dict[str, Any] = {}
        self._safety: Dict[str, Any] = {}
        self._http: Dict[str, Any] = {}

    # ------------------------------------------------------------- observations
    def on_message(
        self,
        device_id: str,
        now: Optional[float] = None,
        *,
        metrics: Optional[Mapping[str, Any]] = None,
        invalid: bool = False,
        error: str = "",
    ) -> None:
        state = self.states.get(str(device_id))
        if state is None or not state.spec.enabled:
            return
        stamp = time.monotonic() if now is None else float(now)
        if state.first_seen is None:
            state.first_seen = stamp
        state.last_seen = stamp
        state.count += 1
        state.timestamps.append(stamp)
        state.metrics = dict(metrics or {})
        state.invalid = bool(invalid)
        state.error = str(error or "")

    def set_publisher_count(self, device_id: str, count: int) -> None:
        state = self.states.get(str(device_id))
        if state is not None:
            state.publisher_count = int(count)

    def set_config_error(self, device_id: str, message: str) -> None:
        state = self.states.get(str(device_id))
        if state is not None:
            state.config_error = str(message)

    def set_ros_info(self, info: Optional[Mapping[str, Any]]) -> None:
        self._ros = dict(info or {})

    def set_safety(self, safety: Optional[Mapping[str, Any]]) -> None:
        self._safety = dict(safety or {})

    def set_http_info(self, info: Optional[Mapping[str, Any]]) -> None:
        self._http = dict(info or {})

    def clear(self) -> None:
        for state in self.states.values():
            state.first_seen = None
            state.last_seen = None
            state.count = 0
            state.timestamps.clear()
            state.publisher_count = 0
            state.metrics.clear()
            state.invalid = False
            state.error = ""
            state.config_error = ""

    # ------------------------------------------------------------------ reports
    def snapshot(
        self,
        now: Optional[float] = None,
        *,
        device_ids: Optional[Sequence[str]] = None,
        ros_info: Optional[Mapping[str, Any]] = None,
        safety: Optional[Mapping[str, Any]] = None,
        http_info: Optional[Mapping[str, Any]] = None,
    ) -> HealthReport:
        stamp = time.monotonic() if now is None else float(now)
        if ros_info is not None:
            self.set_ros_info(ros_info)
        if safety is not None:
            self.set_safety(safety)
        if http_info is not None:
            self.set_http_info(http_info)

        if not self.enabled:
            return HealthReport(
                status=STATUS_DISABLED,
                healthy=False,
                ready=False,
                reason="health monitoring disabled",
                uptime_sec=round(stamp - self.started_at, 3),
                timestamp=_utc_now_iso(),
                service=SERVICE_NAME,
                version=SERVICE_VERSION,
                devices=[],
                summary={
                    "total": 0,
                    "enabled": 0,
                    "ok": 0,
                    "degraded": 0,
                    "error": 0,
                    "unknown": 0,
                    "disabled": 0,
                },
                ros=dict(self._ros),
                safety=dict(self._safety),
                http=dict(self._http),
            )

        if device_ids:
            unknown_ids = [str(i) for i in device_ids if str(i) not in self.states]
            if unknown_ids:
                raise KeyError(",".join(unknown_ids))
            wanted = {str(i) for i in device_ids}
        else:
            wanted = None

        safety_info = dict(self._safety)
        missing_nodes = list(self._ros.get("missing_nodes") or [])
        in_grace = (stamp - self.started_at) < self.startup_grace_sec
        missing_required = any(
            state.spec.enabled and state.spec.required and state.last_seen is None
            for state in self.states.values()
        )
        safety_missing = bool(self.safety_required and not safety_info.get("available", False))
        starting = bool(in_grace and (missing_required or missing_nodes or safety_missing))

        # 整体状态始终基于全部设备；device_ids 只裁剪返回的设备列表。
        all_reports = [
            self._device_report(state, stamp, starting=starting)
            for state in self.states.values()
        ]
        reports = (
            [report for report in all_reports if report.id in wanted]
            if wanted is not None
            else all_reports
        )

        summary = {
            "total": len(reports),
            "enabled": sum(1 for r in reports if r.enabled),
            "ok": sum(1 for r in reports if r.status == STATUS_OK),
            "degraded": sum(1 for r in reports if r.status == STATUS_DEGRADED),
            "error": sum(1 for r in reports if r.status == STATUS_ERROR),
            "unknown": sum(1 for r in reports if r.status == STATUS_UNKNOWN),
            "disabled": sum(1 for r in reports if r.status == STATUS_DISABLED),
        }

        emergency_stop = bool(safety_info.get("emergency_stop", False))
        safety_reason = str(safety_info.get("reason") or "")

        status = self._overall_status(
            all_reports,
            starting=starting,
            in_grace=in_grace,
            missing_nodes=missing_nodes,
            safety_missing=safety_missing,
            emergency_stop=emergency_stop,
        )
        ready = status in _READY_STATUSES and not emergency_stop

        reasons: List[str] = []
        if status == STATUS_STARTING:
            reasons.append("starting")
        if missing_nodes and not in_grace:
            reasons.append("missing nodes: " + ",".join(missing_nodes))
        if safety_missing and not in_grace:
            reasons.append("safety state unavailable")
        for report in all_reports:
            if report.status in {STATUS_ERROR, STATUS_DEGRADED}:
                reasons.append(f"{report.id}: {report.detail}")
            elif report.status == STATUS_UNKNOWN and self.strict_unknown:
                reasons.append(f"{report.id}: {report.detail}")
        if emergency_stop:
            reasons.append(f"emergency_stop: {safety_reason or 'engaged'}")
        reason = "; ".join(reasons)[:512] if reasons else STATUS_OK

        return HealthReport(
            status=status,
            healthy=status in _HEALTHY_STATUSES,
            ready=ready,
            reason=reason,
            uptime_sec=round(stamp - self.started_at, 3),
            timestamp=_utc_now_iso(),
            service=SERVICE_NAME,
            version=SERVICE_VERSION,
            devices=reports,
            summary=summary,
            ros=dict(self._ros),
            safety=safety_info,
            http=dict(self._http),
        )

    # ------------------------------------------------------------------ helpers
    def _rate_hz(self, state: DeviceState, now: float) -> float:
        cutoff = now - self.rate_window_sec
        timestamps = state.timestamps
        while timestamps and timestamps[0] < cutoff:
            timestamps.popleft()
        if len(timestamps) < 2:
            return -1.0
        span = timestamps[-1] - timestamps[0]
        if span <= 1e-6:
            return -1.0
        return (len(timestamps) - 1) / span

    def _device_report(
        self,
        state: DeviceState,
        now: float,
        *,
        starting: bool,
    ) -> DeviceReport:
        spec = state.spec
        rate = self._rate_hz(state, now)
        age = -1.0 if state.last_seen is None else max(0.0, now - state.last_seen)
        data_fresh = age >= 0.0 and (spec.timeout_sec <= 0.0 or age <= spec.timeout_sec)
        connected = state.publisher_count > 0 or state.last_seen is not None

        if not spec.enabled:
            status = STATUS_DISABLED
            detail = "disabled"
        elif state.config_error:
            status = STATUS_ERROR
            detail = state.config_error
        elif state.last_seen is None:
            if spec.required and not starting:
                status = STATUS_ERROR
                detail = "no data received"
            else:
                status = STATUS_UNKNOWN
                detail = (
                    "waiting for first message"
                    if starting
                    else (spec.description or "optional device not present")
                )
        elif spec.timeout_sec > 0.0 and age > spec.timeout_sec:
            status = STATUS_ERROR if spec.required else STATUS_DEGRADED
            detail = f"stale {age:.1f}s > {spec.timeout_sec:.1f}s"
        elif state.invalid:
            status = STATUS_ERROR if spec.required else STATUS_DEGRADED
            detail = state.error or "invalid data"
        elif (
            spec.expected_rate_hz > 0.0
            and rate >= 0.0
            and rate < spec.expected_rate_hz * spec.min_rate_ratio
        ):
            status = STATUS_DEGRADED
            detail = (
                f"low rate {rate:.1f}Hz < "
                f"{spec.expected_rate_hz * spec.min_rate_ratio:.1f}Hz"
            )
        else:
            status = STATUS_OK
            detail = "ok"

        return DeviceReport(
            id=spec.id,
            type=spec.kind,
            status=status,
            healthy=status in _HEALTHY_STATUSES,
            ready=status in _READY_STATUSES,
            enabled=spec.enabled,
            required=spec.required,
            connected=connected,
            data_fresh=data_fresh,
            topic=spec.topic,
            message_type=spec.message_type,
            publisher_count=max(0, state.publisher_count),
            age_sec=round(age, 3),
            rate_hz=round(rate, 2),
            expected_rate_hz=round(spec.expected_rate_hz, 2),
            detail=detail,
            metrics=dict(state.metrics),
        )

    def _overall_status(
        self,
        reports: Sequence[DeviceReport],
        *,
        starting: bool,
        in_grace: bool,
        missing_nodes: Sequence[str],
        safety_missing: bool,
        emergency_stop: bool,
    ) -> str:
        required_error = any(
            report.required and report.status == STATUS_ERROR for report in reports
        )
        optional_error = any(
            not report.required and report.status == STATUS_ERROR for report in reports
        )
        degraded = any(report.status == STATUS_DEGRADED for report in reports)
        unknown = any(report.status == STATUS_UNKNOWN for report in reports)

        if required_error or (missing_nodes and not in_grace) or (
            safety_missing and not in_grace
        ):
            return STATUS_ERROR
        if starting:
            return STATUS_STARTING
        if degraded or optional_error:
            return STATUS_DEGRADED
        if emergency_stop:
            return STATUS_DEGRADED
        if self.strict_unknown and unknown:
            return STATUS_DEGRADED
        return STATUS_OK
