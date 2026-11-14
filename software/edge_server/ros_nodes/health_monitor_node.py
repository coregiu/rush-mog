"""health_monitor_node：ROS 系统与全部连接设备的健康检查。

接口：
    发布  /center/health          custom_msgs/HealthStatus（TRANSIENT_LOCAL，1 Hz）
    服务  /center/healthcheck     custom_msgs/HealthCheck
    HTTP  /ros/health 等         供同机 Edge Server/dashboard 等非 ROS 调用方使用
                                 契约见 software/edge_server/HEALTHCHECK.md

监控范围：
    - 传感器/执行器：相机、雷达、IMU、麦克风、音箱、机械臂、底盘、电池、里程计；
    - ROS 运行状态：期望节点是否在线、话题发布者数量；
    - 安全状态：/center/safety_state 的急停与原因。

所有设备都来自 `health.devices` 配置；新增设备只需加配置和反馈话题，无需改代码。
"""

from __future__ import annotations

import copy
import json
import math
import numbers
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, Image, Imu, JointState, LaserScan
from std_msgs.msg import String

from edge_server.core.health_monitor import (
    DeviceSpec,
    HealthMonitor,
    HealthReport,
    SERVICE_NAME,
    SERVICE_VERSION,
)
from edge_server.http_health import HealthHttpServer
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config
from custom_msgs.msg import AudioData, DeviceHealth, HealthStatus, SafetyState
from custom_msgs.srv import HealthCheck

try:  # 允许配置里使用任意已安装的 ROS 消息类型
    from rosidl_runtime_py.utilities import get_message as _get_message
except Exception:  # noqa: BLE001
    _get_message = None

_KNOWN_MSG_TYPES = {
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/LaserScan": LaserScan,
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/JointState": JointState,
    "sensor_msgs/msg/BatteryState": BatteryState,
    "nav_msgs/msg/Odometry": Odometry,
    "std_msgs/msg/String": String,
    "custom_msgs/msg/AudioData": AudioData,
}

_DEFAULT_EXPECTED_NODES = (
    "bridge_node",
    "vla_inference_node",
    "action_generator_node",
    "reverse_operation_node",
    "safety_manager_node",
    "task_manager_node",
    "health_monitor_node",
)


class HealthMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("health_monitor_node")
        self.cfg = load_config(self)
        health_cfg = dict(self.cfg.get("health") or {})
        self.health_cfg = health_cfg
        self.enabled = bool(health_cfg.get("enabled", True))

        self.publish_rate = max(0.2, float(health_cfg.get("publish_rate", 1.0)))
        self.startup_grace = max(0.0, float(health_cfg.get("startup_grace_sec", 5.0)))
        self.rate_window = max(0.5, float(health_cfg.get("rate_window_sec", 5.0)))
        self.min_rate_ratio = float(health_cfg.get("min_rate_ratio", 0.5))
        self.strict_unknown = bool(health_cfg.get("strict_unknown", False))
        self.safety_required = bool(health_cfg.get("safety_required", True))
        self.safety_timeout = max(0.5, float(health_cfg.get("safety_timeout_sec", 3.0)))
        self.battery_low_percent = float(
            (self.cfg.get("safety") or {}).get("battery_low_percent", 15.0)
        )

        self.topic_name = str(health_cfg.get("topic", "/center/health"))
        self.service_name = str(health_cfg.get("service", "/center/healthcheck"))
        self._check_nodes = bool(health_cfg.get("check_nodes", True))
        expected_nodes = health_cfg.get("expected_nodes")
        if expected_nodes is None:
            expected_nodes = _DEFAULT_EXPECTED_NODES
        self._expected_nodes = [str(name) for name in expected_nodes]

        self.monitor = HealthMonitor(
            health_cfg.get("devices") or {},
            defaults=health_cfg.get("device_defaults") or {},
            enabled=self.enabled,
            startup_grace_sec=self.startup_grace,
            rate_window_sec=self.rate_window,
            min_rate_ratio=self.min_rate_ratio,
            strict_unknown=self.strict_unknown,
            safety_required=self.safety_required,
        )

        self._topic_entries: Dict[str, List[Tuple[str, DeviceSpec]]] = {}
        self._subscriptions: List[Any] = []
        self._safety: Optional[SafetyState] = None
        self._safety_time = 0.0
        self._ros_info: Dict[str, Any] = {}
        self._http_info: Dict[str, Any] = {"enabled": False, "bound": False}
        self._http: Optional[HealthHttpServer] = None
        self._lock = threading.RLock()
        self._payload: Optional[Dict[str, Any]] = None
        self._params_payload: Optional[Dict[str, Any]] = None
        self._pending_publish_rate: Optional[float] = None
        self._odom_track: Dict[str, Tuple[float, float]] = {}
        self._odom_distance: Dict[str, float] = {}

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_health = self.create_publisher(
            HealthStatus, self.topic_name, status_qos
        )
        self.create_service(HealthCheck, self.service_name, self.on_health_service)

        if self.enabled:
            self._build_subscriptions()
            self.create_subscription(
                SafetyState,
                "/center/safety_state",
                self.on_safety,
                status_qos,
            )
        else:
            self.get_logger().warn("health monitoring disabled by config")

        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer, clock=self._steady_clock
        )

        self._refresh_graph()
        self._http_info = self._configured_http_info()
        self._publish_health()
        self._start_http()
        # HTTP 绑定成功后刷新快照里的 http.port/bound 字段
        self._publish_health()

        self.get_logger().info(
            f"health_monitor_node started: topic={self.topic_name}, "
            f"service={self.service_name}, devices={len(self.monitor.devices)}, "
            f"rate={self.publish_rate:.1f}Hz"
        )

    # ------------------------------------------------------------- subscriptions
    def _build_subscriptions(self) -> None:
        for device_id, spec in self.monitor.devices.items():
            if not spec.enabled:
                continue
            if not spec.topic:
                self.monitor.set_config_error(device_id, "topic not configured")
                continue
            if not spec.topic.startswith("/"):
                self.monitor.set_config_error(
                    device_id, f"topic must be absolute: {spec.topic}"
                )
                continue
            if not spec.message_type:
                self.monitor.set_config_error(device_id, "message_type not configured")
                continue
            self._topic_entries.setdefault(spec.topic, []).append((device_id, spec))

        for topic, entries in self._topic_entries.items():
            types = {spec.message_type for _, spec in entries}
            if len(types) != 1:
                for device_id, _ in entries:
                    self.monitor.set_config_error(
                        device_id, f"conflicting message_type on {topic}"
                    )
                continue
            message_type = next(iter(types))
            try:
                message_class = self._resolve_message_class(message_type)
            except Exception as exc:  # noqa: BLE001
                for device_id, _ in entries:
                    self.monitor.set_config_error(
                        device_id, f"unsupported message_type {message_type}: {exc}"
                    )
                continue
            qos = self._qos_for(entries[0][1].qos)
            try:
                subscription = self.create_subscription(
                    message_class,
                    topic,
                    self._make_callback(entries),
                    qos,
                )
            except Exception as exc:  # noqa: BLE001
                for device_id, _ in entries:
                    self.monitor.set_config_error(
                        device_id, f"subscribe failed: {type(exc).__name__}: {exc}"
                    )
                continue
            self._subscriptions.append(subscription)
            self.get_logger().info(
                f"monitoring {topic} ({message_type}, ids="
                f"{','.join(device_id for device_id, _ in entries)})"
            )

    @staticmethod
    def _resolve_message_class(message_type: str) -> Any:
        normalized = message_type.strip()
        if normalized in _KNOWN_MSG_TYPES:
            return _KNOWN_MSG_TYPES[normalized]
        if "/msg/" not in normalized and "/" in normalized:
            package, name = normalized.split("/", 1)
            normalized = f"{package}/msg/{name}"
        if _get_message is None:
            raise ValueError("rosidl_runtime_py unavailable")
        return _get_message(normalized)

    @staticmethod
    def _qos_for(qos_name: str) -> Any:
        name = str(qos_name or "sensor").lower()
        if name in {"transient_local", "transient"}:
            return QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        if name in {"reliable", "default"}:
            return 10
        return qos_profile_sensor_data

    def _make_callback(
        self, entries: Sequence[Tuple[str, DeviceSpec]]
    ) -> Callable[[Any], None]:
        def callback(msg: Any) -> None:
            now = time.monotonic()
            for device_id, spec in entries:
                try:
                    metrics, invalid, error = self._extract_metrics(spec, msg)
                except Exception as exc:  # noqa: BLE001
                    metrics = {}
                    invalid = True
                    error = f"extract failed: {type(exc).__name__}: {exc}"
                self.monitor.on_message(
                    device_id,
                    now,
                    metrics=metrics,
                    invalid=invalid,
                    error=error,
                )

        return callback

    # ---------------------------------------------------------------- extraction
    def _extract_metrics(
        self, spec: DeviceSpec, msg: Any
    ) -> Tuple[Dict[str, Any], bool, str]:
        metrics: Dict[str, Any] = {}
        invalid = False
        error = ""

        header = getattr(msg, "header", None)
        if header is not None and getattr(header, "frame_id", ""):
            metrics["frame_id"] = str(header.frame_id)

        if isinstance(msg, Image):
            metrics.update(
                width=int(msg.width),
                height=int(msg.height),
                encoding=str(msg.encoding),
                bytes=len(msg.data),
            )
            if msg.width <= 0 or msg.height <= 0 or not msg.data:
                invalid = True
                error = "empty image"
        elif isinstance(msg, LaserScan):
            metrics.update(
                ranges=len(msg.ranges),
                angle_min=round(float(msg.angle_min), 3),
                angle_max=round(float(msg.angle_max), 3),
                range_max=round(float(msg.range_max), 3),
            )
            if len(msg.ranges) == 0:
                invalid = True
                error = "empty laser scan"
        elif isinstance(msg, Imu):
            ax = float(msg.linear_acceleration.x)
            ay = float(msg.linear_acceleration.y)
            az = float(msg.linear_acceleration.z)
            gx = float(msg.angular_velocity.x)
            gy = float(msg.angular_velocity.y)
            gz = float(msg.angular_velocity.z)
            accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
            gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
            mag_norm = math.sqrt(
                float(msg.magnetic_field.x) ** 2
                + float(msg.magnetic_field.y) ** 2
                + float(msg.magnetic_field.z) ** 2
            )
            metrics.update(
                accel_norm=round(accel_norm, 3),
                gyro_norm=round(gyro_norm, 3),
                mag_norm=round(mag_norm, 3),
                has_mag=mag_norm > 0.0,
            )
            if not all(
                math.isfinite(value)
                for value in (ax, ay, az, gx, gy, gz, mag_norm)
            ):
                invalid = True
                error = "non-finite IMU sample"
            elif accel_norm <= 0.0 and gyro_norm <= 0.0:
                invalid = True
                error = "all-zero IMU sample"
        elif isinstance(msg, JointState):
            names = [str(name) for name in msg.name]
            metrics["joint_count"] = len(names)
            metrics["joint_names"] = names[:32]
            if spec.joint_names:
                present = [name for name in spec.joint_names if name in names]
                missing = [name for name in spec.joint_names if name not in names]
                metrics["joints_present"] = present
                metrics["joints_missing"] = missing
                if missing:
                    invalid = True
                    error = "missing joints: " + ",".join(missing)
        elif isinstance(msg, BatteryState):
            percentage = float(msg.percentage)
            voltage = float(msg.voltage)
            percent = percentage * 100.0 if math.isfinite(percentage) and percentage >= 0.0 else -1.0
            metrics.update(
                voltage=round(voltage, 3),
                percentage=round(percent, 2),
                present=bool(msg.present),
                power_supply_status=int(msg.power_supply_status),
            )
            if 0.0 <= percent < self.battery_low_percent:
                invalid = True
                error = f"battery low: {percent:.1f}% < {self.battery_low_percent:.1f}%"
        elif isinstance(msg, AudioData):
            metrics.update(
                format=str(msg.format),
                sample_rate=int(msg.sample_rate),
                bytes=len(msg.data),
            )
            if not msg.data:
                invalid = True
                error = "empty audio frame"
        elif isinstance(msg, Odometry):
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            twist = msg.twist.twist
            x = float(position.x)
            y = float(position.y)
            vx = float(twist.linear.x)
            vy = float(twist.linear.y)
            wz = float(twist.angular.z)
            yaw = math.atan2(
                2.0 * (float(orientation.w) * float(orientation.z)
                       + float(orientation.x) * float(orientation.y)),
                1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
            )
            previous = self._odom_track.get(spec.id)
            if previous is not None:
                step = math.hypot(x - previous[0], y - previous[1])
                # 忽略定位跳变，避免把重定位/仿真重置计入里程。
                if 0.0 < step < 5.0:
                    self._odom_distance[spec.id] = (
                        self._odom_distance.get(spec.id, 0.0) + step
                    )
            self._odom_track[spec.id] = (x, y)
            metrics.update(
                x=round(x, 3),
                y=round(y, 3),
                yaw=round(yaw, 4),
                vx=round(vx, 3),
                vy=round(vy, 3),
                wz=round(wz, 3),
                speed=round(math.hypot(vx, vy), 3),
                distance=round(self._odom_distance.get(spec.id, 0.0), 3),
                child_frame_id=str(msg.child_frame_id),
            )
            if not all(math.isfinite(v) for v in (x, y, yaw, vx, vy, wz)):
                invalid = True
                error = "non-finite odometry"
        elif isinstance(msg, String):
            metrics["bytes"] = len(msg.data)
            if spec.kind in {"base", "speaker", "microphone"}:
                try:
                    data = json.loads(msg.data)
                except Exception:  # noqa: BLE001
                    invalid = True
                    error = "invalid JSON payload"
                else:
                    if not isinstance(data, dict):
                        invalid = True
                        error = "JSON payload is not an object"
                    else:
                        metrics.update(self._json_safe(data))
                        if spec.kind == "base" and data.get("encoder_ok") is False:
                            invalid = True
                            error = "encoder_ok=false"
                        if spec.kind == "speaker" and data.get("online") is False:
                            invalid = True
                            error = "speaker offline"
                        if spec.kind == "microphone" and data.get("online") is False:
                            invalid = True
                            error = "microphone offline"
        else:
            metrics["message_class"] = type(msg).__name__

        return self._json_safe(metrics), invalid, error

    def _json_safe(self, value: Any) -> Any:
        if isinstance(value, bool) or isinstance(value, str):
            return value
        if isinstance(value, numbers.Integral):
            return int(value)
        if isinstance(value, numbers.Real):
            number = float(value)
            return number if math.isfinite(number) else None
        if isinstance(value, (list, tuple)):
            return [self._json_safe(item) for item in value]
        if isinstance(value, dict):
            return {str(key): self._json_safe(item) for key, item in value.items()}
        return str(value)

    # --------------------------------------------------------------- ros graph
    def on_safety(self, msg: SafetyState) -> None:
        with self._lock:
            self._safety = msg
            self._safety_time = time.monotonic()

    def _refresh_graph(self) -> None:
        if self._check_nodes:
            try:
                names = sorted(set(str(name) for name in self.get_node_names()))
                missing = [name for name in self._expected_nodes if name not in names]
                self._ros_info = {
                    "ok": not missing,
                    "checked": True,
                    "node_count": len(names),
                    "expected_nodes": list(self._expected_nodes),
                    "missing_nodes": missing,
                }
            except Exception as exc:  # noqa: BLE001
                self._ros_info = {
                    "ok": True,
                    "checked": False,
                    "node_count": -1,
                    "expected_nodes": list(self._expected_nodes),
                    "missing_nodes": [],
                    "error": f"{type(exc).__name__}: {exc}",
                }
        else:
            self._ros_info = {
                "ok": True,
                "checked": False,
                "node_count": -1,
                "expected_nodes": list(self._expected_nodes),
                "missing_nodes": [],
            }
        try:
            self._ros_info["use_sim_time"] = bool(
                self.get_parameter("use_sim_time").value
            )
        except Exception:  # noqa: BLE001
            self._ros_info["use_sim_time"] = False

        for topic, entries in self._topic_entries.items():
            try:
                count = int(self.count_publishers(topic))
            except Exception:  # noqa: BLE001
                count = -1
            for device_id, _ in entries:
                self.monitor.set_publisher_count(device_id, count)

        self.monitor.set_ros_info(self._ros_info)

    def _safety_info(self) -> Dict[str, Any]:
        if not self.enabled or self._safety is None:
            return {
                "available": False,
                "age_sec": -1.0,
                "emergency_stop": False,
                "reason": "",
                "data_ok": None,
                "localization_ok": None,
                "battery_ok": None,
                "battery_percent": -1.0,
                "min_obstacle_distance": -1.0,
            }
        age = time.monotonic() - self._safety_time
        return {
            "available": age <= self.safety_timeout,
            "age_sec": round(age, 3),
            "emergency_stop": bool(self._safety.emergency_stop),
            "reason": str(self._safety.reason),
            "data_ok": bool(self._safety.data_ok),
            "localization_ok": bool(self._safety.localization_ok),
            "battery_ok": bool(self._safety.battery_ok),
            "battery_percent": round(float(self._safety.battery_percent), 2),
            "min_obstacle_distance": round(
                float(self._safety.min_obstacle_distance), 3
            ),
        }

    # ------------------------------------------------------------------ reports
    def _configured_http_info(self) -> Dict[str, Any]:
        http_cfg = dict(self.health_cfg.get("http") or {})
        enabled = bool(http_cfg.get("enabled", True))
        return {
            "enabled": enabled,
            "host": str(http_cfg.get("host", "127.0.0.1")),
            "port": int(http_cfg.get("port", 8090)),
            "configured_port": int(http_cfg.get("port", 8090)),
            "bound": False,
            "error": "",
            "auth": bool(http_cfg.get("api_key")),
        }

    def _http_provider(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._payload is None:
                return None
            return copy.deepcopy(self._payload)

    def _params_provider(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._params_payload is None:
                return None
            return copy.deepcopy(self._params_payload)

    def _params_updater(self, updates: Mapping[str, Any]) -> Dict[str, Any]:
        with self._lock:
            result = self._apply_params(updates)
            self._params_payload = self._build_params_payload()
            return result

    @staticmethod
    def _coerce_float(value: Any, key: str, low: float, high: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} 必须是数字") from exc
        if not math.isfinite(number) or number < low or number > high:
            raise ValueError(f"{key} 必须在 {low}..{high}")
        return number

    @staticmethod
    def _coerce_bool(value: Any, key: str) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off"}:
                return False
        if isinstance(value, (int, float)):
            return bool(value)
        raise ValueError(f"{key} 必须是布尔值")

    @staticmethod
    def _param_item(
        key: str,
        label: str,
        value: Any,
        value_type: str,
        group: str,
        *,
        unit: str = "",
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        step: Optional[float] = None,
        editable: bool = True,
        requires_restart: bool = False,
        help_text: str = "",
    ) -> Dict[str, Any]:
        item: Dict[str, Any] = {
            "key": key,
            "label": label,
            "value": value,
            "type": value_type,
            "group": group,
            "editable": bool(editable),
            "requires_restart": bool(requires_restart),
        }
        if unit:
            item["unit"] = unit
        if min_value is not None:
            item["min"] = min_value
        if max_value is not None:
            item["max"] = max_value
        if step is not None:
            item["step"] = step
        if help_text:
            item["help"] = help_text
        return item

    def _build_params_payload(self) -> Dict[str, Any]:
        params: List[Dict[str, Any]] = [
            self._param_item(
                "publish_rate", "健康发布频率", self.publish_rate, "float",
                "健康检查", unit="Hz", min_value=0.2, max_value=10.0, step=0.1,
            ),
            self._param_item(
                "startup_grace_sec", "启动宽限期", self.startup_grace, "float",
                "健康检查", unit="s", min_value=0.0, max_value=120.0, step=1.0,
            ),
            self._param_item(
                "rate_window_sec", "频率统计窗口", self.rate_window, "float",
                "健康检查", unit="s", min_value=0.5, max_value=60.0, step=0.5,
            ),
            self._param_item(
                "strict_unknown", "未知设备降级", self.strict_unknown, "bool",
                "健康检查", help_text="开启后可选设备 unknown 也把整体降为 degraded",
            ),
            self._param_item(
                "safety_required", "安全状态必需", self.safety_required, "bool",
                "健康检查",
            ),
            self._param_item(
                "safety_timeout_sec", "安全状态超时", self.safety_timeout, "float",
                "健康检查", unit="s", min_value=0.5, max_value=60.0, step=0.5,
            ),
            self._param_item(
                "check_nodes", "ROS 节点发现", self._check_nodes, "bool",
                "健康检查",
            ),
        ]

        for device_id, spec in self.monitor.devices.items():
            group = f"设备 · {device_id}"
            params.extend(
                [
                    self._param_item(
                        f"device.{device_id}.expected_rate_hz", "期望频率",
                        spec.expected_rate_hz, "float", group,
                        unit="Hz", min_value=0.0, max_value=1000.0, step=0.5,
                    ),
                    self._param_item(
                        f"device.{device_id}.timeout_sec", "数据超时",
                        spec.timeout_sec, "float", group,
                        unit="s", min_value=0.0, max_value=60.0, step=0.1,
                    ),
                    self._param_item(
                        f"device.{device_id}.min_rate_ratio", "最低频率比例",
                        spec.min_rate_ratio, "float", group,
                        min_value=0.0, max_value=1.0, step=0.05,
                    ),
                    self._param_item(
                        f"device.{device_id}.required", "必需设备",
                        spec.required, "bool", group,
                    ),
                    self._param_item(
                        f"device.{device_id}.topic", "反馈话题",
                        spec.topic, "string", group, editable=False,
                    ),
                    self._param_item(
                        f"device.{device_id}.message_type", "消息类型",
                        spec.message_type, "string", group, editable=False,
                    ),
                ]
            )

        return {
            "ok": True,
            "module": "ros",
            "name": "ROS 2 edge_server",
            "version": SERVICE_VERSION,
            "editable": True,
            "requires_restart": False,
            "params": params,
        }

    def _apply_params(self, updates: Mapping[str, Any]) -> Dict[str, Any]:
        if not isinstance(updates, Mapping) or not updates:
            raise ValueError("updates 必须是非空对象")

        health_values: Dict[str, Any] = {}
        device_values: Dict[str, Dict[str, Any]] = {}
        updated: Dict[str, Any] = {}

        for raw_key, raw_value in updates.items():
            key = str(raw_key)
            if key == "publish_rate":
                value = self._coerce_float(raw_value, key, 0.2, 10.0)
                health_values["publish_rate"] = value
            elif key == "startup_grace_sec":
                value = self._coerce_float(raw_value, key, 0.0, 120.0)
                health_values["startup_grace_sec"] = value
            elif key == "rate_window_sec":
                value = self._coerce_float(raw_value, key, 0.5, 60.0)
                health_values["rate_window_sec"] = value
            elif key == "strict_unknown":
                value = self._coerce_bool(raw_value, key)
                health_values["strict_unknown"] = value
            elif key == "safety_required":
                value = self._coerce_bool(raw_value, key)
                health_values["safety_required"] = value
            elif key == "safety_timeout_sec":
                value = self._coerce_float(raw_value, key, 0.5, 60.0)
                health_values["safety_timeout_sec"] = value
            elif key == "check_nodes":
                value = self._coerce_bool(raw_value, key)
                health_values["check_nodes"] = value
            elif key.startswith("device."):
                parts = key.split(".")
                if len(parts) != 3:
                    raise ValueError(f"非法参数名: {key}")
                device_id, field = parts[1], parts[2]
                if device_id not in self.monitor.states:
                    raise ValueError(f"未知设备: {device_id}")
                if field == "expected_rate_hz":
                    value = self._coerce_float(raw_value, key, 0.0, 1000.0)
                elif field == "timeout_sec":
                    value = self._coerce_float(raw_value, key, 0.0, 60.0)
                elif field == "min_rate_ratio":
                    value = self._coerce_float(raw_value, key, 0.0, 1.0)
                elif field == "required":
                    value = self._coerce_bool(raw_value, key)
                else:
                    raise ValueError(f"{key} 是只读参数或不存在")
                device_values.setdefault(device_id, {})[field] = value
            else:
                raise ValueError(f"未知参数: {key}")
            updated[key] = value

        # 全部校验通过后再应用，避免半更新。
        if "publish_rate" in health_values:
            self.publish_rate = health_values["publish_rate"]
            self._pending_publish_rate = self.publish_rate
        if "startup_grace_sec" in health_values:
            self.startup_grace = health_values["startup_grace_sec"]
            self.monitor.startup_grace_sec = self.startup_grace
        if "rate_window_sec" in health_values:
            self.rate_window = health_values["rate_window_sec"]
            self.monitor.rate_window_sec = self.rate_window
        if "strict_unknown" in health_values:
            self.strict_unknown = health_values["strict_unknown"]
            self.monitor.strict_unknown = self.strict_unknown
        if "safety_required" in health_values:
            self.safety_required = health_values["safety_required"]
            self.monitor.safety_required = self.safety_required
        if "safety_timeout_sec" in health_values:
            self.safety_timeout = health_values["safety_timeout_sec"]
        if "check_nodes" in health_values:
            self._check_nodes = health_values["check_nodes"]

        for device_id, changes in device_values.items():
            self.monitor.configure_device(device_id, **changes)

        return {
            "updated": updated,
            "requires_restart": False,
            "message": "参数已更新",
        }

    def _apply_pending_publish_rate(self) -> None:
        with self._lock:
            rate = self._pending_publish_rate
            if rate is None:
                return
            self._pending_publish_rate = None
            self.timer.timer_period_ns = int(1_000_000_000 / max(0.2, rate))

    def _build_report(
        self,
        *,
        now: Optional[float] = None,
        device_ids: Optional[Sequence[str]] = None,
    ) -> HealthReport:
        return self.monitor.snapshot(
            now=time.monotonic() if now is None else now,
            device_ids=device_ids,
            ros_info=self._ros_info,
            safety=self._safety_info(),
            http_info=self._http_info,
        )

    def _publish_health(self, *, now: Optional[float] = None) -> HealthReport:
        with self._lock:
            self._refresh_graph()
            report = self._build_report(now=now)
            payload = report.to_dict(include_metrics=True)
            params = self._build_params_payload()
            self._payload = payload
            self._params_payload = params
            message = self._to_message(report, include_metrics=True)
        self.pub_health.publish(message)
        return report

    def _to_message(self, report: HealthReport, *, include_metrics: bool = True) -> HealthStatus:
        msg = HealthStatus()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.status = report.status
        msg.healthy = bool(report.healthy)
        msg.ready = bool(report.ready)
        msg.uptime_sec = float(report.uptime_sec)
        msg.service = report.service
        msg.version = report.version
        msg.reason = report.reason
        msg.emergency_stop = bool(report.safety.get("emergency_stop", False))
        msg.safety_reason = str(report.safety.get("reason", ""))
        msg.safety_available = bool(report.safety.get("available", False))
        summary = report.summary
        msg.device_count = int(summary.get("total", 0))
        msg.ok_count = int(summary.get("ok", 0))
        msg.degraded_count = int(summary.get("degraded", 0))
        msg.error_count = int(summary.get("error", 0))
        msg.unknown_count = int(summary.get("unknown", 0))
        msg.disabled_count = int(summary.get("disabled", 0))
        msg.ros_json = json.dumps(
            report.ros, ensure_ascii=False, separators=(",", ":")
        )
        for report_device in report.devices:
            device = DeviceHealth()
            device.header.stamp = stamp
            device.id = report_device.id
            device.type = report_device.type
            device.status = report_device.status
            device.healthy = bool(report_device.healthy)
            device.ready = bool(report_device.ready)
            device.enabled = bool(report_device.enabled)
            device.required = bool(report_device.required)
            device.connected = bool(report_device.connected)
            device.data_fresh = bool(report_device.data_fresh)
            device.topic = report_device.topic
            device.message_type = report_device.message_type
            device.publisher_count = int(report_device.publisher_count)
            device.age_sec = float(report_device.age_sec)
            device.rate_hz = float(report_device.rate_hz)
            device.expected_rate_hz = float(report_device.expected_rate_hz)
            device.detail = report_device.detail
            if include_metrics and report_device.metrics:
                device.metrics_json = json.dumps(
                    report_device.metrics,
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
            msg.devices.append(device)
        return msg

    # ------------------------------------------------------------------- events
    def on_timer(self) -> None:
        try:
            self._apply_pending_publish_rate()
            self._publish_health()
        except Exception as exc:  # noqa: BLE001 - 健康节点自身不能因一次快照失败退出
            self.get_logger().error(
                f"health snapshot failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )

    def on_health_service(self, request: HealthCheck.Request, response: HealthCheck.Response):
        ids = [str(device_id) for device_id in request.device_ids]
        try:
            with self._lock:
                self._refresh_graph()
                report = self._build_report(device_ids=ids or None)
                message = self._to_message(
                    report, include_metrics=bool(request.include_metrics)
                )
        except KeyError as exc:
            message = f"未知设备: {exc}"
            response.success = False
            response.message = message
            fallback = HealthStatus()
            fallback.header.stamp = self.get_clock().now().to_msg()
            fallback.status = "error"
            fallback.reason = message
            fallback.service = SERVICE_NAME
            fallback.version = SERVICE_VERSION
            response.status = fallback
            return response
        response.success = True
        response.status = message
        response.message = report.reason
        return response

    # --------------------------------------------------------------------- http
    def _start_http(self) -> None:
        http_cfg = dict(self.health_cfg.get("http") or {})
        if not bool(http_cfg.get("enabled", True)):
            self._http_info = self._configured_http_info()
            return
        host = str(http_cfg.get("host", "127.0.0.1"))
        port = int(http_cfg.get("port", 8090))
        self._http = HealthHttpServer(
            host,
            port,
            self._http_provider,
            api_key=str(http_cfg.get("api_key") or ""),
            cors_allow_origin=str(http_cfg.get("cors_allow_origin") or "*"),
            service=SERVICE_NAME,
            version=SERVICE_VERSION,
            params_provider=self._params_provider,
            params_updater=self._params_updater,
        )
        self._http.start()
        self._http_info = self._http.info()
        if self._http_info.get("bound"):
            self.get_logger().info(
                "health HTTP listening on "
                f"http://{self._http_info.get('host')}:{self._http_info.get('port')}"
                f" (auth={'on' if self._http_info.get('auth') else 'off'})"
            )
        else:
            self.get_logger().error(
                "health HTTP bind failed: "
                f"{self._http_info.get('error') or 'unknown error'}"
            )

    # ------------------------------------------------------------------- teardown
    def destroy_node(self) -> None:
        http = self._http
        self._http = None
        if http is not None:
            try:
                http.stop()
            except (Exception, KeyboardInterrupt):
                pass
        try:
            super().destroy_node()
        except (Exception, KeyboardInterrupt):
            # 关停阶段可能被第二个 SIGINT 打断，或 rclpy 已完成部分销毁；
            # 健康检查节点不应因此在正常退出时打印 traceback/返回非零。
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[HealthMonitorNode] = None
    try:
        node = HealthMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
