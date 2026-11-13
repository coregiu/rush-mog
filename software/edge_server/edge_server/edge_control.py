"""Edge Server 控制面纯逻辑（不依赖 ROS/串口/HTTP）。

原 ESP32 `edge_server` 的按钮/摇杆映射、运动保活、STM32 ASCII 遥测解析
迁移到树莓派后，集中放在本模块，便于单元测试和 HTTP/ROS 两层复用。

约定：
- STM32 USART3 115200 8N1，ASCII 命令以 ``\\n`` 结尾；
- 遥测行 ``$B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>``；
- 按钮/摇杆命令沿用原网页协议（见 ``software/api.yaml``）。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional


MAX_WHEEL_MM_S = 5000
BODY_FRESH_SEC = 2.0
RAW_MOTION_LEASE_SEC = 1.0
MOTION_KEEPALIVE_MS = 200


def _clamp_keepalive(value: float) -> int:
    return max(100, min(600, int(value)))


@dataclass
class BodyTelemetry:
    """STM32 最新一帧编码器遥测。"""

    valid: bool = False
    seq: int = 0
    uptime_ms: int = 0
    wheel_mm_s: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    distance_mm: int = -1
    received_at: float = 0.0

    def to_dict(self, now: Optional[float] = None) -> dict:
        moment = time.monotonic() if now is None else float(now)
        valid = bool(self.valid and self.received_at > 0.0)
        age_ms = (
            int(max(0.0, moment - self.received_at) * 1000.0) if valid else -1
        )
        stale = (not valid) or age_ms > int(BODY_FRESH_SEC * 1000.0)
        return {
            "valid": valid,
            "stale": stale,
            "source": "stm32",
            "rate_hz": 10,
            "seq": int(self.seq),
            "uptime_ms": int(self.uptime_ms),
            "age_ms": age_ms,
            "wheel_mm_s": [int(value) for value in self.wheel_mm_s],
            "distance_mm": int(self.distance_mm) if valid else -1,
        }


def parse_body_line(
    line: str, now: Optional[float] = None
) -> Optional[BodyTelemetry]:
    """解析一行 ``$B,...`` 遥测；非法/越界行返回 None。"""
    text = str(line or "").strip()
    if not text.startswith("$B,"):
        return None
    parts = text.split(",")
    if len(parts) != 8:
        return None
    try:
        seq = int(parts[1])
        uptime_ms = int(parts[2])
        wheels = [int(parts[3]), int(parts[4]), int(parts[5]), int(parts[6])]
        distance_mm = int(parts[7])
    except (TypeError, ValueError):
        return None
    if seq < 0 or uptime_ms < 0 or distance_mm < 0 or distance_mm > 2_000_000_000:
        return None
    if any(value < -MAX_WHEEL_MM_S or value > MAX_WHEEL_MM_S for value in wheels):
        return None
    return BodyTelemetry(
        valid=True,
        seq=seq,
        uptime_ms=uptime_ms,
        wheel_mm_s=wheels,
        distance_mm=distance_mm,
        received_at=time.monotonic() if now is None else float(now),
    )


def is_sustained_motion_command(command: str) -> bool:
    """按下后需要一直保持到松开/停止的命令。"""
    cmd = str(command or "")
    if len(cmd) == 1:
        return ("1" <= cmd <= "9") or cmd in {"A", "B", "C"}
    if len(cmd) == 2:
        return "1" <= cmd[0] <= "8" and "0" <= cmd[1] <= "9"
    return False


def is_valid_raw_motion_command(command: str) -> bool:
    """ROS/API ``raw:`` 命令校验：方向 0-9/A-C，可选一档 PWM 0-9。"""
    cmd = str(command or "")
    if not 1 <= len(cmd) <= 2:
        return False
    first = cmd[0]
    if not (("0" <= first <= "9") or first in {"A", "B", "C"}):
        return False
    if len(cmd) == 2:
        if first == "0":
            return False
        return "0" <= cmd[1] <= "9"
    return True


_BUTTON_DOWN = {
    "up": "1",
    "down": "2",
    "left": "3",
    "right": "4",
    "triangle": "E",
    "cross": "F",
    "square": "G",
    "circle": "H",
    "l1": "9",
    "r1": "A",
    "l2": "B",
    "r2": "C",
    "start": "D",
    "select": "J",
    "stop": "0",
}

_STICK_DIRECTIONS = {
    "up": "1",
    "down": "2",
    "left": "3",
    "right": "4",
    "uple": "5",
    "uprt": "6",
    "dnle": "7",
    "dnrt": "8",
}


def button_command(key: str, event_type: str) -> str:
    """按钮事件 -> STM32 命令；空字符串表示不发送。"""
    name = str(key or "").strip().lower()
    event = str(event_type or "").strip().lower()
    if name == "stop":
        return "0"
    if event == "up":
        if name in {"triangle", "cross", "select", "start"}:
            return ""
        if name in {"square", "circle"}:
            return "I"
        return "0"
    return _BUTTON_DOWN.get(name, "")


def stick_command(stick_id: str, direct: str, step_value: str) -> str:
    """左摇杆事件 -> 方向+PWM 命令；非法输入返回空字符串。"""
    if str(stick_id or "").strip() != "left-analog":
        return ""
    step = str(step_value or "").strip()
    direction = str(direct or "").strip()
    if len(step) != 1 or not step.isdigit():
        return ""
    if direction == "stop" or step == "0":
        return "0"
    prefix = _STICK_DIRECTIONS.get(direction)
    if not prefix:
        return ""
    return prefix + step


class MotionState:
    """当前持续运动命令与保活状态（线程安全）。

    ``manual`` 来源是网页按钮/摇杆：按下后保持，直到收到松开/停止；
    ``raw`` 来源是 ROS ``/cmd_vel`` 或 API ``raw:``：保留 PWM 第二字节，
    并带 1s 租约，调用方停止续租后自动停车。
    """

    def __init__(
        self,
        keepalive_ms: int = MOTION_KEEPALIVE_MS,
        raw_lease_sec: float = RAW_MOTION_LEASE_SEC,
    ) -> None:
        self._lock = threading.Lock()
        self._keepalive_ms = _clamp_keepalive(keepalive_ms)
        self._raw_lease_sec = max(0.1, float(raw_lease_sec))
        self._command = ""
        self._active = False
        self._preserve_second_byte = False
        self._raw_updated_at = 0.0
        self._manual = False

    @property
    def keepalive_ms(self) -> int:
        with self._lock:
            return self._keepalive_ms

    @keepalive_ms.setter
    def keepalive_ms(self, value: int) -> None:
        with self._lock:
            self._keepalive_ms = _clamp_keepalive(value)

    def _set_locked(self, command: str, *, manual: bool, now: float) -> None:
        self._command = command
        self._active = True
        self._manual = manual
        self._preserve_second_byte = (not manual) and len(command) == 2
        self._raw_updated_at = 0.0 if manual else float(now)

    def clear(self) -> None:
        with self._lock:
            self._command = ""
            self._active = False
            self._preserve_second_byte = False
            self._raw_updated_at = 0.0
            self._manual = False

    def apply_manual(self, command: str, now: Optional[float] = None) -> str:
        """应用网页按钮/摇杆命令，返回需要立即下发的命令。"""
        cmd = str(command or "")
        if not cmd:
            return ""
        if cmd in {"0", "J", "D"}:
            self.clear()
            return cmd
        moment = time.monotonic() if now is None else float(now)
        if is_sustained_motion_command(cmd):
            with self._lock:
                self._set_locked(cmd, manual=True, now=moment)
        # 变速/微调等瞬时命令不改变持续运动状态，只立即下发。
        return cmd

    def apply_raw(
        self,
        command: str,
        now: Optional[float] = None,
        *,
        respect_manual: bool = True,
    ) -> str:
        """应用 ROS/API raw 命令；网页手动控制期间默认不抢占。"""
        cmd = str(command or "").strip()
        if not cmd or not is_valid_raw_motion_command(cmd):
            return ""
        moment = time.monotonic() if now is None else float(now)
        with self._lock:
            if respect_manual and self._manual and self._active:
                return ""
            if cmd == "0":
                self._command = ""
                self._active = False
                self._preserve_second_byte = False
                self._raw_updated_at = 0.0
                self._manual = False
                return cmd
            self._command = cmd
            self._active = True
            self._manual = False
            self._preserve_second_byte = len(cmd) == 2
            self._raw_updated_at = moment
        return cmd

    def tick(self, now: Optional[float] = None) -> str:
        """保活线程周期调用；返回要重发的命令或 ``""``。"""
        moment = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._active or not self._command:
                return ""
            if (
                not self._manual
                and self._raw_updated_at > 0.0
                and (moment - self._raw_updated_at) > self._raw_lease_sec
            ):
                self._command = ""
                self._active = False
                self._preserve_second_byte = False
                self._raw_updated_at = 0.0
                return "0"
            command = self._command
            if len(command) == 2 and not self._preserve_second_byte:
                # 网页摇杆保活只重发方向，避免覆盖之后调过的速度/微调。
                command = command[0]
            return command

    def snapshot(self, now: Optional[float] = None) -> dict:
        moment = time.monotonic() if now is None else float(now)
        with self._lock:
            age_ms = (
                int(max(0.0, moment - self._raw_updated_at) * 1000.0)
                if self._raw_updated_at > 0.0
                else -1
            )
            return {
                "active": bool(self._active),
                "command": self._command,
                "manual": bool(self._manual),
                "raw_age_ms": age_ms,
                "keepalive_ms": int(self._keepalive_ms),
            }


def dispatch_control_message(
    message: str,
    motion: MotionState,
    now: Optional[float] = None,
) -> str:
    """解析 ``POST /cmd`` 的一条消息，返回要下发的 STM32 命令。"""
    text = str(message or "").strip()
    moment = time.monotonic() if now is None else float(now)
    if text.startswith("raw:"):
        return motion.apply_raw(text[4:].strip(), moment)
    if text.startswith("button:"):
        rest = text[7:]
        key, sep, event_type = rest.partition(":")
        if not sep:
            return ""
        command = button_command(key, event_type)
        return motion.apply_manual(command, moment) if command else ""
    if text.startswith("stick:"):
        parts = text.split(":", 3)
        if len(parts) != 4:
            return ""
        command = stick_command(parts[1], parts[2], parts[3])
        return motion.apply_manual(command, moment) if command else ""
    return ""
