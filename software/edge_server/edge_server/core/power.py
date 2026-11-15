"""树莓派电源/欠压监控（不依赖 ROS）。

数据源（只读）：

- ``vcgencmd get_throttled``：固件记录的欠压/降频/温度限制位；
- ``/sys/class/hwmon/hwmon*/in0_lcrit_alarm``：`rpi_volt` 当前低电压告警；
- ``vcgencmd measure_volts core`` / ``measure_temp``：辅助信息；
- ``/proc/device-tree/model``：板卡型号。

判定约定（与硬件模块“电源及转换板”对应）：

- 当前欠压（bit0 或 alarm=1）→ ``error``；
- 当前降频/温度限制（bit1/bit2/bit3）→ ``degraded``；
- 仅历史位（bit16/bit18/bit19）→ 当前正常 ``ok``，但 ``warning=true``，
  并保留 ``throttled_raw`` 供 dashboard/日志查看。

外部命令与 sysfs 都可注入，便于单元测试。
"""

from __future__ import annotations

import glob
import logging
import os
import re
import shutil
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional

from .speaker import SubprocessRunner

LOGGER = logging.getLogger("edge_server.power")

_THROTTLED_RE = re.compile(r"throttled=(0x[0-9a-fA-F]+)")
_VOLT_RE = re.compile(r"volt=([0-9.]+)V")
_TEMP_RE = re.compile(r"temp=([0-9.]+)'?C")

# get_throttled 位定义（Raspberry Pi firmware）
UV_NOW = 1 << 0
FREQ_CAPPED_NOW = 1 << 1
THROTTLED_NOW = 1 << 2
SOFT_TEMP_LIMIT_NOW = 1 << 3
UV_OCCURRED = 1 << 16
FREQ_CAPPED_OCCURRED = 1 << 17
THROTTLED_OCCURRED = 1 << 18
SOFT_TEMP_LIMIT_OCCURRED = 1 << 19


def parse_throttled(output: Any) -> Dict[str, Any]:
    """解析 ``throttled=0x...``；非法输出返回 available=False。"""
    text = str(output or "")
    match = _THROTTLED_RE.search(text)
    if not match:
        return {"available": False, "mask": 0, "raw": ""}
    try:
        mask = int(match.group(1), 16)
    except (TypeError, ValueError):
        return {"available": False, "mask": 0, "raw": ""}
    return {
        "available": True,
        "mask": mask,
        "raw": f"0x{mask:x}",
        "undervoltage": bool(mask & UV_NOW),
        "frequency_capped": bool(mask & FREQ_CAPPED_NOW),
        "throttled": bool(mask & THROTTLED_NOW),
        "soft_temp_limit": bool(mask & SOFT_TEMP_LIMIT_NOW),
        "undervoltage_occurred": bool(mask & UV_OCCURRED),
        "frequency_capped_occurred": bool(mask & FREQ_CAPPED_OCCURRED),
        "throttled_occurred": bool(mask & THROTTLED_OCCURRED),
        "soft_temp_limit_occurred": bool(mask & SOFT_TEMP_LIMIT_OCCURRED),
    }


def parse_volts(output: Any) -> Optional[float]:
    match = _VOLT_RE.search(str(output or ""))
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def parse_temp(output: Any) -> Optional[float]:
    match = _TEMP_RE.search(str(output or ""))
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class PowerMonitor:
    """缓存式树莓派电源状态读取。"""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        runner: Any = None,
        which: Optional[Callable[[str], Optional[str]]] = None,
        read_text: Optional[Callable[[str], str]] = None,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        cfg = dict(config or {})
        self.enabled = bool(cfg.get("enabled", True))
        self.sample_interval_sec = _clamp(
            float(cfg.get("sample_interval_sec", 2.0)), 0.5, 60.0
        )
        self.model_path = str(
            cfg.get("model_path", "/proc/device-tree/model")
            or "/proc/device-tree/model"
        )
        self.alarm_path = str(cfg.get("alarm_path", "") or "")
        self.vcgencmd = str(cfg.get("vcgencmd", "vcgencmd") or "vcgencmd")
        self._runner = runner if runner is not None else SubprocessRunner()
        self._which = which if which is not None else shutil.which
        self._read_text = read_text if read_text is not None else _read_text_file
        self._clock = clock
        self._wall_clock = wall_clock
        self._lock = threading.Lock()
        self._cache: Optional[Dict[str, Any]] = None
        self._cache_at = 0.0

    # ------------------------------------------------------------- status
    def status(self, *, force: bool = False) -> Dict[str, Any]:
        now = float(self._clock())
        with self._lock:
            if (
                not force
                and self._cache is not None
                and (now - self._cache_at) < self.sample_interval_sec
            ):
                payload = dict(self._cache)
                payload["age_ms"] = int(max(0.0, now - self._cache_at) * 1000.0)
                return payload

        payload = self._read_status()
        with self._lock:
            self._cache = dict(payload)
            self._cache_at = now
        payload = dict(payload)
        payload["age_ms"] = 0
        return payload

    def _read_status(self) -> Dict[str, Any]:
        now_wall = float(self._wall_clock())
        base: Dict[str, Any] = {
            "ok": True,
            "enabled": bool(self.enabled),
            "available": False,
            "source": "rpi",
            "model": "",
            "status": "disabled" if not self.enabled else "unknown",
            "healthy": False,
            "online": False,
            "undervoltage": False,
            "frequency_capped": False,
            "throttled": False,
            "soft_temp_limit": False,
            "undervoltage_occurred": False,
            "throttled_occurred": False,
            "throttled_raw": "",
            "alarm": -1,
            "core_voltage_v": None,
            "temperature_c": None,
            "warning": False,
            "warnings": [],
            "error": "",
            "checked_ms": int(now_wall * 1000.0),
        }
        if not self.enabled:
            return base

        base["model"] = _clean_text(self._read_text(self.model_path))
        base["alarm"] = _read_alarm(self._read_text, self.alarm_path)
        binary = self._which(self.vcgencmd)
        if not binary:
            base["status"] = "error"
            base["error"] = f"未找到 {self.vcgencmd}"
            return base

        throttled = self._run([binary, "get_throttled"], timeout=4.0)
        flags = parse_throttled(throttled.stdout if throttled.ok else "")
        if not flags.get("available"):
            base["status"] = "error"
            base["error"] = "vcgencmd get_throttled 不可用"
            return base
        volts = self._run([binary, "measure_volts", "core"], timeout=4.0)
        temp = self._run([binary, "measure_temp"], timeout=4.0)

        base["available"] = True
        base["throttled_raw"] = str(flags["raw"])
        for key in (
            "undervoltage",
            "frequency_capped",
            "throttled",
            "soft_temp_limit",
            "undervoltage_occurred",
            "frequency_capped_occurred",
            "throttled_occurred",
            "soft_temp_limit_occurred",
        ):
            base[key] = bool(flags.get(key, False))
        base["core_voltage_v"] = parse_volts(volts.stdout) if volts.ok else None
        base["temperature_c"] = parse_temp(temp.stdout) if temp.ok else None

        warnings = []
        if base["undervoltage"]:
            warnings.append("当前检测到 5V 欠压")
        if base["alarm"] == 1 and not base["undervoltage"]:
            warnings.append("rpi_volt 低电压告警")
        if base["frequency_capped"]:
            warnings.append("当前 ARM 频率被限制")
        if base["throttled"]:
            warnings.append("当前处于降频/限流状态")
        if base["soft_temp_limit"]:
            warnings.append("当前触发软温度限制")
        if base["undervoltage_occurred"] and not base["undervoltage"]:
            warnings.append("此前发生过 5V 欠压，当前已恢复")
        if base["throttled_occurred"]:
            warnings.append("此前发生过降频/限流")
        if base["soft_temp_limit_occurred"]:
            warnings.append("此前触发过软温度限制")

        current_power_bad = bool(
            base["undervoltage"] or base["alarm"] == 1 or base["throttled"]
        )
        history_only = bool(
            base["undervoltage_occurred"]
            or base["frequency_capped_occurred"]
            or base["throttled_occurred"]
            or base["soft_temp_limit_occurred"]
            or (base["frequency_capped"] or base["soft_temp_limit"])
        )
        if current_power_bad:
            base["status"] = "error" if (
                base["undervoltage"] or base["alarm"] == 1
            ) else "degraded"
        elif base["frequency_capped"] or base["soft_temp_limit"]:
            base["status"] = "degraded"
        else:
            base["status"] = "ok"
        base["warning"] = bool(history_only and base["status"] == "ok")
        base["warning"] = bool(
            base["warning"]
            or (base["status"] == "ok" and base["undervoltage_occurred"])
        )
        base["warnings"] = warnings
        base["healthy"] = base["status"] == "ok"
        base["online"] = True
        return base

    def _run(self, args: list, *, timeout: float) -> Any:
        try:
            return self._runner.run(args, timeout=timeout)
        except TypeError:
            return self._runner.run(args)
        except Exception as exc:  # noqa: BLE001
            from .speaker import CommandResult

            return CommandResult(126, "", str(exc))


def _read_text_file(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        return ""


def _clean_text(value: Any) -> str:
    return str(value or "").replace("\x00", "").strip()


def _read_alarm(read_text: Callable[[str], str], path: str) -> int:
    candidate = str(path or "").strip()
    if not candidate:
        for name_path in glob.glob("/sys/class/hwmon/hwmon*/name"):
            try:
                if read_text(name_path).strip() == "rpi_volt":
                    candidate = os.path.join(os.path.dirname(name_path), "in0_lcrit_alarm")
                    break
            except Exception:  # noqa: BLE001
                continue
    if not candidate:
        return -1
    try:
        return int(str(read_text(candidate)).strip())
    except (TypeError, ValueError):
        return -1
