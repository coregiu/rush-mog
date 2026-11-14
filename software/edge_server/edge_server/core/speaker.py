"""蓝牙音箱管理与语音播放（不依赖 ROS/网页）。

本模块把树莓派上的外部命令收敛到一个可测试的后台服务：

- 自动发现/配对/信任/连接蓝牙音箱（默认 ``newmine``）；
- 连接后等待 PipeWire/PulseAudio 创建 A2DP sink，并设为默认输出；
- 串行执行 TTS 语音与本地音频文件，避免多个播放器抢占设备；
- 提供线程安全的状态快照，供 HTTP API、ROS 话题和 dashboard 复用。

外部命令只通过参数列表调用（不经过 shell），设备名/文本都按普通参数传递；
单元测试可注入 ``runner``/``which``，不依赖真实蓝牙或声卡。
"""

from __future__ import annotations

import logging
import os
import queue
import re
import shutil
import subprocess
import tempfile
import threading
import time
import wave
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional

LOGGER = logging.getLogger("edge_server.speaker")

_MAC_RE = re.compile(r"^[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}$")
_VOICE_RE = re.compile(r"^[A-Za-z0-9_+\-]{1,24}$")
_DEVICE_LINE_RE = re.compile(
    r"^Device\s+([0-9A-Fa-f:]{17})\s*(.*)$", re.IGNORECASE
)
_WP_SINK_RE = re.compile(r"(\d+)\.\s+(bluez_output\.[^\s]+)")
_CONTROL_RE = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")
_AUDIO_EXTENSIONS = {".wav", ".mp3", ".ogg", ".flac", ".m4a", ".aac"}

_STATE_DISABLED = "disabled"
_STATE_STARTING = "starting"
_STATE_SCANNING = "scanning"
_STATE_CONNECTING = "connecting"
_STATE_CONNECTED = "connected"
_STATE_DISCONNECTED = "disconnected"
_STATE_ERROR = "error"


class SpeakerError(RuntimeError):
    """音箱不可用或播放失败。"""


def normalize_mac(value: Any) -> str:
    """返回大写冒号分隔的 MAC；非法值返回空字符串。"""
    text = str(value or "").strip().replace("-", ":")
    if not _MAC_RE.match(text):
        return ""
    return text.upper()


def sanitize_text(value: Any, max_chars: int = 200) -> str:
    """清理 TTS 文本：去控制字符、折叠空白、限制长度。"""
    text = _CONTROL_RE.sub(" ", str(value or ""))
    text = " ".join(text.split())
    limit = max(1, int(max_chars))
    return text[:limit]


def parse_bluetooth_devices(output: str) -> List[Dict[str, str]]:
    """解析 ``bluetoothctl devices`` 输出。"""
    devices: List[Dict[str, str]] = []
    for line in str(output or "").splitlines():
        match = _DEVICE_LINE_RE.match(line.strip())
        if not match:
            continue
        devices.append(
            {
                "mac": match.group(1).upper(),
                "name": (match.group(2) or "").strip(),
            }
        )
    return devices


def parse_bluetooth_info(output: str) -> Dict[str, Any]:
    """解析 ``bluetoothctl info MAC`` 的关键字段。"""
    info: Dict[str, Any] = {
        "name": "",
        "alias": "",
        "paired": False,
        "trusted": False,
        "connected": False,
    }
    for line in str(output or "").splitlines():
        text = line.strip()
        if not text or ":" not in text:
            continue
        key, _, raw = text.partition(":")
        key = key.strip().lower()
        value = raw.strip()
        if key in {"name", "alias"} and value:
            info[key] = value
        elif key in {"paired", "trusted", "connected"}:
            info[key] = value.lower() == "yes"
    return info


def parse_sink_list(output: str) -> List[Dict[str, Any]]:
    """解析 ``pactl list short sinks``（tab 或空白分隔）。"""
    sinks: List[Dict[str, Any]] = []
    for line in str(output or "").splitlines():
        parts = line.strip().split("\t")
        if len(parts) < 2:
            parts = line.split()
        if len(parts) < 2 or not parts[0].isdigit():
            continue
        sinks.append({"id": int(parts[0]), "name": parts[1]})
    return sinks


def parse_wpctl_status(output: str) -> List[Dict[str, Any]]:
    """从 ``wpctl status`` 文本里提取 bluez sink。"""
    sinks: List[Dict[str, Any]] = []
    for line in str(output or "").splitlines():
        match = _WP_SINK_RE.search(line)
        if not match:
            continue
        sinks.append({"id": int(match.group(1)), "name": match.group(2)})
    return sinks


def select_sink(
    sinks: List[Mapping[str, Any]], mac: str = ""
) -> Optional[Dict[str, Any]]:
    """优先选择与 MAC 匹配的 bluez sink，否则退回第一个 bluez sink。"""
    bluez = [
        dict(item)
        for item in sinks
        if str(item.get("name", "")).startswith("bluez_output.")
    ]
    if not bluez:
        return None
    normalized = normalize_mac(mac).replace(":", "_").lower()
    if normalized:
        for item in bluez:
            if normalized in str(item.get("name", "")).lower():
                return item
    return bluez[0]


@dataclass
class CommandResult:
    returncode: int = 1
    stdout: str = ""
    stderr: str = ""

    @property
    def ok(self) -> bool:
        return self.returncode == 0


class SubprocessRunner:
    """最小 subprocess 封装；超时/缺失命令都转成非零结果。"""

    def run(
        self,
        args: List[str],
        timeout: float = 10.0,
        *,
        input_text: Optional[str] = None,
        env: Optional[Mapping[str, str]] = None,
    ) -> CommandResult:
        try:
            completed = subprocess.run(
                list(args),
                input=input_text,
                capture_output=True,
                text=True,
                timeout=max(0.2, float(timeout)),
                env=dict(env) if env is not None else None,
            )
        except subprocess.TimeoutExpired as exc:
            return CommandResult(
                124,
                _as_text(exc.stdout),
                f"timeout after {float(timeout):.1f}s",
            )
        except FileNotFoundError as exc:
            return CommandResult(127, "", str(exc))
        except OSError as exc:
            return CommandResult(126, "", str(exc))
        return CommandResult(
            int(completed.returncode),
            str(completed.stdout or ""),
            str(completed.stderr or ""),
        )


def _as_text(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value or "")


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class SpeakerManager:
    """自动连接蓝牙音箱并串行播放语音。

    ``config`` 对应 ``edge.speaker``：
    ``enabled``/``device_name``/``device_mac``/``auto_connect``/
    ``connect_interval_sec``/``scan_timeout_sec``/``volume``/
    ``audio_dir``/``tts.*``。所有外部命令都经由 ``runner``。
    """

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        runner: Any = None,
        which: Optional[Callable[[str], Optional[str]]] = None,
        on_event: Optional[Callable[[str, str, str], None]] = None,
        temp_dir: str = "",
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        cfg = dict(config or {})
        tts_cfg = dict(cfg.get("tts") or {})
        self._cfg = cfg
        self._runner = runner if runner is not None else SubprocessRunner()
        self._which = which if which is not None else shutil.which
        self._on_event = on_event
        self._wall_clock = wall_clock

        self.enabled = bool(cfg.get("enabled", False))
        self.auto_connect = bool(cfg.get("auto_connect", True))
        self.device_name = str(cfg.get("device_name", "newmine") or "newmine")
        self.device_mac = normalize_mac(cfg.get("device_mac", ""))
        self.connect_interval_sec = _clamp(
            float(cfg.get("connect_interval_sec", 20.0)), 5.0, 600.0
        )
        self.scan_timeout_sec = _clamp(
            float(cfg.get("scan_timeout_sec", 10.0)), 3.0, 30.0
        )
        self.connect_timeout_sec = _clamp(
            float(cfg.get("connect_timeout_sec", 20.0)), 5.0, 60.0
        )
        self.sink_wait_sec = _clamp(
            float(cfg.get("sink_wait_sec", 8.0)), 1.0, 30.0
        )
        self.volume = int(_clamp(int(cfg.get("volume", 70)), 0, 100))
        self.audio_dir = os.path.abspath(
            os.path.expanduser(str(cfg.get("audio_dir", "~/.local/share/rush-mog/audio")))
        )
        self.temp_dir = os.path.abspath(
            os.path.expanduser(str(temp_dir or os.path.join(tempfile.gettempdir(), "edge_server_audio")))
        )
        self.tts_backend = str(tts_cfg.get("backend", "auto") or "auto").strip().lower()
        self.tts_voice = str(tts_cfg.get("voice", "cmn") or "cmn").strip()
        self.tts_speed = int(_clamp(int(tts_cfg.get("speed", 150)), 80, 300))
        self.tts_pitch = int(_clamp(int(tts_cfg.get("pitch", 50)), 0, 99))
        self.tts_max_chars = int(_clamp(int(tts_cfg.get("max_chars", 200)), 1, 1000))
        # A2DP 从 SUSPENDED 唤醒/功放启动会吞掉开头约 100~400ms；在 TTS
        # 前补一段静音，保证第一个字完整。播放客户端也请求更大缓冲，降低卡顿。
        self.tts_pre_roll_ms = int(
            _clamp(int(tts_cfg.get("pre_roll_ms", 450)), 0, 2000)
        )
        self.playback_latency_ms = int(
            _clamp(int(cfg.get("playback_latency_ms", 250)), 0, 3000)
        )
        self.say_topic = str(cfg.get("say_topic", "/body/say") or "/body/say")
        self.status_topic = str(
            cfg.get("status_topic", "/body/audio_status") or "/body/audio_status"
        )
        self.status_rate_hz = _clamp(
            float(cfg.get("status_rate_hz", 1.0)), 0.1, 10.0
        )

        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._wake = threading.Event()
        self._stop_playback = threading.Event()
        self._queue: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._connect_thread: Optional[threading.Thread] = None
        self._play_thread: Optional[threading.Thread] = None
        self._player: Optional[subprocess.Popen] = None
        self._started = False

        self._state = _STATE_STARTING if self.enabled else _STATE_DISABLED
        self._connected = False
        self._paired = False
        self._trusted = False
        self._display_name = self.device_name
        self._sink_name = ""
        self._sink_id = 0
        self._sink_ready = False
        self._last_error = ""
        self._last_connected_at = 0.0
        self._last_played_at = 0.0
        self._last_text = ""
        self._playing = False
        self._play_error = ""
        self._last_event = ""

        try:
            os.makedirs(self.temp_dir, exist_ok=True)
        except OSError:
            pass

    # ------------------------------------------------------------- lifecycle
    @property
    def state(self) -> str:
        with self._lock:
            return str(self._state)

    def start(self) -> None:
        """启动自动连接与播放线程（可重复调用）。"""
        with self._lock:
            if self._started or not self.enabled:
                if not self.enabled:
                    self._state = _STATE_DISABLED
                return
            self._started = True
            self._closing.clear()
            self._connect_thread = threading.Thread(
                target=self._connect_loop, name="edge-speaker-bt", daemon=True
            )
            self._play_thread = threading.Thread(
                target=self._play_loop, name="edge-speaker-play", daemon=True
            )
            self._connect_thread.start()
            self._play_thread.start()

    def stop(self) -> None:
        """停止线程、清空队列并终止当前播放器。"""
        self._closing.set()
        self._wake.set()
        self.stop_playback()
        with self._lock:
            self._queue.put(None)
        for thread in (self._connect_thread, self._play_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.5)
        with self._lock:
            self._started = False
            self._connect_thread = None
            self._play_thread = None

    # ------------------------------------------------------------- status
    def status(self) -> Dict[str, Any]:
        """线程安全状态快照（供 API/ROS 使用）。"""
        now = float(self._wall_clock())
        with self._lock:
            state = str(self._state)
            connected = bool(self._connected)
            sink_ready = bool(self._sink_ready)
            audio_ready = connected and sink_ready
            if state == _STATE_CONNECTED and not sink_ready:
                audio_status = "degraded"
            elif audio_ready:
                audio_status = "ok"
            elif not self.enabled:
                audio_status = "disabled"
            elif state in {_STATE_SCANNING, _STATE_CONNECTING, _STATE_STARTING}:
                audio_status = "starting"
            elif state == _STATE_ERROR:
                audio_status = "error"
            else:
                audio_status = "unknown"
            return {
                "ok": True,
                "enabled": bool(self.enabled),
                "state": state,
                "status": audio_status,
                "connected": connected,
                "audio_ready": audio_ready,
                "online": bool(audio_ready),
                "device": {
                    "name": str(self._display_name or self.device_name),
                    "mac": str(self.device_mac),
                    "paired": bool(self._paired),
                    "trusted": bool(self._trusted),
                },
                "sink": {
                    "name": str(self._sink_name),
                    "id": int(self._sink_id),
                    "ready": sink_ready,
                    "volume": int(self.volume),
                },
                "tts": {
                    "backend": self._tts_backend_name(),
                    "voice": self.tts_voice,
                    "speed": int(self.tts_speed),
                    "pre_roll_ms": int(self.tts_pre_roll_ms),
                    "playback_latency_ms": int(self.playback_latency_ms),
                },
                "playback": {
                    "playing": bool(self._playing),
                    "queue": int(self._queue.qsize()),
                    "last_text": str(self._last_text),
                    "last_error": str(self._play_error),
                    "last_played_ms": (
                        int(self._last_played_at * 1000.0) if self._last_played_at > 0 else 0
                    ),
                },
                "last_error": str(self._last_error),
                "last_connected_ms": (
                    int(self._last_connected_at * 1000.0)
                    if self._last_connected_at > 0
                    else 0
                ),
                "updated_ms": int(now * 1000.0),
            }

    def summary(self) -> str:
        status = self.status()
        if not status["enabled"]:
            return "speaker=disabled"
        return (
            f"speaker={status['state']}"
            f"{',sink=on' if status['audio_ready'] else ',sink=off'}"
            f"{',playing' if status['playback']['playing'] else ''}"
        )

    # ------------------------------------------------------------- bluetooth card
    def card_name(self) -> str:
        mac = normalize_mac(self.device_mac)
        return "bluez_card." + mac.replace(":", "_") if mac else ""

    def _pactl_cards(self) -> Dict[str, Dict[str, Any]]:
        result = self._run(["pactl", "list", "cards"], timeout=5.0)
        cards: Dict[str, Dict[str, Any]] = {}
        if not result.ok:
            return cards
        current = ""
        for line in result.stdout.splitlines():
            text = line.strip()
            if text.startswith("Name: "):
                current = text[len("Name: "):].strip()
                cards[current] = {"profiles": [], "active_profile": ""}
                continue
            if not current:
                continue
            if text.startswith("Active Profile:"):
                cards[current]["active_profile"] = text.split(":", 1)[1].strip()
                continue
            match = re.match(
                r"^([A-Za-z0-9_.+-]+):\s+.*\(sinks:\s*\d+,\s*sources:\s*\d+",
                text,
            )
            if match:
                cards[current]["profiles"].append(match.group(1))
        return cards

    def card_profiles(self) -> List[str]:
        card = self.card_name()
        if not card:
            return []
        return list((self._pactl_cards().get(card) or {}).get("profiles") or [])

    def active_card_profile(self) -> str:
        card = self.card_name()
        if not card:
            return ""
        return str((self._pactl_cards().get(card) or {}).get("active_profile") or "")

    def set_card_profile(self, profile: str) -> bool:
        card = self.card_name()
        name = str(profile or "").strip()
        if not card or not re.match(r"^[A-Za-z0-9_.+-]{1,64}$", name):
            return False
        result = self._run(
            ["pactl", "set-card-profile", card, name], timeout=8.0
        )
        return bool(result.ok)

    def refresh_sink(self) -> bool:
        with self._lock:
            mac = str(self.device_mac)
        if not mac:
            return False
        return self._refresh_sink(mac, initial=False)

    def play_audio_path(self, path: Any) -> Dict[str, Any]:
        """播放任意已存在音频文件（用于麦克风录音回放）。"""
        text = str(path or "").strip()
        if not text:
            raise ValueError("缺少音频文件")
        resolved = os.path.realpath(os.path.expanduser(text))
        if not os.path.isfile(resolved):
            raise ValueError("音频文件不存在")
        if os.path.splitext(resolved)[1].lower() not in _AUDIO_EXTENSIONS:
            raise ValueError("不支持的音频格式")
        self._require_audio_ready()
        self._enqueue("file", {"path": resolved})
        with self._lock:
            self._last_text = os.path.basename(resolved)
            self._play_error = ""
        return self.status()

    # ------------------------------------------------------------- commands
    def reconnect(self) -> Dict[str, Any]:
        """立即触发一次连接检查（不阻塞 HTTP 线程）。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        self.start()
        self._wake.set()
        return self.status()

    def say(self, text: Any, *, voice: str = "") -> Dict[str, Any]:
        """排队播报一段文本（espeak-ng 等 TTS 后端）。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        cleaned = sanitize_text(text, self.tts_max_chars)
        if not cleaned:
            raise ValueError("播报文本为空")
        target_voice = str(voice or self.tts_voice).strip()
        if not _VOICE_RE.match(target_voice):
            raise ValueError("voice 参数非法")
        self._require_audio_ready()
        self._enqueue(
            "say",
            {
                "text": cleaned,
                "voice": target_voice,
                "speed": int(self.tts_speed),
                "pitch": int(self.tts_pitch),
            },
        )
        with self._lock:
            self._last_text = cleaned
            self._play_error = ""
        return self.status()

    def play_file(self, path: Any) -> Dict[str, Any]:
        """排队播放 ``audio_dir`` 下的音频文件。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        resolved = self.resolve_audio_file(path)
        self._require_audio_ready()
        self._enqueue("file", {"path": resolved})
        with self._lock:
            self._last_text = os.path.basename(resolved)
            self._play_error = ""
        return self.status()

    def _require_audio_ready(self) -> None:
        with self._lock:
            ready = bool(self._connected and self._sink_ready)
            mac = str(self.device_mac)
        if not ready and self._connected:
            # sink 创建比 A2DP 连接稍晚，给一次短暂等待，避免页面点击时误报。
            self._refresh_sink(mac)
            with self._lock:
                ready = bool(self._connected and self._sink_ready)
        if not ready:
            raise SpeakerError("蓝牙音箱未连接或音频输出未就绪")

    def set_volume(self, value: Any) -> Dict[str, Any]:
        """设置 0..100 音量，并立即同步到当前 sink。"""
        try:
            volume = int(str(value).strip())
        except (TypeError, ValueError):
            raise ValueError("volume 必须是 0..100 的整数")
        if volume < 0 or volume > 100:
            raise ValueError("volume 必须在 0..100")
        with self._lock:
            self.volume = volume
        self._apply_volume()
        return self.status()

    def stop_playback(self) -> Dict[str, Any]:
        """清空待播队列并停止当前播放器。"""
        self._stop_playback.set()
        self._clear_queue()
        with self._lock:
            player = self._player
            self._player = None
        if player is not None:
            try:
                player.terminate()
            except Exception:  # noqa: BLE001
                pass
            try:
                player.wait(timeout=1.0)
            except Exception:  # noqa: BLE001
                try:
                    player.kill()
                except Exception:  # noqa: BLE001
                    pass
        with self._lock:
            self._playing = False
            self._play_error = ""
        self._stop_playback.clear()
        return self.status()

    def resolve_audio_file(self, path: Any) -> str:
        """只允许 ``audio_dir`` 内的音频文件，防止任意路径读取。"""
        text = str(path or "").strip()
        if not text:
            raise ValueError("缺少音频文件")
        if os.path.isabs(text):
            candidate = text
        else:
            candidate = os.path.join(self.audio_dir, text)
        candidate = os.path.realpath(os.path.expanduser(candidate))
        root = os.path.realpath(self.audio_dir)
        try:
            inside = os.path.commonpath([candidate, root]) == root
        except ValueError:
            inside = False
        if not inside:
            raise ValueError("音频文件必须位于 audio_dir 内")
        if not os.path.isfile(candidate):
            raise ValueError("音频文件不存在")
        if os.path.splitext(candidate)[1].lower() not in _AUDIO_EXTENSIONS:
            raise ValueError("不支持的音频格式")
        if os.path.getsize(candidate) > 50 * 1024 * 1024:
            raise ValueError("音频文件超过 50MB")
        return candidate

    # ------------------------------------------------------------- bluetooth
    def _connect_loop(self) -> None:
        # 启动后立即尝试一次，之后按 connect_interval_sec 巡检/重连。
        while not self._closing.is_set():
            try:
                self._ensure_connected()
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("speaker connect loop failed: %s", exc)
                self._set_error(f"音箱连接异常: {exc}")
            self._wake.wait(self.connect_interval_sec)
            self._wake.clear()

    def _ensure_connected(self) -> None:
        if self._closing.is_set():
            return
        with self._lock:
            mac = str(self.device_mac)
            connected = bool(self._connected)
            sink_ready = bool(self._sink_ready)
        info = self._bluetooth_info(mac) if mac else {}

        if connected and info.get("connected"):
            if not sink_ready:
                self._refresh_sink(mac)
            return
        if connected and info and not info.get("connected"):
            self._mark_disconnected("蓝牙连接已断开")

        with self._lock:
            mac = str(self.device_mac)
        if not mac:
            mac = self._discover_device()
        if not mac:
            return
        self._connect_device(mac)

    def _discover_device(self) -> str:
        self._set_state(_STATE_SCANNING)
        scan_seconds = int(self.scan_timeout_sec)
        result = self._run(
            ["bluetoothctl", "--timeout", str(scan_seconds), "scan", "on"],
            timeout=scan_seconds + 10.0,
        )
        devices_result = self._run(["bluetoothctl", "devices"], timeout=6.0)
        devices = parse_bluetooth_devices(devices_result.stdout)
        target_name = self.device_name.strip().casefold()
        configured_mac = normalize_mac(self._cfg.get("device_mac", ""))
        match = None
        for device in devices:
            if configured_mac and device["mac"] == configured_mac:
                match = device
                break
            name = device["name"].casefold()
            if target_name and (
                name == target_name
                or (len(target_name) >= 4 and target_name in name)
                or (len(name) >= 4 and name in target_name)
            ):
                match = device
                break
        if match is None:
            detail = _short_output(result.stderr or result.stdout)
            self._set_error(
                f"未发现蓝牙音箱 {self.device_name}"
                + (f"（{detail}）" if detail else "")
            )
            return ""
        with self._lock:
            self.device_mac = match["mac"]
            self._display_name = match["name"] or self.device_name
        self._emit(
            "speaker",
            f"发现蓝牙音箱 {self._display_name} ({self.device_mac})",
            "info",
        )
        return match["mac"]

    def _connect_device(self, mac: str) -> None:
        self._set_state(_STATE_CONNECTING)
        info = self._bluetooth_info(mac)
        if not info:
            self._set_error(f"蓝牙设备不可见: {mac}")
            return
        display_name = info.get("alias") or info.get("name") or self.device_name
        with self._lock:
            self._display_name = str(display_name)
            self._paired = bool(info.get("paired"))
            self._trusted = bool(info.get("trusted"))

        if not info.get("trusted"):
            self._run(["bluetoothctl", "trust", mac], timeout=8.0)
        if not info.get("paired"):
            pair = self._run(
                ["bluetoothctl", "pair", mac], timeout=self.connect_timeout_sec
            )
            info = self._bluetooth_info(mac)
            if not info.get("paired"):
                detail = _short_output(pair.stderr or pair.stdout)
                self._set_error(
                    f"蓝牙配对失败 {display_name}"
                    + (f"（{detail}）" if detail else "")
                )
                return
        if not info.get("trusted"):
            self._run(["bluetoothctl", "trust", mac], timeout=8.0)

        if not info.get("connected"):
            connect = self._run(
                ["bluetoothctl", "connect", mac], timeout=self.connect_timeout_sec
            )
            # bluez 的 connect 在部分适配器上会提前返回，稍等再查一次。
            time.sleep(0.6)
            info = self._bluetooth_info(mac)
            if not info.get("connected"):
                detail = _short_output(connect.stderr or connect.stdout)
                self._set_error(
                    f"蓝牙连接失败 {display_name}"
                    + (f"（{detail}）" if detail else "")
                )
                return

        with self._lock:
            self._connected = True
            self._paired = bool(info.get("paired"))
            self._trusted = bool(info.get("trusted"))
            self._state = _STATE_CONNECTED
            self._last_error = ""
            self._last_connected_at = float(self._wall_clock())
        self._emit("speaker", f"蓝牙音箱已连接 {display_name} ({mac})", "ok")
        self._refresh_sink(mac, initial=True)

    def _refresh_sink(self, mac: str, *, initial: bool = False) -> bool:
        deadline = time.monotonic() + (self.sink_wait_sec if initial else 2.0)
        while True:
            sinks = self._list_sinks()
            sink = select_sink(sinks, mac)
            if sink is not None:
                with self._lock:
                    self._sink_name = str(sink["name"])
                    self._sink_id = int(sink.get("id", 0) or 0)
                    self._sink_ready = True
                self._set_default_sink()
                self._apply_volume()
                if initial:
                    self._emit(
                        "speaker",
                        f"音频输出就绪 {self._sink_name}",
                        "ok",
                    )
                return True
            if self._closing.is_set() or time.monotonic() >= deadline:
                break
            time.sleep(0.5)
        with self._lock:
            self._sink_ready = False
            if self._connected:
                self._last_error = "音箱已连接，但音频 sink 尚未就绪"
        if initial:
            self._emit(
                "speaker",
                "音箱已连接，音频 sink 尚未就绪",
                "warn",
            )
        return False

    def _mark_disconnected(self, reason: str) -> None:
        with self._lock:
            was_connected = self._connected
            self._connected = False
            self._sink_ready = False
            self._sink_name = ""
            self._sink_id = 0
            self._state = _STATE_DISCONNECTED
            self._last_error = str(reason)
        if was_connected:
            self._emit("speaker", f"蓝牙音箱断开：{reason}", "warn")

    def _bluetooth_info(self, mac: str) -> Dict[str, Any]:
        if not mac:
            return {}
        result = self._run(["bluetoothctl", "info", mac], timeout=6.0)
        if not result.ok:
            return {}
        info = parse_bluetooth_info(result.stdout)
        if not info.get("name") and not info.get("alias"):
            return {}
        return info

    def _list_sinks(self) -> List[Dict[str, Any]]:
        if self._which("pactl"):
            result = self._run(["pactl", "list", "short", "sinks"], timeout=5.0)
            if result.ok:
                sinks = parse_sink_list(result.stdout)
                if sinks:
                    return sinks
        if self._which("wpctl"):
            result = self._run(["wpctl", "status"], timeout=5.0)
            if result.ok:
                return parse_wpctl_status(result.stdout)
        return []

    def _set_default_sink(self) -> None:
        with self._lock:
            name = str(self._sink_name)
            sink_id = int(self._sink_id)
        if not name:
            return
        if self._which("pactl"):
            self._run(["pactl", "set-default-sink", name], timeout=5.0)
        elif self._which("wpctl") and sink_id > 0:
            self._run(["wpctl", "set-default", str(sink_id)], timeout=5.0)

    def _apply_volume(self) -> None:
        with self._lock:
            name = str(self._sink_name)
            sink_id = int(self._sink_id)
            volume = int(self.volume)
        if not name and sink_id <= 0:
            return
        if self._which("pactl") and name:
            self._run(
                ["pactl", "set-sink-volume", name, f"{volume}%"], timeout=5.0
            )
        elif self._which("wpctl") and sink_id > 0:
            self._run(
                ["wpctl", "set-volume", str(sink_id), f"{volume / 100.0:.2f}"],
                timeout=5.0,
            )

    # ------------------------------------------------------------- playback
    def _enqueue(self, kind: str, payload: Mapping[str, Any]) -> None:
        with self._lock:
            if self._queue.qsize() >= 20:
                raise SpeakerError("播放队列已满")
            self._queue.put((kind, dict(payload)))
        self.start()

    def _clear_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                break

    def _play_loop(self) -> None:
        while not self._closing.is_set():
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                self._queue.task_done()
                continue
            if self._stop_playback.is_set():
                self._queue.task_done()
                continue
            kind, payload = item
            try:
                if kind == "say":
                    self._play_say(payload)
                elif kind == "file":
                    self._play_path(
                        str(payload.get("path", "")),
                        text=os.path.basename(str(payload.get("path", ""))),
                    )
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("speaker playback failed: %s", exc)
                with self._lock:
                    self._play_error = str(exc)
                self._emit("speaker", f"语音播放失败：{exc}", "error")
            finally:
                with self._lock:
                    self._playing = False
                self._queue.task_done()

    def _play_say(self, payload: Mapping[str, Any]) -> None:
        text = str(payload.get("text", ""))
        voice = str(payload.get("voice", self.tts_voice))
        speed = int(payload.get("speed", self.tts_speed))
        pitch = int(payload.get("pitch", self.tts_pitch))
        with self._lock:
            self._last_text = text
            self._play_error = ""
        path = os.path.join(
            self.temp_dir,
            f"say-{os.getpid()}-{int(time.time() * 1000)}.wav",
        )
        try:
            self._synthesize(text, path, voice=voice, speed=speed, pitch=pitch)
            self._play_path(path, text=text)
        finally:
            try:
                os.remove(path)
            except OSError:
                pass

    def _synthesize(
        self, text: str, path: str, *, voice: str, speed: int, pitch: int
    ) -> None:
        backend = self._tts_backend_name()
        if backend == "none":
            raise SpeakerError("未安装 TTS 后端（espeak-ng）")
        if backend == "edge-tts":
            mp3 = path + ".mp3"
            edge_voice = "zh-CN-XiaoxiaoNeural"
            result = self._run(
                [
                    "edge-tts",
                    "--voice",
                    edge_voice,
                    "--text",
                    text,
                    "--write-media",
                    mp3,
                ],
                timeout=25.0,
            )
            if not result.ok or not os.path.isfile(mp3):
                raise SpeakerError(f"edge-tts 失败: {_short_output(result.stderr)}")
            ffmpeg = self._which("ffmpeg")
            if not ffmpeg:
                raise SpeakerError("edge-tts 需要 ffmpeg 转码")
            convert = self._run(
                [
                    ffmpeg,
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    mp3,
                    "-ar",
                    "44100",
                    "-ac",
                    "2",
                    path,
                ],
                timeout=30.0,
            )
            try:
                os.remove(mp3)
            except OSError:
                pass
            if not convert.ok or not os.path.isfile(path):
                raise SpeakerError(f"音频转码失败: {_short_output(convert.stderr)}")
            _prepend_wav_silence(path, self.tts_pre_roll_ms)
            return

        binary = self._which("espeak-ng" if backend == "espeak-ng" else "espeak")
        if not binary:
            raise SpeakerError("未安装 espeak-ng")
        result = self._run(
            [
                binary,
                "-v",
                voice,
                "-s",
                str(speed),
                "-p",
                str(pitch),
                "-w",
                path,
                text,
            ],
            timeout=20.0,
        )
        if not result.ok or not os.path.isfile(path) or os.path.getsize(path) <= 44:
            raise SpeakerError(f"TTS 合成失败: {_short_output(result.stderr)}")
        _prepend_wav_silence(path, self.tts_pre_roll_ms)

    def _play_path(self, path: str, *, text: str = "") -> None:
        if not os.path.isfile(path):
            raise SpeakerError("音频文件不存在")
        with self._lock:
            sink_name = str(self._sink_name)
            sink_id = int(self._sink_id)
            sink_ready = bool(self._sink_ready)
        if not sink_ready:
            mac = ""
            with self._lock:
                mac = str(self.device_mac)
            self._refresh_sink(mac)
            with self._lock:
                sink_name = str(self._sink_name)
                sink_id = int(self._sink_id)
                sink_ready = bool(self._sink_ready)
        if not sink_ready:
            raise SpeakerError("蓝牙音箱音频输出未就绪")

        command = self._player_command(path, sink_name, sink_id)
        if command is None:
            raise SpeakerError("未找到音频播放器（pw-play/paplay/mpv）")
        with self._lock:
            self._playing = True
            self._play_error = ""
            if text:
                self._last_text = text
        env = dict(os.environ)
        if self.playback_latency_ms > 0:
            # libpulse 客户端请求更大的缓冲，降低蓝牙链路抖动导致的卡顿。
            env["PULSE_LATENCY_MSEC"] = str(int(self.playback_latency_ms))
        try:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                env=env,
            )
        except OSError as exc:
            raise SpeakerError(f"启动播放器失败: {exc}")
        with self._lock:
            self._player = proc
        timeout = _audio_timeout(path)
        try:
            _, stderr = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            _, stderr = proc.communicate()
            with self._lock:
                self._player = None
            raise SpeakerError("播放超时")
        finally:
            with self._lock:
                if self._player is proc:
                    self._player = None
        if proc.returncode != 0:
            raise SpeakerError(f"播放器退出码 {proc.returncode}: {_short_output(stderr or '')}")
        with self._lock:
            self._playing = False
            self._play_error = ""
            self._last_played_at = float(self._wall_clock())

    def _player_command(
        self, path: str, sink_name: str, sink_id: int
    ) -> Optional[List[str]]:
        # Pulse 兼容接口的 --device=<name> 在 PipeWire 0.3.x 上可用；
        # pw-play 0.3.x 的 --target 只接受数字 node id，故放在第二优先。
        paplay = self._which("paplay")
        if paplay and sink_name:
            return [paplay, f"--device={sink_name}", path]
        pw_play = self._which("pw-play")
        if pw_play and sink_id > 0:
            return [pw_play, "--target", str(sink_id), path]
        mpv = self._which("mpv")
        if mpv and sink_name:
            return [
                mpv,
                "--no-video",
                "--really-quiet",
                f"--audio-device=pulse/{sink_name}",
                path,
            ]
        aplay = self._which("aplay")
        if aplay:
            return [aplay, "-q", path]
        ffplay = self._which("ffplay")
        if ffplay:
            return [ffplay, "-nodisp", "-autoexit", "-loglevel", "error", path]
        return None

    # ------------------------------------------------------------- helpers
    def _tts_backend_name(self) -> str:
        if self.tts_backend == "none":
            return "none"
        if self.tts_backend == "edge-tts":
            return "edge-tts"
        if self._which("espeak-ng"):
            return "espeak-ng"
        if self._which("espeak"):
            return "espeak"
        if self.tts_backend == "auto" and self._which("edge-tts"):
            return "edge-tts"
        return "none"

    def _run(
        self,
        args: List[str],
        *,
        timeout: float = 10.0,
    ) -> CommandResult:
        try:
            return self._runner.run(list(args), timeout=timeout)
        except TypeError:
            return self._runner.run(list(args))
        except Exception as exc:  # noqa: BLE001
            return CommandResult(126, "", str(exc))

    def _set_state(self, state: str) -> None:
        with self._lock:
            self._state = str(state)
            if state in {_STATE_SCANNING, _STATE_CONNECTING}:
                self._connected = False
                self._sink_ready = False

    def _set_error(self, message: str) -> None:
        text = sanitize_text(message, 180)
        with self._lock:
            self._state = _STATE_ERROR
            self._last_error = text
        LOGGER.warning("speaker: %s", text)
        self._emit("speaker", text, "error")

    def _emit(self, kind: str, text: str, status: str) -> None:
        with self._lock:
            if text == self._last_event:
                return
            self._last_event = text
        callback = self._on_event
        if callback is None:
            return
        try:
            callback(str(kind), str(text), str(status))
        except Exception:  # noqa: BLE001
            pass


def _prepend_wav_silence(path: str, milliseconds: int) -> None:
    """在 WAV 前补静音；用于 A2DP 唤醒/功放启动的预滚。"""
    ms = int(milliseconds)
    if ms <= 0 or not path.lower().endswith(".wav"):
        return
    try:
        with wave.open(path, "rb") as source:
            params = source.getparams()
            frames = source.readframes(source.getnframes())
        silence_frames = int(params.framerate * ms / 1000.0)
        silence = b"\x00" * (
            silence_frames * params.nchannels * params.sampwidth
        )
        temporary = path + ".preroll.tmp"
        with wave.open(temporary, "wb") as target:
            target.setparams(params)
            target.writeframes(silence)
            target.writeframes(frames)
        os.replace(temporary, path)
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("add TTS pre-roll failed: %s", exc)


def _short_output(value: Any, limit: int = 120) -> str:
    text = _as_text(value).strip().replace("\n", " ")
    return text[:limit]


def _audio_timeout(path: str) -> float:
    """WAV 按时长放宽超时；其他格式给 60s。"""
    duration = 0.0
    if path.lower().endswith(".wav"):
        try:
            import wave

            with wave.open(path, "rb") as handle:
                frames = handle.getnframes()
                rate = handle.getframerate() or 1
                duration = frames / float(rate)
        except Exception:  # noqa: BLE001
            duration = 0.0
    if duration > 0.0:
        return max(5.0, min(180.0, duration * 2.0 + 5.0))
    return 60.0
