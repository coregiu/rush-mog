"""蓝牙音箱管理与语音播放（不依赖 ROS/网页）。

本模块把树莓派上的外部命令收敛到一个可测试的后台服务：

- 自动发现/配对/信任/连接蓝牙音箱（默认 ``newmine``，可切到 ``am510``）；
- 连接后等待 PipeWire/PulseAudio 创建 A2DP sink，并设为默认输出；
- 串行执行 TTS 语音与本地音频文件，避免多个播放器抢占设备；
- 提供线程安全的状态快照，供 HTTP API、ROS 话题和 dashboard 复用。

外部命令只通过参数列表调用（不经过 shell），设备名/文本都按普通参数传递；
单元测试可注入 ``runner``/``which``，不依赖真实蓝牙或声卡。
"""

from __future__ import annotations

import asyncio
import hashlib
import logging
import os
import queue
import random
import re
import shutil
import subprocess
import tempfile
import threading
import time
import wave
from array import array
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional

LOGGER = logging.getLogger("edge_server.speaker")

try:  # edge-tts 是可选依赖；没有就用 CLI/ espeak 回退。
    import edge_tts as _edge_tts_api
except Exception:  # noqa: BLE001
    _edge_tts_api = None

# dashboard 上最常点的几句话，启动后提前合成到缓存。
_PREWARM_TEXTS = (
    "机器人已启动",
    "已到达目标点",
    "请注意避让",
    "任务完成",
)

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

# 可选音箱型号；MAC 留空则由 config 的 models.<id>.device_mac 提供。
_MODEL_DEFAULTS = {
    "newmine": {
        "label": "NewMine",
        "device_name": "newmine",
        "device_mac": "",
        "force_a2dp": False,
    },
    "am510": {
        "label": "HONOR AM510",
        "device_name": "HONOR AM510",
        "device_mac": "",
        "force_a2dp": True,
    },
}
_DEFAULT_MODEL = "newmine"


def _keepalive_pcm(amplitude: int = 128) -> bytes:
    """50ms 48kHz stereo s16le 的低电平 dither。

    不能用全 0：部分蓝牙音箱检测到数字静音会把功放/噪声门关掉，下一条
    TTS 的第一个字就会被吞掉。±128 LSB 约 -53 dBFS，远低于语音，但足以
    让链路保持“有信号”，避免音箱进入静音/省电状态。
    """
    frames = 2400  # 50ms @ 48kHz
    rng = random.Random(0xA510)
    samples = array("h")
    for _ in range(frames * 2):
        samples.append(rng.randint(-amplitude, amplitude))
    return samples.tobytes()


def normalize_model(value: Any) -> str:
    """把用户输入/配置值归一化为型号 id。"""
    text = str(value or "").strip().casefold()
    if text in {"newmine", "new mine"}:
        return "newmine"
    if text in {"am510", "honor am510", "hono am510"}:
        return "am510"
    return ""


def build_speaker_models(cfg: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    """合并内置型号与 ``edge.speaker.models`` 配置。"""
    models = {key: dict(value) for key, value in _MODEL_DEFAULTS.items()}
    raw = cfg.get("models") or {}
    if not isinstance(raw, Mapping):
        return models
    for key, item in raw.items():
        model = normalize_model(key) or str(key or "").strip().casefold()
        if not model:
            continue
        entry = dict(models.get(model) or {})
        entry.setdefault("label", str(key))
        entry.setdefault("device_name", str(key))
        entry.setdefault("device_mac", "")
        entry.setdefault("force_a2dp", False)
        if isinstance(item, Mapping):
            if item.get("label"):
                entry["label"] = str(item["label"]).strip()
            if item.get("device_name"):
                entry["device_name"] = str(item["device_name"]).strip()
            if "device_mac" in item:
                entry["device_mac"] = normalize_mac(item.get("device_mac"))
            if "force_a2dp" in item:
                entry["force_a2dp"] = bool(item.get("force_a2dp"))
        models[model] = entry
    return models


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
    sinks: List[Mapping[str, Any]],
    mac: str = "",
    *,
    require_a2dp: bool = False,
) -> Optional[Dict[str, Any]]:
    """优先选择与 MAC 匹配的 A2DP bluez sink，否则退回第一个 bluez sink。"""
    bluez = [
        dict(item)
        for item in sinks
        if str(item.get("name", "")).startswith("bluez_output.")
    ]
    if not bluez:
        return None
    normalized = normalize_mac(mac).replace(":", "_").lower()
    matched = bluez
    if normalized:
        matched = [
            item
            for item in bluez
            if normalized in str(item.get("name", "")).lower()
        ]
        if not matched:
            # 多台音箱同时在线时，绝不能把另一台的 sink 当成当前型号。
            return None
    a2dp = [
        item for item in matched if "a2dp-sink" in str(item.get("name", ""))
    ]
    if a2dp:
        return a2dp[0]
    if require_a2dp:
        return None
    return matched[0]


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
    ``enabled``/``model``/``models``/``device_name``/``device_aliases``/
    ``device_mac``/``auto_connect``/``connect_interval_sec``/
    ``scan_timeout_sec``/``volume``/``audio_dir``/``tts.*``。
    所有外部命令都经由 ``runner``。
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
        self.keepalive = bool(cfg.get("keepalive", True))
        self.models = build_speaker_models(cfg)
        explicit_model = str(cfg.get("model", "") or "").strip()
        legacy_name = str(cfg.get("device_name", "") or "").strip()
        legacy_mac = normalize_mac(cfg.get("device_mac", ""))
        if explicit_model:
            self.model = normalize_model(explicit_model) or explicit_model.casefold()
        else:
            # 兼容旧配置：没有 model 时按 device_name 推断。
            self.model = normalize_model(legacy_name) or _DEFAULT_MODEL
        if self.model not in self.models:
            LOGGER.warning(
                "未知音箱型号 %s，回退 %s", explicit_model or self.model, _DEFAULT_MODEL
            )
            self.model = _DEFAULT_MODEL
        entry = self.models[self.model]
        if explicit_model:
            self.device_name = str(entry.get("device_name") or self.model).strip()
            self.device_mac = normalize_mac(entry.get("device_mac", ""))
        else:
            self.device_name = legacy_name or str(
                entry.get("device_name") or self.model
            ).strip()
            self.device_mac = legacy_mac or normalize_mac(entry.get("device_mac", ""))
        self.force_a2dp = bool(entry.get("force_a2dp", False))
        alias_cfg = cfg.get("device_aliases")
        if alias_cfg is None:
            alias_cfg = ()
        elif isinstance(alias_cfg, str):
            alias_cfg = [alias_cfg]
        self.device_aliases = tuple(
            name for name in (str(item).strip() for item in alias_cfg) if name
        )
        self.device_names = tuple(
            dict.fromkeys(
                name for name in (self.device_name, *self.device_aliases) if name
            )
        )
        self.connect_interval_sec = _clamp(
            float(cfg.get("connect_interval_sec", 20.0)), 5.0, 600.0
        )
        self.scan_timeout_sec = _clamp(
            float(cfg.get("scan_timeout_sec", 10.0)), 3.0, 30.0
        )
        self.connect_timeout_sec = _clamp(
            float(cfg.get("connect_timeout_sec", 20.0)), 5.0, 60.0
        )
        self.recover_wireplumber = bool(cfg.get("recover_wireplumber", True))
        self.recover_interval_sec = _clamp(
            float(cfg.get("recover_interval_sec", 180.0)), 30.0, 1800.0
        )
        # 重启 WirePlumber 后要等它重新注册 A2DP endpoint，太短会报
        # br-connection-profile-unavailable / Protocol not available。
        self.recover_wait_sec = _clamp(
            float(cfg.get("recover_wait_sec", 15.0)), 1.0, 30.0
        )
        self.sink_wait_sec = _clamp(
            float(cfg.get("sink_wait_sec", 8.0)), 1.0, 30.0
        )
        self.volume = int(_clamp(int(cfg.get("volume", 70)), 0, 100))
        self.audio_dir = os.path.abspath(
            os.path.expanduser(str(cfg.get("audio_dir", "~/.local/share/rush-mog/audio")))
        )
        self.tts_cache_dir = os.path.join(self.audio_dir, "tts_cache")
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
        self._prepare_lock = threading.Lock()
        self._preparing: set = set()
        self._closing = threading.Event()
        self._wake = threading.Event()
        self._stop_playback = threading.Event()
        self._queue: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._connect_thread: Optional[threading.Thread] = None
        self._play_thread: Optional[threading.Thread] = None
        self._synth_thread: Optional[threading.Thread] = None
        self._prewarm_thread: Optional[threading.Thread] = None
        self._keepalive_thread: Optional[threading.Thread] = None
        self._player: Optional[subprocess.Popen] = None
        self._keepalive_proc: Optional[subprocess.Popen] = None
        self._keepalive_chunk = _keepalive_pcm()
        self._synth_queue: "queue.Queue[Optional[dict]]" = queue.Queue()
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
        self._last_profile_recover_at = 0.0
        self._last_played_at = 0.0
        self._last_text = ""
        self._playing = False
        self._play_error = ""
        self._last_event = ""

        try:
            os.makedirs(self.temp_dir, exist_ok=True)
            os.makedirs(self.tts_cache_dir, exist_ok=True)
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
            self._synth_thread = threading.Thread(
                target=self._synth_loop, name="edge-speaker-tts", daemon=True
            )
            self._prewarm_thread = threading.Thread(
                target=self._prewarm_loop,
                name="edge-speaker-prewarm",
                daemon=True,
            )
            self._keepalive_thread = threading.Thread(
                target=self._keepalive_loop,
                name="edge-speaker-keepalive",
                daemon=True,
            )
            self._connect_thread.start()
            self._play_thread.start()
            self._synth_thread.start()
            self._prewarm_thread.start()
            self._keepalive_thread.start()

    def stop(self) -> None:
        """停止线程、清空队列并终止当前播放器。"""
        self._closing.set()
        self._wake.set()
        self.stop_playback()
        self._stop_keepalive()
        with self._lock:
            self._queue.put(None)
            self._synth_queue.put(None)
        for thread in (
            self._connect_thread,
            self._play_thread,
            self._synth_thread,
            self._prewarm_thread,
            self._keepalive_thread,
        ):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.5)
        with self._lock:
            self._started = False
            self._connect_thread = None
            self._play_thread = None
            self._synth_thread = None
            self._prewarm_thread = None
            self._keepalive_thread = None

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
                "keepalive": bool(self.keepalive),
                "model": str(self.model),
                "models": self.available_models(),
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
    def available_models(self) -> List[Dict[str, Any]]:
        """返回可选音箱型号（供 dashboard/API）。"""
        with self._lock:
            models = dict(self.models)
        return [
            {
                "id": key,
                "label": str(item.get("label") or key),
                "device_name": str(item.get("device_name") or key),
                "force_a2dp": bool(item.get("force_a2dp", False)),
            }
            for key, item in models.items()
        ]

    def select_model(self, model: Any) -> Dict[str, Any]:
        """切换音箱型号并立即重连；切换只在本次运行内生效。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        key = normalize_model(model) or str(model or "").strip().casefold()
        with self._lock:
            if key not in self.models:
                raise ValueError(f"未知音箱型号: {model}")
            if key == self.model:
                return self.status()
            entry = dict(self.models[key])
            old_mac = str(self.device_mac)
            was_connected = bool(self._connected)
        self.stop_playback()
        self._stop_keepalive()
        if old_mac and was_connected:
            self._run(["bluetoothctl", "disconnect", old_mac], timeout=8.0)
        if self.recover_wireplumber:
            # 切型号时清掉旧设备的 card/profile 残留，避免新设备
            # 报 br-connection-profile-unavailable。
            self._run(
                ["systemctl", "--user", "restart", "wireplumber"],
                timeout=15.0,
            )
            time.sleep(1.0)
            self._last_profile_recover_at = time.monotonic()
        with self._lock:
            self.model = key
            self.device_name = str(entry.get("device_name") or key).strip()
            self.device_mac = normalize_mac(entry.get("device_mac", ""))
            self.device_aliases = ()
            self.device_names = (self.device_name,)
            self.force_a2dp = bool(entry.get("force_a2dp", False))
            self._display_name = self.device_name
            self._connected = False
            self._paired = False
            self._trusted = False
            self._sink_name = ""
            self._sink_id = 0
            self._sink_ready = False
            self._last_error = ""
            self._state = _STATE_STARTING if self.enabled else _STATE_DISABLED
        self._emit("speaker", f"已切换音箱型号: {self.device_name}", "info")
        self.start()
        self._wake.set()
        return self.status()

    def reconnect(self) -> Dict[str, Any]:
        """立即触发一次连接检查（不阻塞 HTTP 线程）。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        self.start()
        self._wake.set()
        return self.status()

    def prepare(self, text: Any, *, voice: str = "") -> Dict[str, Any]:
        """把一段文字提前合成到缓存；不播放，供 dashboard 输入时调用。"""
        if not self.enabled:
            raise SpeakerError("音箱功能未启用")
        cleaned = sanitize_text(text, self.tts_max_chars)
        if not cleaned:
            raise ValueError("播报文本为空")
        target_voice = str(voice or self.tts_voice).strip()
        if not _VOICE_RE.match(target_voice):
            raise ValueError("voice 参数非法")
        if self._tts_backend_name() == "none":
            raise SpeakerError("未安装 TTS 后端")
        speed = int(self.tts_speed)
        pitch = int(self.tts_pitch)
        cache = self._tts_cache_path(cleaned, target_voice, speed, pitch)
        if self._cache_valid(cache):
            return {
                "ok": True,
                "cached": True,
                "message": "已缓存",
                "text": cleaned,
            }
        with self._prepare_lock:
            if cache in self._preparing:
                return {
                    "ok": True,
                    "cached": False,
                    "message": "正在预合成",
                    "text": cleaned,
                }
            self._preparing.add(cache)

        def worker() -> None:
            path = os.path.join(
                self.temp_dir,
                f"prepare-{os.getpid()}-{time.time_ns()}.wav",
            )
            try:
                self._synthesize(
                    cleaned,
                    path,
                    voice=target_voice,
                    speed=speed,
                    pitch=pitch,
                )
                if self._cache_valid(path):
                    self._store_cache(path, cache)
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("TTS prepare failed: %s", exc)
            finally:
                try:
                    os.remove(path)
                except OSError:
                    pass
                with self._prepare_lock:
                    self._preparing.discard(cache)

        threading.Thread(
            target=worker, name="edge-speaker-prepare", daemon=True
        ).start()
        return {
            "ok": True,
            "cached": False,
            "message": "已开始预合成",
            "text": cleaned,
        }

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
        job = {
            "text": cleaned,
            "voice": target_voice,
            "speed": int(self.tts_speed),
            "pitch": int(self.tts_pitch),
            "event": threading.Event(),
            "path": "",
            "error": "",
            "cancelled": False,
            "cached": False,
        }
        self._enqueue_say(job)
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
            if sink_ready and not self._sink_alive():
                with self._lock:
                    self._sink_ready = False
                    self._sink_name = ""
                    self._sink_id = 0
                    self._last_error = "音频 sink 已消失"
                self._stop_keepalive()
                self._emit(
                    "speaker",
                    "音频 sink 已消失，重新协商 A2DP",
                    "warn",
                )
                sink_ready = False
            if not sink_ready:
                # BlueZ 有时显示 connected，但 A2DP sink 没起来（常见于
                # br-connection-profile-unavailable 之后）。只等 sink 不会自愈，
                # 再发一次 connect 让 BlueZ 重新协商 A2DP profile。
                self._refresh_sink(mac)
                with self._lock:
                    sink_ready = bool(self._sink_ready)
                if not sink_ready:
                    self._emit(
                        "speaker",
                        "蓝牙已连接但 A2DP sink 未就绪，重新协商 profile",
                        "warn",
                    )
                    self._run(
                        ["bluetoothctl", "connect", mac],
                        timeout=self.connect_timeout_sec,
                    )
                    self._refresh_sink(mac, initial=True)
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
        target_names = [name.casefold() for name in self.device_names]
        configured_mac = normalize_mac(self.device_mac)
        match = None
        for device in devices:
            if configured_mac and device["mac"] == configured_mac:
                match = device
                break
        if match is None:
            # 按 device_name → device_aliases 的优先级做模糊匹配。
            for target_name in target_names:
                for device in devices:
                    name = device["name"].casefold()
                    if target_name and (
                        name == target_name
                        or (len(target_name) >= 4 and target_name in name)
                        or (len(name) >= 4 and name in target_name)
                    ):
                        match = device
                        break
                if match is not None:
                    break
        if match is None:
            detail = _short_output(result.stderr or result.stdout)
            configured = " / ".join(self.device_names)
            self._set_error(
                f"未发现蓝牙音箱 {configured}"
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
                # BlueZ 已连过该设备但 A2DP endpoint 未注册时，重试 connect
                # 往往仍报 profile-unavailable；重启 WirePlumber 可恢复。
                if self._recover_audio_profile(detail):
                    connect = self._run(
                        ["bluetoothctl", "connect", mac],
                        timeout=self.connect_timeout_sec,
                    )
                    time.sleep(0.6)
                    info = self._bluetooth_info(mac)
                    if not info.get("connected"):
                        detail = _short_output(connect.stderr or connect.stdout)
                if not info.get("connected"):
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

    def _recover_audio_profile(self, detail: str) -> bool:
        """profile-unavailable 时重启一次用户级 WirePlumber（带冷却）。"""
        if not self.recover_wireplumber:
            return False
        text = str(detail or "").lower()
        if "profile-unavailable" not in text and "profile unavailable" not in text:
            return False
        now = time.monotonic()
        if (now - self._last_profile_recover_at) < self.recover_interval_sec:
            return False
        self._last_profile_recover_at = now
        self._emit(
            "speaker",
            "检测到 A2DP profile 不可用，重启 WirePlumber 后重试",
            "warn",
        )
        self._run(
            ["systemctl", "--user", "restart", "wireplumber"],
            timeout=15.0,
        )
        time.sleep(self.recover_wait_sec)
        return True

    def _force_a2dp_profile(self, mac: str) -> None:
        """把 headset 类设备的 card profile 从 HFP 切回 A2DP。

        AM510 的 Class 是 audio-headset，PipeWire 有时会把初始 profile 选成
        ``headset-head-unit``，此时只有 HFP sink，没有 A2DP sink。
        """
        card = "bluez_card." + mac.replace(":", "_") if mac else ""
        if not card:
            return
        self._run(
            ["pactl", "set-card-profile", card, "a2dp-sink"], timeout=6.0
        )

    def _refresh_sink(self, mac: str, *, initial: bool = False) -> bool:
        deadline = time.monotonic() + (self.sink_wait_sec if initial else 2.0)
        profile_tried = False
        while True:
            sinks = self._list_sinks()
            now = time.monotonic()
            # 只有显式 force_a2dp 的型号（AM510）才要求 A2DP；NewMine 保持
            # 原来的“有 bluez sink 就用”，避免反复切 profile 导致断连。
            sink = select_sink(sinks, mac, require_a2dp=self.force_a2dp)
            if sink is None and self.force_a2dp and not profile_tried:
                self._force_a2dp_profile(mac)
                profile_tried = True
            if sink is None and now >= deadline:
                sink = select_sink(sinks, mac)
            if sink is not None:
                with self._lock:
                    self._sink_name = str(sink["name"])
                    self._sink_id = int(sink.get("id", 0) or 0)
                    self._sink_ready = True
                    self._last_error = ""
                self._set_default_sink()
                self._apply_volume()
                if initial:
                    self._emit(
                        "speaker",
                        f"音频输出就绪 {self._sink_name}",
                        "ok",
                    )
                return True
            if self._closing.is_set():
                break
            if now >= deadline:
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
        self._stop_keepalive()
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

    def _sink_alive(self) -> bool:
        """当前记录的 A2DP sink 是否仍在 PipeWire/PulseAudio 里。"""
        with self._lock:
            name = str(self._sink_name)
            ready = bool(self._sink_ready)
        if not ready or not name:
            return False
        return any(
            str(item.get("name", "")) == name for item in self._list_sinks()
        )

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

    # ------------------------------------------------------------- keepalive
    def _keepalive_loop(self) -> None:
        """持续送极低电平 dither，既防休眠也防音箱对数字静音自动 mute。"""
        while not self._closing.is_set():
            if not self.keepalive:
                self._stop_keepalive()
                self._closing.wait(2.0)
                continue
            with self._lock:
                sink = str(self._sink_name)
                ready = bool(self._connected and self._sink_ready)
            if not ready or not sink:
                self._stop_keepalive()
                self._closing.wait(2.0)
                continue
            proc = self._keepalive_proc
            if (
                proc is None
                or proc.poll() is not None
                or proc.stdin is None
            ):
                self._start_keepalive(sink)
                proc = self._keepalive_proc
            if proc is None or proc.stdin is None:
                self._closing.wait(1.0)
                continue
            try:
                proc.stdin.write(self._keepalive_chunk)
                proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                self._stop_keepalive()
                self._closing.wait(0.5)
                continue
            # 50ms 的 PCM 每 40ms 送一次，留 20% 余量，避免欠载。
            self._closing.wait(0.04)
        self._stop_keepalive()

    def _start_keepalive(self, sink: str) -> None:
        pacat = self._which("pacat")
        if not pacat:
            return
        self._stop_keepalive()
        try:
            proc = subprocess.Popen(
                [
                    pacat,
                    "--playback",
                    f"--device={sink}",
                    "--format=s16le",
                    "--rate=48000",
                    "--channels=2",
                    "--latency-msec=1000",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError:
            return
        with self._lock:
            self._keepalive_proc = proc

    def _stop_keepalive(self) -> None:
        with self._lock:
            proc = self._keepalive_proc
            self._keepalive_proc = None
        if proc is None:
            return
        if proc.stdin is not None:
            try:
                proc.stdin.close()
            except (BrokenPipeError, OSError, ValueError):
                pass
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()

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

    def _enqueue_say(self, job: dict) -> None:
        with self._lock:
            if self._queue.qsize() >= 20:
                raise SpeakerError("播放队列已满")
            self._queue.put(("say", job))
            self._synth_queue.put(job)
        self.start()

    def _cancel_job(self, job: Mapping[str, Any]) -> None:
        job["cancelled"] = True  # type: ignore[index]
        event = job.get("event")
        if event is not None:
            event.set()
        path = str(job.get("path") or "")
        if path:
            try:
                os.remove(path)
            except OSError:
                pass

    def _clear_queue(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            if item is not None:
                kind, payload = item
                if kind == "say":
                    self._cancel_job(payload)
            self._queue.task_done()
        while True:
            try:
                job = self._synth_queue.get_nowait()
            except queue.Empty:
                break
            if job is not None:
                self._cancel_job(job)
            self._synth_queue.task_done()

    def _tts_cache_path(
        self, text: str, voice: str, speed: int, pitch: int
    ) -> str:
        backend = self._tts_backend_name()
        raw = f"{backend}|{voice}|{speed}|{pitch}|{text}".encode("utf-8")
        return os.path.join(
            self.tts_cache_dir, hashlib.sha1(raw).hexdigest() + ".wav"
        )

    @staticmethod
    def _cache_valid(path: str) -> bool:
        try:
            return os.path.isfile(path) and os.path.getsize(path) > 44
        except OSError:
            return False

    def _store_cache(self, source: str, cache: str) -> None:
        try:
            os.makedirs(self.tts_cache_dir, exist_ok=True)
            temporary = cache + ".tmp"
            shutil.copyfile(source, temporary)
            os.replace(temporary, cache)
            # 任意文本缓存会增长；保留最近 200 条即可。
            entries = [
                os.path.join(self.tts_cache_dir, name)
                for name in os.listdir(self.tts_cache_dir)
                if name.endswith(".wav")
            ]
            if len(entries) > 200:
                entries.sort(key=os.path.getmtime)
                for old in entries[:-200]:
                    try:
                        os.remove(old)
                    except OSError:
                        pass
        except OSError as exc:
            LOGGER.warning("TTS cache write failed: %s", exc)

    def _prewarm_loop(self) -> None:
        """启动后把 dashboard 最常点的几句提前合成到缓存。"""
        if self._tts_backend_name() == "none":
            return
        for text in _PREWARM_TEXTS:
            if self._closing.is_set():
                return
            voice = self.tts_voice
            speed = int(self.tts_speed)
            pitch = int(self.tts_pitch)
            cache = self._tts_cache_path(text, voice, speed, pitch)
            if self._cache_valid(cache):
                continue
            path = os.path.join(
                self.temp_dir, f"prewarm-{os.getpid()}-{time.time_ns()}.wav"
            )
            try:
                self._synthesize(
                    text, path, voice=voice, speed=speed, pitch=pitch
                )
                if self._cache_valid(path):
                    self._store_cache(path, cache)
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("TTS prewarm failed (%s): %s", text, exc)
            finally:
                try:
                    os.remove(path)
                except OSError:
                    pass

    def _synth_loop(self) -> None:
        """后台预合成 TTS；播放当前一条时，下一条已经在合成。"""
        while not self._closing.is_set():
            try:
                job = self._synth_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                self._synth_queue.task_done()
                break
            path = ""
            try:
                if job.get("cancelled"):
                    continue
                text = str(job.get("text", ""))
                voice = str(job.get("voice") or self.tts_voice)
                speed = int(job.get("speed", self.tts_speed))
                pitch = int(job.get("pitch", self.tts_pitch))
                cache = self._tts_cache_path(text, voice, speed, pitch)
                if self._cache_valid(cache):
                    job["path"] = cache
                    job["cached"] = True
                    continue
                path = os.path.join(
                    self.temp_dir,
                    f"say-{os.getpid()}-{time.time_ns()}.wav",
                )
                self._synthesize(
                    text, path, voice=voice, speed=speed, pitch=pitch
                )
                if job.get("cancelled"):
                    try:
                        os.remove(path)
                    except OSError:
                        pass
                else:
                    job["path"] = path
                    job["cached"] = False
                    if self._cache_valid(path):
                        self._store_cache(path, cache)
            except Exception as exc:  # noqa: BLE001
                job["error"] = str(exc)
                if path:
                    try:
                        os.remove(path)
                    except OSError:
                        pass
            finally:
                job["event"].set()
                self._synth_queue.task_done()

    def _play_say_job(self, job: Mapping[str, Any]) -> None:
        event = job.get("event")
        if event is not None and not event.wait(timeout=90.0):
            raise SpeakerError("TTS 合成超时")
        if job.get("cancelled"):
            return
        error = str(job.get("error") or "")
        if error:
            raise SpeakerError(error)
        path = str(job.get("path") or "")
        try:
            if not path or not os.path.isfile(path):
                raise SpeakerError("TTS 合成结果不存在")
            self._play_path(path, text=str(job.get("text", "")))
        finally:
            if not job.get("cached"):
                try:
                    os.remove(path)
                except OSError:
                    pass

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
                if item[0] == "say":
                    self._cancel_job(item[1])
                self._queue.task_done()
                continue
            kind, payload = item
            try:
                if kind == "say":
                    self._play_say_job(payload)
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

    async def _edge_tts_write(self, text: str, voice: str, path: str) -> None:
        communicate = _edge_tts_api.Communicate(text, voice)
        with open(path, "wb") as handle:
            async for chunk in communicate.stream():
                if chunk.get("type") == "audio" and chunk.get("data"):
                    handle.write(chunk["data"])

    def _synthesize(
        self, text: str, path: str, *, voice: str, speed: int, pitch: int
    ) -> None:
        backend = self._tts_backend_name()
        if backend == "none":
            raise SpeakerError("未安装 TTS 后端（espeak-ng）")
        if backend == "edge-tts":
            try:
                mp3 = path + ".mp3"
                edge_voice = "zh-CN-XiaoxiaoNeural"
                if _edge_tts_api is not None:
                    # 进程内 API 比每次拉起 edge-tts CLI 少约 1 秒启动开销。
                    try:
                        asyncio.run(
                            self._edge_tts_write(text, edge_voice, mp3)
                        )
                    except Exception as exc:  # noqa: BLE001
                        raise SpeakerError(f"edge-tts 失败: {exc}")
                    if not os.path.isfile(mp3) or os.path.getsize(mp3) <= 0:
                        raise SpeakerError("edge-tts 输出为空")
                else:
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
                        raise SpeakerError(
                            f"edge-tts 失败: {_short_output(result.stderr)}"
                        )
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
                        "48000",
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
                    raise SpeakerError(
                        f"音频转码失败: {_short_output(convert.stderr)}"
                    )
                _prepend_wav_silence(path, self.tts_pre_roll_ms)
                return
            except SpeakerError as exc:
                # 没网/微软接口不可用时退回本地 espeak，至少还能播报。
                LOGGER.warning("edge-tts failed, fallback to espeak-ng: %s", exc)
                backend = "espeak-ng"

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
        # espeak 默认 22050Hz mono；提前转成 A2DP sink 的 48kHz stereo，
        # 避免播放时实时做非整数倍重采样造成卡顿/失真。
        ffmpeg = self._which("ffmpeg")
        if ffmpeg:
            converted = path + ".48k.wav"
            convert = self._run(
                [
                    ffmpeg,
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    path,
                    "-ar",
                    "48000",
                    "-ac",
                    "2",
                    converted,
                ],
                timeout=20.0,
            )
            if convert.ok and os.path.isfile(converted):
                os.replace(converted, path)
            else:
                try:
                    os.remove(converted)
                except OSError:
                    pass
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
            detail = _short_output(stderr or "")
            if not self._sink_alive():
                # 播放失败且 sink 不在了：立即失效状态并唤醒重连，
                # 不能只记一条 play_error 继续假装在线。
                with self._lock:
                    self._sink_ready = False
                    self._sink_name = ""
                    self._sink_id = 0
                    self._last_error = "播放失败，音频 sink 已消失"
                self._stop_keepalive()
                self._wake.set()
            raise SpeakerError(
                f"播放器退出码 {proc.returncode}: {detail}"
            )
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
