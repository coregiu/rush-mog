"""newmine 蓝牙耳机麦克风（HFP）采集与录音管理。

蓝牙音箱同时是 headset：A2DP 只提供播放，麦克风要走 HFP/HSP。本模块负责：

- 录音前把 BlueZ/PipeWire 的 card profile 从 A2DP 切到 HFP；
- 等待 ``bluez_input.<MAC>...`` 输入源出现；
- 用 ``parec`` 读原始 PCM，边写 WAV 边回调（供 ROS ``AudioData`` 发布）；
- 停止后恢复原 A2DP profile，让语音播报恢复高音质输出；
- 录音文件列表/下载/回放，文件名严格限制在录音目录内。

不依赖 ROS，单元测试可注入 ``runner``/``which`` 和 fake speaker。
"""

from __future__ import annotations

import audioop
import logging
import os
import re
import shutil
import subprocess
import threading
import time
import wave
from typing import Any, Callable, Dict, List, Mapping, Optional

from .speaker import SpeakerError, SubprocessRunner, normalize_mac, parse_sink_list

LOGGER = logging.getLogger("edge_server.microphone")

_RECORDING_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,79}\.wav$", re.IGNORECASE)


def select_mic_source(
    sources: List[Mapping[str, Any]], mac: str = ""
) -> Optional[Dict[str, Any]]:
    """从 ``pactl list short sources`` 中选 bluez_input 输入源。"""
    normalized = normalize_mac(mac).replace(":", "_").lower()
    candidates: List[Dict[str, Any]] = []
    for item in sources:
        name = str(item.get("name", ""))
        if not name.startswith("bluez_input."):
            continue
        if normalized and normalized not in name.lower():
            continue
        candidates.append(dict(item))
    if not candidates:
        return None
    for item in candidates:
        if "headset-head-unit" in str(item.get("name", "")):
            return item
    return candidates[0]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _wav_duration_ms(path: str) -> int:
    try:
        with wave.open(path, "rb") as handle:
            frames = handle.getnframes()
            rate = handle.getframerate() or 1
            return int(frames * 1000.0 / float(rate))
    except Exception:  # noqa: BLE001
        return 0


class MicrophoneManager:
    """HFP 麦克风录音；同一时间只允许一个录音会话。"""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        speaker: Any,
        runner: Any = None,
        which: Optional[Callable[[str], Optional[str]]] = None,
        on_event: Optional[Callable[[str, str, str], None]] = None,
        on_audio_chunk: Optional[Callable[[bytes, int, int], None]] = None,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        cfg = dict(config or {})
        self.enabled = bool(cfg.get("enabled", True))
        self.record_dir = os.path.abspath(
            os.path.expanduser(
                str(
                    cfg.get("record_dir", "")
                    or os.path.join(
                        os.path.expanduser("~/.local/share/rush-mog/audio"), "mic"
                    )
                )
            )
        )
        self.sample_rate = int(
            _clamp(int(cfg.get("sample_rate", 8000)), 8000, 48000)
        )
        self.channels = int(_clamp(int(cfg.get("channels", 1)), 1, 2))
        self.chunk_ms = int(_clamp(int(cfg.get("chunk_ms", 100)), 20, 1000))
        self.max_duration_sec = _clamp(
            float(cfg.get("max_duration_sec", 60.0)), 1.0, 600.0
        )
        self.volume = int(_clamp(int(cfg.get("volume", 100)), 0, 100))
        # HFP/CVSD 麦克风通常偏小；软件增益在写 WAV/发布 ROS 前应用。
        self.gain = float(_clamp(float(cfg.get("gain", 3.0)), 1.0, 8.0))
        self.topic = str(cfg.get("topic", "/edge/audio") or "/edge/audio")
        self.status_topic = str(
            cfg.get("status_topic", "/body/mic_status") or "/body/mic_status"
        )
        self.status_rate_hz = _clamp(
            float(cfg.get("status_rate_hz", 1.0)), 0.1, 10.0
        )
        self.auto_restore_profile = bool(cfg.get("auto_restore_profile", True))
        self.capability_interval_sec = _clamp(
            float(cfg.get("capability_interval_sec", 10.0)), 2.0, 120.0
        )
        self.profile_preference = str(
            cfg.get("profile", "headset-head-unit") or "headset-head-unit"
        )

        self._speaker = speaker
        self._runner = runner if runner is not None else SubprocessRunner()
        self._which = which if which is not None else shutil.which
        self._on_event = on_event
        self._on_audio_chunk = on_audio_chunk
        self._wall_clock = wall_clock

        self._lock = threading.RLock()
        self._recording = False
        self._supported = False
        self._available = False
        self._source = ""
        self._file = ""
        self._started_at = 0.0
        self._duration_limit = self.max_duration_sec
        self._last_file = ""
        self._last_duration = 0.0
        self._error = ""
        self._bytes = 0
        self._thread: Optional[threading.Thread] = None
        self._proc: Optional[subprocess.Popen] = None
        self._wave: Optional[wave.Wave_write] = None
        self._stop_event = threading.Event()
        self._card = ""
        self._restore_profile = ""
        self._finalized = True
        self._last_capability_check = 0.0
        self._last_event = ""

        try:
            os.makedirs(self.record_dir, exist_ok=True)
        except OSError:
            pass
        # 服务重启后仍让 dashboard 能下载/回放最近一条录音。
        try:
            candidates = [
                os.path.join(self.record_dir, name)
                for name in os.listdir(self.record_dir)
                if name.lower().endswith(".wav")
            ]
            if candidates:
                latest = max(candidates, key=os.path.getmtime)
                self._last_file = latest
                self._last_duration = _wav_duration_ms(latest) / 1000.0
        except OSError:
            pass

    # ------------------------------------------------------------- status
    @property
    def supported(self) -> bool:
        with self._lock:
            return bool(self._supported)

    def refresh_capabilities(self, *, force: bool = False) -> bool:
        """检测 card 是否提供 HFP/HSP（带节流，避免高频 pactl）。"""
        now = time.monotonic()
        with self._lock:
            if not self.enabled:
                self._supported = False
                self._available = False
                return False
            if not force and (now - self._last_capability_check) < self.capability_interval_sec:
                return bool(self._supported)
            self._last_capability_check = now
        speaker_status = self._speaker.status()
        if not speaker_status.get("connected"):
            with self._lock:
                self._supported = False
                self._available = False
            return False
        profiles = [
            str(item) for item in (self._speaker.card_profiles() or [])
        ]
        supported = any(
            item.startswith("headset-head-unit") for item in profiles
        )
        with self._lock:
            self._supported = bool(supported)
            self._available = bool(supported and speaker_status.get("connected"))
        return supported

    def status(self) -> Dict[str, Any]:
        speaker_status = self._speaker.status()
        connected = bool(speaker_status.get("connected"))
        now = float(self._wall_clock())
        with self._lock:
            if not connected:
                self._supported = False
                self._available = False
            available = bool(self.enabled and connected and self._supported)
            duration = (
                max(0.0, now - self._started_at)
                if self._recording and self._started_at > 0
                else self._last_duration
            )
            file_name = os.path.basename(self._file) if self._file else ""
            last_name = os.path.basename(self._last_file) if self._last_file else ""
            return {
                "ok": True,
                "enabled": bool(self.enabled),
                "supported": bool(self._supported),
                "available": available,
                "online": available,
                "recording": bool(self._recording),
                "source": str(self._source),
                "sample_rate": int(self.sample_rate),
                "channels": int(self.channels),
                "chunk_ms": int(self.chunk_ms),
                "duration_ms": int(max(0.0, duration) * 1000.0),
                "started_ms": (
                    int(self._started_at * 1000.0) if self._started_at > 0 else 0
                ),
                "file": str(self._file),
                "file_name": file_name,
                "last_file": str(self._last_file),
                "last_file_name": last_name,
                "last_duration_ms": int(max(0.0, self._last_duration) * 1000.0),
                "bytes": int(self._bytes),
                "volume": int(self.volume),
                "gain": round(float(self.gain), 2),
                "error": str(self._error),
                "max_duration_sec": float(self.max_duration_sec),
                "record_dir": self.record_dir,
                "topic": self.topic,
                "status_topic": self.status_topic,
                "status_rate_hz": float(self.status_rate_hz),
            }

    def summary(self) -> str:
        status = self.status()
        if not status["enabled"]:
            return "mic=disabled"
        if status["recording"]:
            return f"mic=recording({status['duration_ms'] / 1000.0:.1f}s)"
        return f"mic={'ready' if status['available'] else 'unavailable'}"

    # ------------------------------------------------------------- recording
    def start(self, duration: Optional[Any] = None) -> Dict[str, Any]:
        if not self.enabled:
            raise SpeakerError("麦克风功能未启用")
        with self._lock:
            if self._recording:
                raise ValueError("正在录音，请先停止")
            thread = self._thread
            if thread is not None and thread.is_alive():
                raise ValueError("上一次录音正在收尾")
        speaker_status = self._speaker.status()
        if not speaker_status.get("connected"):
            raise SpeakerError("蓝牙音箱未连接")
        if not self.refresh_capabilities(force=True):
            raise SpeakerError("蓝牙音箱未提供 HFP/HSP 麦克风")

        limit = self.max_duration_sec
        if duration is not None and str(duration).strip() != "":
            try:
                value = float(str(duration).strip())
            except (TypeError, ValueError):
                raise ValueError("duration 必须是秒数")
            if value <= 0.0 or value > self.max_duration_sec:
                raise ValueError(f"duration 必须在 0..{self.max_duration_sec:.0f} 秒")
            limit = value

        card = self._speaker.card_name()
        profiles = [str(item) for item in (self._speaker.card_profiles() or [])]
        target = self._choose_profile(profiles)
        if not target:
            raise SpeakerError("未找到 HFP/HSP profile")
        restore = self._speaker.active_card_profile() or "a2dp-sink"
        if restore == "off":
            restore = "a2dp-sink"

        # 播放中的 A2DP 流会阻塞 profile 切换，先停播。
        try:
            self._speaker.stop_playback()
        except Exception:  # noqa: BLE001
            pass

        switched = False
        if not str(restore).startswith("headset-head-unit"):
            switched = self._speaker.set_card_profile(target)
            if not switched:
                alternate = self._choose_profile(
                    [item for item in profiles if item != target]
                )
                if alternate:
                    switched = self._speaker.set_card_profile(alternate)
            if not switched:
                raise SpeakerError("切换 HFP 录音 profile 失败")

        mac = normalize_mac(getattr(self._speaker, "device_mac", ""))
        source = self._wait_source(mac, timeout=min(6.0, self._speaker.sink_wait_sec))
        if not source:
            if switched:
                self._restore_speaker_profile(card, restore)
            raise SpeakerError("未找到蓝牙麦克风输入源")
        # HFP 下输出 sink 也会变成 headset-head-unit；刷新 speaker 状态，
        # 这样录音期间如果还有 TTS，会走当前 HFP sink 而不是已消失的 A2DP sink。
        try:
            self._speaker.refresh_sink()
        except Exception:  # noqa: BLE001
            pass

        path = os.path.join(
            self.record_dir,
            "mic-{}-{}.wav".format(
                time.strftime("%Y%m%d-%H%M%S", time.localtime()),
                int((self._wall_clock() * 1000.0) % 1000),
            ),
        )
        wave_handle = None
        proc = None
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            wave_handle = wave.open(path, "wb")
            wave_handle.setnchannels(self.channels)
            wave_handle.setsampwidth(2)
            wave_handle.setframerate(self.sample_rate)
            command = self._capture_command(source)
            if command is None:
                raise SpeakerError("未找到录音工具 parec/pw-record")
            proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception:
            if wave_handle is not None:
                try:
                    wave_handle.close()
                except Exception:  # noqa: BLE001
                    pass
            try:
                os.remove(path)
            except OSError:
                pass
            if switched:
                self._restore_speaker_profile(card, restore)
            raise

        stop_event = threading.Event()
        started_at = float(self._wall_clock())
        thread = threading.Thread(
            target=self._capture_loop,
            args=(proc, wave_handle, path, source, stop_event, started_at, limit),
            name="edge-mic-capture",
            daemon=True,
        )
        with self._lock:
            self._recording = True
            self._supported = True
            self._available = True
            self._source = source
            self._file = path
            self._started_at = started_at
            self._duration_limit = limit
            self._bytes = 0
            self._error = ""
            self._stop_event = stop_event
            self._thread = thread
            self._proc = proc
            self._wave = wave_handle
            self._card = card
            self._restore_profile = restore
            self._finalized = False
        thread.start()
        self._apply_volume(source)
        self._emit("microphone", f"开始录音 {os.path.basename(path)}", "info")
        return self.status()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            recording = bool(self._recording)
            thread = self._thread
            proc = self._proc
            stop_event = self._stop_event
        if not recording and (thread is None or not thread.is_alive()):
            return self.status()
        stop_event.set()
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:  # noqa: BLE001
                pass
        if thread is not None and thread.is_alive():
            if thread is not threading.current_thread():
                thread.join(timeout=6.0)
        else:
            self._finish("")
        return self.status()

    def _capture_loop(
        self,
        proc: subprocess.Popen,
        wave_handle: wave.Wave_write,
        path: str,
        source: str,
        stop_event: threading.Event,
        started_at: float,
        limit: float,
    ) -> None:
        error = ""
        chunk_bytes = max(
            2,
            int(self.sample_rate * self.channels * 2 * self.chunk_ms / 1000.0),
        )
        buffer = bytearray()

        def publish_chunk(chunk: bytes) -> None:
            if not chunk:
                return
            pcm = chunk
            with self._lock:
                gain = float(self.gain)
            if gain > 1.0:
                try:
                    pcm = audioop.mul(chunk, 2, gain)
                except Exception as exc:  # noqa: BLE001
                    LOGGER.warning("microphone gain failed: %s", exc)
                    pcm = chunk
            wave_handle.writeframes(pcm)
            with self._lock:
                self._bytes += len(pcm)
            callback = self._on_audio_chunk
            if callback is not None:
                try:
                    callback(bytes(pcm), self.sample_rate, self.channels)
                except Exception as exc:  # noqa: BLE001
                    LOGGER.warning("microphone chunk callback failed: %s", exc)

        try:
            stdout = proc.stdout
            if stdout is None:
                raise SpeakerError("录音进程没有 stdout")
            while not stop_event.is_set():
                # parec 的管道往往按 10~20ms 小块返回；先累积到 chunk_ms
                # 再发布，避免 ROS 以 50Hz 发 20ms 碎片帧。
                data = stdout.read(chunk_bytes)
                if not data:
                    break
                buffer.extend(data)
                while len(buffer) >= chunk_bytes:
                    chunk = bytes(buffer[:chunk_bytes])
                    del buffer[:chunk_bytes]
                    publish_chunk(chunk)
                    if (self._wall_clock() - started_at) >= limit:
                        stop_event.set()
                        break
                if (self._wall_clock() - started_at) >= limit:
                    break
            if buffer and not error:
                publish_chunk(bytes(buffer))
        except Exception as exc:  # noqa: BLE001
            error = f"录音失败: {exc}"
        finally:
            self._finish(error)

    def _finish(self, error: str) -> None:
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
            recording = self._recording
            self._recording = False
            proc = self._proc
            wave_handle = self._wave
            path = self._file
            card = self._card
            restore = self._restore_profile
            started_at = self._started_at
            self._proc = None
            self._wave = None
            if error:
                self._error = str(error)
            self._last_duration = (
                max(0.0, float(self._wall_clock()) - started_at)
                if started_at > 0
                else 0.0
            )
            if path and os.path.isfile(path):
                self._last_file = path
                size = os.path.getsize(path)
                if size <= 44 and not error:
                    error = "未采集到音频数据"
                    self._error = error
        if proc is not None:
            if proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=1.0)
                except Exception:  # noqa: BLE001
                    try:
                        proc.kill()
                    except Exception:  # noqa: BLE001
                        pass
        if wave_handle is not None:
            try:
                wave_handle.close()
            except Exception:  # noqa: BLE001
                pass
        if card and restore and self.auto_restore_profile:
            self._restore_speaker_profile(card, restore)
        if recording:
            if self._error:
                self._emit("microphone", f"录音失败：{self._error}", "error")
            else:
                self._emit(
                    "microphone",
                    f"录音已保存 {os.path.basename(self._last_file)}"
                    f"（{self._last_duration:.1f}s）",
                    "ok",
                )

    def _restore_speaker_profile(self, card: str, profile: str) -> None:
        try:
            self._speaker.set_card_profile(profile)
            time.sleep(0.4)
            self._speaker.refresh_sink()
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("restore A2DP profile failed: %s", exc)

    def _wait_source(self, mac: str, *, timeout: float) -> str:
        deadline = time.monotonic() + max(0.5, float(timeout))
        while True:
            result = self._run(["pactl", "list", "short", "sources"], timeout=5.0)
            sources = parse_sink_list(result.stdout) if result.ok else []
            selected = select_mic_source(sources, mac)
            if selected is not None:
                return str(selected["name"])
            if time.monotonic() >= deadline:
                return ""
            time.sleep(0.3)

    def _choose_profile(self, profiles: List[str]) -> str:
        preferred = [
            self.profile_preference,
            "headset-head-unit",
            "headset-head-unit-cvsd",
        ]
        for name in preferred:
            if name in profiles:
                return name
        for name in profiles:
            if name.startswith("headset-head-unit"):
                return name
        return ""

    def _capture_command(self, source: str) -> Optional[List[str]]:
        parec = self._which("parec")
        if parec:
            return [
                parec,
                f"--device={source}",
                f"--rate={self.sample_rate}",
                f"--channels={self.channels}",
                "--format=s16le",
                "--raw",
                f"--latency-msec={self.chunk_ms}",
                f"--process-time-msec={self.chunk_ms}",
            ]
        pw_record = self._which("pw-record")
        if pw_record:
            # pw-record 0.3.x 的 --target 只接受数字 node id；不指定时用默认源。
            self._run(["pactl", "set-default-source", source], timeout=5.0)
            return [
                pw_record,
                "--rate",
                str(self.sample_rate),
                "--channels",
                str(self.channels),
                "--format",
                "s16",
                "-",
            ]
        return None

    def _apply_volume(self, source: str) -> None:
        if not source:
            return
        with self._lock:
            volume = int(self.volume)
        self._run(
            ["pactl", "set-source-volume", source, f"{volume}%"], timeout=5.0
        )

    # ------------------------------------------------------------- files
    def recordings(self) -> Dict[str, Any]:
        entries: List[Dict[str, Any]] = []
        try:
            names = os.listdir(self.record_dir)
        except OSError:
            names = []
        for name in names:
            if not name.lower().endswith(".wav"):
                continue
            path = os.path.join(self.record_dir, name)
            try:
                stat = os.stat(path)
            except OSError:
                continue
            entries.append(
                {
                    "name": name,
                    "size": int(stat.st_size),
                    "duration_ms": _wav_duration_ms(path),
                    "modified_ms": int(stat.st_mtime * 1000.0),
                }
            )
        entries.sort(key=lambda item: item["modified_ms"], reverse=True)
        return {
            "ok": True,
            "count": len(entries),
            "recordings": entries[:50],
            "dir": self.record_dir,
        }

    def recording_path(self, name: Any) -> str:
        text = str(name or "").strip()
        if not text or os.path.basename(text) != text:
            raise ValueError("录音文件名非法")
        if not _RECORDING_NAME_RE.match(text):
            raise ValueError("录音文件名非法")
        root = os.path.realpath(self.record_dir)
        path = os.path.realpath(os.path.join(root, text))
        try:
            inside = os.path.commonpath([path, root]) == root
        except ValueError:
            inside = False
        if not inside:
            raise ValueError("录音文件必须位于 record_dir 内")
        if not os.path.isfile(path):
            raise ValueError("录音文件不存在")
        return path

    def play_recording(self, name: Any) -> Dict[str, Any]:
        path = self.recording_path(name)
        return self._speaker.play_audio_path(path)

    def set_volume(self, value: Any) -> Dict[str, Any]:
        try:
            volume = int(str(value).strip())
        except (TypeError, ValueError):
            raise ValueError("volume 必须是 0..100 的整数")
        if volume < 0 or volume > 100:
            raise ValueError("volume 必须在 0..100")
        with self._lock:
            self.volume = volume
            source = str(self._source)
        if source:
            self._apply_volume(source)
        return self.status()

    def set_gain(self, value: Any) -> Dict[str, Any]:
        try:
            gain = float(str(value).strip())
        except (TypeError, ValueError):
            raise ValueError("gain 必须是数字")
        if gain < 1.0 or gain > 8.0:
            raise ValueError("gain 必须在 1.0..8.0")
        with self._lock:
            self.gain = gain
        return self.status()

    # ------------------------------------------------------------- helpers
    def _run(self, args: List[str], *, timeout: float = 10.0) -> Any:
        try:
            return self._runner.run(list(args), timeout=timeout)
        except TypeError:
            return self._runner.run(list(args))
        except Exception as exc:  # noqa: BLE001
            from .speaker import CommandResult

            return CommandResult(126, "", str(exc))

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
