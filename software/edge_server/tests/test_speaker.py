"""蓝牙音箱/语音播报核心逻辑测试（不依赖真实蓝牙/声卡）。"""

import os
import threading

import pytest

from edge_server.core.speaker import (
    CommandResult,
    SpeakerManager,
    _prepend_wav_silence,
    normalize_mac,
    parse_bluetooth_devices,
    parse_bluetooth_info,
    parse_sink_list,
    parse_wpctl_status,
    sanitize_text,
    select_sink,
)

INFO = """Device 73:DB:39:F2:15:4B (public)
\tName: newmine
\tAlias: newmine
\tClass: 0x00340404
\tIcon: audio-headset
\tPaired: yes
\tTrusted: yes
\tConnected: yes
"""


class FakeRunner:
    def __init__(self, sinks=None):
        self.calls = []
        self.sinks = sinks if sinks is not None else [
            {
                "id": 42,
                "name": "bluez_output.73_DB_39_F2_15_4B.1",
            }
        ]

    def run(self, args, timeout=10.0, **kwargs):
        self.calls.append((list(args), float(timeout)))
        command = [str(item) for item in args]
        if command[:2] == ["bluetoothctl", "info"]:
            return CommandResult(0, INFO, "")
        if command[:3] == ["pactl", "list", "short"]:
            lines = "\n".join(
                f"{item['id']}\t{item['name']}\tmodule-bluez5-device.c\t"
                "s16le\t2ch\t44100Hz\tSUSPENDED"
                for item in self.sinks
            )
            return CommandResult(0, lines, "")
        if command[:2] == ["pactl", "set-default-sink"]:
            return CommandResult(0, "", "")
        if command[:2] == ["pactl", "set-sink-volume"]:
            return CommandResult(0, "", "")
        return CommandResult(0, "", "")

    def called(self, *parts):
        return any(call[: len(parts)] == list(parts) for call, _ in self.calls)


def fake_which(name):
    return f"/usr/bin/{name}"


def test_parse_bluetooth_helpers():
    devices = parse_bluetooth_devices(
        "Device 73:DB:39:F2:15:4B newmine\n"
        "Device AA:BB:CC:DD:EE:FF 5B-9A-BA-5E-A0-35\n"
    )
    assert devices[0] == {"mac": "73:DB:39:F2:15:4B", "name": "newmine"}
    assert devices[1]["mac"] == "AA:BB:CC:DD:EE:FF"

    info = parse_bluetooth_info(INFO)
    assert info["name"] == "newmine"
    assert info["paired"] is True
    assert info["trusted"] is True
    assert info["connected"] is True


def test_default_speaker_names(tmp_path):
    manager = SpeakerManager(
        {"enabled": False, "audio_dir": str(tmp_path)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    assert manager.model == "newmine"
    assert manager.device_name == "newmine"
    assert manager.device_names == ("newmine",)
    assert manager.force_a2dp is False
    assert manager.keepalive is True
    assert [item["id"] for item in manager.available_models()] == [
        "newmine",
        "am510",
    ]


def test_select_model_switches_device(tmp_path):
    runner = FakeRunner()
    manager = SpeakerManager(
        {
            "enabled": True,
            "model": "newmine",
            "models": {
                "newmine": {
                    "device_name": "newmine",
                    "device_mac": "73:DB:39:F2:15:4B",
                    "force_a2dp": False,
                },
                "am510": {
                    "device_name": "HONOR AM510",
                    "device_mac": "28:37:13:7F:10:09",
                    "force_a2dp": True,
                },
            },
            "audio_dir": str(tmp_path),
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager.start = lambda: None  # 只验证切型号，不启动真实线程
    status = manager.select_model("am510")
    assert status["model"] == "am510"
    assert manager.device_name == "HONOR AM510"
    assert manager.device_mac == "28:37:13:7F:10:09"
    assert manager.force_a2dp is True
    assert runner.called(
        "systemctl", "--user", "restart", "wireplumber"
    )


def test_newmine_hfp_sink_does_not_force_a2dp(tmp_path):
    runner = FakeRunner(
        sinks=[
            {
                "id": 7,
                "name": "bluez_output.73_DB_39_F2_15_4B.headset-head-unit",
            }
        ]
    )
    manager = SpeakerManager(
        {
            "enabled": True,
            "model": "newmine",
            "device_mac": "73:DB:39:F2:15:4B",
            "audio_dir": str(tmp_path),
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    assert manager.force_a2dp is False
    assert manager._refresh_sink("73:DB:39:F2:15:4B", initial=True) is True
    assert manager.status()["sink"]["name"].endswith("headset-head-unit")
    assert not runner.called("pactl", "set-card-profile")


def test_select_sink_prefers_a2dp():
    hfp = {
        "id": 1,
        "name": "bluez_output.28_37_13_7F_10_09.headset-head-unit",
    }
    a2dp = {
        "id": 2,
        "name": "bluez_output.28_37_13_7F_10_09.a2dp-sink",
    }
    assert select_sink([hfp, a2dp], "28:37:13:7F:10:09")["id"] == 2
    assert select_sink([hfp], "28:37:13:7F:10:09", require_a2dp=True) is None


def test_force_a2dp_profile_uses_pactl(tmp_path):
    runner = FakeRunner()
    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(tmp_path)},
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager._force_a2dp_profile("28:37:13:7F:10:09")
    assert runner.called(
        "pactl",
        "set-card-profile",
        "bluez_card.28_37_13_7F_10_09",
        "a2dp-sink",
    )


def test_sink_alive_detects_disappeared_sink(tmp_path):
    name = "bluez_output.73_DB_39_F2_15_4B.a2dp-sink"
    runner = FakeRunner(sinks=[{"id": 42, "name": name}])
    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(tmp_path)},
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager._sink_name = name
    manager._sink_ready = True
    assert manager._sink_alive() is True
    runner.sinks = []
    assert manager._sink_alive() is False


def test_discover_uses_device_aliases(tmp_path):
    class DevicesRunner(FakeRunner):
        def run(self, args, timeout=10.0, **kwargs):
            command = [str(item) for item in args]
            if command[:3] == ["bluetoothctl", "devices"]:
                self.calls.append((list(args), float(timeout)))
                return CommandResult(
                    0, "Device 11:22:33:44:55:66 newmine\n", ""
                )
            return super().run(args, timeout=timeout, **kwargs)

    runner = DevicesRunner()
    manager = SpeakerManager(
        {
            "enabled": True,
            "device_name": "HONOR AM510",
            "device_aliases": ["newmine"],
            "audio_dir": str(tmp_path),
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    assert manager.device_names == ("HONOR AM510", "newmine")
    assert manager._discover_device() == "11:22:33:44:55:66"
    assert manager.status()["device"]["name"] == "newmine"


def test_parse_sink_helpers():
    sinks = parse_sink_list(
        "42\tbluez_output.73_DB_39_F2_15_4B.1\tmodule-bluez5-device.c\t"
        "s16le\t2ch\t44100Hz\tSUSPENDED\n"
        "7\talsa_output.platform-bcm2835.hdmi-stereo\tmodule-alsa-card.c\t"
        "s16le\t2ch\t44100Hz\tIDLE\n"
    )
    assert sinks[0]["id"] == 42
    selected = select_sink(sinks, "73:db:39:f2:15:4b")
    assert selected["name"] == "bluez_output.73_DB_39_F2_15_4B.1"
    assert select_sink(sinks, "AA:BB:CC:DD:EE:FF") is None

    wp = parse_wpctl_status(
        " ├─ Sinks:\n"
        " │  *   42. bluez_output.73_DB_39_F2_15_4B.1 [vol: 0.70]\n"
    )
    assert wp == [
        {"id": 42, "name": "bluez_output.73_DB_39_F2_15_4B.1"}
    ]


def test_text_and_mac_normalization():
    assert normalize_mac("73-db-39-f2-15-4b") == "73:DB:39:F2:15:4B"
    assert normalize_mac("not-a-mac") == ""
    assert sanitize_text("  你好\x00 世界\n第二行  ", 20) == "你好 世界 第二行"
    assert sanitize_text("abcdef", 3) == "abc"


def test_connect_selects_bluez_sink(tmp_path):
    runner = FakeRunner()
    manager = SpeakerManager(
        {
            "enabled": True,
            "device_name": "newmine",
            "device_mac": "73:DB:39:F2:15:4B",
            "volume": 65,
            "audio_dir": str(tmp_path),
            "tts": {"backend": "espeak-ng", "voice": "cmn"},
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager._connect_device("73:DB:39:F2:15:4B")
    status = manager.status()
    assert status["state"] == "connected"
    assert status["connected"] is True
    assert status["audio_ready"] is True
    assert status["sink"]["name"] == "bluez_output.73_DB_39_F2_15_4B.1"
    assert status["sink"]["volume"] == 65
    assert status["device"]["name"] == "newmine"
    assert status["tts"]["backend"] == "espeak-ng"
    assert runner.called("pactl", "set-default-sink")
    assert runner.called("pactl", "set-sink-volume")


def test_profile_unavailable_restarts_wireplumber_once(tmp_path, monkeypatch):
    import edge_server.core.speaker as speaker_module

    monkeypatch.setattr(speaker_module.time, "sleep", lambda _seconds: None)
    runner = FakeRunner()
    manager = SpeakerManager(
        {
            "enabled": True,
            "device_mac": "73:DB:39:F2:15:4B",
            "audio_dir": str(tmp_path),
            "recover_interval_sec": 60.0,
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    assert manager._recover_audio_profile("org.bluez.Error.Failed br-connection-profile-unavailable")
    assert runner.called("systemctl", "--user", "restart", "wireplumber")
    assert not manager._recover_audio_profile("br-connection-profile-unavailable")


def test_connected_without_sink_reconnects_profile(tmp_path):
    runner = FakeRunner(sinks=[])
    manager = SpeakerManager(
        {
            "enabled": True,
            "device_mac": "73:DB:39:F2:15:4B",
            "audio_dir": str(tmp_path),
            "connect_timeout_sec": 5.0,
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager._connected = True
    manager._sink_ready = False
    manager._refresh_sink = lambda *args, **kwargs: False
    manager._ensure_connected()
    assert runner.called("bluetoothctl", "connect")


def test_prepare_hits_existing_cache(tmp_path):
    manager = SpeakerManager(
        {
            "enabled": True,
            "audio_dir": str(tmp_path),
            "tts": {"backend": "espeak-ng", "voice": "cmn"},
        },
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    cache = manager._tts_cache_path(
        "你好", "cmn", manager.tts_speed, manager.tts_pitch
    )
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    with open(cache, "wb") as handle:
        handle.write(b"RIFF" + b"\x00" * 100)
    result = manager.prepare("你好")
    assert result["cached"] is True


def test_play_say_job_uses_prepared_file(tmp_path):
    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(tmp_path)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    path = tmp_path / "say.wav"
    path.write_bytes(b"RIFF")
    played = {}
    manager._play_path = lambda p, text="": played.update(path=p, text=text)
    job = {
        "text": "你好",
        "event": threading.Event(),
        "path": str(path),
        "error": "",
        "cancelled": False,
    }
    job["event"].set()
    manager._play_say_job(job)
    assert played == {"path": str(path), "text": "你好"}
    assert not path.exists()


def test_queue_say_and_stop_without_threads(tmp_path):
    runner = FakeRunner()
    manager = SpeakerManager(
        {
            "enabled": True,
            "device_mac": "73:DB:39:F2:15:4B",
            "audio_dir": str(tmp_path),
        },
        runner=runner,
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    manager.start = lambda: None  # 只验证排队，不启动真实线程
    manager._connected = True
    manager._sink_ready = True
    manager.say("机器人已启动")
    assert manager.status()["playback"]["queue"] == 1
    manager.stop_playback()
    assert manager.status()["playback"]["queue"] == 0
    assert manager.status()["playback"]["playing"] is False


def test_say_requires_audio_ready(tmp_path):
    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(tmp_path)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    with pytest.raises(Exception, match="音频输出"):
        manager.say("你好")


def test_say_validation_and_disabled(tmp_path):
    manager = SpeakerManager(
        {"enabled": False, "audio_dir": str(tmp_path)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    with pytest.raises(Exception):
        manager.say("你好")
    assert manager.status()["status"] == "disabled"


def test_prepend_wav_silence(tmp_path):
    import wave

    path = tmp_path / "say.wav"
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(8000)
        handle.writeframes(b"\x10\x00" * 800)
    _prepend_wav_silence(str(path), 450)
    with wave.open(str(path), "rb") as handle:
        assert handle.getnframes() == 800 + int(8000 * 0.45)
        assert handle.getframerate() == 8000


def test_player_command_prefers_paplay(tmp_path):
    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(tmp_path)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    command = manager._player_command(
        "/tmp/hello.wav", "bluez_output.73_DB_39_F2_15_4B.1", 42
    )
    assert command[0].endswith("paplay")
    assert "--device=bluez_output.73_DB_39_F2_15_4B.1" in command


def test_audio_file_is_confined_to_audio_dir(tmp_path):
    audio_dir = tmp_path / "audio"
    audio_dir.mkdir()
    (audio_dir / "hello.wav").write_bytes(b"RIFF")
    outside = tmp_path / "outside.wav"
    outside.write_bytes(b"RIFF")

    manager = SpeakerManager(
        {"enabled": True, "audio_dir": str(audio_dir)},
        runner=FakeRunner(),
        which=fake_which,
        temp_dir=str(tmp_path / "tmp"),
    )
    assert manager.resolve_audio_file("hello.wav").endswith("hello.wav")
    with pytest.raises(ValueError, match="audio_dir"):
        manager.resolve_audio_file(str(outside))
    with pytest.raises(ValueError, match="audio_dir"):
        manager.resolve_audio_file("../outside.wav")
    with pytest.raises(ValueError):
        manager.resolve_audio_file("missing.wav")
