"""蓝牙音箱/语音播报核心逻辑测试（不依赖真实蓝牙/声卡）。"""

import pytest

from edge_server.core.speaker import (
    CommandResult,
    SpeakerManager,
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
    assert select_sink(sinks, "AA:BB:CC:DD:EE:FF")["id"] == 42

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
