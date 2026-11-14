"""蓝牙 HFP 麦克风采集/录音测试（不依赖真实蓝牙）。"""

import os
import stat
import time

import pytest

from edge_server.core.microphone import (
    MicrophoneManager,
    select_mic_source,
)
from edge_server.core.speaker import CommandResult


class FakeSpeaker:
    device_mac = "73:DB:39:F2:15:4B"
    sink_wait_sec = 3.0

    def __init__(self):
        self.connected = True
        self.profiles = [
            "a2dp-sink",
            "headset-head-unit",
            "headset-head-unit-cvsd",
        ]
        self.active_profile = "a2dp-sink"
        self.calls = []

    def status(self):
        return {"connected": self.connected, "state": "connected"}

    def card_name(self):
        return "bluez_card.73_DB_39_F2_15_4B"

    def card_profiles(self):
        self.calls.append(("card_profiles",))
        return list(self.profiles)

    def active_card_profile(self):
        self.calls.append(("active_profile",))
        return self.active_profile

    def set_card_profile(self, profile):
        self.calls.append(("set_profile", profile))
        if profile not in self.profiles:
            return False
        self.active_profile = profile
        return True

    def stop_playback(self):
        self.calls.append(("stop_playback",))
        return {"ok": True}

    def refresh_sink(self):
        self.calls.append(("refresh_sink",))
        return True

    def play_audio_path(self, path):
        self.calls.append(("play_audio_path", path))
        return {"ok": True}


class FakeRunner:
    def __init__(self, source="bluez_input.73_DB_39_F2_15_4B.headset-head-unit"):
        self.source = source
        self.calls = []

    def run(self, args, timeout=10.0, **kwargs):
        self.calls.append(list(args))
        command = [str(item) for item in args]
        if command[:4] == ["pactl", "list", "short", "sources"]:
            line = (
                f"443\t{self.source}\tPipeWire\ts16le 1ch 8000Hz\tSUSPENDED"
                if self.source
                else ""
            )
            return CommandResult(0, line, "")
        return CommandResult(0, "", "")


def make_manager(tmp_path, speaker=None, runner=None, which=None, **overrides):
    cfg = {
        "enabled": True,
        "record_dir": str(tmp_path / "mic"),
        "sample_rate": 8000,
        "channels": 1,
        "chunk_ms": 20,
        "max_duration_sec": 2,
        "volume": 90,
    }
    cfg.update(overrides)
    return MicrophoneManager(
        cfg,
        speaker=speaker or FakeSpeaker(),
        runner=runner or FakeRunner(),
        which=which or (lambda name: None),
        on_audio_chunk=None,
    )


def test_select_mic_source():
    sources = [
        {"id": 1, "name": "alsa_output.monitor"},
        {"id": 443, "name": "bluez_input.73_DB_39_F2_15_4B.headset-head-unit"},
        {"id": 444, "name": "bluez_output.73_DB_39_F2_15_4B.headset-head-unit.monitor"},
    ]
    selected = select_mic_source(sources, "73:DB:39:F2:15:4B")
    assert selected["id"] == 443
    assert select_mic_source(sources, "AA:BB:CC:DD:EE:FF") is None


def test_capabilities_and_status(tmp_path):
    speaker = FakeSpeaker()
    manager = make_manager(tmp_path, speaker=speaker)
    assert manager.refresh_capabilities(force=True) is True
    status = manager.status()
    assert status["available"] is True
    assert status["online"] is True
    assert status["recording"] is False


def test_capabilities_false_without_hfp(tmp_path):
    speaker = FakeSpeaker()
    speaker.profiles = ["a2dp-sink"]
    manager = make_manager(tmp_path, speaker=speaker)
    assert manager.refresh_capabilities(force=True) is False
    assert manager.status()["available"] is False


def test_disabled_and_disconnected(tmp_path):
    speaker = FakeSpeaker()
    manager = make_manager(tmp_path, speaker=speaker, enabled=False)
    with pytest.raises(Exception):
        manager.start()
    assert manager.status()["enabled"] is False

    speaker = FakeSpeaker()
    speaker.connected = False
    manager = make_manager(tmp_path, speaker=speaker)
    with pytest.raises(Exception):
        manager.start()


def test_recording_path_confined(tmp_path):
    speaker = FakeSpeaker()
    manager = make_manager(tmp_path, speaker=speaker)
    os.makedirs(manager.record_dir, exist_ok=True)
    target = os.path.join(manager.record_dir, "mic-test.wav")
    with open(target, "wb") as handle:
        handle.write(b"RIFF")
    assert manager.recording_path("mic-test.wav") == os.path.realpath(target)
    with pytest.raises(ValueError):
        manager.recording_path("../mic-test.wav")
    with pytest.raises(ValueError):
        manager.recording_path("/tmp/mic-test.wav")
    with pytest.raises(ValueError):
        manager.recording_path("missing.wav")


def test_recordings_list(tmp_path):
    speaker = FakeSpeaker()
    manager = make_manager(tmp_path, speaker=speaker)
    os.makedirs(manager.record_dir, exist_ok=True)
    target = os.path.join(manager.record_dir, "mic-one.wav")
    with open(target, "wb") as handle:
        handle.write(b"RIFF" + b"\x00" * 100)
    payload = manager.recordings()
    assert payload["count"] == 1
    assert payload["recordings"][0]["name"] == "mic-one.wav"


def test_record_and_finish_restores_profile(tmp_path, monkeypatch):
    # 用一个真实可执行的假 parec 输出连续静音 PCM，验证采集线程/WAV/收尾。
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    fake_parec = bin_dir / "parec"
    fake_parec.write_text(
        "#!/bin/sh\n"
        "while true; do dd if=/dev/zero bs=320 count=1 2>/dev/null; sleep 0.02; done\n",
        encoding="utf-8",
    )
    fake_parec.chmod(fake_parec.stat().st_mode | stat.S_IXUSR)

    speaker = FakeSpeaker()
    manager = make_manager(
        tmp_path,
        speaker=speaker,
        which=lambda name: str(fake_parec) if name == "parec" else None,
    )
    manager.refresh_capabilities(force=True)
    manager.start(duration=0.3)
    assert manager.status()["recording"] is True
    time.sleep(0.8)
    status = manager.status()
    assert status["recording"] is False
    assert status["last_file"]
    assert os.path.isfile(status["last_file"])
    assert status["last_duration_ms"] > 0
    assert ("set_profile", "a2dp-sink") in speaker.calls
    with open(status["last_file"], "rb") as handle:
        assert handle.read(4) == b"RIFF"
