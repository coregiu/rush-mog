import base64
import io

import pytest
from PIL import Image

from center_server.config import ConfigError, load_config
from center_server.errors import ModelUnavailable, UnprocessableEntity
from center_server.engine import SmolVLAEngine, StubEngine, create_engine
from center_server.service import InferenceService


def _config(**model_overrides):
    cfg = load_config(None)
    cfg["model"].update(
        {
            "backend": "stub",
            "stub_action": [0.2, 0.0, 0.0],
            "image_keys": ["camera1", "camera2"],
            "image_aliases": {"front": "camera1"},
            "require_images": True,
            "state_dim": 3,
        }
    )
    cfg["model"].update(model_overrides)
    return cfg


def _png_b64():
    image = Image.new("RGB", (8, 8), (255, 255, 255))
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def test_missing_camera_is_filled_and_warned():
    svc = InferenceService(_config())
    try:
        result = svc.infer(
            {
                "instruction": "go",
                "images": {"front": _png_b64()},
                "state": [0.0, 0.0, 0.0],
            }
        )
        assert result["ok"] is True
        assert any("camera2" in item for item in result["warnings"])
    finally:
        svc.close()


def test_missing_camera_error_policy():
    svc = InferenceService(_config(missing_image_policy="error"))
    try:
        with pytest.raises(UnprocessableEntity) as exc:
            svc.infer(
                {
                    "instruction": "go",
                    "images": {"front": _png_b64()},
                    "state": [0.0, 0.0, 0.0],
                }
            )
        assert exc.value.code == "missing_image"
    finally:
        svc.close()


def test_state_padding_and_truncation():
    svc = InferenceService(_config(state_dim=3, pad_state=True))
    try:
        padded = svc.infer(
            {"instruction": "go", "images": {"front": _png_b64()}, "state": [1.0]}
        )
        assert "state 已从 1 补齐到 3 维" in padded["warnings"]

        truncated = svc.infer(
            {
                "instruction": "go",
                "images": {"front": _png_b64()},
                "state": [1.0, 2.0, 3.0, 4.0],
            }
        )
        assert any("截断" in item for item in truncated["warnings"])
    finally:
        svc.close()


def test_invalid_chunk_size():
    svc = InferenceService(_config())
    try:
        with pytest.raises(UnprocessableEntity):
            svc.infer(
                {
                    "instruction": "go",
                    "images": {"front": _png_b64()},
                    "options": {"chunk_size": 0},
                }
            )
    finally:
        svc.close()


def test_unknown_image_key():
    svc = InferenceService(_config())
    try:
        with pytest.raises(UnprocessableEntity) as exc:
            svc.infer(
                {
                    "instruction": "go",
                    "images": {"rear": _png_b64()},
                }
            )
        assert exc.value.code == "unknown_image_key"
    finally:
        svc.close()


def test_stub_engine_factory():
    engine = create_engine(_config()["model"])
    assert isinstance(engine, StubEngine)
    assert engine.status()["loaded"] is True


def test_smolvla_load_failure_is_503(monkeypatch):
    def _fail():
        raise ModelUnavailable("no torch in test")

    monkeypatch.setattr(SmolVLAEngine, "_import_torch", staticmethod(_fail))
    cfg = _config(backend="smolvla", path="lerobot/smolvla_base")
    engine = SmolVLAEngine(cfg["model"])
    svc = InferenceService(cfg, engine=engine)
    try:
        assert svc.ready() is False
        with pytest.raises(ModelUnavailable):
            svc.infer(
                {
                    "instruction": "go",
                    "images": {"front": _png_b64()},
                }
            )
    finally:
        svc.close()


def test_config_invalid_port(tmp_path):
    path = tmp_path / "bad_port.yaml"
    path.write_text("server:\n  port: 70000\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)


def test_config_rejects_bad_backend(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("model:\n  backend: magic\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)
