import contextlib
import sys
import types
from types import SimpleNamespace

import numpy as np

from center_server.engine import SmolVLAEngine


class FakeTensor:
    def __init__(self, array):
        self.array = np.asarray(array, dtype=np.float32)

    @property
    def shape(self):
        return self.array.shape

    @property
    def ndim(self):
        return self.array.ndim

    @property
    def dtype(self):
        return "float32"

    def unsqueeze(self, dim):
        return FakeTensor(np.expand_dims(self.array, dim))

    def to(self, *args, **kwargs):
        return self

    def detach(self):
        return self

    def float(self):
        return FakeTensor(self.array.astype(np.float32))

    def cpu(self):
        return self

    def numpy(self):
        return self.array

    def max(self):
        return float(self.array.max()) if self.array.size else 0.0


class FakeTorch(types.ModuleType):
    def __init__(self):
        super().__init__("torch")
        self.float32 = "float32"
        self.float16 = "float16"
        self.bfloat16 = "bfloat16"
        self.cuda = SimpleNamespace(is_available=lambda: False)

    def from_numpy(self, array):
        return FakeTensor(array)

    def tensor(self, values, dtype=None):
        return FakeTensor(values)

    def inference_mode(self):
        return contextlib.nullcontext()


class FakePolicy:
    def __init__(self):
        self.config = SimpleNamespace(
            image_features=["observation.images.camera1"],
            input_features={"observation.state": SimpleNamespace(shape=(3,))},
            output_features={"action": SimpleNamespace(shape=(3,))},
            chunk_size=2,
            n_action_steps=1,
        )
        self.reset_called = 0

    @classmethod
    def from_pretrained(cls, path, **kwargs):
        return cls()

    def to(self, *args, **kwargs):
        return self

    def eval(self):
        return self

    def parameters(self):
        return iter([FakeTensor(np.zeros(1))])

    def predict_action_chunk(self, batch):
        return FakeTensor(
            np.array([[[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]], dtype=np.float32)
        )

    def reset(self):
        self.reset_called += 1


def _install_fakes(monkeypatch):
    torch = FakeTorch()
    monkeypatch.setitem(sys.modules, "torch", torch)

    modules = {
        "lerobot": types.ModuleType("lerobot"),
        "lerobot.policies": types.ModuleType("lerobot.policies"),
        "lerobot.policies.smolvla": types.ModuleType("lerobot.policies.smolvla"),
        "lerobot.policies.smolvla.modeling_smolvla": types.ModuleType(
            "lerobot.policies.smolvla.modeling_smolvla"
        ),
        "lerobot.policies.factory": types.ModuleType("lerobot.policies.factory"),
    }
    modules["lerobot.policies.smolvla.modeling_smolvla"].SmolVLAPolicy = FakePolicy
    modules["lerobot.policies.factory"].make_pre_post_processors = (
        lambda **kwargs: (lambda batch: batch, lambda actions: actions)
    )
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)


def test_smolvla_engine_load_and_infer(monkeypatch):
    _install_fakes(monkeypatch)
    engine = SmolVLAEngine(
        {
            "path": "fake/smolvla",
            "device": "cpu",
            "dtype": "auto",
            "image_keys": ["camera1"],
            "state_dim": 3,
            "chunk_size": 2,
            "n_action_steps": 1,
            "require_processors": True,
            "reload_interval_sec": 0,
        }
    )
    try:
        engine.ensure_loaded()
        status = engine.status()
        assert status["loaded"] is True
        assert status["action_dim"] == 3
        assert status["state_dim"] == 3
        assert status["chunk_size"] == 2
        assert status["image_keys"] == ["camera1"]

        result = engine.infer(
            "go",
            {"camera1": np.zeros((3, 8, 8), dtype=np.float32)},
            [0.0, 0.0, 0.0],
        )
        assert result.actions == [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        assert result.latency_ms >= 0

        engine.reset()
        assert engine._policy.reset_called == 1
    finally:
        engine.close()
