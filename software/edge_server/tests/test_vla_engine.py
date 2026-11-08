import numpy as np

from edge_server.core.data_preprocessor import SyncedDTO
from edge_server.core.vla_engine import VLAEngine


def make_dto(instruction: str) -> SyncedDTO:
    return SyncedDTO(
        image=np.zeros((3, 8, 8), dtype=np.float32),
        laser=np.ones(10, dtype=np.float32),
        audio=np.zeros((64, 4), dtype=np.float32),
        odom={"x": 0.0},
        joint={},
        task_id="task-1",
        instruction=instruction,
    )


def test_forward_intent():
    engine = VLAEngine({"device": "cpu", "confidence_threshold": 0.6})
    intent = engine.predict(make_dto("请前进"))
    assert intent.intent == "forward"
    assert intent.task_id == "task-1"
    assert intent.value > 0.0
    assert intent.confidence >= 0.6


def test_stop_has_priority_over_forward():
    engine = VLAEngine({"confidence_threshold": 0.6})
    intent = engine.predict(make_dto("停止前进"))
    assert intent.intent == "stop"
    assert intent.value == 0.0


def test_unknown_instruction_requests_confirmation():
    engine = VLAEngine({"confidence_threshold": 0.6})
    intent = engine.predict(make_dto("帮我泡一杯咖啡"))
    assert intent.intent == "stop"
    assert intent.confidence < 0.6


def test_unload_is_idempotent():
    engine = VLAEngine({"confidence_threshold": 0.6})
    engine.unload()
    engine.unload()
    assert engine.model is None
