"""任务状态机（DESIGN.md 5.7）。"""

from __future__ import annotations

import threading
from enum import Enum


class State(Enum):
    IDLE = "IDLE"
    RECEIVING_DATA = "RECEIVING_DATA"
    INFERENCING = "INFERENCING"
    EXECUTING = "EXECUTING"
    REVERSING = "REVERSING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    TASK_SUCCESS = "TASK_SUCCESS"
    TASK_FAILED = "TASK_FAILED"


_TRANSITIONS = {
    State.IDLE: {
        "recv_data": State.RECEIVING_DATA,
        "start_task": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
        "error": State.ERROR,
        "reset": State.IDLE,
    },
    State.RECEIVING_DATA: {
        "start_infer": State.INFERENCING,
        "data_timeout": State.IDLE,
        "task_cancel": State.IDLE,
        "reverse_req": State.REVERSING,
        "error": State.ERROR,
    },
    State.INFERENCING: {
        "infer_ok": State.EXECUTING,
        "infer_low_conf": State.PAUSED,
        "reverse_req": State.REVERSING,
        "task_cancel": State.IDLE,
        "error": State.ERROR,
    },
    State.EXECUTING: {
        "task_done": State.TASK_SUCCESS,
        "task_failed": State.TASK_FAILED,
        "pause": State.PAUSED,
        "reverse_req": State.REVERSING,
        "task_cancel": State.IDLE,
        "error": State.ERROR,
    },
    State.REVERSING: {
        "reverse_done": State.IDLE,
        "task_cancel": State.IDLE,
        "error": State.ERROR,
    },
    State.PAUSED: {
        "resume": State.IDLE,
        "task_cancel": State.IDLE,
        "reverse_req": State.REVERSING,
        "error": State.ERROR,
    },
    State.ERROR: {
        "reset": State.IDLE,
        "reverse_req": State.REVERSING,
    },
    State.TASK_SUCCESS: {
        "reset": State.IDLE,
        "recv_data": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
    },
    State.TASK_FAILED: {
        "reset": State.IDLE,
        "recv_data": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
    },
}


class TaskStateMachine:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = State.IDLE

    @property
    def state(self) -> State:
        with self._lock:
            return self._state

    @property
    def state_name(self) -> str:
        return self.state.value

    def transition(self, event: str) -> bool:
        with self._lock:
            next_state = _TRANSITIONS.get(self._state, {}).get(event)
            if next_state is None:
                return False
            self._state = next_state
            return True

    def force(self, state: State):
        with self._lock:
            self._state = state

    def reset(self):
        self.force(State.IDLE)
