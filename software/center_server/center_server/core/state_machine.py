"""任务/任务状态机（DESIGN.md 5.7 + SLAM/导航扩展）。

在原有 VLA 单步任务状态之外，增加建图（MAPPING）、导航（NAVIGATING）、
搜索/探索（SEARCHING）三种长任务状态。三者与 EXECUTING 互斥，反向操作和
急停可以随时抢占。
"""

from __future__ import annotations

import threading
from enum import Enum


class State(Enum):
    IDLE = "IDLE"
    RECEIVING_DATA = "RECEIVING_DATA"
    INFERENCING = "INFERENCING"
    EXECUTING = "EXECUTING"
    MAPPING = "MAPPING"
    NAVIGATING = "NAVIGATING"
    SEARCHING = "SEARCHING"
    REVERSING = "REVERSING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    TASK_SUCCESS = "TASK_SUCCESS"
    TASK_FAILED = "TASK_FAILED"

    @property
    def is_mission(self) -> bool:
        return self in (State.MAPPING, State.NAVIGATING, State.SEARCHING)

    @property
    def is_busy(self) -> bool:
        return self in (
            State.RECEIVING_DATA,
            State.INFERENCING,
            State.EXECUTING,
            State.MAPPING,
            State.NAVIGATING,
            State.SEARCHING,
            State.REVERSING,
        )


_MISSION_ENTRY = {
    "start_mapping": State.MAPPING,
    "start_nav": State.NAVIGATING,
    "start_search": State.SEARCHING,
}

_MISSION_BODY = {
    "task_done": State.TASK_SUCCESS,
    "task_failed": State.TASK_FAILED,
    "task_cancel": State.IDLE,
    "pause": State.PAUSED,
    "reverse_req": State.REVERSING,
    "error": State.ERROR,
}

_TRANSITIONS = {
    State.IDLE: {
        "recv_data": State.RECEIVING_DATA,
        "start_task": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
        "error": State.ERROR,
        "reset": State.IDLE,
        **_MISSION_ENTRY,
    },
    State.RECEIVING_DATA: {
        "start_infer": State.INFERENCING,
        "data_timeout": State.IDLE,
        "task_cancel": State.IDLE,
        "reverse_req": State.REVERSING,
        "error": State.ERROR,
        **_MISSION_ENTRY,
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
    State.MAPPING: dict(_MISSION_BODY),
    State.NAVIGATING: dict(_MISSION_BODY),
    State.SEARCHING: dict(_MISSION_BODY),
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
        "reset": State.IDLE,
        **_MISSION_ENTRY,
    },
    State.ERROR: {
        "reset": State.IDLE,
        "reverse_req": State.REVERSING,
    },
    State.TASK_SUCCESS: {
        "reset": State.IDLE,
        "recv_data": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
        **_MISSION_ENTRY,
    },
    State.TASK_FAILED: {
        "reset": State.IDLE,
        "recv_data": State.RECEIVING_DATA,
        "reverse_req": State.REVERSING,
        **_MISSION_ENTRY,
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


def mission_entry_event(mission_type: str) -> str:
    """mission_type -> 进入状态的事件名。"""
    if mission_type == "map":
        return "start_mapping"
    if mission_type in ("search", "explore"):
        return "start_search"
    return "start_nav"
