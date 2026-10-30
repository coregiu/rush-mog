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

class TaskStateMachine:
    def __init__(self):
        self.state = State.IDLE

    def transition(self, event: str) -> bool:
        allowed = {
            self.state: event
        }
        if self.state == State.IDLE and event == "recv_data":
            self.state = State.RECEIVING_DATA
            return True
        if self.state == State.RECEIVING_DATA and event == "start_infer":
            self.state = State.INFERENCING
            return True
        if self.state == State.INFERENCING and event == "infer_ok":
            self.state = State.EXECUTING
            return True
        if event == "reverse_req":
            self.state = State.REVERSING
            return True
        if event == "task_done":
            self.state = State.TASK_SUCCESS
            return True
        if event == "error":
            self.state = State.ERROR
            return True
        return False
