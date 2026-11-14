import math
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edge_server.core.action_planner import ActionPlanner
from edge_server.core.data_preprocessor import MultiModalPreprocessor
from edge_server.core.reverse_handler import ReverseHandler
from edge_server.core.safety_checker import SafetyChecker
from edge_server.core.state_machine import State, TaskStateMachine


def make_intent(intent="forward", value=0.5, target="base", reverse=False):
    return SimpleNamespace(
        task_id="t1",
        target=target,
        intent=intent,
        value=value,
        confidence=0.9,
        reverse_request=reverse,
    )


# --------------------------------------------------------------- state machine
def test_state_machine_normal_flow():
    sm = TaskStateMachine()
    assert sm.state == State.IDLE
    assert sm.transition("recv_data")
    assert sm.state == State.RECEIVING_DATA
    assert sm.transition("start_infer")
    assert sm.state == State.INFERENCING
    assert sm.transition("infer_ok")
    assert sm.state == State.EXECUTING
    assert sm.transition("task_done")
    assert sm.state == State.TASK_SUCCESS
    assert sm.transition("reset")
    assert sm.state == State.IDLE


def test_state_machine_rejects_invalid_transition():
    sm = TaskStateMachine()
    assert not sm.transition("infer_ok")
    assert sm.state == State.IDLE


def test_state_machine_reverse_from_executing():
    sm = TaskStateMachine()
    sm.force(State.EXECUTING)
    assert sm.transition("reverse_req")
    assert sm.state == State.REVERSING
    assert sm.transition("reverse_done")
    assert sm.state == State.IDLE


# --------------------------------------------------------------------- planner
def test_planner_twist_limits_and_directions():
    planner = ActionPlanner(
        {"max_linear_speed": 0.8, "max_angular_speed": 1.0, "acceleration_limit": 0.3}
    )
    assert planner.intent_to_twist(make_intent("forward", 0.5)).linear.x == 0.5
    assert planner.intent_to_twist(make_intent("forward", 5.0)).linear.x == 0.8
    assert planner.intent_to_twist(make_intent("backward", 0.4)).linear.x == -0.4
    assert planner.intent_to_twist(make_intent("turn_left", 0.6)).angular.z == 0.6
    assert planner.intent_to_twist(make_intent("turn_right", 0.6)).angular.z == -0.6
    assert planner.intent_to_twist(make_intent("unknown", 0.6)).linear.x == 0.0
    assert planner.intent_to_twist(make_intent("forward", 0.5, reverse=True)).linear.x == 0.0


def test_planner_acceleration_limit():
    planner = ActionPlanner({"acceleration_limit": 0.5})
    target = Twist()
    target.linear.x = 0.8
    previous = Twist()
    limited = planner.limit_acceleration(target, previous, dt=0.1)
    assert abs(limited.linear.x - 0.05) < 1e-9


def test_planner_hold_trajectory():
    planner = ActionPlanner({})
    traj = planner.make_hold_traj({"j1": 0.1, "j2": 0.2})
    assert isinstance(traj, JointTrajectory)
    assert traj.joint_names == ["j1", "j2"]
    assert list(traj.points[0].positions) == [0.1, 0.2]
    assert planner.make_hold_traj({}) is None
    assert planner.intent_to_arm_traj(make_intent("forward"), {"j1": 0.0}) is None


# ---------------------------------------------------------------------- safety
def make_scan(ranges):
    return np.asarray(ranges, dtype=np.float32), -math.pi, 2.0 * math.pi / len(ranges)


def test_safety_limits_and_obstacle():
    checker = SafetyChecker(
        {
            "obstacle_stop_distance": 0.5,
            "max_twist_x": 0.8,
            "max_twist_z": 1.0,
            "laser_front_fov_deg": 60.0,
        }
    )
    ranges, angle_min, angle_inc = make_scan([2.0] * 360)
    twist = Twist()
    twist.linear.x = 0.3
    assert checker.check_twist(twist, ranges, angle_min, angle_inc)[0]

    # 正前方（角度 0）放置障碍
    ranges[180] = 0.2
    ok, reason = checker.check_twist(twist, ranges, angle_min, angle_inc)
    assert not ok and "obstacle" in reason

    # 正后方障碍不应阻止前进
    ranges[:] = 2.0
    ranges[0] = 0.2
    assert checker.check_twist(twist, ranges, angle_min, angle_inc)[0]

    twist.linear.x = 1.5
    assert not checker.check_twist(twist)[0]
    twist.linear.x = float("nan")
    assert not checker.check_twist(twist)[0]


def test_safety_ignores_inf_and_invalid_ranges():
    checker = SafetyChecker({"obstacle_stop_distance": 0.5})
    twist = Twist()
    twist.linear.x = 0.2
    ranges = np.array([np.inf, np.nan, 0.0, 2.0], dtype=np.float32)
    assert checker.check_twist(twist, ranges)[0]
    assert checker.nearest_obstacle(ranges) == 2.0


def test_safety_arm_joint_limits():
    checker = SafetyChecker({"joint_limits": {"j1": [-1.0, 1.0]}})
    traj = JointTrajectory()
    traj.joint_names = ["j1"]
    point = JointTrajectoryPoint()
    point.positions = [0.5]
    traj.points.append(point)
    assert checker.check_arm_traj(traj)[0]

    point.positions = [2.0]
    ok, reason = checker.check_arm_traj(traj)
    assert not ok and "out of range" in reason


# ------------------------------------------------------------------ preprocess
def test_preprocessor_image_and_audio():
    pre = MultiModalPreprocessor(img_size=(8, 8), n_mels=16)
    image = np.zeros((4, 6, 3), dtype=np.uint8)
    tensor = pre.process_image(image)
    assert tensor.shape == (3, 8, 8)
    assert tensor.dtype == np.float32
    assert 0.0 <= tensor.min() and tensor.max() <= 1.0

    gray = np.zeros((4, 6), dtype=np.uint8)
    assert pre.process_image(gray).shape == (3, 8, 8)

    t = np.arange(1600) / 16000.0
    samples = (np.sin(2 * np.pi * 440 * t) * 0.3 * 32767).astype(np.int16)
    features = pre.process_audio(samples.tobytes(), sample_rate=16000)
    assert features.shape[0] == 16
    assert features.shape[1] >= 1
    assert np.isfinite(features).all()

    assert pre.process_audio(b"").shape == (16, 1)
    assert pre.process_laser([1.0, 2.0]).shape == (2,)


# --------------------------------------------------------------- reverse ops
def test_reverse_handler():
    handler = ReverseHandler(
        {
            "reverse_speed": 0.25,
            "reverse_distance": 0.5,
            "operation_timeout": 5.0,
            "arm_joint_names": ["j1", "j2"],
            "arm_home_positions": [0.0, 1.0],
        }
    )
    assert handler.reverse_duration(0.5) == 2.0
    assert handler.reverse_duration(0.0) == 2.0
    assert handler.reverse_duration(100.0) == 5.0

    twist, traj = handler.handle_backward(None)
    assert twist.linear.x == -0.25
    assert traj is None

    twist, _ = handler.handle_stop(None)
    assert twist.linear.x == 0.0

    twist, traj = handler.handle_arm_reset()
    assert twist.linear.x == 0.0
    assert traj.joint_names == ["j1", "j2"]
    assert list(traj.points[0].positions) == [0.0, 1.0]
