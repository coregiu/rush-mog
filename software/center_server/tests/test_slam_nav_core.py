"""SLAM / 导航 / 目标搜索核心模块单元测试（不依赖 ROS 运行时）。"""

import math
import struct
from collections import deque

import numpy as np
import pytest

from center_server.core import base_protocol as proto
from center_server.core.imu_monitor import HeadingEstimator, ImuMonitor
from center_server.core.instruction_parser import InstructionParser
from center_server.core.nav_supervisor import (
    DirectController,
    Goal,
    GoalValidator,
    ProgressMonitor,
)
from center_server.core.object_search import SearchConfig, SearchCoordinator, SearchState
from center_server.core.occupancy_map import Frontier, OccupancyMap
from center_server.core.odometry import MecanumGeometry, WheelOdometry
from center_server.core.pose_utils import Pose2D, pose_from_msg, wrap_angle
from center_server.core.state_machine import State, TaskStateMachine
from center_server.core.target_localizer import (
    CameraModel,
    TargetLocalizer,
    approach_pose,
    lidar_distance,
)
from center_server.core.target_tracker import DetectionInput, TargetTracker, Track
from center_server.core.velocity_mux import VelocityMux
from center_server.core.vla_engine import VLAEngine
from center_server.drivers.icm20948 import Calibration, ICM20948
from center_server.core.data_preprocessor import SyncedDTO


# --------------------------------------------------------------- instruction
def test_instruction_parser_missions():
    parser = InstructionParser(
        {
            "object_labels": {"cup": ["杯子", "cup"]},
            "waypoints": {
                "kitchen": {"x": 1.2, "y": 0.8, "yaw": 0.0, "aliases": ["厨房"]}
            },
            "home_pose": [0.0, 0.0, 0.0],
        }
    )
    spec = parser.parse("帮我找红色杯子")
    assert spec.mission_type == "search"
    assert spec.target_label == "cup"

    spec = parser.parse("去厨房")
    assert spec.mission_type == "navigate"
    assert spec.waypoint == "kitchen"
    assert spec.x == pytest.approx(1.2)

    assert parser.parse("建图").mission_type == "map"
    assert parser.parse("探索一下").mission_type == "explore"
    assert parser.parse("回家").mission_type == "return_home"
    assert parser.parse("前进").mission_type == "vla"

    spec = parser.parse("去 x=1.5 y=-2.0 yaw=90")
    assert spec.mission_type == "navigate"
    assert spec.x == pytest.approx(1.5)
    assert spec.y == pytest.approx(-2.0)
    assert spec.yaw == pytest.approx(math.pi / 2)


def test_vla_engine_mission_intents():
    engine = VLAEngine(
        {
            "confidence_threshold": 0.6,
            "instruction": {
                "object_labels": {"cup": ["杯子", "cup"]},
                "waypoints": {
                    "kitchen": {"x": 1.0, "y": 2.0, "yaw": 0.0, "aliases": ["厨房"]}
                },
            },
        }
    )

    def dto(text):
        return SyncedDTO(
            image=np.zeros((3, 4, 4), dtype=np.float32),
            laser=np.ones(4, dtype=np.float32),
            audio=np.zeros((4, 1), dtype=np.float32),
            instruction=text,
        )

    intent = engine.predict(dto("找杯子"))
    assert intent.task_type == "search"
    assert intent.object_label == "cup"

    intent = engine.predict(dto("去厨房"))
    assert intent.task_type == "navigate"
    assert intent.goal_x == pytest.approx(1.0)

    intent = engine.predict(dto("前进"))
    assert intent.task_type == "vla"
    assert intent.intent == "forward"


# --------------------------------------------------------------------- map
def _sample_map() -> OccupancyMap:
    data = np.full((20, 20), -1, dtype=np.int8)
    data[5:15, 5:15] = 0
    data[10, 5:15] = 100
    return OccupancyMap(
        width=20,
        height=20,
        resolution=0.1,
        origin=Pose2D(0.0, 0.0, 0.0, "map"),
        data=data,
    )


def test_occupancy_map_geometry_and_frontiers():
    occ = _sample_map()
    assert occ.known_ratio == pytest.approx(100 / 400)
    assert occ.is_free(0.75, 0.75)
    assert occ.is_occupied(0.75, 1.05)
    assert occ.world_to_grid(0.75, 0.75) == (7, 7)
    x, y = occ.grid_to_world(7, 7)
    assert x == pytest.approx(0.75)
    assert y == pytest.approx(0.75)

    frontiers = occ.compute_frontiers(min_size=1)
    assert frontiers
    assert all(isinstance(f, Frontier) for f in frontiers)
    best = occ.select_frontier(Pose2D(0.75, 0.75, 0.0), frontiers, min_size=1)
    assert best is not None
    near = occ.find_free_near(0.75, 0.75, radius_m=0.2)
    assert near is not None and occ.is_free(near.x, near.y)


def test_occupancy_map_save_load_roundtrip(tmp_path):
    occ = _sample_map()
    yaml_path = occ.save(str(tmp_path / "test_map"))
    loaded = OccupancyMap.load(yaml_path)
    assert loaded.width == occ.width
    assert loaded.height == occ.height
    assert loaded.resolution == pytest.approx(occ.resolution)
    assert loaded.known_ratio == pytest.approx(occ.known_ratio)
    assert loaded.is_occupied(0.75, 1.05)
    assert loaded.is_free(0.75, 0.75)


# --------------------------------------------------------------- localizer
def test_camera_and_lidar_localization():
    cam = CameraModel(width=640, height=480, hfov_deg=60.0)
    assert cam.bearing_from_u(cam.cx) == pytest.approx(0.0, abs=1e-6)
    assert cam.bearing_from_u(640.0) > 0.0

    ranges = [5.0] * 360
    index = 180  # angle 0
    ranges[index] = 1.5
    distance, count = lidar_distance(ranges, -math.pi, 2 * math.pi / 360, 0.0)
    assert count >= 1
    assert distance == pytest.approx(1.5, abs=0.2)

    loc = TargetLocalizer(
        camera_cfg={"width": 640, "height": 480, "hfov_deg": 60.0},
        targets_cfg={"cup": {"real_height_m": 0.12}},
    )
    est = loc.estimate(
        "cup",
        0.9,
        (0.45, 0.4, 0.55, 0.6),
        scan={
            "ranges": ranges,
            "angle_min": -math.pi,
            "angle_increment": 2 * math.pi / 360,
            "range_min": 0.05,
            "range_max": 12.0,
        },
        robot_pose=Pose2D(0.0, 0.0, 0.0, "map"),
    )
    assert est.distance_source == "lidar"
    assert est.position_frame == "map"
    assert est.x == pytest.approx(1.5, abs=0.3)
    assert abs(est.y) < 0.2


def test_bbox_height_distance_and_approach():
    loc = TargetLocalizer(
        camera_cfg={"width": 640, "height": 480, "hfov_deg": 60.0},
        targets_cfg={"cup": {"real_height_m": 0.12}},
        prefer_lidar=False,
    )
    est = loc.estimate("cup", 0.9, (0.45, 0.30, 0.55, 0.50))
    assert est.distance_source == "bbox_height"
    assert est.distance_m > 0.0

    goal = approach_pose((2.0, 0.0), Pose2D(0.0, 0.0, 0.0, "map"), standoff=0.6)
    assert goal.x == pytest.approx(1.4)
    assert goal.y == pytest.approx(0.0)
    assert goal.yaw == pytest.approx(0.0)


def test_target_tracker_iou_and_best():
    tracker = TargetTracker(iou_threshold=0.2, max_age_sec=1.0)
    det = DetectionInput(
        label="cup",
        confidence=0.8,
        bbox=(0.4, 0.4, 0.6, 0.6),
        position=(1.0, 2.0),
        position_frame="map",
    )
    tracks = tracker.update([det], now=1.0)
    assert len(tracks) == 1
    track_id = tracks[0].track_id
    tracker.update([det], now=1.1)
    best = tracker.best("cup", min_confidence=0.5)
    assert best is not None and best.track_id == track_id
    assert best.hits == 2
    assert best.stable(0.1)


# -------------------------------------------------------------------- search
def _track(track_id, label, x, y, hits=5, now=10.0, conf=0.9):
    track = Track(
        track_id=track_id,
        label=label,
        bbox=(0.4, 0.4, 0.6, 0.6),
        confidence=conf,
        position=(x, y),
        position_frame="map",
        hits=hits,
        first_seen=now - 1.0,
        last_seen=now,
    )
    track.history.extend([(x, y)] * 3)
    return track


def test_search_coordinator_explores_then_finds():
    cfg = SearchConfig(confirm_hits=3, confirm_time=0.3, standoff_distance=0.5, replan_delay=0.0)
    coord = SearchCoordinator(cfg, target_label="cup", now=0.0, max_duration=100.0)
    coord.update_pose(Pose2D(0.0, 0.0, 0.0, "map"))
    coord.update_map(0.2, [Frontier(2.0, 0.0, 10)], now=0.0)

    decision = coord.decide(1.0)
    assert decision.action == "navigate"
    assert decision.goal is not None

    coord.update_nav(active=False, result="succeeded", goal=decision.goal, now=1.5)
    coord.update_track(_track("trk_1", "cup", 2.0, 0.0, now=2.0))
    decision = coord.decide(2.0)
    assert decision.action == "approach"  # 已确认，前往接近点
    assert decision.goal is not None and decision.goal.x < 2.0

    coord.update_pose(Pose2D(1.45, 0.0, 0.0, "map"))
    coord.update_track(_track("trk_1", "cup", 2.0, 0.0, now=2.5))
    decision = coord.decide(2.5)
    assert decision.action == "found"
    assert coord.state == SearchState.FOUND


def test_search_coordinator_blacklists_failed_frontier():
    cfg = SearchConfig(replan_delay=0.0, blacklist_sec=30.0)
    coord = SearchCoordinator(cfg, target_label="cup", now=0.0, max_duration=100.0)
    coord.update_pose(Pose2D(0.0, 0.0, 0.0, "map"))
    coord.update_map(0.2, [Frontier(2.0, 0.0, 10), Frontier(0.0, 3.0, 8)], now=0.0)
    decision = coord.decide(1.0)
    first = decision.goal
    coord.update_nav(active=False, result="failed", goal=first, now=1.2)
    decision = coord.decide(1.5)
    assert decision.action == "navigate"
    assert (decision.goal.x, decision.goal.y) != (first.x, first.y)


def test_search_coordinator_explore_only_success():
    cfg = SearchConfig(explore_coverage=0.5, replan_delay=0.0)
    coord = SearchCoordinator(
        cfg, target_label="", now=0.0, max_duration=100.0, explore_only=True
    )
    coord.update_pose(Pose2D(0.0, 0.0, 0.0, "map"))
    coord.update_map(0.6, [], now=0.0)
    decision = coord.decide(1.0)
    assert decision.action == "found"


# --------------------------------------------------------------------- nav
def test_direct_controller_and_validator():
    ctrl = DirectController({"max_linear_speed": 0.3, "max_angular_speed": 1.0})
    cmd = ctrl.compute(Pose2D(0, 0, 0), Goal(1.0, 0.0, 0.0))
    assert cmd.vx > 0.0
    assert not cmd.done

    cmd = ctrl.compute(Pose2D(0, 0, 0), Goal(0.05, 0.0, 0.0, tolerance_xy=0.1))
    assert cmd.done

    cmd = ctrl.compute(Pose2D(0, 0, 0), Goal(0.0, 1.0, math.pi / 2))
    assert abs(cmd.vx) < 1e-9
    assert cmd.wz > 0.0

    validator = GoalValidator({"min_localization_quality": 0.5})
    ok, _ = validator.validate(Goal(0.0, 0.0), map_ready=False, localization_quality=1.0)
    assert not ok
    ok, _ = validator.validate(Goal(0.0, 0.0), map_ready=True, localization_quality=0.1)
    assert not ok
    ok, _ = validator.validate(Goal(0.0, 0.0), map_ready=True, localization_quality=0.9)
    assert ok


def test_progress_monitor_detects_stuck():
    monitor = ProgressMonitor(stuck_timeout=1.0, min_progress=0.1)
    monitor.reset(5.0, now=0.0)
    assert not monitor.update(5.0, now=0.5)
    assert monitor.update(5.0, now=1.1)
    assert not monitor.update(4.0, now=1.2)


# --------------------------------------------------------------------- IMU
def test_imu_monitor_heading_and_quality():
    monitor = ImuMonitor({"expected_rate_hz": 100.0, "max_gap_sec": 0.2})
    yaw = 0.0
    for i in range(100):
        status = monitor.update(
            (0.0, 0.0, 9.81), (0.0, 0.0, 0.1), (30.0, 0.0, 0.0), now=i * 0.01
        )
        yaw = status.yaw_rad
    assert status.rate_hz > 50.0
    assert status.quality > 0.5
    assert yaw > 0.02
    monitor.mark_stale(now=2.0)
    assert not monitor.status.online


def test_heading_estimator_mag_correction():
    est = HeadingEstimator(mag_blend=0.5)
    est.reset(0.0)
    for _ in range(20):
        est.update(0.0, 0.1, mag_yaw=1.0, mag_ok=True)
    assert abs(wrap_angle(est.yaw - 1.0)) < 0.1


# --------------------------------------------------------------------- mux
def test_velocity_mux_priority_timeout_and_estop():
    from geometry_msgs.msg import Twist

    mux = VelocityMux(
        default_timeout=0.5,
        priorities={"safety": 0, "nav": 3, "vla": 4},
    )
    vla = Twist()
    vla.linear.x = 0.2
    nav = Twist()
    nav.linear.x = -0.3
    mux.set("vla", vla, now=0.0)
    mux.set("nav", nav, now=0.0)
    out, source, active, estop = mux.compute(0.1)
    assert source == "nav"
    assert out.linear.x == pytest.approx(-0.3)
    assert set(active) == {"vla", "nav"}
    assert not estop

    safety = Twist()
    mux.set("safety", safety, now=0.2)
    out, source, _, _ = mux.compute(0.3)
    assert source == "safety"
    assert out.linear.x == 0.0

    out, source, _, _ = mux.compute(2.0)
    assert source == "none"
    assert out.linear.x == 0.0

    mux.set_emergency(True, "test")
    out, source, _, estop = mux.compute(2.1)
    assert estop and source == "emergency"
    assert out.linear.x == 0.0


# ---------------------------------------------------------------- odometry
def test_mecanum_roundtrip_and_integration():
    geom = MecanumGeometry(wheel_radius=0.05, wheelbase_x=0.2, track_y=0.18)
    speeds = geom.twist_to_wheel_speeds(0.3, 0.1, 0.2)
    vx, vy, wz = geom.wheel_speeds_to_twist(speeds)
    assert vx == pytest.approx(0.3)
    assert vy == pytest.approx(0.1)
    assert wz == pytest.approx(0.2)

    odom = WheelOdometry(geometry=geom)
    for _ in range(10):
        odom.update_twist(0.2, 0.0, 0.0, 0.1)
    assert odom.x == pytest.approx(0.2, abs=1e-6)
    assert odom.y == pytest.approx(0.0, abs=1e-6)
    assert odom.yaw == pytest.approx(0.0, abs=1e-6)

    odom.reset()
    for _ in range(10):
        odom.update_twist(0.0, 0.0, 0.5, 0.1)
    assert odom.yaw == pytest.approx(0.5, abs=1e-6)


# ----------------------------------------------------------------- protocol
def test_crc_and_protocol_roundtrip():
    assert proto.crc16_ccitt(b"123456789") == 0x29B1

    frame = proto.encode_frame(proto.TYPE_PING, b"", seq=7)
    buffer = bytearray(frame[:3])
    assert proto.decode_stream(buffer) == []
    buffer.extend(frame[3:])
    frames = proto.decode_stream(buffer)
    assert len(frames) == 1
    assert frames[0].msg_type == proto.TYPE_PING
    assert frames[0].seq == 7
    assert buffer == bytearray()

    payload = struct.pack(
        "<I4i4hHhBI", 1, 10, -20, 30, -40, 100, -200, 300, -400, 12000, -500, 0x0A, 12345
    )
    tel_frame = proto.encode_frame(proto.TYPE_TELEMETRY, payload)
    tel = proto.decode_telemetry(proto.decode_stream(bytearray(tel_frame))[0].payload)
    assert tel.encoders == (10, -20, 30, -40)
    assert tel.wheel_speeds[0] == pytest.approx(0.1)
    assert tel.battery_mv == 12000
    assert tel.uptime_ms == 12345
    assert tel.armed and not tel.estop

    cmd = proto.encode_cmd_vel(0.25, -0.5, estop=True, seq=3)
    frames = proto.decode_stream(bytearray(cmd))
    assert len(frames) == 1
    assert frames[0].msg_type == proto.TYPE_CMD_VEL
    vx, wz, flags = proto._CMD_VEL.unpack(frames[0].payload)
    assert vx == 250 and wz == -500
    assert flags & proto.FLAG_ESTOP


# --------------------------------------------------------------- ICM-20948
class FakeBus:
    def __init__(self):
        self.registers = {}
        self.writes = []
        self.who = 0xEA
        self.ak_who = 0x09
        self.imu_data = bytearray(14)
        # accel x = 8192 (1g @ ±4g), gyro z = 655 (1000dps/32.8 ≈ 30.5? use 32)
        struct.pack_into(">h", self.imu_data, 0, 8192)
        struct.pack_into(">h", self.imu_data, 6, 0)      # gyro x
        struct.pack_into(">h", self.imu_data, 8, 0)      # gyro y
        struct.pack_into(">h", self.imu_data, 10, 328)   # gyro z -> 10 dps
        struct.pack_into(">h", self.imu_data, 12, 0)     # temp
        self.mag_data = bytearray(9)
        self.mag_data[0] = 0x01
        self.mag_data[8] = 0x00
        struct.pack_into(">h", self.mag_data, 1, 100)    # 100 * 0.15 = 15 uT
        struct.pack_into(">h", self.mag_data, 3, 0)
        struct.pack_into(">h", self.mag_data, 5, 0)

    def write_byte_data(self, addr, reg, value):
        self.writes.append((addr, reg, value))
        self.registers[(addr, reg)] = value

    def read_byte_data(self, addr, reg):
        if reg == 0x00 and addr == 0x68:
            return self.who
        return self.registers.get((addr, reg), 0)

    def read_i2c_block_data(self, addr, reg, length):
        if addr == 0x68 and reg == 0x00 and length == 1:
            return [self.who]
        if addr == 0x0C and reg == 0x01 and length == 1:
            return [self.ak_who]
        if addr == 0x68 and reg == 0x2D and length == 14:
            return list(self.imu_data)
        if addr == 0x0C and reg == 0x10 and length == 9:
            return list(self.mag_data)
        return [0] * length

    def close(self):
        pass


def test_pose_from_msg_variants():
    from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    yaw = 0.5
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    pose = Pose()
    pose.position.x, pose.position.y = 1.0, 2.0
    pose.orientation.z, pose.orientation.w = qz, qw
    parsed = pose_from_msg(pose, frame_id="map")
    assert (parsed.x, parsed.y) == pytest.approx((1.0, 2.0))
    assert parsed.yaw == pytest.approx(yaw)

    stamped = PoseStamped()
    stamped.header.frame_id = "map"
    stamped.pose = pose
    parsed = pose_from_msg(stamped)
    assert parsed.frame_id == "map"
    assert parsed.yaw == pytest.approx(yaw)

    with_cov = PoseWithCovarianceStamped()
    with_cov.header.frame_id = "map"
    with_cov.pose.pose = pose
    parsed = pose_from_msg(with_cov)
    assert (parsed.x, parsed.y) == pytest.approx((1.0, 2.0))

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.pose.pose = pose
    parsed = pose_from_msg(odom)
    assert parsed.frame_id == "odom"
    assert parsed.yaw == pytest.approx(yaw)


def test_state_machine_mission_flow():
    sm = TaskStateMachine()
    assert sm.transition("start_mapping")
    assert sm.state == State.MAPPING
    assert sm.transition("task_done")
    assert sm.state == State.TASK_SUCCESS
    assert sm.transition("start_search")
    assert sm.state == State.SEARCHING
    assert sm.transition("reverse_req")
    assert sm.state == State.REVERSING
    assert sm.transition("reverse_done")
    assert sm.state == State.IDLE


def test_icm20948_driver_scaling_and_bank_switch():
    bus = FakeBus()
    driver = ICM20948(bus, address=0x68, accel_fs=1, gyro_fs=2, sample_rate_hz=100.0)
    assert driver.begin()
    sample = driver.read_sample()
    assert sample.accel[0] == pytest.approx(9.80665, rel=1e-3)
    assert sample.gyro[2] == pytest.approx(math.radians(10.0), rel=1e-3)
    assert sample.mag[0] == pytest.approx(15.0, rel=1e-3)
    assert sample.mag_valid
    # Bank select register 0x7F must have been written with bank 2 at least once
    assert any(reg == 0x7F and (value >> 4) == 2 for _, reg, value in bus.writes)

    cal = Calibration(gyro_bias=(0.1, 0.0, 0.0), mag_bias=(5.0, 0.0, 0.0))
    driver2 = ICM20948(bus, address=0x68, calibration=cal)
    driver2.begin()
    sample = driver2.read_sample()
    assert sample.gyro[0] == pytest.approx(-0.1 + math.radians(0.0), abs=0.12)
    assert sample.mag[0] == pytest.approx(10.0, rel=1e-3)
