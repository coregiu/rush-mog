from edge_server.core.body_drive import BodyDriveConfig, twist_to_body_command

CFG = BodyDriveConfig(
    max_linear=0.35,
    max_angular=1.0,
    deadband_linear=0.02,
    deadband_angular=0.05,
    max_level=9,
    twist_mode="priority",
    arc_period_sec=0.5,
)


def test_stop_and_deadband():
    assert twist_to_body_command(0.0, 0.0, 0.0, CFG) == "0"
    assert twist_to_body_command(0.01, 0.0, 0.0, CFG) == "0"
    assert twist_to_body_command(0.0, 0.0, 0.04, CFG) == "0"


def test_translation_directions():
    assert twist_to_body_command(0.2, 0.0, 0.0, CFG) == "15"
    assert twist_to_body_command(-0.2, 0.0, 0.0, CFG) == "25"
    assert twist_to_body_command(0.0, 0.2, 0.0, CFG) == "35"
    assert twist_to_body_command(0.0, -0.2, 0.0, CFG) == "45"
    assert twist_to_body_command(0.2, 0.2, 0.0, CFG) == "57"
    assert twist_to_body_command(0.2, -0.2, 0.0, CFG) == "67"
    assert twist_to_body_command(-0.2, 0.2, 0.0, CFG) == "77"
    assert twist_to_body_command(-0.2, -0.2, 0.0, CFG) == "87"


def test_turn_directions():
    assert twist_to_body_command(0.0, 0.0, 0.6, CFG) == "95"
    assert twist_to_body_command(0.0, 0.0, -0.6, CFG) == "A5"


def test_priority_mode():
    # 平移归一化更大 -> 平移；转向归一化更大 -> 转向。
    assert twist_to_body_command(0.2, 0.0, 0.3, CFG) == "15"
    assert twist_to_body_command(0.05, 0.0, 0.8, CFG) == "97"


def test_time_share_mode():
    cfg = BodyDriveConfig(
        max_linear=0.35,
        max_angular=1.0,
        twist_mode="time_share",
        arc_period_sec=0.5,
    )
    assert twist_to_body_command(0.2, 0.0, 0.6, cfg, now=0.0) == "15"
    assert twist_to_body_command(0.2, 0.0, 0.6, cfg, now=0.3) == "95"
