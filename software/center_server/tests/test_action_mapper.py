import math

import pytest

from center_server.action_mapper import ActionMapper
from center_server.errors import UnprocessableEntity


def test_base_twist_normalized_and_clipped():
    mapper = ActionMapper(
        {
            "mode": "base_twist",
            "normalized": True,
            "max_linear_x": 0.5,
            "max_linear_y": 0.4,
            "max_angular_z": 1.0,
        }
    )
    result = mapper.map([0.2, -0.5, 1.5])
    assert result["base_cmd"]["linear"]["x"] == pytest.approx(0.1)
    assert result["base_cmd"]["linear"]["y"] == pytest.approx(-0.2)
    assert result["base_cmd"]["angular"]["z"] == pytest.approx(1.0)
    assert result["clipped"] is True


def test_base_twist_direct_units_and_invert():
    mapper = ActionMapper(
        {
            "mode": "base_twist",
            "normalized": False,
            "max_linear_x": 0.5,
            "max_linear_y": 0.4,
            "max_angular_z": 1.0,
            "invert": [True, False, True],
        }
    )
    result = mapper.map([0.9, 0.0, 0.2])
    assert result["base_cmd"]["linear"]["x"] == pytest.approx(-0.5)
    assert result["base_cmd"]["angular"]["z"] == pytest.approx(-0.2)
    assert result["clipped"] is True


def test_deadband():
    mapper = ActionMapper(
        {
            "mode": "base_twist",
            "normalized": True,
            "max_linear_x": 0.5,
            "deadband": 0.05,
        }
    )
    result = mapper.map([0.05, 0.0, 0.0])
    assert result["base_cmd"]["linear"]["x"] == 0.0


def test_hybrid_maps_arm_and_applies_limits():
    mapper = ActionMapper(
        {
            "mode": "hybrid",
            "normalized": True,
            "max_linear_x": 0.5,
            "arm_start": 3,
            "joint_names": ["j1", "j2"],
            "joint_limits": {"j2": [-1.0, 1.0]},
        }
    )
    result = mapper.map([0.2, 0.0, 0.0, 0.5, 2.0])
    assert result["base_cmd"]["linear"]["x"] == pytest.approx(0.1)
    assert result["arm_cmd"]["j1"] == pytest.approx(0.5)
    assert result["arm_cmd"]["j2"] == pytest.approx(1.0)
    assert result["clipped"] is True


def test_joint_mode():
    mapper = ActionMapper({"mode": "joint", "joint_names": ["j1", "j2"]})
    result = mapper.map([0.1, 0.2])
    assert result["base_cmd"] is None
    assert result["arm_cmd"] == {"j1": 0.1, "j2": 0.2}


def test_raw_mode_keeps_values():
    mapper = ActionMapper({"mode": "raw"})
    result = mapper.map([1.0, 2.0, 3.0])
    assert result["base_cmd"] is None
    assert result["arm_cmd"] is None
    assert result["raw"] == [1.0, 2.0, 3.0]


@pytest.mark.parametrize("bad", [[], [math.nan], [math.inf], ["x"]])
def test_invalid_actions_rejected(bad):
    mapper = ActionMapper({"mode": "raw"})
    with pytest.raises(UnprocessableEntity):
        mapper.map(bad)
