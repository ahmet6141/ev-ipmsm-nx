"""Tests for suspension_nx (NX-independent layers): params round-trip, engineering
rates + validation, and the geometry blueprint's structural integrity."""

import json

import pytest

from suspension_nx import blueprint as bp
from suspension_nx import engineering as eng
from suspension_nx.params import SuspensionParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = SuspensionParams()
    p2 = SuspensionParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = SuspensionParams.from_dict({"spring": {"spring_rate_n_per_mm": 50.0}})
    assert p.spring.spring_rate_n_per_mm == 50.0
    assert p.spring.free_length_mm == SuspensionParams().spring.free_length_mm  # untouched
    assert p.geometry.track_width_mm == 1580.0


def test_overridden_dotted():
    p = SuspensionParams().overridden(**{"geometry.type": "macpherson",
                                         "antiroll.enabled": False})
    assert p.geometry.type == "macpherson"
    assert p.antiroll.enabled is False


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in SuspensionParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_wheel_rate_is_spring_rate_times_mr_squared():
    p = SuspensionParams()
    g = eng.derive(p)
    expected = p.spring.spring_rate_n_per_mm * p.spring.motion_ratio ** 2
    assert g.wheel_rate_n_per_mm == pytest.approx(expected, rel=1e-6)


def test_ride_frequency_in_sane_band():
    g = eng.derive(SuspensionParams())
    assert 0.8 <= g.ride_frequency_hz <= 2.0


def test_softer_spring_lowers_ride_frequency():
    soft = eng.derive(SuspensionParams().overridden(**{"spring.spring_rate_n_per_mm": 30.0}))
    stiff = eng.derive(SuspensionParams().overridden(**{"spring.spring_rate_n_per_mm": 60.0}))
    assert soft.ride_frequency_hz < stiff.ride_frequency_hz


def test_wheel_hop_frequency_in_realistic_band():
    # unsprung-mass hop: tyre + wheel rate act in PARALLEL -> ~8-16 Hz for a
    # passenger car (a series model collapses this to an unrealistic ~3 Hz).
    g = eng.derive(SuspensionParams())
    assert 8.0 <= g.wheel_hop_frequency_hz <= 16.0


def test_default_design_is_buildable():
    assert eng.validate(SuspensionParams()) == []


def test_validate_catches_arm_longer_than_half_track():
    p = SuspensionParams().overridden(**{"geometry.lower_arm_length_mm": 900.0})  # > 1580/2
    issues = eng.validate(p)
    assert any("lower_arm_length_mm" in i for i in issues)


def test_validate_catches_spring_deflection_over_free_length():
    p = SuspensionParams().overridden(**{"spring.ride_height_load_n": 99999.0})  # huge deflection
    issues = eng.validate(p)
    assert any("free_length" in i for i in issues)


# --------------------------------------------------------------------------- #
# blueprint
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(SuspensionParams())
    assert blue["schema"] == "suspension_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 5


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(SuspensionParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_axle_has_more_steps_than_one_corner():
    one = bp.generate(SuspensionParams())
    axle = bp.generate(SuspensionParams().overridden(corners="axle"))
    assert len(axle["build_steps"]) > len(one["build_steps"])


def test_axle_models_both_corners():
    blue = bp.generate(SuspensionParams().overridden(corners="axle"))
    ids = [s["id"] for s in blue["build_steps"]]
    assert any(i.endswith("_l") for i in ids) and any(i.endswith("_r") for i in ids)
    assert "knuckle_l" in ids and "knuckle_r" in ids


def test_macpherson_drops_the_upper_arm():
    blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": "macpherson"}))
    ids = [s["id"] for s in blue["build_steps"]]
    assert not any(i.startswith("upper_arm") for i in ids)


def test_knuckle_carries_a_hub_bore():
    blue = bp.generate(SuspensionParams())
    assert any(s["role"] == "hub_bore_cut" for s in blue["build_steps"])
