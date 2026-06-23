"""Tests for driveline_nx (NX-independent layers): params round-trip, engineering
ratings + validation, and the geometry blueprint's structural integrity."""

import json

import pytest

from driveline_nx import blueprint as bp
from driveline_nx import engineering as eng
from driveline_nx.params import DrivelineParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = DrivelineParams()
    p2 = DrivelineParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = DrivelineParams.from_dict({"halfshaft": {"diameter": 34.0}})
    assert p.halfshaft.diameter == 34.0
    assert p.halfshaft.length == DrivelineParams().halfshaft.length  # untouched
    assert p.differential.final_drive_ratio == 9.0


def test_default_halfshaft_has_sound_static_margin():
    """Half-shafts are fatigue-critical: the default must clear a >= 1.5 static SF."""
    g = eng.derive(DrivelineParams())
    assert DrivelineParams().halfshaft.diameter == 36.0
    assert g.halfshaft_safety_factor >= 1.5


def test_overridden_dotted():
    p = DrivelineParams().overridden(**{"differential.type": "elsd",
                                        "wheel_hub.lug_count": 4})
    assert p.differential.type == "elsd"
    assert p.wheel_hub.lug_count == 4


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in DrivelineParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_torque_multiplies_by_ratio():
    p = DrivelineParams()
    g = eng.derive(p)
    assert g.ring_gear_torque_nm == pytest.approx(p.motor_peak_torque_nm * p.differential.final_drive_ratio, rel=1e-6)


def test_diff_type_biases_per_wheel_torque():
    g_open = eng.derive(DrivelineParams().overridden(**{"differential.type": "open"}))
    g_tv = eng.derive(DrivelineParams().overridden(**{"differential.type": "torque_vectoring"}))
    g_spool = eng.derive(DrivelineParams().overridden(**{"differential.type": "spool"}))
    assert g_open.per_wheel_torque_nm == pytest.approx(g_open.ring_gear_torque_nm * 0.5)
    assert g_spool.per_wheel_torque_nm == pytest.approx(g_spool.ring_gear_torque_nm)  # full axle torque
    assert g_open.per_wheel_torque_nm < g_tv.per_wheel_torque_nm < g_spool.per_wheel_torque_nm


def test_wheel_speed_is_motor_over_ratio():
    p = DrivelineParams()
    g = eng.derive(p)
    assert g.wheel_max_speed_rpm == pytest.approx(p.motor_max_speed_rpm / p.differential.final_drive_ratio, rel=1e-6)


def test_hollow_shaft_raises_stress_vs_solid():
    solid = eng.derive(DrivelineParams().overridden(**{"halfshaft.bore_diameter": 0.0}))
    hollow = eng.derive(DrivelineParams().overridden(**{"halfshaft.bore_diameter": 18.0}))
    assert hollow.halfshaft_shear_stress_mpa > solid.halfshaft_shear_stress_mpa


def test_default_design_is_buildable():
    assert eng.validate(DrivelineParams()) == []


def test_validate_catches_lug_off_flange():
    p = DrivelineParams().overridden(**{"wheel_hub.lug_pcd": 200.0})  # > flange OD 150
    issues = eng.validate(p)
    assert any("lug bolt circle" in i for i in issues)


def test_validate_catches_thick_halfshaft():
    p = DrivelineParams().overridden(**{"halfshaft.diameter": 80.0})  # > side gear 62
    assert any("halfshaft.diameter" in i for i in eng.validate(p))


def test_bearing_life_positive():
    est = eng.bearing_life_estimate(DrivelineParams())
    assert est["L10_hours_est"] > 0


# --------------------------------------------------------------------------- #
# blueprint
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(DrivelineParams())
    assert blue["schema"] == "driveline_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 10


def test_both_sides_modelled_symmetric():
    blue = bp.generate(DrivelineParams())  # sides="both"
    ids = [s["id"] for s in blue["build_steps"]]
    assert any(i.endswith("_l") for i in ids) and any(i.endswith("_r") for i in ids)
    assert "halfshaft_l" in ids and "halfshaft_r" in ids


def test_single_side_halves_the_chain():
    both = bp.generate(DrivelineParams())
    left = bp.generate(DrivelineParams().overridden(sides="left"))
    assert len(left["build_steps"]) < len(both["build_steps"])
    assert "halfshaft_r" not in [s["id"] for s in left["build_steps"]]


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(DrivelineParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_lug_holes_present_and_patterned():
    blue = bp.generate(DrivelineParams())
    lugs = [s for s in blue["build_steps"] if s["role"] == "lug_cut"]
    assert lugs, "expected wheel lug holes"
    assert any(s["pattern_count"] == DrivelineParams().wheel_hub.lug_count for s in lugs)


def test_centre_nut_replaces_lug_circle():
    p = DrivelineParams().overridden(**{"wheel_hub.single_centre_nut": True})
    blue = bp.generate(p)
    ids = [s["id"] for s in blue["build_steps"]]
    assert any(i.startswith("hub_centre_nut") for i in ids)
    assert not any(i.startswith("hub_lug") for i in ids)


def test_solid_shaft_has_no_bore_cut():
    p = DrivelineParams().overridden(**{"halfshaft.bore_diameter": 0.0})
    blue = bp.generate(p)
    assert not any(s["role"] == "halfshaft_bore_cut" for s in blue["build_steps"])


def test_input_flange_carries_motor_bore_and_keyway():
    blue = bp.generate(DrivelineParams())
    roles = {s["role"] for s in blue["build_steps"]}
    assert "input_bore_cut" in roles and "input_keyway_cut" in roles
