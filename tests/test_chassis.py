"""Tests for chassis_nx (NX-independent layers): params round-trip, engineering
mass/stiffness + validation, and the geometry blueprint's structural integrity."""

import json

import pytest

from chassis_nx import blueprint as bp
from chassis_nx import engineering as eng
from chassis_nx.params import ChassisParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = ChassisParams()
    p2 = ChassisParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = ChassisParams.from_dict({"frame": {"rail_height_mm": 130.0}})
    assert p.frame.rail_height_mm == 130.0
    assert p.frame.rail_width_mm == ChassisParams().frame.rail_width_mm  # untouched
    assert p.battery_tray.enabled is True


def test_overridden_dotted():
    p = ChassisParams().overridden(**{"frame.crossmember_count": 7,
                                      "battery_tray.enabled": False})
    assert p.frame.crossmember_count == 7
    assert p.battery_tray.enabled is False


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in ChassisParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_mass_estimate_positive():
    g = eng.derive(ChassisParams())
    assert g.total_mass_kg > 0
    assert g.rail_mass_kg > 0 and g.battery_tray_mass_kg > 0


def test_mass_increases_with_rail_height():
    base = eng.derive(ChassisParams())
    taller = eng.derive(ChassisParams().overridden(**{"frame.rail_height_mm": 160.0}))
    assert taller.total_mass_kg > base.total_mass_kg


def test_torsional_stiffness_positive():
    g = eng.derive(ChassisParams())
    assert g.torsional_stiffness_nm_per_deg > 0


def test_default_design_is_buildable():
    assert eng.validate(ChassisParams()) == []


def test_validate_catches_rails_wider_than_track():
    p = ChassisParams().overridden(**{"frame.frame_inner_width_mm": 1600.0})  # +2*rail>track
    issues = eng.validate(p)
    assert any("track" in i for i in issues)


def test_validate_catches_thick_rail_wall():
    p = ChassisParams().overridden(**{"frame.rail_wall_mm": 40.0})  # 2*40 > 70 width
    assert any("rail_wall" in i for i in eng.validate(p))


def test_mass_breakdown_has_total_and_torsion():
    est = eng.mass_breakdown(ChassisParams())
    assert est["total_structural_mass_kg"] > 0
    assert est["torsional_stiffness_Nm_per_deg_est"] > 0


# --------------------------------------------------------------------------- #
# blueprint
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(ChassisParams())
    assert blue["schema"] == "chassis_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 10


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(ChassisParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_battery_tray_present_when_enabled():
    blue = bp.generate(ChassisParams())  # enabled by default
    ids = [s["id"] for s in blue["build_steps"]]
    assert "battery_tray" in ids


def test_battery_tray_absent_when_disabled():
    p = ChassisParams().overridden(**{"battery_tray.enabled": False})
    blue = bp.generate(p)
    ids = [s["id"] for s in blue["build_steps"]]
    assert "battery_tray" not in ids
    assert not any(i.startswith("battery_") for i in ids)


def test_two_frame_rails_present():
    blue = bp.generate(ChassisParams())
    ids = [s["id"] for s in blue["build_steps"]]
    assert "rail_l" in ids and "rail_r" in ids
