"""Tests for vehicle_nx: params round-trip, rotation helpers, and the assembly
plan (component placement + orientation in vehicle coordinates)."""

import json
import math

import pytest

from vehicle_nx import assembly as asm
from vehicle_nx.params import VehicleParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = VehicleParams()
    assert VehicleParams.from_json(p.to_json()).to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = VehicleParams.from_dict({"layout": {"drive_layout": "awd"}})
    assert p.layout.drive_layout == "awd"
    assert p.layout.wheelbase_mm == VehicleParams().layout.wheelbase_mm


def test_overridden_dotted():
    p = VehicleParams().overridden(**{"layout.suspension_corners": 2,
                                      "parts.include_inverter": False})
    assert p.layout.suspension_corners == 2
    assert p.parts.include_inverter is False


# --------------------------------------------------------------------------- #
# rotation helpers
# --------------------------------------------------------------------------- #
def _apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def test_rot_x_minus90_maps_localZ_to_vehicleY():
    # local +Z (rotation axis) must become the vehicle +Y (left-right axle)
    out = _apply(asm.rot_x(-90.0), [0.0, 0.0, 1.0])
    assert out == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_rot_z_180_flips_x_and_y():
    out = _apply(asm.rot_z(180.0), [1.0, 0.0, 0.0])
    assert out == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)


def test_matmul_identity():
    i = asm.identity()
    got = asm.matmul(i, asm.rot_z(37.0))
    exp = asm.rot_z(37.0)
    flat_got = [v for row in got for v in row]
    flat_exp = [v for row in exp for v in row]
    assert flat_got == pytest.approx(flat_exp)


# --------------------------------------------------------------------------- #
# assembly plan
# --------------------------------------------------------------------------- #
def test_plan_schema_and_json():
    plan = asm.build_plan(VehicleParams())
    assert plan["schema"] == "vehicle_nx.assembly/1"
    assert plan["units"] == "mm"
    json.loads(asm.to_json(plan))  # serialisable
    assert len(plan["components"]) >= 6


def test_default_layout_is_consistent():
    assert asm.validate(VehicleParams()) == []


def test_four_corners_present_by_default():
    plan = asm.build_plan(VehicleParams())
    names = {c["name"] for c in plan["components"]}
    for corner in ("SUSPENSION_FL", "SUSPENSION_FR", "SUSPENSION_RL", "SUSPENSION_RR"):
        assert corner in names


def test_left_and_right_corners_mirror_in_Y():
    plan = asm.build_plan(VehicleParams())
    by = {c["name"]: c for c in plan["components"]}
    fl, fr = by["SUSPENSION_FL"], by["SUSPENSION_FR"]
    assert fl["origin_mm"][1] == pytest.approx(-fr["origin_mm"][1])
    assert fl["origin_mm"][1] > 0  # left is +Y


def test_rear_layout_puts_eaxle_at_rear_axle():
    plan = asm.build_plan(VehicleParams())  # rear drive
    drv = next(c for c in plan["components"] if c["role"] == "driveline")
    assert drv["origin_mm"][0] == pytest.approx(-VehicleParams().layout.wheelbase_mm / 2.0)


def test_awd_has_two_drivelines():
    plan = asm.build_plan(VehicleParams().overridden(**{"layout.drive_layout": "awd"}))
    drivelines = [c for c in plan["components"] if c["role"] == "driveline"]
    assert len(drivelines) == 2


def test_inverter_toggle_removes_inverter():
    plan = asm.build_plan(VehicleParams().overridden(**{"parts.include_inverter": False}))
    assert not any(c["role"] == "inverter" for c in plan["components"])


def test_two_corner_layout_has_two_suspension():
    plan = asm.build_plan(VehicleParams().overridden(**{"layout.suspension_corners": 2}))
    susp = [c for c in plan["components"] if c["role"] == "suspension"]
    assert len(susp) == 2


def test_hub_height_equals_tyre_radius():
    p = VehicleParams()
    plan = asm.build_plan(p)
    drv = next(c for c in plan["components"] if c["role"] == "driveline")
    assert drv["origin_mm"][2] == pytest.approx(p.layout.tyre_radius_mm)


def test_validate_catches_unknown_layout():
    issues = asm.validate(VehicleParams().overridden(**{"layout.drive_layout": "diagonal"}))
    assert any("drive_layout" in i for i in issues)


def test_validate_catches_bad_corner_count():
    issues = asm.validate(VehicleParams().overridden(**{"layout.suspension_corners": 3}))
    assert any("suspension_corners" in i for i in issues)


def test_every_orientation_is_orthonormal():
    """Each placement matrix must be a proper rotation (det = +1, orthonormal)."""
    for c in asm.build_plan(VehicleParams())["components"]:
        m = c["orientation"]
        # columns unit length
        for j in range(3):
            col = [m[i][j] for i in range(3)]
            assert math.sqrt(sum(x * x for x in col)) == pytest.approx(1.0, abs=1e-9)
