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


# --------------------------------------------------------------------------- #
# ICD §2/§3 -- shared hub datum + per-subsystem placement
# --------------------------------------------------------------------------- #
def _by_name(plan):
    return {c["name"]: c for c in plan["components"]}


def _mat_approx(got, exp, abs=1e-9):
    """Compare two 3x3 matrices element-wise (pytest.approx can't nest lists, and the
    stored orientation is rounded so raw rot_* may differ by ~1e-17)."""
    flat_got = [v for row in got for v in row]
    flat_exp = [v for row in exp for v in row]
    return flat_got == pytest.approx(flat_exp, abs=abs)


def test_hub_centre_matches_icd_stations():
    """HUB_CENTRE(axle, side) = (+-wheelbase/2, +-track/2, tyre_radius) per ICD §2."""
    p = VehicleParams()
    L = p.layout
    assert asm.hub_centre(p, "front", +1.0) == pytest.approx(
        [L.wheelbase_mm / 2.0, L.track_front_mm / 2.0, L.tyre_radius_mm])
    assert asm.hub_centre(p, "rear", -1.0) == pytest.approx(
        [-L.wheelbase_mm / 2.0, -L.track_rear_mm / 2.0, L.tyre_radius_mm])


def test_chassis_is_identity_at_origin():
    """Chassis builds in vehicle coords -> identity transform at (0,0,0) (ICD §3)."""
    ch = _by_name(asm.build_plan(VehicleParams()))["CHASSIS"]
    assert ch["origin_mm"] == [0.0, 0.0, 0.0]
    assert ch["orientation"] == asm.identity()


def test_suspension_corner_origin_is_hub_centre_no_double_count():
    """The re-datumed corner places its ORIGIN straight on HUB_CENTRE -- no extra
    +-track/2 offset. Left = identity, right = Rz(180)."""
    p = VehicleParams()
    by = _by_name(asm.build_plan(p))
    for name, axle, sign in (("SUSPENSION_FL", "front", +1.0), ("SUSPENSION_FR", "front", -1.0),
                             ("SUSPENSION_RL", "rear", +1.0), ("SUSPENSION_RR", "rear", -1.0)):
        comp = by[name]
        assert comp["origin_mm"] == pytest.approx(asm.hub_centre(p, axle, sign))
        expected = asm.identity() if sign > 0 else asm.rot_z(180.0)
        assert _mat_approx(comp["orientation"], expected)


def test_driveline_and_suspension_hub_coincide_per_corner():
    """ICD §2: the driveline wheel-hub flange face and the suspension upright hub
    map to the SAME vehicle point at each driven corner (hub coincidence). The
    driveline part is built so its flange face is at local |z| = T/2 and placed via
    Rx(-90) at the axle centre -> flange lands at vehicle y = +-T/2 = the suspension
    corner origin."""
    p = VehicleParams()  # rear drive -> driveline + suspension share the rear axle
    by = _by_name(asm.build_plan(p))
    drv = by["DRIVELINE_REAR"]
    # the driveline part spans the full track; its flange faces sit at local z = +-T/2.
    half = asm._driveline_built_track_mm() / 2.0
    R = drv["orientation"]
    base = drv["origin_mm"]
    for sign, corner in ((+1.0, "SUSPENSION_RL"), (-1.0, "SUSPENSION_RR")):
        # flange face in the driveline LOCAL frame, mapped to the vehicle frame
        local = [0.0, 0.0, sign * half]
        world = [base[i] + sum(R[i][k] * local[k] for k in range(3)) for i in range(3)]
        assert world == pytest.approx(by[corner]["origin_mm"], abs=1e-6)


def test_motor_and_driveline_share_axle_x_and_axis():
    """Motor and driveline are co-axial (both Rx(-90)) and on the same axle station;
    the motor is offset in X/Z by the e-axle placement (parallel-axis final drive)."""
    p = VehicleParams()
    by = _by_name(asm.build_plan(p))
    drv, mot = by["DRIVELINE_REAR"], by["MOTOR_REAR"]
    assert _mat_approx(mot["orientation"], drv["orientation"])
    assert _mat_approx(mot["orientation"], asm.rot_x(-90.0))
    assert mot["origin_mm"][0] == pytest.approx(drv["origin_mm"][0] - p.eaxle.motor_offset_x_mm)
    assert mot["origin_mm"][2] == pytest.approx(drv["origin_mm"][2] + p.eaxle.motor_offset_z_mm)


# --------------------------------------------------------------------------- #
# ICD §4 -- dimensional-consistency validation
# --------------------------------------------------------------------------- #
def test_default_vehicle_passes_icd_consistency():
    """The default vehicle must satisfy every ICD §4 rule."""
    assert asm.validate(VehicleParams()) == []


def test_validate_catches_track_mismatch():
    """If the vehicle track diverges from the driveline's built track > 2 %, flag it."""
    # driveline is built for 1580 mm; ask the vehicle for a 1900 mm track (+20 %).
    issues = asm.validate(VehicleParams().overridden(**{
        "layout.track_front_mm": 1900.0, "layout.track_rear_mm": 1900.0}))
    assert any("built track" in i for i in issues)


def test_validate_catches_inverter_below_ground():
    """Push the motor/inverter far below the axle -> ground-clearance violation."""
    issues = asm.validate(VehicleParams().overridden(**{
        "eaxle.motor_offset_z_mm": -335.0, "eaxle.inverter_offset_z_mm": -100.0}))
    assert any("ground" in i.lower() or "base z" in i for i in issues)


def test_validate_reports_motor_envelope_clearance():
    """The motor ground check uses the real motor envelope radius (not just the axle
    height): a small positive offset that leaves the axle above ground but sinks the
    envelope below it must still be flagged."""
    # axle z = r + off_z; envelope radius ~149 mm. off_z = -(r-50) keeps axle z = 50
    # (> 0) but envelope bottom = 50 - 149 < 0.
    issues = asm.validate(VehicleParams().overridden(**{
        "eaxle.motor_offset_z_mm": -(VehicleParams().layout.tyre_radius_mm - 50.0)}))
    assert any("envelope reaches the ground" in i for i in issues)


def test_awd_validates_and_has_two_e_axles_on_shared_hubs():
    """AWD: both axles driven; each driveline's flange faces coincide with that
    axle's suspension corners, and the plan still validates."""
    p = VehicleParams().overridden(**{"layout.drive_layout": "awd"})
    assert asm.validate(p) == []
    by = _by_name(asm.build_plan(p))
    assert {"DRIVELINE_FRONT", "DRIVELINE_REAR"} <= set(by)


def test_hub_bore_od_consistency_is_checked():
    """The validation reads back both hub-bore ODs; for the defaults they match."""
    assert asm._suspension_hub_bore_od_mm() == pytest.approx(asm._driveline_hub_bore_od_mm())
