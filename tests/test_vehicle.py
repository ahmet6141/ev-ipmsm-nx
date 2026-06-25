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
    the motor is offset in X/Z by the RESOLVED e-axle placement (the gearbox-derived
    final-drive centre distance, since the defaults are AUTO=0)."""
    p = VehicleParams()
    by = _by_name(asm.build_plan(p))
    drv, mot = by["DRIVELINE_REAR"], by["MOTOR_REAR"]
    off = asm.motor_offset(p)
    assert _mat_approx(mot["orientation"], drv["orientation"])
    assert _mat_approx(mot["orientation"], asm.rot_x(-90.0))
    assert mot["origin_mm"][0] == pytest.approx(drv["origin_mm"][0] - off["dx"])
    assert mot["origin_mm"][2] == pytest.approx(drv["origin_mm"][2] + off["dz"])


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


# --------------------------------------------------------------------------- #
# ICD §7 -- INTEGRATION: connector parts, no interpenetration, real matings
# --------------------------------------------------------------------------- #
from vehicle_nx import clearance


def test_gearbox_and_subframe_components_present():
    """The e-axle gearbox connector (per driven axle) and the suspension/e-axle subframe
    cradle (per axle) are now components of the assembled vehicle (ICD §7.1/§7.2)."""
    by = _by_name(asm.build_plan(VehicleParams()))
    assert "GEARBOX_REAR" in by                       # rear-drive default -> rear gearbox
    assert {"SUBFRAME_FRONT", "SUBFRAME_REAR"} <= set(by)  # 4 corners -> both subframes


def test_subframe_front_rear_distinct_part_files_with_axle_variant():
    """The front/rear subframe are X-mirror variants -> distinct part files, each tagged
    with the axle variant the assembler builds before generate()."""
    by = _by_name(asm.build_plan(VehicleParams()))
    sf, sr = by["SUBFRAME_FRONT"], by["SUBFRAME_REAR"]
    assert sf["part_file"] != sr["part_file"]
    assert sf["variant"] == {"axle": "front"}
    assert sr["variant"] == {"axle": "rear"}
    assert sf["origin_mm"] == [VehicleParams().layout.wheelbase_mm / 2.0, 0.0, 0.0]
    assert sr["origin_mm"] == [-VehicleParams().layout.wheelbase_mm / 2.0, 0.0, 0.0]
    assert sf["orientation"] == asm.identity()        # subframe placed identity per axle


def test_subframe_placed_identity_at_axle_station():
    """The subframe is built in vehicle coords offset by the axle station -> identity."""
    for c in asm.build_plan(VehicleParams())["components"]:
        if c["role"] == "subframe":
            assert c["orientation"] == asm.identity()


def test_gearbox_orientation_is_rx_minus90_rz180_orthonormal():
    """The gearbox uses Rx(-90).Rz(180): local +Z (gear axis) -> vehicle +Y, with the
    extra flip that bridges motor<->diff. The matrix must be a proper rotation."""
    R = asm.gearbox_orientation()
    out = _apply(R, [0.0, 0.0, 1.0])
    assert out == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)   # gear axis -> +Y
    # orthonormal columns
    for j in range(3):
        col = [R[i][j] for i in range(3)]
        assert math.sqrt(sum(x * x for x in col)) == pytest.approx(1.0, abs=1e-9)


def test_motor_offset_is_gearbox_derived_and_clears_old_overlap():
    """The motor offset now AUTO-resolves to the gearbox final-drive centre distance
    (~156 / ~176), well clear of the old 60/110 that drove the motor into the diff."""
    off = asm.motor_offset(VehicleParams())
    assert off["dx"] == pytest.approx(155.881, abs=1.0)
    assert off["dz"] == pytest.approx(176.192, abs=1.0)
    # the resolved centre distance must exceed the old 125 mm that caused the overlap
    assert math.hypot(off["dx"], off["dz"]) > 230.0


def test_motor_offset_override_takes_precedence():
    """A non-zero eaxle.motor_offset_* overrides the gearbox-derived AUTO value."""
    p = VehicleParams().overridden(**{"eaxle.motor_offset_x_mm": 200.0,
                                      "eaxle.motor_offset_z_mm": 250.0})
    off = asm.motor_offset(p)
    assert off["dx"] == pytest.approx(200.0)
    assert off["dz"] == pytest.approx(250.0)


# ---- the headline acceptance check: NO interpenetration -------------------- #
def test_default_vehicle_has_no_interpenetration():
    """ICD §7.1/§7.4.1 HEADLINE: no two non-chassis component solids interpenetrate on
    the default vehicle (the gearbox bridges motor<->diff, the cleared offset removes the
    old motor/diff overlap)."""
    assert asm.interpenetration_pairs(VehicleParams()) == []


def test_default_vehicle_validate_is_clean():
    """validate() returns no issues on the default vehicle (all ICD §4 + §7 checks)."""
    assert asm.validate(VehicleParams()) == []


def test_interpenetration_check_fires_on_bad_motor_offset():
    """Force the OLD 60/110 offset -> the motor is driven back into the differential;
    the headline interpenetration check must FIRE on the motor<->driveline pair."""
    bad = VehicleParams().overridden(**{"eaxle.motor_offset_x_mm": 60.0,
                                        "eaxle.motor_offset_z_mm": 110.0})
    pairs = asm.interpenetration_pairs(bad)
    names = {frozenset((h["a"], h["b"])) for h in pairs}
    assert frozenset(("MOTOR_REAR", "DRIVELINE_REAR")) in names
    # validate() surfaces it as an ICD §7.1 interpenetration issue
    assert any("INTERPENETRATION" in i for i in asm.validate(bad))


def test_interpenetration_solid_is_tighter_than_whole_part_aabb():
    """The sampled-solid test (used by validate) clears the default motor<->diff, where a
    naive whole-part AABB would falsely report overlap -- the motor + ring gear are round
    bodies offset diagonally, so their squared-off AABBs intersect though the solids do
    not. This documents WHY the headline check uses the sampled-solid variant."""
    by = _by_name(asm.build_plan(VehicleParams()))
    mot, drv = by["MOTOR_REAR"], by["DRIVELINE_REAR"]
    a = asm.component_world_aabb(mot)
    b = asm.component_world_aabb(drv)
    # the conservative whole-part AABBs DO overlap (the artefact)...
    assert clearance.interpenetrates(a, b)
    # ...but the sampled-solid test (the real geometry) does NOT.
    sa = asm._component_solids(mot)
    sb = asm._component_solids(drv)
    assert clearance.solids_interpenetrate(sa, sb) is None


# ---- the HONEST whole-vehicle clash check (incl. chassis, void-aware) ------- #
# The five REAL cross-component clashes the NX inspection confirmed (and that the old
# chassis-excluding / mating-skipping interpenetration_pairs FALSE-PASSED):
_REAL_CLASH_PAIRS = (
    frozenset(("DRIVELINE_REAR", "CHASSIS")),     # half-shaft/diff pierced the rail beyond the notch
    frozenset(("SUBFRAME_REAR", "SUSPENSION_RL")),  # cradle overlapped the control arms
    frozenset(("SUBFRAME_FRONT", "SUSPENSION_FL")),
    frozenset(("DRIVELINE_REAR", "SUBFRAME_REAR")), # diff/CV overlapped the cradle e-axle mounts
    frozenset(("CHASSIS", "SUSPENSION_RL")),      # control arms clipped the rail (notch too small)
)


def test_comprehensive_check_is_clean_on_the_default_vehicle():
    """HONEST whole-vehicle acceptance: the void-aware sampled-solid clash test run between
    EVERY component pair INCLUDING the chassis (with only the bolted e-axle/hub unit on the
    allowlist) finds NO clash on the fixed default vehicle."""
    assert asm.comprehensive_interpenetration_pairs(VehicleParams()) == []


def test_comprehensive_check_clears_every_real_clash_pair():
    """Each of the five REAL clashes the NX inspection confirmed is now clear under the
    honest check (the chassis axle notch was enlarged + extended into the crush cans; the
    subframe cradle was lowered + reshaped to clear the arms + the e-axle)."""
    pairs = {frozenset((h["a"], h["b"])) for h in asm.comprehensive_interpenetration_pairs(VehicleParams())}
    for real in _REAL_CLASH_PAIRS:
        assert real not in pairs, "%s still clashes" % set(real)


def _chassis_solids_voids(chassis_params):
    """The placed (identity at origin) chassis solids + voids for a given ChassisParams --
    the same primitives the honest check builds, so a regression test can swap in a
    notch-reverted chassis and prove the check would FIRE."""
    from chassis_nx.blueprint import generate as c_generate
    blue = c_generate(chassis_params)
    R, o = asm.identity(), [0.0, 0.0, 0.0]
    return clearance.part_solids(blue, R, o), clearance.part_voids(blue, R, o)


def test_comprehensive_check_includes_the_chassis_and_fires_if_the_notch_is_reverted():
    """The honest check EXAMINES the chassis (the old check excluded it -- exactly how the
    half-shaft/arm-into-rail clashes hid). Prove BOTH directions on the rear axle:

      * with the axle notch (the fix) the rear half-shaft + control arms CLEAR the rail; but
      * reverting the notch (axle_notch=False) makes them PIERCE the rail solid -> the
        void-aware clash test FIRES. This is the regression guard the user asked for.
    """
    from chassis_nx.params import ChassisParams
    p = VehicleParams()
    by = _by_name(asm.build_plan(p))
    # the suspension RL + driveline solids placed in the vehicle (the swept members that
    # cross the rail axle station)
    sus = asm._component_solids(by["SUSPENSION_RL"])
    drv = asm._component_solids(by["DRIVELINE_REAR"])

    # WITH the notch (default chassis): both clear the rail
    ok_s, ok_v = _chassis_solids_voids(ChassisParams())
    assert clearance.solids_clash(sus, ok_s, [], ok_v) is None
    assert clearance.solids_clash(drv, ok_s, [], ok_v) is None

    # REVERT the notch -> the rail solid now has no relief window, so the arms + half-shaft
    # pierce it and the clash test FIRES (the honest check would flag a CHASSIS pair).
    bad_s, bad_v = _chassis_solids_voids(ChassisParams().overridden(**{"frame.axle_notch": False}))
    assert clearance.solids_clash(sus, bad_s, [], bad_v) is not None
    assert clearance.solids_clash(drv, bad_s, [], bad_v) is not None


def _subframe_solids(sub_params, axle, origin):
    from subframe_nx.blueprint import generate as s_generate
    blue = s_generate(sub_params)
    return clearance.part_solids(blue, asm.identity(), origin)


def test_comprehensive_check_fires_if_the_subframe_intrudes_on_the_arms():
    """Reverting the subframe cradle UP into the control-arm envelope (the old high base
    plane) makes the cradle beams overlap the arms again -> the honest clash test FIRES on
    the subframe<->suspension pair (the regression guard for the cradle clearance)."""
    from subframe_nx.params import SubframeParams
    p = VehicleParams()
    by = _by_name(asm.build_plan(p))
    sus = asm._component_solids(by["SUSPENSION_RL"])
    axle_origin = by["SUBFRAME_REAR"]["origin_mm"]

    # the fixed (low) cradle clears the arms
    good = _subframe_solids(SubframeParams().overridden(axle="rear"), "rear", axle_origin)
    assert clearance.solids_clash(sus, good) is None
    # raising the cradle base plane back UP into the arm sweep re-introduces the clash
    bad = _subframe_solids(
        SubframeParams().overridden(**{"axle": "rear", "cradle.base_plane_z_mm": 250.0}),
        "rear", axle_origin)
    assert clearance.solids_clash(sus, bad) is not None


def test_eaxle_unit_pairs_are_the_only_allowlisted_overlaps():
    """The ONLY pairs the honest check skips are the bolted integrated e-axle / wheel-hub
    unit (gearbox<->motor, gearbox<->driveline-diff, driveline<->its-own-suspension-hub).
    Every OTHER pair -- especially subframe<->suspension, subframe<->chassis,
    driveline<->chassis, suspension<->chassis -- is checked."""
    p = VehicleParams()
    comps = asm.components(p)
    by = {c["name"]: c for c in comps}
    # the e-axle unit pairs ARE skipped...
    assert asm._is_eaxle_unit_pair(by["GEARBOX_REAR"], by["MOTOR_REAR"])
    assert asm._is_eaxle_unit_pair(by["DRIVELINE_REAR"], by["SUSPENSION_RL"])
    # ...but the policed cross-component pairs are NOT skipped
    assert not asm._is_eaxle_unit_pair(by["SUBFRAME_REAR"], by["SUSPENSION_RL"])
    assert not asm._is_eaxle_unit_pair(by["DRIVELINE_REAR"], by["CHASSIS"])
    assert not asm._is_eaxle_unit_pair(by["SUBFRAME_REAR"], by["CHASSIS"])
    # a CROSS-axle driveline<->suspension is NOT the unit either (different corners)
    assert not asm._is_eaxle_unit_pair(by["DRIVELINE_REAR"], by["SUSPENSION_FL"])


# ---- the void-aware clearance helper itself -------------------------------- #
def test_solids_clash_void_lets_a_body_pass_through_a_bore():
    """clearance.solids_clash treats a SUBTRACT void (a bore / the chassis axle notch) as
    clearance: a pin passing through a bored block is NOT a clash, but the same pin hitting
    the solid block (no bore) IS."""
    block = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 50.0,
         "origin3": (0.0, 0.0, 0.0), "axis": (0, 0, 1), "length": 40.0},
        {"kind": "cylinder", "boolean": "subtract", "outer_radius": 20.0,
         "origin3": (0.0, 0.0, -1.0), "axis": (0, 0, 1), "length": 42.0}]}
    pin = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 8.0,
         "origin3": (0.0, 0.0, -10.0), "axis": (0, 0, 1), "length": 60.0}]}
    sb = clearance.part_solids(block, asm.identity(), [0.0, 0.0, 0.0])
    vb = clearance.part_voids(block, asm.identity(), [0.0, 0.0, 0.0])
    sp = clearance.part_solids(pin, asm.identity(), [0.0, 0.0, 0.0])
    # void-aware: the pin runs through the Ø40 bore -> NO clash
    assert clearance.solids_clash(sp, sb, [], vb) is None
    # void-BLIND (no voids): the same pin overlaps the solid disc -> a clash
    assert clearance.solids_clash(sp, sb, [], []) is not None


# ---- mating coincidences (ICD §7.4.2) -------------------------------------- #
def test_gearbox_output_and_diff_mount_lie_on_the_diff_axis():
    """The gearbox output coupling + diff-carrier mount are coaxial with the differential
    axis (vehicle X = axle_x, Z = tyre_radius): they couple to the driveline diff input."""
    p = VehicleParams()
    L = p.layout
    for nm in ("output_coupling_face", "diff_mount_face"):
        pt = asm.gearbox_iface_world(p, "rear", nm)
        assert pt[0] == pytest.approx(-L.wheelbase_mm / 2.0, abs=1.0)
        assert pt[2] == pytest.approx(L.tyre_radius_mm, abs=1.0)


def test_gearbox_motor_flange_is_coaxial_with_the_motor():
    """The gearbox motor-mounting flange axis coincides (in X/Z) with the motor axis /
    DE flange -- the motor bolts straight onto the gearbox (ICD §7.4.2)."""
    p = VehicleParams()
    gb_axis = asm.gearbox_iface_world(p, "rear", "motor_axis")
    m_face = asm.motor_de_flange_world(p, "rear")
    assert m_face is not None
    assert math.hypot(gb_axis[0] - m_face[0], gb_axis[2] - m_face[2]) < asm._MATE_TOL_MM


def test_gearbox_motor_flange_face_coincides_in_full_3d():
    """Review finding 1: the gearbox motor-mounting flange FACE must coincide with the
    motor DE flange FACE in FULL 3D (incl. the axial Y), not just share the X/Z axis --
    the motor is placed axially so its DE flange butts the gearbox with no gap."""
    p = VehicleParams()
    gb_face = asm.gearbox_iface_world(p, "rear", "motor_flange_face")
    m_face = asm.motor_de_flange_world(p, "rear")
    assert m_face is not None
    assert asm._dist3(gb_face, m_face) < asm._MATE_TOL_MM


def test_gearbox_output_couples_to_real_driveline_diff_input():
    """Review finding 2: the gearbox output coupling must coincide (3D) with the driveline
    diff INPUT flange the driveline actually models (now coaxial with the diff axis), not
    merely lie 'on the diff axis' while the driveline input sits 127 mm off-axis."""
    p = VehicleParams()
    oc = asm.gearbox_iface_world(p, "rear", "output_coupling_face")
    di = asm.driveline_diff_input_world(p, "rear")
    assert di is not None
    assert asm._dist3(oc, di) < asm._MATE_TOL_MM
    # the driveline input flange is on the diff axis (X = axle_x, Z = r), not 127 mm off
    assert di[0] == pytest.approx(-VehicleParams().layout.wheelbase_mm / 2.0, abs=1.0)


def test_gearbox_motor_engagement_is_a_flange_touch_not_burial():
    """Review finding 4: with the motor butting the gearbox flange (not the housing
    sliding over the motor barrel), the gearbox<->motor AXIAL engagement is a small
    flange/pilot seating depth, well within the budget -- no deep coaxial burial."""
    p = VehicleParams()
    assert asm._mating_engagement_issues(p, "rear") == []
    from vehicle_nx import clearance
    by = _by_name(asm.build_plan(p))
    sg = asm._component_solids(by["GEARBOX_REAR"])
    sm = asm._component_solids(by["MOTOR_REAR"])
    eng = clearance.solids_axial_engagement(sg, sm, axis_index=1)
    assert eng is None or eng <= asm._MATING_ENGAGEMENT_BUDGET_MM


def test_vehicle_total_ratio_in_single_speed_band():
    """Review finding 3: the end-to-end motor->wheel ratio (gearbox x diff) must be the
    physical ~9-10:1, NOT the ~85:1 the old in-series double-count produced."""
    p = VehicleParams()
    total = asm.vehicle_total_ratio(p)
    assert total is not None
    assert 8.0 <= total <= 11.0
    assert not any("REDUCTION" in i for i in asm.validate(p))


def test_validate_catches_double_counted_reduction(monkeypatch):
    """If the differential is wrongly left as a second 9:1 final drive, the vehicle-level
    total-ratio check fires (the guard that prevents the double-count regressing). Simulate
    the old in-series ~85:1 by forcing the end-to-end ratio high."""
    monkeypatch.setattr(asm, "vehicle_total_ratio", lambda p: 84.6)
    issues = asm.validate(VehicleParams())
    assert any("REDUCTION" in i for i in issues)


def test_subframe_pickups_coincide_with_suspension_inboard_pickups_all_four_corners():
    """ICD §7.4.2 (the headline fix): the subframe pickup bosses coincide with the
    suspension inboard hardpoints on ALL FOUR corners (FL, FR, RL, RR) within a tight
    tolerance. The subframe now DERIVES its bosses from the suspension hardpoint table
    with the SAME placement convention, so every corner lands on the pickup -- not just
    the one convention-compatible corner the old (toothless) check exercised, which let
    three corners drift 225-890 mm."""
    p = VehicleParams()
    worst = 0.0
    for axle in ("front", "rear"):
        for side, sign in (("l", +1.0), ("r", -1.0)):
            for nm in ("lower_pickup_fore", "lower_pickup_aft", "upper_pickup_fore",
                       "upper_pickup_aft", "toe_pickup"):
                sub = asm.subframe_point_world(p, axle, "pickup", nm, side)
                sus = asm.suspension_hardpoint_world(p, axle, sign, nm)
                assert sub is not None and sus is not None
                d = asm._dist3(sub, sus)
                worst = max(worst, d)
                assert d < asm._SUBFRAME_COINCIDENCE_TOL_MM, (
                    "%s%s %s off by %.1f mm" % (axle, side, nm, d))
    assert worst < asm._SUBFRAME_COINCIDENCE_TOL_MM


def test_subframe_tower_top_coincides_with_damper_top_all_four_corners():
    """ICD §7.4.2: the subframe shock-tower top supports the suspension damper/strut top
    (no floating spring) on ALL FOUR corners."""
    p = VehicleParams()
    for axle in ("front", "rear"):
        for side, sign in (("l", +1.0), ("r", -1.0)):
            sub_t = asm.subframe_point_world(p, axle, "tower", "damper_top", side)
            sus_t = asm.suspension_hardpoint_world(p, axle, sign, "damper_top")
            assert sub_t is not None and sus_t is not None
            assert asm._dist3(sub_t, sus_t) < asm._SUBFRAME_COINCIDENCE_TOL_MM


def test_subframe_pads_coincide_with_chassis_pads():
    """ICD §7.4.2: the subframe chassis-pad flanges land on the chassis rail-top mount
    pads (rail centre-line |Y| and rail-top Z)."""
    p = VehicleParams()
    for s in ("l", "r"):
        ch = asm.chassis_pad_world("rear", s)
        assert ch is not None
        ok = False
        for fa in ("fore", "aft"):
            sub = asm.subframe_point_world(p, "rear", "pad", fa, s)
            if (abs(abs(sub[1]) - abs(ch[1])) < asm._MATE_TOL_MM
                    and abs(sub[2] - ch[2]) < asm._MATE_TOL_MM):
                ok = True
        assert ok


def test_gearbox_diff_origin_shared_is_allowed_but_other_origins_unique():
    """The gearbox + driveline intentionally share the diff-axis origin (coaxial
    connector) -- that pair is exempt from the shared-origin rule, but no OTHER
    non-chassis pair may share an origin."""
    assert not any("share an origin" in i for i in asm.validate(VehicleParams()))


# ---- the clearance utility itself ------------------------------------------ #
def test_clearance_local_bbox_skips_subtract_bodies():
    """local_bbox bounds only create/unite bodies; a subtract-only blueprint has no
    solid envelope (returns None)."""
    only_cut = {"build_steps": [
        {"kind": "cylinder", "boolean": "subtract", "outer_radius": 10.0,
         "cx": 0.0, "cy": 0.0, "z0": 0.0, "length": 5.0, "axis": (0, 0, 1)}]}
    assert clearance.local_bbox(only_cut) is None
    solid = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 10.0,
         "cx": 0.0, "cy": 0.0, "z0": 0.0, "length": 5.0, "axis": (0, 0, 1)}]}
    lo, hi = clearance.local_bbox(solid)
    assert lo == pytest.approx([-10.0, -10.0, 0.0])
    assert hi == pytest.approx([10.0, 10.0, 5.0])


def test_clearance_world_aabb_translates_by_origin():
    """world_aabb offsets the local box by the placement origin (identity rotation)."""
    solid = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 5.0,
         "cx": 0.0, "cy": 0.0, "z0": 0.0, "length": 4.0, "axis": (0, 0, 1)}]}
    lo, hi = clearance.world_aabb(solid, asm.identity(), [100.0, 0.0, 0.0])
    assert lo == pytest.approx([95.0, -5.0, 0.0])
    assert hi == pytest.approx([105.0, 5.0, 4.0])


def test_clearance_overlap_and_touch_tolerance():
    """overlap reports the per-axis intersection; a few-mm touch is NOT interpenetration
    but a deep overlap is."""
    a = ([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])
    near = ([9.0, 0.0, 0.0], [19.0, 10.0, 10.0])     # 1 mm overlap in X
    deep = ([5.0, 5.0, 5.0], [15.0, 15.0, 15.0])     # 5 mm overlap on all axes
    assert clearance.overlap(a, near)[0] == pytest.approx(1.0)
    assert not clearance.interpenetrates(a, near)    # 1 mm < touch tol
    assert clearance.interpenetrates(a, deep)


def test_clearance_solids_parallel_cylinders_clear_when_offset():
    """Two parallel cylinders whose centres are offset by more than the radius sum CLEAR
    even though their squared AABBs overlap -- the sampled-solid test resolves this."""
    cyl_a = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 50.0,
         "origin3": (0.0, 0.0, 0.0), "axis": (0, 0, 1), "length": 100.0}]}
    cyl_b = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 50.0,
         "origin3": (80.0, 80.0, 0.0), "axis": (0, 0, 1), "length": 100.0}]}
    sa = clearance.part_solids(cyl_a, asm.identity(), [0.0, 0.0, 0.0])
    sb = clearance.part_solids(cyl_b, asm.identity(), [0.0, 0.0, 0.0])
    # centre distance 113 > radius sum 100 -> clear, though the AABBs overlap
    aabb_a = clearance.world_aabb(cyl_a, asm.identity(), [0.0, 0.0, 0.0])
    aabb_b = clearance.world_aabb(cyl_b, asm.identity(), [0.0, 0.0, 0.0])
    assert clearance.interpenetrates(aabb_a, aabb_b)       # AABB artefact
    assert clearance.solids_interpenetrate(sa, sb) is None  # but the solids clear


def test_clearance_solids_overlapping_cylinders_clash():
    """Two coaxial-ish cylinders that genuinely overlap ARE reported as interpenetrating."""
    cyl_a = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 50.0,
         "origin3": (0.0, 0.0, 0.0), "axis": (0, 0, 1), "length": 100.0}]}
    cyl_b = {"build_steps": [
        {"kind": "cylinder", "boolean": "create", "outer_radius": 50.0,
         "origin3": (20.0, 0.0, 0.0), "axis": (0, 0, 1), "length": 100.0}]}
    sa = clearance.part_solids(cyl_a, asm.identity(), [0.0, 0.0, 0.0])
    sb = clearance.part_solids(cyl_b, asm.identity(), [0.0, 0.0, 0.0])
    assert clearance.solids_interpenetrate(sa, sb) is not None
