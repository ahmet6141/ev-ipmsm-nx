"""Tests for gearbox_nx (NX-independent layers): params round-trip, engineering
ratios / centre distances / validation, the ICD §7.1 motor<->diff clearance, the
mating-interface match to motor_nx + driveline_nx, and the geometry blueprint."""

import math

import pytest

from gearbox_nx import blueprint as bp
from gearbox_nx import engineering as eng
from gearbox_nx.params import GearboxParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = GearboxParams()
    p2 = GearboxParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = GearboxParams.from_dict({"stage1": {"module_mm": 3.0}})
    assert p.stage1.module_mm == 3.0
    assert p.stage1.pinion_teeth == GearboxParams().stage1.pinion_teeth   # untouched
    assert p.stage2.gear_teeth == GearboxParams().stage2.gear_teeth


def test_overridden_dotted():
    p = GearboxParams().overridden(**{"stage2.gear_teeth": 60,
                                      "housing.wall_thickness_mm": 12.0})
    assert p.stage2.gear_teeth == 60
    assert p.housing.wall_thickness_mm == 12.0


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in GearboxParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


def test_centre_distances_are_nx_expressions():
    # the stage face widths + modules drive the centre distances; they must be editable
    names = {n for (n, _v, _u) in GearboxParams().expressions()}
    assert "stage1_module_mm" in names and "stage2_module_mm" in names
    assert "stage1_gear_teeth" in names and "stage2_gear_teeth" in names


# --------------------------------------------------------------------------- #
# engineering -- kinematics
# --------------------------------------------------------------------------- #
def test_total_ratio_is_product_of_stages():
    p = GearboxParams()
    g = eng.derive(p)
    r1 = p.stage1.gear_teeth / p.stage1.pinion_teeth
    r2 = p.stage2.gear_teeth / p.stage2.pinion_teeth
    assert g.total_ratio == pytest.approx(r1 * r2, rel=1e-4)


def test_total_ratio_in_ev_band():
    g = eng.derive(GearboxParams())
    assert 9.0 <= g.total_ratio <= 10.0


def test_output_torque_multiplies_by_ratio():
    p = GearboxParams()
    g = eng.derive(p)
    assert g.output_torque_nm == pytest.approx(p.motor_peak_torque_nm * g.total_ratio, rel=1e-3)


def test_output_speed_is_input_over_ratio():
    p = GearboxParams()
    g = eng.derive(p)
    assert g.output_speed_rpm == pytest.approx(p.motor_max_speed_rpm / g.total_ratio, rel=1e-3)


def test_centre_distances_sum_to_motor_offset():
    """ICD §7.1: the two stage centre distances must SUM to the motor<->diff offset so
    the gear train physically reaches. This is the connector's load-bearing invariant."""
    g = eng.derive(GearboxParams())
    assert g.centre_distance_1_mm + g.centre_distance_2_mm == pytest.approx(g.motor_offset_mm, abs=1e-6)


def test_centre_distance_equals_module_times_teeth():
    """Centre distance = m*(z_p + z_g)/2 -- the gear-geometry definition."""
    p = GearboxParams()
    g = eng.derive(p)
    c1 = p.stage1.module_mm * (p.stage1.pinion_teeth + p.stage1.gear_teeth) / 2.0
    c2 = p.stage2.module_mm * (p.stage2.pinion_teeth + p.stage2.gear_teeth) / 2.0
    assert g.centre_distance_1_mm == pytest.approx(c1, abs=1e-6)
    assert g.centre_distance_2_mm == pytest.approx(c2, abs=1e-6)


# --------------------------------------------------------------------------- #
# ICD §7.1 -- the headline: motor and differential must CLEAR (no interpenetration)
# --------------------------------------------------------------------------- #
def test_motor_offset_clears_the_icd_minimum():
    """motor<->diff centre distance >= motor_OD/2 + ring_gear_pitch/2 + 15 mm (231.5 mm
    for the defaults). The whole point of this package is to push the motor far enough
    out that it stops overlapping the diff."""
    g = eng.derive(GearboxParams())
    assert g.icd_min_centre_distance_mm == pytest.approx(112.5 + 104.0 + 15.0, abs=0.5)
    assert g.motor_offset_mm >= g.icd_min_centre_distance_mm
    assert g.centre_distance_margin_mm >= 0.0


def test_motor_offset_matches_icd_default_placement():
    """ICD §7.1 default offset is dx~=+155 (toward centre), dz~=+175 (up) -> ~234 mm.
    The derived offset components must land near that (so the assembler's placement and
    the gear train agree)."""
    g = eng.derive(GearboxParams())
    assert g.motor_offset_dx_mm == pytest.approx(155.0, abs=8.0)
    assert g.motor_offset_dz_mm == pytest.approx(175.0, abs=8.0)


def test_clearance_fails_when_offset_too_small():
    """Shrink the gears so C1+C2 falls below the ICD minimum -> validate() must flag the
    interpenetration (this is the failure mode the package exists to prevent)."""
    p = GearboxParams().overridden(**{"stage1.module_mm": 1.5, "stage2.module_mm": 2.0})
    g = eng.derive(p)
    assert g.motor_offset_mm < g.icd_min_centre_distance_mm
    assert any("interpenetrate" in i or "ICD §7.1" in i for i in eng.validate(p))


# --------------------------------------------------------------------------- #
# mating interfaces -- match the motor DE flange + the diff input flange
# --------------------------------------------------------------------------- #
def test_motor_flange_matches_motor_de_flange():
    """The gearbox motor-mounting flange (AUTO) must equal the motor_nx DE flange
    diameter, bolt circle, bolt count -- so it bolts straight on."""
    p = GearboxParams()
    mf = eng.resolve_motor_flange(p)
    motor = eng.motor_de_flange()
    assert mf["flange_diameter_mm"] == pytest.approx(motor["flange_diameter_mm"], abs=1e-6)
    assert mf["bolt_circle_diameter_mm"] == pytest.approx(motor["bolt_circle_diameter_mm"], abs=1e-6)
    assert mf["bolt_count"] == motor["bolt_count"]
    assert mf["pilot_diameter_mm"] == pytest.approx(motor["pilot_diameter_mm"], abs=1e-6)


def test_motor_de_flange_reads_real_motor_geometry():
    """Sanity-pin the values read from motor_nx (OD 298, PCD 281.2, 8x M10, pilot 226)
    so a future motor_nx change that breaks the bolt-on surfaces here."""
    motor = eng.motor_de_flange()
    assert motor["flange_diameter_mm"] == pytest.approx(298.0, abs=0.5)
    assert motor["bolt_circle_diameter_mm"] == pytest.approx(281.2, abs=0.5)
    assert motor["bolt_count"] == 8


def test_output_coupling_matches_driveline_input_flange():
    """The gearbox OUTPUT coupling must match the driveline diff input flange (the
    gearbox output -> diff interface, ICD §7.1)."""
    from driveline_nx.params import DrivelineParams
    p = GearboxParams()
    di = eng.diff_input_interface()
    assert p.output.flange_diameter_mm == pytest.approx(di["input_flange_diameter_mm"], abs=1e-6)
    assert p.output.bore_diameter_mm == pytest.approx(di["input_bore_diameter_mm"], abs=1e-6)
    # and that it really is the driveline's value, not a coincidence
    assert di["input_flange_diameter_mm"] == DrivelineParams().differential.input_flange_diameter


def test_diff_carrier_bore_seats_the_driveline_carrier():
    di = eng.diff_input_interface()
    p = GearboxParams()
    assert p.housing.diff_carrier_diameter_mm >= di["carrier_outer_diameter_mm"]


# --------------------------------------------------------------------------- #
# engineering -- validation
# --------------------------------------------------------------------------- #
def test_default_design_is_buildable():
    assert eng.validate(GearboxParams()) == []


def test_validate_catches_non_reducing_stage():
    p = GearboxParams().overridden(**{"stage1.gear_teeth": 10})  # gear <= pinion
    assert any("gear_teeth must exceed pinion_teeth" in i for i in eng.validate(p))


def test_validate_catches_same_band_gear_clash():
    """A NEGATIVE inter-gear gap overlaps the two mesh bands on the layshaft, so the big
    stage-1 gear (layshaft) and the output gear (diff) -- whose XY pitch circles overlap
    in the inline layout -- now share an axial band and physically collide. validate()
    must catch it."""
    p = GearboxParams().overridden(**{"layshaft.inter_gear_gap_mm": -50.0,
                                      "housing.axial_length_mm": 250.0})
    assert any("collide" in i for i in eng.validate(p))


def test_validate_catches_thin_housing_axial():
    p = GearboxParams().overridden(**{"housing.axial_length_mm": 10.0})
    assert any("axial length" in i for i in eng.validate(p))


def test_meshes_sit_in_disjoint_axial_bands():
    """The connector relies on the two meshes being AXIALLY separated so the otherwise-
    overlapping big gears never collide -- assert the bands are disjoint by default."""
    bands = eng.axial_bands(GearboxParams())
    b1, b2 = bands["stage1"], bands["stage2"]
    assert b1[1] <= b2[0]            # stage-1 band ends before stage-2 begins


# --------------------------------------------------------------------------- #
# blueprint
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    import json
    blue = bp.generate(GearboxParams())
    assert blue["schema"] == "gearbox_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))     # serialisable
    assert len(blue["build_steps"]) > 10


def test_every_boolean_targets_an_existing_create():
    blue = bp.generate(GearboxParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_four_gear_blanks_present():
    blue = bp.generate(GearboxParams())
    gear_ids = {s["id"] for s in blue["build_steps"] if s["role"] == "gear"}
    assert {"motor_pinion", "layshaft_gear", "layshaft_pinion", "output_gear"} <= gear_ids


def test_housing_is_a_hollow_prism_shell():
    blue = bp.generate(GearboxParams())
    steps = {s["id"]: s for s in blue["build_steps"]}
    assert steps["housing_shell"]["kind"] == "prism" and steps["housing_shell"]["boolean"] == "create"
    assert steps["housing_cavity"]["boolean"] == "subtract"
    assert steps["housing_cavity"]["target"] == "housing_shell"


def test_motor_flange_centred_on_the_motor_axis():
    """The motor-mounting flange must be coaxial with the MOTOR axis (offset from the
    diff origin), not the diff axis -- else the motor would not bolt on where it sits."""
    p = GearboxParams()
    blue = bp.generate(p)
    pos = eng.axis_positions(p)
    mf = next(s for s in blue["build_steps"] if s["id"] == "motor_flange")
    assert mf["origin3"][0] == pytest.approx(pos["motor"][0], abs=1e-6)
    assert mf["origin3"][1] == pytest.approx(pos["motor"][1], abs=1e-6)


def test_output_and_diff_mount_coaxial_with_diff_axis():
    """The output coupling and the diff-carrier mount are coaxial with the DIFF axis
    (the local origin), so the driveline (datumed on the diff axis) mates to them."""
    blue = bp.generate(GearboxParams())
    steps = {s["id"]: s for s in blue["build_steps"]}
    oc = steps["output_coupling"]
    assert oc["cx"] == pytest.approx(0.0) and oc["cy"] == pytest.approx(0.0)
    dm = steps["diff_mount_flange"]
    assert dm["origin3"][0] == pytest.approx(0.0) and dm["origin3"][1] == pytest.approx(0.0)


def test_gears_lie_inside_the_housing_cavity_axially():
    """Every gear blank must sit within the housing cavity axial span (0 .. cavity_len)."""
    p = GearboxParams()
    g = eng.derive(p)
    blue = bp.generate(p)
    for s in blue["build_steps"]:
        if s["role"] == "gear":
            z_lo, z_hi = s["origin3"][2], s["origin3"][2] + s["length"]
            assert 0.0 - 1e-6 <= z_lo and z_hi <= g.housing_axial_length_mm + 1e-6


def test_gears_fit_radially_inside_the_inner_wall():
    """Each gear tip must clear the housing inner wall (gear tip radius from its axis,
    plus its axis distance from the diff origin, < housing inner radius)."""
    p = GearboxParams()
    g = eng.derive(p)
    pos = eng.axis_positions(p)
    blue = bp.generate(p)
    axis_of = {"motor_pinion": pos["motor"], "layshaft_gear": pos["layshaft"],
               "layshaft_pinion": pos["layshaft"], "output_gear": pos["diff"]}
    for s in blue["build_steps"]:
        if s["role"] == "gear":
            cx, cy = axis_of[s["id"]]
            reach = math.hypot(cx, cy) + s["outer_radius"]
            assert reach <= g.housing_inner_radius_mm + 1e-6


def test_motor_bolt_circle_count_matches_motor():
    """The motor bolt holes must be emitted as explicit instances matching the motor DE
    bolt count (patterned about the OFFSET motor axis, not the global Z)."""
    p = GearboxParams()
    blue = bp.generate(p)
    motor = eng.motor_de_flange()
    bolts = [s for s in blue["build_steps"] if s["role"] == "motor_bolt_cut"]
    assert len(bolts) == motor["bolt_count"]


# --------------------------------------------------------------------------- #
# assembler hookup -- the package reports the data the vehicle assembler needs
# --------------------------------------------------------------------------- #
def test_derived_exposes_the_assembler_offset():
    """The vehicle assembler places the motor at the derived offset; expose it so the
    motor and diff stop interpenetrating (ICD §7.1 replaces the old 60/110 mm)."""
    g = eng.derive(GearboxParams())
    # the offset must be the documented ~234 mm, well above the old overlapping 125 mm
    assert g.motor_offset_mm > 200.0
    assert g.motor_offset_dx_mm > 0.0 and g.motor_offset_dz_mm > 0.0
