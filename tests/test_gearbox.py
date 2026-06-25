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


def _gear_profile_radii(s):
    """Polar radii of a helical gear's LOCAL (origin-centred) loft_twist profile, from its
    own axis. The gear's world position is origin3; the profile is in local (u, v)."""
    return [math.hypot(x, y) for (x, y) in s["profile"]]


def test_gears_lie_inside_the_housing_cavity_axially():
    """Every helical gear must sit within the housing cavity axial span (0 .. cavity_len)."""
    p = GearboxParams()
    g = eng.derive(p)
    blue = bp.generate(p)
    for s in blue["build_steps"]:
        if s["role"] == "gear":
            z_lo, z_hi = s["origin3"][2], s["origin3"][2] + s["length"]
            assert 0.0 - 1e-6 <= z_lo and z_hi <= g.housing_axial_length_mm + 1e-6


def test_gears_fit_radially_inside_the_inner_wall():
    """Each gear TOOTH TIP must clear the housing inner wall (max LOCAL profile radius,
    plus the gear axis distance from the diff origin, < housing inner radius)."""
    p = GearboxParams()
    g = eng.derive(p)
    pos = eng.axis_positions(p)
    blue = bp.generate(p)
    axis_of = {"motor_pinion": pos["motor"], "layshaft_gear": pos["layshaft"],
               "layshaft_pinion": pos["layshaft"], "output_gear": pos["diff"]}
    for s in blue["build_steps"]:
        if s["role"] == "gear":
            centre = axis_of[s["id"]]
            tip_r = max(_gear_profile_radii(s))
            reach = math.hypot(centre[0], centre[1]) + tip_r
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


# --------------------------------------------------------------------------- #
# NO BODY INTERPENETRATION (ICD §7.6) -- a gear is mounted ON a shaft, so it must
# NEVER be a solid disc that overlaps the shaft it sits on. nx_inspect found the gear
# + pinion blanks (and the housing end cover) sharing metal with the layshaft; the fix
# bores each blank to the mating shaft OD (a press-fit ring) or UNITES the layshaft
# blanks into the shaft as one rotating cluster, and bores the housing where each shaft
# crosses the cast covers. The NX-free clearance.py test ignores subtract/unite bodies
# (it cannot see a bore or a boolean), so the load-bearing checks are STRUCTURAL on the
# blueprint: bore ID >= shaft OD, unite present + ordered, housing bore clears the shaft.
# --------------------------------------------------------------------------- #
def _steps_by_id(p):
    return {s["id"]: s for s in bp.generate(p)["build_steps"]}


def _shaft_od_for(p):
    """Mating-shaft OD (mm) each gear blank sits on."""
    ls = p.layshaft
    return {
        "motor_pinion": p.motor_pinion_bore_diameter_mm,   # motor rotor shaft
        "layshaft_gear": ls.shaft_diameter_mm,             # the layshaft
        "layshaft_pinion": ls.shaft_diameter_mm,           # the layshaft
        "output_gear": p.output.bore_diameter_mm,          # diff input shaft
    }


def test_no_gear_is_a_solid_disc_on_its_shaft():
    """Every REAL HELICAL gear (loft_twist) is mounted on a shaft WITHOUT sharing solid,
    one of two physically-correct ways (loft_twist cannot boolean-unite inline, so the old
    cluster-unite is gone -- everything is now a press fit or integral):
      * a helical gear CREATEd then bored to the mating shaft OD (a touching PRESS FIT;
        `<gid>_bore` subtract whose ID >= the shaft OD) -- when the gear has a hub.
      * a SOLID integral pinion (no bore) -- when the gear is too small to bore (bore
        radius >= root radius), e.g. the motor pinion on the rotor-shaft tip."""
    from gearbox_nx.gear_profile import gear_metrics as _gm
    p = GearboxParams()
    steps = _steps_by_id(p)
    shaft_od = _shaft_od_for(p)
    for gid, od in shaft_od.items():
        s = steps[gid]
        assert s["kind"] == "loft_twist", "%s is a true helical gear, not %s" % (gid, s["kind"])
        assert s["boolean"] == "create"            # loft_twist is always standalone create
        stage = getattr(p, _GEAR_SPEC[gid][0])
        teeth = getattr(stage, _GEAR_SPEC[gid][1])
        root_r = _gm(stage.module_mm, teeth, stage.pressure_angle_deg)["root_radius"]
        if od / 2.0 >= root_r:
            # integral pinion -- no hub to bore, so NO bore step (and no keyway)
            assert ("%s_bore" % gid) not in steps, (
                "%s has bore Ø%.1f >= root Ø%.1f: it must be a solid integral pinion"
                % (gid, od, 2.0 * root_r))
            assert ("%s_keyway" % gid) not in steps
        else:
            bore = steps["%s_bore" % gid]
            assert bore["boolean"] == "subtract" and bore["target"] == gid
            bore_id = 2.0 * bore["outer_radius"]
            assert od - 1e-6 <= bore_id, (
                "%s bore Ø%.1f < shaft Ø%.1f: the gear would interpenetrate the shaft"
                % (gid, bore_id, od))
            assert bore_id < 2.0 * root_r          # bore stays inside the hub (root)


def test_layshaft_gears_press_fit_after_the_shaft_exists():
    """The layshaft gears press-fit onto the layshaft (bore = shaft OD), and the layshaft
    is CREATEd BEFORE them so the bore subtracts from the gear (not the shaft) and the
    press fit is well-defined. loft_twist cannot unite inline -- so no cluster-unite."""
    p = GearboxParams()
    order = [s["id"] for s in bp.generate(p)["build_steps"]]
    steps = _steps_by_id(p)
    assert order.index("layshaft") < order.index("layshaft_gear")
    assert order.index("layshaft") < order.index("layshaft_pinion")
    for gid in ("layshaft_gear", "layshaft_pinion"):
        assert steps[gid]["kind"] == "loft_twist" and steps[gid]["boolean"] == "create"
        bore = steps["%s_bore" % gid]
        assert bore["boolean"] == "subtract" and bore["target"] == gid
        assert 2.0 * bore["outer_radius"] == pytest.approx(p.layshaft.shaft_diameter_mm, abs=1e-6)


def test_layshaft_gears_bore_to_the_shaft_od():
    """The layshaft helical gears are CREATEd loft bodies bored to the shaft OD (press
    fit) -- no solid-disc-on-shaft overlap."""
    p = GearboxParams()
    steps = _steps_by_id(p)
    for gid in ("layshaft_gear", "layshaft_pinion"):
        s = steps[gid]
        assert s["kind"] == "loft_twist" and s["boolean"] == "create"
        bore = steps["%s_bore" % gid]
        assert bore["boolean"] == "subtract" and bore["target"] == gid
        assert 2.0 * bore["outer_radius"] >= p.layshaft.shaft_diameter_mm - 1e-6


def test_housing_bearing_bore_clears_every_shaft():
    """The cast housing must not share solid with a shaft (the housing<->layshaft clash):
    each shaft passes through a bearing bore subtracted from the housing shell, coaxial
    with the shaft and sized clear of the shaft OD."""
    p = GearboxParams()
    pos = eng.axis_positions(p)
    steps = _steps_by_id(p)
    expect = {
        "bearing_bore_layshaft": (pos["layshaft"], p.layshaft.shaft_diameter_mm),
        "bearing_bore_motor": (pos["motor"], p.motor_pinion_bore_diameter_mm),
        "bearing_bore_output": (pos["diff"], p.output.bore_diameter_mm),
    }
    for bid, (centre, shaft_od) in expect.items():
        assert bid in steps, "missing housing bearing bore %s" % bid
        s = steps[bid]
        assert s["boolean"] == "subtract" and s["target"] == "housing_shell"
        assert 2.0 * s["outer_radius"] >= shaft_od, (
            "%s Ø%.1f does not clear shaft Ø%.1f" % (bid, 2.0 * s["outer_radius"], shaft_od))
        # coaxial with the shaft it clears
        assert s["origin3"][0] == pytest.approx(centre[0], abs=1e-6)
        assert s["origin3"][1] == pytest.approx(centre[1], abs=1e-6)


def test_bearing_bore_pierces_both_end_covers():
    """The bearing bore must span the FULL housing length (pierce both cast end covers),
    so a shaft crossing either cover sits in the bore, not in solid metal."""
    p = GearboxParams()
    g = eng.derive(p)
    h = p.housing
    steps = _steps_by_id(p)
    full_len = g.housing_axial_length_mm + 2.0 * h.end_cover_thickness_mm
    s = steps["bearing_bore_layshaft"]
    z_lo = s["origin3"][2]
    z_hi = z_lo + s["length"]
    cover_lo_top = 0.0                                  # -Z cover spans -cover .. 0
    cover_hi_bot = g.housing_axial_length_mm           # +Z cover spans cavity_len .. +cover
    assert z_lo <= -h.end_cover_thickness_mm + 1e-6     # reaches the outer -Z face
    assert z_hi >= cover_hi_bot + h.end_cover_thickness_mm - 1e-6  # through the +Z face
    assert s["length"] >= full_len - 1e-6


def test_clearance_module_sees_no_gross_solid_overlap_growth():
    """Sanity gross-overlap guard with vehicle_nx.clearance (NX-free). The sampled-solid
    test ignores subtract/unite, so it bounds each blank by its OUTER envelope -- it
    cannot validate a bore. We use it only to confirm the FIX does not GROW the part's
    outer envelope vs the gear pitch circles (no body sticks out further than its gear
    tip), i.e. the bores/unites are interior changes. The true no-overlap proof is the
    structural bore/unite asserts above."""
    from vehicle_nx import clearance as cl
    p = GearboxParams()
    g = eng.derive(p)
    blue = bp.generate(p)
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    lb = cl.local_bbox(blue)
    assert lb is not None
    (lo, hi) = lb
    # the part envelope stays bounded by the housing oval + flanges (a few hundred mm),
    # i.e. nothing exploded; a finite, sane bounding box.
    assert all(abs(v) < 1000.0 for v in lo + hi)


# --------------------------------------------------------------------------- #
# REAL HELICAL INVOLUTE TEETH -- each gear is a TRUE helical loft_twist of the involute
# outline (LOCAL, origin-centred): a closed loop between root and tip with exactly z teeth,
# a non-zero helix TWIST, opposite hand per mesh, PHASED so the meshing pair interlocks.
# --------------------------------------------------------------------------- #
from gearbox_nx.gear_profile import gear_metrics   # noqa: E402

_GEAR_SPEC = {  # gid -> (stage_attr, teeth_attr)
    "motor_pinion": ("stage1", "pinion_teeth"),
    "layshaft_gear": ("stage1", "gear_teeth"),
    "layshaft_pinion": ("stage2", "pinion_teeth"),
    "output_gear": ("stage2", "gear_teeth"),
}


def _gear_axis(p):
    pos = eng.axis_positions(p)
    return {"motor_pinion": pos["motor"], "layshaft_gear": pos["layshaft"],
            "layshaft_pinion": pos["layshaft"], "output_gear": pos["diff"]}


def test_every_gear_is_a_true_helical_loft():
    """Every gear is a TRUE helical solid (kind="loft_twist") of a closed involute outline,
    with a non-zero helix twist -- not a spur prism and not a smooth disc."""
    p = GearboxParams()
    blue = bp.generate(p)
    twist = eng.gear_twists(p)
    gears = {s["id"]: s for s in blue["build_steps"] if s["role"] == "gear"}
    assert set(gears) == set(_GEAR_SPEC)
    for gid, s in gears.items():
        assert s["kind"] == "loft_twist" and s["profile"] is not None
        assert len(s["profile"]) > 100          # a real toothed loop, not a 4-pt blank
        assert abs(s["twist_deg"]) > 1.0        # a real helix lead, not spur
        assert s["twist_deg"] == pytest.approx(twist[gid], abs=1e-6)


def test_meshing_gears_have_opposite_helix_hand():
    """Meshing gears must have OPPOSITE helix hand (opposite twist sign) to mesh: stage-1
    motor_pinion vs layshaft_gear, stage-2 layshaft_pinion vs output_gear."""
    twist = eng.gear_twists(GearboxParams())
    assert twist["motor_pinion"] * twist["layshaft_gear"] < 0
    assert twist["layshaft_pinion"] * twist["output_gear"] < 0


def test_gear_outline_radii_lie_between_root_and_tip():
    """The LOCAL (origin-centred) outline radii all fall in [root, tip] -- a genuine
    involute profile, never inside the root or beyond the tip circle."""
    p = GearboxParams()
    blue = bp.generate(p)
    for s in blue["build_steps"]:
        if s["role"] != "gear":
            continue
        stage = getattr(p, _GEAR_SPEC[s["id"]][0])
        teeth = getattr(stage, _GEAR_SPEC[s["id"]][1])
        shift = (stage.pinion_profile_shift if "pinion" in _GEAR_SPEC[s["id"]][1]
                 else stage.gear_profile_shift)
        m = gear_metrics(stage.module_mm, teeth, stage.pressure_angle_deg, profile_shift=shift)
        radii = [math.hypot(x, y) for (x, y) in s["profile"]]   # profile is LOCAL (origin)
        assert min(radii) >= m["root_radius"] - 1e-6
        assert max(radii) <= m["tip_radius"] + 1e-6
        assert max(radii) == pytest.approx(m["tip_radius"], abs=1e-3)   # teeth reach the tip


def test_tooth_count_equals_z():
    """The LOCAL outline has exactly z teeth: count the radial peaks at the tip circle."""
    p = GearboxParams()
    blue = bp.generate(p)
    for s in blue["build_steps"]:
        if s["role"] != "gear":
            continue
        stage = getattr(p, _GEAR_SPEC[s["id"]][0])
        teeth = getattr(stage, _GEAR_SPEC[s["id"]][1])
        shift = (stage.pinion_profile_shift if "pinion" in _GEAR_SPEC[s["id"]][1]
                 else stage.gear_profile_shift)
        m = gear_metrics(stage.module_mm, teeth, stage.pressure_angle_deg, profile_shift=shift)
        # a vertex is "at the tip" if within 1% of (tip-root) of the tip radius; the tip
        # arcs form `teeth` contiguous runs of such vertices around the gear.
        thr = m["tip_radius"] - 0.01 * (m["tip_radius"] - m["root_radius"])
        at_tip = [math.hypot(x, y) >= thr for (x, y) in s["profile"]]
        runs = sum(1 for i in range(len(at_tip))
                   if at_tip[i] and not at_tip[i - 1])      # rising edges (wraps around)
        assert runs == teeth, "%s: counted %d tooth tips, expected z=%d" % (s["id"], runs, teeth)


def _gear_solids(p, gid):
    from vehicle_nx import clearance as cl
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return cl.part_solids({"build_steps": [steps[gid]]}, I3, [0.0, 0.0, 0.0])


def test_meshing_pairs_do_not_interpenetrate():
    """The two meshing pairs must be PHASED (tooth-in-gap at the line of centres) so the
    helical solids interlock without clashing -- the sampled-solid clearance check (which
    bounds a loft_twist body by its true twisting section) reports NO interpenetration."""
    from vehicle_nx import clearance as cl
    p = GearboxParams()
    for a, b in (("motor_pinion", "layshaft_gear"), ("layshaft_pinion", "output_gear")):
        clash = cl.solids_interpenetrate(_gear_solids(p, a), _gear_solids(p, b), touch_tol=0.5)
        assert clash is None, "%s <-> %s interpenetrate by %s mm (mesh not phased)" % (a, b, clash)


def test_mis_set_mesh_would_clash_so_the_check_is_real():
    """Sanity the helical phasing test has teeth: breaking the driven gear's setup -- either
    SAME hand (twist sign flipped) or NO phase gap (tooth-on-tooth) -- makes the sampled
    twisting solids DO clash, so the pass above is the phasing/hand working, not a blind
    spot in the loft_twist clearance sampler."""
    import copy
    from vehicle_nx import clearance as cl
    p = GearboxParams()
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def sol(step):
        return cl.part_solids({"build_steps": [step]}, I3, [0.0, 0.0, 0.0])

    for drv, dvn, z in (("motor_pinion", "layshaft_gear", p.stage1.gear_teeth),
                        ("layshaft_pinion", "output_gear", p.stage2.gear_teeth)):
        # (a) SAME hand as the driver -> clash
        same = copy.deepcopy(steps[dvn])
        same["twist_deg"] = -same["twist_deg"]
        assert cl.solids_interpenetrate(sol(steps[drv]), sol(same), touch_tol=0.5) is not None
        # (b) phase gap removed (tooth-on-tooth) -> clash
        nogap = copy.deepcopy(steps[dvn])
        nogap["start_twist_deg"] = nogap["start_twist_deg"] - 180.0 / z
        assert cl.solids_interpenetrate(sol(steps[drv]), sol(nogap), touch_tol=0.5) is not None


# --------------------------------------------------------------------------- #
# BEARINGS -- representative rolling bearings (race rings) at each shaft journal,
# press-fit (inner-race bore = the journal OD), seated in a housing counterbore pocket.
# --------------------------------------------------------------------------- #
# the three modelled bearing seats: the two layshaft (internal shaft) journals + the
# output shaft. The motor-pinion shaft has NO gearbox bearing (the pinion is integral with
# the motor rotor, journalled by the motor's own bearings).
_BEARING_SEATS = ("layshaft_de", "layshaft_nde", "output_shaft")


def test_bearings_present_at_every_seat():
    """Each bearing seat is modelled as three concentric race rings (inner / rolling /
    outer) -- all TUBE bodies; the motor-pinion shaft is NOT journalled here."""
    blue = bp.generate(GearboxParams())
    steps = {s["id"]: s for s in blue["build_steps"]}
    for label in _BEARING_SEATS:
        for ring in ("inner", "rolling", "outer"):
            sid = "bearing_%s_%s" % (label, ring)
            assert sid in steps, "missing bearing ring %s" % sid
            assert steps[sid]["kind"] == "tube" and steps[sid]["role"] == "bearing"
    # no gearbox bearing on the integral motor pinion
    assert not any(k.startswith("bearing_motor_shaft") for k in steps)


def test_bearing_inner_race_bore_is_a_press_fit_on_the_journal():
    """The inner-race bore = the journal OD it presses onto (no shared solid, ICD §7.6);
    the race rings stack OUTWARD (inner OD < rolling ID, rolling OD < outer ID) and the
    outer-race OD = the bearing OD."""
    p = GearboxParams()
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    journal = {
        "layshaft_de": (p.layshaft.bearing_seat_diameter_mm, p.layshaft_bearing),
        "layshaft_nde": (p.layshaft.bearing_seat_diameter_mm, p.layshaft_bearing),
        "output_shaft": (p.output.bore_diameter_mm, p.output_shaft_bearing),
    }
    for label, (od, brg) in journal.items():
        inner = steps["bearing_%s_inner" % label]
        rolling = steps["bearing_%s_rolling" % label]
        outer = steps["bearing_%s_outer" % label]
        assert 2.0 * inner["inner_radius"] == pytest.approx(od, abs=1e-6), (
            "%s inner-race bore Ø%.1f != journal Ø%.1f" % (label, 2.0 * inner["inner_radius"], od))
        assert inner["outer_radius"] < rolling["inner_radius"]
        assert rolling["outer_radius"] < outer["inner_radius"]
        assert 2.0 * outer["outer_radius"] == pytest.approx(brg.od_d_mm, abs=1e-6)


def test_bearing_outer_race_seats_in_a_housing_pocket():
    """The outer race seats in a housing counterbore pocket (= bearing OD), cut from the
    housing shell coaxial with the shaft -- so the outer OD touches metal, never overlaps."""
    p = GearboxParams()
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    brg_of = {"layshaft_de": p.layshaft_bearing, "layshaft_nde": p.layshaft_bearing,
              "output_shaft": p.output_shaft_bearing}
    for label, brg in brg_of.items():
        pocket = steps["bearing_pocket_%s" % label]
        assert pocket["boolean"] == "subtract" and pocket["target"] == "housing_shell"
        assert 2.0 * pocket["outer_radius"] == pytest.approx(brg.od_d_mm, abs=1e-6)


def test_bearings_do_not_interpenetrate_the_gears_or_coupling():
    """Each bearing (bounded by its outer-race envelope) clears the gear on its shaft AND
    the output coupling: the layshaft bearings are flush at the cavity face (never enter the
    cavity), and the output bearing sits OUTBOARD of the coupling (no shared solid)."""
    from vehicle_nx import clearance as cl
    p = GearboxParams()
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def sol(sid):
        return cl.part_solids({"build_steps": [steps[sid]]}, I3, [0.0, 0.0, 0.0])
    pairs = [("bearing_layshaft_de_outer", "motor_pinion"),
             ("bearing_output_shaft_outer", "output_gear"),
             ("bearing_layshaft_nde_outer", "output_gear"),
             ("bearing_output_shaft_outer", "output_coupling"),
             ("bearing_output_shaft_inner", "output_coupling")]
    for a, b in pairs:
        assert cl.solids_interpenetrate(sol(a), sol(b), touch_tol=0.5) is None, (
            "%s interpenetrates %s" % (a, b))


def test_layshaft_de_bearing_clears_the_motor_flange():
    """The layshaft DE bearing is wider than the -Z cover, so its -Z face protrudes into the
    motor-flange axial band. A clearance relief is cast into the motor flange (coaxial with
    the LAYSHAFT axis, Ø >= bearing OD) over exactly that protrusion, so the bearing never
    shares solid with the Ø298 flange disc. The flange OD / bolt circle / pilot / mating face
    are untouched (the relief sits between the pilot bore and the bolt circle)."""
    p = GearboxParams()
    g = eng.derive(p)
    h = p.housing
    pos = eng.axis_positions(p)
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    relief = steps["motor_flange_layshaft_relief"]
    bearing = steps["bearing_layshaft_de_outer"]
    # cuts the flange, coaxial with the layshaft axis, wide enough for the bearing OD
    assert relief["boolean"] == "subtract" and relief["target"] == "motor_flange"
    assert relief["origin3"][0] == pytest.approx(pos["layshaft"][0], abs=1e-6)
    assert relief["origin3"][1] == pytest.approx(pos["layshaft"][1], abs=1e-6)
    assert 2.0 * relief["outer_radius"] >= 2.0 * bearing["outer_radius"]
    # the relief spans the bearing's protrusion into the flange band (z <= cover outer face)
    cover_outer_z = -h.end_cover_thickness_mm
    flange_lo = cover_outer_z - h.motor_flange_thickness_mm
    r_lo, r_hi = relief["origin3"][2], relief["origin3"][2] + relief["length"]
    b_lo = bearing["origin3"][2]
    assert r_lo <= b_lo + 1e-6                          # reaches the bearing -Z face
    assert r_hi >= cover_outer_z - 1e-6                 # up to / through the flange +Z face
    assert r_lo >= flange_lo - 1.0                      # cuts within the flange band (real metal)
    # the flange itself is unchanged: still matches the motor DE flange
    mf = eng.resolve_motor_flange(p)
    motor = eng.motor_de_flange()
    assert mf["flange_diameter_mm"] == pytest.approx(motor["flange_diameter_mm"], abs=1e-6)
    # the relief stays inside the bolt circle (does not break out a bolt / the OD)
    bc_r = mf["bolt_circle_diameter_mm"] / 2.0
    d_axes = math.hypot(pos["layshaft"][0] - pos["motor"][0], pos["layshaft"][1] - pos["motor"][1])
    assert d_axes + relief["outer_radius"] < bc_r - mf["bolt_diameter_mm"] / 2.0


def test_output_coupling_clears_the_housing_cover():
    """The output coupling pokes through a clearance counterbore in the +Z cover (= coupling
    OD + clearance) so it does NOT bury into the cast end cover, and its outer mating face
    stays at b2[1] + flange_thickness (the driveline diff-input coupling point, unchanged)."""
    p = GearboxParams()
    bands = eng.axial_bands(p)
    steps = {s["id"]: s for s in bp.generate(p)["build_steps"]}
    oc = steps["output_coupling"]
    cb = steps["output_coupling_clearance"]
    assert cb["boolean"] == "subtract" and cb["target"] == "housing_shell"
    # the clearance bore is wider than the coupling OD (so the coupling never touches metal)
    assert 2.0 * cb["outer_radius"] > p.output.flange_diameter_mm
    # mating face unchanged
    face_z = oc["z0"] + oc["length"]
    assert face_z == pytest.approx(bands["stage2"][1] + p.output.flange_thickness_mm, abs=1e-6)


# --------------------------------------------------------------------------- #
# ISO 6336 gear rating + ISO 281 bearing life
# --------------------------------------------------------------------------- #
def test_iso6336_safety_factors_computed_and_sane():
    """Both stages get an ISO 6336 bending + contact rating; the safety factors are
    positive, finite and in a sane engineering range, and the DEFAULT clears the targets."""
    p = GearboxParams()
    ratings = eng.gear_ratings(p)
    assert {r.stage for r in ratings} == {"stage1", "stage2"}
    for r in ratings:
        assert r.tangential_force_n > 0 and r.radial_force_n > 0
        assert 0.1 < r.bending_safety < 10.0      # sane range, not a divide-by-zero
        assert 0.1 < r.contact_safety < 10.0
        assert r.bending_safety >= eng._SF_TARGET   # default clears bending target
        assert r.contact_safety >= eng._SH_TARGET   # default clears contact target
    # exposed on the derived summary too
    g = eng.derive(p)
    assert g.min_bending_safety == pytest.approx(min(r.bending_safety for r in ratings))
    assert g.min_contact_safety == pytest.approx(min(r.contact_safety for r in ratings))


def test_iso6336_flags_a_weak_stage():
    """Shrinking a face width (without touching the centre distance) drops the safety
    below target, and rating_warnings()/validate() flag it."""
    p = GearboxParams().overridden(**{"stage2.face_width_mm": 8.0})
    warns = eng.rating_warnings(p)
    assert any("stage2" in w and ("S_F" in w or "S_H" in w) for w in warns)
    assert any("stage2" in i for i in eng.validate(p))


def test_iso281_bearing_life_computed_and_sane():
    """Every bearing gets an ISO 281 L10 / L10h life; the lives are positive, finite, use
    the right exponent (ball p=3, roller p=10/3) and the DEFAULT clears the 8000 h target."""
    p = GearboxParams()
    lives = eng.bearing_lives(p)
    assert {bl.seat for bl in lives} == {"layshaft_de", "layshaft_nde", "output_shaft"}
    for bl in lives:
        assert bl.l10_mrev > 0 and bl.l10h_hours > 0
        assert bl.equivalent_load_n > 0 and bl.dynamic_rating_n > 0
        assert bl.l10h_hours >= eng._L10H_TARGET_H     # default clears the life target
    g = eng.derive(p)
    assert g.min_bearing_l10h == pytest.approx(min(bl.l10h_hours for bl in lives))


def test_iso281_flags_a_short_lived_bearing():
    """Down-rating a bearing (tiny C) drops L10h below target and validate() flags it."""
    p = GearboxParams().overridden(**{"output_shaft_bearing.dynamic_load_rating_c_n": 3000.0})
    assert any("output_shaft" in i and "L10h" in i for i in eng.validate(p))


def test_report_includes_iso_ratings():
    """report() surfaces the per-stage S_F/S_H and the per-bearing L10h."""
    txt = eng.report(GearboxParams())
    assert "S_F" in txt and "S_H" in txt and "L10h" in txt
