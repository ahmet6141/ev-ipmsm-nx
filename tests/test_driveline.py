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
    # the differential is now a TRUE 1:1 differential -- the ~9.4:1 reduction lives in
    # gearbox_nx (ICD §7.1, review finding 3), so the diff no longer double-counts it.
    assert p.differential.final_drive_ratio == 1.0


def test_default_halfshaft_has_sound_static_margin():
    """Half-shafts are fatigue-critical: the default must clear a >= 1.5 static SF
    against the WORST-CASE single-wheel torque. The default diff is a torque-vectoring
    eDiff (bias 1.0 -> the full axle torque can pass through one shaft), so the 44 mm
    OD / 18 mm bore shaft is sized to keep SF >= 1.5 at that full-bias load."""
    g = eng.derive(DrivelineParams())
    assert DrivelineParams().halfshaft.diameter == 44.0
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
def test_axle_torque_is_gearbox_output_times_diff_ratio():
    """The differential input is the GEARBOX OUTPUT torque (the reduction is upstream in
    the gearbox, ICD §7.1 / review finding 3); the diff itself is 1:1, so the axle torque
    equals the gearbox output torque -- NOT motor_peak x 9 a second time."""
    from gearbox_nx.engineering import derive as g_derive
    from gearbox_nx.params import GearboxParams
    p = DrivelineParams()
    g = eng.derive(p)
    gbox_out = g_derive(GearboxParams()).output_torque_nm
    assert g.input_torque_nm == pytest.approx(gbox_out, rel=1e-3)
    assert g.ring_gear_torque_nm == pytest.approx(g.input_torque_nm * p.differential.final_drive_ratio, rel=1e-6)


def test_total_motor_to_wheel_ratio_is_physical():
    """gearbox total_ratio x diff ratio must land in the ~9-10:1 single-speed EV band --
    the reduction is NOT double-counted across the two packages (review finding 3)."""
    from gearbox_nx.engineering import derive as g_derive
    from gearbox_nx.params import GearboxParams
    total = g_derive(GearboxParams()).total_ratio * DrivelineParams().differential.final_drive_ratio
    assert 8.0 <= total <= 11.0


def test_diff_type_biases_per_wheel_torque():
    g_open = eng.derive(DrivelineParams().overridden(**{"differential.type": "open"}))
    g_elsd = eng.derive(DrivelineParams().overridden(**{"differential.type": "elsd"}))
    g_tv = eng.derive(DrivelineParams().overridden(**{"differential.type": "torque_vectoring"}))
    g_spool = eng.derive(DrivelineParams().overridden(**{"differential.type": "spool"}))
    assert g_open.per_wheel_torque_nm == pytest.approx(g_open.ring_gear_torque_nm * 0.5, abs=0.1)
    assert g_spool.per_wheel_torque_nm == pytest.approx(g_spool.ring_gear_torque_nm)  # full axle torque
    # a twin-clutch active eDiff (torque_vectoring) can route effectively the WHOLE
    # axle torque to one wheel, so its worst-case bias is the full axle torque -- the
    # same worst case as a locked spool (not the milder e-LSD 0.6 bias).
    assert g_open.per_wheel_torque_nm < g_elsd.per_wheel_torque_nm < g_tv.per_wheel_torque_nm
    assert g_tv.per_wheel_torque_nm == pytest.approx(g_spool.per_wheel_torque_nm)


def test_wheel_speed_is_gearbox_output_over_diff_ratio():
    """Wheel speed = gearbox OUTPUT speed / diff ratio (the diff is 1:1, the ~9.4:1
    reduction is upstream in the gearbox). Before review finding 3 this divided the motor
    speed by the diff's own 9:1 a SECOND time, giving an impossibly low wheel speed."""
    from gearbox_nx.engineering import derive as g_derive
    from gearbox_nx.params import GearboxParams
    p = DrivelineParams()
    g = eng.derive(p)
    gbox_out_rpm = g_derive(GearboxParams()).output_speed_rpm
    assert g.wheel_max_speed_rpm == pytest.approx(gbox_out_rpm / p.differential.final_drive_ratio, rel=1e-3)


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


# --------------------------------------------------------------------------- #
# ICD §3/§4 track tie: the wheel-hub flange faces must land on HUB_CENTRE
# --------------------------------------------------------------------------- #
def _hub_flange_outer_face_z(blue, tag):
    """Local z of the OUTER (wheel-mounting) face of a side's hub flange, read
    straight off the built geometry. The flange create step stores its lower-z
    face as z0 and grows by `length`; the OUTER face is the one farther from the
    differential centre (z = 0)."""
    for s in blue["build_steps"]:
        if s["id"] == "hub_flange_%s" % tag and s["boolean"] == "create":
            z_lo, z_hi = s["z0"], s["z0"] + s["length"]
            return z_hi if abs(z_hi) >= abs(z_lo) else z_lo
    raise AssertionError("no hub flange for side %s" % tag)


def test_hub_flange_face_lands_on_half_track():
    """ICD §3: each wheel-hub flange face must land at local z = +-T/2 = +-790
    (within +-2 %) so it coincides with the shared HUB_CENTRE after assembly."""
    p = DrivelineParams()
    blue = bp.generate(p)
    half = p.target_track_mm / 2.0
    tol = 0.02 * half
    z_r = _hub_flange_outer_face_z(blue, "r")
    z_l = _hub_flange_outer_face_z(blue, "l")
    assert z_r == pytest.approx(+half, abs=tol)
    assert z_l == pytest.approx(-half, abs=tol)
    # the default catalogue chain is solved to hit it EXACTLY (not just within 2 %)
    assert z_r == pytest.approx(+790.0, abs=1e-6)
    assert z_l == pytest.approx(-790.0, abs=1e-6)


def test_derived_track_matches_built_geometry():
    """The engineering-derived track / flange-face values must equal what the
    blueprint actually builds (no drift between the two layers)."""
    p = DrivelineParams()
    g = eng.derive(p)
    blue = bp.generate(p)
    z_r = _hub_flange_outer_face_z(blue, "r")
    assert g.flange_face_z_mm == pytest.approx(z_r, abs=1e-6)
    assert g.total_track_length_mm == pytest.approx(2.0 * z_r, abs=1e-6)
    assert g.track_target_mm == pytest.approx(p.target_track_mm)
    assert abs(g.track_error_pct) <= 2.0


def test_track_tie_holds_across_track_targets():
    """Changing the target track re-solves the inboard clearance so the flange face
    follows it (the geometry is genuinely tied to T, not a coincidence at 1580)."""
    # all of these are >= the minimum achievable track (fixed chain + plunge floor)
    for T in (1580.0, 1620.0, 1700.0):
        p = DrivelineParams().overridden(target_track_mm=T)
        g = eng.derive(p)
        blue = bp.generate(p)
        assert g.inboard_clearance_mm >= 5.0                 # never below the plunge floor
        assert _hub_flange_outer_face_z(blue, "r") == pytest.approx(T / 2.0, abs=1e-6)
        assert eng.validate(p) == []                         # within +-2 % => buildable


def test_track_tie_fails_when_chain_outgrows_target():
    """If the catalogue chain is already longer than target_track/2, the clearance
    clamps to its floor and validate() must FLAG the >2 % overshoot (geometry has to
    shrink -- the gap cannot go negative)."""
    p = DrivelineParams().overridden(target_track_mm=1000.0)  # 500 mm/side < ~771 chain
    g = eng.derive(p)
    assert g.inboard_clearance_mm == pytest.approx(5.0)       # clamped to the floor
    assert g.total_track_length_mm > 1000.0 * 1.02            # overshoots by >2 %
    assert any("built track" in i for i in eng.validate(p))


def test_chain_segments_physically_connect():
    """The per-side chain (inboard CV -> halfshaft -> outboard CV -> bearing ->
    flange) must be axially contiguous: each create body's near face touches the
    previous body's far face (no gaps / overlaps along the rotation axis)."""
    p = DrivelineParams()
    blue = bp.generate(p)
    steps = {s["id"]: s for s in blue["build_steps"]}
    # right side grows toward +Z; verify face-to-face continuity in chain order
    chain = ["cv_inboard_r", "halfshaft_r", "cv_outboard_r", "hub_bearing_r", "hub_flange_r"]
    prev_far = None
    for sid in chain:
        s = steps[sid]
        z_lo, z_hi = s["z0"], s["z0"] + s["length"]
        if prev_far is not None:
            assert z_lo == pytest.approx(prev_far, abs=1e-6), "%s does not touch its neighbour" % sid
        prev_far = z_hi
    # the first inboard bell starts after the carrier + solved clearance
    first = steps["cv_inboard_r"]
    expect = p.differential.carrier_length / 2.0 + eng.inboard_clearance(p)
    assert first["z0"] == pytest.approx(expect, abs=1e-6)


def test_target_track_is_an_nx_expression():
    names = {n for (n, _v, _u) in DrivelineParams().expressions()}
    assert "target_track_mm" in names
