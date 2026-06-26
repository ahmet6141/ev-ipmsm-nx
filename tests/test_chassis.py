"""Tests for chassis_nx (NX-independent layers): params round-trip, engineering
mass/stiffness + validation, and the geometry blueprint's structural integrity.

The geometry tests assert the platform is built in TRUE vehicle coordinates
(ICD §1, §3): world bounding boxes via motor_nx.blueprint.profile_to_world,
that the rails/crossmembers/tray physically connect, and that the subframe mount
pads land on the ICD axle x-stations / track."""

import json

import pytest

from motor_nx.blueprint import profile_to_world

from chassis_nx import blueprint as bp
from chassis_nx import engineering as eng
from chassis_nx.params import ChassisParams


# --------------------------------------------------------------------------- #
# geometry helpers (NX-free world bounding boxes)
# --------------------------------------------------------------------------- #
def _steps_by_id(blue):
    return {s["id"]: s for s in blue["build_steps"]}


def _prism_world_bbox(step):
    """World (x,y,z) axis-aligned bounding box of a kind='prism' build step: map the
    (u,v) section to world at origin3, then sweep it along `axis` by `length`."""
    assert step["kind"] == "prism"
    prof = [tuple(pt) for pt in step["profile"]]
    base = profile_to_world(prof, step["origin3"], step["axis"], step["u_dir"])
    ax, L = step["axis"], step["length"]
    swept = base + [(x + ax[0] * L, y + ax[1] * L, z + ax[2] * L) for (x, y, z) in base]
    xs = [p[0] for p in swept]
    ys = [p[1] for p in swept]
    zs = [p[2] for p in swept]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


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
    # the platform is built in the TRUE vehicle frame; the dominant beams (rails,
    # tray, crush cans) run along +X, so the documentary global axis is "X".
    assert blue["units"] == "mm" and blue["axis"] == "X"
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


# --------------------------------------------------------------------------- #
# engineering -- new structural numbers
# --------------------------------------------------------------------------- #
def test_section_modulus_and_bending_case():
    g = eng.derive(ChassisParams())
    assert g.rail_Sx_mm3 > 0 and g.rail_Sy_mm3 > 0
    # Sx = Ix / (h/2) must be consistent with the reported Ix and rail height
    p = ChassisParams()
    assert g.rail_Sx_mm3 == pytest.approx(g.rail_Ix_mm4 / (p.frame.rail_height_mm / 2.0), rel=1e-3)
    # the static bench check must be physical: a positive finite stress, a real
    # safety factor against yield, and a sane mid-span deflection (mm, not metres).
    assert 0 < g.rail_max_bending_stress_mpa < 260.0      # below 6082-T6 proof stress
    assert g.rail_bending_safety_factor > 1.0
    assert 0 < g.rail_mid_deflection_mm < 60.0


def test_taller_rail_lowers_bending_stress():
    base = eng.derive(ChassisParams())
    taller = eng.derive(ChassisParams().overridden(**{"frame.rail_height_mm": 160.0}))
    # a deeper section has a larger modulus -> lower bending stress, higher SF
    assert taller.rail_max_bending_stress_mpa < base.rail_max_bending_stress_mpa
    assert taller.rail_bending_safety_factor > base.rail_bending_safety_factor


# --------------------------------------------------------------------------- #
# validate -- ICD §4.3 dimensional-consistency checks
# --------------------------------------------------------------------------- #
def test_validate_tray_must_fit_between_rails():
    # widen the tray past the inner channel -> must be flagged
    p = ChassisParams().overridden(**{"battery_tray.width_mm": 1090.0})  # +2*20 > 1100
    assert any("between the rails" in i for i in eng.validate(p))


def test_validate_length_must_exceed_wheelbase():
    p = ChassisParams().overridden(**{"frame.overall_length_mm": 2875.0})  # == wheelbase
    assert any("overall_length" in i for i in eng.validate(p))


# --------------------------------------------------------------------------- #
# geometry -- TRUE vehicle-frame world coordinates (ICD §1, §3)
# --------------------------------------------------------------------------- #
def test_rails_run_along_x_on_track_centrelines():
    """Each rail is a +X prism, centred on x=0 spanning the WHEELBASE PLUS a mount_zone
    extension beyond EACH axle (so the fore/aft subframe pads land on solid rail framing
    the axle relief), centre-line at y = ±(frame_inner_width/2 + rail_width/2) (ICD §3).
    The crush cans form the remaining overhangs and butt onto the extended rail ends."""
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    f = p.frame
    rail_cl = f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0
    rail_end = bp.rail_end_x(p)
    assert rail_end > f.wheelbase_mm / 2.0            # the rail extends past the axles
    for tag, sign in (("l", -1.0), ("r", +1.0)):
        (xlo, xhi), (ylo, yhi), (zlo, zhi) = _prism_world_bbox(by["rail_%s" % tag])
        # spans wheelbase + 2*mount_zone, centred on x = 0
        assert xlo == pytest.approx(-rail_end)
        assert xhi == pytest.approx(+rail_end)
        assert (xhi - xlo) == pytest.approx(2.0 * rail_end)
        assert (yhi - ylo) == pytest.approx(f.rail_width_mm)
        assert (zhi - zlo) == pytest.approx(f.rail_height_mm)
        # centre-line Y sits on the correct side at ±rail_cl
        assert (ylo + yhi) / 2.0 == pytest.approx(sign * rail_cl)


def test_crush_cans_butt_onto_rails_without_interpenetration():
    """The crush cans form the overhangs and BUTT onto the rail ends -- they must NOT
    be buried inside the rails (the adversarial-review HIGH finding). The rails span
    the wheelbase, so each can touches the rail only at the shared end face (zero X
    overlap)."""
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    for side in ("l", "r"):
        (rxlo, rxhi), _, _ = _prism_world_bbox(by["rail_%s" % side])
        for tag in ("front", "rear"):
            (cxlo, cxhi), _, _ = _prism_world_bbox(by["crush_%s_%s" % (tag, side)])
            overlap = min(cxhi, rxhi) - max(cxlo, rxlo)
            assert overlap <= 1e-6, (tag, side, overlap)   # touch, not bury
    # and engineering.validate() is clean for the default geometry
    assert eng.validate(p) == []


def test_crossmembers_bridge_the_two_rails():
    """Each crossmember is a +Y prism whose ends land INSIDE both rails (it spans the
    full inner width and embeds into each rail), and it lives within the wheelbase."""
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    f = p.frame
    inner_face = f.frame_inner_width_mm / 2.0
    rail_outer = inner_face + f.rail_width_mm
    cms = [s for s in blue["build_steps"] if s["id"].startswith("crossmember_")
           and s["boolean"] == "create"]
    assert len(cms) == f.crossmember_count
    for s in cms:
        (xlo, xhi), (ylo, yhi), (zlo, zhi) = _prism_world_bbox(s)
        # the Y span reaches past each inner face (physically connects) but stays
        # within the rail outer faces (does not stick out past the rails)
        assert ylo <= -inner_face and yhi >= inner_face          # bridges rail-to-rail
        assert ylo >= -rail_outer - 1e-6 and yhi <= rail_outer + 1e-6
        # distributed along the wheelbase
        xc = (xlo + xhi) / 2.0
        assert -f.wheelbase_mm / 2.0 <= xc <= f.wheelbase_mm / 2.0


def test_battery_tray_sits_low_between_rails():
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    f, b = p.frame, p.battery_tray
    (xlo, xhi), (ylo, yhi), (zlo, zhi) = _prism_world_bbox(by["battery_tray"])
    # centred on x = 0, within the wheelbase
    assert (xlo + xhi) / 2.0 == pytest.approx(0.0)
    assert xlo >= -f.wheelbase_mm / 2.0 and xhi <= f.wheelbase_mm / 2.0
    # fits between the rail inner faces (with the side clearance)
    inner_face = f.frame_inner_width_mm / 2.0
    assert ylo >= -inner_face and yhi <= inner_face
    # LOW in Z: tray bottom on the ground-clearance datum, below the rail tops
    z = bp._z_layout(p)
    assert zlo == pytest.approx(z["tray_bottom"])
    assert zhi <= z["rail_top"]
    assert zlo < z["rail_bottom"] + 1e-6        # tray is the floor, under the rails


def test_rails_and_tray_share_a_z_datum():
    """The rails sit directly on top of the tray height band: rail bottom == tray top
    (a shared structural datum, not an exploded pile)."""
    p = ChassisParams()
    z = bp._z_layout(p)
    assert z["rail_bottom"] == pytest.approx(z["tray_top"])


def test_subframe_pads_land_on_solid_rail_straddling_the_relief():
    """FOUR subframe mount pads per axle (fore + aft, l + r) STRADDLE the axle relief
    window at x = axle ± pad_reach, on the rail centre-line y = ±rail_cl, at the rail
    top. The mount is a bolt circle drilled DOWN into the SOLID extended rail (NO boss --
    the subframe carries its own flange to the rail top, so a chassis boss would clash
    with it); the old design floated a boss IN the relief window where every bolt missed."""
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    f, s = p.frame, p.subframe
    rail_cl = f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0
    z = bp._z_layout(p)
    for axle, ax in (("front", +f.wheelbase_mm / 2.0), ("rear", -f.wheelbase_mm / 2.0)):
        x_sign = -1.0 if axle == "front" else 1.0
        for fore_aft in ("fore", "aft"):
            fa_sign = 1.0 if fore_aft == "fore" else -1.0
            ex = ax + x_sign * fa_sign * s.pad_reach_mm
            for side, ey in (("l", -rail_cl), ("r", +rail_cl)):
                px, py, pz = bp.subframe_pad_centre(p, axle, fore_aft, side)
                assert px == pytest.approx(ex)
                assert py == pytest.approx(ey)
                assert pz == pytest.approx(z["rail_bottom"])    # mate the rail UNDERSIDE
                # the pad's bolt circle exists and drills the solid rail of that side
                bid = "subframe_bolt_%s_%s_%s_0" % (axle, fore_aft, side)
                assert bid in by, bid
                assert by[bid]["boolean"] == "subtract" and by[bid]["target"] == "rail_%s" % side
    # no floating boss bodies any more
    assert not any(sid.startswith("subframe_boss") for sid in by)


def test_subframe_pads_straddle_the_axle_stations():
    """The fore/aft pads bracket the axle x-station (one inboard, one outboard) so the
    cradle bolts down both sides of the half-shaft / control-arm relief window."""
    p = ChassisParams()
    for axle, ax in (("front", +p.frame.wheelbase_mm / 2.0),
                     ("rear", -p.frame.wheelbase_mm / 2.0)):
        xs = sorted(bp.subframe_pad_centre(p, axle, fa, "r")[0] for fa in ("fore", "aft"))
        assert xs[0] < ax < xs[1]                                   # straddle the axle
        for x in xs:
            assert abs(abs(x - ax) - p.subframe.pad_reach_mm) < 1e-6


def test_chassis_pads_coincide_with_subframe_flanges():
    """THE MATING CONTRACT: the chassis bolt-pad centres (vehicle frame) must equal the
    subframe pad-flange centres as a POINT SET per axle, so the two bolt patterns line up
    hole-for-hole. The subframe is placed at (axle_x, 0, 0) identity, so its local pad +
    axle_x is the vehicle pad. (l/r labels differ between the packages -- the chassis rail
    'l' is -Y, the subframe 'l' is +Y -- but the geometry must coincide.)"""
    sub_params = pytest.importorskip("subframe_nx.params")
    p = ChassisParams()
    for axle, ax in (("front", +p.frame.wheelbase_mm / 2.0),
                     ("rear", -p.frame.wheelbase_mm / 2.0)):
        sp = sub_params.SubframeParams(axle=axle)
        sub_pads = {tuple(round(c, 3) for c in (ax + lx, ly, lz))
                    for fa in ("fore", "aft") for sd in ("l", "r")
                    for (lx, ly, lz) in [sp.pad_centre_local(fa, sd)]}
        chassis_pads = {tuple(round(c, 3) for c in bp.subframe_pad_centre(p, axle, fa, sd))
                        for fa in ("fore", "aft") for sd in ("l", "r")}
        assert chassis_pads == sub_pads, (axle, sorted(chassis_pads), sorted(sub_pads))


def test_crush_cans_extend_beyond_the_wheelbase_along_x():
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    f = p.frame
    rail_end = bp.rail_end_x(p)
    half_len = f.overall_length_mm / 2.0
    # front can: butts onto the EXTENDED rail end, reaching the front end of the platform
    (fxlo, fxhi), _, _ = _prism_world_bbox(by["crush_front_l"])
    assert fxlo == pytest.approx(rail_end)           # starts at the extended rail end
    assert fxhi == pytest.approx(half_len)           # reaches the nose
    assert fxhi > rail_end
    # rear can: butts onto the extended rear rail end, reaching the rear end
    (rxlo, rxhi), _, _ = _prism_world_bbox(by["crush_rear_l"])
    assert rxhi == pytest.approx(-rail_end)          # ends at the extended rail end
    assert rxlo == pytest.approx(-half_len)          # reaches the tail
    assert rxlo < -rail_end


def test_crush_cans_are_coaxial_with_the_rails():
    """Each crush can shares its rail's Y centre-line and Z band (it butts onto the
    rail end), so the crash load path runs straight into the rail."""
    p = ChassisParams()
    blue = bp.generate(p)
    by = _steps_by_id(blue)
    for side in ("l", "r"):
        _, rail_y, rail_z = _prism_world_bbox(by["rail_%s" % side])
        for tag in ("front", "rear"):
            _, can_y, can_z = _prism_world_bbox(by["crush_%s_%s" % (tag, side)])
            assert can_y == pytest.approx(rail_y)
            assert can_z == pytest.approx(rail_z)


def test_every_prism_beam_is_hollow():
    """Each box beam create has a matching concentric subtract (hollow section)."""
    blue = bp.generate(ChassisParams())
    ids = {s["id"] for s in blue["build_steps"]}
    for bid in ("rail_l", "rail_r", "battery_tray", "crush_front_l", "crossmember_0"):
        assert ("%s_hollow" % bid) in ids


def test_body_mounts_drilled_down_into_rail_tops():
    """Body-mount holes are -Z holes on the rail tops (ICD: body mounts along the rail
    tops)."""
    p = ChassisParams()
    blue = bp.generate(p)
    z = bp._z_layout(p)
    holes = [s for s in blue["build_steps"] if s["id"].startswith("body_mount_")]
    assert holes
    rail_cl = p.frame.frame_inner_width_mm / 2.0 + p.frame.rail_width_mm / 2.0
    for s in holes:
        assert s["kind"] == "hole" and s["boolean"] == "subtract"
        assert tuple(s["axis"]) == (0.0, 0.0, -1.0)            # drilled downward
        assert s["target"] in ("rail_l", "rail_r")
        assert abs(abs(s["cy"]) - rail_cl) < 1e-6             # on a rail centre-line
        assert s["z0"] >= z["rail_top"]                        # starts above the rail top
