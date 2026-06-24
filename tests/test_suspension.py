"""Tests for suspension_nx (NX-independent layers): params round-trip, engineering
rates + validation, the 3D hardpoint table, and the geometry blueprint's structural
integrity + WORLD bounding boxes.

The redesign (ICD-09 section 3) re-datums the corner onto the WHEEL-HUB CENTRE
(local origin) and models every link along its TRUE 3D axis with kind="prism" /
axis-placed cylinders. These tests assert that contract: hub bore on the origin,
inboard pickups at -Y, links spanning hub -> inboard, and sane world boxes per type.
"""

import json
import math

import pytest

from motor_nx.blueprint import prism_frame, profile_to_world
from suspension_nx import blueprint as bp
from suspension_nx import engineering as eng
from suspension_nx.params import SuspensionParams

TYPES = ("multilink", "double_wishbone", "macpherson")


# --------------------------------------------------------------------------- #
# helpers: map a build step to its world point cloud (NX-free, via the twins)
# --------------------------------------------------------------------------- #
def _unit(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _step_world_points(s):
    """World (x,y,z) corner points of a build step, using only the pure-math twins.
    prism      -> the section rectangle at both ends of the extrude;
    cylinder   -> the two end-centres +/- radius along the two perpendicular axes;
    tube       -> same as cylinder (outer radius)."""
    kind = s["kind"]
    if kind == "prism":
        near = profile_to_world(s["profile"], tuple(s["origin3"]),
                                tuple(s["axis"]), tuple(s["u_dir"]))
        w = _unit(s["axis"])
        L = s["length"]
        far = [(x + w[0] * L, y + w[1] * L, z + w[2] * L) for (x, y, z) in near]
        return near + far
    if kind in ("cylinder", "tube") and s.get("origin3") is not None:
        base = tuple(s["origin3"])
        w = _unit(s["axis"])
        L = s["length"]
        r = s["outer_radius"]
        # a stable perpendicular for the radial extent
        helper = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
        u = _unit((w[1] * helper[2] - w[2] * helper[1],
                   w[2] * helper[0] - w[0] * helper[2],
                   w[0] * helper[1] - w[1] * helper[0]))
        v = (w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0])
        pts = []
        for t in (0.0, L):
            c = (base[0] + w[0] * t, base[1] + w[1] * t, base[2] + w[2] * t)
            for d in (u, v):
                pts.append((c[0] + r * d[0], c[1] + r * d[1], c[2] + r * d[2]))
                pts.append((c[0] - r * d[0], c[1] - r * d[1], c[2] - r * d[2]))
        return pts
    return []


def _blueprint_bbox(blue):
    xs, ys, zs = [], [], []
    for s in blue["build_steps"]:
        for (x, y, z) in _step_world_points(s):
            xs.append(x)
            ys.append(y)
            zs.append(z)
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


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


def test_roll_centre_height_in_realistic_band():
    # a passenger-car geometric roll centre sits a little above the ground.
    for t in TYPES:
        g = eng.derive(SuspensionParams().overridden(**{"geometry.type": t}))
        assert 0.0 < g.roll_centre_height_mm < 250.0, t


def test_anti_feature_note_is_present():
    assert "anti-dive" in eng.derive(SuspensionParams()).anti_feature_note


def test_damping_ratio_is_wheel_frame_not_damper_frame():
    """The headline damping ratio must be the WHEEL-frame value (bump rate referred
    through MR^2 to match the wheel-frame stiffness): ~0.25, not the un-referred
    damper-frame ~0.65. The damper-frame value is reported separately for cross-check."""
    p = SuspensionParams()
    g = eng.derive(p)
    mr = p.spring.motion_ratio
    # the wheel-frame zeta is the damper-frame zeta scaled by MR^2 (both reported
    # values are rounded to 3 dp, so allow a small absolute tolerance)
    assert g.damping_ratio == pytest.approx(g.damping_ratio_damper_frame * mr ** 2, abs=1e-3)
    assert 0.15 <= g.damping_ratio <= 0.45            # lightly-damped ride (wheel frame)
    assert g.damping_ratio_damper_frame > g.damping_ratio   # damper-frame reads higher


def test_roll_centre_does_not_double_count_track():
    """ICD §3 re-datuming: the corner is hub-datumed, so the contact patch sits ~below
    the hub (offset only by the scrub radius), NOT at +track/2. Placing it at +track/2
    mixed the vehicle frame into the local kinematics (a double-count). The corrected
    RC stays in the plausible band and moves only MILDLY with track (it depends on the
    arm geometry + the centre-plane distance, not on a spurious +track/2 patch)."""
    base = eng.derive(SuspensionParams()).roll_centre_height_mm
    # a large track change must NOT swing the RC by anything like track/2 (the old
    # double-count would have moved it by ~tens of mm per 100 mm of track via the patch)
    wide = eng.derive(SuspensionParams().overridden(**{"geometry.track_width_mm": 1700.0}))
    assert 30.0 < base < 120.0
    assert abs(wide.roll_centre_height_mm - base) < 20.0      # mild, not a double-count
    # and a bigger scrub radius (which DOES move the contact patch) shifts the RC --
    # proving the patch is now tied to the scrub radius, not to the wheel's vehicle Y.
    more_scrub = eng.derive(SuspensionParams().overridden(**{"geometry.scrub_radius_mm": 60.0}))
    assert more_scrub.roll_centre_height_mm != pytest.approx(base, abs=0.5)


def test_toe_outboard_sits_inboard_of_the_hub():
    """The outboard toe (tie-rod) joint must sit inboard of the hub face (y < 0), on
    the steering arm -- never outboard of the wheel face (the low-severity finding)."""
    for t in TYPES:
        hp = eng.hardpoints(SuspensionParams().overridden(**{"geometry.type": t}))
        assert hp["toe_outboard"][1] < 0.0, t
    # validate() guards it too
    assert not any("toe_outboard" in i for i in eng.validate(SuspensionParams()))


def test_default_design_is_buildable():
    assert eng.validate(SuspensionParams()) == []


def test_every_type_is_buildable():
    for t in TYPES:
        assert eng.validate(SuspensionParams().overridden(**{"geometry.type": t})) == [], t


def test_validate_catches_arm_longer_than_half_track():
    p = SuspensionParams().overridden(**{"geometry.lower_arm_length_mm": 900.0})  # > 1580/2
    issues = eng.validate(p)
    assert any("lower_arm_length_mm" in i for i in issues)


def test_validate_catches_spring_deflection_over_free_length():
    p = SuspensionParams().overridden(**{"spring.ride_height_load_n": 99999.0})  # huge deflection
    issues = eng.validate(p)
    assert any("free_length" in i for i in issues)


def test_validate_catches_degenerate_upright():
    # a knuckle too short in height collapses the upper/lower ball-joint Z span so
    # the upright joints interfere around the hub bore (small bore keeps the other
    # bore/reach checks quiet so this check is exercised in isolation).
    p = SuspensionParams().overridden(**{"knuckle.height_mm": 40.0,
                                         "knuckle.hub_bore_diameter_mm": 30.0})
    issues = eng.validate(p)
    assert any("self-overlap" in i for i in issues)
    # the default (taller) knuckle is fine
    assert not any("self-overlap" in i for i in eng.validate(SuspensionParams()))


# --------------------------------------------------------------------------- #
# 3D hardpoint table (the redesign's datum contract)
# --------------------------------------------------------------------------- #
def test_hub_centre_is_local_origin():
    for t in TYPES:
        hp = eng.hardpoints(SuspensionParams().overridden(**{"geometry.type": t}))
        assert hp["hub_centre"] == (0.0, 0.0, 0.0), t


def test_inboard_pickups_sit_at_minus_y():
    """Inboard chassis pickups must be at local -Y (toward the chassis centreline),
    NOT at +track/2 -- the whole point of the re-datuming."""
    hp = eng.hardpoints(SuspensionParams())
    for nm in ("lower_pickup_fore", "lower_pickup_aft",
               "upper_pickup_fore", "upper_pickup_aft", "toe_pickup"):
        assert hp[nm][1] < 0.0, nm
        assert -hp[nm][1] < SuspensionParams().geometry.track_width_mm / 2.0, nm


def test_lower_pickups_reach_toward_chassis_by_arm_length():
    p = SuspensionParams()
    hp = eng.hardpoints(p)
    span = abs(hp["lower_pickup_fore"][1] - hp["lower_ball_joint"][1])
    assert span == pytest.approx(p.geometry.lower_arm_length_mm, rel=0.05)


def test_ball_joints_straddle_the_hub_in_z():
    hp = eng.hardpoints(SuspensionParams())
    assert hp["lower_ball_joint"][2] < 0.0 < hp["upper_ball_joint"][2]


def test_outboard_joints_reach_the_hub():
    p = SuspensionParams()
    hp = eng.hardpoints(p)
    reach = math.hypot(p.knuckle.width_mm, p.knuckle.height_mm)
    for jn in ("lower_ball_joint", "upper_ball_joint"):
        assert _dist(hp[jn], (0.0, 0.0, 0.0)) < reach


# --------------------------------------------------------------------------- #
# blueprint -- schema, integrity, and world geometry
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(SuspensionParams())
    assert blue["schema"] == "suspension_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 5
    assert "hardpoints" in blue and blue["hardpoints"]["hub_centre"] == [0.0, 0.0, 0.0]


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(SuspensionParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_links_are_true_3d_not_flat_z_plates():
    """The redesign forbids flat +Z plates for the links: the control-arm LEGS must be
    PRISMS along their true 3D axes (axis not a pure +Z column), the toe/tie link a
    round bar along its true axis, and every spring/damper body axis-placed (origin3
    set). Ball-joint hub bosses / eye-ends (round) are allowed alongside the legs."""
    blue = bp.generate(SuspensionParams())
    # the A-arm LEGS (the structural members) are the prisms; assert they are true 3D.
    leg_steps = [s for s in blue["build_steps"]
                 if s["role"] in ("lower_arm", "upper_arm") and s["kind"] == "prism"]
    assert leg_steps
    for s in leg_steps:
        ax = s["axis"]
        # a true leg runs predominantly inboard (-Y), never a pure +Z column
        assert abs(ax[1]) > 1e-6, s["id"]
        assert not (abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9), s["id"]
    # every link role is axis-placed along its true 3D axis (origin3 set, never a
    # legacy +Z column) -- arms, toe link, spring, damper, anti-roll.
    for role in ("lower_arm", "upper_arm", "toe_link", "spring", "damper", "anti_roll_bar"):
        st = [s for s in blue["build_steps"] if s["role"] == role]
        assert st, role
        for s in st:
            assert s.get("origin3") is not None, s["id"]
            ax = s["axis"]
            assert not (abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9), s["id"]


def test_hub_bore_is_on_the_lateral_axis_through_origin():
    """The hub bore is a cylinder coaxial with the lateral wheel-spin axis (local Y)
    passing through the origin -- it carries the wheel hub at (0,0,0)."""
    blue = bp.generate(SuspensionParams())
    bore = [s for s in blue["build_steps"] if s["role"] == "hub_bore_cut"][0]
    assert bore["kind"] == "cylinder"
    base = bore["origin3"]
    ax = _unit(bore["axis"])
    # base on the X=Z=0 line, axis along +/-Y
    assert abs(base[0]) < 1e-6 and abs(base[2]) < 1e-6
    assert abs(abs(ax[1]) - 1.0) < 1e-6
    # the bore spans through the origin
    assert base[1] < 0.0 < base[1] + bore["length"] * ax[1]


def test_world_bbox_is_sane_for_every_type():
    """World box: the corner reaches OUTBOARD only as far as the knuckle (small +Y),
    well INBOARD toward the chassis (large -Y, ~arm length), and brackets the hub in
    Z. It must never reach +track/2 in +Y (that would re-introduce the double-count)."""
    for t in TYPES:
        p = SuspensionParams().overridden(**{"geometry.type": t})
        blue = bp.generate(p)
        (xlo, xhi), (ylo, yhi), (zlo, zhi) = _blueprint_bbox(blue)
        half_track = p.geometry.track_width_mm / 2.0
        # outboard extent stays near the hub, far short of the half-track
        assert yhi < 0.3 * half_track, (t, yhi)
        # inboard extent reaches toward the chassis centreline (at least an arm)
        assert ylo < -0.8 * p.geometry.lower_arm_length_mm, (t, ylo)
        assert -ylo < half_track, (t, ylo)        # but not past the centreline
        # the corner brackets the hub vertically and rises to the body mount
        assert zlo < 0.0 < zhi, (t, zlo, zhi)
        assert zhi > 0.5 * p.geometry.ride_height_mm, (t, zhi)
        # longitudinal extent is bounded by the wishbone base + toe-link reach
        assert (xhi - xlo) < 2.0 * p.geometry.track_width_mm, t


def test_arms_span_hub_to_inboard():
    """Each control ARM (taken as the whole set of its build steps) must reach from
    near the hub (its outboard ball-joint boss, only slightly inboard of the hub) to
    well inboard (-Y) at its chassis pickups -- i.e. the A-arm genuinely spans the
    corner, it is not a stub near the hub nor a bar floating inboard."""
    p = SuspensionParams()
    blue = bp.generate(p)
    for role, length in (("lower_arm", p.geometry.lower_arm_length_mm),
                         ("upper_arm", p.geometry.upper_arm_length_mm)):
        ys = []
        for s in blue["build_steps"]:
            if s["role"] == role:
                ys += [q[1] for q in _step_world_points(s)]
        assert ys, role
        assert max(ys) < 0.0, role                 # whole arm is inboard of the hub face
        assert min(ys) < -0.7 * length, role       # reaches the chassis pickups
        # the outboard end (ball-joint boss) is close to the hub centre laterally
        assert max(ys) > -2.0 * p.knuckle.width_mm, role


def test_spring_and_damper_are_inclined_and_reach_up():
    """Spring & damper sit at a real inclination (not vertical) and reach from a low
    seat (below/at the hub) up to a body mount above the hub. The damper BODY and the
    COIL turns are all axis-placed along the same inclined working axis."""
    blue = bp.generate(SuspensionParams())
    # the damper body cylinder
    damper = [x for x in blue["build_steps"] if x["id"] == "damper_r"][0]
    ax = _unit(damper["axis"])
    assert ax[2] > 0.3                          # predominantly upward
    assert math.hypot(ax[0], ax[1]) > 0.05      # but inclined (real lean)
    # the whole coil-over (damper body + rod + top mount) reaches above the hub
    tops = []
    for r in ("damper", "spring"):
        for s in blue["build_steps"]:
            if s["role"] == r:
                w = _unit(s["axis"])
                tops.append(s["origin3"][2] + s["length"] * w[2])
    assert max(tops) > 0.0                       # body/tower mount is above the hub
    # the topmost coil turn sits above the hub too (the coil works up to the body)
    turns = [s for s in blue["build_steps"] if s["role"] == "spring" and "turn" in s["id"]]
    assert turns
    assert max(s["origin3"][2] for s in turns) > 0.0


def test_macpherson_drops_the_upper_arm_but_keeps_a_strut():
    blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": "macpherson"}))
    ids = [s["id"] for s in blue["build_steps"]]
    assert not any(i.startswith("upper_arm") for i in ids)
    # the strut (damper) is the upper link and rises above the upright top
    damper = [s for s in blue["build_steps"] if s["role"] == "damper"][0]
    top_z = damper["origin3"][2] + damper["length"] * _unit(damper["axis"])[2]
    assert top_z > SuspensionParams().knuckle.height_mm / 2.0


def test_axle_models_both_corners_mirrored():
    one = bp.generate(SuspensionParams())
    axle = bp.generate(SuspensionParams().overridden(corners="axle"))
    assert len(axle["build_steps"]) > len(one["build_steps"])
    ids = [s["id"] for s in axle["build_steps"]]
    assert any(i.endswith("_l") for i in ids) and any(i.endswith("_r") for i in ids)
    assert "knuckle_l" in ids and "knuckle_r" in ids
    # the mirrored corner's hub sits at -track in Y (the opposite wheel)
    track = SuspensionParams().geometry.track_width_mm
    bore_l = [s for s in axle["build_steps"] if s["id"] == "knuckle_hub_bore_l"][0]
    assert bore_l["origin3"][1] < -0.5 * track       # the other hub, fully inboard


def test_knuckle_carries_a_hub_bore():
    blue = bp.generate(SuspensionParams())
    assert any(s["role"] == "hub_bore_cut" for s in blue["build_steps"])


# --------------------------------------------------------------------------- #
# realism: every link visibly connects its TWO hardpoints (nothing mid-air)
# --------------------------------------------------------------------------- #
def _axis_endpoints(s):
    """The two world endpoints of an axis-placed body (the base origin3 and the far
    end origin3 + axis*length). Works for the prism legs and the axis-placed
    cylinders/tubes the redesign uses for every link."""
    o = tuple(s["origin3"])
    w = _unit(s["axis"])
    L = s["length"]
    return o, (o[0] + w[0] * L, o[1] + w[1] * L, o[2] + w[2] * L)


def _near_any(pt, targets, tol):
    return any(_dist(pt, t) <= tol for t in targets)


def test_every_link_spans_its_two_hardpoints():
    """The redesign's headline realism contract: every structural link's two ENDS sit
    on its two hardpoints (no link floating in mid-air). Asserted per link from the
    shared hardpoint table -- the single source of truth -- with a generous tolerance
    for the section/eye sizes wrapped onto the joints."""
    p = SuspensionParams()
    hp = eng.hardpoints(p)
    blue = bp.generate(p)
    steps = {s["id"]: s for s in blue["build_steps"]}
    tol = 40.0  # mm: a ball-joint boss / bushing eye radius around the hardpoint

    # control-arm legs: each leg runs from its inboard pickup to the shared ball joint
    cases = [
        ("lower_arm_fore_r_s0", "lower_pickup_fore", "lower_arm_fore_r_s2", "lower_ball_joint"),
        ("lower_arm_aft_r_s0", "lower_pickup_aft", "lower_arm_aft_r_s2", "lower_ball_joint"),
        ("upper_arm_fore_r_s0", "upper_pickup_fore", "upper_arm_fore_r_s2", "upper_ball_joint"),
        ("upper_arm_aft_r_s0", "upper_pickup_aft", "upper_arm_aft_r_s2", "upper_ball_joint"),
    ]
    for in_id, in_hp, out_id, out_hp in cases:
        in_a, _ = _axis_endpoints(steps[in_id])     # leg starts at the inboard pickup
        _, out_b = _axis_endpoints(steps[out_id])   # last segment ends at the ball joint
        assert _dist(in_a, hp[in_hp]) <= tol, in_id
        assert _dist(out_b, hp[out_hp]) <= tol, out_id

    # toe / tie link spans the toe pickup -> toe outboard (steering-arm) point
    a, b = _axis_endpoints(steps["toe_link_r"])
    assert _near_any(a, [hp["toe_pickup"], hp["toe_outboard"]], tol)
    assert _near_any(b, [hp["toe_pickup"], hp["toe_outboard"]], tol)

    # anti-roll drop link spans its two bar/arm hardpoints
    a, b = _axis_endpoints(steps["antiroll_r"])
    assert _near_any(a, [hp["arb_link_lower"], hp["arb_link_upper"]], tol)
    assert _near_any(b, [hp["arb_link_lower"], hp["arb_link_upper"]], tol)


def test_coilover_seats_on_the_two_damper_hardpoints():
    """The coil-over reaches from its lower seat (damper_lower) on the lower arm up to
    its top mount (damper_top) -- it is not a floating cylinder. Asserted from the
    hardpoint table."""
    p = SuspensionParams()
    hp = eng.hardpoints(p)
    blue = bp.generate(p)
    steps = {s["id"]: s for s in blue["build_steps"]}
    seat, _ = _axis_endpoints(steps["damper_r"])         # damper body base = lower seat
    _, rod_top = _axis_endpoints(steps["damper_rod_r"])  # rod tip = top mount
    assert _dist(seat, hp["damper_lower"]) <= 5.0
    assert _dist(rod_top, hp["damper_top"]) <= 5.0


def test_coilover_is_coaxial_spring_around_damper():
    """The COIL SPRING must be coaxial AROUND the damper: every coil turn is centred on
    the damper working axis, and the coil bore clears the damper body (the spring
    wraps the rod, it is not a separate bare tube). The defining realism fix."""
    blue = bp.generate(SuspensionParams())
    damper = [s for s in blue["build_steps"] if s["id"] == "damper_r"][0]
    base = tuple(damper["origin3"])
    w = _unit(damper["axis"])
    damper_r = damper["outer_radius"]
    turns = [s for s in blue["build_steps"] if s["role"] == "spring" and "turn" in s["id"]]
    assert len(turns) >= 3, "the coil must read as several turns, not one tube"

    def _cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    for t in turns:
        c = tuple(t["origin3"])
        d = (c[0] - base[0], c[1] - base[1], c[2] - base[2])
        off = math.sqrt(sum(x * x for x in _cross(d, w)))   # distance of turn centre to axis
        assert off < 1.0, t["id"]                            # turn centre ON the damper axis
        # the coil bore is larger than the damper body radius -> it wraps around it
        assert t["inner_radius"] > damper_r, t["id"]
        # and the turn axis is the damper axis (concentric, not skew)
        assert _dist(_unit(t["axis"]), w) < 1e-6, t["id"]


def test_knuckle_is_one_cast_body_uniting_features():
    """The upright reads as ONE cast body: a create plus several UNITE features (hub
    barrel + web + ball-joint arms + caliper bridge + steering arm), then the hub bore
    cut -- not a bare box. All unite/subtract target the single knuckle create."""
    blue = bp.generate(SuspensionParams())
    knuckle = [s for s in blue["build_steps"] if s["role"] == "knuckle"]
    creates = [s for s in knuckle if s["boolean"] == "create"]
    unites = [s for s in knuckle if s["boolean"] == "unite"]
    assert len(creates) == 1                       # exactly one cast body
    assert len(unites) >= 3                         # several cast features merged in
    for s in unites:
        assert s["target"] == creates[0]["id"]
    # the hub bore is cut through that same casting
    bore = [s for s in blue["build_steps"] if s["role"] == "hub_bore_cut"][0]
    assert bore["boolean"] == "subtract" and bore["target"] == creates[0]["id"]


def test_arms_are_a_arms_two_legs_one_ball_joint():
    """Each control arm is a proper A-ARM: a FORE and an AFT leg that converge on the
    SAME outboard ball-joint hub boss (not two isolated parallel bars). Asserted by
    both legs uniting into the one ball-joint hub create."""
    blue = bp.generate(SuspensionParams())
    for role, hub_id in (("lower_arm", "lower_arm_hub_r"), ("upper_arm", "upper_arm_hub_r")):
        legs = [s for s in blue["build_steps"]
                if s["role"] == role and s["boolean"] == "unite"]
        fore = [s for s in legs if "fore" in s["id"]]
        aft = [s for s in legs if "aft" in s["id"]]
        assert fore and aft, role                   # two distinct legs
        for s in legs:
            assert s["target"] == hub_id, s["id"]   # both converge on one ball-joint boss
