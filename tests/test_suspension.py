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
    """The redesign forbids flat +Z plates for the links: the control-arm LEGS must run
    along their true 3D axes (axis not a pure +Z column), the toe/tie link a round bar
    along its true axis, and every spring/damper body axis-placed (origin3 set).
    Ball-joint hub bosses / eye-ends (round) are allowed alongside the legs."""
    blue = bp.generate(SuspensionParams())
    # the A-arm LEGS (the structural members) are round bars; assert they are true 3D.
    # The arm hub EYE (a ring coaxial with the near-vertical kingpin axis) is NOT a leg,
    # so filter to the leg bodies (ids carry "fore"/"aft", and exclude the bored eyes).
    leg_steps = [s for s in blue["build_steps"]
                 if s["role"] in ("lower_arm", "upper_arm")
                 and s["kind"] in ("prism", "loft_twist", "cylinder")
                 and ("fore" in s["id"] or "aft" in s["id"]) and "eye" not in s["id"]]
    assert leg_steps
    for s in leg_steps:
        ax = s["axis"]
        # a true leg runs predominantly inboard (-Y), never a pure +Z column
        assert abs(ax[1]) > 1e-6, s["id"]
        assert not (abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9), s["id"]
    # every link role is axis-placed along its true 3D axis (origin3 set, never a
    # legacy +Z column).  Eyes/perches/top-mount discs are deliberately on a joint PIVOT
    # axis (which may be vertical -- a real tapered-stud tie-rod / drop-link rod-end), so
    # the +Z check applies to the structural SHANK / LEG members, not the joint discs.
    for role in ("lower_arm", "upper_arm", "toe_link", "spring", "damper", "anti_roll_bar"):
        st = [s for s in blue["build_steps"] if s["role"] == role]
        assert st, role
        for s in st:
            assert s.get("origin3") is not None, s["id"]
        shanks = [s for s in st if "eye" not in s["id"] and "perch" not in s["id"]
                  and "mount" not in s["id"] and "turn" not in s["id"]]
        for s in shanks:
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
    tol = 55.0  # mm: a ball-joint boss / bushing eye radius + the kingpin BJ offset

    # control-arm legs: each leg is now ONE lofted taper whose two ENDS run from near its
    # inboard pickup to near the ball joint (the leg starts at the bored pickup eye and
    # ends at the arm hub eye, offset from the ball joint along the kingpin).
    cases = [
        ("lower_arm_fore_r", "lower_pickup_fore", "lower_ball_joint"),
        ("lower_arm_aft_r", "lower_pickup_aft", "lower_ball_joint"),
        ("upper_arm_fore_r", "upper_pickup_fore", "upper_ball_joint"),
        ("upper_arm_aft_r", "upper_pickup_aft", "upper_ball_joint"),
    ]
    for leg_id, in_hp, out_hp in cases:
        e0, e1 = _axis_endpoints(steps[leg_id])     # the lofted leg's two world ends
        assert min(_dist(e0, hp[in_hp]), _dist(e1, hp[in_hp])) <= tol, leg_id
        assert min(_dist(e0, hp[out_hp]), _dist(e1, hp[out_hp])) <= tol, leg_id

    # toe / tie link spans the toe pickup -> toe outboard (steering-arm) point
    a, b = _axis_endpoints(steps["toe_link_r"])
    assert _near_any(a, [hp["toe_pickup"], hp["toe_outboard"]], tol)
    assert _near_any(b, [hp["toe_pickup"], hp["toe_outboard"]], tol)

    # anti-roll DROP LINK: the link runs parallel to the bar-end -> arm line but OFFSET
    # clear of the arm (so it does not bury into it); a cast BRACKET on the lower arm
    # reaches from the ARB pickup out to the link's lower eye.  Assert (a) the bracket
    # roots on the arm at the ARB lower hardpoint, and (b) the link spans two points the
    # same DISTANCE apart as the two ARB hardpoints (it is the drop link, just shifted).
    # the bracket roots on the arm at the ARB lower hardpoint (one of its segment ends
    # sits there); check the closest bracket-segment endpoint.
    br_ends = []
    for sid in ("antiroll_bracket_r_s0", "antiroll_bracket_r_s1", "antiroll_bracket_r_s2"):
        br_ends.extend(_axis_endpoints(steps[sid]))
    assert min(_dist(e, hp["arb_link_lower"]) for e in br_ends) <= tol
    a, b = _axis_endpoints(steps["antiroll_r"])
    link_len = _dist(a, b)
    hp_len = _dist(hp["arb_link_lower"], hp["arb_link_upper"])
    assert abs(link_len - hp_len) <= tol               # spans the drop-link length


def test_coilover_seats_on_the_two_damper_hardpoints():
    """The coil-over reaches from its lower seat (damper_lower) on the lower arm up to
    its top mount (damper_top) -- it is not a floating cylinder. Asserted from the
    hardpoint table."""
    p = SuspensionParams()
    hp = eng.hardpoints(p)
    blue = bp.generate(p)
    steps = {s["id"]: s for s in blue["build_steps"]}
    seat, _ = _axis_endpoints(steps["damper_r"])         # damper body base = lower seat
    _, rod_top = _axis_endpoints(steps["damper_rod_r"])  # rod tip ~ top mount
    assert _dist(seat, hp["damper_lower"]) <= 5.0
    # the rod runs a touch PAST the top mount so its boss is fully bored, so allow the
    # small overshoot (the rod tip is one spring-wire diameter beyond damper_top).
    assert _dist(rod_top, hp["damper_top"]) <= 20.0


def test_coilover_is_coaxial_spring_around_damper():
    """The COIL SPRING must be coaxial AROUND the damper: every coil turn is centred on
    the damper working axis, and the coil BORE clears the damper body (the spring wraps
    the rod, it is not a separate bare tube and never overlaps it). The realism fix.
    Each turn is a kind='tube' ring, so its bore is its inner_radius."""
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
        assert t["kind"] == "tube", t["id"]                  # a real hollow ring
        # the turn CENTRE (origin3 is one face; centre = origin3 + axis*length/2)
        wt = _unit(t["axis"])
        c = tuple(t["origin3"][i] + wt[i] * t["length"] / 2.0 for i in range(3))
        d = (c[0] - base[0], c[1] - base[1], c[2] - base[2])
        off = math.sqrt(sum(x * x for x in _cross(d, w)))   # distance of turn centre to axis
        assert off < 1.0, t["id"]                            # turn centre ON the damper axis
        # the coil bore is larger than the damper body radius -> it wraps around it
        assert t["inner_radius"] > damper_r, t["id"]
        # and the turn axis is the damper axis (concentric, not skew)
        assert _dist(wt, w) < 1e-6, t["id"]


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


# --------------------------------------------------------------------------- #
# REAL, CLEAN JOINTS -- dedicated bolts, coaxial same-Ø holes, eyes united into
# their link, no self-intersecting sections (so it BUILDS in NX), and no solid
# interpenetration BY CONSTRUCTION (the redesign brief).
#
# IMPORTANT: vehicle_nx/clearance.py is BLIND to `subtract` voids and treats a
# `tube` as a SOLID disc, so it gives FALSE interpenetration readings on bored
# parts (a bolt in a bored eye, a hollow knuckle barrel near an offset arm).  The
# DEFINITIVE no-interpenetration arbiter is verification/nx_inspect.py (real NX
# point-in-solid containment, which DOES see the subtracted bores) -- run in NX.
# These tests therefore prove cleanliness STRUCTURALLY (united members, coaxial
# same-Ø bores, members separated along the joint axis and bridged only by a
# bolt/stud/sleeve sitting in a subtracted bore), and use clearance.py ONLY for
# gross solid-member overlap it can actually see (the two arms, arm vs link).
# --------------------------------------------------------------------------- #
from suspension_nx import fasteners as F            # noqa: E402
from vehicle_nx import clearance as clr             # noqa: E402

_IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _by_id(blue):
    return {s["id"]: s for s in blue["build_steps"]}


def _tube_bore_od(step):
    """(bore_d, outer_d) of a kind='tube' ring body."""
    assert step["kind"] == "tube", step["id"]
    return 2.0 * step["inner_radius"], 2.0 * step["outer_radius"]


def _eye_bore(blue, eye_id):
    """The through-bore diameter of a `united_eye` (a unite boss + a '<id>_bore'
    subtract cylinder).  Read from the subtract step the helper emits."""
    steps = _by_id(blue)
    bore = steps[eye_id + "_bore"]
    assert bore["boolean"] == "subtract", eye_id
    return 2.0 * bore["outer_radius"]


# --------------------------------------------------------------------------- #
# (1) it must BUILD in NX: no self-intersecting sections / single-loop annulus
# --------------------------------------------------------------------------- #
def test_no_self_intersecting_sections_every_hollow_is_outer_plus_inner():
    """NX's Section rejects a single profile holding an outer AND an inner loop as
    "self intersecting" (the failure the coordinator hit in real NX).  So every prism
    profile must be ONE simple loop (a 4-point rectangle here), and every HOLLOW must
    be either a kind='tube' (outer create + inner subtract, handled by the builder) or
    a `united_eye` (a solid boss UNITE + a separate bore SUBTRACT) -- never an annulus
    profile.  This guards the "builds in NX" property by construction."""
    for t in TYPES:
        blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": t}))
        for s in blue["build_steps"]:
            if s["kind"] == "prism" and s.get("profile") is not None:
                # a simple, non-self-intersecting loop -- a rectangle leg/web section
                assert len(s["profile"]) == 4, (t, s["id"], len(s["profile"]))
            if s["kind"] == "tube":                  # outer+inner, the builder bores it
                assert 0.0 < s["inner_radius"] < s["outer_radius"], (t, s["id"])
        # every hollow eye/bore that joins a member is a unite-boss + subtract-bore PAIR
        ids = {s["id"]: s for s in blue["build_steps"]}
        for s in blue["build_steps"]:
            if s["id"].endswith("_bore"):
                assert s["boolean"] == "subtract", (t, s["id"])
                boss = ids.get(s["id"][:-5])
                assert boss is not None and boss["boolean"] == "unite", (t, s["id"])


# --- guard the "Tool body completely outside target body" failure class --------- #
def _step_bbox(s):
    """AABB of a build-step body (cylinder/tube/prism), in the part's local frame."""
    o = s["origin3"]
    w = _unit(s["axis"])
    L = s["length"]
    if s["kind"] in ("cylinder", "tube"):
        r = s["outer_radius"]
    else:                                            # prism: max section vertex radius
        r = max(math.hypot(pu, pv) for (pu, pv) in s["profile"])
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for tt in (0.0, L):
        c = [o[i] + w[i] * tt for i in range(3)]
        for i in range(3):
            lo[i] = min(lo[i], c[i] - r)
            hi[i] = max(hi[i], c[i] + r)
    return lo, hi


def _boxes_overlap(a, b):
    (alo, ahi), (blo, bhi) = a, b
    return all(alo[i] <= bhi[i] and blo[i] <= ahi[i] for i in range(3))


@pytest.mark.parametrize("t", TYPES)
def test_every_subtract_bore_hits_its_target(t):
    """REGRESSION GUARD for the NX "Tool body completely outside target body" failure:
    replay the build steps in order; for every boolean=='subtract', the ACCUMULATED
    solid of its target (every create/unite body sharing that target emitted BEFORE the
    subtract) must overlap the bore tool's bbox.  If it does not, NX rejects the bore
    (it lands outside the target) -- leaving the feature SOLID and causing interference.
    (This is exactly why three bores failed in NX while the interference check passed --
    the perches/top-mount/ARB-bracket-eye are now STANDALONE tubes whose bore is coaxial
    by construction, so the only subtracts left are real ones that DO hit their target.)"""
    blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": t}))
    accum = {}                                       # target id -> [bbox, ...] so far
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            accum.setdefault(s["id"], []).append(_step_bbox(s))
        elif s["boolean"] == "unite":
            accum.setdefault(s["target"], []).append(_step_bbox(s))
        elif s["boolean"] == "subtract":
            tgt = s.get("target")
            boxes = accum.get(tgt)
            assert boxes, (t, s["id"], "subtract before its target create")
            tlo = [min(b[0][i] for b in boxes) for i in range(3)]
            thi = [max(b[1][i] for b in boxes) for i in range(3)]
            assert _boxes_overlap(_step_bbox(s), (tlo, thi)), (
                t, "%s bore lands OUTSIDE target %s (NX would FAIL the subtract)"
                % (s["id"], tgt))


def test_every_boolean_targets_an_existing_create_after_redesign():
    """Every subtract/unite (including the new fastener bores / united eyes) targets a
    body an earlier create made -- so the NX builder never hits a missing-target skip."""
    for t in TYPES:
        blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": t}))
        created = set()
        for s in blue["build_steps"]:
            if s["boolean"] == "create":
                created.add(s["id"])
            else:
                assert s["target"] in created, (t, s["id"], s["target"])


def test_every_body_is_named():
    """G) every build-step body carries a body_name (including the arm legs and every
    fastener / sleeve) -- nothing is left "(unnamed)" as the NX inspector found."""
    for t in TYPES:
        for s in bp.generate(SuspensionParams().overridden(**{"geometry.type": t}))["build_steps"]:
            assert s.get("body_name"), (t, s["id"])


# --------------------------------------------------------------------------- #
# (2) clean BY CONSTRUCTION -- structural assertions (the NX inspector confirms)
# --------------------------------------------------------------------------- #
def test_each_member_is_one_united_body_with_its_eyes():
    """B) each link is ONE body: its eyes UNITE into its create (no separate eye body
    overlapping its own link).  The toe link + its two eye bosses, and the anti-roll
    link + its two eye bosses, must each be a single part."""
    blue = bp.generate(SuspensionParams())
    steps = _by_id(blue)
    for create_id, eye_ids in (
            ("toe_link_r", ("toe_eye_in_r", "toe_eye_out_r")),
            ("antiroll_r", ("antiroll_eye_lo_r", "antiroll_eye_hi_r"))):
        assert steps[create_id]["boolean"] == "create"
        for eid in eye_ids:
            assert steps[eid]["boolean"] == "unite", eid           # boss unites in
            assert steps[eid]["target"] == create_id, eid
            assert steps[eid + "_bore"]["target"] == create_id, eid  # bore cuts the link


def test_arm_legs_and_hub_are_one_united_body():
    """Each control arm is ONE body: the hub eye (create) + fore + aft leg segments
    (unite) all share the single arm create id -- so the arm never overlaps its own
    legs/eye (the LOWER_ARM_HUB <-> arm-leg interference is gone by union)."""
    blue = bp.generate(SuspensionParams())
    for role, hub_id in (("lower_arm", "lower_arm_hub_r"), ("upper_arm", "upper_arm_hub_r")):
        members = [s for s in blue["build_steps"] if s["role"] == role]
        creates = [s for s in members if s["boolean"] == "create"]
        assert [c["id"] for c in creates] == [hub_id], role        # exactly one body
        for s in members:
            if s["boolean"] == "unite":
                assert s["target"] == hub_id, s["id"]


def test_every_pin_joint_has_a_dedicated_bolt():
    """C) every pin joint between two members is a real bolted joint: a shank + head +
    nut.  Assert a fastener trio exists at each inboard pickup, each toe joint, and each
    anti-roll-link end, and that the shank is a SOLID round cylinder."""
    blue = bp.generate(SuspensionParams())
    steps = _by_id(blue)
    joints = ["lower_fore", "lower_aft", "upper_fore", "upper_aft",
              "toe_in", "toe_out", "antiroll_lo", "antiroll_hi"]
    for j in joints:
        for part in ("shank", "head", "nut"):
            assert "%s_%s_r" % (j, part) in steps, (j, part)
        assert steps["%s_shank_r" % j]["kind"] == "cylinder", j


def test_pin_joint_bolt_is_thinner_than_the_bore_it_threads():
    """C) the bolt shank Ø is the bore Ø minus a clearance, so the shank fills the
    subtracted void without reaching the eye/sleeve material (no solid overlap) -- the
    join is a bolt-in-hole, the physically correct clean fit."""
    blue = bp.generate(SuspensionParams())
    steps = _by_id(blue)
    # toe outboard clevis: the bolt threads the knuckle steering-eye bore
    steer_bore = _eye_bore(blue, "knuckle_steer_eye_r")
    assert 2.0 * steps["toe_out_shank_r"]["outer_radius"] < steer_bore
    # inboard bushing joints: the bolt threads the steel sleeve bore (a tube)
    for j in ("lower_fore", "lower_aft", "toe_in"):
        sleeve_bore, _ = _tube_bore_od(steps["%s_sleeve_r" % j])
        assert 2.0 * steps["%s_shank_r" % j]["outer_radius"] < sleeve_bore, j


def test_ball_joint_bridges_arm_eye_and_knuckle_socket_separated_on_kingpin():
    """D) the ball joint is ONE body (a solid HOUSING disc + a UNITED STUD) bridging the
    ARM eye and the KNUCKLE socket, which are SEPARATED along the kingpin axis by the
    ball-joint length -- so the arm and the knuckle never share volume.  The housing OD
    is the arm-eye bore minus a clearance (a clean press fit) and the stud OD is the
    knuckle-socket bore minus a clearance: coaxial, bridged only by the ball joint."""
    p = SuspensionParams()
    blue = bp.generate(p)
    steps = _by_id(blue)
    hp = eng.hardpoints(p)
    kp_axis = _unit(tuple(hp["upper_ball_joint"][i] - hp["lower_ball_joint"][i] for i in range(3)))
    for arm, jhp, sock in (("lower", "lower_ball_joint", "knuckle_lbj_socket_r"),
                           ("upper", "upper_ball_joint", "knuckle_ubj_socket_r")):
        housing = steps["%s_bj_housing_r" % arm]            # SOLID disc in the arm-eye bore
        stud = steps["%s_bj_stud_r" % arm]                  # rod into the knuckle socket
        arm_eye = steps["%s_arm_hub_r" % arm]               # the arm hub is a tube ring
        # housing + stud are ONE body (the stud unites into the housing create)
        assert housing["boolean"] == "create" and housing["kind"] == "cylinder", arm
        assert stud["boolean"] == "unite" and stud["target"] == housing["id"], arm
        # housing OD fits the arm-eye bore with a clearance (a clean press fit)
        arm_eye_bore, _ = _tube_bore_od(arm_eye)
        hous_od = 2.0 * housing["outer_radius"]
        assert hous_od < arm_eye_bore, arm
        assert hous_od > arm_eye_bore - 4.0 * F.FIT_CLEARANCE - 0.1, arm
        # stud OD fits the knuckle-socket bore with a clearance
        sock_bore = _eye_bore(blue, sock)
        assert 2.0 * stud["outer_radius"] < sock_bore, arm
        # the arm eye (housing centre) and the knuckle socket sit on OPPOSITE sides of
        # the joint along the kingpin axis, ~_BJ_HALF_SEP each -- the stud spans the gap.
        joint = hp[jhp]
        hc = tuple(housing["origin3"][i] + _unit(housing["axis"])[i] * housing["length"] / 2.0
                   for i in range(3))
        sep = abs(sum((hc[i] - joint[i]) * kp_axis[i] for i in range(3)))
        assert sep == pytest.approx(bp._BJ_HALF_SEP, abs=3.0), arm


def test_inboard_bushing_sleeve_fills_can_bore_and_bolt_clears(  ):
    """E) at each inboard pickup the bushing is concentric with STRICT diameter nesting
    and a clearance gap at every step (so NX point-in-solid reads it ~0, not as an
    interference): can OD > can bore > sleeve OD > sleeve bore > bolt shank, each with a
    real radial gap.  The sleeve sits in the can's subtracted void, the bolt in the
    sleeve's; all coaxial."""
    blue = bp.generate(SuspensionParams())
    steps = _by_id(blue)
    for joint in ("lower_fore", "lower_aft", "upper_fore", "upper_aft"):
        can = steps["%s_can_r" % joint]
        sleeve = steps["%s_sleeve_r" % joint]
        shank = steps["%s_shank_r" % joint]
        assert _dist(_unit(can["axis"]), _unit(sleeve["axis"])) < 1e-6, joint   # coaxial
        can_od, can_bore = 2.0 * can["outer_radius"], 2.0 * can["inner_radius"]
        sl_od, sl_bore = 2.0 * sleeve["outer_radius"], 2.0 * sleeve["inner_radius"]
        bolt = 2.0 * shank["outer_radius"]
        gap = 2.0 * F.FIT_CLEARANCE - 1e-6
        assert can_od > can_bore, joint
        assert sl_od <= can_bore - gap, joint          # sleeve clears the can bore
        assert sl_bore > bolt, joint                   # bolt clears the sleeve bore


@pytest.mark.parametrize("t", TYPES)
def test_distinct_solid_members_do_not_grossly_overlap(t):
    """A gross-overlap guard using the part of clearance.py that is RELIABLE: SOLID
    members that should be FULLY APART -- the two control arms, and the lower arm vs the
    toe link -- must not interpenetrate.  (Bored/hollow pairs -- the knuckle, eyes,
    bushings, bolts -- and bolted attachments -- the ARB drop link onto the lower arm --
    are NOT checked here because clearance.py is blind to their subtracted bores / shared
    joint and would over-report; nx_inspect.py is their arbiter.)"""
    blue = bp.generate(SuspensionParams().overridden(**{"geometry.type": t}))
    steps = blue["build_steps"]

    def solids_of(create_id):
        # the create body + everything united into it, as oriented solids
        ids = {create_id} | {s["id"] for s in steps
                             if s["boolean"] == "unite" and s.get("target") == create_id}
        out = []
        for s in steps:
            if s["id"] in ids:
                b = clr._world_body(s, _IDENTITY, [0.0, 0.0, 0.0])
                if b is not None:
                    out.append(b)
        return out

    # members that are genuinely separated (not joined by a shared bolt/bore):
    # the lower arm <-> upper arm, and the lower arm <-> toe link.
    pairs = [("lower_arm_hub_r", "toe_link_r")]
    if t in ("multilink", "double_wishbone"):
        pairs.append(("lower_arm_hub_r", "upper_arm_hub_r"))
        pairs.append(("upper_arm_hub_r", "toe_link_r"))
    for a, b in pairs:
        res = clr.solids_interpenetrate(solids_of(a), solids_of(b), touch_tol=clr.TOUCH_TOL_MM)
        assert res is None or res[0] <= clr.TOUCH_TOL_MM, (t, a, b, res)
