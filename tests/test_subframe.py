"""Tests for subframe_nx (NX-independent layers): params round-trip + hardpoint
table, engineering load paths / mass / bolt sizing + validation, and the geometry
blueprint's structural integrity + the ICD §7.4.2 mating ties (pads on the chassis
pads, pickup bosses on the suspension inboard hardpoints, tower at the damper top)."""

import json
import math

import pytest

from subframe_nx import blueprint as bp
from subframe_nx import engineering as eng
from subframe_nx.params import SubframeParams, _LEFT_HARDPOINTS_LOCAL

from motor_nx.blueprint import profile_to_world


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = SubframeParams()
    p2 = SubframeParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = SubframeParams.from_dict({"tower": {"post_diameter_mm": 80.0}})
    assert p.tower.post_diameter_mm == 80.0
    assert p.tower.seat_diameter_mm == SubframeParams().tower.seat_diameter_mm  # untouched
    assert p.axle == "rear"


def test_overridden_dotted():
    p = SubframeParams().overridden(**{"axle": "front", "pad.bolt_count": 6})
    assert p.axle == "front"
    assert p.pad.bolt_count == 6


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in SubframeParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


def test_hardpoints_match_icd_rear_left_vehicle_coords():
    """The local hardpoint table must reproduce the ICD §7.2 rear-left VEHICLE
    coordinates once the rear axle station (-1437.5,0,0) is added back."""
    p = SubframeParams()  # rear
    hp = p.hardpoints_local("l")
    axle_x = -1437.5
    expect_vehicle = {
        "lower_pickup_fore": (-1310.0, 344.0, 277.0),
        "lower_pickup_aft": (-1550.0, 344.0, 277.0),
        "upper_pickup_fore": (-1324.0, 424.0, 386.0),
        "upper_pickup_aft": (-1564.0, 424.0, 386.0),
        "toe_pickup": (-1606.0, 412.0, 343.0),
        "damper_top": (-1437.0, 427.0, 692.0),
    }
    for nm, (vx, vy, vz) in expect_vehicle.items():
        lx, ly, lz = hp[nm]
        assert (lx + axle_x, ly, lz) == pytest.approx((vx, vy, vz), abs=0.6)


def test_right_side_is_rz180_of_left():
    """The right corner is Rz(180) of the same canonical suspension corner, so its
    subframe-local boss negates BOTH local X and Y of the left (the +-track/2 offset is
    already folded into the per-side values), Z unchanged. (The OLD convention only
    mirrored Y and left X alone -- the four-corner-mismatch bug.)"""
    p = SubframeParams()
    l = p.hardpoints_local("l")
    r = p.hardpoints_local("r")
    for nm in _LEFT_HARDPOINTS_LOCAL:
        assert r[nm][0] == pytest.approx(-l[nm][0])
        assert r[nm][1] == pytest.approx(-l[nm][1])
        assert r[nm][2] == pytest.approx(l[nm][2])


def test_front_variant_is_not_x_mirrored():
    """The suspension is ONE canonical corner reused front and rear (the front axle is
    NOT X-mirrored -- only the placement origin's X changes). So the subframe boss table
    is IDENTICAL front and rear; the assembler places each at its axle station. (The OLD
    convention X-mirrored the front, which -- combined with the suspension not mirroring --
    is exactly what threw three of the four corners off.)"""
    rear = SubframeParams()  # rear
    front = SubframeParams().overridden(axle="front")
    for nm in _LEFT_HARDPOINTS_LOCAL:
        assert front.hardpoints_local("l")[nm] == pytest.approx(rear.hardpoints_local("l")[nm])


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_default_design_is_buildable():
    assert eng.validate(SubframeParams()) == []


def test_front_variant_is_buildable():
    assert eng.validate(SubframeParams().overridden(axle="front")) == []


def test_load_path_has_sound_margins():
    """The cradle side rail + the pad / pickup bolts must clear a >= 1.5 static SF."""
    g = eng.derive(SubframeParams())
    assert g.side_rail_safety_factor >= 1.5
    assert g.pad_bolt_safety_factor >= 1.5
    assert g.pickup_bolt_safety_factor >= 1.5


def test_tower_reaches_damper_top():
    g = eng.derive(SubframeParams())
    assert g.tower_reach_z_mm == pytest.approx(g.damper_top_z_mm)
    assert g.damper_top_z_mm == pytest.approx(692.0)


def test_higher_corner_load_lowers_safety_factor():
    light = eng.derive(SubframeParams().overridden(corner_vertical_load_n=8000.0))
    heavy = eng.derive(SubframeParams().overridden(corner_vertical_load_n=20000.0))
    assert heavy.side_rail_safety_factor < light.side_rail_safety_factor
    assert heavy.pickup_bolt_safety_factor < light.pickup_bolt_safety_factor


def test_mass_breakdown_positive_and_sums():
    g = eng.derive(SubframeParams())
    assert g.total_mass_kg > 0
    parts = g.cradle_mass_kg + g.pad_post_mass_kg + g.tower_mass_kg + g.boss_mass_kg
    assert g.total_mass_kg == pytest.approx(parts, rel=1e-3)


def test_validate_catches_unknown_axle():
    assert any("axle" in i for i in eng.validate(SubframeParams().overridden(axle="middle")))


def test_validate_catches_pad_off_chassis_y():
    """ICD §7.4.2: the pads must align to the chassis rail centre-line y=±585."""
    p = SubframeParams().overridden(**{"pad.pad_y_mm": 500.0})
    assert any("pad_y_mm" in i and "585" in i for i in eng.validate(p))


def test_validate_catches_thin_beam_wall():
    p = SubframeParams().overridden(**{"cradle.beam_wall_mm": 40.0})  # 2*40 > 60
    assert any("beam_wall" in i for i in eng.validate(p))


def test_validate_catches_tower_below_upper_pickups():
    """The damper top must rise above the upper pickups (a real shock tower)."""
    p = SubframeParams().overridden(**{"cradle.base_plane_z_mm": 700.0})  # above damper top
    issues = eng.validate(p)
    assert any("base_plane" in i for i in issues)


# --------------------------------------------------------------------------- #
# blueprint -- structural integrity
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(SubframeParams())
    assert blue["schema"] == "subframe_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "X"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 20


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(SubframeParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_both_sides_present():
    ids = [s["id"] for s in bp.generate(SubframeParams())["build_steps"]]
    assert any(i.endswith("_l") for i in ids) and any(i.endswith("_r") for i in ids)


def test_all_required_components_present():
    roles = {s["role"] for s in bp.generate(SubframeParams())["build_steps"]}
    assert "cradle_rail" in roles            # perimeter cradle
    assert "pad_post" in roles               # chassis pad interface
    assert "pickup_boss" in roles            # suspension pickups
    assert "shock_tower" in roles            # damper-top support
    assert "eaxle_mount" in roles            # e-axle / diff mounts


def test_eaxle_mounts_can_be_disabled():
    blue = bp.generate(SubframeParams().overridden(**{"eaxle.enabled": False}))
    assert not any(s["role"] == "eaxle_mount" for s in blue["build_steps"])


# --------------------------------------------------------------------------- #
# ONE WELDED CRADLE -- no solid interpenetration by construction (the redesign)
# --------------------------------------------------------------------------- #
def test_cradle_is_one_united_body():
    """The whole cradle must be ONE welded body: exactly ONE create step (the spine), and
    every other structural member is a unite onto it / every hole a subtract from it. Two
    overlapping create bodies were the 22 NX interpenetrations the redesign removed, so a
    second create would be a regression. Built for both axles."""
    for axle in ("rear", "front"):
        blue = bp.generate(SubframeParams().overridden(axle=axle))
        creates = [s for s in blue["build_steps"] if s["boolean"] == "create"]
        assert len(creates) == 1, "%s cradle has %d create bodies (want 1 welded body)" % (
            axle, len(creates))
        assert creates[0]["id"] == bp.CRADLE
        # every unite/subtract targets the one cradle body
        for s in blue["build_steps"]:
            if s["boolean"] in ("unite", "subtract"):
                assert s["target"] == bp.CRADLE, "%s targets %s, not the one cradle body" % (
                    s["id"], s["target"])


# --- oriented-solid VOLUME overlap (NX-merge semantics), NX-free + numpy-free -------- #
# NX merges a unite ONLY when the tool shares real VOLUME with a body already connected to
# the single create body. Surface-point sampling both misses and over-reports (a thin slab
# crossing a thick beam has its sampled corners OUTSIDE the beam yet a real volume overlap),
# which is exactly how the first build left disconnected toe-pickup fragments. So these
# helpers test true VOLUMETRIC overlap by sampling each body's INTERIOR and asking whether a
# point lies inside the other oriented primitive (a box prism or a cylinder), both ways.
def _norm3(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _prim(step):
    """An oriented solid primitive (cylinder/box-prism) with `contains(pt)->bool` and
    `interior()-> list of interior points`, for a create/unite body."""
    kind = step["kind"]
    if kind in ("cylinder", "tube"):
        o = list(step["origin3"]); ax = _norm3(step["axis"])
        L = float(step["length"]); r = float(step["outer_radius"])
        helper = [0, 0, 1.0] if abs(ax[2]) < 0.9 else [1.0, 0, 0]
        u = _norm3(_cross(ax, helper)); v = _cross(ax, u)

        def contains(p):
            rel = [p[i] - o[i] for i in range(3)]
            t = sum(rel[i] * ax[i] for i in range(3))
            if t < 0 or t > L:
                return False
            rad = [rel[i] - t * ax[i] for i in range(3)]
            return math.sqrt(sum(c * c for c in rad)) <= r

        def interior():
            pts = []
            # include stations NEAR BOTH ENDS (0.02, 0.98): perimeter members overlap at
            # the corners, i.e. at a member's extreme ends -- sampling only the middle
            # would miss a real corner merge (a sampling artefact, not a real disconnect).
            for tt in (0.02, 0.1, 0.3, 0.5, 0.7, 0.9, 0.98):
                c = [o[i] + tt * L * ax[i] for i in range(3)]
                pts.append(list(c))
                for rr in (0.4 * r, 0.8 * r):
                    for k in range(8):
                        a = 2 * math.pi * k / 8
                        d = [math.cos(a) * u[i] + math.sin(a) * v[i] for i in range(3)]
                        pts.append([c[i] + rr * d[i] for i in range(3)])
            return pts
        return contains, interior
    if kind == "prism":
        from motor_nx.blueprint import prism_frame
        o = list(step["origin3"])
        u, v, w = prism_frame(tuple(step["axis"]), tuple(step.get("u_dir", (1, 0, 0))))
        L = float(step["length"])
        poly = [(float(a), float(b)) for a, b in step["profile"]]

        def _in_poly(x, y):
            inside = False
            n = len(poly); j = n - 1
            for i in range(n):
                xi, yi = poly[i]; xj, yj = poly[j]
                if (yi > y) != (yj > y):
                    xint = (xj - xi) * (y - yi) / (yj - yi) + xi
                    if x < xint:
                        inside = not inside
                j = i
            return inside

        def contains(p):
            rel = [p[i] - o[i] for i in range(3)]
            t = sum(rel[i] * w[i] for i in range(3))
            if t < 0 or t > L:
                return False
            return _in_poly(sum(rel[i] * u[i] for i in range(3)),
                            sum(rel[i] * v[i] for i in range(3)))

        def interior():
            us = [q[0] for q in poly]; vs = [q[1] for q in poly]
            umin, umax, vmin, vmax = min(us), max(us), min(vs), max(vs)
            pts = []
            # stations near both ends (0.02, 0.98) too -- a beam overlaps its neighbour at
            # the corner (its extreme end), so middle-only sampling would miss a real merge.
            fr = (0.1, 0.25, 0.4, 0.55, 0.7, 0.85)
            for tt in (0.02, 0.15, 0.35, 0.5, 0.65, 0.85, 0.98):
                base = [o[i] + tt * L * w[i] for i in range(3)]
                for a in (umin + (umax - umin) * f for f in fr):
                    for b in (vmin + (vmax - vmin) * f for f in fr):
                        if _in_poly(a, b):
                            pts.append([base[i] + a * u[i] + b * v[i] for i in range(3)])
            return pts
        return contains, interior
    return None


def _vol_overlap(pa, pb):
    """True iff the two oriented solids share VOLUME (an interior point of one lies inside
    the other -- tested both ways so a thin body crossing a thick one is caught)."""
    ca, ia = pa; cb, ib = pb
    if any(cb(p) for p in ia()):
        return True
    if any(ca(p) for p in ib()):
        return True
    return False


def test_cradle_is_one_connected_component_in_nx():
    """UNION-FIND connectivity that mimics the REAL NX merge: NX keeps a unite as a SEPARATE
    body unless the tool shares VOLUME with a body ALREADY connected to the single create
    body (in build order). Replaying that, EVERY unite body must end up in the create's
    component -> NX yields exactly ONE solid (the headline acceptance: one body, built and
    inspected in NX). The first build failed this on the toe-pickup ear fragments (NX
    returned 3 bodies, 2 interpenetrating); this locks the fix in. Both axles."""
    for axle in ("rear", "front"):
        blue = bp.generate(SubframeParams().overridden(axle=axle))
        nodes = []          # [id, prim, connected?]
        for s in blue["build_steps"]:
            if s["boolean"] in ("create", "unite"):
                pr = _prim(s)
                if pr is None:
                    continue
                nodes.append([s["id"], pr, s["boolean"] == "create"])
        assert nodes and nodes[0][2], "%s: first weld body must be the create" % axle
        # replay in build order: a unite body joins the create's component iff it
        # volumetrically overlaps a body ALREADY connected (earlier in the order).
        for i in range(len(nodes)):
            if nodes[i][2]:
                continue
            for j in range(i):
                if nodes[j][2] and _vol_overlap(nodes[i][1], nodes[j][1]):
                    nodes[i][2] = True
                    break
        disconnected = [n[0] for n in nodes if not n[2]]
        assert not disconnected, (
            "%s cradle is NOT one connected solid -- NX would leave %d separate body(ies): "
            "%s. Extend the ear/bracket so it overlaps a CONNECTED perimeter body when "
            "united." % (axle, len(disconnected), disconnected[:6]))


# --------------------------------------------------------------------------- #
# ICD §7.4.2 mating ties -- read straight off the built geometry (world coords)
# --------------------------------------------------------------------------- #
def _create(blue, sid):
    for s in blue["build_steps"]:
        if s["id"] == sid and s["boolean"] == "create":
            return s
    raise AssertionError("no create body %s" % sid)


def _step(blue, sid):
    """Find a build step by id regardless of its boolean op (united pad flanges /
    tower seats are not create steps but still carry their world placement)."""
    for s in blue["build_steps"]:
        if s["id"] == sid:
            return s
    raise AssertionError("no build step %s" % sid)


def _cyl_centre(step):
    """Local centre of an axis-placed cylinder = origin3 + unit(axis) * length/2. Handles
    both a unit axis (length is the real length) and a stored full-delta axis (length =
    |axis|, as the pin bores use) by normalising the axis first."""
    o, a, L = step["origin3"], step["axis"], step["length"]
    n = math.sqrt(sum(c * c for c in a)) or 1.0
    return tuple(o[i] + (a[i] / n) * L / 2.0 for i in range(3))


def test_pickup_bosses_land_on_the_hardpoints():
    """Every pickup PIN BORE centre must sit on its suspension inboard hardpoint
    (ICD §7.4.2: the suspension arm inboard end bolts here -- no floating arm).

    The pickup is now a CLEVIS -- two ear plates straddling the suspension bushing eye,
    with ONE +X pin bore through both ears centred on the hardpoint (the eye sits in the
    clear gap between the ears). The pin-BORE centre is the preserved contract (it stays
    EXACTLY on the hardpoint, so check_corners.py / subframe_point_world are unaffected);
    the bore is the +X cylinder ``pickup_boss_<nm>_<side>_bore``, whose mid-point is the
    hardpoint. (The clevis ears are symmetric about the hardpoint, so the bore centre is
    the joint centre.)"""
    p = SubframeParams()
    blue = bp.generate(p)
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        for nm in ("lower_pickup_fore", "lower_pickup_aft", "upper_pickup_fore",
                   "upper_pickup_aft", "toe_pickup"):
            c = _cyl_centre(_step(blue, "pickup_boss_%s_%s_bore" % (nm, side)))
            assert c == pytest.approx(hp[nm], abs=1e-6), "boss %s_%s off hardpoint" % (nm, side)


def test_tower_seat_supports_the_damper_top():
    """The shock-tower seat must support the suspension damper/strut top (no floating
    spring, ICD §7.2). The seat lower face sits a small STANDOFF directly ABOVE the
    damper-top hardpoint (in X/Y on the hardpoint, in Z just clear of the suspension's own
    damper top-mount cap so the top mount bolts UP into it as a face touch -- not a buried
    overlap). The reported tower MATING point (subframe_point_world 'tower'/'damper_top')
    reads the hardpoint, so this standoff does not move the mating datum."""
    p = SubframeParams()
    blue = bp.generate(p)
    for side in ("l", "r"):
        seat = _step(blue, "tower_seat_%s" % side)
        hp = p.hardpoints_local(side)["damper_top"]
        # on the hardpoint in X/Y; a small standoff above it in Z
        assert seat["origin3"][0] == pytest.approx(hp[0], abs=1e-6)
        assert seat["origin3"][1] == pytest.approx(hp[1], abs=1e-6)
        assert seat["origin3"][2] == pytest.approx(hp[2] + p.tower.seat_standoff_mm, abs=1e-6)
        assert seat["origin3"][2] > hp[2]      # the seat is ABOVE the damper top (caps it)


def test_pad_flanges_seat_under_the_chassis_rail_bottom():
    """ICD §7.4.2: the four chassis-pad flanges sit at the rail centre-line y=±585 and
    seat UP against the rail UNDERSIDE (the subframe's pad_z = chassis rail bottom = 270),
    a small assembly gap below it. The cradle HANGS BELOW the rails so the Ø56 riser posts
    never pierce the rail box (bolting to the rail top would drive them straight through)."""
    p = SubframeParams()
    blue = bp.generate(p)
    flange_t = p.pad.flange_thickness_mm
    for fa in ("fore", "aft"):
        for side in ("l", "r"):
            flange = _step(blue, "pad_flange_%s_%s" % (fa, side))
            px, py, pz = p.pad_centre_local(fa, side)
            ox, oy, oz = flange["origin3"]
            assert ox == pytest.approx(px, abs=1e-6)
            assert oy == pytest.approx(py, abs=1e-6)
            assert abs(oy) == pytest.approx(585.0)
            # the flange TOP (the seating face) sits just BELOW the rail-bottom mating plane
            flange_top = oz + flange_t
            assert flange_top < pz                       # hangs under the rail bottom
            assert flange_top == pytest.approx(pz - 1.0, abs=1e-6)   # the 1 mm assembly gap
            assert pz == pytest.approx(270.0)            # chassis rail bottom


def test_cradle_perimeter_brackets_the_pad_y_via_world_bbox():
    """The perimeter cradle (side rails + crossbeams) must span the pad lateral width:
    its world bounding box in Y must reach out to ±pad_y. Computed from the prism
    sections via the NX-free profile_to_world twin (a real world-bbox check)."""
    p = SubframeParams()
    blue = bp.generate(p)
    ys = []
    for s in blue["build_steps"]:
        if s["role"] in ("cradle_rail", "cradle_cross") and s["boolean"] == "create":
            near = profile_to_world([tuple(q) for q in s["profile"]], tuple(s["origin3"]),
                                    tuple(s["axis"]), tuple(s["u_dir"]))
            a, L = s["axis"], s["length"]
            far = [(x + a[0] * L, y + a[1] * L, z + a[2] * L) for (x, y, z) in near]
            ys += [q[1] for q in near + far]
    assert max(ys) >= p.pad.pad_y_mm - 1.0
    assert min(ys) <= -(p.pad.pad_y_mm - 1.0)


def test_chassis_pad_alignment_against_chassis_package():
    """Cross-check: the subframe's FOUR pad flanges must land EXACTLY on the chassis
    subframe mount pads (vehicle frame), as a point set per axle -- the chassis now builds
    a coinciding pad (fore + aft, l + r) at axle ± pad_reach, not one boss at the axle."""
    from chassis_nx.blueprint import subframe_pad_centre
    from chassis_nx.params import ChassisParams
    cp = ChassisParams()
    for axle in ("rear", "front"):
        p = SubframeParams().overridden(axle=axle)
        ax = cp.frame.wheelbase_mm / 2.0 if axle == "front" else -cp.frame.wheelbase_mm / 2.0
        sub = {tuple(round(c, 3) for c in (ax + lx, ly, lz))
               for fa in ("fore", "aft") for sd in ("l", "r")
               for (lx, ly, lz) in [p.pad_centre_local(fa, sd)]}
        ch = {tuple(round(c, 3) for c in subframe_pad_centre(cp, axle, fa, sd))
              for fa in ("fore", "aft") for sd in ("l", "r")}
        assert ch == sub, (axle, sorted(ch), sorted(sub))


def test_pickup_bosses_match_suspension_inboard_hardpoints():
    """ICD §7.4.2 coincidence: mapping the suspension corner's inboard pickups (hub-
    datumed) into the subframe local frame must land on the subframe bosses (the
    headline 'no floating arm' tie)."""
    from suspension_nx.engineering import hardpoints as s_hardpoints
    from suspension_nx.params import SuspensionParams
    s_hp = s_hardpoints(SuspensionParams())
    p = SubframeParams()  # rear-left corner
    sub = p.hardpoints_local("l")
    # hub-frame -> subframe-local: rear-left HUB_CENTRE=(-1437.5,790,335), axle station
    # =(-1437.5,0,0) -> offset (0,790,335).
    offset = (0.0, 790.0, 335.0)
    for nm in ("lower_pickup_fore", "lower_pickup_aft", "upper_pickup_fore",
               "upper_pickup_aft", "toe_pickup"):
        sx, sy, sz = s_hp[nm]
        mapped = (sx + offset[0], sy + offset[1], sz + offset[2])
        d = sum((mapped[i] - sub[nm][i]) ** 2 for i in range(3)) ** 0.5
        assert d < 5.0, "boss %s is %.1f mm from the suspension hardpoint" % (nm, d)
