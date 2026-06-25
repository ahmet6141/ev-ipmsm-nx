"""Pure-math subframe (cradle) geometry, emitted as the SAME ordered build-step list
the NX builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(prism / cylinder / tube / hole + boolean create/subtract/unite), so motor_nx's
hardened NXOpen engine builds the subframe with no new geometry code.

Coordinate convention -- THE VEHICLE FRAME, datumed at the AXLE STATION (ICD §7.2)
    +X = forward, +Y = left, +Z = up (ISO 8855, ICD §1), so the assembler places the
    part with the IDENTITY transform per axle. BUT the local origin (0, 0, 0)
    represents the AXLE CENTRE ON THE GROUND -- vehicle point (axle_x, 0, 0). Every
    coordinate written here is therefore a VEHICLE coordinate MINUS the axle station:

        local = vehicle - (axle_x, 0, 0)        (axle_x = ±wheelbase/2)

    The assembler restores absolute vehicle coordinates by placing the part at
    origin = (axle_x, 0, 0). The hardpoint table (params.hardpoints_local) and the pad
    accessors are already in this local frame, so the geometry reads straight off
    params.hardpoints_local() / params.pad_centre_local().

ONE WELDED CRADLE -- NO SOLID INTERPENETRATION BY CONSTRUCTION (the redesign)
    A real subframe is ONE cast / welded weldment, not a pile of overlapping boxes. A
    first NX build of this part modelled the cradle as ~22 SEPARATE solids that
    overlapped wherever they met (the perimeter box beams crossing at the corners, and
    the pickup ears / shock towers / e-axle mounts buried in the beams they attach to):
    nx_inspect flagged 22 body interpenetrations.

    This module now builds the WHOLE cradle as ONE united body (NX body id ``cradle``):

      * a single CREATE body (the front crossbeam) is the weldment spine;
      * every other structural member -- the rear crossbeam, the two side rails, the
        corner GUSSETS, the inboard pickup STRINGERS, the chassis-pad RISER POSTS, the
        suspension PICKUP EARS, the SHOCK-TOWER turrets and the E-AXLE mount brackets --
        is a boolean="unite" onto that one body, emitted in an order where each tool
        OVERLAPS the running weldment when it is applied (so NX merges it instead of
        leaving a separate lump or throwing "tool body completely outside target");
      * EVERY hole -- the box-beam bores, the pad bolt circles, the pickup pin bores,
        the damper-rod bores + top-mount bolt circles, the e-axle carrier bores -- is a
        boolean="subtract" from that one body.

    Net result: ONE clean welded solid with zero internal interference (the bores are
    real voids the pins/bolts pass through, scoring ~0 in NX point-in-solid containment),
    plus the standalone diff/e-axle carrier nubs that only TOUCH it. Cleanliness is
    structural BY CONSTRUCTION: united members + subtracted bores, never two overlapping
    create bodies. The arbiter is verification/nx_inspect.py (real NX point-in-solid
    containment), not vehicle_nx/clearance.py (which is blind to subtracted voids).

NX-BUILD LESSONS honoured here (each, if violated, fails the real NX build)
    * Hollow box beams = an OUTER prism (create/unite) plus a slightly smaller CONCENTRIC
      inner prism (subtract) -- never a single annulus loop (NX rejects "self
      intersecting section").
    * Every boolean="unite"/"subtract" tool physically overlaps the running ``cradle``
      solid at its location (the spine reaches every node), so a unite merges and a bore
      lands inside the target (never "tool body completely outside target body").
    * Every prism profile is ONE simple rectangle (4 vertices).

VISUAL QUALITY -- it reads as a real automotive subframe
    a smooth perimeter cradle with GUSSETED corners, INTEGRATED pickup ears (tapered
    legs blended into the perimeter, not stuck-on lumps), proper shock-tower TURRETS with
    a capped top-mount seat, and e-axle mount BRACKETS -- sensible cross-sections and
    proportions (a production cast/welded cradle, not a heap of boxes).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import SubframeParams

Vec3 = Tuple[float, float, float]

# the single welded-cradle body id: the first CREATE makes it; every structural member
# UNITES into it and every hole SUBTRACTS from it, so NX leaves ONE clean solid.
CRADLE = "cradle"

# how far an ear/bracket leg is driven PAST each end (mm) so it overlaps real VOLUME of
# the perimeter beam at the root and of the boss/seat at the tip -- a connected NX unite,
# not a grazing face-touch (which NX leaves as a separate body).
_EAR_GRIP = 16.0

# component colours (RGB 0-255)
COL_CRADLE = (120, 128, 140)
COL_POST = (138, 142, 110)
COL_BOSS = (96, 140, 112)
COL_TOWER = (150, 118, 88)
COL_EAXLE = (110, 116, 128)
COL_AIR = (0, 0, 0)


# --------------------------------------------------------------------------- #
# small 3D helpers (pure math; the NX-free twins live in motor_nx.blueprint)
# --------------------------------------------------------------------------- #
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v: Vec3) -> Vec3:
    n = _norm(v) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


# --------------------------------------------------------------------------- #
# 2D section profiles (local u, v) -- centred on the prism origin
# --------------------------------------------------------------------------- #
def _rect_uv(half_u: float, half_v: float) -> List[Tuple[float, float]]:
    """Closed (u, v) rectangle centred on the origin, CCW (ONE simple 4-vertex loop)."""
    return [(-half_u, -half_v), (half_u, -half_v),
            (half_u, half_v), (-half_u, half_v)]


# --------------------------------------------------------------------------- #
# beam / member primitives -- all UNITE onto the one welded cradle body
# --------------------------------------------------------------------------- #
def _beam(steps: List[BuildStep], bid: str, role: str, body_name: str,
          origin3: Vec3, axis: Vec3, u_dir: Vec3, length: float,
          sec_u: float, sec_v: float, wall: float, color, boolean: str,
          target: str = None) -> None:
    """A hollow box beam: an outer rectangle prism (create/unite) extruded along ``axis``
    by ``length`` from ``origin3``, plus a slightly smaller CONCENTRIC inner rectangle
    prism (subtract) leaving a ``wall``-thick wall. The section is ``sec_u`` (along local
    +u) by ``sec_v`` (along local +v); the inner prism over-runs the ends by 1 mm so the
    bore is a guaranteed through-cut. ``boolean`` is "create" for the spine and "unite"
    for every other member, with ``target`` = the running cradle body.

    Two simple loops (outer + inner), never a single annulus -- so it builds in NX."""
    steps.append(BuildStep(
        id=bid, role=role, kind="prism", boolean=boolean, target=target,
        body_name=body_name, material="aluminium", color=color,
        profile=_rect_uv(sec_u / 2.0, sec_v / 2.0),
        origin3=origin3, axis=axis, u_dir=u_dir, length=length))
    iu, iv = sec_u - 2.0 * wall, sec_v - 2.0 * wall
    if iu > 0 and iv > 0:
        ax_n = _unit(axis)
        o_in = (origin3[0] - 0.5 * ax_n[0],
                origin3[1] - 0.5 * ax_n[1],
                origin3[2] - 0.5 * ax_n[2])
        # the bore SUBTRACTS from the running cradle body (the outer beam was just united
        # into it, so the bore lands inside the target -- never "tool outside target").
        steps.append(BuildStep(
            id="%s_hollow" % bid, role="%s_hollow_cut" % role, kind="prism",
            boolean="subtract", target=CRADLE, body_name="%s_Hollow" % body_name,
            material="air", color=COL_AIR,
            profile=_rect_uv(iu / 2.0, iv / 2.0),
            origin3=o_in, axis=axis, u_dir=u_dir, length=length + 1.0))


def _tapered_ear(steps: List[BuildStep], step_id: str, role: str, body_name: str,
                 p_root: Vec3, p_tip: Vec3, w_root: float, w_tip: float,
                 h_root: float, h_tip: float, color, u_dir: Vec3, target: str) -> None:
    """A cast EAR / bracket leg that tapers from a fat blended ROOT (w_root x h_root, deep
    in the perimeter) to a slim TIP (w_tip x h_tip, at the pickup/mount), built from a
    short stack of constant-section prisms whose section shrinks toward the tip. ALL
    segments UNITE into ``target`` (the running cradle), so the ear is integrated into the
    weldment as ONE connected member (not a stuck-on lump or a disconnected fragment).

    CONNECTIVITY BY CONSTRUCTION (the NX-merge lesson the inspector taught): NX merges a
    unite ONLY when the tool shares VOLUME with a body ALREADY connected to the single
    create body. A grazing face-touch is NOT enough -- it leaves a separate body. So:
      * the ear is EXTENDED past both ends -- it starts ``_EAR_GRIP`` BEFORE the root
        (driving the first segment deep INTO the perimeter beam it grows from) and runs
        ``_EAR_GRIP`` PAST the tip (so the boss/seat solidly overlaps the last segment);
      * consecutive segments OVERLAP by ~50 % of a segment length (each starts at the
        previous segment's midpoint), a real shared VOLUME, not a face touch;
      * segments are emitted ROOT -> TIP, so each unites into a target that already
        contains the previous (overlapping) segment -- the chain stays connected."""
    direction = _unit(_sub(p_tip, p_root))
    a0 = _sub(p_root, _scale(direction, _EAR_GRIP))      # start INSIDE the perimeter
    a1 = _add(p_tip, _scale(direction, _EAR_GRIP))       # end PAST the tip (into the boss)
    total = _norm(_sub(a1, a0))
    n = max(2, int(math.ceil(total / 36.0)))             # ~36 mm nominal segment length
    seg_len = total / n
    for i in range(n):
        # 50 %-overlapping segments: each starts half a segment back from where a simple
        # abutting stack would, so neighbours share real volume (a robust connected unite).
        t_start = max(0.0, (i - 0.5)) / n
        a = _lerp(a0, a1, t_start)
        # the section tapers with the param at the segment centre.
        tm = min(1.0, (i + 0.5) / n)
        w = w_root + (w_tip - w_root) * tm
        h = h_root + (h_tip - h_root) * tm
        # length runs from this (possibly backed-up) start to one full nominal segment
        # past the nominal end -> generous overlap with the next segment.
        L = seg_len * 1.6
        steps.append(BuildStep(
            id="%s_s%d" % (step_id, i), role=role, kind="prism", boolean="unite",
            target=target, body_name=body_name, material="aluminium", color=color,
            profile=_rect_uv(w / 2.0, h / 2.0), origin3=a, axis=direction, u_dir=u_dir,
            length=L))


def _united_cyl(steps: List[BuildStep], step_id: str, role: str, body_name: str,
                p0: Vec3, p1: Vec3, diameter: float, color, target: str,
                boolean: str = "unite") -> None:
    """A solid cylinder coaxial with the TRUE 3D axis p0 -> p1 (a riser post, a pickup
    boss, a tower post, an e-axle boss). UNITES into ``target`` (the running cradle) so it
    is one weld with it; the bore that fits the pin/bolt is SUBTRACTED separately."""
    axis = _sub(p1, p0)
    steps.append(BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean, target=target,
        body_name=body_name, material="aluminium", color=color,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=_norm(axis)))


def _bore(steps: List[BuildStep], step_id: str, role: str, body_name: str,
          p0: Vec3, p1: Vec3, diameter: float, target: str = CRADLE) -> None:
    """A cylindrical through-bore SUBTRACTED from ``target`` (a pin bore, a bolt-clearance
    hole, a damper-rod bore). The tool overlaps the boss/seat just united into the cradle,
    so it lands inside the target body."""
    axis = _sub(p1, p0)
    steps.append(BuildStep(
        id=step_id, role=role, kind="cylinder", boolean="subtract", target=target,
        body_name=body_name, material="air", color=COL_AIR,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=_norm(axis)))


# --------------------------------------------------------------------------- #
# perimeter cradle -- ONE welded loop of hollow box beams + gusseted corners
# --------------------------------------------------------------------------- #
def cradle_steps(p: SubframeParams) -> List[BuildStep]:
    """The perimeter weldment loop: a front + rear crossbeam tied by two side rails, with
    a triangular GUSSET filling each corner, plus the two inboard pickup stringers. The
    FRONT crossbeam is the single CREATE spine; everything else UNITES onto it (the loop
    closes corner-to-corner so each unite tool overlaps the running body)."""
    c, pad = p.cradle, p.pad
    steps: List[BuildStep] = []
    cz = c.base_plane_z_mm
    rail_cy = pad.pad_y_mm                       # side rails sit under the chassis pads
    half_len = c.side_rail_length_mm / 2.0
    bw, bh, wall = c.beam_width_mm, c.beam_height_mm, c.beam_wall_mm
    embed = 0.5 * bw                             # crossbeam ends embed into the side rails
    cross_span = 2.0 * rail_cy + 2.0 * embed

    # 1) SPINE = the FRONT (+X end) crossbeam, running laterally (+Y), section (u,v)->(Z,X)
    #    (u = +Z height, v = +X width). This is the ONE create the whole cradle unites into.
    _beam(steps, CRADLE, "cradle_cross", "Cradle_Cross_FRONT",
          origin3=(half_len, -(rail_cy + embed), cz),
          axis=(0.0, 1.0, 0.0), u_dir=(0.0, 0.0, 1.0), length=cross_span,
          sec_u=bh, sec_v=bw, wall=wall, color=COL_CRADLE, boolean="create")

    # 2) two SIDE RAILS running fore/aft (+X) under each chassis-pad Y. Their +X end
    #    overlaps the front crossbeam (which spans the full lateral width at X=+half_len),
    #    so this unite merges into the spine. Section (u,v)->(Y,Z).
    for tag, sign in (("l", +1.0), ("r", -1.0)):
        _beam(steps, "cradle_side_%s" % tag, "cradle_rail", "Cradle_Side_%s" % tag.upper(),
              origin3=(-half_len, sign * rail_cy, cz),
              axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0), length=c.side_rail_length_mm,
              sec_u=bw, sec_v=bh, wall=wall, color=COL_CRADLE, boolean="unite", target=CRADLE)

    # 3) REAR (-X end) crossbeam. Its ends overlap the two side rails (now part of the
    #    weldment) at the rear corners, so this unite stays connected.
    _beam(steps, "cradle_cross_rear", "cradle_cross", "Cradle_Cross_REAR",
          origin3=(-half_len, -(rail_cy + embed), cz),
          axis=(0.0, 1.0, 0.0), u_dir=(0.0, 0.0, 1.0), length=cross_span,
          sec_u=bh, sec_v=bw, wall=wall, color=COL_CRADLE, boolean="unite", target=CRADLE)

    # 4) corner GUSSETS: a triangular plate filling the INSIDE angle of each perimeter
    #    corner, blending the side rail smoothly into the crossbeam (a real cast/welded
    #    gusset web, not a crossing solid). Built as a vertical prism (axis +Z) whose (u, v)
    #    section is a right TRIANGLE in the X-Y plane spanning from the corner along the rail
    #    and along the crossbeam; it overlaps both members at the corner, so the unite merges
    #    it into the weldment. The plate is the beam height tall so it ties the full webs.
    g_in = bw * 1.8                              # reach in from the corner along each beam
    gusset_h = bh                                # gusset plate height (= beam height, +Z)
    for x_end, fa in ((half_len, +1.0), (-half_len, -1.0)):
        for sign in (+1.0, -1.0):
            # corner (rail/crossbeam intersection) in the X-Y plane; the triangle's right
            # angle sits at the inside corner and the hypotenuse cuts diagonally across.
            cx_corner, cy_corner = x_end, sign * rail_cy
            # local (u, v) = (X, Y); place the prism origin at the corner, section in X-Y.
            # triangle: corner -> inboard along the crossbeam (-fa*X) -> inboard along the
            # rail (-sign*Y). A simple 3-vertex loop, so it builds in NX.
            tri = [(0.0, 0.0), (-fa * g_in, 0.0), (0.0, -sign * g_in)]
            steps.append(BuildStep(
                id="cradle_gusset_%d_%d" % (int(fa), int(sign)), role="cradle_gusset",
                kind="prism", boolean="unite", target=CRADLE,
                body_name="Cradle_Gusset_%s%s" % (
                    "F" if fa > 0 else "R", "L" if sign > 0 else "R"),
                material="aluminium", color=COL_CRADLE,
                profile=tri,
                origin3=(cx_corner, cy_corner, cz - gusset_h / 2.0),
                axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0), length=gusset_h))

    # 5) INBOARD PICKUP STRINGERS: a fore/aft (+X) box beam per side at the pickup |Y| band,
    #    running the full side-rail length so its ends embed into the front + rear crossbeams
    #    (which span the full width). The suspension pickup ears blend onto THIS stringer, so
    #    the load path closes pickup -> ear -> stringer -> crossbeam -> side rail -> pad ->
    #    chassis. United onto the weldment (its ends overlap both crossbeams).
    for tag, sign in (("l", +1.0), ("r", -1.0)):
        _beam(steps, "cradle_stringer_%s" % tag, "cradle_stringer",
              "Cradle_Stringer_%s" % tag.upper(),
              origin3=(-half_len, sign * c.stringer_y_mm, cz),
              axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0), length=c.side_rail_length_mm,
              sec_u=c.stringer_width_mm, sec_v=bh, wall=wall, color=COL_CRADLE,
              boolean="unite", target=CRADLE)
    return steps


# --------------------------------------------------------------------------- #
# chassis-pad riser posts -- carry the cradle UP to the rail-top mount pads
# --------------------------------------------------------------------------- #
def pad_steps(p: SubframeParams) -> List[BuildStep]:
    """Four vertical riser posts that carry the weldment UP from its base plane to the
    chassis subframe mount pads (rail-top, y=±585, z≈390), each capped by a bolt-flange
    with a bolt circle. All UNITE onto / SUBTRACT from the one cradle body."""
    c, pad = p.cradle, p.pad
    steps: List[BuildStep] = []
    base_z = c.base_plane_z_mm

    for fore_aft in ("fore", "aft"):
        for side in ("l", "r"):
            px, py, pz = p.pad_centre_local(fore_aft, side)
            pid = "pad_post_%s_%s" % (fore_aft, side)
            # riser post: a +Z cylinder from the side-rail top up THROUGH the pad face. It
            # starts a little BELOW the base plane (overlaps the side-rail box -> united
            # cleanly) and runs a touch ABOVE the pad face so it overlaps the bolt-flange
            # disc that caps it (a solid overlap, not a fragile coincident-face unite).
            _united_cyl(steps, pid, "pad_post", "Pad_Post_%s_%s" % (fore_aft.upper(), side.upper()),
                        (px, py, base_z - c.beam_height_mm / 2.0),
                        (px, py, pz + pad.flange_thickness_mm * 0.5),
                        pad.post_diameter_mm, COL_POST, CRADLE)
            # bolt-flange disc on the pad face (the bolted interface to the chassis boss). Its
            # origin3 sits EXACTLY at the pad-centre (the chassis-mate contract the tests +
            # subframe_point_world read); the post above already overlaps it, so the unite
            # merges into the weldment.
            fid = "pad_flange_%s_%s" % (fore_aft, side)
            _united_cyl(steps, fid, "pad_flange", "Pad_Flange_%s_%s" % (fore_aft.upper(), side.upper()),
                        (px, py, pz), (px, py, pz + pad.flange_thickness_mm),
                        pad.flange_diameter_mm, COL_POST, CRADLE)
            # bolt circle drilled UP (+Z) through the flange (clearance for the chassis bolts)
            if pad.bolt_count > 0 and pad.bolt_diameter_mm > 0:
                pcd_r = max(pad.bolt_diameter_mm,
                            pad.flange_diameter_mm / 2.0 - max(pad.bolt_diameter_mm, 6.0))
                for k in range(pad.bolt_count):
                    ang = 2.0 * math.pi * k / pad.bolt_count
                    hx = px + pcd_r * math.cos(ang)
                    hy = py + pcd_r * math.sin(ang)
                    steps.append(BuildStep(
                        id="pad_bolt_%s_%s_%d" % (fore_aft, side, k),
                        role="pad_bolt_cut", kind="hole", boolean="subtract", target=CRADLE,
                        body_name="Pad_Bolt_%s_%s_%d" % (fore_aft.upper(), side.upper(), k),
                        material="air", color=COL_AIR,
                        outer_radius=pad.bolt_diameter_mm / 2.0,
                        cx=hx, cy=hy, z0=pz - 0.5,
                        axis=(0.0, 0.0, 1.0), length=pad.flange_thickness_mm + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# suspension inboard pickup ears -- a tapered ear + a bored boss at each hardpoint
# --------------------------------------------------------------------------- #
# the inboard hardpoints that get a pickup boss (the upright/ball-joint hardpoints
# belong to the suspension, not the subframe).
_PICKUP_NAMES = ("lower_pickup_fore", "lower_pickup_aft",
                 "upper_pickup_fore", "upper_pickup_aft", "toe_pickup")


def boss_steps(p: SubframeParams) -> List[BuildStep]:
    """A suspension PICKUP EAR at every inboard hardpoint, both sides: a tapered cast ear
    blends UP from the inboard stringer to the hardpoint, ending in a cylindrical boss
    (axis fore/aft, +X) whose cross bore takes the control-arm / toe-link bushing pin. The
    ear + boss UNITE onto the one cradle body; the pin bore SUBTRACTS from it. The boss is
    an integrated ear, not a stuck-on lump.

    The boss placement (a +X cylinder centred on the hardpoint) is preserved exactly -- the
    pin-bore centre stays at the suspension hardpoint, so the four-corner coincidence
    (check_corners.py) and subframe_point_world(...,'pickup',...) are unaffected by the
    unification. The boss UNITES onto the weldment (it is not a separate create body), so no
    second solid overlaps it."""
    b = p.boss
    c = p.cradle
    steps: List[BuildStep] = []
    half = b.boss_length_mm / 2.0
    stringer_centre_z = c.base_plane_z_mm
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        for nm in _PICKUP_NAMES:
            x, y, z = hp[nm]
            bid = "pickup_boss_%s_%s" % (nm, side)
            # 1) tapered EAR from the inboard stringer up to the hardpoint. Root inside the
            #    stringer box (overlaps it -> connected unite); tip at the pickup. The ear's
            #    +Y position runs from the stringer band to the pickup |Y| so it blends in.
            root = (x, math.copysign(c.stringer_y_mm, y), stringer_centre_z - c.beam_height_mm * 0.25)
            tip = (x, y, z)
            if _norm(_sub(tip, root)) > 1.0:
                _tapered_ear(steps, "%s_ear" % bid, "pickup_ear",
                             "Pickup_Ear_%s_%s" % (nm.upper(), side.upper()),
                             root, tip, b.boss_diameter_mm * 1.15, b.boss_diameter_mm,
                             c.beam_height_mm, b.boss_diameter_mm * 0.9,
                             COL_BOSS, u_dir=(1.0, 0.0, 0.0), target=CRADLE)
            # 2) the bored boss, axis +X, centred on the hardpoint (start half-length back so
            #    its mid-point IS the hardpoint). UNITED into the weldment (the ear above
            #    reaches the boss, so the boss overlaps the cradle through the ear -> one
            #    connected solid). The cylinder placement (origin3/axis/length) IS the
            #    contract the tests + assembly accessors read for the pin-bore centre.
            steps.append(BuildStep(
                id=bid, role="pickup_boss", kind="cylinder", boolean="unite", target=CRADLE,
                body_name="Pickup_Boss_%s_%s" % (nm.upper(), side.upper()),
                material="aluminium", color=COL_BOSS,
                outer_radius=b.boss_diameter_mm / 2.0, length=b.boss_length_mm,
                origin3=(x - half, y, z), axis=(1.0, 0.0, 0.0)))
            # 3) cross bore through the boss along its axis (pin / bushing-bolt clearance),
            #    SUBTRACTED from the weldment.
            _bore(steps, "%s_bore" % bid, "pickup_bore_cut",
                  "Pickup_Bore_%s_%s" % (nm.upper(), side.upper()),
                  (x - half - 0.5, y, z), (x + half + 0.5, y, z), b.bore_diameter_mm)
    return steps


# --------------------------------------------------------------------------- #
# shock / body towers -- a turret reaching the damper/strut top, per side
# --------------------------------------------------------------------------- #
def tower_steps(p: SubframeParams) -> List[BuildStep]:
    """A SHOCK-TOWER turret per side: a vertical post rises from the inboard stringer to
    the damper/strut top, capped by a seat plate with a damper-rod bore + a top-mount bolt
    circle (no floating spring, ICD §7.2). The post + seat UNITE onto the one cradle body;
    the bore + bolt circle SUBTRACT from it.

    The seat CREATE/PLACE (origin3 at the damper-top hardpoint) is preserved exactly so the
    tower top stays at the suspension damper/strut top."""
    tw = p.tower
    c = p.cradle
    steps: List[BuildStep] = []
    base_z = c.base_plane_z_mm
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        tx, ty, tz = hp["damper_top"]
        # the tower foot sits on the inboard stringer (|Y| = stringer_y) and the turret leans
        # out to the damper top so it BLENDS into the perimeter rather than floating inboard.
        foot = (tx, math.copysign(c.stringer_y_mm, ty), base_z - c.beam_height_mm / 2.0)
        tid = "tower_post_%s" % side
        # 1) the turret POST: a leaning solid cylinder from the stringer up to just under the
        #    seat. United into the weldment (its foot overlaps the stringer box).
        seat_bottom = (tx, ty, tz)
        _united_cyl(steps, tid, "shock_tower", "Shock_Tower_%s" % side.upper(),
                    foot, seat_bottom, tw.post_diameter_mm, COL_TOWER, CRADLE)
        # 2) top SEAT plate capping the turret AT the damper-top hardpoint (its lower face at
        #    tz -> the top mount lands there). origin3 = the hardpoint (preserved contract).
        sid = "tower_seat_%s" % side
        steps.append(BuildStep(
            id=sid, role="tower_seat", kind="cylinder", boolean="unite", target=CRADLE,
            body_name="Tower_Seat_%s" % side.upper(),
            material="aluminium", color=COL_TOWER,
            outer_radius=tw.seat_diameter_mm / 2.0, length=tw.seat_thickness_mm,
            origin3=(tx, ty, tz), axis=(0.0, 0.0, 1.0)))
        # 3) damper-rod / top-mount clearance bore down through the seat + into the post
        steps.append(BuildStep(
            id="%s_bore" % sid, role="tower_bore_cut", kind="cylinder",
            boolean="subtract", target=CRADLE,
            body_name="Tower_Bore_%s" % side.upper(), material="air", color=COL_AIR,
            outer_radius=tw.rod_bore_diameter_mm / 2.0,
            length=tw.seat_thickness_mm + 30.0,
            origin3=(tx, ty, tz - 30.0), axis=(0.0, 0.0, 1.0)))
        # 4) top-mount bolt circle through the seat plate (fastens the damper top mount)
        if tw.bolt_count > 0 and tw.bolt_diameter_mm > 0:
            pcd_r = max(tw.bolt_diameter_mm,
                        tw.seat_diameter_mm / 2.0 - max(tw.bolt_diameter_mm, 6.0))
            for k in range(tw.bolt_count):
                ang = 2.0 * math.pi * k / tw.bolt_count
                hx = tx + pcd_r * math.cos(ang)
                hy = ty + pcd_r * math.sin(ang)
                steps.append(BuildStep(
                    id="%s_bolt_%d" % (sid, k), role="tower_bolt_cut", kind="hole",
                    boolean="subtract", target=CRADLE,
                    body_name="Tower_Bolt_%s_%d" % (side.upper(), k),
                    material="air", color=COL_AIR,
                    outer_radius=tw.bolt_diameter_mm / 2.0,
                    cx=hx, cy=hy, z0=tz + tw.seat_thickness_mm + 0.5,
                    axis=(0.0, 0.0, -1.0), length=tw.seat_thickness_mm + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# e-axle / diff mounts -- carry the gearbox + differential under the cradle
# --------------------------------------------------------------------------- #
def eaxle_steps(p: SubframeParams) -> List[BuildStep]:
    """E-AXLE / diff mount BRACKETS straddling the centre plane (±Y) at the carrier height.
    Each is a short bracket that drops from a perimeter crossbeam (which spans the full
    lateral width at the base plane, so it passes over |Y|=mount_y) down to a mount boss
    the diff/gearbox carrier bolts to. The bracket + boss UNITE onto the one cradle body;
    the carrier bore SUBTRACTS from it."""
    ea = p.eaxle
    if not ea.enabled:
        return []
    c = p.cradle
    steps: List[BuildStep] = []
    base_z = c.base_plane_z_mm
    half_len = c.side_rail_length_mm / 2.0
    n = max(1, ea.mount_count)
    for i in range(n):
        # spread the mounts symmetrically fore/aft within the cradle X span, near the
        # crossbeam ends so each bracket reaches a real crossbeam to hang from.
        if n == 1:
            mx = 0.0
            x_anchor = half_len                  # hang from the front crossbeam
        else:
            frac = (i + 0.5) / n
            mx = (-0.5 + frac) * (c.side_rail_length_mm * 0.6)
            # nearest crossbeam X (front if fore of centre, rear if aft)
            x_anchor = half_len if mx >= 0 else -half_len
        for side, sign in (("l", +1.0), ("r", -1.0)):
            my = sign * ea.mount_y_mm
            bid = "eaxle_mount_%d_%s" % (i, side)
            top = (mx, my, ea.mount_z_mm + ea.boss_height_mm)
            bot = (mx, my, ea.mount_z_mm)
            # 1) hanger BRACKET: a slim prism from the crossbeam (at x_anchor, |Y|=my, the
            #    base plane) across to the boss top, so the boss is carried by the perimeter.
            anchor = (x_anchor, my, base_z)
            if _norm(_sub(top, anchor)) > 1.0:
                _tapered_ear(steps, "%s_arm" % bid, "eaxle_arm",
                             "EAxle_Arm_%d_%s" % (i, side.upper()),
                             anchor, top, c.beam_height_mm * 0.8, ea.boss_diameter_mm * 0.8,
                             c.beam_width_mm * 0.8, ea.boss_diameter_mm * 0.8,
                             COL_EAXLE, u_dir=(0.0, 0.0, 1.0), target=CRADLE)
            # 2) a +Z mount boss the diff/gearbox bolts down onto. UNITED into the weldment
            #    (the bracket arm above reaches it, so it overlaps the cradle -> one solid).
            steps.append(BuildStep(
                id=bid, role="eaxle_mount", kind="cylinder", boolean="unite", target=CRADLE,
                body_name="EAxle_Mount_%d_%s" % (i, side.upper()),
                material="aluminium", color=COL_EAXLE,
                outer_radius=ea.boss_diameter_mm / 2.0, length=ea.boss_height_mm,
                origin3=bot, axis=(0.0, 0.0, 1.0)))
            # 3) carrier bolt bore down the boss axis, SUBTRACTED from the weldment.
            _bore(steps, "%s_bore" % bid, "eaxle_bolt_cut",
                  "EAxle_Bolt_%d_%s" % (i, side.upper()),
                  (mx, my, ea.mount_z_mm - 0.5), (mx, my, ea.mount_z_mm + ea.boss_height_mm + 0.5),
                  ea.bolt_diameter_mm)
    return steps


def build_steps(p: SubframeParams) -> List[BuildStep]:
    """The full ordered weldment build: the perimeter loop (the ONE create + its unites)
    first, then every member UNITES onto it and every hole SUBTRACTS from it -- ONE clean
    welded cradle body by construction."""
    steps = cradle_steps(p)
    steps.extend(pad_steps(p))
    steps.extend(boss_steps(p))
    steps.extend(tower_steps(p))
    steps.extend(eaxle_steps(p))
    return steps


def generate(p: SubframeParams = None) -> Dict[str, Any]:
    """Full subframe blueprint dict (NX-independent). Same schema as chassis_nx /
    driveline_nx / motor_nx so the same run_journal builder consumes it.

    `axis` is reported as "X" because the dominant perimeter side rails run along the
    vehicle +X; this is documentation only -- every step carries its own explicit
    origin3/axis, so the builder never relies on a global axis."""
    if p is None:
        p = SubframeParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "subframe_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "X",
        "stack_length": p.cradle.side_rail_length_mm,   # representative beam length
        "parameters": p.to_dict(),
        "expressions": [
            {"name": n, "value": v, "unit": u} for (n, v, u) in p.expressions()
        ],
        "derived": asdict(g),
        "validation": engineering.validate(p),
        "build_steps": [step.as_dict() for step in steps],
    }


def to_json(blueprint: Dict[str, Any], indent: int = 2) -> str:
    import json
    return json.dumps(blueprint, indent=indent)
