"""Pure-math corner-suspension geometry, emitted as the SAME ordered build-step
list the NX builder consumes (motor_nx.blueprint schema). NX-independent +
unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(prism / cylinder / tube / extrude + boolean create/subtract/unite), so motor_nx's
hardened NXOpen engine builds a suspension corner with no new geometry code.

Coordinate convention (LOCAL corner frame -- ICD-09 section 3)
    The LOCAL ORIGIN is the WHEEL-HUB CENTRE: the knuckle/upright hub bore is
    centred on local (0, 0, 0).
        +X = vehicle longitudinal (forward),
        +Y = OUTBOARD (toward the wheel; away from the chassis centreline),
        +Z = vertical (up).
    The wheel SPIN AXIS is the local Y axis (lateral); the hub bore is therefore a
    cylinder coaxial with +/-Y through the origin. Inboard control-arm chassis
    pickups sit at local -Y (toward the chassis centreline) at y ~ -arm_length.

    This re-datuming is REQUIRED so the assembly placing this part's origin at
    HUB_CENTRE(axle, side) = (axle_x, +/-T/2, r) puts the hub on the wheel with NO
    double-count of track/2 (vehicle_nx.assembly: left corner = identity, right
    corner = Rz(180), each at origin HUB_CENTRE). Build ONE canonical corner in
    this frame; the assembly mirrors it per side.

TRUE-3D MODELLING (real components, no flat +Z plates, nothing mid-air)
    Every link is modelled along its TRUE 3D axis with the general beam primitive
    kind="prism" (a 2D (u, v) section extruded along an arbitrary world axis at an
    arbitrary world origin) or an axis-placed cylinder/tube, and every link
    visibly CONNECTS its two hardpoints:

      * lower / upper control arm : a proper A-ARM -- a FORE and an AFT leg, each a
                       tapered prism that converges from its inboard chassis pickup
                       to a SINGLE outboard ball-joint hub boss just inboard of the
                       wheel.  The two legs share that ball-joint boss (so they read
                       as one A-arm, not two isolated parallel bars), and each leg is
                       wider/deeper at the loaded inboard end and necks down toward
                       the ball joint (real forged/cast taper).
      * toe / tie link : a slender round link (the steering tie-rod) from the toe
                       pickup to the steering-arm point on the knuckle, with a
                       threaded-rod look (a thin shank with eye ends).
      * coil-over    : the damper as a body cylinder + an exposed piston rod along
                       the damper_lower -> damper_top axis, with the COIL SPRING
                       modelled as N helical turns (a stack of coaxial torus-rings
                       swept around that same axis) seated between a lower perch on
                       the arm and an upper perch / top mount -- so the spring is
                       visibly COAXIAL around the damper, not a bare tube.  MacPherson
                       routes the coil-over coaxially through the upright top.
      * knuckle / upright : a CAST upright -- a hub barrel around the bore tying into
                       an upright web that grows arms to the lower & upper ball
                       joints, a caliper-mount bridge, and a steering arm -- united
                       into one cast body (then the hub bore is cut through it).
      * ball joints / bushings : ball joints as a tapered stud + ball at each
                       outboard hardpoint; compliance bushings as an outer can + an
                       inner sleeve (an eye) at each inboard pickup.
      * anti-roll drop link : a slender link with eye ends along its true near-
                       vertical axis from the lower arm up to the bar end.

    All hardpoint coordinates come from engineering.hardpoints(p), which is the one
    source of truth shared with the world-bounding-box / connectivity tests and the
    validate() reach checks.  This module ONLY makes the geometry follow that table
    convincingly; it never moves a hardpoint.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import SuspensionParams

Vec3 = Tuple[float, float, float]

# component colours (RGB 0-255)
COL_KNUCKLE = (95, 100, 110)
COL_ARM = (120, 124, 132)
COL_SPRING = (150, 90, 60)
COL_DAMPER = (70, 110, 160)
COL_ROD = (200, 202, 208)
COL_PERCH = (90, 92, 100)
COL_ARB = (140, 130, 90)
COL_MOUNT = (60, 62, 70)
COL_AIR = (0, 0, 0)

# helical-coil resolution: turns are rendered as a stack of coaxial torus-rings so
# the coil reads as a real spring wrapped around the damper (the engine has no helix
# sweep, so a real coil is approximated by its turns -- the same "representative
# blank" philosophy motor_nx uses for gear/end-winding blanks).
_COIL_TURNS = 7
_COIL_RING_SEG = 20          # facets per torus ring (revolve about the coil axis)


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


def _mirror_corner(pt: Vec3, track_mm: float) -> Vec3:
    """Reflect a hardpoint of the reference (+Y outboard) corner onto the OPPOSITE
    corner of the same axle for the in-package `corners="axle"` preview. The other
    hub sits at local (0, -track, 0) and its outboard direction is -Y, so we reflect
    Y about the mid-plane y = -track/2:  y' = -track - y  (X, Z unchanged)."""
    return (pt[0], -track_mm - pt[1], pt[2])


def _rect_uv(width: float, height: float) -> List[Tuple[float, float]]:
    """Closed (u, v) rectangle width x height centred on the local section origin."""
    hw, hh = width / 2.0, height / 2.0
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


# --------------------------------------------------------------------------- #
# beam / link primitives
# --------------------------------------------------------------------------- #
def _prism_link(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
                width: float, height: float, color, material: str = "arm_steel",
                u_dir: Vec3 = (1.0, 0.0, 0.0), boolean: str = "create",
                target: str = None) -> BuildStep:
    """A structural link as a rectangular-section PRISM along its TRUE 3D axis from
    p0 -> p1. The section (width x height in local u,v) is extruded along the unit
    axis (p1 - p0) by the link length, with the section plane placed at p0. `u_dir`
    sets the local +u in world (default +X = longitudinal); +v = axis x u. This is
    the general beam: an arm leg, an inclined toe link, a perch web, anything."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="prism", boolean=boolean,
        body_name=body_name, material=material, color=color,
        profile=_rect_uv(width, height), origin3=p0, axis=axis, u_dir=u_dir,
        length=length, target=target)


def _tapered_leg(steps: List[BuildStep], step_id: str, role: str, body_name: str,
                 p_in: Vec3, p_out: Vec3, w_in: float, w_out: float,
                 h_in: float, h_out: float, color, material: str,
                 u_dir: Vec3, target: str) -> None:
    """An A-arm LEG that tapers from a fat loaded inboard end (w_in x h_in at the
    chassis pickup p_in) to a slim outboard end (w_out x h_out at the ball joint
    p_out). The engine has no lofted prism, so the taper is built from a short
    stack of constant-section prisms whose section shrinks toward the ball joint --
    a faceted taper that reads as a forged/cast arm, not a slab.  All segments unite
    into `target` (the arm's first create), so the leg is one solid."""
    n = 3
    for i in range(n):
        t0 = i / float(n)
        t1 = (i + 1) / float(n)
        a = _lerp(p_in, p_out, t0)
        b = _lerp(p_in, p_out, t1)
        tm = (t0 + t1) / 2.0
        w = w_in + (w_out - w_in) * tm
        h = h_in + (h_out - h_in) * tm
        boolean = "create" if (target is None and i == 0) else "unite"
        tgt = None if boolean == "create" else target
        steps.append(_prism_link(
            "%s_s%d" % (step_id, i), role, body_name, a, b, w, h, color,
            material=material, u_dir=u_dir, boolean=boolean, target=tgt))


def _axis_cyl(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
              diameter: float, color, material: str = "joint_steel",
              boolean: str = "create", target: str = None) -> BuildStep:
    """A solid cylinder coaxial with the TRUE 3D axis p0 -> p1 (a damper body, a
    piston rod, a ball-joint stud, an anti-roll drop link). Built from the base
    point p0 along the unit axis by the segment length."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=length,
        target=target)


def _axis_tube(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
               outer_d: float, inner_d: float, color, material: str,
               boolean: str = "create", target: str = None) -> BuildStep:
    """A hollow tube coaxial with the TRUE 3D axis p0 -> p1 (a spring perch ring, a
    bushing can, a damper sleeve)."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="tube", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=outer_d / 2.0, inner_radius=max(1.0, inner_d / 2.0),
        origin3=p0, axis=axis, length=length, target=target)


def _coil_turns(steps: List[BuildStep], step_id: str, body_name: str,
                seat: Vec3, top: Vec3, coil_mean_d: float, wire_d: float,
                color, material: str, n_turns: int = _COIL_TURNS) -> None:
    """The COIL SPRING as `n_turns` coaxial torus-rings stacked along the seat->top
    axis at the coil mean diameter.  Each ring is a tube (a thin ring of axial
    thickness ~ wire_d) coaxial with the spring axis, so together they read as a
    real helical coil WRAPPED AROUND the damper (the engine has no helix sweep; a
    coil rendered as its turns is the established representative-blank approach).
    The ring inner diameter (coil_mean_d - wire_d) clears the damper body, so the
    coil is visibly concentric around the rod -- not a solid tube."""
    axis = _sub(top, seat)
    L = _norm(axis)
    w = _unit(axis)
    ring_outer = coil_mean_d + wire_d
    ring_inner = coil_mean_d - wire_d
    # distribute n rings so the first sits ON the seat and the last just below the top
    span = max(1e-6, L - wire_d)
    for i in range(n_turns):
        t = (i + 0.5) / float(n_turns)
        c = _add(seat, _scale(w, wire_d / 2.0 + t * span))
        a = _add(c, _scale(w, -wire_d / 2.0))
        b = _add(c, _scale(w, +wire_d / 2.0))
        steps.append(_axis_tube(
            "%s_turn%d" % (step_id, i), "spring", body_name, a, b,
            ring_outer, ring_inner, color, material))


# --------------------------------------------------------------------------- #
# cast upright / knuckle
# --------------------------------------------------------------------------- #
def _knuckle_steps(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str,
                   P, M, hp: Dict[str, Vec3]) -> str:
    """A CAST UPRIGHT tying the hub bore, the lower & upper ball joints, the caliper
    mount and the steering arm into one body.  Built (all in the local frame) as:
      1) a hub BARREL -- a thick tube around the wheel-spin (Y) axis through the
         origin (this is the bearing housing the Gen-3 hub presses into);
      2) an upright WEB -- a vertical prism slab spanning the lower->upper ball
         joints, united to the barrel;
      3) BALL-JOINT ARMS -- a tapered leg from the barrel out to each ball joint,
         united (so the joints are carried by the casting, not floating);
      4) a CALIPER BRIDGE -- a boss united fore of the hub;
      5) a STEERING ARM -- a stub united to the toe-outboard point.
    Finally the hub bore is cut along the Y axis through the whole casting.  Returns
    the knuckle body id (the create step) so callers can target it."""
    k = p.knuckle
    kid = "knuckle_%s" % tag
    lbj = P("lower_ball_joint")
    ubj = P("upper_ball_joint")
    z_lo = lbj[2] - 10.0
    z_hi = (ubj[2] if p.geometry.type in ("multilink", "double_wishbone")
            else P("strut_top")[2]) + 10.0
    barrel_half = k.width_mm / 2.0 + 6.0           # barrel extends each side of hub
    barrel_od = k.hub_bore_diameter_mm + 2.0 * k.thickness_mm  # cast wall around bore

    # 1) hub barrel: a solid cylinder around the wheel-spin (Y) axis (the bore is cut
    #    last, through everything). This is the CREATE that everything unites into.
    steps.append(BuildStep(
        id=kid, role="knuckle", kind="cylinder", boolean="create",
        body_name="Knuckle_Upright_%s" % U, material="cast_al", color=COL_KNUCKLE,
        outer_radius=barrel_od / 2.0,
        origin3=M((0.0, -barrel_half, 0.0)), axis=(0.0, 1.0, 0.0),
        length=2.0 * barrel_half))

    # 2) upright web: a vertical slab from the lower to the upper joint, on the
    #    inboard face of the barrel (so the wheel/disc clears it). United to barrel.
    web_lo = M((0.0, -(k.width_mm / 2.0 + 2.0), z_lo))
    web_hi = M((0.0, -(k.width_mm / 2.0 + 2.0), z_hi))
    steps.append(_prism_link(
        "knuckle_web_%s" % tag, "knuckle", "Knuckle_Web_%s" % U, web_lo, web_hi,
        k.thickness_mm * 1.6, k.thickness_mm, COL_KNUCKLE, material="cast_al",
        u_dir=(1.0, 0.0, 0.0), boolean="unite", target=kid))

    # 3) ball-joint arms: a tapered cast arm from the barrel out to each ball joint.
    barrel_face_lo = M((0.0, -barrel_half * 0.5, max(z_lo, lbj[2])))
    barrel_face_hi = M((0.0, -barrel_half * 0.5, min(z_hi, ubj[2])))
    arm_w = k.thickness_mm * 1.5
    _tapered_leg(steps, "knuckle_lbj_arm_%s" % tag, "knuckle",
                 "Knuckle_LBJ_Arm_%s" % U, barrel_face_lo, lbj,
                 arm_w, arm_w * 0.7, k.thickness_mm, k.thickness_mm * 0.8,
                 COL_KNUCKLE, "cast_al", u_dir=(1.0, 0.0, 0.0), target=kid)
    if p.geometry.type in ("multilink", "double_wishbone"):
        _tapered_leg(steps, "knuckle_ubj_arm_%s" % tag, "knuckle",
                     "Knuckle_UBJ_Arm_%s" % U, barrel_face_hi, ubj,
                     arm_w, arm_w * 0.7, k.thickness_mm, k.thickness_mm * 0.8,
                     COL_KNUCKLE, "cast_al", u_dir=(1.0, 0.0, 0.0), target=kid)

    # 4) caliper mount: a boss bridge united fore (+X) of the hub.
    if k.brake_caliper_mount:
        cm = P("caliper_mount")
        base = M((0.0, hp["caliper_mount"][1], hp["caliper_mount"][2]))
        steps.append(_axis_cyl(
            "caliper_mount_%s" % tag, "caliper_mount", "Caliper_Mount_%s" % U,
            base, cm, k.hub_bore_diameter_mm * 0.42, COL_KNUCKLE,
            material="cast_al", boolean="unite", target=kid))

    # 5) steering arm: a cast stub from the web out to the toe-outboard (tie-rod)
    #    point, so the tie-rod has something to pull on (the steering lever).
    toe_out = P("toe_outboard")
    steer_base = M((toe_out[0] * 0.3, -(k.width_mm / 2.0 + 2.0), toe_out[2]))
    steps.append(_prism_link(
        "knuckle_steer_arm_%s" % tag, "knuckle", "Steering_Arm_%s" % U,
        steer_base, toe_out, k.thickness_mm, k.thickness_mm * 0.9, COL_KNUCKLE,
        material="cast_al", u_dir=(0.0, 0.0, 1.0), boolean="unite", target=kid))

    # FINAL: cut the wheel-hub bearing bore along the lateral (Y) wheel-spin axis
    #        through the whole casting -- the Gen-3 hub presses into this.
    bore_half = barrel_half + 2.0
    steps.append(_axis_cyl(
        "knuckle_hub_bore_%s" % tag, "hub_bore_cut", "Hub_Bore_%s" % U,
        M((0.0, -bore_half, 0.0)), M((0.0, bore_half, 0.0)),
        k.hub_bore_diameter_mm, COL_AIR, material="air",
        boolean="subtract", target=kid))
    return kid


# --------------------------------------------------------------------------- #
# joints + bushings (real eyes, not bare stubs)
# --------------------------------------------------------------------------- #
def _bushing(steps: List[BuildStep], name: str, U: str, pt: Vec3, axis_along: Vec3,
             bush_d: float) -> None:
    """A compliance BUSHING at an inboard pickup: an outer steel can with a bonded
    inner sleeve (an eye), its bore axis along the link's longitudinal run so the
    arm leg plugs straight into it."""
    bush_l = max(16.0, bush_d * 0.7)
    ua = _unit(axis_along)
    p0 = _add(pt, _scale(ua, -0.5 * bush_l))
    p1 = _add(pt, _scale(ua, +0.5 * bush_l))
    # outer can
    steps.append(_axis_tube(name + "_can", "bushing", "Bushing_%s" % U.lower(),
                            p0, p1, bush_d, bush_d * 0.55, COL_MOUNT, "rubber"))
    # inner metal sleeve (the eye the bolt passes through)
    steps.append(_axis_tube(name + "_sleeve", "bushing", "Bushing_Sleeve_%s" % U.lower(),
                            p0, p1, bush_d * 0.5, bush_d * 0.28, COL_MOUNT, "joint_steel"))


def _balljoint(steps: List[BuildStep], name: str, U: str, pt: Vec3, stud_to: Vec3,
               bj_d: float) -> None:
    """A BALL JOINT at an outboard hardpoint: a near-spherical ball (a stubby tube-
    less cylinder approximating the ball) plus a tapered stud pointing toward the
    knuckle barrel (stud_to), so the joint reads as a real ball-and-socket, not a
    bare peg."""
    # the ball: a short cylinder centred on the joint, axis vertical (display)
    p0 = (pt[0], pt[1], pt[2] - bj_d / 2.0)
    p1 = (pt[0], pt[1], pt[2] + bj_d / 2.0)
    steps.append(_axis_cyl(name + "_ball", "ball_joint", "Ball_Joint_%s" % U.lower(),
                           p0, p1, bj_d, COL_MOUNT))
    # the tapered stud from the ball toward the knuckle
    stud_end = _add(pt, _scale(_unit(_sub(stud_to, pt)), 0.6 * bj_d))
    steps.append(_axis_cyl(name + "_stud", "ball_joint", "Ball_Stud_%s" % U.lower(),
                           pt, stud_end, bj_d * 0.55, COL_MOUNT))


# --------------------------------------------------------------------------- #
# one corner -- built from the engineering hardpoints (true 3D)
# --------------------------------------------------------------------------- #
def corner_steps(p: SuspensionParams, tag: str, mirror: bool) -> List[BuildStep]:
    """Build one corner from the shared hardpoint table.

    `mirror=False` builds the canonical reference corner (hub at local origin, +Y
    outboard). `mirror=True` builds the OPPOSITE corner of the same axle for the
    in-package `corners="axle"` preview (reflected onto the other hub at -track in
    Y). The vehicle assembler does NOT use mirror -- it places this canonical part
    via its own per-side transform (identity / Rz(180))."""
    g, s, d = p.geometry, p.spring, p.damper
    a, k, arm = p.antiroll, p.knuckle, p.arm
    U = tag.upper()
    hp = engineering.hardpoints(p)
    track = g.track_width_mm

    def M(pt: Vec3) -> Vec3:
        """Mirror a LITERAL world point onto the opposite corner (axle preview)."""
        return _mirror_corner(pt, track) if mirror else pt

    def P(name: str) -> Vec3:
        """A named hardpoint, mirrored for the opposite corner when requested."""
        return M(hp[name])

    steps: List[BuildStep] = []

    # --- KNUCKLE / UPRIGHT (cast, ties hub bore + joints + caliper + steer arm) -- #
    kid = _knuckle_steps(steps, p, tag, U, P, M, hp)

    # leg sections: a fat, loaded inboard end necking to a slim ball-joint end.
    w_in = max(26.0, arm.arm_diameter_mm + 16.0)    # inboard leg width (loaded)
    w_out = max(16.0, arm.arm_diameter_mm + 2.0)    # outboard (ball-joint) width
    h_in = max(16.0, arm.arm_diameter_mm + 4.0)     # inboard leg depth
    h_out = max(10.0, arm.arm_diameter_mm - 6.0)    # outboard depth

    # --- LOWER CONTROL ARM: a proper A-ARM (fore + aft legs -> one ball joint) --- #
    lbj = P("lower_ball_joint")
    lpf = P("lower_pickup_fore")
    lpa = P("lower_pickup_aft")
    # both legs converge on the SAME outboard ball-joint hub boss
    steps.append(_axis_cyl(
        "lower_arm_hub_%s" % tag, "lower_arm", "Lower_Arm_Hub_%s" % U,
        _add(lbj, (0.0, 0.0, -h_out)), _add(lbj, (0.0, 0.0, h_out)),
        w_out * 1.2, COL_ARM, material="arm_steel"))
    _tapered_leg(steps, "lower_arm_fore_%s" % tag, "lower_arm", "Lower_Arm_Fore_%s" % U,
                 lpf, lbj, w_in, w_out, h_in, h_out, COL_ARM, "arm_steel",
                 u_dir=(0.0, 0.0, 1.0), target="lower_arm_hub_%s" % tag)
    _tapered_leg(steps, "lower_arm_aft_%s" % tag, "lower_arm", "Lower_Arm_Aft_%s" % U,
                 lpa, lbj, w_in, w_out, h_in, h_out, COL_ARM, "arm_steel",
                 u_dir=(0.0, 0.0, 1.0), target="lower_arm_hub_%s" % tag)

    # --- UPPER CONTROL ARM (multilink / double_wishbone only): an A-ARM too ------ #
    if g.type in ("multilink", "double_wishbone"):
        ubj = P("upper_ball_joint")
        upf = P("upper_pickup_fore")
        upa = P("upper_pickup_aft")
        steps.append(_axis_cyl(
            "upper_arm_hub_%s" % tag, "upper_arm", "Upper_Arm_Hub_%s" % U,
            _add(ubj, (0.0, 0.0, -h_out)), _add(ubj, (0.0, 0.0, h_out)),
            w_out * 1.1, COL_ARM, material="arm_steel"))
        _tapered_leg(steps, "upper_arm_fore_%s" % tag, "upper_arm",
                     "Upper_Arm_Fore_%s" % U, upf, ubj, w_in * 0.85, w_out * 0.9,
                     h_in * 0.85, h_out, COL_ARM, "arm_steel",
                     u_dir=(0.0, 0.0, 1.0), target="upper_arm_hub_%s" % tag)
        _tapered_leg(steps, "upper_arm_aft_%s" % tag, "upper_arm",
                     "Upper_Arm_Aft_%s" % U, upa, ubj, w_in * 0.85, w_out * 0.9,
                     h_in * 0.85, h_out, COL_ARM, "arm_steel",
                     u_dir=(0.0, 0.0, 1.0), target="upper_arm_hub_%s" % tag)

    # --- TOE / TIE LINK: a slender steering tie-rod (shank + eye ends) ----------- #
    toe_in = P("toe_pickup")
    toe_out = P("toe_outboard")
    rod_d = max(12.0, arm.arm_diameter_mm * 0.5)
    steps.append(_axis_cyl(
        "toe_link_%s" % tag, "toe_link", "Toe_Link_%s" % U, toe_in, toe_out,
        rod_d, COL_ARM, material="arm_steel"))
    # eye ends (a small can at each end so it reads as a tie-rod with rod-ends)
    for end_name, pt, other in (("in", toe_in, toe_out), ("out", toe_out, toe_in)):
        ua = _sub(other, pt)
        eye0 = _add(pt, _scale(_unit(ua), -rod_d * 0.6))
        eye1 = _add(pt, _scale(_unit(ua), rod_d * 0.6))
        steps.append(_axis_tube(
            "toe_eye_%s_%s" % (end_name, tag), "toe_link", "Toe_Eye_%s_%s" % (end_name, U),
            eye0, eye1, rod_d * 1.7, rod_d * 0.7, COL_MOUNT, "joint_steel"))

    # --- INBOARD BUSHINGS + OUTBOARD BALL JOINTS (real eyes/balls, 3D placed) ---- #
    bj_d = arm.ball_joint_diameter_mm
    bush_d = arm.bushing_diameter_mm
    _bushing(steps, "lower_bushing_fore_%s" % tag, U, lpf, _sub(lbj, lpf), bush_d)
    _bushing(steps, "lower_bushing_aft_%s" % tag, U, lpa, _sub(lbj, lpa), bush_d)
    _balljoint(steps, "lower_balljoint_%s" % tag, U, lbj, P("hub_centre"), bj_d)
    if g.type in ("multilink", "double_wishbone"):
        ubj = P("upper_ball_joint")
        _bushing(steps, "upper_bushing_fore_%s" % tag, U, P("upper_pickup_fore"),
                 _sub(ubj, P("upper_pickup_fore")), bush_d)
        _bushing(steps, "upper_bushing_aft_%s" % tag, U, P("upper_pickup_aft"),
                 _sub(ubj, P("upper_pickup_aft")), bush_d)
        _balljoint(steps, "upper_balljoint_%s" % tag, U, ubj, P("hub_centre"), bj_d)

    # --- COIL-OVER: damper body + piston rod + coil spring + perches ------------- #
    _coilover_steps(steps, p, tag, U, P)

    # --- ANTI-ROLL (STABILISER) DROP LINK (slender, eye ends, true axis) --------- #
    if a.enabled:
        arb_lo = P("arb_link_lower")
        arb_hi = P("arb_link_upper")
        steps.append(_axis_cyl(
            "antiroll_%s" % tag, "anti_roll_bar", "Anti_Roll_Link_%s" % U,
            arb_lo, arb_hi, a.bar_diameter_mm * 0.7, COL_ARB, material="bar_steel"))
        for end_name, pt, other in (("lo", arb_lo, arb_hi), ("hi", arb_hi, arb_lo)):
            ua = _sub(other, pt)
            eye0 = _add(pt, _scale(_unit(ua), -a.bar_diameter_mm * 0.5))
            eye1 = _add(pt, _scale(_unit(ua), a.bar_diameter_mm * 0.5))
            steps.append(_axis_tube(
                "antiroll_eye_%s_%s" % (end_name, tag), "anti_roll_bar",
                "Anti_Roll_Eye_%s_%s" % (end_name, U), eye0, eye1,
                a.bar_diameter_mm * 1.4, a.bar_diameter_mm * 0.5, COL_MOUNT, "joint_steel"))
    return steps


def _coilover_steps(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str, P) -> None:
    """The COIL-OVER: a damper body (cylinder) with an exposed piston ROD reaching to
    the top mount, the COIL SPRING rendered as helical turns COAXIAL around that same
    axis, and a lower + upper spring PERCH seating the coil.  Seated on the lower arm
    (damper_lower) and reaching the body/tower top (damper_top, or strut_top for a
    MacPherson strut routed coaxially through the upright)."""
    s, d, g, k = p.spring, p.damper, p.geometry, p.knuckle
    coil_d = s.coil_outer_diameter_mm
    wire_d = max(8.0, coil_d * 0.10)               # representative spring-wire dia

    if g.type == "macpherson":
        seat = P("lower_ball_joint")
        top = P("strut_top")
    else:
        seat = P("damper_lower")
        top = P("damper_top")

    axis = _sub(top, seat)
    L = _norm(axis)
    w = _unit(axis)

    # damper BODY: the lower ~55 % of the run (the pressure tube the rod slides in).
    body_top = _add(seat, _scale(w, 0.55 * L))
    steps.append(_axis_cyl(
        "damper_%s" % tag, "damper",
        "Strut_Damper_%s" % U if g.type == "macpherson" else "Damper_%s" % U,
        seat, body_top, d.damper_diameter_mm, COL_DAMPER, material="damper_steel"))
    # piston ROD: a thinner shaft from the body top up to the top mount.
    steps.append(_axis_cyl(
        "damper_rod_%s" % tag, "damper", "Damper_Rod_%s" % U,
        body_top, top, d.damper_diameter_mm * 0.42, COL_ROD, material="rod_steel"))

    # lower + upper spring PERCH (the seats the coil reacts against).
    perch_lo0 = _add(seat, _scale(w, wire_d * 0.5))
    perch_lo1 = _add(seat, _scale(w, wire_d * 1.5))
    steps.append(_axis_tube(
        "spring_perch_lo_%s" % tag, "spring", "Spring_Perch_Lo_%s" % U,
        perch_lo0, perch_lo1, coil_d + 2.0 * wire_d + 6.0, d.damper_diameter_mm + 2.0,
        COL_PERCH, "perch_steel"))
    # the coil works over the lower ~80 % of the run (rod + top mount take the rest)
    spring_top = _add(seat, _scale(w, 0.80 * L))
    perch_hi0 = _add(spring_top, _scale(w, -wire_d * 0.5))
    perch_hi1 = _add(spring_top, _scale(w, wire_d * 0.5))
    steps.append(_axis_tube(
        "spring_perch_hi_%s" % tag, "spring", "Spring_Perch_Hi_%s" % U,
        perch_hi0, perch_hi1, coil_d + 2.0 * wire_d + 6.0, d.damper_diameter_mm * 0.5,
        COL_PERCH, "perch_steel"))

    # the COIL itself -- helical turns coaxial around the damper, seated perch->perch.
    _coil_turns(steps, "spring_%s" % tag, "Coil_Spring_%s" % U,
                perch_lo1, perch_hi0, coil_d, wire_d, COL_SPRING, "spring_steel")

    # TOP MOUNT: a body/tower mount cap at the top so the coil-over does not float.
    cap0 = _add(top, _scale(w, -wire_d))
    steps.append(_axis_tube(
        "damper_top_mount_%s" % tag, "damper", "Top_Mount_%s" % U, cap0, top,
        coil_d * 0.9, d.damper_diameter_mm * 0.42, COL_MOUNT, "mount_steel"))


def _corners(p: SuspensionParams):
    """Yield (tag, mirror) per modelled corner. `corners="axle"` adds the mirrored
    opposite corner for a standalone two-corner preview; the reference corner is
    always the canonical (un-mirrored) one whose hub is on the local origin."""
    if p.corners == "axle":
        return [("r", False), ("l", True)]
    return [("r", False)]


def build_steps(p: SuspensionParams) -> List[BuildStep]:
    steps: List[BuildStep] = []
    for tag, mirror in _corners(p):
        steps.extend(corner_steps(p, tag, mirror))
    return steps


def generate(p: SuspensionParams = None) -> Dict[str, Any]:
    """Full suspension-corner blueprint dict (NX-independent). Same schema as
    driveline / motor so the same run_journal builder consumes it."""
    if p is None:
        p = SuspensionParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "suspension_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": g.corner_envelope_height_mm,   # advisory envelope height
        "parameters": p.to_dict(),
        "expressions": [
            {"name": n, "value": v, "unit": u} for (n, v, u) in p.expressions()
        ],
        "derived": asdict(g),
        "hardpoints": {k: list(v) for k, v in engineering.hardpoints(p).items()},
        "validation": engineering.validate(p),
        "build_steps": [step.as_dict() for step in steps],
    }


def to_json(blueprint: Dict[str, Any], indent: int = 2) -> str:
    import json
    return json.dumps(blueprint, indent=indent)
