"""Pure-math corner-suspension geometry, emitted as the SAME ordered build-step
list the NX builder consumes (motor_nx.blueprint schema). NX-independent +
unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(prism / cylinder / tube + boolean create/subtract/unite), so motor_nx's hardened
NXOpen engine builds a suspension corner with no new geometry code.

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
    HUB_CENTRE(axle, side) puts the hub on the wheel with NO double-count of
    track/2. Build ONE canonical corner in this frame; the assembly mirrors it per
    side.

REAL, CLEAN JOINTS -- NO SOLID INTERPENETRATION BY CONSTRUCTION (the redesign)
    A first NX build had 32 body pairs that genuinely interpenetrated (arm hubs
    buried in the knuckle, the ball joint overlapping the arm AND the knuckle, eyes
    overlapping their own link, sleeves buried in bushings).  This module now models
    every joint so two distinct solids never share volume:

      B) ONE BODY PER MEMBER -- each control arm = its hub eye + fore leg + aft leg
         UNITED into one arm body; the toe link + its two eyes = one body; the
         anti-roll link + its two eyes = one body; the knuckle + caliper mount +
         steering arm = one cast body.  So a member never overlaps its own features.
      C) PIN JOINTS between two DIFFERENT members are real bolted clevis/lap joints:
         the two eyes are STACKED AXIALLY along the bolt axis (they do not share
         space), the eye bores are coaxial and the same diameter, and a dedicated
         BOLT (hex head + shank, suspension_nx.fasteners) fills the (clear) bores
         with a NUT on the far end.  A compliance BUSHING (outer can + inner sleeve,
         a touching press fit) sits in the inboard eyes.
      D) BALL JOINT bridges the ARM eye and the KNUCKLE socket, which are separated
         ALONG the kingpin axis: the ball HOUSING presses into the arm-eye bore
         (housing OD == eye bore ID) and the STUD seats in the knuckle socket bore
         (stud OD == socket bore).  So ARM<->KNUCKLE and BALLJOINT<->(arm/knuckle)
         no longer overlap as solids.
      E) BUSHING = concentric outer ring + inner sleeve (sleeve OD == ring bore ID,
         touching); the joint bolt runs through the sleeve.
      F) COIL-OVER: the coil seats ON the lower-perch face and UNDER the upper-perch
         face (touching, not buried); the rod is concentric inside the coil with a
         clear radial gap.  Each perch is united to its arm/mount carrier.
      G) every body carries a body_name (including the arm legs).

    HOW HOLLOWS ARE BUILT (so it BUILDS IN NX): every eye / bushing-can / sleeve /
    perch / hub-barrel / coil-turn is a kind="tube" (outer cylinder create + inner
    cylinder SUBTRACT, done by the NX builder) or -- when the bore must join a member
    -- a SOLID boss UNITE + a separate bore SUBTRACT (suspension_nx.fasteners.
    united_eye).  NEVER a single annulus profile: NX's Section rejects an outer+inner
    loop in one profile as "self intersecting".  A bolt/stud/rod is a SOLID cylinder
    sitting in the subtracted bore.  The no-interpenetration proof is therefore NOT
    vehicle_nx.clearance.py (it is blind to subtract voids and treats a tube as a solid
    disc -- false readings on bored parts); the arbiter is verification/nx_inspect.py
    (real NX point-in-solid containment, which sees the bores).  Cleanliness is
    structural BY CONSTRUCTION: united members; coaxial same-Ø bores; members separated
    along the joint axis and bridged only by a bolt/stud/sleeve in a subtracted bore.

    All hardpoint coordinates come from engineering.hardpoints(p) -- the single
    source of truth.  This module only makes the geometry follow that table cleanly;
    it never moves a hardpoint.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from . import fasteners as F
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

# helical-coil resolution: turns are rendered as a stack of coaxial tube-rings so the
# coil reads as a real spring wrapped around the damper (the engine has no helix sweep,
# so a real coil is approximated by its turns -- each a hollow tube around the rod).
_COIL_TURNS = 7

# kingpin-axis half-separation of the arm eye and the knuckle socket at a ball joint
# (each member's eye sits this far from the joint centre on OPPOSITE sides, so the ball
# joint bridges the gap and the two members never share volume).  Generous so the fat
# arm-eye and the knuckle web/socket bodies stay clear of each other.
_BJ_HALF_SEP = 24.0


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


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _nearest_on_seg(p: Vec3, a: Vec3, b: Vec3) -> Vec3:
    """The point on the segment a->b closest to p (the foot of the perpendicular,
    clamped to the segment).  Used to ROOT a bracket on a member's leg: rooting at this
    point guarantees the bracket's first segment overlaps the leg solid, so the unite
    connects (a disjoint root leaves the bracket a floating, unnamed lump)."""
    ab = _sub(b, a)
    denom = _dot(ab, ab) or 1.0
    t = max(0.0, min(1.0, _dot(_sub(p, a), ab) / denom))
    return _add(a, _scale(ab, t))


def rod_d_of(p: SuspensionParams) -> float:
    """The toe / tie-rod shank diameter (shared by the knuckle steering eye and the toe
    link so their clevis bores match)."""
    return max(12.0, p.arm.arm_diameter_mm * 0.5)


def _transverse(axis: Vec3) -> Vec3:
    """A unit vector perpendicular to ``axis`` -- a sensible transverse PIVOT axis for a
    chassis bushing (the joint pivots about an axis across the link, not along it)."""
    w = _unit(axis)
    helper = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
    return _unit((w[1] * helper[2] - w[2] * helper[1],
                  w[2] * helper[0] - w[0] * helper[2],
                  w[0] * helper[1] - w[1] * helper[0]))


def _mirror_corner(pt: Vec3, track_mm: float) -> Vec3:
    """Reflect a hardpoint of the reference (+Y outboard) corner onto the OPPOSITE
    corner of the same axle for the in-package `corners="axle"` preview."""
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
    axis (p1 - p0) by the link length, with the section plane placed at p0."""
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
    p_out), built from a short stack of constant-section prisms whose section shrinks
    toward the joint.  ALL segments UNITE into `target` (a body created earlier), so
    the leg -- and the whole arm -- is ONE solid.

    UNITE-ORDER (priority 2): the segments are emitted from the OUTBOARD end (p_out,
    which touches the arm hub eye = `target`) inboard to the pickup, so each segment
    unites into a target that ALREADY contains the previous (touching) segment -- the
    chain stays connected.  Emitting pickup-first would unite the far inboard segment
    into the disjoint eye, which NX leaves as a separate lump (the bug the inspector saw:
    arm legs not united).  The segments overlap their neighbours (and the eye) slightly
    so every unite is a real touching boolean."""
    n = 3
    seg = []
    for i in range(n):
        t0 = i / float(n)
        t1 = (i + 1) / float(n)
        a = _lerp(p_in, p_out, t0)
        b = _lerp(p_in, p_out, t1)
        tm = (t0 + t1) / 2.0
        w = w_in + (w_out - w_in) * tm
        h = h_in + (h_out - h_in) * tm
        seg.append((i, a, b, w, h))
    # emit OUTBOARD (eye-touching) segment first so the unite chain stays connected
    for i, a, b, w, h in reversed(seg):
        steps.append(_prism_link(
            "%s_s%d" % (step_id, i), role, body_name, a, b, w, h, color,
            material=material, u_dir=u_dir, boolean="unite", target=target))


def _axis_cyl(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
              diameter: float, color, material: str = "joint_steel",
              boolean: str = "create", target: str = None) -> BuildStep:
    """A solid cylinder coaxial with the TRUE 3D axis p0 -> p1 (a damper body, a
    piston rod)."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=length,
        target=target)


def _coil_turns(steps: List[BuildStep], step_id: str, body_name: str,
                seat: Vec3, top: Vec3, coil_mean_d: float, wire_d: float,
                color, material: str, n_turns: int = _COIL_TURNS) -> None:
    """The COIL SPRING as `n_turns` coaxial tube-ring turns (each a kind="tube":
    outer cylinder create + inner cylinder subtract -> builds in NX) stacked along the
    seat->top axis.  `coil_mean_d` is the spring OUTER diameter (the param name): the
    ring OD is that diameter exactly and the bore (OD - 2*wire) clears the damper rod,
    so the coil is visibly concentric AROUND the rod with a real gap (the NX inspector
    sees the bore, so it never reads as overlapping the rod).  The first ring sits ON
    the seat face and the last just below the top face (touching the perches)."""
    axis = _sub(top, seat)
    L = _norm(axis)
    w = _unit(axis)
    ring_outer = coil_mean_d                         # spring OUTER diameter (true OD)
    ring_bore = coil_mean_d - 2.0 * wire_d
    # pitch the turns so each ring is SHORTER than the inter-turn spacing -> the turns
    # never overlap each other (the inspector flagged overlapping turn-segments).  Each
    # turn's axial thickness is capped at ~70 % of the pitch, leaving a clear gap.
    pitch = L / float(n_turns)
    turn_len = min(wire_d, 0.7 * pitch)
    for i in range(n_turns):
        c = _add(seat, _scale(w, (i + 0.5) * pitch))
        # each turn gets a UNIQUE body_name suffix (Coil_Spring_R_000..006) so the
        # vehicle assembler -- which names bodies straight from body_name -- gives every
        # turn a distinct name instead of 7x the same "COIL_SPRING_R" (the inspector's
        # duplicate-name info).  The standalone builder already uniquifies via its
        # per-role counter; this aligns the assembler path with it.
        steps.append(F.ring_body(
            "%s_turn%d" % (step_id, i), "spring", "%s_%03d" % (body_name, i), c, w,
            ring_outer, ring_bore, turn_len, color, material))


# --------------------------------------------------------------------------- #
# cast upright / knuckle
# --------------------------------------------------------------------------- #
def _knuckle_steps(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str,
                   P, M, hp: Dict[str, Vec3]) -> Tuple[str, Dict[str, Vec3]]:
    """A CAST UPRIGHT tying the hub bore, the lower & upper ball-joint SOCKETS, the
    caliper mount and the steering arm into ONE body.  The ball-joint arms reach to a
    SOCKET point that is offset from the joint centre TOWARD the hub along the kingpin
    axis (by _BJ_HALF_SEP), so the knuckle solid stops short of the arm eye (which is
    offset the other way) -- the ball joint bridges the gap.  Returns the knuckle body
    id and the per-joint socket centres + steering-eye centre for the joint builders."""
    k = p.knuckle
    kid = "knuckle_%s" % tag
    lbj = P("lower_ball_joint")
    ubj = P("upper_ball_joint")
    kp_axis = _unit(_sub(ubj, lbj))                 # kingpin (steering) axis, lo->hi
    # the knuckle SOCKETS sit _BJ_HALF_SEP toward the hub from each ball joint; the arm
    # EYES sit the same distance the OTHER way.  The cast upright (web + arms) must stay
    # on the SOCKET side and never reach the arm eyes -- so the web spans only between
    # the sockets (a touch inboard of them), NOT down to the ball joints.
    lo_socket_z = (lbj[2] + _BJ_HALF_SEP * kp_axis[2])
    hi_socket_z = (ubj[2] - _BJ_HALF_SEP * kp_axis[2])
    z_lo = lo_socket_z - 4.0
    z_hi = hi_socket_z + 4.0
    barrel_half = k.width_mm / 2.0 + 6.0
    # Bearing-boss WALL only (not the whole knuckle thickness): a sane cast wall around
    # the hub bore, kept small enough that the barrel OD clears the ball joints / toe
    # joint (which sit ~72-81 mm from the hub axis) -- otherwise a fat solid barrel
    # would swallow them in the interference check (it ignores the bore cut, so the
    # barrel reads as a solid disc).  The bore Ø stays the source of truth.
    barrel_wall = min(16.0, k.thickness_mm * 0.4)
    barrel_od = k.hub_bore_diameter_mm + 2.0 * barrel_wall

    # 1) hub barrel: a hollow RING (kind="tube" -> outer cylinder create + inner bore
    #    subtract, so it BUILDS in NX) around the wheel-spin (Y) axis; the hub presses
    #    into the bore.  This is the CREATE that every cast feature unites into.
    steps.append(F.ring_body(
        kid, "knuckle", "Knuckle_Upright_%s" % U, M((0.0, 0.0, 0.0)),
        (0.0, 1.0, 0.0), barrel_od, k.hub_bore_diameter_mm, 2.0 * barrel_half,
        COL_KNUCKLE, "cast_al"))

    # 2) upright web: a vertical slab from the lower to the upper joint, on the inboard
    #    face of the barrel. United to the barrel.
    web_lo = M((0.0, -(k.width_mm / 2.0 + 2.0), z_lo))
    web_hi = M((0.0, -(k.width_mm / 2.0 + 2.0), z_hi))
    steps.append(_prism_link(
        "knuckle_web_%s" % tag, "knuckle", "Knuckle_Web_%s" % U, web_lo, web_hi,
        k.thickness_mm * 1.6, k.thickness_mm, COL_KNUCKLE, material="cast_al",
        u_dir=(1.0, 0.0, 0.0), boolean="unite", target=kid))

    # 3) ball-joint arms + SOCKETS: a tapered cast arm from the barrel out toward each
    #    ball joint, ending at a SOCKET point offset TOWARD the hub (-kingpin side) so
    #    the knuckle stops short of the arm eye.  The socket itself is a ring boss
    #    united in, whose bore takes the ball stud.
    sockets: Dict[str, Vec3] = {}
    arm_w = k.thickness_mm * 1.5
    bj_d = p.arm.ball_joint_diameter_mm
    socket_od = bj_d * 1.7
    socket_bore = bj_d * 0.5                         # == the ball-stud OD (a press fit)

    def _socket(name: str, joint: Vec3, toward_hub_sign: float, z_target: float):
        # socket centre: offset from the joint along the kingpin axis toward the hub
        sc = _add(joint, _scale(kp_axis, toward_hub_sign * _BJ_HALF_SEP))
        sockets[name] = sc
        base = M((0.0, -barrel_half * 0.5, z_target))
        # cast arm from the barrel face to the socket OUTER FACE (it stops at the socket
        # OD, not the socket centre, so the solid arm never surrounds the ball stud --
        # only the bored socket ring does).  United into the casting.
        arm_tip = _add(sc, _scale(_unit(_sub(base, sc)), socket_od * 0.45))
        _tapered_leg(steps, "knuckle_%s_arm_%s" % (name, tag), "knuckle",
                     "Knuckle_%s_Arm_%s" % (name.title(), U), base, arm_tip,
                     arm_w, arm_w * 0.75, k.thickness_mm, k.thickness_mm * 0.85,
                     COL_KNUCKLE, "cast_al", u_dir=(1.0, 0.0, 0.0), target=kid)
        # socket boss carrying the ball-stud bore: a SOLID boss UNITED into the casting
        # plus the ball-stud bore SUBTRACTED from it (a create+subtract pair, not a
        # single annulus -- so it builds in NX).
        F.united_eye(steps, "knuckle_%s_socket_%s" % (name, tag), "knuckle",
                     "Knuckle_%s_Socket_%s" % (name.title(), U), kid, sc, kp_axis,
                     socket_od, socket_bore, _BJ_HALF_SEP * 0.8, COL_KNUCKLE, "cast_al")

    # lower socket sits ABOVE its joint (toward the hub); upper socket BELOW its joint.
    _socket("lbj", lbj, +1.0, max(z_lo, lbj[2]))
    if p.geometry.type in ("multilink", "double_wishbone"):
        _socket("ubj", ubj, -1.0, min(z_hi, ubj[2]))

    # 4) caliper mount: a boss bridge united fore (+X) of the hub.
    if k.brake_caliper_mount:
        cm = P("caliper_mount")
        base = M((0.0, hp["caliper_mount"][1], hp["caliper_mount"][2]))
        steps.append(_axis_cyl(
            "caliper_mount_%s" % tag, "caliper_mount", "Caliper_Mount_%s" % U,
            base, cm, k.hub_bore_diameter_mm * 0.42, COL_KNUCKLE,
            material="cast_al", boolean="unite", target=kid))

    # 5) steering arm: a cast stub from the web out to a STEERING EYE at the toe-outboard
    #    (tie-rod) point.  The tie-rod joint is a real tapered-stud joint on a VERTICAL
    #    (Z) pivot axis: the tie-rod eye stacks ABOVE the steer eye and a vertical bolt
    #    drops through both, so the bolt head/nut go up/down -- clear of the horizontal
    #    link shank and the knuckle barrel (the old in-plane clevis fouled both).
    toe_out = P("toe_outboard")
    toe_in = P("toe_pickup")
    toe_axis = _unit(_sub(toe_in, toe_out))
    steer_pivot = (0.0, 0.0, 1.0)                # vertical pivot (transverse to the rod)
    clevis_t = max(8.0, rod_d_of(p) * 0.7)
    steer_eye = _add(toe_out, _scale(steer_pivot, -0.5 * clevis_t))  # lower disc of the stack
    rod_d = rod_d_of(p)
    eye_bore = F.bolt_clearance(rod_d * 1.0) + 2.0
    steer_base = M((toe_out[0] * 0.3, -(k.width_mm / 2.0 + 2.0), toe_out[2]))
    # the steer arm TAPERS to a slim tip approaching the eye from the inboard side.
    steer_tip = _add(steer_eye, _scale(_unit(_sub(steer_base, steer_eye)), rod_d))
    _tapered_leg(steps, "knuckle_steer_arm_%s" % tag, "knuckle", "Steering_Arm_%s" % U,
                 steer_base, steer_tip, k.thickness_mm, rod_d * 1.0,
                 k.thickness_mm * 0.9, rod_d * 1.0, COL_KNUCKLE, "cast_al",
                 u_dir=(0.0, 0.0, 1.0), target=kid)
    # steering eye: a SOLID boss UNITED into the knuckle + the toe-joint bolt bore
    # SUBTRACTED (create+subtract, not an annulus), coaxial with the toe-link axis.
    F.united_eye(steps, "knuckle_steer_eye_%s" % tag, "knuckle", "Steering_Eye_%s" % U,
                 kid, steer_eye, steer_pivot, rod_d * 2.0, eye_bore, clevis_t,
                 COL_KNUCKLE, "cast_al")
    sockets["steer_eye"] = steer_eye
    sockets["steer_eye_bore"] = (eye_bore, 0.0, 0.0)             # carry the bore dia
    sockets["clevis_t"] = (clevis_t, 0.0, 0.0)                   # carry the clevis thickness
    sockets["steer_pivot"] = steer_pivot                        # vertical clevis pivot axis

    # FINAL: cut the wheel-hub bearing bore along the lateral (Y) wheel-spin axis
    #        through the whole casting.
    bore_half = barrel_half + 2.0
    steps.append(_axis_cyl(
        "knuckle_hub_bore_%s" % tag, "hub_bore_cut", "Hub_Bore_%s" % U,
        M((0.0, -bore_half, 0.0)), M((0.0, bore_half, 0.0)),
        k.hub_bore_diameter_mm, COL_AIR, material="air",
        boolean="subtract", target=kid))
    return kid, sockets


# --------------------------------------------------------------------------- #
# one control ARM = hub eye + fore leg + aft leg, UNITED into one body, with a
# ball joint bridging the arm eye and the knuckle socket
# --------------------------------------------------------------------------- #
def _control_arm(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str,
                 prefix: str, bj: Vec3, pf: Vec3, pa: Vec3, socket: Vec3,
                 kp_axis: Vec3, w_in: float, w_out: float, h_in: float, h_out: float,
                 bush_d: float) -> None:
    """Build one A-arm as a SINGLE united body: a hub EYE at the ball joint (offset
    along the kingpin axis AWAY from the knuckle socket), a fore leg and an aft leg
    that converge on that eye.  Then the BALL JOINT (housing pressed in the arm-eye
    bore + stud into the knuckle socket) and the two inboard BUSHINGS + their bolts.

      knuckle socket  <-- ball stud -->  arm eye  (separated along the kingpin axis)
    """
    bj_d = p.arm.ball_joint_diameter_mm
    role = "%s_arm" % prefix                     # semantic role ("lower_arm"/"upper_arm")
    arm_id = "%s_hub_%s" % (role, tag)           # the arm's create-body id
    # arm eye centre: offset from the joint along the kingpin axis AWAY from the
    # knuckle socket (the knuckle socket was offset toward the hub by _BJ_HALF_SEP), so
    # the two members sit on opposite sides of the joint and never share volume.
    toward = _unit(_sub(socket, bj))
    eye_c = _sub(bj, _scale(toward, _BJ_HALF_SEP))
    # the eye OD is sized so the rectangular LEG cross-section embeds well inside the
    # round eye where they meet.  If the eye OD ~= the leg width, a flat leg side face
    # sits TANGENT to the eye cylinder and NX leaves a spiky/sliver trim face (the
    # Check-Mate "Faces - Spikes/Cuts" defect, shortest edge ~0.1 mm).  Sizing the eye
    # to comfortably exceed the leg's cross-section DIAGONAL turns those tangencies into
    # clean chord cuts.  (A real A-arm ball-joint boss is a chunky hub, larger than the
    # arm section, so this is also more realistic.)
    # ROUND-BAR legs (below) join the ROUND eye in a clean cylinder-cylinder boolean -- a
    # smooth lens-shaped intersection curve with NO sharp corners -- so they cannot leave
    # the thin sliver/spike trim faces a rectangular leg does where its FLAT faces graze
    # the round eye (the Check-Mate "Faces - Spikes/Cuts" defect).  The toe link and the
    # anti-roll drop link are round bars for exactly this reason and never spiked.  A
    # round leg's circular section is perpendicular to its (radial) axis, so it does NOT
    # reach radially toward the bore -- the bar stays ~leg_out from the kingpin axis, well
    # clear of the ball-joint bore.
    leg_d = 0.5 * (w_out + h_out)                # round control-arm leg-bar diameter
    eye_od = bj_d * 1.9                          # ball-joint boss OD (round)
    eye_bore = bj_d                              # ball housing OD == this bore (press fit)
    eye_len = max(bj_d * 1.1, leg_d + 8.0)       # contain the round leg bar in the eye barrel

    # 1) hub EYE = the create body the legs unite into (a ring around the ball-joint
    #    housing, coaxial with the kingpin axis).  The legs blend into the eye OD (one
    #    united body); the eye's real bore is what the ball-joint housing presses into.
    steps.append(F.ring_body(
        arm_id, role, "%s_Arm_Hub_%s" % (prefix.title(), U),
        eye_c, kp_axis, eye_od, eye_bore, eye_len, COL_ARM, "arm_steel"))

    # 2) fore + aft legs from the inboard PICKUP EYES to the hub eye.  Each leg ends at
    #    the hub-eye OUTER face (so the solid leg never surrounds the ball stud) and
    #    starts at a bored PICKUP EYE (so the leg never fills the bushing can -- the
    #    bushing presses into the eye bore, the bolt runs transverse through it).
    pivots = {}
    arm_legs = {}                                # nm -> (leg_in, leg_out) for bracket rooting
    for nm, pt in (("fore", pf), ("aft", pa)):
        pivot = _transverse(_sub(eye_c, pt))     # transverse bushing/bolt pivot axis
        pivots[nm] = (pt, pivot)
        # leg from just outboard of the pickup eye -> into the hub-eye boss.  The tip sits
        # at 0.38*eye_od from the eye centre: inside the ring OD (0.95*bj_d) for a real
        # cylinder-cylinder overlap, yet ~1.7*bj_d - safely OUTSIDE the bore radius
        # (0.5*bj_d), so the round bar never reaches the (pre-bored) eye void where the
        # ball-joint housing seats.
        leg_in = _add(pt, _scale(_unit(_sub(eye_c, pt)), bush_d * 0.55))
        leg_out = _add(eye_c, _scale(_unit(_sub(pt, eye_c)), eye_od * 0.38))
        arm_legs[nm] = (leg_in, leg_out)
        # ROUND leg bar (a solid cylinder, like the toe / anti-roll links) -- a clean
        # cylinder-cylinder union into the round eye, no flat-face tangency slivers.
        steps.append(_axis_cyl(
            "%s_%s_%s" % (role, nm, tag), role,
            "%s_Arm_%s_%s" % (prefix.title(), nm.title(), U), leg_in, leg_out,
            leg_d, COL_ARM, material="arm_steel", boolean="unite", target=arm_id))
        # PICKUP EYE: a bored boss UNITED into the arm at the pickup (the leg reaches it,
        # so the unite is connected); the bushing can presses into this eye bore with a
        # clearance gap (eye bore = can OD + 2*clr, so the fit reads clean, not as
        # interference).
        eye_l = max(20.0, bush_d * 0.9)
        pickup_eye_bore = bush_d + 2.0 * F.FIT_CLEARANCE
        F.united_eye(steps, "%s_%s_eye_%s" % (prefix, nm, tag), role,
                     "%s_Arm_%s_Eye_%s" % (prefix.title(), nm.title(), U), arm_id,
                     pt, pivot, bush_d + 12.0, pickup_eye_bore, eye_l,
                     COL_ARM, "arm_steel")

    # 3) BALL JOINT = ONE body bridging the arm eye -> the knuckle socket:
    #      * a HOUSING (solid disc) pressed into the arm-eye bore (housing OD = eye bore
    #        - clearance, so it fills the void without touching the eye material);
    #      * a STUD (rod) UNITED to the housing, reaching into the knuckle socket bore
    #        (stud OD = socket bore - clearance).
    #    Housing + stud are one create-body, so they never interfere with each other,
    #    and each press fit reads ~0 (a clearance gap inside each host bore).
    bj_id = "%s_bj_housing_%s" % (prefix, tag)
    hous_len = bj_d * 1.1
    hous_d = eye_bore - 2.0 * F.FIT_CLEARANCE
    steps.append(F.solid_disc(
        bj_id, "ball_joint", "%s_BJ_Housing_%s" % (prefix.title(), U),
        eye_c, kp_axis, hous_d, hous_len, F.COL_SLEEVE, "joint_steel"))
    stud_d = bj_d * 0.5 - 2.0 * F.FIT_CLEARANCE
    steps.append(F.rod_body(
        "%s_bj_stud_%s" % (prefix, tag), "ball_joint",
        "%s_BJ_Stud_%s" % (prefix.title(), U), eye_c, socket, stud_d,
        F.COL_SLEEVE, "joint_steel", boolean="unite", target=bj_id))

    # 4) inboard bushings + transverse bolts, seated in the pickup eyes built above.
    for nm, (pt, pivot) in pivots.items():
        # the bushing can presses into the pickup-eye bore (Ø bush_d), the steel sleeve
        # sits in the can, and the bolt runs TRANSVERSE through the sleeve -- all coaxial
        # on the pivot axis, none filling the leg solid.
        bush_l = max(16.0, bush_d * 0.7)
        sb, _ = F.bushing(steps, tag, U, "%s_%s" % (prefix, nm), pt, pivot,
                          bush_d, bush_l, F.bolt_clearance(bush_d * 0.3))
        F.bolt_assembly(steps, tag, U, "%s_%s" % (prefix, nm), pt, pivot,
                        bush_l, sb)
    return arm_id, {"eye_c": eye_c, "legs": arm_legs}


# --------------------------------------------------------------------------- #
# one corner -- built from the engineering hardpoints (true 3D)
# --------------------------------------------------------------------------- #
def corner_steps(p: SuspensionParams, tag: str, mirror: bool) -> List[BuildStep]:
    """Build one corner from the shared hardpoint table."""
    g, s, d = p.geometry, p.spring, p.damper
    a, k, arm = p.antiroll, p.knuckle, p.arm
    U = tag.upper()
    hp = engineering.hardpoints(p)
    track = g.track_width_mm

    def M(pt: Vec3) -> Vec3:
        return _mirror_corner(pt, track) if mirror else pt

    def P(name: str) -> Vec3:
        return M(hp[name])

    steps: List[BuildStep] = []

    # --- KNUCKLE / UPRIGHT (cast; ties hub bore + ball-joint sockets + caliper +
    #     steering eye), returns the per-joint socket centres ------------------- #
    kid, sockets = _knuckle_steps(steps, p, tag, U, P, M, hp)
    kp_axis = _unit(_sub(P("upper_ball_joint"), P("lower_ball_joint")))

    # leg sections: a fat, loaded inboard end necking to a slim ball-joint end.
    w_in = max(26.0, arm.arm_diameter_mm + 16.0)
    w_out = max(16.0, arm.arm_diameter_mm + 2.0)
    h_in = max(16.0, arm.arm_diameter_mm + 4.0)
    h_out = max(10.0, arm.arm_diameter_mm - 6.0)
    bush_d = arm.bushing_diameter_mm

    # --- LOWER CONTROL ARM: a proper A-ARM, ONE united body, ball joint to knuckle -
    lower_arm_id, lower_arm_geo = _control_arm(
        steps, p, tag, U, "lower", P("lower_ball_joint"),
        P("lower_pickup_fore"), P("lower_pickup_aft"),
        sockets["lbj"], kp_axis, w_in, w_out, h_in, h_out, bush_d)

    # --- UPPER CONTROL ARM (multilink / double_wishbone only) ------------------- #
    if g.type in ("multilink", "double_wishbone"):
        _control_arm(steps, p, tag, U, "upper", P("upper_ball_joint"),
                     P("upper_pickup_fore"), P("upper_pickup_aft"), sockets["ubj"],
                     kp_axis, w_in * 0.85, w_out * 0.9, h_in * 0.85, h_out, bush_d)

    # --- TOE / TIE LINK: shank + two eyes UNITED into one body ------------------ #
    _toe_link(steps, p, tag, U, P, sockets, bush_d)

    # --- COIL-OVER: damper body + piston rod + coil spring + perches ------------ #
    _coilover_steps(steps, p, tag, U, P)

    # --- ANTI-ROLL (STABILISER) DROP LINK: link + two eyes UNITED into one body -- #
    if a.enabled:
        _antiroll_link(steps, p, tag, U, P, lower_arm_id, lower_arm_geo)
    return steps


def _toe_link(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str, P,
              sockets: Dict[str, Vec3], bush_d: float) -> None:
    """The TOE / tie link as ONE united body: a slender shank with an EYE at each end
    (both united into the shank create).  The inboard eye takes a bushing + bolt to the
    chassis; the outboard eye STACKS against the knuckle steering eye (a clevis) with a
    bolt through both -- so the link never overlaps the knuckle as a solid."""
    arm = p.arm
    toe_in = P("toe_pickup")
    toe_out = P("toe_outboard")
    rod_d = rod_d_of(p)
    axis = _unit(_sub(toe_out, toe_in))
    eye_od = rod_d * 2.0
    # the tie-rod OUTBOARD joint is a vertical tapered-stud joint: the knuckle steering
    # eye is the LOWER disc (on the vertical pivot) and the tie-rod outboard eye is the
    # UPPER disc stacked on top of it; a vertical bolt drops through both with its head
    # ABOVE and nut BELOW -- clear of the horizontal link shank and the knuckle barrel.
    steer_eye = sockets["steer_eye"]
    steer_bore = sockets["steer_eye_bore"][0]
    clevis_t = sockets["clevis_t"][0]
    steer_pivot = sockets["steer_pivot"]
    # upper disc of the stack, with a 3 mm face gap so the touching clevis faces do not
    # sample as interference.
    out_eye = _add(steer_eye, _scale(steer_pivot, clevis_t + 3.0))
    in_eye = toe_in
    in_axis = _transverse(axis)                  # inboard pivot axis (transverse to rod)
    link_id = "toe_link_%s" % tag

    # 1) shank create (from just inside the inboard eye to just inside the outboard eye)
    sh0 = _add(in_eye, _scale(_unit(_sub(out_eye, in_eye)), rod_d * 0.5))
    sh1 = _add(out_eye, _scale(_unit(_sub(in_eye, out_eye)), rod_d * 0.5))
    steps.append(_axis_cyl(
        link_id, "toe_link", "Toe_Link_%s" % U, sh0, sh1, rod_d, COL_ARM,
        material="arm_steel"))
    # 2) eyes united into the shank: the outboard eye is coaxial with the link (it
    #    stacks against the knuckle steering eye); the inboard eye is coaxial with the
    #    TRANSVERSE pivot axis (a real tie-rod inner joint), so its bushing/bolt do not
    #    run collinear into the shank.  The inboard eye bore holds the whole bushing can
    #    (can OD == eye bore), so nothing protrudes past the eye into the shank.
    in_bush_can = eye_od * 0.62                  # bushing can OD == inboard-eye bore
    in_eye_len = max(rod_d * 1.1, bush_d * 0.6 + 2.0)
    F.united_eye(steps, "toe_eye_in_%s" % tag, "toe_link", "Toe_Eye_In_%s" % U,
                 link_id, in_eye, in_axis, eye_od, in_bush_can, in_eye_len,
                 COL_MOUNT, "joint_steel")
    F.united_eye(steps, "toe_eye_out_%s" % tag, "toe_link", "Toe_Eye_Out_%s" % U,
                 link_id, out_eye, steer_pivot, eye_od, steer_bore, clevis_t,
                 COL_MOUNT, "joint_steel")

    # 3) inboard joint: a bushing in the inboard eye + bolt to the chassis pickup, on a
    #    pivot axis TRANSVERSE to the link (a real tie-rod inner joint pivots about an
    #    axis perpendicular to the rod) so the bolt does not run collinear into the
    #    link shank.  The inboard eye above is built transverse to match.
    bush_l = in_eye_len
    sb, _ = F.bushing(steps, tag, U, "toe_in", in_eye, in_axis, in_bush_can, bush_l,
                      F.bolt_clearance(in_bush_can * 0.3))
    F.bolt_assembly(steps, tag, U, "toe_in", in_eye, in_axis, bush_l, sb)
    # 4) outboard joint: a VERTICAL bolt through both stacked clevis discs (tie-rod
    #    upper eye + knuckle lower steer eye).  span = the two clevis thicknesses; the
    #    head sits ABOVE and the nut BELOW, clear of the link shank and the barrel.
    pin_c = _lerp(out_eye, steer_eye, 0.5)
    span = _norm(_sub(out_eye, steer_eye)) + clevis_t   # both eyes + the face gap
    F.bolt_assembly(steps, tag, U, "toe_out", pin_c, steer_pivot, span, steer_bore,
                    head_d=rod_d * 1.3)


def _antiroll_link(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str, P,
                   arm_id: str, arm_geo: Dict[str, Any]) -> None:
    """The anti-roll DROP LINK as ONE united body (a slender shank + an eye at each end).
    The drop link runs FORE of (offset +X from) the control arm so it never buries into
    the arm; a cast BRACKET on the lower arm reaches out to the lower eye, and the joint
    is a bolt through the stacked bracket-eye + drop-link eye (a real clevis).  The eyes
    pivot transverse to the link so the bolt head/nut sit clear to the side.

    ``arm_geo`` carries the lower arm's leg endpoints (from _control_arm) so the bracket
    can be rooted ON a leg -- see br_root below."""
    a = p.antiroll
    lo0 = P("arb_link_lower")
    hi0 = P("arb_link_upper")
    rod_d = a.bar_diameter_mm * 0.7
    axis = _unit(_sub(hi0, lo0))
    pivot = _transverse(axis)                    # eye/bolt pivot axis (transverse to link)
    eye_od = a.bar_diameter_mm * 1.6
    eye_len = a.bar_diameter_mm * 0.9
    bore = F.bolt_clearance(a.bar_diameter_mm * 0.5) + 1.0
    # offset the whole drop link FORE (+X) so it clears the control-arm legs; the bracket
    # below bridges back to the arm.  (Mirroring flips X via the M() that produced lo0.)
    off = 1.0 if lo0[0] >= 0 else -1.0
    offset = (off * (eye_od * 0.5 + 50.0), 0.0, 0.0)
    lo = _add(lo0, offset)
    hi = _add(hi0, offset)
    link_id = "antiroll_%s" % tag
    # the shank runs the FULL eye-centre to eye-centre span (it passes THROUGH both eye
    # bosses), so each eye boss overlaps the shank -> the unite makes one connected body
    # and the eye bore cuts it (priority 1: a disjoint boss makes the bore land outside).
    steps.append(_axis_cyl(
        link_id, "anti_roll_bar", "Anti_Roll_Link_%s" % U, lo, hi, rod_d, COL_ARB,
        material="bar_steel"))
    for nm, pt in (("lo", lo), ("hi", hi)):
        F.united_eye(steps, "antiroll_eye_%s_%s" % (nm, tag), "anti_roll_bar",
                     "Anti_Roll_Eye_%s_%s" % (nm.title(), U), link_id, pt, pivot,
                     eye_od, bore, eye_len, COL_MOUNT, "joint_steel")
    # the UPPER eye bolts to the bar (a free rod-end here): a bolt centred on it.
    F.bolt_assembly(steps, tag, U, "antiroll_hi", hi, pivot, eye_len, bore)

    # cast BRACKET on the LOWER ARM: a tapered stub UNITED into the arm from the ARB
    # pickup (on the arm leg) out toward the drop link, ending at a standalone bracket
    # EYE RING.  The bracket eye is a STANDALONE kind="tube" ring (atomic create+bore --
    # never "tool outside target"); the bracket-leg tip reaches into it (touching) and
    # the joint bolt threads the ring + the drop-link lower eye.
    # ROOT the bracket ON the lower arm: project the ARB-pickup hardpoint onto the
    # NEAREST arm leg, so the bracket's first (root) segment overlaps the leg solid and
    # the unite CONNECTS.  The old root = the bare arb_link_lower hardpoint sat ~30 mm
    # OFF the leg centreline, so every bracket segment missed the arm -> the unite left
    # 3 floating, unnamed lumps (the inspector's "3 unnamed bodies").  Projecting onto
    # the leg fixes the structural disconnect AND removes the unnamed bodies.
    legs = (arm_geo or {}).get("legs", {})
    cand = [(_norm(_sub(_nearest_on_seg(lo0, la, lb), lo0)), _nearest_on_seg(lo0, la, lb))
            for (la, lb) in legs.values()]
    br_root = min(cand, key=lambda c: c[0])[1] if cand else lo0
    # stacked along the pivot from the drop eye, with a 1.5 mm face gap so the touching
    # clevis faces do not sample as interference.
    br_eye_c = _add(lo, _scale(pivot, eye_len + 1.5))
    # the bracket leg stops at the eye RING's OUTER face (not its centre), so the solid
    # leg only TOUCHES the standalone eye ring -- it does not fill the ring wall (which
    # would read as interference now the ring is its own body).
    br_leg_tip = _add(br_eye_c, _scale(_unit(_sub(br_root, br_eye_c)), eye_od * 0.55 + 4.0))
    # build the bracket from the EYE end (p_in) to the ARM ROOT (p_out): _tapered_leg
    # emits the p_out segment FIRST, and p_out = the arm root (on the leg), so the first
    # unite touches the arm -> the bracket stays connected (priority 2 unite-order).
    _tapered_leg(steps, "antiroll_bracket_%s" % tag, "lower_arm",
                 "Anti_Roll_Bracket_%s" % U, br_leg_tip, br_root,
                 rod_d * 1.2, rod_d * 1.6, rod_d * 1.0, rod_d * 1.2,
                 COL_ARM, "arm_steel", u_dir=(0.0, 0.0, 1.0), target=arm_id)
    steps.append(F.ring_body(
        "antiroll_bracket_eye_%s" % tag, "anti_roll_bar", "Anti_Roll_Bracket_Eye_%s" % U,
        br_eye_c, pivot, eye_od, bore, eye_len, COL_ARM, "arm_steel"))
    # the LOWER joint bolt threads BOTH stacked eyes (drop-link lower eye + bracket eye).
    pin_c = _lerp(lo, br_eye_c, 0.5)
    span = _norm(_sub(lo, br_eye_c)) + eye_len          # both eyes + the face gap
    F.bolt_assembly(steps, tag, U, "antiroll_lo", pin_c, pivot, span, bore)


def _coilover_steps(steps: List[BuildStep], p: SuspensionParams, tag: str, U: str, P) -> None:
    """The COIL-OVER: a damper BODY cylinder + an exposed piston ROD united into one
    body, with the COIL SPRING as ring-turns COAXIAL around the rod, and a lower + upper
    spring PERCH + a TOP MOUNT modelled as STANDALONE kind="tube" RINGS the rod passes
    through.  The perches/top-mount are NOT united onto the rod: a tube's bore is coaxial
    with its OD by construction, so it can never throw the NX "tool body completely
    outside target body" error (the boss-unite-onto-a-spine + separate-bore pattern did,
    three rounds running, when the boss did not reliably overlap the rod).  The rod OD is
    a clearance under each ring bore, so the rod sits in the void -- no interference --
    and the perch reads as a real ring the rod runs through; the coil seats on the perch
    faces."""
    s, d, g, k = p.spring, p.damper, p.geometry, p.knuckle
    coil_d = s.coil_outer_diameter_mm
    wire_d = max(8.0, coil_d * 0.10)

    if g.type == "macpherson":
        # MacPherson: the strut IS the upper link, leaning along lower_ball_joint ->
        # strut_top.  Its visible body/perch start ABOVE the upright (a real strut tube
        # clamps to the knuckle and the lower spring seat sits well up the tube), so the
        # damper body clears the hub barrel + ball-joint zone rather than burying into
        # it.  Keep the lean (the axis) but raise the seat clear of the knuckle envelope.
        bj = P("lower_ball_joint")
        top = P("strut_top")
        w = _unit(_sub(top, bj))
        barrel_clear = (k.hub_bore_diameter_mm + 2.0 * min(16.0, k.thickness_mm * 0.4)) / 2.0
        clear_z = barrel_clear + d.damper_diameter_mm / 2.0 + 14.0    # clear the barrel disc
        # advance along the strut axis until the seat rises above the knuckle envelope
        t = (clear_z - bj[2]) / w[2] if w[2] > 1e-6 else 0.0
        seat = _add(bj, _scale(w, max(0.0, t)))
    else:
        seat = P("damper_lower")
        top = P("damper_top")

    axis = _sub(top, seat)
    L = _norm(axis)
    w = _unit(axis)
    cov_id = "damper_%s" % tag

    # damper BODY: the lower ~45 % of the run -- the create the coil-over unites into.
    rod_d = d.damper_diameter_mm * 0.42
    body_top = _add(seat, _scale(w, 0.45 * L))
    steps.append(_axis_cyl(
        cov_id, "damper",
        "Strut_Damper_%s" % U if g.type == "macpherson" else "Damper_%s" % U,
        seat, body_top, d.damper_diameter_mm, COL_DAMPER, material="damper_steel"))
    # piston ROD: a thinner shaft from the body top up to (and a touch past) the top
    # mount, UNITED in.  The rod is the spine every perch/top-mount boss overlaps, so
    # those unites stay connected and their bores cut a clean rod-clearance hole.
    rod_tip = _add(top, _scale(w, wire_d))
    steps.append(_axis_cyl(
        "damper_rod_%s" % tag, "damper", "Damper_Rod_%s" % U,
        body_top, rod_tip, rod_d, COL_ROD, material="rod_steel",
        boolean="unite", target=cov_id))

    # The fat coil (and its perches) sit ABOVE the chassis rail band, on the ROD region.
    coil_lo_local_z = coil_d / 2.0 + 8.0          # local Z that clears the rail band
    t_clear = (coil_lo_local_z - seat[2]) / w[2] if w[2] > 1e-6 else 0.0
    t_lo = max(0.46 * L, min(t_clear, 0.60 * L))  # on the rod, above body_top
    coil_base = _add(seat, _scale(w, t_lo))
    rod_hole = rod_d + 2.0 * F.FIT_CLEARANCE      # the rod passes the perch with a gap

    # lower + upper spring PERCH and the TOP MOUNT are STANDALONE kind="tube" RINGS (an
    # atomic outer-create + coaxial inner-subtract -- the bore is coaxial with the OD by
    # construction, so it can never throw "tool body completely outside target").  The
    # damper ROD passes THROUGH each ring's bore as a thinner solid (rod_OD < bore -
    # clearance), so the rod sits in the void with no interference and the perch reads as
    # a real ring the rod runs through.  They are their OWN bodies (NOT united onto the
    # rod) -- at most they touch the rod at the bore wall (a clearance gap, so ~0).
    perch_od = coil_d + 8.0                       # just larger than the spring OD
    perch_lo_c = _add(coil_base, _scale(w, wire_d))
    steps.append(F.ring_body(
        "spring_perch_lo_%s" % tag, "spring", "Spring_Perch_Lo_%s" % U,
        perch_lo_c, w, perch_od, rod_hole, wire_d, COL_PERCH, "perch_steel"))
    spring_top = _add(seat, _scale(w, 0.82 * L))
    perch_hi_c = _add(spring_top, _scale(w, wire_d * 0.5))
    steps.append(F.ring_body(
        "spring_perch_hi_%s" % tag, "spring", "Spring_Perch_Hi_%s" % U,
        perch_hi_c, w, perch_od, rod_hole, wire_d, COL_PERCH, "perch_steel"))

    # TOP MOUNT: a body/tower mount RING at the top (the rod passes through its bore) so
    # the coil-over does not float.  Centred BELOW the rod tip so the rod runs through.
    top_c = _add(top, _scale(w, -wire_d))
    steps.append(F.ring_body(
        "damper_top_mount_%s" % tag, "damper", "Top_Mount_%s" % U,
        top_c, w, coil_d * 0.9, rod_hole, wire_d, COL_MOUNT, "mount_steel"))

    # the COIL itself -- ring-turns coaxial around the damper, seated perch face to
    # perch face (a SEPARATE body, clear of the damper body by construction).
    coil_seat = _add(perch_lo_c, _scale(w, wire_d * 0.5))
    coil_topf = _add(perch_hi_c, _scale(w, -wire_d * 0.5))
    _coil_turns(steps, "spring_%s" % tag, "Coil_Spring_%s" % U,
                coil_seat, coil_topf, coil_d, wire_d, COL_SPRING, "spring_steel")


def _corners(p: SuspensionParams):
    """Yield (tag, mirror) per modelled corner."""
    if p.corners == "axle":
        return [("r", False), ("l", True)]
    return [("r", False)]


def build_steps(p: SuspensionParams) -> List[BuildStep]:
    steps: List[BuildStep] = []
    for tag, mirror in _corners(p):
        steps.extend(corner_steps(p, tag, mirror))
    return steps


def generate(p: SuspensionParams = None) -> Dict[str, Any]:
    """Full suspension-corner blueprint dict (NX-independent)."""
    if p is None:
        p = SuspensionParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "suspension_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": g.corner_envelope_height_mm,
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
