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

TRUE-3D MODELLING (no more flat +Z plates)
    Every link is modelled along its TRUE 3D axis with the general beam primitive
    kind="prism" (a 2D (u, v) section extruded along an arbitrary world axis at an
    arbitrary world origin) or an axis-placed cylinder/tube:
      * lower A-arm  : two legs (fore + aft) from inboard chassis pickups (-Y, low
                       Z) up/out to the lower ball joint just inboard of the hub;
      * upper A-arm  : (multilink / double_wishbone) two legs from inboard pickups
                       (-Y, high Z) to the upper ball joint above the hub;
      * toe / tie link: a single bar set rearward (-X) from an inboard pickup to a
                       steering-arm point on the knuckle;
      * coil spring  : an axis-placed tube at its REAL inclination, seated on the
                       lower arm and reaching up to a body mount;
      * damper       : an axis-placed cylinder coaxial with the spring (MacPherson:
                       a coaxial strut through the upright top, the upper link);
      * ball joints / bushings / caliper mount / anti-roll drop link: placed in 3D
                       at their true hardpoints.

    All hardpoint coordinates come from engineering.hardpoints(p), which is the one
    source of truth shared with the world-bounding-box / connectivity tests and the
    validate() reach checks.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict, List, Sequence, Tuple

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import SuspensionParams

Vec3 = Tuple[float, float, float]

# component colours (RGB 0-255)
COL_KNUCKLE = (95, 100, 110)
COL_ARM = (120, 124, 132)
COL_SPRING = (150, 90, 60)
COL_DAMPER = (70, 110, 160)
COL_ARB = (140, 130, 90)
COL_MOUNT = (60, 62, 70)
COL_AIR = (0, 0, 0)


# --------------------------------------------------------------------------- #
# small 3D helpers (pure math; the NX-free twins live in motor_nx.blueprint)
# --------------------------------------------------------------------------- #
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


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


def _prism_link(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
                width: float, height: float, color, material: str = "arm_steel",
                u_dir: Vec3 = (1.0, 0.0, 0.0)) -> BuildStep:
    """A structural link as a rectangular-section PRISM along its TRUE 3D axis from
    p0 -> p1. The section (width x height in local u,v) is extruded along the unit
    axis (p1 - p0) by the link length, with the section plane placed at p0. `u_dir`
    sets the local +u in world (default +X = longitudinal); +v = axis x u. This is
    the general beam: a horizontal arm leg, an inclined toe link, anything."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="prism", boolean="create",
        body_name=body_name, material=material, color=color,
        profile=_rect_uv(width, height), origin3=p0, axis=axis, u_dir=u_dir,
        length=length)


def _axis_cyl(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
              diameter: float, color, material: str = "joint_steel",
              boolean: str = "create", target: str = None) -> BuildStep:
    """A solid cylinder coaxial with the TRUE 3D axis p0 -> p1 (e.g. a damper rod,
    a ball-joint stud, an anti-roll drop link). Built from the base point p0 along
    the unit axis by the segment length."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=length,
        target=target)


def _axis_tube(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
               outer_d: float, inner_d: float, color, material: str) -> BuildStep:
    """A hollow tube coaxial with the TRUE 3D axis p0 -> p1 (the coil spring blank:
    a representative annular envelope at the coil mean diameter, inclined to its
    real working axis)."""
    axis = _sub(p1, p0)
    length = _norm(axis)
    return BuildStep(
        id=step_id, role=role, kind="tube", boolean="create",
        body_name=body_name, material=material, color=color,
        outer_radius=outer_d / 2.0, inner_radius=max(2.0, inner_d / 2.0),
        origin3=p0, axis=axis, length=length)


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
    kid = "knuckle_%s" % tag

    # --- KNUCKLE / UPRIGHT -------------------------------------------------- #
    # A vertical box upright straddling the hub centre (local origin), spanning the
    # lower ball joint (below) to the upper mount (above). Built as a +Z prism:
    # section = thickness (X) x width (Y), extruded up Z. The hub bore is cut along
    # the lateral (Y) wheel-spin axis.
    z_lo = P("lower_ball_joint")[2] - 10.0
    z_hi = (P("upper_ball_joint")[2] if g.type in ("multilink", "double_wishbone")
            else P("strut_top")[2]) + 10.0
    upright_h = max(k.height_mm, z_hi - z_lo)
    steps.append(BuildStep(
        id=kid, role="knuckle", kind="prism", boolean="create",
        body_name="Knuckle_Upright_%s" % U, material="cast_al", color=COL_KNUCKLE,
        profile=_rect_uv(k.thickness_mm, k.width_mm),
        origin3=M((0.0, 0.0, z_lo)), axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0),
        length=upright_h))
    # wheel-hub bearing bore: a cylinder coaxial with the lateral wheel-spin axis
    # (local Y) through the hub centre (origin). Matches the Gen-3 hub OD.
    bore_half = k.width_mm / 2.0 + 1.0
    steps.append(_axis_cyl(
        "knuckle_hub_bore_%s" % tag, "hub_bore_cut", "Hub_Bore_%s" % U,
        M((0.0, -bore_half, 0.0)), M((0.0, bore_half, 0.0)),
        k.hub_bore_diameter_mm, COL_AIR, material="air",
        boolean="subtract", target=kid))
    # brake-caliper mount lug: a boss united to the upright, set fore (+X) of the
    # hub at the typical trailing/leading caliper clock position.
    if k.brake_caliper_mount:
        cm = P("caliper_mount")
        steps.append(_axis_cyl(
            "caliper_mount_%s" % tag, "caliper_mount", "Caliper_Mount_%s" % U,
            M((0.0, hp["caliper_mount"][1], hp["caliper_mount"][2])), cm,
            k.hub_bore_diameter_mm * 0.45, COL_KNUCKLE,
            material="cast_al", boolean="unite", target=kid))

    # --- LOWER CONTROL ARM (A-arm: fore + aft legs) ------------------------- #
    obj = P("lower_ball_joint")
    arm_w = max(20.0, arm.arm_diameter_mm + 12.0)   # in-plane leg width
    arm_h = max(10.0, arm.arm_diameter_mm)          # leg thickness
    steps.append(_prism_link(
        "lower_arm_fore_%s" % tag, "lower_arm", "Lower_Arm_Fore_%s" % U,
        P("lower_pickup_fore"), obj, arm_w, arm_h, COL_ARM, u_dir=(0.0, 0.0, 1.0)))
    steps.append(_prism_link(
        "lower_arm_aft_%s" % tag, "lower_arm", "Lower_Arm_Aft_%s" % U,
        P("lower_pickup_aft"), obj, arm_w, arm_h, COL_ARM, u_dir=(0.0, 0.0, 1.0)))

    # --- UPPER CONTROL ARM (multilink / double_wishbone only) --------------- #
    if g.type in ("multilink", "double_wishbone"):
        obu = P("upper_ball_joint")
        steps.append(_prism_link(
            "upper_arm_fore_%s" % tag, "upper_arm", "Upper_Arm_Fore_%s" % U,
            P("upper_pickup_fore"), obu, arm_w, arm_h, COL_ARM, u_dir=(0.0, 0.0, 1.0)))
        steps.append(_prism_link(
            "upper_arm_aft_%s" % tag, "upper_arm", "Upper_Arm_Aft_%s" % U,
            P("upper_pickup_aft"), obu, arm_w, arm_h, COL_ARM, u_dir=(0.0, 0.0, 1.0)))

    # --- TOE / TIE LINK (rearward, single bar) ------------------------------ #
    steps.append(_prism_link(
        "toe_link_%s" % tag, "toe_link", "Toe_Link_%s" % U,
        P("toe_pickup"), P("toe_outboard"), arm_h, arm_h, COL_ARM,
        u_dir=(0.0, 0.0, 1.0)))

    # --- INBOARD BUSHINGS + OUTBOARD BALL JOINTS (3D placed) ---------------- #
    bj_d = arm.ball_joint_diameter_mm
    bush_d = arm.bushing_diameter_mm
    bush_l = max(12.0, bush_d / 2.0)

    def _bushing(name: str, pt: Vec3, axis_along: Vec3):
        """A compliance bushing as a short tube-less cylinder centred on the pickup,
        its bore axis along the link's longitudinal run."""
        n = _norm(axis_along) or 1.0
        ua = (axis_along[0] / n, axis_along[1] / n, axis_along[2] / n)
        p0 = (pt[0] - 0.5 * bush_l * ua[0], pt[1] - 0.5 * bush_l * ua[1],
              pt[2] - 0.5 * bush_l * ua[2])
        p1 = (pt[0] + 0.5 * bush_l * ua[0], pt[1] + 0.5 * bush_l * ua[1],
              pt[2] + 0.5 * bush_l * ua[2])
        steps.append(_axis_cyl(name, "bushing", "Bushing_%s" % U.lower(), p0, p1,
                               bush_d, COL_MOUNT))

    def _balljoint(name: str, pt: Vec3):
        """A ball joint as a small sphere-ish stub (short vertical cylinder) at the
        outboard hardpoint."""
        p0 = (pt[0], pt[1], pt[2] - bj_d / 2.0)
        p1 = (pt[0], pt[1], pt[2] + bj_d / 2.0)
        steps.append(_axis_cyl(name, "ball_joint", "Ball_Joint_%s" % U.lower(), p0, p1,
                               bj_d, COL_MOUNT))

    _bushing("lower_bushing_fore_%s" % tag, P("lower_pickup_fore"),
             _sub(obj, P("lower_pickup_fore")))
    _bushing("lower_bushing_aft_%s" % tag, P("lower_pickup_aft"),
             _sub(obj, P("lower_pickup_aft")))
    _balljoint("lower_balljoint_%s" % tag, obj)
    if g.type in ("multilink", "double_wishbone"):
        obu = P("upper_ball_joint")
        _bushing("upper_bushing_fore_%s" % tag, P("upper_pickup_fore"),
                 _sub(obu, P("upper_pickup_fore")))
        _bushing("upper_bushing_aft_%s" % tag, P("upper_pickup_aft"),
                 _sub(obu, P("upper_pickup_aft")))
        _balljoint("upper_balljoint_%s" % tag, obu)

    # --- COIL SPRING + DAMPER (real inclination) ---------------------------- #
    # Both run from the lower spring/damper seat on the lower arm up to a body mount.
    seat = P("damper_lower")
    top = P("damper_top")
    coil_d = s.coil_outer_diameter_mm
    if g.type == "macpherson":
        # Coaxial strut: the damper is the upper link, running through the upright
        # top (strut_top), and the coil seats concentrically around it.
        strut_lo = P("lower_ball_joint")
        strut_hi = P("strut_top")
        steps.append(_axis_cyl(
            "damper_%s" % tag, "damper", "Strut_Damper_%s" % U, strut_lo, strut_hi,
            d.damper_diameter_mm, COL_DAMPER, material="damper_steel"))
        # coil seats around the strut over its sprung working length
        s_lo = (strut_lo[0], strut_lo[1], strut_lo[2] + 0.20 * (strut_hi[2] - strut_lo[2]))
        s_hi = (strut_lo[0] + 0.85 * (strut_hi[0] - strut_lo[0]),
                strut_lo[1] + 0.85 * (strut_hi[1] - strut_lo[1]),
                strut_lo[2] + 0.85 * (strut_hi[2] - strut_lo[2]))
        steps.append(_axis_tube(
            "spring_%s" % tag, "spring", "Coil_Spring_%s" % U, s_lo, s_hi,
            coil_d, coil_d - 24.0, COL_SPRING, material="spring_steel"))
    else:
        # Separate inclined coil-over (spring around the damper body) -- a typical
        # multilink/wishbone packaging: a single inclined unit on the lower arm.
        steps.append(_axis_cyl(
            "damper_%s" % tag, "damper", "Damper_%s" % U, seat, top,
            d.damper_diameter_mm, COL_DAMPER, material="damper_steel"))
        # spring around the lower 75 % of the damper run
        sp_hi = (seat[0] + 0.78 * (top[0] - seat[0]),
                 seat[1] + 0.78 * (top[1] - seat[1]),
                 seat[2] + 0.78 * (top[2] - seat[2]))
        steps.append(_axis_tube(
            "spring_%s" % tag, "spring", "Coil_Spring_%s" % U, seat, sp_hi,
            coil_d, coil_d - 24.0, COL_SPRING, material="spring_steel"))

    # --- ANTI-ROLL (STABILISER) DROP LINK ----------------------------------- #
    # The bar proper runs across the axle inboard; this corner carries the vertical
    # drop link from the lower arm up to the bar end. Modelled at its true near-
    # vertical inclination.
    if a.enabled:
        steps.append(_axis_cyl(
            "antiroll_%s" % tag, "anti_roll_bar", "Anti_Roll_Link_%s" % U,
            P("arb_link_lower"), P("arb_link_upper"), a.bar_diameter_mm, COL_ARB,
            material="bar_steel"))
    return steps


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
