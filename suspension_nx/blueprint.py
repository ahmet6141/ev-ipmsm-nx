"""Pure-math corner-suspension geometry, emitted as the SAME ordered build-step
list the NX builder consumes (motor_nx.blueprint schema). NX-independent +
unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(tube / cylinder / extrude + boolean create/subtract/unite), so motor_nx's
hardened NXOpen engine builds a suspension corner with no new geometry code.

Coordinate convention (LOCAL corner frame)
    X = vehicle longitudinal, Y = lateral (outboard = +Y toward the wheel),
    Z = vertical (up). The knuckle / upright sits outboard (high +Y); the
    control-arm chassis pickups sit inboard (near Y = 0).

BUILDER PRIMITIVE LIMITATION (and the simplification it forces)
    The reused builder only EXTRUDES along +Z and revolves about Z; cylinders and
    tubes are coaxial with +Z (optionally offset in XY via cx/cy). So nothing can
    be modelled as a beam lying along the Y axis directly. Following driveline's
    "production-representative BLANK" philosophy (gear blanks at pitch diameter),
    the control arms are represented as THIN EXTRUDED BOXES lying in horizontal
    (XY) planes at their respective Z heights -- a buildable, sensible blank that
    reads as an arm spanning inboard pickup -> outboard knuckle. The coil spring is
    a +Z tube blank, the damper / anti-roll bar / bushings / ball joints are +Z
    cylinders. Physical exactness (true 3D link axes, joint articulation) is traded
    for a BUILDABLE representation, exactly as driveline does for its gear teeth.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any, Dict, List

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import SuspensionParams

# component colours (RGB 0-255)
COL_KNUCKLE = (95, 100, 110)
COL_ARM = (120, 124, 132)
COL_SPRING = (150, 90, 60)
COL_DAMPER = (70, 110, 160)
COL_ARB = (140, 130, 90)
COL_MOUNT = (60, 62, 70)
COL_AIR = (0, 0, 0)


def _arm_box(arm_id: str, role: str, body_name: str, y_in: float, y_out: float,
             z_plane: float, width: float, thickness: float, sign: int) -> BuildStep:
    """A control arm as a thin extruded box lying in a horizontal (XY) plane.

    The box spans inboard (y_in) -> outboard (y_out) in Y and is centred on X, with
    a small longitudinal width. `sign` mirrors the corner about the X-Z plane
    (negate Y) for the second corner of an axle. Extruded from z_plane along +Z by
    `thickness` (the only axis the builder extrudes along)."""
    ya, yb = sign * y_in, sign * y_out
    ylo, yhi = (ya, yb) if ya <= yb else (yb, ya)
    hw = width / 2.0
    profile = [(-hw, ylo), (hw, ylo), (hw, yhi), (-hw, yhi)]
    return BuildStep(
        id=arm_id, role=role, kind="extrude", boolean="create",
        body_name=body_name, material="arm_steel", color=COL_ARM,
        profile=profile, z0=z_plane, length=thickness)


def _mount(mount_id: str, role: str, body_name: str, cx: float, cy: float,
           z0: float, diameter: float, length: float, sign: int) -> BuildStep:
    """A small +Z cylinder representing a bushing or ball-joint mount point."""
    return BuildStep(
        id=mount_id, role=role, kind="cylinder", boolean="create",
        body_name=body_name, material="joint_steel", color=COL_MOUNT,
        outer_radius=diameter / 2.0, cx=cx, cy=sign * cy, z0=z0, length=length)


# --------------------------------------------------------------------------- #
# one corner
# --------------------------------------------------------------------------- #
def corner_steps(p: SuspensionParams, tag: str, sign: int) -> List[BuildStep]:
    """Build one corner. `sign` = +1 (reference corner) or -1 (mirrored about X-Z)."""
    g, s, d = p.geometry, p.spring, p.damper
    a, k, arm = p.antiroll, p.knuckle, p.arm
    steps: List[BuildStep] = []
    U = tag.upper()

    # outboard knuckle / upright: an extruded box centred near the wheel (high +Y),
    # standing up the local Z by `height_mm` about the wheel-centre (ride height).
    y_knuckle = g.track_width_mm / 2.0
    z_hub = g.ride_height_mm
    hw_x = k.thickness_mm / 2.0          # longitudinal half-width of the upright
    hw_y = k.width_mm / 2.0              # lateral half-width of the upright
    yc = sign * y_knuckle
    knuckle_profile = [
        (-hw_x, yc - hw_y), (hw_x, yc - hw_y),
        (hw_x, yc + hw_y), (-hw_x, yc + hw_y),
    ]
    kid = "knuckle_%s" % tag
    steps.append(BuildStep(
        id=kid, role="knuckle", kind="extrude", boolean="create",
        body_name="Knuckle_Upright_%s" % U, material="cast_al", color=COL_KNUCKLE,
        profile=knuckle_profile, z0=z_hub - k.height_mm / 2.0, length=k.height_mm))
    # wheel-hub bearing bore through the upright (matches the Gen-3 hub OD). The bore
    # is a +Z cylinder through the block (a representative blank bore).
    steps.append(BuildStep(
        id="knuckle_hub_bore_%s" % tag, role="hub_bore_cut", kind="cylinder",
        boolean="subtract", target=kid, body_name="Hub_Bore_%s" % U,
        material="air", color=COL_AIR, outer_radius=k.hub_bore_diameter_mm / 2.0,
        cx=0.0, cy=yc, z0=z_hub - k.height_mm / 2.0 - 0.5, length=k.height_mm + 1.0))
    # brake-caliper mount lug (a small raised boss united to the upright)
    if k.brake_caliper_mount:
        steps.append(BuildStep(
            id="caliper_mount_%s" % tag, role="caliper_mount", kind="cylinder",
            boolean="unite", target=kid, body_name="Caliper_Mount_%s" % U,
            material="cast_al", color=COL_KNUCKLE, outer_radius=18.0,
            cx=hw_x, cy=yc, z0=z_hub + k.height_mm / 4.0, length=k.thickness_mm))

    # control arms as thin horizontal box blanks from inboard pickup -> knuckle.
    # Inboard pickups sit a short distance off the centre-plane; outboard ends reach
    # the knuckle face. Each arm sits at its own Z plane.
    inboard_y = max(20.0, y_knuckle - g.lower_arm_length_mm)
    arm_t = max(8.0, arm.arm_diameter_mm)          # box thickness ~ the bar diameter
    arm_w = max(20.0, arm.arm_diameter_mm + 12.0)  # longitudinal box width

    # lower control arm (low Z)
    steps.append(_arm_box(
        "lower_arm_%s" % tag, "lower_arm", "Lower_Control_Arm_%s" % U,
        inboard_y, y_knuckle - hw_y, z_hub - k.height_mm / 2.0,
        arm_w, arm_t, sign))
    # upper arm (high Z) -- only multilink / double_wishbone carry a real upper arm;
    # MacPherson uses the strut as the upper link (modelled by the damper below).
    upper_in_y = max(20.0, y_knuckle - g.upper_arm_length_mm)
    if g.type in ("multilink", "double_wishbone"):
        steps.append(_arm_box(
            "upper_arm_%s" % tag, "upper_arm", "Upper_Control_Arm_%s" % U,
            upper_in_y, y_knuckle - hw_y, z_hub + k.height_mm / 2.0 - arm_t,
            arm_w, arm_t, sign))
    # toe / tie link (mid Z, set rearward in X)
    toe_in_y = max(20.0, y_knuckle - g.toe_link_length_mm)
    steps.append(BuildStep(
        id="toe_link_%s" % tag, role="toe_link", kind="extrude", boolean="create",
        body_name="Toe_Link_%s" % U, material="arm_steel", color=COL_ARM,
        profile=_toe_profile(toe_in_y, y_knuckle - hw_y, arm_w, sign),
        z0=z_hub - arm_t / 2.0, length=arm_t))

    # inboard compliance bushings + outboard ball joints (small +Z cylinders)
    bj_len = max(10.0, arm.ball_joint_diameter_mm)
    bh_len = max(12.0, arm.bushing_diameter_mm / 2.0)
    steps.append(_mount("lower_bushing_%s" % tag, "bushing", "Lower_Arm_Bushing_%s" % U,
                        0.0, inboard_y, z_hub - k.height_mm / 2.0 - bh_len / 2.0,
                        arm.bushing_diameter_mm, bh_len, sign))
    steps.append(_mount("lower_balljoint_%s" % tag, "ball_joint", "Lower_Ball_Joint_%s" % U,
                        0.0, y_knuckle - hw_y, z_hub - k.height_mm / 2.0 - bj_len / 2.0,
                        arm.ball_joint_diameter_mm, bj_len, sign))
    if g.type in ("multilink", "double_wishbone"):
        steps.append(_mount("upper_bushing_%s" % tag, "bushing", "Upper_Arm_Bushing_%s" % U,
                            0.0, upper_in_y, z_hub + k.height_mm / 2.0 - bh_len,
                            arm.bushing_diameter_mm, bh_len, sign))
        steps.append(_mount("upper_balljoint_%s" % tag, "ball_joint", "Upper_Ball_Joint_%s" % U,
                            0.0, y_knuckle - hw_y, z_hub + k.height_mm / 2.0 - bj_len,
                            arm.ball_joint_diameter_mm, bj_len, sign))

    # coil spring as a hollow tube blank, standing up the local Z just inboard of
    # the knuckle (a representative spring seat position).
    spring_y = y_knuckle - g.upper_arm_length_mm / 2.0
    coil_r = s.coil_outer_diameter_mm / 2.0
    steps.append(BuildStep(
        id="spring_%s" % tag, role="spring", kind="tube", boolean="create",
        body_name="Coil_Spring_%s" % U, material="spring_steel", color=COL_SPRING,
        outer_radius=coil_r, inner_radius=max(2.0, coil_r - 12.0),
        cx=0.0, cy=sign * spring_y, z0=z_hub - k.height_mm / 2.0,
        length=s.free_length_mm))

    # damper as a +Z cylinder beside the spring (the strut for MacPherson)
    damper_y = spring_y - s.coil_outer_diameter_mm / 2.0 - d.damper_diameter_mm / 2.0 - 10.0
    if g.type == "macpherson":
        damper_y = spring_y  # coaxial strut: damper inside the spring envelope
    steps.append(BuildStep(
        id="damper_%s" % tag, role="damper", kind="cylinder", boolean="create",
        body_name="Damper_%s" % U, material="damper_steel", color=COL_DAMPER,
        outer_radius=d.damper_diameter_mm / 2.0, cx=0.0, cy=sign * damper_y,
        z0=z_hub - k.height_mm / 2.0, length=d.damper_length_mm))

    # anti-roll bar drop-link stub as a +Z cylinder near the lower arm (the bar
    # proper runs across the axle; here we represent the corner's drop link).
    if a.enabled:
        steps.append(BuildStep(
            id="antiroll_%s" % tag, role="anti_roll_bar", kind="cylinder",
            boolean="create", body_name="Anti_Roll_Link_%s" % U, material="bar_steel",
            color=COL_ARB, outer_radius=a.bar_diameter_mm / 2.0, cx=0.0,
            cy=sign * (inboard_y + a.arm_length_mm / 2.0),
            z0=z_hub - k.height_mm / 2.0, length=a.arm_length_mm))
    return steps


def _toe_profile(y_in: float, y_out: float, width: float, sign: int):
    """Toe-link box profile, offset rearward in X so it does not overlap the arms."""
    ya, yb = sign * y_in, sign * y_out
    ylo, yhi = (ya, yb) if ya <= yb else (yb, ya)
    x0 = -width * 1.5
    return [(x0, ylo), (x0 + width, ylo), (x0 + width, yhi), (x0, yhi)]


def _corners(p: SuspensionParams):
    """Yield (tag, sign) for each modelled corner."""
    if p.corners == "axle":
        return [("r", +1), ("l", -1)]
    return [("r", +1)]


def build_steps(p: SuspensionParams) -> List[BuildStep]:
    steps: List[BuildStep] = []
    for tag, sign in _corners(p):
        steps.extend(corner_steps(p, tag, sign))
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
        "validation": engineering.validate(p),
        "build_steps": [step.as_dict() for step in steps],
    }


def to_json(blueprint: Dict[str, Any], indent: int = 2) -> str:
    import json
    return json.dumps(blueprint, indent=indent)
