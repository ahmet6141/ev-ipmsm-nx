"""Pure-math chassis geometry, emitted as the SAME ordered build-step list the NX
builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(tube / cylinder / extrude / revolve / hole + boolean create/subtract/unite),
so motor_nx's hardened NXOpen engine builds a chassis with no new geometry code.

The +Z extrude abstraction
    The proven builder extrudes a closed XY polygon from z0 along +Z by `length`.
    A chassis is an assembly of long box beams pointing in DIFFERENT directions
    (rails along X, crossmembers along Y), but the builder only extrudes along +Z.
    So we adopt a deliberate, BUILDABLE abstraction: every box beam is drawn as a
    +Z extrude whose extrude direction (+Z) is THE BEAM'S OWN LENGTH, positioned in
    the XY plane by its (cx, cy) and stacked by z0. In other words "+Z is the long
    axis of whatever beam I am currently drawing". The result is a valid solid
    assembly of box beams; rotating each beam into its true vehicle orientation
    (rails -> X, crossmembers -> Y) is a downstream NX transform and is NOT done
    here. We keep it buildable above all.

    Each hollow box beam = an outer rectangle extrude (create) + a slightly smaller
    concentric rectangle extrude (subtract) leaving the wall. The battery tray is
    one big hollow box + a few crossbrace boxes. Subframe bosses are small cylinders
    united to a beam; body-mount + subframe-bolt holes are small cylinders subtracted.

Coordinate convention (as drawn here -- a buildable abstraction, not the vehicle frame)
    Z  = the long axis of the beam currently being extruded.
    XY = the placement plane that lays the beams out side by side.
"""

from __future__ import annotations

from typing import Any, Dict, List

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import ChassisParams

# component colours (RGB 0-255)
COL_RAIL = (130, 135, 145)
COL_CROSS = (110, 116, 128)
COL_TRAY = (80, 120, 160)
COL_BRACE = (95, 130, 165)
COL_BOSS = (150, 150, 90)
COL_CRUSH = (170, 120, 80)
COL_AIR = (0, 0, 0)


def _rect(cx: float, cy: float, w: float, h: float):
    """Closed XY rectangle (w x h) centred at (cx, cy), CCW."""
    hw, hh = w / 2.0, h / 2.0
    return [(cx - hw, cy - hh), (cx + hw, cy - hh),
            (cx + hw, cy + hh), (cx - hw, cy + hh)]


def _box_beam(steps: List[BuildStep], bid: str, role: str, body_name: str,
              cx: float, cy: float, w: float, h: float, wall: float,
              z0: float, length: float, color):
    """Append a hollow box beam: an outer-rectangle extrude (create) along +Z plus
    a concentric inner-rectangle subtract leaving a `wall`-thick wall. Returns the
    create id so callers can boolean further features onto it."""
    steps.append(BuildStep(
        id=bid, role=role, kind="extrude", boolean="create",
        body_name=body_name, material="aluminium", color=color,
        profile=_rect(cx, cy, w, h), z0=z0, length=length))
    iw, ih = w - 2.0 * wall, h - 2.0 * wall
    if iw > 0 and ih > 0:
        steps.append(BuildStep(
            id="%s_hollow" % bid, role="%s_hollow_cut" % role, kind="extrude",
            boolean="subtract", target=bid, body_name="%s_Hollow" % body_name,
            material="air", color=COL_AIR,
            profile=_rect(cx, cy, iw, ih), z0=z0 - 0.5, length=length + 1.0))
    return bid


# --------------------------------------------------------------------------- #
# frame: two longitudinal rails + lateral crossmembers
# --------------------------------------------------------------------------- #
def frame_steps(p: ChassisParams) -> List[BuildStep]:
    f = p.frame
    steps: List[BuildStep] = []

    # lateral centre of each rail (in the placement plane): the rails sit on either
    # side of the inner channel, so their centres are inner_width/2 + rail_width/2.
    rail_cx = f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0

    # two longitudinal rails -- each extruded along +Z by the overall length.
    for tag, sign in (("l", -1), ("r", +1)):
        _box_beam(steps, "rail_%s" % tag, "frame_rail", "Frame_Rail_%s" % tag.upper(),
                  cx=sign * rail_cx, cy=0.0,
                  w=f.rail_width_mm, h=f.rail_height_mm, wall=f.rail_wall_mm,
                  z0=0.0, length=f.overall_length_mm, color=COL_RAIL)

    # crossmembers tie the two rails -- each extruded along +Z by the inner spacing.
    # They are stacked in the placement plane along +X so they do not overlap; their
    # true vehicle orientation (along Y, distributed along the wheelbase) is a later
    # NX transform.
    n = max(0, f.crossmember_count)
    if n > 0:
        # distribute crossmembers along the wheelbase for placement bookkeeping
        span = f.wheelbase_mm
        for i in range(n):
            frac = (i + 0.5) / n
            # offset each crossmember's section centre in +Y so the parts are distinct
            cy = (i - (n - 1) / 2.0) * (f.crossmember_width_mm + 6.0)
            _box_beam(steps, "crossmember_%d" % i, "crossmember",
                      "Crossmember_%d" % i,
                      cx=rail_cx + f.rail_width_mm / 2.0 + f.crossmember_height_mm,
                      cy=cy,
                      w=f.crossmember_width_mm, h=f.crossmember_height_mm,
                      wall=f.crossmember_wall_mm,
                      z0=0.0, length=f.frame_inner_width_mm, color=COL_CROSS)
            # (frac/span retained as design intent; placement is abstracted above)
            _ = (frac, span)
    return steps


# --------------------------------------------------------------------------- #
# battery tray: one large hollow box + a few crossbraces
# --------------------------------------------------------------------------- #
def battery_tray_steps(p: ChassisParams) -> List[BuildStep]:
    b = p.battery_tray
    if not b.enabled:
        return []
    f = p.frame
    steps: List[BuildStep] = []

    # the tray is laid out below the rails in the placement plane (cy offset) and
    # extruded along +Z by its length.
    tray_cy = -(f.rail_height_mm + b.height_mm)   # park it clear of the rails
    _box_beam(steps, "battery_tray", "battery_tray", "Battery_Tray",
              cx=0.0, cy=tray_cy, w=b.width_mm, h=b.height_mm, wall=b.wall_mm,
              z0=0.0, length=b.length_mm, color=COL_TRAY)

    # crossbraces inside the tray -- thin lateral webs spanning the tray width,
    # spaced along the tray length (the +Z extrude axis).
    n = max(0, b.crossbrace_count)
    inner_w = b.width_mm - 2.0 * b.wall_mm
    inner_h = b.height_mm - 2.0 * b.wall_mm
    if n > 0 and inner_w > 0 and inner_h > 0:
        usable = b.length_mm - 2.0 * b.wall_mm
        for i in range(n):
            z = b.wall_mm + usable * (i + 0.5) / n - b.wall_mm / 2.0
            steps.append(BuildStep(
                id="battery_brace_%d" % i, role="battery_brace", kind="extrude",
                boolean="unite", target="battery_tray",
                body_name="Battery_Crossbrace_%d" % i, material="aluminium",
                color=COL_BRACE,
                profile=_rect(0.0, tray_cy, inner_w, inner_h),
                z0=z, length=b.wall_mm))
    return steps


# --------------------------------------------------------------------------- #
# subframe mount bosses + bolt holes, body mount holes, crush cans
# --------------------------------------------------------------------------- #
def mount_steps(p: ChassisParams) -> List[BuildStep]:
    f, s, bm = p.frame, p.subframe, p.body_mount
    steps: List[BuildStep] = []
    rail_cx = f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0
    boss_d = max(2.0 * s.mount_bolt_diameter_mm, 30.0)

    # subframe mount bosses: small cylinders united to a rail end, with a bolt-hole
    # circle subtracted. Front subframe bosses at the +Z end of each rail, rear at z0.
    def subframe_set(tag: str, z_boss: float):
        for side, sign in (("l", -1), ("r", +1)):
            rid = "rail_%s" % side
            bid = "subframe_boss_%s_%s" % (tag, side)
            steps.append(BuildStep(
                id=bid, role="subframe_boss", kind="cylinder", boolean="unite",
                target=rid, body_name="Subframe_Boss_%s_%s" % (tag.upper(), side.upper()),
                material="aluminium", color=COL_BOSS,
                outer_radius=boss_d / 2.0, cx=sign * rail_cx, cy=0.0,
                z0=z_boss, length=12.0))
            if s.mount_bolt_count > 0 and s.mount_bolt_diameter_mm > 0:
                steps.append(BuildStep(
                    id="subframe_bolt_%s_%s" % (tag, side), role="subframe_bolt_cut",
                    kind="cylinder", boolean="subtract", target=rid,
                    body_name="Subframe_Bolt_%s_%s" % (tag.upper(), side.upper()),
                    material="air", color=COL_AIR,
                    outer_radius=s.mount_bolt_diameter_mm / 2.0,
                    cx=sign * rail_cx + boss_d / 4.0, cy=0.0,
                    z0=z_boss - 0.5, length=12.0 + 1.0,
                    pattern_count=s.mount_bolt_count,
                    pattern_angle_deg=360.0 / s.mount_bolt_count))

    if s.front_subframe:
        subframe_set("front", f.overall_length_mm - 12.0)
    if s.rear_subframe:
        subframe_set("rear", 0.0)

    # body mount holes along each rail top (subtracted from the rails). Split the
    # count between the two rails.
    if bm.body_mount_count > 0 and bm.body_mount_diameter_mm > 0:
        per_rail = max(1, bm.body_mount_count // 2)
        for side, sign in (("l", -1), ("r", +1)):
            rid = "rail_%s" % side
            for i in range(per_rail):
                z = f.overall_length_mm * (i + 0.5) / per_rail
                steps.append(BuildStep(
                    id="body_mount_%s_%d" % (side, i), role="body_mount_cut",
                    kind="cylinder", boolean="subtract", target=rid,
                    body_name="Body_Mount_%s_%d" % (side.upper(), i),
                    material="air", color=COL_AIR,
                    outer_radius=bm.body_mount_diameter_mm / 2.0,
                    cx=sign * rail_cx, cy=f.rail_height_mm / 2.0 - 2.0,
                    z0=z, length=f.rail_width_mm))

    # crush cans: short box beams ahead of / behind the wheelbase on each rail.
    can_len = (f.overall_length_mm - f.wheelbase_mm) / 2.0
    if can_len > 0:
        if bm.crush_can_front:
            for side, sign in (("l", -1), ("r", +1)):
                _box_beam(steps, "crush_front_%s" % side, "crush_can",
                          "Crush_Can_Front_%s" % side.upper(),
                          cx=sign * rail_cx, cy=f.rail_height_mm + can_len,
                          w=f.rail_width_mm, h=f.rail_height_mm, wall=f.rail_wall_mm,
                          z0=0.0, length=can_len, color=COL_CRUSH)
        if bm.crush_can_rear:
            for side, sign in (("l", -1), ("r", +1)):
                _box_beam(steps, "crush_rear_%s" % side, "crush_can",
                          "Crush_Can_Rear_%s" % side.upper(),
                          cx=sign * rail_cx, cy=-(f.rail_height_mm + can_len),
                          w=f.rail_width_mm, h=f.rail_height_mm, wall=f.rail_wall_mm,
                          z0=0.0, length=can_len, color=COL_CRUSH)
    return steps


def build_steps(p: ChassisParams) -> List[BuildStep]:
    steps = frame_steps(p)
    steps.extend(battery_tray_steps(p))
    steps.extend(mount_steps(p))
    return steps


def generate(p: ChassisParams = None) -> Dict[str, Any]:
    """Full chassis blueprint dict (NX-independent). Same schema as driveline_nx /
    motor_nx so the same run_journal builder consumes it."""
    if p is None:
        p = ChassisParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "chassis_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": p.frame.overall_length_mm,   # representative beam length
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
