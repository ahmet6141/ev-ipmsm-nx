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
    origin = (axle_x, 0, 0). The hardpoint table (params._LEFT_HARDPOINTS_LOCAL) and
    the pad accessors are already in this local frame, so the geometry reads straight
    off params.hardpoints_local() / params.pad_centre_local().

The cradle, built directly in this frame
    * PERIMETER CRADLE: a closed rectangular ladder of hollow box beams (kind="prism").
      Two SIDE RAILS run fore/aft (±X) at each chassis-pad |Y|; a FRONT and a REAR
      CROSSBEAM run laterally (±Y) tying the side rails. The perimeter sits at the
      cradle base plane (low, near the lower-pickup Z band).
    * CHASSIS-PAD POSTS: four vertical (+Z) riser posts carry the cradle UP from the
      base plane to the chassis subframe mount pads (y=±585, z≈400 at the axle station,
      ICD §7.2), each capped by a bolt-flange + bolt circle that matches the chassis
      Subframe_Boss. This is the chassis<-subframe bolted joint.
    * SUSPENSION PICKUP BOSSES: a bored cylindrical boss at every inboard hardpoint
      (lower fore/aft, upper fore/aft, toe), both sides, axis fore/aft (±X) -- the
      control-arm / toe-link inboard ends bolt here (no floating arm, ICD §7.4.2).
    * SHOCK TOWERS: a vertical (+Z) post per side rising to the damper/strut top, capped
      by a seat plate with a rod bore + top-mount bolt circle (no floating spring).
    * E-AXLE / DIFF MOUNTS: bosses under the cradle straddling the centre plane to carry
      the gearbox + differential carrier (ICD §7.1/§7.2).

Hollow box sections (same construction as chassis_nx)
    Every box beam = an OUTER prism (create) plus a slightly smaller CONCENTRIC inner
    prism (subtract) leaving a `wall`-thick wall; the two share origin3/axis/u_dir and
    only the (u, v) profile shrinks, so the bore stays concentric.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import SubframeParams

Vec3 = Tuple[float, float, float]

# component colours (RGB 0-255)
COL_CRADLE = (120, 128, 140)
COL_POST = (150, 150, 90)
COL_BOSS = (90, 140, 110)
COL_TOWER = (150, 110, 80)
COL_EAXLE = (110, 116, 128)
COL_AIR = (0, 0, 0)


# --------------------------------------------------------------------------- #
# 2D section profiles (local u, v) -- centred on the prism origin
# --------------------------------------------------------------------------- #
def _rect_uv(half_u: float, half_v: float) -> List[Tuple[float, float]]:
    """Closed (u, v) rectangle centred on the origin, CCW."""
    return [(-half_u, -half_v), (half_u, -half_v),
            (half_u, half_v), (-half_u, half_v)]


def _unit(v: Vec3) -> Vec3:
    n = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5 or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _box_beam(steps: List[BuildStep], bid: str, role: str, body_name: str,
              origin3: Vec3, axis: Vec3, u_dir: Vec3, length: float,
              sec_u: float, sec_v: float, wall: float, color) -> str:
    """Append a hollow box beam as a prism in vehicle coordinates: an outer rectangle
    prism (create) extruded along `axis` by `length` from `origin3`, plus a slightly
    smaller CONCENTRIC inner rectangle prism (subtract) leaving a `wall`-thick wall.
    The section is `sec_u` (along local +u) by `sec_v` (along local +v); the inner
    prism over-runs the ends by 1 mm so the bore is a guaranteed through-cut. Returns
    the create id so callers can boolean further features onto the beam. (Identical
    construction to chassis_nx._box_beam.)"""
    steps.append(BuildStep(
        id=bid, role=role, kind="prism", boolean="create",
        body_name=body_name, material="aluminium", color=color,
        profile=_rect_uv(sec_u / 2.0, sec_v / 2.0),
        origin3=origin3, axis=axis, u_dir=u_dir, length=length))
    iu, iv = sec_u - 2.0 * wall, sec_v - 2.0 * wall
    if iu > 0 and iv > 0:
        ax_n = _unit(axis)
        o_in = (origin3[0] - 0.5 * ax_n[0],
                origin3[1] - 0.5 * ax_n[1],
                origin3[2] - 0.5 * ax_n[2])
        steps.append(BuildStep(
            id="%s_hollow" % bid, role="%s_hollow_cut" % role, kind="prism",
            boolean="subtract", target=bid, body_name="%s_Hollow" % body_name,
            material="air", color=COL_AIR,
            profile=_rect_uv(iu / 2.0, iv / 2.0),
            origin3=o_in, axis=axis, u_dir=u_dir, length=length + 1.0))
    return bid


# --------------------------------------------------------------------------- #
# perimeter cradle -- a closed rectangular ladder of box beams (kind="prism")
# --------------------------------------------------------------------------- #
def cradle_steps(p: SubframeParams) -> List[BuildStep]:
    c, pad = p.cradle, p.pad
    steps: List[BuildStep] = []
    cz = c.base_plane_z_mm
    rail_cy = pad.pad_y_mm                       # side rails sit under the chassis pads
    half_len = c.side_rail_length_mm / 2.0

    # two SIDE RAILS running fore/aft (+X), one under each chassis-pad Y. Section (u,v)
    # lays out in (Y, Z): u = +Y spans the beam width, v = +Z spans the beam height.
    for tag, sign in (("l", +1.0), ("r", -1.0)):
        _box_beam(
            steps, "cradle_side_%s" % tag, "cradle_rail", "Cradle_Side_%s" % tag.upper(),
            origin3=(-half_len, sign * rail_cy, cz),
            axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
            length=c.side_rail_length_mm,
            sec_u=c.beam_width_mm, sec_v=c.beam_height_mm, wall=c.beam_wall_mm,
            color=COL_CRADLE)

    # FRONT (+X end) and REAR (-X end) CROSSBEAMS running laterally (+Y) that bridge the
    # two side rails. They span the full pad-to-pad lateral width plus a small embed so
    # the ends land inside the side-rail boxes. Section (u,v) -> (Z, X): u = +Z height,
    # v = +X width. Vertically centred on the side rails so they tie the rail webs.
    embed = 0.5 * c.beam_width_mm
    cross_span = 2.0 * rail_cy + 2.0 * embed
    for tag, x_end in (("front", +half_len), ("rear", -half_len)):
        _box_beam(
            steps, "cradle_cross_%s" % tag, "cradle_cross", "Cradle_Cross_%s" % tag.upper(),
            origin3=(x_end, -(rail_cy + embed), cz),
            axis=(0.0, 1.0, 0.0), u_dir=(0.0, 0.0, 1.0),
            length=cross_span,
            sec_u=c.beam_height_mm, sec_v=c.beam_width_mm, wall=c.beam_wall_mm,
            color=COL_CRADLE)

    # INBOARD PICKUP STRINGERS (review finding 5): a fore/aft (+X) box beam per side at the
    # pickup |Y| band, running the full side-rail length so its ends embed into the front +
    # rear crossbeams. The suspension pickup-boss legs unite onto THIS stringer, so the
    # bosses are physically carried into the perimeter (pickup -> boss -> leg -> stringer ->
    # crossbeam -> side rail -> pad -> chassis) instead of floating inboard of the side
    # rails. United onto the front crossbeam so the whole perimeter is one solid.
    for tag, sign in (("l", +1.0), ("r", -1.0)):
        sid = "cradle_stringer_%s" % tag
        steps.append(BuildStep(
            id=sid, role="cradle_stringer", kind="prism", boolean="create",
            body_name="Cradle_Stringer_%s" % tag.upper(), material="aluminium", color=COL_CRADLE,
            profile=_rect_uv(c.stringer_width_mm / 2.0, c.beam_height_mm / 2.0),
            origin3=(-half_len, sign * c.stringer_y_mm, cz),
            axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
            length=c.side_rail_length_mm))
        # hollow it like the other box beams (concentric inner prism)
        iu = c.stringer_width_mm - 2.0 * c.beam_wall_mm
        iv = c.beam_height_mm - 2.0 * c.beam_wall_mm
        if iu > 0 and iv > 0:
            steps.append(BuildStep(
                id="%s_hollow" % sid, role="cradle_stringer_hollow_cut", kind="prism",
                boolean="subtract", target=sid,
                body_name="Cradle_Stringer_%s_Hollow" % tag.upper(),
                material="air", color=COL_AIR,
                profile=_rect_uv(iu / 2.0, iv / 2.0),
                origin3=(-half_len - 0.5, sign * c.stringer_y_mm, cz),
                axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
                length=c.side_rail_length_mm + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# chassis-pad riser posts -- carry the cradle UP to the rail-top mount pads
# --------------------------------------------------------------------------- #
def pad_steps(p: SubframeParams) -> List[BuildStep]:
    c, pad = p.cradle, p.pad
    steps: List[BuildStep] = []
    base_z = c.base_plane_z_mm
    post_h = max(0.0, pad.pad_z_mm - base_z)

    for fore_aft in ("fore", "aft"):
        for side in ("l", "r"):
            px, py, pz = p.pad_centre_local(fore_aft, side)
            pid = "pad_post_%s_%s" % (fore_aft, side)
            # riser post: a +Z cylinder from the cradle base plane up to the pad face.
            # United into the side rail it sits on (so the load runs into the perimeter).
            tgt = "cradle_side_%s" % side
            steps.append(BuildStep(
                id=pid, role="pad_post", kind="cylinder", boolean="unite", target=tgt,
                body_name="Pad_Post_%s_%s" % (fore_aft.upper(), side.upper()),
                material="aluminium", color=COL_POST,
                outer_radius=pad.post_diameter_mm / 2.0, length=post_h,
                origin3=(px, py, base_z), axis=(0.0, 0.0, 1.0)))
            # bolt-flange disc on the pad face (the bolted interface to the chassis boss)
            fid = "pad_flange_%s_%s" % (fore_aft, side)
            steps.append(BuildStep(
                id=fid, role="pad_flange", kind="cylinder", boolean="unite", target=tgt,
                body_name="Pad_Flange_%s_%s" % (fore_aft.upper(), side.upper()),
                material="aluminium", color=COL_POST,
                outer_radius=pad.flange_diameter_mm / 2.0, length=pad.flange_thickness_mm,
                origin3=(px, py, pz), axis=(0.0, 0.0, 1.0)))
            # bolt circle drilled UP (+Z) through the flange (clearance for the bolts
            # that fasten to the chassis Subframe_Boss above).
            if pad.bolt_count > 0 and pad.bolt_diameter_mm > 0:
                pcd_r = max(pad.bolt_diameter_mm,
                            pad.flange_diameter_mm / 2.0 - max(pad.bolt_diameter_mm, 6.0))
                for k in range(pad.bolt_count):
                    ang = 2.0 * math.pi * k / pad.bolt_count
                    hx = px + pcd_r * math.cos(ang)
                    hy = py + pcd_r * math.sin(ang)
                    steps.append(BuildStep(
                        id="pad_bolt_%s_%s_%d" % (fore_aft, side, k),
                        role="pad_bolt_cut", kind="hole", boolean="subtract", target=tgt,
                        body_name="Pad_Bolt_%s_%s_%d" % (fore_aft.upper(), side.upper(), k),
                        material="air", color=COL_AIR,
                        outer_radius=pad.bolt_diameter_mm / 2.0,
                        cx=hx, cy=hy, z0=pz - 0.5,
                        axis=(0.0, 0.0, 1.0), length=pad.flange_thickness_mm + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# suspension inboard pickup bosses -- bored bosses at the hardpoint coordinates
# --------------------------------------------------------------------------- #
# the inboard hardpoints that get a pickup boss (the upright/ball-joint hardpoints
# belong to the suspension, not the subframe).
_PICKUP_NAMES = ("lower_pickup_fore", "lower_pickup_aft",
                 "upper_pickup_fore", "upper_pickup_aft", "toe_pickup")


def boss_steps(p: SubframeParams) -> List[BuildStep]:
    b = p.boss
    steps: List[BuildStep] = []
    half = b.boss_length_mm / 2.0
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        # a leg from the cradle perimeter up to each pickup, then the bored boss. The
        # boss axis is fore/aft (+X): the control-arm bushing pin runs longitudinally.
        for nm in _PICKUP_NAMES:
            x, y, z = hp[nm]
            bid = "pickup_boss_%s_%s" % (nm, side)
            # the boss is created centred on the hardpoint, axis +X (start half-length
            # back so its mid-point IS the hardpoint).
            steps.append(BuildStep(
                id=bid, role="pickup_boss", kind="cylinder", boolean="create",
                body_name="Pickup_Boss_%s_%s" % (nm.upper(), side.upper()),
                material="aluminium", color=COL_BOSS,
                outer_radius=b.boss_diameter_mm / 2.0, length=b.boss_length_mm,
                origin3=(x - half, y, z), axis=(1.0, 0.0, 0.0)))
            # cross bore through the boss along its axis (pin / bushing-bolt clearance)
            steps.append(BuildStep(
                id="%s_bore" % bid, role="pickup_bore_cut", kind="cylinder",
                boolean="subtract", target=bid,
                body_name="Pickup_Bore_%s_%s" % (nm.upper(), side.upper()),
                material="air", color=COL_AIR,
                outer_radius=b.bore_diameter_mm / 2.0, length=b.boss_length_mm + 1.0,
                origin3=(x - half - 0.5, y, z), axis=(1.0, 0.0, 0.0)))
            # a tie LEG (a slim prism) from the INBOARD PICKUP STRINGER up to the boss, so
            # the boss is physically carried by the perimeter (no mid-air boss, review
            # finding 5). The leg starts a little BELOW the stringer centre so it runs
            # THROUGH the stringer box (guaranteeing solid overlap) and reaches up to the
            # pickup; section in (X, Y). It unites onto the boss (boss+leg one solid).
            stringer_centre_z = p.cradle.base_plane_z_mm
            leg_bottom = stringer_centre_z - p.cradle.beam_height_mm / 2.0
            leg_h = max(0.0, z - leg_bottom)
            half_sec = b.boss_diameter_mm / 2.0
            if leg_h > 1.0:
                steps.append(BuildStep(
                    id="%s_leg" % bid, role="pickup_leg", kind="prism", boolean="unite",
                    target=bid, body_name="Pickup_Leg_%s_%s" % (nm.upper(), side.upper()),
                    material="aluminium", color=COL_BOSS,
                    profile=_rect_uv(half_sec, half_sec),
                    origin3=(x, y, leg_bottom), axis=(0.0, 0.0, 1.0),
                    u_dir=(1.0, 0.0, 0.0), length=leg_h + 1.0))
                # TIE the boss+leg solid INTO the inboard stringer: a foot prism occupying
                # the leg's base volume inside the stringer, united onto the stringer. The
                # shared volume merges the boss/leg chain with the perimeter so the unite
                # leaves ONE solid (the load path pickup -> boss -> leg -> stringer ->
                # crossbeam -> rail -> pad -> chassis is closed, review finding 5).
                tgt_stringer = "cradle_stringer_%s" % side
                steps.append(BuildStep(
                    id="%s_foot" % bid, role="pickup_leg_tie", kind="prism", boolean="unite",
                    target=tgt_stringer,
                    body_name="Pickup_Foot_%s_%s" % (nm.upper(), side.upper()),
                    material="aluminium", color=COL_BOSS,
                    profile=_rect_uv(half_sec, half_sec),
                    origin3=(x, y, leg_bottom),
                    axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0),
                    length=p.cradle.beam_height_mm + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# shock / body towers -- a vertical post reaching the damper/strut top, per side
# --------------------------------------------------------------------------- #
def tower_steps(p: SubframeParams) -> List[BuildStep]:
    tw = p.tower
    steps: List[BuildStep] = []
    base_z = p.cradle.base_plane_z_mm
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        tx, ty, tz = hp["damper_top"]
        post_h = max(0.0, tz - base_z)
        tid = "tower_post_%s" % side
        # the tower post: a +Z cylinder from the cradle base plane to the damper top.
        steps.append(BuildStep(
            id=tid, role="shock_tower", kind="cylinder", boolean="create",
            body_name="Shock_Tower_%s" % side.upper(),
            material="aluminium", color=COL_TOWER,
            outer_radius=tw.post_diameter_mm / 2.0, length=post_h,
            origin3=(tx, ty, base_z), axis=(0.0, 0.0, 1.0)))
        # top seat plate (upper spring seat / damper top-mount), capping the post AT the
        # damper-top hardpoint (its lower face sits at tz, so the top mount lands there).
        sid = "tower_seat_%s" % side
        steps.append(BuildStep(
            id=sid, role="tower_seat", kind="cylinder", boolean="unite", target=tid,
            body_name="Tower_Seat_%s" % side.upper(),
            material="aluminium", color=COL_TOWER,
            outer_radius=tw.seat_diameter_mm / 2.0, length=tw.seat_thickness_mm,
            origin3=(tx, ty, tz), axis=(0.0, 0.0, 1.0)))
        # damper-rod / top-mount clearance bore down through the seat + into the post
        steps.append(BuildStep(
            id="%s_bore" % sid, role="tower_bore_cut", kind="cylinder",
            boolean="subtract", target=tid,
            body_name="Tower_Bore_%s" % side.upper(), material="air", color=COL_AIR,
            outer_radius=tw.rod_bore_diameter_mm / 2.0,
            length=tw.seat_thickness_mm + 30.0,
            origin3=(tx, ty, tz - 30.0), axis=(0.0, 0.0, 1.0)))
        # top-mount bolt circle through the seat plate (fastens the damper top mount)
        if tw.bolt_count > 0 and tw.bolt_diameter_mm > 0:
            pcd_r = max(tw.bolt_diameter_mm,
                        tw.seat_diameter_mm / 2.0 - max(tw.bolt_diameter_mm, 6.0))
            for k in range(tw.bolt_count):
                ang = 2.0 * math.pi * k / tw.bolt_count
                hx = tx + pcd_r * math.cos(ang)
                hy = ty + pcd_r * math.sin(ang)
                steps.append(BuildStep(
                    id="%s_bolt_%d" % (sid, k), role="tower_bolt_cut", kind="hole",
                    boolean="subtract", target=tid,
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
    ea = p.eaxle
    if not ea.enabled:
        return []
    steps: List[BuildStep] = []
    # mounts straddle the centre plane (±Y) at the diff-carrier height, spread fore/aft
    # so they bracket the diff. mount_count mounts per side-pair, fore/aft about x=0.
    n = max(1, ea.mount_count)
    for i in range(n):
        # spread the mounts symmetrically fore/aft within the cradle X span
        if n == 1:
            mx = 0.0
        else:
            frac = (i + 0.5) / n
            mx = (-0.5 + frac) * (p.cradle.side_rail_length_mm * 0.6)
        for side, sign in (("l", +1.0), ("r", -1.0)):
            my = sign * ea.mount_y_mm
            bid = "eaxle_mount_%d_%s" % (i, side)
            # a +Z boss rising from the carrier height; the diff/gearbox bolts on top.
            steps.append(BuildStep(
                id=bid, role="eaxle_mount", kind="cylinder", boolean="create",
                body_name="EAxle_Mount_%d_%s" % (i, side.upper()),
                material="aluminium", color=COL_EAXLE,
                outer_radius=ea.boss_diameter_mm / 2.0, length=ea.boss_height_mm,
                origin3=(mx, my, ea.mount_z_mm), axis=(0.0, 0.0, 1.0)))
            # carrier bolt bore down the boss axis
            steps.append(BuildStep(
                id="%s_bore" % bid, role="eaxle_bolt_cut", kind="cylinder",
                boolean="subtract", target=bid,
                body_name="EAxle_Bolt_%d_%s" % (i, side.upper()),
                material="air", color=COL_AIR,
                outer_radius=ea.bolt_diameter_mm / 2.0, length=ea.boss_height_mm + 1.0,
                origin3=(mx, my, ea.mount_z_mm - 0.5), axis=(0.0, 0.0, 1.0)))
    return steps


def build_steps(p: SubframeParams) -> List[BuildStep]:
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
