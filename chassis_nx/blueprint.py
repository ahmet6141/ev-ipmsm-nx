"""Pure-math chassis geometry, emitted as the SAME ordered build-step list the NX
builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(prism / tube / cylinder / extrude / revolve / hole + boolean create/subtract/unite),
so motor_nx's hardened NXOpen engine builds a chassis with no new geometry code.

Coordinate convention -- THE TRUE VEHICLE FRAME (ISO 8855, ICD §1)
    +X = forward, +Y = left, +Z = up.
    Origin = vehicle centre, mid-wheelbase, on the GROUND plane (z = 0).
    The chassis package's local frame IS the vehicle frame, so the assembly places
    it with the identity transform (ICD §3) -- every coordinate written here is a
    vehicle coordinate.

The skateboard platform, built DIRECTLY in vehicle coordinates
    * Two longitudinal RAILS are hollow box beams running along +X (kind="prism",
      axis=+X). Their centre-lines sit at y = ±(frame_inner_width/2 + rail_width/2)
      and they span the WHEELBASE centred on x = 0
      (x = -wheelbase/2 .. +wheelbase/2). The front/rear crush cans form the overhangs
      and butt onto the rail ends (so the cans are NOT buried inside the rails).
    * Lateral CROSSMEMBERS are hollow box beams running along +Y (axis=+Y) that
      PHYSICALLY BRIDGE the two rails: each spans the full inner width and embeds a
      little into both rails, distributed along the wheelbase in X.
    * The sealed structural BATTERY TRAY is a hollow box BETWEEN the rails, low in Z
      (the floor), centred on x = 0, with internal lateral crossbraces.
    * Front & rear SUBFRAME MOUNT PADS sit at the axle x-stations (x = ±wheelbase/2)
      on each rail -- a boss united to the rail plus a bolt-hole circle -- so the
      e-axle / suspension subframes bolt where the ICD HUB_CENTRE x lands.
    * BODY-MOUNT holes run along the rail tops; front/rear CRUSH CANS extend along
      +X beyond the wheelbase (coaxial with the rails).

Vertical (Z) datum -- a derived skateboard floor stack (no new params needed)
    GROUND_CLEARANCE  = floor (battery tray bottom) above the road.
    The battery tray occupies z = GROUND_CLEARANCE .. GROUND_CLEARANCE + tray_height.
    The rails sit ON TOP of the tray height band: rail bottom = tray top, so the rail
    centre-line is rail_cz = GROUND_CLEARANCE + tray_height + rail_height/2. With the
    default dimensions (clearance 140 + tray 110 + rail 120) the rail mid-height is
    rail_cz = 310 mm and the rail top is 370 mm; the subframe mount pads sit ON the
    rail top (370 mm), ~35 mm above the ICD hub-centre line (r = 335) -- the e-axle /
    suspension subframes bolt to the pad on the rail top at the axle x-stations.

Hollow sections
    Every box beam = an OUTER prism (create) plus a slightly smaller CONCENTRIC inner
    prism (subtract) leaving a `wall`-thick wall. The two prisms share origin3 / axis /
    u_dir; only the (u, v) profile shrinks by the wall, so the bore stays concentric.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import ChassisParams

Vec3 = Tuple[float, float, float]

# component colours (RGB 0-255)
COL_RAIL = (130, 135, 145)
COL_CROSS = (110, 116, 128)
COL_TRAY = (80, 120, 160)
COL_BRACE = (95, 130, 165)
COL_BOSS = (150, 150, 90)
COL_CRUSH = (170, 120, 80)
COL_AIR = (0, 0, 0)

# Skateboard floor stack (vertical datum, derived -- see module docstring). The
# battery tray bottom sits this far above the ground plane (z = 0). A typical
# passenger-EV skateboard floor clears the road by ~120-150 mm.
GROUND_CLEARANCE_MM = 140.0
# How far each crossmember end embeds INTO a rail (so the ends physically land
# inside the rail box rather than merely touching its inner face).
CROSSMEMBER_EMBED_MM = 6.0


# --------------------------------------------------------------------------- #
# vertical (Z) placement of the floor stack (derived from the params, no new fields)
# --------------------------------------------------------------------------- #
def _z_layout(p: ChassisParams) -> Dict[str, float]:
    """Z (vertical) stations of the platform, in vehicle coordinates (z = 0 ground).

    tray:  z = tray_bottom .. tray_top
    rails: bottom on the tray top, so rail_cz = tray_top + rail_height/2.
    When the battery tray is deleted the rails simply sit on the ground-clearance
    datum (rail bottom = GROUND_CLEARANCE)."""
    f, b = p.frame, p.battery_tray
    tray_bottom = GROUND_CLEARANCE_MM
    tray_h = b.height_mm if b.enabled else 0.0
    tray_top = tray_bottom + tray_h
    rail_bottom = tray_top
    rail_cz = rail_bottom + f.rail_height_mm / 2.0
    return {
        "tray_bottom": tray_bottom,
        "tray_top": tray_top,
        "tray_cz": tray_bottom + tray_h / 2.0,
        "rail_bottom": rail_bottom,
        "rail_cz": rail_cz,
        "rail_top": rail_cz + f.rail_height_mm / 2.0,
    }


def _axle_notch_band(p: ChassisParams) -> Dict[str, float]:
    """The axle-notch relief window geometry (Z band + X half-width), shared by the rail
    notch (frame_steps) and the crush-can notch (mount_steps) so the two halves of the
    window line up. The relief reaches the FULL rail height (notch_top >= rail top) over
    X = axle +- half_width."""
    f = p.frame
    z = _z_layout(p)
    notch_top = max(f.axle_notch_top_mm, z["rail_top"] + 1.0)   # relieve the FULL rail height
    notch_bottom = z["rail_bottom"] - 1.0                       # over-run below for a clean cut
    return {
        "half_width": f.axle_notch_half_width_mm,
        "top": notch_top,
        "bottom": notch_bottom,
        "h": max(0.0, notch_top - notch_bottom),
        "cz": 0.5 * (notch_bottom + notch_top),
    }


def rail_centreline_y(p: ChassisParams) -> float:
    """Lateral (Y) centre-line of the +Y rail (the -Y rail is its mirror): the rail
    sits just outboard of the inner channel, so its centre is
    frame_inner_width/2 + rail_width/2 (ICD §3)."""
    f = p.frame
    return f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0


def subframe_pad_centre(p: ChassisParams, axle: str, side: str) -> Vec3:
    """World (vehicle-frame) centre of a front/rear subframe mount pad face on the
    rail top. `axle` in {"front","rear"} -> x = ±wheelbase/2 (the ICD axle station);
    `side` in {"l","r"} -> y = ∓/± rail centre-line. z = rail top (boss face)."""
    f = p.frame
    z = _z_layout(p)
    x = +f.wheelbase_mm / 2.0 if axle == "front" else -f.wheelbase_mm / 2.0
    y = rail_centreline_y(p) * (-1.0 if side == "l" else +1.0)
    return (x, y, z["rail_top"])


# --------------------------------------------------------------------------- #
# 2D section profiles (local u, v) -- centred on the prism origin
# --------------------------------------------------------------------------- #
def _rect_uv(half_u: float, half_v: float) -> List[Tuple[float, float]]:
    """Closed (u, v) rectangle centred on the origin, CCW."""
    return [(-half_u, -half_v), (half_u, -half_v),
            (half_u, half_v), (-half_u, half_v)]


def _box_beam(steps: List[BuildStep], bid: str, role: str, body_name: str,
              origin3: Vec3, axis: Vec3, u_dir: Vec3, length: float,
              sec_u: float, sec_v: float, wall: float, color) -> str:
    """Append a hollow box beam as a prism in TRUE vehicle coordinates: an outer
    rectangle prism (create) extruded along `axis` by `length` from `origin3`, plus a
    slightly smaller CONCENTRIC inner rectangle prism (subtract) leaving a `wall`-thick
    wall. The section is `sec_u` (along local +u) by `sec_v` (along local +v); the
    inner prism over-runs the ends by 1 mm so the bore is a guaranteed through-cut.
    Returns the create id so callers can boolean further features onto the beam."""
    steps.append(BuildStep(
        id=bid, role=role, kind="prism", boolean="create",
        body_name=body_name, material="aluminium", color=color,
        profile=_rect_uv(sec_u / 2.0, sec_v / 2.0),
        origin3=origin3, axis=axis, u_dir=u_dir, length=length))
    iu, iv = sec_u - 2.0 * wall, sec_v - 2.0 * wall
    if iu > 0 and iv > 0:
        # nudge the inner-prism origin 0.5 mm back along the axis and over-run 1 mm so
        # the cut pierces both end faces cleanly (an open-ended hollow box section).
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


def _unit(v: Vec3) -> Vec3:
    n = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5 or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


# --------------------------------------------------------------------------- #
# frame: two longitudinal rails along +X + lateral crossmembers along +Y
# --------------------------------------------------------------------------- #
def frame_steps(p: ChassisParams) -> List[BuildStep]:
    f = p.frame
    z = _z_layout(p)
    steps: List[BuildStep] = []

    rail_cy = rail_centreline_y(p)
    # The rails span the WHEELBASE (axle-to-axle), centred on x = 0; the front/rear
    # CRUSH CANS form the overhangs and BUTT onto the rail ends (ICD §3 docstring:
    # "crush cans extend BEYOND the wheelbase ... butt onto the rail ends"). Ending the
    # rails at the wheelbase -- instead of running them the full overall_length -- is
    # what stops the crush cans being fully buried inside the rails (the adversarial-
    # review HIGH finding: two coincident solids in each overhang).
    x_start = -f.wheelbase_mm / 2.0

    # two longitudinal rails -- hollow box beams running along +X. The (u, v) section
    # lays out in (Y, Z): u = +Y spans rail_width, v = +Z spans rail_height.
    for tag, sign in (("l", -1.0), ("r", +1.0)):
        _box_beam(
            steps, "rail_%s" % tag, "frame_rail", "Frame_Rail_%s" % tag.upper(),
            origin3=(x_start, sign * rail_cy, z["rail_cz"]),
            axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
            length=f.wheelbase_mm,
            sec_u=f.rail_width_mm, sec_v=f.rail_height_mm, wall=f.rail_wall_mm,
            color=COL_RAIL)

    # AXLE NOTCH / relief: at each axle x-station the suspension control arms, toe link,
    # anti-roll link/damper AND the driveline half-shaft sweep through the rail's Y band
    # running from the inboard pickups out to the wheel hub, reaching the FULL rail height
    # and into the abutting crush-can overhang. Cut a FULL-SECTION clearance WINDOW
    # centred on the axle x-station -- relieving the rail (here) and the crush-can end (in
    # mount_steps, after the cans are created) over X = axle +- axle_notch_half_width up to
    # axle_notch_top_mm -- so the whole corner + half-shaft envelope passes through the open
    # axle bay. The rail stays structurally continuous through the battery tray +
    # crossmembers; the subframe pads sit just inboard of the window. Resolves the rail/arm
    # + rail/half-shaft + crush-can/arm collisions the NX inspection found. The INBOARD
    # half subtracts from the rail (this cut overlaps the rail solid, so NX leaves one
    # clean body).
    if f.axle_notch and f.axle_notch_half_width_mm > 0:
        nb = _axle_notch_band(p)
        for axle, axle_x in (("front", +f.wheelbase_mm / 2.0),
                             ("rear", -f.wheelbase_mm / 2.0)):
            for tag, sign in (("l", -1.0), ("r", +1.0)):
                cy = sign * rail_cy
                # INBOARD half -> relieve the RAIL (from the axle station toward x=0).
                # The rail spans [-wb/2, +wb/2]; the front axle is its +X end and the rear
                # axle its -X end, so the inboard direction is -X (front) / +X (rear).
                rail_inboard = -1.0 if axle == "front" else +1.0
                rail_x0 = min(axle_x, axle_x + rail_inboard * nb["half_width"])
                steps.append(BuildStep(
                    id="axle_notch_%s_%s" % (axle, tag), role="axle_notch_cut",
                    kind="prism", boolean="subtract", target="rail_%s" % tag,
                    body_name="Axle_Notch_%s_%s" % (axle.upper(), tag.upper()),
                    material="air", color=COL_AIR,
                    profile=_rect_uv(f.rail_width_mm / 2.0 + 1.0, nb["h"] / 2.0),
                    origin3=(rail_x0, cy, nb["cz"]),
                    axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
                    length=nb["half_width"]))

    # lateral crossmembers -- hollow box beams running along +Y that BRIDGE the rails.
    # Each spans the inner channel plus a small embed into both rails, so the ends land
    # inside the rail boxes. The (u, v) section lays out in (Z, X): u = +Z spans the
    # crossmember height, v = +X spans its width. Distributed along the wheelbase in X.
    n = max(0, f.crossmember_count)
    if n > 0:
        y_start = -(f.frame_inner_width_mm / 2.0 + CROSSMEMBER_EMBED_MM)
        span_y = f.frame_inner_width_mm + 2.0 * CROSSMEMBER_EMBED_MM
        # crossmembers vertically centred on the rails (so they tie the rail webs)
        cm_cz = z["rail_cz"]
        for i in range(n):
            # spread the crossmembers symmetrically across the wheelbase
            frac = (i + 0.5) / n
            x = -f.wheelbase_mm / 2.0 + frac * f.wheelbase_mm
            _box_beam(
                steps, "crossmember_%d" % i, "crossmember", "Crossmember_%d" % i,
                origin3=(x, y_start, cm_cz),
                axis=(0.0, 1.0, 0.0), u_dir=(0.0, 0.0, 1.0),
                length=span_y,
                sec_u=f.crossmember_height_mm, sec_v=f.crossmember_width_mm,
                wall=f.crossmember_wall_mm, color=COL_CROSS)
    return steps


# --------------------------------------------------------------------------- #
# battery tray: one large hollow box BETWEEN the rails (low) + lateral crossbraces
# --------------------------------------------------------------------------- #
def battery_tray_steps(p: ChassisParams) -> List[BuildStep]:
    b = p.battery_tray
    if not b.enabled:
        return []
    z = _z_layout(p)
    steps: List[BuildStep] = []

    # The tray is a hollow box running along +X (axis=+X), centred on x = 0, dropped
    # low between the rails. Section (u, v) -> (Y, Z): u = +Y spans the tray width,
    # v = +Z spans the tray height. Its centre Z is the floor-stack tray mid-height.
    x_start = -b.length_mm / 2.0
    _box_beam(
        steps, "battery_tray", "battery_tray", "Battery_Tray",
        origin3=(x_start, 0.0, z["tray_cz"]),
        axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
        length=b.length_mm,
        sec_u=b.width_mm, sec_v=b.height_mm, wall=b.wall_mm, color=COL_TRAY)

    # internal crossbraces: full-section lateral webs (run along +Y), spaced along the
    # tray length in X. United to the tray so they stiffen the cavity floor.
    n = max(0, b.crossbrace_count)
    inner_w = b.width_mm - 2.0 * b.wall_mm          # clear cavity width (Y)
    inner_h = b.height_mm - 2.0 * b.wall_mm         # clear cavity height (Z)
    if n > 0 and inner_w > 0 and inner_h > 0:
        usable = b.length_mm - 2.0 * b.wall_mm
        for i in range(n):
            x = x_start + b.wall_mm + usable * (i + 0.5) / n
            # a thin web spanning the cavity width along +Y; section (u,v)->(Z,X):
            # u = +Z spans the cavity height, v = +X spans one wall thickness.
            _box_beam_solid_web(steps, i, x, inner_w, inner_h, b.wall_mm, z["tray_cz"])
    return steps


def _box_beam_solid_web(steps: List[BuildStep], i: int, x: float,
                        inner_w: float, inner_h: float, wall: float, cz: float):
    """A solid lateral crossbrace web inside the battery tray: a prism running along
    +Y (across the cavity), `wall` thick in X, united to the tray."""
    steps.append(BuildStep(
        id="battery_brace_%d" % i, role="battery_brace", kind="prism",
        boolean="unite", target="battery_tray",
        body_name="Battery_Crossbrace_%d" % i, material="aluminium", color=COL_BRACE,
        profile=_rect_uv(inner_h / 2.0, wall / 2.0),
        origin3=(x, -inner_w / 2.0, cz), axis=(0.0, 1.0, 0.0), u_dir=(0.0, 0.0, 1.0),
        length=inner_w))


# --------------------------------------------------------------------------- #
# subframe mount pads + bolt holes, body-mount holes, crush cans
# --------------------------------------------------------------------------- #
def mount_steps(p: ChassisParams) -> List[BuildStep]:
    f, s, bm = p.frame, p.subframe, p.body_mount
    z = _z_layout(p)
    steps: List[BuildStep] = []
    rail_cy = rail_centreline_y(p)
    boss_d = max(2.0 * s.mount_bolt_diameter_mm, 30.0)
    boss_h = 14.0                                   # pad height above the rail top

    # subframe mount pads: a vertical (+Z) cylindrical boss UNITED to the rail top at
    # the axle x-station, with a bolt-hole circle drilled down through it. Placed at
    # x = ±wheelbase/2 so the e-axle / suspension subframe bolts to the ICD axle line.
    def subframe_set(axle: str, x_station: float):
        for side, sign in (("l", -1.0), ("r", +1.0)):
            rid = "rail_%s" % side
            cy = sign * rail_cy
            bid = "subframe_boss_%s_%s" % (axle, side)
            # boss: a short cylinder on a +Z axis, base on the rail top, united to rail
            steps.append(BuildStep(
                id=bid, role="subframe_boss", kind="cylinder", boolean="unite",
                target=rid, body_name="Subframe_Boss_%s_%s" % (axle.upper(), side.upper()),
                material="aluminium", color=COL_BOSS,
                outer_radius=boss_d / 2.0, length=boss_h,
                origin3=(x_station, cy, z["rail_top"]), axis=(0.0, 0.0, 1.0)))
            # bolt-hole circle: drilled straight DOWN (-Z) through the boss + rail top
            if s.mount_bolt_count > 0 and s.mount_bolt_diameter_mm > 0:
                pcd_r = boss_d / 2.0 - max(s.mount_bolt_diameter_mm, 6.0)
                pcd_r = max(pcd_r, s.mount_bolt_diameter_mm)
                for k in range(s.mount_bolt_count):
                    ang = 2.0 * math.pi * k / s.mount_bolt_count
                    hx = x_station + pcd_r * math.cos(ang)
                    hy = cy + pcd_r * math.sin(ang)
                    steps.append(BuildStep(
                        id="subframe_bolt_%s_%s_%d" % (axle, side, k),
                        role="subframe_bolt_cut", kind="hole", boolean="subtract",
                        target=rid,
                        body_name="Subframe_Bolt_%s_%s_%d" % (axle.upper(), side.upper(), k),
                        material="air", color=COL_AIR,
                        outer_radius=s.mount_bolt_diameter_mm / 2.0,
                        cx=hx, cy=hy, z0=z["rail_top"] + boss_h + 0.5,
                        axis=(0.0, 0.0, -1.0), length=boss_h + f.rail_height_mm + 1.0))

    if s.front_subframe:
        subframe_set("front", +f.wheelbase_mm / 2.0)
    if s.rear_subframe:
        subframe_set("rear", -f.wheelbase_mm / 2.0)

    # body-mount holes along each rail TOP, drilled DOWN (-Z) into the rail, spread
    # over the rail span (the WHEELBASE -- the rails now end at the axles; the overhangs
    # are the crush cans). Split the count between the two rails.
    if bm.body_mount_count > 0 and bm.body_mount_diameter_mm > 0:
        per_rail = max(1, bm.body_mount_count // 2)
        for side, sign in (("l", -1.0), ("r", +1.0)):
            rid = "rail_%s" % side
            cy = sign * rail_cy
            for i in range(per_rail):
                frac = (i + 0.5) / per_rail
                x = -f.wheelbase_mm / 2.0 + frac * f.wheelbase_mm
                steps.append(BuildStep(
                    id="body_mount_%s_%d" % (side, i), role="body_mount_cut",
                    kind="hole", boolean="subtract", target=rid,
                    body_name="Body_Mount_%s_%d" % (side.upper(), i),
                    material="air", color=COL_AIR,
                    outer_radius=bm.body_mount_diameter_mm / 2.0,
                    cx=x, cy=cy, z0=z["rail_top"] + 0.5,
                    axis=(0.0, 0.0, -1.0), length=f.rail_wall_mm + 1.0))

    # crush cans: short hollow box beams extending along +X BEYOND the wheelbase,
    # coaxial with each rail (front ahead of +X, rear behind -X). The rails END at the
    # axles (x = +-wheelbase/2), so each can starts exactly on the rail end face and
    # grows outward into the overhang -- they BUTT onto the rail ends (no overlap) and
    # form the front/rear crash structure (ICD §3). The can length is the overhang.
    overhang = (f.overall_length_mm - f.wheelbase_mm) / 2.0
    can_len = overhang
    if can_len > 0:
        if bm.crush_can_front:
            for side, sign in (("l", -1.0), ("r", +1.0)):
                _box_beam(
                    steps, "crush_front_%s" % side, "crush_can",
                    "Crush_Can_Front_%s" % side.upper(),
                    origin3=(f.wheelbase_mm / 2.0, sign * rail_cy, z["rail_cz"]),
                    axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0), length=can_len,
                    sec_u=f.rail_width_mm, sec_v=f.rail_height_mm,
                    wall=f.rail_wall_mm, color=COL_CRUSH)
        if bm.crush_can_rear:
            for side, sign in (("l", -1.0), ("r", +1.0)):
                _box_beam(
                    steps, "crush_rear_%s" % side, "crush_can",
                    "Crush_Can_Rear_%s" % side.upper(),
                    origin3=(-f.wheelbase_mm / 2.0 - can_len, sign * rail_cy, z["rail_cz"]),
                    axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0), length=can_len,
                    sec_u=f.rail_width_mm, sec_v=f.rail_height_mm,
                    wall=f.rail_wall_mm, color=COL_CRUSH)

    # AXLE NOTCH (crush-can half): relieve the OUTBOARD half of the axle window from the
    # crush-can END (now that the cans exist), so the corner + half-shaft envelope that
    # spills past the axle into the overhang clears the can too. Each cut overlaps the can
    # solid at its inboard end -> NX leaves one clean can body. The rail half is cut in
    # frame_steps; the two halves share _axle_notch_band so they line up.
    if f.axle_notch and f.axle_notch_half_width_mm > 0 and can_len > 0:
        nb = _axle_notch_band(p)
        for axle, axle_x, fitted in (("front", +f.wheelbase_mm / 2.0, bm.crush_can_front),
                                     ("rear", -f.wheelbase_mm / 2.0, bm.crush_can_rear)):
            if not fitted:
                continue
            can_outboard = +1.0 if axle == "front" else -1.0
            cut_len = min(nb["half_width"], can_len)
            can_x0 = min(axle_x, axle_x + can_outboard * cut_len)
            for side, sign in (("l", -1.0), ("r", +1.0)):
                steps.append(BuildStep(
                    id="axle_notch_can_%s_%s" % (axle, side), role="axle_notch_cut",
                    kind="prism", boolean="subtract", target="crush_%s_%s" % (axle, side),
                    body_name="Axle_Notch_Can_%s_%s" % (axle.upper(), side.upper()),
                    material="air", color=COL_AIR,
                    profile=_rect_uv(f.rail_width_mm / 2.0 + 1.0, nb["h"] / 2.0),
                    origin3=(can_x0, sign * rail_cy, nb["cz"]),
                    axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
                    length=cut_len))
    return steps


def build_steps(p: ChassisParams) -> List[BuildStep]:
    steps = frame_steps(p)
    steps.extend(battery_tray_steps(p))
    steps.extend(mount_steps(p))
    return steps


def generate(p: ChassisParams = None) -> Dict[str, Any]:
    """Full chassis blueprint dict (NX-independent). Same schema as driveline_nx /
    motor_nx so the same run_journal builder consumes it.

    `axis` is reported as "X" because the dominant beams (rails, tray, crush cans) run
    along the vehicle +X; this is documentation only -- every step carries its own
    explicit origin3/axis, so the builder never relies on a global axis."""
    if p is None:
        p = ChassisParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "chassis_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "X",
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
