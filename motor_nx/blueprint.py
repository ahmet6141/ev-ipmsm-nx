"""Pure-math geometry for the IPMSM, emitted as an ordered list of CAD *build
steps* plus JSON. NO NX dependency -- runs under plain CPython and is unit-tested.

A :class:`BuildStep` is a deliberately small, version-independent instruction the
NX builder knows how to execute:

    kind="tube"      -> hollow cylinder on the Z axis (outer/inner radius)
    kind="cylinder"  -> solid cylinder, optionally offset to (cx, cy)
    kind="extrude"   -> extrude a closed XY polygon from z0 along +Z by `length`
    kind="revolve"   -> revolve a closed (r, z) polygon 360 deg about Z

Each step also carries a boolean op (create / subtract / unite), an optional
`target` body to boolean against, and an optional circular `pattern` (count +
angle about Z). The builder keeps an id->body registry and replays the list.

All repeated features (stator slots, rotor magnets/pockets, hairpin bars,
cooling channels) are emitted ONCE in a reference frame on the +X axis together
with a pattern count/angle, so the blueprint stays compact while the geometry is
fully manufacturable. `expand_step_instances()` materialises the rotated copies
for builders that prefer explicit instancing over NX pattern features.

Coordinate convention
    Z  = motor (rotation) axis. Active stack: z = 0 .. stack_length.
    XY = lamination cross-section.   Radial ~ X, tangential ~ Y for a feature
         whose reference instance is centred on the +X axis.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from . import em_design
from .params import MotorParams

Point = Tuple[float, float]

# material colours (RGB 0-255) for the NX bodies
COL_STEEL = (120, 125, 130)
COL_ROTOR = (96, 100, 108)
COL_MAGNET = (40, 40, 45)
COL_COPPER = (184, 115, 51)
COL_SHAFT = (180, 182, 188)
COL_HOUSING = (70, 110, 160)

_CIRCLE_SEG = 48  # polygon segments when a round profile must be tessellated


@dataclass
class BuildStep:
    id: str
    role: str                       # semantic role, e.g. "stator_slot_cut"
    kind: str                       # tube | cylinder | extrude | revolve
    boolean: str = "create"         # create | subtract | unite
    target: Optional[str] = None    # body id to boolean into (subtract/unite)
    body_name: str = ""             # NX body display name (create)
    material: str = ""
    color: Tuple[int, int, int] = (200, 200, 200)
    # primitives (tube/cylinder), centred on Z unless cx/cy given
    outer_radius: float = 0.0
    inner_radius: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    # extrude/revolve profile
    profile: Optional[List[Point]] = None   # extrude: [x,y]; revolve: [r,z]
    z0: float = 0.0
    length: float = 0.0
    # circular pattern about Z
    pattern_count: int = 1
    pattern_angle_deg: float = 0.0
    # bind the extrude axial length to the NX 'stack_length' expression (active-stack
    # laminations/slots/conductors). DISCRETE bodies (magnets, end-windings, housing)
    # keep a literal length so editing stack_length in NX does not rescale them.
    drive_with_stack: bool = False
    # partial / positioned revolve (kind="revolve"): sweep `angle_deg`, profile
    # placed in the half-plane at `start_angle_deg` about Z (default = full 360 at +X)
    angle_deg: float = 360.0
    start_angle_deg: float = 0.0
    # kind="hole": a cylindrical hole on an ARBITRARY axis (radial ports, oil cross-
    # holes, ...). base point = (cx, cy, z0), direction = `axis` (need not be +Z),
    # radius = outer_radius, depth = length. A circular `pattern` (count/angle about
    # Z) rotates BOTH the base point and the axis. (tube/cylinder/extrude stay +Z.)
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    def as_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        if self.profile is not None:
            d["profile"] = [[round(x, 6), round(y, 6)] for (x, y) in self.profile]
        return d


# --------------------------------------------------------------------------- #
# small geometry helpers
# --------------------------------------------------------------------------- #
def _rotate(points: List[Point], angle_deg: float) -> List[Point]:
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return [(x * c - y * s, x * s + y * c) for (x, y) in points]


def _mirror_y(points: List[Point]) -> List[Point]:
    return [(x, -y) for (x, y) in points]


def _rect(center: Point, u: Point, v: Point, half_len: float, half_thk: float) -> List[Point]:
    """Rectangle from a centre + length axis u + thickness axis v (both unit)."""
    cx, cy = center
    ux, uy = u
    vx, vy = v
    corners = [
        (cx - half_len * ux - half_thk * vx, cy - half_len * uy - half_thk * vy),
        (cx + half_len * ux - half_thk * vx, cy + half_len * uy - half_thk * vy),
        (cx + half_len * ux + half_thk * vx, cy + half_len * uy + half_thk * vy),
        (cx - half_len * ux + half_thk * vx, cy - half_len * uy + half_thk * vy),
    ]
    return corners


def _circle(center: Point, radius: float, segments: int = _CIRCLE_SEG) -> List[Point]:
    cx, cy = center
    return [
        (cx + radius * math.cos(2 * math.pi * i / segments),
         cy + radius * math.sin(2 * math.pi * i / segments))
        for i in range(segments)
    ]


def _round_corner(p0: Point, p1: Point, p2: Point, radius: float, segs: int = 5) -> List[Point]:
    """Polyline-arc fillet of `radius` replacing corner p1, tangent to edges
    p0-p1 and p1-p2. ROBUST: returns [p1] (left sharp) whenever the corner is
    degenerate or the radius will not fit, so a rounded section can never
    self-intersect -- the worst case is simply "no fillet here"."""
    ax, ay = p0[0] - p1[0], p0[1] - p1[1]
    bx, by = p2[0] - p1[0], p2[1] - p1[1]
    la, lb = math.hypot(ax, ay), math.hypot(bx, by)
    if la < 1e-9 or lb < 1e-9 or radius <= 0:
        return [p1]
    ax, ay, bx, by = ax / la, ay / la, bx / lb, by / lb
    dot = max(-1.0, min(1.0, ax * bx + ay * by))
    theta = math.acos(dot)                       # interior angle at p1
    if theta < 1e-4 or abs(theta - math.pi) < 1e-4:
        return [p1]                              # straight or doubled back
    t = radius / math.tan(theta / 2.0)
    t = min(t, 0.49 * la, 0.49 * lb)             # keep tangent points on the edges
    r = t * math.tan(theta / 2.0)
    if r < 1e-6:
        return [p1]
    t1 = (p1[0] + t * ax, p1[1] + t * ay)
    bisx, bisy = ax + bx, ay + by
    bl = math.hypot(bisx, bisy)
    if bl < 1e-9:
        return [p1]
    d = r / math.sin(theta / 2.0)
    cx, cy = p1[0] + d * bisx / bl, p1[1] + d * bisy / bl
    a1 = math.atan2(t1[1] - cy, t1[0] - cx)
    a2 = math.atan2((p1[1] + t * by) - cy, (p1[0] + t * bx) - cx)
    da = a2 - a1
    while da > math.pi:
        da -= 2 * math.pi
    while da < -math.pi:
        da += 2 * math.pi
    return [(cx + r * math.cos(a1 + da * i / segs),
             cy + r * math.sin(a1 + da * i / segs)) for i in range(segs + 1)]


def _round_polygon(poly: List[Point], radius: float, corners=None, segs: int = 5) -> List[Point]:
    """Round corners of a closed polygon. `corners`: iterable of vertex indices
    to round, or None for all corners. Non-fitting corners are left sharp."""
    n = len(poly)
    if n < 3 or radius <= 0:
        return list(poly)
    sel = set(range(n)) if corners is None else set(corners)
    out: List[Point] = []
    for i in range(n):
        if i in sel:
            out.extend(_round_corner(poly[(i - 1) % n], poly[i], poly[(i + 1) % n], radius, segs))
        else:
            out.append(poly[i])
    return out


# --------------------------------------------------------------------------- #
# reference profiles (used by both the blueprint and em_design validation)
# --------------------------------------------------------------------------- #
def stator_slot_polygon(p: MotorParams, g: em_design.DerivedGeometry) -> List[Point]:
    """One semi-closed, parallel-wall (hairpin) slot, centred on +X.

    The mouth starts a hair below the bore so the subtract cut is clean.
    """
    s = p.stator
    r_bore = g.bore_radius
    r1 = g.slot_body_inner_radius
    r2 = g.slot_body_outer_radius
    ow = s.slot_opening_width / 2.0
    sw = g.slot_width / 2.0
    margin = 0.5  # mouth pokes into the bore for a guaranteed through-cut
    poly = [
        (r_bore - margin, -ow),
        (r1, -ow),
        (r1, -sw),
        (r2, -sw),   # slot-bottom corner
        (r2, sw),    # slot-bottom corner
        (r1, sw),
        (r1, ow),
        (r_bore - margin, ow),
    ]
    # round the two slot-bottom corners (stress relief at the back of the slot)
    return _round_polygon(poly, s.slot_bottom_fillet, corners={3, 4})


def v_tilt_deg(v_angle_deg: float) -> float:
    """Magnet tilt from the tangential direction, from the V's included opening
    angle: a 180 deg "V" is a flat tangential bar (tilt 0); a narrower V is
    steeper. tilt = (180 - v_angle) / 2."""
    return (180.0 - v_angle_deg) / 2.0


def _v_magnet_axes(v_angle_deg: float) -> Tuple[Point, Point]:
    """Length axis u (inner->outer, toward the pole edge) and thickness axis v
    (toward the air-gap side) for the +Y magnet of the V."""
    th = math.radians(v_tilt_deg(v_angle_deg))
    u = (math.sin(th), math.cos(th))
    v = (math.cos(th), -math.sin(th))
    return u, v


def _v_inner_end_center(p: MotorParams, g: em_design.DerivedGeometry) -> Point:
    """Centre of the inner (vertex-side) end face of the +Y magnet, placed on the
    apex circle of radius (shaft + vertex_gap) at tangential offset y. If that
    circle is smaller than the offset (degenerate: vertex_gap too small -- caught
    by em_design.validate), x collapses to 0 rather than a hidden fudge value."""
    r = p.rotor
    y = r.center_post_halfwidth + r.magnet_thickness / 2.0
    r_anchor = g.shaft_radius + r.vertex_gap
    x = math.sqrt(max(r_anchor ** 2 - y ** 2, 0.0))
    return (x, y)


def magnet_pocket_polygons(p: MotorParams, g: em_design.DerivedGeometry) -> List[List[Point]]:
    """Two magnet-pocket cuts (the +Y and -Y arms of one V), reference pole on +X.

    A pocket is the magnet rectangle grown by `end_barrier` along its length
    (flux barriers) and by `pocket_clearance` across its thickness.
    """
    r = p.rotor
    u, v = _v_magnet_axes(r.v_angle_deg)
    e_in = _v_inner_end_center(p, g)
    e_out = (e_in[0] + r.magnet_width * u[0], e_in[1] + r.magnet_width * u[1])
    center = ((e_in[0] + e_out[0]) / 2.0, (e_in[1] + e_out[1]) / 2.0)
    half_len = r.magnet_width / 2.0 + r.end_barrier
    half_thk = r.magnet_thickness / 2.0 + r.pocket_clearance
    plus = _rect(center, u, v, half_len, half_thk)
    plus = _round_polygon(plus, r.magnet_pocket_fillet)   # round all 4 pocket / flux-barrier corners
    minus = _mirror_y(plus)
    return [plus, minus]


def magnet_polygons(p: MotorParams, g: em_design.DerivedGeometry) -> List[List[Point]]:
    """The two magnet solids of one V (sit inside the pockets with clearance)."""
    r = p.rotor
    u, v = _v_magnet_axes(r.v_angle_deg)
    e_in = _v_inner_end_center(p, g)
    e_out = (e_in[0] + r.magnet_width * u[0], e_in[1] + r.magnet_width * u[1])
    center = ((e_in[0] + e_out[0]) / 2.0, (e_in[1] + e_out[1]) / 2.0)
    plus = _rect(center, u, v, r.magnet_width / 2.0, r.magnet_thickness / 2.0)
    return [plus, _mirror_y(plus)]


def conductor_polygons(p: MotorParams, g: em_design.DerivedGeometry) -> List[List[Point]]:
    """Stacked rectangular hairpin bars for one slot, reference slot on +X."""
    w = p.winding
    n = max(1, w.conductors_per_slot)
    clr = w.bar_clearance
    bw = g.slot_width - 2 * clr                      # tangential bar width
    bar_h = (g.slot_depth - clr * (n + 1)) / n       # radial bar height
    bars: List[List[Point]] = []
    if bw <= 0 or bar_h <= 0:
        return bars
    for k in range(n):
        r_lo = g.slot_body_inner_radius + clr + k * (bar_h + clr)
        r_hi = r_lo + bar_h
        bar = [(r_lo, -bw / 2), (r_hi, -bw / 2), (r_hi, bw / 2), (r_lo, bw / 2)]
        bars.append(_round_polygon(bar, w.bar_corner_radius))  # rounded hairpin-bar corners
    return bars


def shaft_profile(p: MotorParams, g: em_design.DerivedGeometry) -> List[Point]:
    """Closed (r, z) profile of a stepped shaft, revolved 360 deg about Z.

    DE (+Z) order inside->out: main journal -> bearing seat -> optional OUTPUT STUB
    (a reduced-diameter extension past the bearing that carries the drive key)."""
    sh = p.shaft
    r_main = sh.diameter / 2.0
    r_brg = sh.bearing_seat_diameter / 2.0
    r_axis = sh.bore_diameter / 2.0 if sh.bore_diameter > 0 else 0.0
    z_l = -sh.overhang
    z_r = p.stack_length + sh.overhang
    seat = sh.bearing_seat_length
    outer = [
        (r_brg, z_l),
        (r_brg, z_l + seat),
        (r_main, z_l + seat),
        (r_main, z_r - seat),
        (r_brg, z_r - seat),
        (r_brg, z_r),
    ]
    z_end = z_r
    if sh.drive_stub_length > 0 and 0 < sh.drive_stub_diameter < sh.bearing_seat_diameter:
        r_stub = sh.drive_stub_diameter / 2.0
        outer += [(r_stub, z_r), (r_stub, z_r + sh.drive_stub_length)]
        z_end = z_r + sh.drive_stub_length
    # close the loop down the (hollow or solid) axis side
    return outer + [(r_axis, z_end), (r_axis, z_l)]


# --------------------------------------------------------------------------- #
# build-step assembly
# --------------------------------------------------------------------------- #
def build_steps(p: MotorParams, g: em_design.DerivedGeometry) -> List[BuildStep]:
    s, r, w, c = p.stator, p.rotor, p.winding, p.cooling
    steps: List[BuildStep] = []

    # 1) stator lamination steel (annulus)
    steps.append(BuildStep(
        id="stator_steel", role="stator_steel", kind="tube", boolean="create",
        body_name="Stator_Lamination", material="electrical_steel", color=COL_STEEL,
        outer_radius=g.stator_outer_radius, inner_radius=g.bore_radius,
        z0=0.0, length=p.stack_length, drive_with_stack=True,
    ))
    # 2) slot cuts (one reference slot, patterned Q times)
    steps.append(BuildStep(
        id="stator_slots", role="stator_slot_cut", kind="extrude", boolean="subtract",
        target="stator_steel", body_name="Slot_Cut", material="air", color=COL_STEEL,
        profile=stator_slot_polygon(p, g), z0=0.0, length=p.stack_length,
        pattern_count=s.slot_count, pattern_angle_deg=360.0 / s.slot_count,
        drive_with_stack=True,
    ))

    # 3) rotor lamination steel (annulus, bore = shaft fit). NOTE: the rotor bore
    #    is modelled coincident with the shaft OD -- torque transfer is a press/
    #    shrink fit (interference, see docs/MANUFACTURING.md tolerances), not a
    #    modelled keyway/spline; magnet axial retention (end rings) and balance
    #    lands are likewise manufacturing features, not in this electromagnetic solid.
    steps.append(BuildStep(
        id="rotor_steel", role="rotor_steel", kind="tube", boolean="create",
        body_name="Rotor_Lamination", material="electrical_steel", color=COL_ROTOR,
        outer_radius=g.rotor_outer_radius, inner_radius=g.shaft_radius,
        z0=0.0, length=p.stack_length, drive_with_stack=True,
    ))
    # 4) magnet pockets (V): two arms, each patterned over the poles
    for idx, poly in enumerate(magnet_pocket_polygons(p, g)):
        steps.append(BuildStep(
            id=f"magnet_pocket_{idx}", role="magnet_pocket_cut", kind="extrude",
            boolean="subtract", target="rotor_steel",
            body_name=f"Magnet_Pocket_{idx}", material="air", color=COL_ROTOR,
            profile=poly, z0=0.0, length=p.stack_length,
            pattern_count=r.pole_count, pattern_angle_deg=360.0 / r.pole_count,
            drive_with_stack=True,
        ))
    # 4b) optional rotor lightening / cooling holes
    if r.lightening_holes > 0:
        steps.append(BuildStep(
            id="rotor_lightening", role="rotor_hole_cut", kind="cylinder",
            boolean="subtract", target="rotor_steel",
            body_name="Lightening_Hole", material="air", color=COL_ROTOR,
            outer_radius=r.lightening_hole_diameter / 2.0,
            cx=r.lightening_hole_pitch_radius, cy=0.0, z0=0.0, length=p.stack_length,
            pattern_count=r.lightening_holes, pattern_angle_deg=360.0 / r.lightening_holes,
        ))
    # 5) magnets as separate bodies (NdFeB), split into axial segments to cut
    #    rotor-magnet eddy-current loss (each segment insulated by a thin gap).
    #    n_seg == 1 reproduces the original single full-length block exactly.
    #    NOTE: segmented magnets are DISCRETE fixed-length pieces -- unlike the
    #    laminated stack they are not driven by the NX 'stack_length' expression,
    #    so editing stack_length in NX rescales the steel/winding but not the
    #    magnet segments; regenerate from params for a different stack length.
    n_seg = max(1, int(p.material.magnet_segments_axial))
    seg_gap = p.material.magnet_seg_gap_mm if n_seg > 1 else 0.0
    seg_len = (p.stack_length - seg_gap * (n_seg - 1)) / n_seg
    for idx, poly in enumerate(magnet_polygons(p, g)):
        for j in range(n_seg):
            seg = "" if n_seg == 1 else "_seg%d" % j
            steps.append(BuildStep(
                id="magnet_%d%s" % (idx, seg), role="magnet", kind="extrude",
                boolean="create", body_name="Magnet_%d%s" % (idx, seg),
                material="NdFeB", color=COL_MAGNET,
                profile=poly, z0=j * (seg_len + seg_gap), length=seg_len,
                pattern_count=r.pole_count, pattern_angle_deg=360.0 / r.pole_count,
            ))

    # 6) hairpin conductor bars (one stack per slot, patterned Q times)
    for k, poly in enumerate(conductor_polygons(p, g)):
        steps.append(BuildStep(
            id=f"conductor_{k}", role="conductor", kind="extrude", boolean="create",
            body_name=f"Hairpin_Bar_L{k}", material="copper", color=COL_COPPER,
            profile=poly, z0=0.0, length=p.stack_length,
            pattern_count=s.slot_count, pattern_angle_deg=360.0 / s.slot_count,
            drive_with_stack=True,
        ))

    # 6b) end-winding. "envelope" = a toroidal ring per stack end (default, robust).
    #     "hairpin" = an individual crown ARC per slot per end -- a positioned
    #     partial revolve spanning the coil pitch, much closer to real bent hairpin
    #     end-turns (experimental; the arcs nest/overlap as in a real crown bundle).
    if w.model_endwindings and w.end_winding_height > 0:
        if getattr(w, "end_winding_style", "envelope") == "hairpin":
            # Flat-top U hairpin end-turn PER SLOT: an axial riser rises out of the
            # slot to the crown height, a circumferential crown arc spans the coil
            # pitch, and the next slot's riser brings it back down. Much closer to a
            # real bent hairpin than a flat ring. (Per-slot bundle, not per-conductor:
            # a full per-conductor solid winding -- 864 bent bars -- is impractical
            # as solids here and is industry-done with winding tools + FEA.)
            r_mid = 0.5 * (g.slot_body_inner_radius + g.slot_body_outer_radius)
            half_r = 0.40 * (g.slot_body_outer_radius - g.slot_body_inner_radius)
            bw = max(1.5, g.slot_width - 2 * w.bar_clearance)
            apex = w.end_winding_height
            pitch_ang = (s.slot_count // r.pole_count) * (360.0 / s.slot_count)  # coil span
            slot_ang = 360.0 / s.slot_count
            n_step = 7                      # facets across a crown roof (rise -> peak -> fall)
            shoulder = apex * 0.55          # riser height (the crown peaks above this)
            rise = apex * 0.45              # crown apex above the shoulder
            step_z = rise * 2.0 / (n_step - 1)
            crown_hz = 0.8 * step_z         # crown axial half-height -> facets overlap (no gaps)
            sub = pitch_ang / n_step
            riser_ref = [
                (r_mid - half_r, -bw / 2.0), (r_mid + half_r, -bw / 2.0),
                (r_mid + half_r, bw / 2.0), (r_mid - half_r, bw / 2.0),
            ]
            for end in ("front", "rear"):
                z0_riser = p.stack_length if end == "rear" else -shoulder
                for i in range(s.slot_count):
                    ang = i * slot_ang
                    steps.append(BuildStep(
                        id="hp_riser_%s_%d" % (end, i), role="end_winding", kind="extrude",
                        boolean="create", body_name="HP_Riser_%s_%d" % (end, i),
                        material="copper", color=COL_COPPER,
                        profile=_rotate(riser_ref, ang), z0=z0_riser, length=shoulder,
                    ))
                    for k in range(n_step):
                        tri = 1.0 - abs(2.0 * k / (n_step - 1) - 1.0)   # 0 -> 1 -> 0 roof
                        h = shoulder + rise * tri
                        zc = (p.stack_length + h) if end == "rear" else -h
                        crown_rz = [
                            (r_mid - half_r, zc - crown_hz), (r_mid + half_r, zc - crown_hz),
                            (r_mid + half_r, zc + crown_hz), (r_mid - half_r, zc + crown_hz),
                        ]
                        steps.append(BuildStep(
                            id="hp_crown_%s_%d_%d" % (end, i, k), role="end_winding", kind="revolve",
                            boolean="create", body_name="HP_Crown_%s_%d_%d" % (end, i, k),
                            material="copper", color=COL_COPPER,
                            profile=crown_rz, start_angle_deg=ang + k * sub, angle_deg=sub,
                        ))
        else:
            for z0, end in ((-w.end_winding_height, "front"), (p.stack_length, "rear")):
                steps.append(BuildStep(
                    id="endwinding_%s" % end, role="end_winding", kind="tube",
                    boolean="create", body_name="EndWinding_%s" % end,
                    material="copper", color=COL_COPPER,
                    outer_radius=g.slot_body_outer_radius,
                    inner_radius=g.slot_body_inner_radius,
                    z0=z0, length=w.end_winding_height,
                ))

    # 7) shaft (stepped, revolved)
    steps.append(BuildStep(
        id="shaft", role="shaft", kind="revolve", boolean="create",
        body_name="Shaft", material="shaft_steel", color=COL_SHAFT,
        profile=shaft_profile(p, g),
    ))

    # 8) cooling jacket / housing sleeve. The jacket should axially OVERSHADOW the
    #    end-windings (the hottest copper), so the end margin tracks their extent.
    jacket_inner = g.stator_outer_radius + c.housing_gap
    jacket_outer = jacket_inner + c.jacket_thickness
    end_margin = 8.0
    if w.model_endwindings and w.end_winding_height > 0:
        ew_extent = w.end_winding_height
        if getattr(w, "end_winding_style", "envelope") == "hairpin":
            ew_extent *= 1.12   # crown shoulder+rise+crown_hz overshoots the nominal height
        end_margin = max(end_margin, ew_extent + 3.0)
    steps.append(BuildStep(
        id="housing", role="housing", kind="tube", boolean="create",
        body_name="Cooling_Jacket", material="aluminium", color=COL_HOUSING,
        outer_radius=jacket_outer, inner_radius=jacket_inner,
        z0=-end_margin, length=p.stack_length + 2 * end_margin,
    ))
    # 9) axial cooling channels through the jacket
    if c.channel_type == "axial" and c.channel_count > 0:
        pitch_r = 0.5 * (jacket_inner + jacket_outer)
        steps.append(BuildStep(
            id="cooling_channels", role="cooling_channel_cut", kind="cylinder",
            boolean="subtract", target="housing",
            body_name="Cooling_Channel", material="coolant", color=COL_HOUSING,
            outer_radius=c.channel_diameter / 2.0, cx=pitch_r, cy=0.0,
            z0=-end_margin, length=p.stack_length + 2 * end_margin,
            pattern_count=c.channel_count, pattern_angle_deg=360.0 / c.channel_count,
        ))

    # 10) manufacturing / assembly features (fastening holes, keyways, mounting
    #     flanges, coolant ports, terminal, lifting eye) -- the production details
    #     a real, assemblable part needs on top of the EM-active solid.
    steps.extend(assembly_steps(p, g, jacket_inner, jacket_outer, end_margin))

    return steps


# --------------------------------------------------------------------------- #
# manufacturing / assembly features (holes, keyways, flange, ports, ...)
# --------------------------------------------------------------------------- #
def _radial_hole(step_id: str, role: str, body_name: str, target: str,
                 angle_deg: float, z: float, from_radius: float,
                 radius: float, depth: float, color, pattern_count: int = 1) -> BuildStep:
    """A cylindrical hole drilled RADIALLY inward, starting on the cylinder of
    `from_radius` at `angle_deg` and pointing toward the Z axis for `depth` mm.
    (A circular pattern rotates the base point AND the inward axis about Z.)"""
    a = math.radians(angle_deg)
    cx, cy = from_radius * math.cos(a), from_radius * math.sin(a)
    return BuildStep(
        id=step_id, role=role, kind="hole", boolean="subtract", target=target,
        body_name=body_name, material="air", color=color,
        outer_radius=radius, cx=cx, cy=cy, z0=z, length=depth,
        axis=(-math.cos(a), -math.sin(a), 0.0),
        pattern_count=pattern_count, pattern_angle_deg=360.0 / max(1, pattern_count),
    )


def assembly_steps(p: MotorParams, g: em_design.DerivedGeometry,
                   jacket_inner: float, jacket_outer: float, end_margin: float) -> List[BuildStep]:
    """All manufacturing / assembly hole + feature cuts (and the mounting flange).
    Every feature is independently toggleable (count/size 0 => skipped) so a pure
    EM solid is produced by `assembly.enabled = False`. em_design.validate() has
    already geometrically vetted every feature placed here."""
    a = p.assembly
    if not a.enabled:
        return []
    s, r, sh, c = p.stator, p.rotor, p.shaft, p.cooling
    steps: List[BuildStep] = []
    L = p.stack_length

    # ---- STATOR: axial tie-rod / clamping holes in the yoke (back-iron) ----- #
    if a.stator_tie_rod_count > 0 and a.stator_tie_rod_diameter > 0:
        pr = a.stator_tie_rod_pitch_radius or 0.5 * (g.slot_body_outer_radius + g.stator_outer_radius)
        steps.append(BuildStep(
            id="stator_tie_rod", role="stator_tie_rod_cut", kind="cylinder",
            boolean="subtract", target="stator_steel", body_name="Stator_TieRod_Hole",
            material="air", color=COL_STEEL, outer_radius=a.stator_tie_rod_diameter / 2.0,
            cx=pr, cy=0.0, z0=0.0, length=L, drive_with_stack=True,
            pattern_count=a.stator_tie_rod_count, pattern_angle_deg=360.0 / a.stator_tie_rod_count,
        ))
    # ---- STATOR: anti-rotation key-notches on the OD ------------------------ #
    if a.stator_key_count > 0 and a.stator_key_width > 0 and a.stator_key_depth > 0:
        Rod = g.stator_outer_radius
        hw = a.stator_key_width / 2.0
        notch = [(Rod - a.stator_key_depth, -hw), (Rod + 0.5, -hw),
                 (Rod + 0.5, hw), (Rod - a.stator_key_depth, hw)]
        steps.append(BuildStep(
            id="stator_key", role="stator_key_cut", kind="extrude",
            boolean="subtract", target="stator_steel", body_name="Stator_OD_Key",
            material="air", color=COL_STEEL, profile=notch, z0=0.0, length=L,
            drive_with_stack=True, pattern_count=a.stator_key_count,
            pattern_angle_deg=360.0 / a.stator_key_count,
        ))

    # ---- ROTOR: axial rivet / end-plate-retention holes in the hub ---------- #
    if a.rotor_rivet_count > 0 and a.rotor_rivet_diameter > 0:
        pr = a.rotor_rivet_pitch_radius or (g.shaft_radius + 0.45 * r.vertex_gap)
        steps.append(BuildStep(
            id="rotor_rivet", role="rotor_rivet_cut", kind="cylinder",
            boolean="subtract", target="rotor_steel", body_name="Rotor_Rivet_Hole",
            material="air", color=COL_ROTOR, outer_radius=a.rotor_rivet_diameter / 2.0,
            cx=pr, cy=0.0, z0=0.0, length=L, drive_with_stack=True,
            pattern_count=a.rotor_rivet_count, pattern_angle_deg=360.0 / a.rotor_rivet_count,
        ))
    # ---- ROTOR: optional bore keyway (default OFF -> press/shrink fit) ------- #
    if a.rotor_keyway_width > 0 and a.rotor_keyway_depth > 0:
        rb = g.shaft_radius
        hw = a.rotor_keyway_width / 2.0
        key = [(rb - 0.5, -hw), (rb + a.rotor_keyway_depth, -hw),
               (rb + a.rotor_keyway_depth, hw), (rb - 0.5, hw)]
        steps.append(BuildStep(
            id="rotor_keyway", role="rotor_keyway_cut", kind="extrude",
            boolean="subtract", target="rotor_steel", body_name="Rotor_Bore_Keyway",
            material="air", color=COL_ROTOR, profile=key, z0=0.0, length=L, drive_with_stack=True,
        ))

    # ---- SHAFT: drive-end keyway (DIN 6885-A) ------------------------------- #
    z_r = L + sh.overhang                       # bearing-seat (+Z) end of the shaft
    r_brg = sh.bearing_seat_diameter / 2.0
    r_main = sh.diameter / 2.0
    has_stub = sh.drive_stub_length > 0 and 0 < sh.drive_stub_diameter < sh.bearing_seat_diameter
    if a.shaft_keyway_width > 0 and a.shaft_keyway_depth > 0 and a.shaft_keyway_length > 0:
        # Put the keyway on the OUTPUT STUB when present (the correct place); else on
        # the DE bearing seat, clamped to the seat length so it never crosses the
        # journal->main-diameter step (which would leave a messy sub-surface slot).
        if has_stub:
            r_surf = sh.drive_stub_diameter / 2.0
            kl = min(a.shaft_keyway_length, sh.drive_stub_length)
            zk0 = z_r + (sh.drive_stub_length - kl)   # keyway ends at the stub tip
        else:
            r_surf = r_brg
            kl = min(a.shaft_keyway_length, sh.bearing_seat_length)
            zk0 = z_r - kl
        hw = a.shaft_keyway_width / 2.0
        key = [(r_surf - a.shaft_keyway_depth, -hw), (r_surf + 0.5, -hw),
               (r_surf + 0.5, hw), (r_surf - a.shaft_keyway_depth, hw)]
        steps.append(BuildStep(
            id="shaft_keyway", role="shaft_keyway_cut", kind="extrude",
            boolean="subtract", target="shaft", body_name="Shaft_DE_Keyway",
            material="air", color=COL_SHAFT, profile=key,
            z0=zk0, length=kl,
        ))
    # ---- SHAFT: retaining-ring (DIN 471) groove inboard of the DE seat ------ #
    if a.shaft_snap_ring_width > 0 and a.shaft_snap_ring_depth > 0:
        z_groove = z_r - sh.bearing_seat_length - a.shaft_snap_ring_width
        groove = [(r_main - a.shaft_snap_ring_depth, z_groove),
                  (r_main + 0.5, z_groove),
                  (r_main + 0.5, z_groove + a.shaft_snap_ring_width),
                  (r_main - a.shaft_snap_ring_depth, z_groove + a.shaft_snap_ring_width)]
        steps.append(BuildStep(
            id="shaft_snap_groove", role="shaft_groove_cut", kind="revolve",
            boolean="subtract", target="shaft", body_name="Shaft_RetainingRing_Groove",
            material="air", color=COL_SHAFT, profile=groove,
        ))
    # ---- SHAFT: radial oil cross-holes (hollow-shaft rotor cooling) --------- #
    if sh.bore_diameter > 0 and a.shaft_oil_hole_count > 0 and a.shaft_oil_hole_diameter > 0:
        depth = r_main - sh.bore_diameter / 2.0 + 1.0   # surface -> bore
        steps.append(_radial_hole(
            "shaft_oil_hole", "shaft_oil_cut", "Shaft_Oil_Hole", "shaft",
            angle_deg=0.0, z=L / 2.0, from_radius=r_main + 0.5,
            radius=a.shaft_oil_hole_diameter / 2.0, depth=depth, color=COL_SHAFT,
            pattern_count=a.shaft_oil_hole_count))

    # ---- HOUSING: mounting flanges at BOTH ends (unite a disk, reopen bore) - #
    z_de = L + end_margin                       # +Z (drive-end) housing face
    z_nde = -end_margin                         # -Z (non-drive-end) housing face
    flange_outer = jacket_outer + a.housing_flange_od_margin
    if a.housing_flange_thickness > 0 and a.housing_flange_od_margin > 0:
        for end, z0 in (("de", z_de - a.housing_flange_thickness), ("nde", z_nde)):
            steps.append(BuildStep(
                id="housing_flange_%s_disk" % end, role="housing", kind="cylinder",
                boolean="unite", target="housing", body_name="Housing_Flange_%s" % end.upper(),
                material="aluminium", color=COL_HOUSING, outer_radius=flange_outer,
                cx=0.0, cy=0.0, z0=z0, length=a.housing_flange_thickness))
            steps.append(BuildStep(
                id="housing_flange_%s_bore" % end, role="housing", kind="cylinder",
                boolean="subtract", target="housing", body_name="Housing_Flange_Bore_%s" % end.upper(),
                material="air", color=COL_HOUSING, outer_radius=jacket_inner,
                cx=0.0, cy=0.0, z0=z0 - 0.5, length=a.housing_flange_thickness + 1.0))
        # end-shield / bearing-cap bolt circle in BOTH flange lips (through)
        if a.housing_endshield_bolt_count > 0 and a.housing_endshield_bolt_diameter > 0:
            pr = jacket_outer + 0.30 * a.housing_flange_od_margin
            for end, z0 in (("de", z_de - a.housing_flange_thickness), ("nde", z_nde)):
                steps.append(BuildStep(
                    id="housing_endshield_bolt_%s" % end, role="housing_bolt_cut", kind="cylinder",
                    boolean="subtract", target="housing", body_name="EndShield_Bolt_%s" % end.upper(),
                    material="air", color=COL_HOUSING,
                    outer_radius=a.housing_endshield_bolt_diameter / 2.0, cx=pr, cy=0.0,
                    z0=z0 - 0.5, length=a.housing_flange_thickness + 1.0,
                    pattern_count=a.housing_endshield_bolt_count,
                    pattern_angle_deg=360.0 / a.housing_endshield_bolt_count))
        # gearbox-mounting bolt circle in the DE flange only (through)
        if a.housing_mount_bolt_count > 0 and a.housing_mount_bolt_diameter > 0:
            pr = jacket_outer + 0.72 * a.housing_flange_od_margin
            steps.append(BuildStep(
                id="housing_mount_bolt", role="housing_bolt_cut", kind="cylinder",
                boolean="subtract", target="housing", body_name="Mount_Bolt_DE",
                material="air", color=COL_HOUSING,
                outer_radius=a.housing_mount_bolt_diameter / 2.0, cx=pr, cy=0.0,
                z0=z_de - a.housing_flange_thickness - 0.5, length=a.housing_flange_thickness + 1.0,
                pattern_count=a.housing_mount_bolt_count,
                pattern_angle_deg=360.0 / a.housing_mount_bolt_count))

    # ---- HOUSING: radial coolant inlet + outlet ports ----------------------- #
    port_depth = c.jacket_thickness + 2.0
    if a.housing_coolant_port_diameter > 0:
        for sid, ang, z in (("housing_coolant_in", 80.0, z_nde + a.housing_flange_thickness + 8.0),
                            ("housing_coolant_out", 100.0, z_de - a.housing_flange_thickness - 8.0)):
            steps.append(_radial_hole(
                sid, "housing_port_cut", "Coolant_Port", "housing",
                angle_deg=ang, z=z, from_radius=jacket_outer + 1.0,
                radius=a.housing_coolant_port_diameter / 2.0, depth=port_depth, color=COL_HOUSING))
    # ---- HOUSING: radial power-terminal / cable lead-through (+ raised boss) - #
    if a.housing_terminal_diameter > 0:
        boss_h = max(0.0, a.housing_terminal_boss)
        if boss_h > 0:
            # a raised cast pad the terminal box / gland bolts to (radial protrusion)
            steps.append(BuildStep(
                id="housing_terminal_boss", role="housing", kind="hole", boolean="unite",
                target="housing", body_name="Terminal_Boss", material="aluminium", color=COL_HOUSING,
                outer_radius=a.housing_terminal_diameter / 2.0 + 8.0,
                cx=jacket_outer, cy=0.0, z0=L / 2.0, length=boss_h, axis=(1.0, 0.0, 0.0)))
        steps.append(_radial_hole(
            "housing_terminal", "housing_terminal_cut", "Terminal_LeadThrough", "housing",
            angle_deg=0.0, z=L / 2.0, from_radius=jacket_outer + boss_h + 1.0,
            radius=a.housing_terminal_diameter / 2.0, depth=boss_h + port_depth, color=COL_HOUSING))
    # ---- HOUSING: radial tapped lifting-eye hole on top --------------------- #
    if a.housing_lifting_hole_diameter > 0:
        steps.append(_radial_hole(
            "housing_lifting", "housing_lifting_cut", "Lifting_Eye_Hole", "housing",
            angle_deg=90.0, z=L / 2.0, from_radius=jacket_outer + 1.0,
            radius=a.housing_lifting_hole_diameter / 2.0,
            depth=c.jacket_thickness * 0.9, color=COL_HOUSING))

    # ---- END-SHIELDS / bearing caps (DE + NDE) as separate cast parts ------- #
    if a.endshield_enabled and a.endshield_thickness > 0 and a.housing_flange_thickness > 0:
        r_bore = a.endshield_bearing_bore / 2.0
        for end, z0 in (("de", z_de), ("nde", z_nde - a.endshield_thickness)):
            steps.append(BuildStep(
                id="endshield_%s" % end, role="endshield", kind="tube", boolean="create",
                body_name="EndShield_%s" % end.upper(), material="aluminium", color=COL_HOUSING,
                outer_radius=flange_outer, inner_radius=r_bore,
                z0=z0, length=a.endshield_thickness))
            # bolt holes through the end-shield, aligned to the housing end-shield circle
            if a.housing_endshield_bolt_count > 0:
                pr = jacket_outer + 0.30 * a.housing_flange_od_margin
                steps.append(BuildStep(
                    id="endshield_%s_bolt" % end, role="endshield_bolt_cut", kind="cylinder",
                    boolean="subtract", target="endshield_%s" % end,
                    body_name="EndShield_Bolt_%s" % end.upper(), material="air", color=COL_HOUSING,
                    outer_radius=a.housing_endshield_bolt_diameter / 2.0, cx=pr, cy=0.0,
                    z0=z0 - 0.5, length=a.endshield_thickness + 1.0,
                    pattern_count=a.housing_endshield_bolt_count,
                    pattern_angle_deg=360.0 / a.housing_endshield_bolt_count))

    return steps


def expand_step_instances(step: BuildStep) -> List[Dict[str, Any]]:
    """Materialise a patterned step into explicit instances (rotated copies about
    Z). Used by builders that prefer explicit instancing to NX pattern features.
    Returns a list of dicts with concrete `profile` / `cx,cy` per instance.
    """
    out: List[Dict[str, Any]] = []
    for i in range(max(1, step.pattern_count)):
        ang = i * step.pattern_angle_deg
        inst: Dict[str, Any] = {"index": i, "angle_deg": ang}
        if step.profile is not None and step.kind == "extrude":
            inst["profile"] = _rotate(step.profile, ang)
        if step.kind in ("cylinder", "hole"):
            cx, cy = _rotate([(step.cx, step.cy)], ang)[0]
            inst["cx"], inst["cy"] = cx, cy
        if step.kind == "hole":
            ax, ay, az = step.axis
            rx, ry = _rotate([(ax, ay)], ang)[0]
            inst["axis"] = (rx, ry, az)
        out.append(inst)
    return out


def iter_cross_section(blueprint: Dict[str, Any], roles: Optional[set] = None):
    """Project the build steps onto the XY lamination plane, patterns expanded,
    end-windings (axially outside the section) skipped. Yields (role, shape) where
    shape is ("circle", cx, cy, r) or ("polygon", [(x, y), ...]).

    The single source of truth for the 2D section, shared by preview.py,
    fea.to_dxf and drawings.py (previously copy-pasted in all three)."""
    for st in blueprint["build_steps"]:
        role = st.get("role", "")
        if role == "end_winding" or (roles is not None and role not in roles):
            continue
        kind = st["kind"]
        count = max(1, st.get("pattern_count", 1))
        ang = st.get("pattern_angle_deg", 0.0)
        if kind == "tube":
            yield role, ("circle", 0.0, 0.0, st["outer_radius"])
            yield role, ("circle", 0.0, 0.0, st["inner_radius"])
        elif kind == "cylinder":
            for i in range(count):
                cx, cy = _rotate([(st.get("cx", 0.0), st.get("cy", 0.0))], i * ang)[0]
                yield role, ("circle", cx, cy, st["outer_radius"])
        elif kind == "extrude" and st.get("profile"):
            base = [(pt[0], pt[1]) for pt in st["profile"]]
            for i in range(count):
                yield role, ("polygon", _rotate(base, i * ang))
        elif kind == "revolve" and st.get("profile"):
            rs = [pt[0] for pt in st["profile"]]
            if rs:
                yield role, ("circle", 0.0, 0.0, max(rs))
                if min(rs) > 1e-6:
                    yield role, ("circle", 0.0, 0.0, min(rs))


# --------------------------------------------------------------------------- #
# top-level blueprint
# --------------------------------------------------------------------------- #
def generate(p: Optional[MotorParams] = None) -> Dict[str, Any]:
    """Full blueprint dict for one motor variant (NX-independent)."""
    if p is None:
        p = MotorParams()
    g = em_design.derive(p)
    steps = build_steps(p, g)
    return {
        "schema": "motor_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": p.stack_length,
        "parameters": p.to_dict(),
        "expressions": [
            {"name": n, "value": v, "unit": u} for (n, v, u) in p.expressions()
        ],
        "derived": asdict(g),
        "validation": em_design.validate(p),
        "build_steps": [step.as_dict() for step in steps],
    }


def to_json(blueprint: Dict[str, Any], indent: int = 2) -> str:
    return json.dumps(blueprint, indent=indent)
