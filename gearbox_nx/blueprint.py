"""Pure-math reduction-gearbox geometry, emitted as the SAME ordered build-step list
the NX builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(tube / cylinder / extrude / revolve / prism + boolean create/subtract/unite), so
motor_nx's hardened NXOpen engine builds the gearbox with no new geometry code.

Coordinate convention (ICD §7.1)
    Z = the gear (rotation) axes -- the motor, layshaft and differential axes are all
        parallel to +Z. The DIFFERENTIAL axis is the LOCAL ORIGIN (x = y = 0). The
        layshaft and motor axes are offset in the local XY plane (engineering.axis_positions).
    The two reduction MESHES sit in DISJOINT axial Z bands on the layshaft
    (engineering.axial_bands): stage-1 (motor pinion <-> layshaft gear) in band 1,
    stage-2 (layshaft pinion <-> output gear) in band 2.

Modelling abstraction
    Gears are BLANKS at pitch diameter (teeth cut later by hobbing) -- the same
    "envelope" philosophy motor_nx uses for end-windings and driveline_nx uses for its
    gears. The production INTERFACES are modelled exactly: the motor-mounting flange
    (matched to the motor DE flange from motor_nx), the differential-carrier mount, and
    the output coupling (matched to the driveline diff input flange).

    The cast housing is a representative SHELL: an outer cast wall whose footprint is
    the convex hull (a stadium/oval) of the three gear envelopes, extruded along the
    gear axis and hollowed to the gear cavity, capped by two end covers; the motor and
    diff mounting flanges + the oil sump are united on.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .gear_profile import gear_metrics, gear_outline
from .params import GearboxParams

# involute-flank points per flank (4 reads cleanly + builds 1 clean solid in NX 2506)
_FLANK_PTS = 4

# component colours (RGB 0-255)
COL_HOUSING = (95, 105, 120)
COL_COVER = (110, 120, 135)
COL_GEAR = (150, 140, 90)
COL_SHAFT = (170, 172, 178)
COL_FLANGE = (90, 130, 175)
COL_OIL = (120, 90, 40)
COL_AIR = (0, 0, 0)

_SEG = 64   # polygon segments for a round XY footprint


# --------------------------------------------------------------------------- #
# housing footprint (local XY) -- a stadium/oval hull of the gear envelopes
# --------------------------------------------------------------------------- #
def _stadium_hull(centres: List[Tuple[float, float]], radii: List[float],
                  segs: int = _SEG) -> List[Tuple[float, float]]:
    """Closed convex polygon approximating the outer boundary of a set of discs
    (centre + radius). Sampled as the per-angle max support point over all discs --
    a smooth oval/stadium that wraps every gear envelope. Used for the cast shell
    footprint in the local (u, v) prism plane."""
    pts: List[Tuple[float, float]] = []
    for i in range(segs):
        th = 2.0 * math.pi * i / segs
        cx, sy = math.cos(th), math.sin(th)
        # farthest support point in direction th over all discs
        best = max(c[0] * cx + c[1] * sy + r for c, r in zip(centres, radii))
        pts.append((best * cx, best * sy))
    return pts


def _shrink_hull(poly: List[Tuple[float, float]], wall: float) -> List[Tuple[float, float]]:
    """Offset a (near-convex, origin-containing) footprint inward by `wall` along each
    vertex radius -- the inner cavity boundary of the cast shell. Robust for the
    stadium hull here (every vertex is radial from the origin region)."""
    out = []
    for (x, y) in poly:
        r = math.hypot(x, y) or 1.0
        s = max(0.0, r - wall) / r
        out.append((x * s, y * s))
    return out


# --------------------------------------------------------------------------- #
# REAL involute gears (toothed outline extruded on each parallel axis)
# --------------------------------------------------------------------------- #
def _placed_outline(centre: Tuple[float, float], module_mm: float, teeth: int,
                    pressure_angle_deg: float, profile_shift: float,
                    rotate_deg: float) -> List[Tuple[float, float]]:
    """The closed involute tooth outline (a single simple loop), PHASED by ``rotate_deg``
    about its own axis and TRANSLATED to the gear's axis (centre). Used as the prism
    profile (a 2D u,v polygon swept along +Z)."""
    out = gear_outline(module_mm, teeth, pressure_angle_deg=pressure_angle_deg,
                       profile_shift=profile_shift, flank_pts=_FLANK_PTS, rotate_deg=rotate_deg)
    return [(x + centre[0], y + centre[1]) for (x, y) in out]


def _bore_keyway(target: str, step_id: str, body_name: str, centre: Tuple[float, float],
                 bore_radius: float, key_width: float, key_depth: float,
                 z0: float, length: float, color) -> BuildStep:
    """DIN 6885-A parallel keyway cut into a press-fit BORE wall: a slot that opens at the
    bore (r = bore_radius) and reaches key_depth radially OUTWARD into the hub, centred on
    +X of the gear axis (a representative single key). Extruded through the gear face so
    the cut is clean. Mirrors motor_nx's shaft keyway (a radial rectangle subtract)."""
    hw = key_width / 2.0
    r_in = bore_radius - 0.5                         # start just inside the bore for a clean cut
    r_out = bore_radius + key_depth
    key = [(centre[0] + r_in, centre[1] - hw), (centre[0] + r_out, centre[1] - hw),
           (centre[0] + r_out, centre[1] + hw), (centre[0] + r_in, centre[1] + hw)]
    return BuildStep(
        id=step_id, role="gear_keyway_cut", kind="prism", boolean="subtract", target=target,
        body_name=body_name, material="air", color=COL_AIR, profile=key,
        origin3=(0.0, 0.0, z0 - 0.5), axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0),
        length=length + 1.0)


def _toothed_gear(step_id: str, body_name: str, centre: Tuple[float, float],
                  module_mm: float, teeth: int, pressure_angle_deg: float,
                  profile_shift: float, rotate_deg: float, z0: float, face_width: float,
                  *, unite_target: str = None, bore_diameter: float = 0.0,
                  key_width: float = 0.0, key_depth: float = 0.0) -> List[BuildStep]:
    """A REAL involute toothed gear (no longer a smooth pitch-diameter blank): the closed
    tooth outline is extruded (kind="prism") along +Z spanning z0 .. z0+face_width, PHASED
    by ``rotate_deg`` so it meshes cleanly with its mate. A gear is mounted ON a shaft, so
    it NEVER shares solid with it (ICD §7.6):

      * ``unite_target`` set -> the toothed solid is UNITED onto a shaft body modelled here
        (the layshaft cluster) so the rotating group is one body (the layshaft gears).
      * else, hub exists (bore radius < root radius) -> the toothed solid is CREATEd, then a
        central BORE (= mating shaft OD, a press fit) and a DIN 6885 KEYWAY (the torque
        connection) are SUBTRACTed (the output gear pressed on the diff input shaft).
      * else, NO hub (bore radius >= root radius) -> the gear is too small to bore: it is
        machined INTEGRAL with its shaft (a pinion cut on the rotor-shaft tip). It is built
        as a SOLID toothed pinion (no bore, no keyway -- there is nothing to press it onto in
        THIS part; the rotor shaft lives in motor_nx). This is the motor pinion (Ø45 shaft >
        Ø41 root): boring it would slice through the tooth roots and leave no hub, which is
        exactly the "tool body completely outside target" the keyway hit."""
    profile = _placed_outline(centre, module_mm, teeth, pressure_angle_deg,
                              profile_shift, rotate_deg)
    steps: List[BuildStep] = []
    if unite_target is not None:
        steps.append(BuildStep(
            id=step_id, role="gear", kind="prism", boolean="unite", target=unite_target,
            body_name=body_name, material="gear_steel", color=COL_GEAR, profile=profile,
            origin3=(0.0, 0.0, z0), axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0),
            length=face_width))
        return steps
    # external-shaft gear: create the toothed solid, then bore + keyway it to the shaft.
    steps.append(BuildStep(
        id=step_id, role="gear", kind="prism", boolean="create",
        body_name=body_name, material="gear_steel", color=COL_GEAR, profile=profile,
        origin3=(0.0, 0.0, z0), axis=(0.0, 0.0, 1.0), u_dir=(1.0, 0.0, 0.0),
        length=face_width))
    root_r = gear_metrics(module_mm, teeth, pressure_angle_deg,
                          profile_shift=profile_shift)["root_radius"]
    # only bore + key a gear that HAS a hub (the bore must stay inside the tooth roots so a
    # solid annular hub remains for the keyway to cut). A bore at/above the root would leave
    # no hub -> the gear is integral with its shaft (solid pinion), no bore / keyway.
    has_hub = bore_diameter and bore_diameter > 0.0 and (bore_diameter / 2.0) < root_r - 1e-6
    if has_hub:
        steps.append(BuildStep(
            id="%s_bore" % step_id, role="gear_bore_cut", kind="cylinder",
            boolean="subtract", target=step_id, body_name="%s_Bore" % body_name,
            material="air", color=COL_AIR, outer_radius=bore_diameter / 2.0,
            origin3=(centre[0], centre[1], z0 - 0.5), axis=(0.0, 0.0, 1.0),
            length=face_width + 1.0))
        if key_width > 0.0 and key_depth > 0.0:
            steps.append(_bore_keyway(
                step_id, "%s_keyway" % step_id, "%s_Keyway" % body_name, centre,
                bore_diameter / 2.0, key_width, key_depth, z0, face_width, COL_GEAR))
    return steps


def gear_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """The four REAL toothed gears of the 2-stage train, on their three axes, in the two
    axially-separated mesh bands, PHASED so each mesh interlocks (engineering.mesh_phasing).
    Each gear is mounted on its shaft WITHOUT sharing solid (ICD §7.6): the two layshaft-
    mounted gears (stage-1 gear + stage-2 pinion) are UNITED into the layshaft cluster
    (``layshaft.cluster_gears``); the motor pinion and the output gear are CREATEd then
    bored + keyed to their EXTERNAL mating shaft (a press fit). NOTE: the layshaft is
    created BEFORE these in build_steps() so the unite targets already exist."""
    steps: List[BuildStep] = []
    b1, b2 = bands["stage1"], bands["stage2"]
    ls = p.layshaft
    cluster = "layshaft" if ls.cluster_gears else None
    phase = engineering.mesh_phasing(p)
    s1, s2 = p.stage1, p.stage2

    # stage-1 mesh (band 1): motor pinion (on the motor rotor shaft) <-> layshaft gear
    steps.extend(_toothed_gear(
        "motor_pinion", "Motor_Pinion", pos["motor"], s1.module_mm, s1.pinion_teeth,
        s1.pressure_angle_deg, s1.pinion_profile_shift, phase["motor_pinion"],
        b1[0], s1.face_width_mm, bore_diameter=p.motor_pinion_bore_diameter_mm,
        key_width=p.motor_pinion_key.width_mm, key_depth=p.motor_pinion_key.depth_mm))
    steps.extend(_toothed_gear(
        "layshaft_gear", "Layshaft_Gear", pos["layshaft"], s1.module_mm, s1.gear_teeth,
        s1.pressure_angle_deg, s1.gear_profile_shift, phase["layshaft_gear"],
        b1[0], s1.face_width_mm, unite_target=cluster,
        bore_diameter=0.0 if ls.cluster_gears else ls.shaft_diameter_mm,
        key_width=ls.shaft_diameter_mm and 10.0, key_depth=ls.shaft_diameter_mm and 3.3))
    # stage-2 mesh (band 2): layshaft pinion <-> output gear (on the diff input shaft)
    steps.extend(_toothed_gear(
        "layshaft_pinion", "Layshaft_Pinion", pos["layshaft"], s2.module_mm, s2.pinion_teeth,
        s2.pressure_angle_deg, s2.pinion_profile_shift, phase["layshaft_pinion"],
        b2[0], s2.face_width_mm, unite_target=cluster,
        bore_diameter=0.0 if ls.cluster_gears else ls.shaft_diameter_mm,
        key_width=ls.shaft_diameter_mm and 10.0, key_depth=ls.shaft_diameter_mm and 3.3))
    steps.extend(_toothed_gear(
        "output_gear", "Output_Gear", pos["diff"], s2.module_mm, s2.gear_teeth,
        s2.pressure_angle_deg, s2.gear_profile_shift, phase["output_gear"],
        b2[0], s2.face_width_mm, bore_diameter=p.output.bore_diameter_mm,
        key_width=p.output.keyway_width_mm, key_depth=p.output.keyway_depth_mm))
    return steps


def layshaft_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """The intermediate (counter) shaft carrying the stage-1 gear + stage-2 pinion,
    journalled at both ends. The MAIN body (shaft_diameter) spans the gear region; each
    END is turned DOWN to ``bearing_seat_diameter`` over the bearing-seat length so the
    inner race of each layshaft bearing presses onto a real journal (bore = seat OD, no
    shared solid -- ICD §7.6). The two layshaft-mounted gears are UNITED onto this body in
    gear_steps() (cluster) so the rotating group is one solid -- no gear-vs-shaft overlap."""
    ls = p.layshaft
    b1, b2 = bands["stage1"], bands["stage2"]
    cavity_hi = g.housing_axial_length_mm
    bw = p.layshaft_bearing.width_mm
    # the layshaft reaches the bearings seated in the end covers: DE seat from below the
    # -Z cover up to the gear region, NDE seat from the gear region up through the +Z cover.
    z_lo = b1[0] - bw                                 # reaches the DE bearing (z = -bw .. 0)
    z_hi = cavity_hi + bw                             # reaches the NDE bearing (cavity .. +bw)
    cx, cy = pos["layshaft"]
    r_main = ls.shaft_diameter_mm / 2.0
    r_seat = ls.bearing_seat_diameter_mm / 2.0
    # main journal over the gear region (band 1 start .. band 2 end)
    steps: List[BuildStep] = [BuildStep(
        id="layshaft", role="layshaft", kind="cylinder", boolean="create",
        body_name="Layshaft", material="shaft_steel", color=COL_SHAFT,
        outer_radius=r_main, origin3=(cx, cy, b1[0]), axis=(0.0, 0.0, 1.0),
        length=b2[1] - b1[0])]
    # the two turned-down bearing-seat journals, UNITED on (overlap consumed by boolean)
    steps.append(BuildStep(
        id="layshaft_seat_de", role="layshaft", kind="cylinder", boolean="unite",
        target="layshaft", body_name="Layshaft_Seat_DE", material="shaft_steel",
        color=COL_SHAFT, outer_radius=r_seat, origin3=(cx, cy, z_lo), axis=(0.0, 0.0, 1.0),
        length=b1[0] - z_lo))
    steps.append(BuildStep(
        id="layshaft_seat_nde", role="layshaft", kind="cylinder", boolean="unite",
        target="layshaft", body_name="Layshaft_Seat_NDE", material="shaft_steel",
        color=COL_SHAFT, outer_radius=r_seat, origin3=(cx, cy, b2[1]), axis=(0.0, 0.0, 1.0),
        length=z_hi - b2[1]))
    if 0.0 < ls.bore_diameter_mm < ls.shaft_diameter_mm:
        steps.append(BuildStep(
            id="layshaft_bore", role="layshaft_bore_cut", kind="cylinder", boolean="subtract",
            target="layshaft", body_name="Layshaft_Bore", material="air", color=COL_AIR,
            outer_radius=ls.bore_diameter_mm / 2.0,
            origin3=(cx, cy, z_lo - 0.5), axis=(0.0, 0.0, 1.0),
            length=(z_hi - z_lo) + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# rolling bearings (representative race rings at each shaft journal)
# --------------------------------------------------------------------------- #
def bearing_seats(p: GearboxParams, g, pos, bands) -> List[Dict[str, Any]]:
    """The bearing seats as placement dicts: which shaft axis, the journal OD the inner race
    presses onto, the BearingParams, the axial z-start, and a label. The inner-race bore =
    the journal OD (a press fit, no shared solid -- ICD §7.6); the outer race seats in a
    housing counterbore pocket (= bearing OD) cut from the cover.

    Layshaft (the only shaft fully internal to the gearbox) runs in TWO bearings in the
    end covers, flush at the cavity face and protruding OUTWARD so neither enters the cavity
    to clash a gear. The OUTPUT shaft runs in one bearing in the +Z cover, placed OUTBOARD of
    the output-coupling flange (z >= the coupling outer face) so the race rings never bury
    into the coupling solid. The MOTOR-PINION shaft gets NO gearbox bearing: the pinion is
    machined integral with the motor ROTOR shaft (Ø45 journal > Ø41 root, so it has no hub to
    bore), and that rotor is journalled by the motor's OWN DE/NDE bearings -- a gearbox
    bearing there would only bury into the motor-mount flange (the e-axle layout supports the
    input pinion on the motor)."""
    ls = p.layshaft
    b1, b2 = bands["stage1"], bands["stage2"]
    cavity_lo = b1[0]                                 # cavity / -Z cover inner face (z = 0)
    cavity_hi = g.housing_axial_length_mm             # cavity / +Z cover inner face
    lb, ob = p.layshaft_bearing, p.output_shaft_bearing
    lay_journal = ls.bearing_seat_diameter_mm         # the layshaft turns down to this at its ends
    coupling_face = b2[1] + p.output.flange_thickness_mm   # +Z outer face of the output coupling
    seats = [
        # layshaft DE: in the -Z cover, flush at the cavity face, protruding outward
        dict(label="layshaft_de", axis=pos["layshaft"], journal_od=lay_journal,
             bearing=lb, z0=cavity_lo - lb.width_mm),
        # layshaft NDE: in the +Z cover, flush at the cavity face, protruding outward
        dict(label="layshaft_nde", axis=pos["layshaft"], journal_od=lay_journal,
             bearing=lb, z0=cavity_hi),
        # output shaft: OUTBOARD of the output coupling flange (starts at the coupling outer
        # face) so the race rings clear the coupling solid; seats in the +Z cover.
        dict(label="output_shaft", axis=pos["diff"], journal_od=p.output.bore_diameter_mm,
             bearing=ob, z0=coupling_face),
    ]
    return seats


def bearing_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """Each bearing seat as concentric RACE rings (ICD §7.6 press fits, no shared solid):
    an INNER race (bore = journal OD, OD just above it), a representative ROLLING-ELEMENT
    ring centred on the pitch diameter, and an OUTER race (OD = bearing OD, ID just below
    it). Modelled as TUBE bodies (outer-create - inner-subtract inside the engine, never a
    self-intersecting annulus) coaxial with the shaft. The inner bore touches the shaft and
    the outer OD touches the housing counterbore pocket -- touching, not overlapping."""
    steps: List[BuildStep] = []
    for seat in bearing_seats(p, g, pos, bands):
        brg = seat["bearing"]
        cx, cy = seat["axis"]
        z0 = seat["z0"]
        B = brg.width_mm
        r_bore = seat["journal_od"] / 2.0            # inner-race bore = the journal it grips
        r_od = brg.od_d_mm / 2.0                      # outer-race OD = housing bore pocket
        # race wall thickness ~ a fraction of the (OD - bore) radial span
        span = r_od - r_bore
        t = max(2.0, 0.18 * 2.0 * span)               # each race ring radial wall
        ring_t = min(brg.ball_ring_thickness_mm, max(1.0, span - 2.0 * t - 1.0))
        r_inner_out = r_bore + t                      # inner-race OD
        r_outer_in = r_od - t                          # outer-race ID
        r_mid = 0.5 * (r_bore + r_od)                  # rolling-element pitch radius
        label = seat["label"]
        common = dict(origin3=(cx, cy, z0), axis=(0.0, 0.0, 1.0), length=B)
        steps.append(BuildStep(
            id="bearing_%s_inner" % label, role="bearing", kind="tube", boolean="create",
            body_name="Bearing_%s_InnerRace" % label, material="bearing_steel",
            color=COL_SHAFT, outer_radius=r_inner_out, inner_radius=r_bore, **common))
        steps.append(BuildStep(
            id="bearing_%s_rolling" % label, role="bearing", kind="tube", boolean="create",
            body_name="Bearing_%s_Rolling" % label, material="bearing_steel",
            color=COL_SHAFT, outer_radius=r_mid + ring_t / 2.0,
            inner_radius=r_mid - ring_t / 2.0, **common))
        steps.append(BuildStep(
            id="bearing_%s_outer" % label, role="bearing", kind="tube", boolean="create",
            body_name="Bearing_%s_OuterRace" % label, material="bearing_steel",
            color=COL_SHAFT, outer_radius=r_od, inner_radius=r_outer_in, **common))
    return steps


# --------------------------------------------------------------------------- #
# cast housing shell (prism) + end covers + flanges + oil sump
# --------------------------------------------------------------------------- #
def housing_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    h = p.housing
    steps: List[BuildStep] = []

    # gear envelopes for the footprint hull (tip radius = pitch r + a small addendum) --
    # the SAME disc set engineering.derive() bounds, so the reported oval and the built
    # shell agree (review finding 6).
    centres, radii = engineering.gear_envelopes(p, g, pos)
    # the cast wall stands `radial_clearance + wall` proud of the gear tips
    outer = _stadium_hull(centres, [r + h.radial_clearance_mm + h.wall_thickness_mm
                                    for r in radii])
    inner = _stadium_hull(centres, [r + h.radial_clearance_mm for r in radii])

    z0 = bands["stage1"][0] - h.end_cover_thickness_mm
    cavity_len = g.housing_axial_length_mm
    full_len = cavity_len + 2.0 * h.end_cover_thickness_mm

    # 1) solid cast block (footprint extruded along the gear axis), then hollow it
    steps.append(BuildStep(
        id="housing_shell", role="housing", kind="prism", boolean="create",
        body_name="Gearbox_Housing", material="cast_aluminium", color=COL_HOUSING,
        profile=outer, origin3=(0.0, 0.0, z0), axis=(0.0, 0.0, 1.0),
        u_dir=(1.0, 0.0, 0.0), length=full_len))
    # 2) hollow the gear cavity (leave the two end covers as the cast walls)
    steps.append(BuildStep(
        id="housing_cavity", role="housing_cavity_cut", kind="prism", boolean="subtract",
        target="housing_shell", body_name="Gearbox_Cavity", material="air", color=COL_AIR,
        profile=inner, origin3=(0.0, 0.0, bands["stage1"][0]), axis=(0.0, 0.0, 1.0),
        u_dir=(1.0, 0.0, 0.0), length=cavity_len))

    # 2b) BEARING BORES -- each shaft passes THROUGH the cast end covers via a clearance
    #     bore, so the shaft sits in the bore and never shares solid with the housing
    #     (ICD §7.6: the only reported housing<->layshaft clash was the layshaft piercing
    #     the solid -Z end cover). One clearance hole per shaft axis, subtracted through
    #     the FULL housing length so it pierces BOTH covers; placed coaxial with the
    #     shaft. The bore overlaps the cast covers (it cuts real metal), not just air, so
    #     NX has a target to subtract from. Bore Ø = shaft OD + 2*clearance.
    bore_z0 = z0 - 0.5
    bore_len = full_len + 1.0
    # (axis centre, shaft OD, body label) -- the three parallel shaft axes
    shaft_axes = [
        (pos["layshaft"], p.layshaft.shaft_diameter_mm, "Layshaft"),
        (pos["motor"], p.motor_pinion_bore_diameter_mm, "Motor"),
        (pos["diff"], p.output.bore_diameter_mm, "Output"),
    ]
    for (cx, cy), shaft_od, label in shaft_axes:
        if shaft_od <= 0.0:
            continue
        steps.append(BuildStep(
            id="bearing_bore_%s" % label.lower(), role="bearing_bore_cut",
            kind="cylinder", boolean="subtract", target="housing_shell",
            body_name="%s_Bearing_Bore" % label, material="air", color=COL_AIR,
            outer_radius=shaft_od / 2.0 + h.bearing_bore_clearance_mm,
            origin3=(cx, cy, bore_z0), axis=(0.0, 0.0, 1.0), length=bore_len))

    # 2c) BEARING POCKETS (counterbores) -- each bearing's OUTER race seats in a bore in
    #     the cast wall (= bearing OD). Subtract a pocket = bearing OD over the bearing
    #     width (+ a seating margin) coaxial with the shaft, so the outer race fills the
    #     pocket (touching, not overlapping, ICD §7.6). Targets the housing shell.
    for seat in bearing_seats(p, g, pos, bands):
        brg = seat["bearing"]
        cx, cy = seat["axis"]
        steps.append(BuildStep(
            id="bearing_pocket_%s" % seat["label"], role="bearing_pocket_cut",
            kind="cylinder", boolean="subtract", target="housing_shell",
            body_name="Bearing_Pocket_%s" % seat["label"], material="air", color=COL_AIR,
            outer_radius=brg.od_d_mm / 2.0,
            origin3=(cx, cy, seat["z0"] - 0.5), axis=(0.0, 0.0, 1.0),
            length=brg.width_mm + 1.0))

    # 2d) OUTPUT-COUPLING THROUGH-BORE -- the output coupling flange (Ø = the driveline
    #     input flange) pokes OUT through the +Z (diff) end cover to mate with the diff
    #     input. Bore a clearance counterbore in the cover (= coupling OD + clearance)
    #     coaxial with the diff axis over the cover so the coupling passes through without
    #     burying into the cast end cover (ICD §7.6). The coupling spans b2[1] .. + flange
    #     thickness; it crosses the cover from cavity_hi outward.
    coupling_face = bands["stage2"][1] + p.output.flange_thickness_mm
    cb_z0 = cavity_len - 0.5                          # from the +Z cavity face
    cb_len = (coupling_face - cavity_len) + 1.0       # through where the coupling exits
    steps.append(BuildStep(
        id="output_coupling_clearance", role="output_clearance_cut", kind="cylinder",
        boolean="subtract", target="housing_shell", body_name="Output_Coupling_Clearance",
        material="air", color=COL_AIR,
        outer_radius=p.output.flange_diameter_mm / 2.0 + h.bearing_bore_clearance_mm,
        origin3=(0.0, 0.0, cb_z0), axis=(0.0, 0.0, 1.0), length=cb_len))

    # 3) MOTOR-MOUNTING FLANGE -- a disc on the diff-side end cover (z0 face), coaxial
    #    with the MOTOR axis, matched to the motor DE flange. The motor bolts to this.
    mf = engineering.resolve_motor_flange(p)
    mz0 = z0 - h.motor_flange_thickness_mm        # stands proud on the -Z (motor) face
    steps.append(BuildStep(
        id="motor_flange", role="motor_flange", kind="cylinder", boolean="create",
        body_name="Motor_Mount_Flange", material="cast_aluminium", color=COL_FLANGE,
        outer_radius=mf["flange_diameter_mm"] / 2.0,
        origin3=(pos["motor"][0], pos["motor"][1], mz0), axis=(0.0, 0.0, 1.0),
        length=h.motor_flange_thickness_mm))
    # pilot spigot bore (the motor DE spigot pilots into this register)
    steps.append(BuildStep(
        id="motor_pilot", role="motor_pilot_cut", kind="cylinder", boolean="subtract",
        target="motor_flange", body_name="Motor_Pilot_Bore", material="air", color=COL_AIR,
        outer_radius=mf["pilot_diameter_mm"] / 2.0,
        origin3=(pos["motor"][0], pos["motor"][1], mz0 - 0.5), axis=(0.0, 0.0, 1.0),
        length=h.motor_flange_thickness_mm + 1.0))
    # LAYSHAFT-DE-BEARING CLEARANCE RELIEF -- the layshaft DE bearing seats in the -Z cover
    #   (z = -B .. 0) but the bearing is wider than the 12 mm cover, so its -Z face protrudes
    #   into the motor-flange axial band (the big Ø298 flange disc, on the motor axis, sweeps
    #   over the layshaft axis 90 mm away and would swallow the bearing). Cast a local relief
    #   pocket in the bell (= bearing OD + clearance, coaxial with the LAYSHAFT axis) so the
    #   bearing clears -- this does NOT change the flange OD / bolt circle / pilot register /
    #   mating face (the relief sits between the Ø226 pilot bore and the bolt circle, a real
    #   bell-housing bearing relief). Only cut if the bearing actually reaches the flange band.
    lde = next((s for s in bearing_seats(p, g, pos, bands) if s["label"] == "layshaft_de"), None)
    if lde is not None:
        bz0 = lde["z0"]                                  # bearing -Z face
        if bz0 < z0:                                     # protrudes past the cover into the flange
            lb = lde["bearing"]
            relief_lo = bz0 - 0.5
            relief_hi = z0 + 0.5                         # up through the flange +Z (cover outer) face
            steps.append(BuildStep(
                id="motor_flange_layshaft_relief", role="motor_bearing_relief_cut",
                kind="cylinder", boolean="subtract", target="motor_flange",
                body_name="Motor_Flange_Layshaft_Bearing_Relief", material="air", color=COL_AIR,
                outer_radius=lb.od_d_mm / 2.0 + h.bearing_bore_clearance_mm,
                origin3=(pos["layshaft"][0], pos["layshaft"][1], relief_lo),
                axis=(0.0, 0.0, 1.0), length=relief_hi - relief_lo))
    # motor bolt circle (explicit instances around the flange's OWN centre -- the
    # engine's circular pattern rotates about the GLOBAL Z (diff axis), which for an
    # OFFSET flange would swing the bolts off the flange; mirror driveline_nx's fix).
    if mf["bolt_count"] > 0 and mf["bolt_diameter_mm"] > 0:
        bc = mf["bolt_circle_diameter_mm"] / 2.0
        n = int(mf["bolt_count"])
        for i in range(n):
            th = 2.0 * math.pi * i / n
            steps.append(BuildStep(
                id="motor_flange_bolt_%d" % i, role="motor_bolt_cut", kind="cylinder",
                boolean="subtract", target="motor_flange",
                body_name="Motor_Bolt_%d" % i, material="air", color=COL_AIR,
                outer_radius=mf["bolt_diameter_mm"] / 2.0,
                origin3=(pos["motor"][0] + bc * math.cos(th),
                         pos["motor"][1] + bc * math.sin(th), mz0 - 0.5),
                axis=(0.0, 0.0, 1.0), length=h.motor_flange_thickness_mm + 1.0))

    # 4) DIFFERENTIAL-CARRIER MOUNT -- a flange on the +Z (diff/output) end cover,
    #    coaxial with the DIFF axis, with a carrier bore that seats the diff carrier OD.
    dz0 = z0 + full_len                              # +Z (output) face
    steps.append(BuildStep(
        id="diff_mount_flange", role="diff_mount", kind="cylinder", boolean="create",
        body_name="Diff_Mount_Flange", material="cast_aluminium", color=COL_FLANGE,
        outer_radius=h.diff_mount_flange_diameter_mm / 2.0,
        origin3=(0.0, 0.0, dz0), axis=(0.0, 0.0, 1.0),
        length=h.diff_mount_thickness_mm))
    # carrier seating bore through the diff-mount flange (receives the carrier OD)
    steps.append(BuildStep(
        id="diff_carrier_bore", role="diff_carrier_cut", kind="cylinder", boolean="subtract",
        target="diff_mount_flange", body_name="Diff_Carrier_Bore", material="air", color=COL_AIR,
        outer_radius=h.diff_carrier_diameter_mm / 2.0, cx=0.0, cy=0.0,
        z0=dz0 - 0.5, length=h.diff_mount_thickness_mm + 1.0))
    # diff-mount bolt circle (patterned about the diff axis = global Z, so a standard
    # circular pattern works here -- this flange IS coaxial with the origin)
    if h.diff_mount_bolt_count > 0 and h.diff_mount_bolt_diameter_mm > 0:
        pr = 0.5 * (h.diff_carrier_diameter_mm / 2.0 + h.diff_mount_flange_diameter_mm / 2.0)
        steps.append(BuildStep(
            id="diff_mount_bolt", role="diff_bolt_cut", kind="cylinder", boolean="subtract",
            target="diff_mount_flange", body_name="Diff_Mount_Bolt", material="air", color=COL_AIR,
            outer_radius=h.diff_mount_bolt_diameter_mm / 2.0, cx=pr, cy=0.0,
            z0=dz0 - 0.5, length=h.diff_mount_thickness_mm + 1.0,
            pattern_count=h.diff_mount_bolt_count,
            pattern_angle_deg=360.0 / h.diff_mount_bolt_count))

    # 5) OIL SUMP -- a cast box united below the lowest gear (the splash-lube bath).
    #    "Down" in the local frame is -v of the gear-axis plane; here the gears spread
    #    toward +u/+v (motor up-and-over), so the sump hangs on the LOW side (-Y local,
    #    which becomes -Z = downward in the vehicle after Rx(-90)). Modelled as a prism
    #    slab on the -Y face spanning the cavity length.
    if h.oil_sump_depth_mm > 0:
        # seat the sump at the SHELL inner-wall LOW point (the oval hull's -v extreme),
        # NOT at -(env_r - wall): the single radius placed the sump ~133 mm below the
        # oval wall, detaching it. y_top is the real inner-wall low v so the slab unites
        # to the shell (review finding 6).
        y_top = g.housing_sump_v_low_mm
        half_w = 0.6 * (g.housing_envelope_short_mm / 2.0)
        sump = [(-half_w, y_top - h.oil_sump_depth_mm), (half_w, y_top - h.oil_sump_depth_mm),
                (half_w, y_top), (-half_w, y_top)]
        steps.append(BuildStep(
            id="oil_sump", role="oil_sump", kind="prism", boolean="unite",
            target="housing_shell", body_name="Oil_Sump", material="cast_aluminium",
            color=COL_HOUSING, profile=sump,
            origin3=(0.0, 0.0, bands["stage1"][0]), axis=(0.0, 0.0, 1.0),
            u_dir=(1.0, 0.0, 0.0), length=cavity_len))

    return steps


# --------------------------------------------------------------------------- #
# output coupling (gearbox output -> diff input flange)
# --------------------------------------------------------------------------- #
def output_coupling_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """The gearbox OUTPUT -> differential coupling: a flange on the output-gear hub,
    coaxial with the diff axis, sized to the driveline diff input flange (the
    driveline's diff_input_pinion/flange is the mating interface). Bored + bolt-circled
    to receive the diff input flange / coupling shaft."""
    o = p.output
    steps: List[BuildStep] = []
    # the coupling flange sits just past the output gear's +Z face, toward the diff mount
    zc = bands["stage2"][1]
    steps.append(BuildStep(
        id="output_coupling", role="output_coupling", kind="cylinder", boolean="create",
        body_name="Output_Coupling_Flange", material="gear_steel", color=COL_GEAR,
        outer_radius=o.flange_diameter_mm / 2.0, cx=0.0, cy=0.0,
        z0=zc, length=o.flange_thickness_mm))
    steps.append(BuildStep(
        id="output_coupling_bore", role="output_bore_cut", kind="cylinder", boolean="subtract",
        target="output_coupling", body_name="Output_Coupling_Bore", material="air", color=COL_AIR,
        outer_radius=o.bore_diameter_mm / 2.0, cx=0.0, cy=0.0,
        z0=zc - 0.5, length=o.flange_thickness_mm + 1.0))
    if o.bolt_count > 0 and o.bolt_diameter_mm > 0:
        pr = 0.5 * (o.bore_diameter_mm / 2.0 + o.flange_diameter_mm / 2.0)
        steps.append(BuildStep(
            id="output_coupling_bolt", role="output_bolt_cut", kind="cylinder", boolean="subtract",
            target="output_coupling", body_name="Output_Coupling_Bolt", material="air", color=COL_AIR,
            outer_radius=o.bolt_diameter_mm / 2.0, cx=pr, cy=0.0,
            z0=zc - 0.5, length=o.flange_thickness_mm + 1.0,
            pattern_count=o.bolt_count, pattern_angle_deg=360.0 / o.bolt_count))
    return steps


# --------------------------------------------------------------------------- #
# build-step assembly
# --------------------------------------------------------------------------- #
def build_steps(p: GearboxParams) -> List[BuildStep]:
    g = engineering.derive(p)
    pos = engineering.axis_positions(p)
    bands = engineering.axial_bands(p)
    steps: List[BuildStep] = []
    steps.extend(housing_steps(p, g, pos, bands))
    # the LAYSHAFT is created BEFORE the gear blanks: the two layshaft-mounted blanks are
    # UNITED into it as one rotating cluster (gear_steps), so the unite target must exist.
    steps.extend(layshaft_steps(p, g, pos, bands))
    steps.extend(gear_steps(p, g, pos, bands))
    steps.extend(output_coupling_steps(p, g, pos, bands))
    # bearings LAST: their housing counterbore pockets are cut in housing_steps, so the
    # race rings drop into the seated pockets (own bodies, not booleaned into others).
    steps.extend(bearing_steps(p, g, pos, bands))
    return steps


def generate(p: GearboxParams = None) -> Dict[str, Any]:
    """Full gearbox blueprint dict (NX-independent). Same schema as motor_nx so the
    same run_journal builder consumes it."""
    if p is None:
        p = GearboxParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "gearbox_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": g.housing_axial_length_mm,   # advisory; no active-stack here
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
