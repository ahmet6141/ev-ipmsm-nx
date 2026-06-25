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
from .params import GearboxParams

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
# gear blanks (axis-placed cylinders on the three parallel axes)
# --------------------------------------------------------------------------- #
def _gear(step_id: str, body_name: str, centre: Tuple[float, float],
          pitch_diameter: float, z0: float, face_width: float,
          bore_diameter: float = 0.0, *, unite_target: str = None) -> BuildStep:
    """A gear BLANK at pitch diameter, coaxial with +Z through (centre_x, centre_y),
    spanning z0 .. z0+face_width. A gear is mounted ON a shaft, so it is NEVER a bare
    solid disc that overlaps the shaft it sits on (ICD §7.6 forbids two solids sharing
    metal). Exactly one of:

      * ``unite_target`` set -> the blank is UNITED into a shaft body modelled in THIS
        part (the layshaft cluster): a solid disc booleaned onto the shaft so the two
        become one rotating body (they really do turn together). NX uniting two
        overlapping coaxial cylinders is clean.
      * ``bore_diameter`` > 0 -> the blank is a TUBE bored to the mating shaft OD (a
        press-fit ring seated on an EXTERNAL shaft -- the motor rotor shaft, the diff
        input shaft -- that lives in a neighbouring part). bore = shaft OD => press fit.
      * neither -> a plain solid disc (only valid when nothing shares its axis)."""
    if unite_target is not None:
        # solid disc united onto the shaft cluster (overlap with the shaft is intended
        # and is consumed by the boolean -- one body results, so no interpenetration).
        return BuildStep(
            id=step_id, role="gear", kind="cylinder", boolean="unite",
            target=unite_target, body_name=body_name, material="gear_steel", color=COL_GEAR,
            outer_radius=pitch_diameter / 2.0,
            origin3=(centre[0], centre[1], z0), axis=(0.0, 0.0, 1.0),
            length=face_width)
    if bore_diameter and 0.0 < bore_diameter < pitch_diameter:
        # press-fit ring: an atomic TUBE (outer create - inner subtract inside the engine,
        # NEVER a self-intersecting annulus profile) bored to the mating shaft OD.
        return BuildStep(
            id=step_id, role="gear", kind="tube", boolean="create",
            body_name=body_name, material="gear_steel", color=COL_GEAR,
            outer_radius=pitch_diameter / 2.0, inner_radius=bore_diameter / 2.0,
            origin3=(centre[0], centre[1], z0), axis=(0.0, 0.0, 1.0),
            length=face_width)
    return BuildStep(
        id=step_id, role="gear", kind="cylinder", boolean="create",
        body_name=body_name, material="gear_steel", color=COL_GEAR,
        outer_radius=pitch_diameter / 2.0,
        origin3=(centre[0], centre[1], z0), axis=(0.0, 0.0, 1.0),
        length=face_width)


def gear_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """The four gear blanks of the 2-stage train, on their three axes, in the two
    axially-separated mesh bands. Each blank is mounted on its shaft WITHOUT sharing
    solid (ICD §7.6): the two layshaft-mounted blanks (stage-1 gear + stage-2 pinion)
    are UNITED into the layshaft cluster (``layshaft.cluster_gears``) or bored to the
    shaft OD; the motor pinion and the output gear are bored to their EXTERNAL mating
    shaft OD (a press fit). NOTE: the layshaft is created BEFORE these in build_steps()
    so the unite targets already exist."""
    steps: List[BuildStep] = []
    b1, b2 = bands["stage1"], bands["stage2"]
    ls = p.layshaft
    cluster = "layshaft" if ls.cluster_gears else None
    # a layshaft-mounted blank that is NOT clustered gets a bore = shaft OD (press fit)
    lay_bore = 0.0 if ls.cluster_gears else ls.shaft_diameter_mm

    # stage-1 mesh (band 1): motor pinion (on the motor rotor shaft) <-> layshaft gear
    steps.append(_gear("motor_pinion", "Motor_Pinion_Blank", pos["motor"],
                       g.motor_pinion_pd_mm, b1[0], p.stage1.face_width_mm,
                       bore_diameter=p.motor_pinion_bore_diameter_mm))
    steps.append(_gear("layshaft_gear", "Layshaft_Gear_Blank", pos["layshaft"],
                       g.layshaft_gear_pd_mm, b1[0], p.stage1.face_width_mm,
                       bore_diameter=lay_bore, unite_target=cluster))
    # stage-2 mesh (band 2): layshaft pinion <-> output gear (on the diff input shaft)
    steps.append(_gear("layshaft_pinion", "Layshaft_Pinion_Blank", pos["layshaft"],
                       g.layshaft_pinion_pd_mm, b2[0], p.stage2.face_width_mm,
                       bore_diameter=lay_bore, unite_target=cluster))
    steps.append(_gear("output_gear", "Output_Gear_Blank", pos["diff"],
                       g.output_gear_pd_mm, b2[0], p.stage2.face_width_mm,
                       bore_diameter=p.output.bore_diameter_mm))
    return steps


def layshaft_steps(p: GearboxParams, g, pos, bands) -> List[BuildStep]:
    """The intermediate (counter) shaft carrying the stage-1 gear + stage-2 pinion,
    journalled at both ends. A cylinder coaxial with the layshaft axis spanning a
    bearing seat below band 1 to a bearing seat above band 2. The two layshaft-mounted
    gear blanks are UNITED onto this body in gear_steps() (cluster) so the rotating
    group is one solid -- no gear-vs-shaft interpenetration."""
    ls = p.layshaft
    b1, b2 = bands["stage1"], bands["stage2"]
    z_lo = b1[0] - ls.bearing_seat_length_mm
    z_hi = b2[1] + ls.bearing_seat_length_mm
    steps: List[BuildStep] = [BuildStep(
        id="layshaft", role="layshaft", kind="cylinder", boolean="create",
        body_name="Layshaft", material="shaft_steel", color=COL_SHAFT,
        outer_radius=ls.shaft_diameter_mm / 2.0,
        origin3=(pos["layshaft"][0], pos["layshaft"][1], z_lo), axis=(0.0, 0.0, 1.0),
        length=z_hi - z_lo)]
    if 0.0 < ls.bore_diameter_mm < ls.shaft_diameter_mm:
        steps.append(BuildStep(
            id="layshaft_bore", role="layshaft_bore_cut", kind="cylinder", boolean="subtract",
            target="layshaft", body_name="Layshaft_Bore", material="air", color=COL_AIR,
            outer_radius=ls.bore_diameter_mm / 2.0,
            origin3=(pos["layshaft"][0], pos["layshaft"][1], z_lo - 0.5), axis=(0.0, 0.0, 1.0),
            length=(z_hi - z_lo) + 1.0))
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
