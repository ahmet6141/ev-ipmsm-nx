"""Pure-math driveline geometry, emitted as the SAME ordered build-step list the
NX builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary
(tube / cylinder / extrude / revolve / hole + boolean create/subtract/unite),
so motor_nx's hardened NXOpen engine builds a driveline with no new geometry code.

Coordinate convention
    Z = driveline (wheel) rotation axis. Differential centre at z = 0.
    Right half-shaft grows toward +Z, left toward -Z (symmetric).
    Gears + spline teeth are represented as BLANKS at pitch/major diameter
    (cut later by hobbing/broaching) -- the same "envelope" philosophy motor_nx
    uses for end-windings; the production interfaces (flanges, bolt circles,
    bores, keyways) are modelled exactly.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import DrivelineParams

# component colours (RGB 0-255)
COL_CARRIER = (90, 95, 105)
COL_GEAR = (150, 140, 90)
COL_SHAFT = (170, 172, 178)
COL_JOINT = (110, 115, 125)
COL_HUB = (120, 130, 150)
COL_BEARING = (60, 62, 70)
COL_AIR = (0, 0, 0)


def _axial(sign: int, z_ref: float, length: float):
    """Return (z0, next_ref) for a body of `length` grown OUTWARD from z_ref in the
    +Z (sign=+1) or -Z (sign=-1) direction. z0 is always the lower-Z face."""
    if sign >= 0:
        return z_ref, z_ref + length
    return z_ref - length, z_ref - length


# --------------------------------------------------------------------------- #
# differential (centre, shared by both sides)
# --------------------------------------------------------------------------- #
def differential_steps(p: DrivelineParams) -> List[BuildStep]:
    d = p.differential
    steps: List[BuildStep] = []
    half = d.carrier_length / 2.0

    # carrier / case -- a tube blank centred on z=0
    steps.append(BuildStep(
        id="diff_carrier", role="diff_carrier", kind="tube", boolean="create",
        body_name="Differential_Carrier", material="cast_iron", color=COL_CARRIER,
        outer_radius=d.carrier_outer_diameter / 2.0,
        inner_radius=d.carrier_outer_diameter / 2.0 - d.carrier_wall,
        z0=-half, length=d.carrier_length))

    # ring gear BLANK -- a tube on the +Z face of the carrier (at pitch diameter)
    steps.append(BuildStep(
        id="ring_gear", role="ring_gear", kind="tube", boolean="create",
        body_name="Ring_Gear_Blank", material="gear_steel", color=COL_GEAR,
        outer_radius=d.ring_gear_pitch_diameter / 2.0,
        inner_radius=d.carrier_outer_diameter / 2.0,
        z0=half, length=d.ring_gear_face_width))
    # ring-gear-to-carrier bolt circle (through the ring gear web)
    if d.ring_gear_bolt_count > 0 and d.ring_gear_bolt_diameter > 0:
        pr = 0.5 * (d.carrier_outer_diameter / 2.0 + d.ring_gear_pitch_diameter / 2.0)
        steps.append(BuildStep(
            id="ring_gear_bolt", role="gear_bolt_cut", kind="cylinder", boolean="subtract",
            target="ring_gear", body_name="Ring_Gear_Bolt", material="air", color=COL_AIR,
            outer_radius=d.ring_gear_bolt_diameter / 2.0, cx=pr, cy=0.0,
            z0=half - 0.5, length=d.ring_gear_face_width + 1.0,
            pattern_count=d.ring_gear_bolt_count,
            pattern_angle_deg=360.0 / d.ring_gear_bolt_count))

    # side-gear / output bosses (one per modelled side), inside the carrier
    for tag, sign in _sides(p):
        z0, _ = _axial(sign, sign * (half - d.side_gear_length), d.side_gear_length)
        steps.append(BuildStep(
            id="side_gear_%s" % tag, role="side_gear", kind="cylinder", boolean="create",
            body_name="Side_Gear_%s" % tag.upper(), material="gear_steel", color=COL_GEAR,
            outer_radius=d.side_gear_diameter / 2.0, cx=0.0, cy=0.0,
            z0=z0, length=d.side_gear_length))

    # input pinion + motor-coupling flange (parallel-axis: offset in +X by the
    # ring+pinion centre distance). The flange bore = the motor drive-stub Ø.
    offset = 0.5 * (d.ring_gear_pitch_diameter + d.input_pinion_pitch_diameter)
    zc = half + d.ring_gear_face_width / 2.0
    steps.append(BuildStep(
        id="diff_input_pinion", role="input_pinion", kind="cylinder", boolean="create",
        body_name="Input_Pinion_Blank", material="gear_steel", color=COL_GEAR,
        outer_radius=d.input_pinion_pitch_diameter / 2.0, cx=offset, cy=0.0,
        z0=zc - d.ring_gear_face_width / 2.0, length=d.ring_gear_face_width))
    steps.append(BuildStep(
        id="diff_input_flange", role="input_flange", kind="cylinder", boolean="create",
        body_name="Motor_Coupling_Flange", material="gear_steel", color=COL_GEAR,
        outer_radius=d.input_flange_diameter / 2.0, cx=offset, cy=0.0,
        z0=zc + d.ring_gear_face_width / 2.0, length=d.input_flange_thickness))
    # central bore that receives the keyed motor stub
    steps.append(BuildStep(
        id="diff_input_bore", role="input_bore_cut", kind="cylinder", boolean="subtract",
        target="diff_input_flange", body_name="Input_Bore", material="air", color=COL_AIR,
        outer_radius=d.input_bore_diameter / 2.0, cx=offset, cy=0.0,
        z0=zc + d.ring_gear_face_width / 2.0 - 0.5, length=d.input_flange_thickness + 1.0))
    # input keyway (matches the motor DIN 6885-A key) -- a rectangular slot in the bore
    if d.input_keyway_width > 0 and d.input_keyway_depth > 0:
        rb = d.input_bore_diameter / 2.0
        hw = d.input_keyway_width / 2.0
        key = [(offset - hw, rb - 0.5), (offset + hw, rb - 0.5),
               (offset + hw, rb + d.input_keyway_depth), (offset - hw, rb + d.input_keyway_depth)]
        steps.append(BuildStep(
            id="diff_input_keyway", role="input_keyway_cut", kind="extrude", boolean="subtract",
            target="diff_input_flange", body_name="Input_Keyway", material="air", color=COL_AIR,
            profile=key, z0=zc + d.ring_gear_face_width / 2.0 - 0.5,
            length=d.input_flange_thickness + 1.0))
    # flange-to-coupling bolt circle. The flange is PARALLEL-AXIS (offset in +X), but
    # the engine's circular pattern rotates about the GLOBAL Z (the wheel axis at the
    # origin) -- so a patterned bolt would swing about the origin and fall completely
    # outside the offset flange (NX: "Tool body completely outside target body"). Emit
    # EXPLICIT instances around the flange's OWN centre (offset, 0) instead.
    if d.input_flange_bolt_count > 0 and d.input_flange_bolt_diameter > 0:
        bc = 0.5 * (d.input_bore_diameter / 2.0 + d.input_flange_diameter / 2.0)
        n = d.input_flange_bolt_count
        for i in range(n):
            th = 2.0 * math.pi * i / n
            steps.append(BuildStep(
                id="diff_input_flange_bolt_%d" % i, role="flange_bolt_cut", kind="cylinder",
                boolean="subtract", target="diff_input_flange",
                body_name="Coupling_Bolt_%d" % i, material="air", color=COL_AIR,
                outer_radius=d.input_flange_bolt_diameter / 2.0,
                cx=offset + bc * math.cos(th), cy=bc * math.sin(th),
                z0=zc + d.ring_gear_face_width / 2.0 - 0.5,
                length=d.input_flange_thickness + 1.0))
    return steps


# --------------------------------------------------------------------------- #
# half-shaft + CV joints + wheel hub (one chain per modelled side)
# --------------------------------------------------------------------------- #
def side_steps(p: DrivelineParams, tag: str, sign: int) -> List[BuildStep]:
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    steps: List[BuildStep] = []
    U = tag.upper()
    # Start just outside the carrier, after the SOLVED inboard plunge clearance. This
    # gap (engineering.inboard_clearance) is sized so the chain's final wheel-hub
    # flange OUTER face lands at local z = +-target_track/2 (ICD §3/§4 track tie),
    # i.e. on the shared HUB_CENTRE after the assembly Rx(-90). It is the one slack
    # term in the chain; every other length is a catalogue component dimension.
    clearance = engineering.inboard_clearance(p)
    ref = sign * (d.carrier_length / 2.0 + clearance)   # start just outside the carrier

    def hollow(shaft_id: str, z0: float, length: float):
        """Bore a coaxial hole through a just-created shaft body (hollow shaft)."""
        if 0.0 < h.bore_diameter < h.diameter:
            steps.append(BuildStep(
                id="%s_bore" % shaft_id, role="halfshaft_bore_cut", kind="cylinder",
                boolean="subtract", target=shaft_id, body_name="Halfshaft_Bore_%s" % U,
                material="air", color=COL_AIR, outer_radius=h.bore_diameter / 2.0,
                cx=0.0, cy=0.0, z0=z0 - 0.5, length=length + 1.0))

    # inboard plunging (tripod) CV joint -- a bell housing blank
    z0, ref = _axial(sign, ref, h.inboard_bell_length)
    steps.append(BuildStep(
        id="cv_inboard_%s" % tag, role="cv_joint", kind="cylinder", boolean="create",
        body_name="CV_Inboard_%s_%s" % (h.inboard_joint, U), material="joint_steel",
        color=COL_JOINT, outer_radius=h.inboard_bell_diameter / 2.0, cx=0.0, cy=0.0,
        z0=z0, length=h.inboard_bell_length))

    # the half-shaft bar
    z0, ref = _axial(sign, ref, h.length)
    sid = "halfshaft_%s" % tag
    steps.append(BuildStep(
        id=sid, role="halfshaft", kind="cylinder", boolean="create",
        body_name="Halfshaft_%s" % U, material="halfshaft_steel", color=COL_SHAFT,
        outer_radius=h.diameter / 2.0, cx=0.0, cy=0.0, z0=z0, length=h.length))
    hollow(sid, z0, h.length)

    # outboard fixed (Rzeppa) CV joint -- a bell housing blank
    z0, ref = _axial(sign, ref, h.outboard_bell_length)
    steps.append(BuildStep(
        id="cv_outboard_%s" % tag, role="cv_joint", kind="cylinder", boolean="create",
        body_name="CV_Outboard_%s_%s" % (h.outboard_joint, U), material="joint_steel",
        color=COL_JOINT, outer_radius=h.outboard_bell_diameter / 2.0, cx=0.0, cy=0.0,
        z0=z0, length=h.outboard_bell_length))

    # Gen-3 wheel-hub bearing unit (tube blank)
    z0, ref = _axial(sign, ref, w.bearing_width)
    bearing_z0 = z0
    steps.append(BuildStep(
        id="hub_bearing_%s" % tag, role="hub_bearing", kind="tube", boolean="create",
        body_name="Hub_Bearing_%s" % U, material="bearing_steel", color=COL_BEARING,
        outer_radius=w.bearing_outer_diameter / 2.0, inner_radius=w.bearing_bore_diameter / 2.0,
        z0=z0, length=w.bearing_width))

    # ABS / wheel-speed encoder ring (thin tube on the bearing's inboard face --
    # the face toward the differential, where the wheel-speed sensor sits)
    if w.abs_encoder_ring and w.encoder_ring_width > 0:
        # inboard face: low-Z end (z0) for the +Z side, high-Z end for the -Z side
        ez0 = bearing_z0 if sign >= 0 else bearing_z0 + w.bearing_width - w.encoder_ring_width
        steps.append(BuildStep(
            id="encoder_ring_%s" % tag, role="encoder_ring", kind="tube", boolean="create",
            body_name="ABS_Encoder_Ring_%s" % U, material="encoder", color=COL_BEARING,
            outer_radius=w.encoder_ring_diameter / 2.0,
            inner_radius=w.encoder_ring_diameter / 2.0 - 4.0,
            z0=ez0, length=w.encoder_ring_width))

    # wheel-mounting flange (the wheel bolts to this face)
    z0, ref = _axial(sign, ref, w.hub_flange_thickness)
    fid = "hub_flange_%s" % tag
    steps.append(BuildStep(
        id=fid, role="wheel_hub", kind="cylinder", boolean="create",
        body_name="Wheel_Hub_Flange_%s" % U, material="hub_steel", color=COL_HUB,
        outer_radius=w.hub_flange_diameter / 2.0, cx=0.0, cy=0.0,
        z0=z0, length=w.hub_flange_thickness))
    # centre pilot (hub-centric) bore through the flange
    steps.append(BuildStep(
        id="hub_pilot_%s" % tag, role="hub_pilot_cut", kind="cylinder", boolean="subtract",
        target=fid, body_name="Hub_Pilot_Bore_%s" % U, material="air", color=COL_AIR,
        outer_radius=w.pilot_bore_diameter / 2.0, cx=0.0, cy=0.0,
        z0=z0 - 0.5, length=w.hub_flange_thickness + 1.0))
    # brake-disc centring pilot boss (optional raised ring)
    if w.brake_pilot_diameter > 0 and w.brake_pilot_height > 0:
        bz0, _ = _axial(sign, ref, w.brake_pilot_height)
        steps.append(BuildStep(
            id="brake_pilot_%s" % tag, role="wheel_hub", kind="tube", boolean="unite",
            target=fid, body_name="Brake_Pilot_%s" % U, material="hub_steel", color=COL_HUB,
            outer_radius=w.brake_pilot_diameter / 2.0,
            inner_radius=w.pilot_bore_diameter / 2.0, z0=bz0, length=w.brake_pilot_height))

    # WHEEL CONNECTION ELEMENTS: lug bolt circle OR a single centre-lock nut seat
    if w.single_centre_nut:
        steps.append(BuildStep(
            id="hub_centre_nut_%s" % tag, role="lug_cut", kind="cylinder", boolean="subtract",
            target=fid, body_name="Centre_Nut_Seat_%s" % U, material="air", color=COL_AIR,
            outer_radius=w.centre_nut_diameter / 2.0, cx=0.0, cy=0.0,
            z0=z0 - 0.5, length=w.hub_flange_thickness + 1.0))
    elif w.lug_count > 0 and w.lug_hole_diameter > 0:
        steps.append(BuildStep(
            id="hub_lug_%s" % tag, role="lug_cut", kind="cylinder", boolean="subtract",
            target=fid, body_name="Lug_Hole_%s" % U, material="air", color=COL_AIR,
            outer_radius=w.lug_hole_diameter / 2.0, cx=w.lug_pcd / 2.0, cy=0.0,
            z0=z0 - 0.5, length=w.hub_flange_thickness + 1.0,
            pattern_count=w.lug_count, pattern_angle_deg=360.0 / w.lug_count))
    return steps


def _sides(p: DrivelineParams):
    """Yield (tag, sign) for each modelled side."""
    if p.sides == "left":
        return [("l", -1)]
    if p.sides == "right":
        return [("r", +1)]
    return [("l", -1), ("r", +1)]


def build_steps(p: DrivelineParams) -> List[BuildStep]:
    steps = differential_steps(p)
    for tag, sign in _sides(p):
        steps.extend(side_steps(p, tag, sign))
    return steps


def generate(p: DrivelineParams = None) -> Dict[str, Any]:
    """Full driveline blueprint dict (NX-independent). Same schema as motor_nx so
    the same run_journal builder consumes it."""
    if p is None:
        p = DrivelineParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "driveline_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": g.total_track_length_mm,   # advisory; no active-stack here
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
