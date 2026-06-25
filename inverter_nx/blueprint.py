"""Pure-math inverter geometry, emitted as the SAME ordered build-step list the NX
builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary (extrude /
cylinder / tube + boolean create/subtract/unite, plus the axis-placed ``hole`` cut for
radial coolant ports), so motor_nx's hardened NXOpen engine builds the inverter with no
new geometry code.

Coordinate convention (inverter LOCAL frame)
    Z = stacking / height axis.  X = enclosure length, Y = enclosure width.
    The enclosure footprint is centred on (0, 0).

    >>> MOUNTING-FACE DATUM <<<
    The enclosure BASE face lies in the plane z = 0 and is centred on (0, 0). That
    point -- ``(0, 0, 0)`` in the local frame -- is the MOUNTING-FACE DATUM the vehicle
    assembly uses to sit the inverter on top of the motor (ICD section 3: "inverter
    local datum = mounting face centre", placed "above the motor"). All geometry grows
    in +Z from this face, so the assembly transform is a pure translation that lands
    this datum on the motor's top. ``engineering.layout().mounting_face_xyz`` exposes it.

The power devices, DC link, busbars and connectors are modelled as representative
BLANKS (the "envelope" philosophy motor_nx uses for end-windings) -- the cold plate,
enclosure walls, ports, bolt pattern and module footprints are modelled to size for
packaging + thermal studies, not as routed/wire-bonded detail.
"""

from __future__ import annotations

from typing import Any, Dict, List

from dataclasses import asdict

from motor_nx.blueprint import BuildStep   # reuse the proven, version-independent step
from . import engineering
from .params import InverterParams

# component colours (RGB 0-255)
COL_ENCLOSURE = (70, 110, 160)
COL_COLDPLATE = (150, 155, 165)
COL_CAP = (90, 95, 105)
COL_MODULE = (40, 40, 45)
COL_CONNECTOR = (184, 115, 51)
COL_BUSBAR = (200, 130, 60)
COL_AIR = (0, 0, 0)


def _rect(width_x: float, width_y: float) -> List[List[float]]:
    """Closed rectangular XY profile centred on (0, 0): width_x along X, width_y along Y."""
    hx, hy = width_x / 2.0, width_y / 2.0
    return [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]


def _rect_at(cx: float, cy: float, width_x: float, width_y: float) -> List[List[float]]:
    """Closed rectangular XY profile centred on (cx, cy)."""
    hx, hy = width_x / 2.0, width_y / 2.0
    return [[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]]


# --------------------------------------------------------------------------- #
# enclosure (the sealed HV housing -- box, hollowed, with a top sealing flange,
# the lid bolt pattern, and the HV / phase / LV connectors)
# --------------------------------------------------------------------------- #
def enclosure_steps(p: InverterParams) -> List[BuildStep]:
    e = p.enclosure
    lay = engineering.layout(p)
    steps: List[BuildStep] = []

    # outer box: extrude the full footprint from z=0 (the MOUNTING-FACE DATUM) up by the
    # full height. z = 0 is the face that sits on the motor.
    steps.append(BuildStep(
        id="enclosure", role="enclosure", kind="extrude", boolean="create",
        body_name="Inverter_Enclosure", material="aluminium", color=COL_ENCLOSURE,
        profile=_rect(e.length_mm, e.width_mm), z0=0.0, length=e.height_mm))
    # hollow it: subtract a smaller box, leaving the floor + walls (lid is open on top)
    inner_l, inner_w = lay.inner_l, lay.inner_w
    steps.append(BuildStep(
        id="enclosure_cavity", role="enclosure_cavity_cut", kind="extrude", boolean="subtract",
        target="enclosure", body_name="Enclosure_Cavity", material="air", color=COL_AIR,
        profile=_rect(inner_l, inner_w), z0=e.wall_mm, length=e.height_mm))

    # raised LID SEALING FLANGE: a rectangular ring lip on top of the wall the lid bolts
    # down onto (gasket seat). Built robustly the same way as the enclosure shell -- a
    # solid outer slab united on, then an inner pocket subtracted -- so it is a ring lip
    # (wall thickness + flange width) not a solid cap. The lid bolt circle goes here.
    if e.lid_flange_mm > 0.0 and e.lid_flange_thickness_mm > 0.0:
        flange_inner_l = inner_l - 2.0 * e.lid_flange_mm
        flange_inner_w = inner_w - 2.0 * e.lid_flange_mm
        steps.append(BuildStep(
            id="enclosure_lid_flange", role="enclosure", kind="extrude", boolean="unite",
            target="enclosure", body_name="Lid_Sealing_Flange", material="aluminium",
            color=COL_ENCLOSURE,
            profile=_rect(e.length_mm, e.width_mm),
            z0=e.height_mm, length=e.lid_flange_thickness_mm))
        steps.append(BuildStep(
            id="enclosure_lid_flange_pocket", role="enclosure_cavity_cut", kind="extrude",
            boolean="subtract", target="enclosure", body_name="Lid_Flange_Pocket",
            material="air", color=COL_AIR,
            profile=_rect(flange_inner_l, flange_inner_w),
            z0=e.height_mm - 0.5, length=e.lid_flange_thickness_mm + 1.0))
        # lid bolt pattern: tapped holes around the perimeter on the flange centre-line.
        steps.extend(_lid_bolt_steps(p))

    # phase + HV + LV connectors: bosses united onto the enclosure end walls, placed
    # ABOVE the cold plate so the cables clear the internals.
    z_conn = max(lay.plate_top_z + 6.0, e.height_mm / 2.0 - 8.0)
    if e.connector_count > 0:
        pitch = inner_w / (e.connector_count + 1)
        for i in range(e.connector_count):
            cy = -inner_w / 2.0 + pitch * (i + 1)
            steps.append(BuildStep(
                id="phase_connector_%d" % (i + 1), role="connector", kind="cylinder", boolean="unite",
                target="enclosure", body_name="Phase_Connector_%d" % (i + 1),
                material="connector", color=COL_CONNECTOR, outer_radius=8.0,
                cx=e.length_mm / 2.0, cy=cy, z0=z_conn, length=16.0))
    if e.hv_connector:
        steps.append(BuildStep(
            id="hv_connector", role="connector", kind="cylinder", boolean="unite",
            target="enclosure", body_name="HV_DC_Connector", material="connector",
            color=COL_CONNECTOR, outer_radius=12.0, cx=-e.length_mm / 2.0, cy=-inner_w * 0.25,
            z0=z_conn, length=20.0))
    if e.lv_connector:
        # the low-voltage signal/control connector (CAN-FD, gate, resolver, interlock)
        # on the same -X end wall, offset +Y from the HV connector for HV/LV separation.
        steps.append(BuildStep(
            id="lv_connector", role="connector", kind="cylinder", boolean="unite",
            target="enclosure", body_name="LV_Signal_Connector", material="connector",
            color=COL_CONNECTOR, outer_radius=7.0, cx=-e.length_mm / 2.0, cy=inner_w * 0.28,
            z0=z_conn, length=14.0))
    return steps


def _lid_bolt_steps(p: InverterParams) -> List[BuildStep]:
    """The lid bolt pattern: ``lid_bolt_count`` tapped holes drilled DOWN (-Z) through
    the raised sealing flange, distributed evenly around the rectangular perimeter on a
    bolt line inset ``lid_bolt_inset_mm`` from the outer wall. Each is an axis-placed
    +Z hole cut; positions are computed in pure Python so the test can check them."""
    e = p.enclosure
    n = e.lid_bolt_count
    if n <= 0 or e.lid_bolt_diameter_mm <= 0.0:
        return []
    bx = e.length_mm / 2.0 - e.lid_bolt_inset_mm   # bolt-line half-extent in X
    by = e.width_mm / 2.0 - e.lid_bolt_inset_mm    # bolt-line half-extent in Y
    pts = _perimeter_points(bx, by, n)
    z_top = e.height_mm + e.lid_flange_thickness_mm
    depth = e.lid_flange_thickness_mm + e.wall_mm   # through the flange + into the wall top
    steps: List[BuildStep] = []
    for i, (px, py) in enumerate(pts):
        steps.append(BuildStep(
            id="lid_bolt_%d" % (i + 1), role="lid_bolt_cut", kind="hole", boolean="subtract",
            target="enclosure", body_name="Lid_Bolt_%d" % (i + 1), material="air", color=COL_AIR,
            outer_radius=e.lid_bolt_diameter_mm / 2.0,
            cx=px, cy=py, z0=z_top + 0.5, length=depth + 1.0, axis=(0.0, 0.0, -1.0)))
    return steps


def _perimeter_points(bx: float, by: float, n: int) -> List[List[float]]:
    """``n`` points spread evenly (by arc length) around the rectangle with half-extents
    (bx, by) centred on the origin. Deterministic + pure so tests can verify the bolt
    pattern lies on the flange centre-line."""
    if n <= 0:
        return []
    # perimeter parameterised 0..P; walk it in n equal steps starting at a corner.
    seg = [bx, by, bx, by]              # +X edge half, then quarter lengths conceptually
    P = 2.0 * (2.0 * bx + 2.0 * by)
    pts: List[List[float]] = []
    for i in range(n):
        t = (i / n) * P
        # corners: start at (+bx, -by) going CCW
        if t <= 2.0 * by:               # right edge: x=+bx, y -by->+by
            pts.append([bx, -by + t])
        elif t <= 2.0 * by + 2.0 * bx:  # top edge: y=+by, x +bx->-bx
            d = t - 2.0 * by
            pts.append([bx - d, by])
        elif t <= 4.0 * by + 2.0 * bx:  # left edge: x=-bx, y +by->-by
            d = t - (2.0 * by + 2.0 * bx)
            pts.append([-bx, by - d])
        else:                           # bottom edge: y=-by, x -bx->+bx
            d = t - (4.0 * by + 2.0 * bx)
            pts.append([-bx + d, -by])
    return pts


# --------------------------------------------------------------------------- #
# cold plate + coolant ports + power stage + DC link + busbars
# (the internals sitting on the enclosure floor)
# --------------------------------------------------------------------------- #
def internal_steps(p: InverterParams) -> List[BuildStep]:
    c, d = p.cooling, p.dc_link
    lay = engineering.layout(p)
    steps: List[BuildStep] = []

    # liquid cold plate -- a flat slab on the enclosure floor
    steps.append(BuildStep(
        id="cold_plate", role="cold_plate", kind="extrude", boolean="create",
        body_name="Liquid_Cold_Plate", material="aluminium", color=COL_COLDPLATE,
        profile=_rect(c.coldplate_length_mm, c.coldplate_width_mm),
        z0=lay.floor_z, length=c.coldplate_thickness_mm))

    # coolant inlet + outlet ports: bored RADIALLY into the cold-plate -X end face (where
    # the fittings thread in), one each side of the plate centre by +/- port_pitch/2 in Y,
    # at the plate mid-thickness. The bore runs +X into the plate (representative of the
    # internal flow channel feed). Axis-placed 'hole' cuts -> never registered as bodies.
    if c.port_diameter_mm > 0.0:
        z_mid = lay.floor_z + c.coldplate_thickness_mm / 2.0
        x_face = -c.coldplate_length_mm / 2.0
        bore_depth = c.coldplate_length_mm * 0.35
        for sid, name, sign in (("coolant_inlet", "Coolant_Inlet_Port", -1.0),
                                 ("coolant_outlet", "Coolant_Outlet_Port", 1.0)):
            steps.append(BuildStep(
                id=sid, role="coolant_port_cut", kind="hole", boolean="subtract",
                target="cold_plate", body_name=name, material="air", color=COL_AIR,
                outer_radius=c.port_diameter_mm / 2.0,
                cx=x_face - 0.5, cy=sign * c.port_pitch_mm / 2.0, z0=z_mid,
                length=bore_depth + 0.5, axis=(1.0, 0.0, 0.0)))

    # six SiC power modules in a row on the cold plate (one per switch position)
    for i, cx in enumerate(lay.module_xs):
        steps.append(BuildStep(
            id="power_module_%d" % (i + 1), role="power_module", kind="extrude", boolean="create",
            body_name="SiC_Power_Module_%d" % (i + 1), material="power_module", color=COL_MODULE,
            profile=_rect_at(cx, lay.module_y, lay.module_len, lay.module_wid),
            z0=lay.plate_top_z, length=lay.module_hgt))

    # DC-link capacitor block -- a slab beside the module row (toward +Y)
    steps.append(BuildStep(
        id="dc_link_cap", role="dc_link", kind="extrude", boolean="create",
        body_name="DC_Link_Capacitor", material="film_cap", color=COL_CAP,
        profile=_rect_at(lay.cap_cx, lay.cap_cy, lay.cap_len, lay.cap_wid),
        z0=lay.plate_top_z, length=lay.cap_hgt))

    # representative laminated DC busbar tying the cap to the module row (a +/- pair of
    # copper bars standing off above the modules, spanning the cap length in X).
    #
    # The bars LAND ON the DC-link cap terminal face, they do NOT bury into the cap: each
    # bar bolts to a short terminal PAD that is UNITED onto the cap (one solid with it),
    # and the bar's +Y edge meets that pad's outer face -- a touching mating contact (the
    # inverter_nx interpenetration fix). The pad overlaps INTO the cap so the unite is
    # robust (NX lesson: every unite must overlap its target); it stands proud of the cap
    # -Y face by terminal_pad_proj_mm, and the bar +Y edge stops exactly there.
    if p.busbar.enabled:
        b = p.busbar
        bus_x = lay.cap_len   # span the cap length in X
        cy = lay.busbar_cy    # both bars share Y; their +Y edge lands on the pad face
        bar_pos_y = cy + b.width_mm / 2.0   # the +Y (cap-side) edge of each bar
        # the two bars stack in Z above the modules; both touch the same pad face in Y.
        for k, name in enumerate(("DC_Busbar_Pos", "DC_Busbar_Neg")):
            z0 = lay.plate_top_z + lay.module_hgt + b.height_mm + k * (b.thickness_mm + 0.5)
            steps.append(BuildStep(
                id="busbar_%d" % k, role="busbar", kind="extrude", boolean="create",
                body_name=name, material="copper", color=COL_BUSBAR,
                profile=_rect_at(0.0, cy, bus_x, b.width_mm),
                z0=z0, length=b.thickness_mm))
            # terminal pad united onto the cap at this bar's height: a small block that
            # OVERLAPS into the cap (robust unite) and stands proud of the cap -Y face so
            # the bar's +Y edge butts against it. The pad spans the bar thickness in Z and
            # is centred on X. Its outer (-Y) face sits at bar_pos_y, so the bar touches it.
            if b.terminal_pad_proj_mm > 0.0:
                pad_y0 = bar_pos_y                         # pad outer face (the bar lands here)
                pad_y1 = lay.cap_terminal_face_y + 2.0     # 2 mm INTO the cap -> robust unite
                pad_cy = (pad_y0 + pad_y1) / 2.0
                pad_len_y = pad_y1 - pad_y0
                pad_z0 = z0 - 1.0                          # bracket the bar thickness in Z
                pad_len_z = b.thickness_mm + 2.0
                steps.append(BuildStep(
                    id="busbar_pad_%d" % k, role="busbar", kind="extrude", boolean="unite",
                    target="dc_link_cap", body_name="%s_Terminal_Pad" % name,
                    material="copper", color=COL_BUSBAR,
                    profile=_rect_at(0.0, pad_cy, b.terminal_pad_width_mm, pad_len_y),
                    z0=pad_z0, length=pad_len_z))
    return steps


def build_steps(p: InverterParams) -> List[BuildStep]:
    steps = enclosure_steps(p)
    steps.extend(internal_steps(p))
    return steps


def generate(p: InverterParams = None) -> Dict[str, Any]:
    """Full inverter blueprint dict (NX-independent). Same schema as motor_nx so the
    same run_journal builder consumes it. The blueprint metadata carries the
    MOUNTING-FACE DATUM the vehicle assembly mounts on (see module docstring)."""
    if p is None:
        p = InverterParams()
    g = engineering.derive(p)
    lay = engineering.layout(p)
    steps = build_steps(p)
    return {
        "schema": "inverter_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": p.enclosure.height_mm,   # advisory; the enclosure height
        "mounting_face_xyz": list(lay.mounting_face_xyz),  # the datum the assembly sits on the motor
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
