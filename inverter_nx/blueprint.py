"""Pure-math inverter geometry, emitted as the SAME ordered build-step list the NX
builder consumes (motor_nx.blueprint schema). NX-independent + unit-tested.

It reuses motor_nx.blueprint.BuildStep and its primitive vocabulary (extrude /
cylinder + boolean create/subtract/unite), so motor_nx's hardened NXOpen engine builds
the inverter with no new geometry code.

Coordinate convention
    Z = stacking / height axis. Cold-plate base sits at z = 0; the power modules sit on
    it, the DC-link capacitor block beside them, all inside the HV enclosure box. The
    enclosure footprint is centred on (0, 0); X = length, Y = width.

The power devices, DC link and connectors are modelled as representative BLANKS (the
"envelope" philosophy motor_nx uses for end-windings) -- the cold plate, enclosure
walls and module footprints are modelled to size for packaging + thermal studies.
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
COL_AIR = (0, 0, 0)

_NUM_POWER_MODULES = 6   # three half-bridges = six SiC switch positions


def _rect(width_x: float, width_y: float) -> List[List[float]]:
    """Closed rectangular XY profile centred on (0, 0): width_x along X, width_y along Y."""
    hx, hy = width_x / 2.0, width_y / 2.0
    return [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]


def _rect_at(cx: float, cy: float, width_x: float, width_y: float) -> List[List[float]]:
    """Closed rectangular XY profile centred on (cx, cy)."""
    hx, hy = width_x / 2.0, width_y / 2.0
    return [[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]]


# --------------------------------------------------------------------------- #
# enclosure (the sealed HV housing -- box, then hollowed)
# --------------------------------------------------------------------------- #
def enclosure_steps(p: InverterParams) -> List[BuildStep]:
    e = p.enclosure
    steps: List[BuildStep] = []

    # outer box: extrude the full footprint from z=0 up by the full height
    steps.append(BuildStep(
        id="enclosure", role="enclosure", kind="extrude", boolean="create",
        body_name="Inverter_Enclosure", material="aluminium", color=COL_ENCLOSURE,
        profile=_rect(e.length_mm, e.width_mm), z0=0.0, length=e.height_mm))
    # hollow it: subtract a smaller box, leaving the floor + walls (lid is open)
    inner_l = e.length_mm - 2.0 * e.wall_mm
    inner_w = e.width_mm - 2.0 * e.wall_mm
    steps.append(BuildStep(
        id="enclosure_cavity", role="enclosure_cavity_cut", kind="extrude", boolean="subtract",
        target="enclosure", body_name="Enclosure_Cavity", material="air", color=COL_AIR,
        profile=_rect(inner_l, inner_w), z0=e.wall_mm, length=e.height_mm))

    # phase + HV connectors: bosses united onto the enclosure on the +X end wall
    if e.connector_count > 0:
        pitch = inner_w / (e.connector_count + 1)
        for i in range(e.connector_count):
            cy = -inner_w / 2.0 + pitch * (i + 1)
            steps.append(BuildStep(
                id="phase_connector_%d" % (i + 1), role="connector", kind="cylinder", boolean="unite",
                target="enclosure", body_name="Phase_Connector_%d" % (i + 1),
                material="connector", color=COL_CONNECTOR, outer_radius=8.0,
                cx=e.length_mm / 2.0, cy=cy, z0=e.height_mm / 2.0 - 8.0, length=16.0))
    if e.hv_connector:
        steps.append(BuildStep(
            id="hv_connector", role="connector", kind="cylinder", boolean="unite",
            target="enclosure", body_name="HV_DC_Connector", material="connector",
            color=COL_CONNECTOR, outer_radius=12.0, cx=-e.length_mm / 2.0, cy=0.0,
            z0=e.height_mm / 2.0 - 10.0, length=20.0))
    return steps


# --------------------------------------------------------------------------- #
# cold plate + power stage + DC link (the internals sitting on the enclosure floor)
# --------------------------------------------------------------------------- #
def internal_steps(p: InverterParams) -> List[BuildStep]:
    e, c, d = p.enclosure, p.cooling, p.dc_link
    steps: List[BuildStep] = []
    floor = e.wall_mm   # internals stack from the inner floor up

    # liquid cold plate -- a flat slab on the enclosure floor
    steps.append(BuildStep(
        id="cold_plate", role="cold_plate", kind="extrude", boolean="create",
        body_name="Liquid_Cold_Plate", material="aluminium", color=COL_COLDPLATE,
        profile=_rect(c.coldplate_length_mm, c.coldplate_width_mm),
        z0=floor, length=c.coldplate_thickness_mm))

    plate_top = floor + c.coldplate_thickness_mm

    # six SiC power modules in a row on the cold plate (one per switch position)
    mod_len = 26.0
    mod_wid = 40.0
    mod_hgt = 14.0
    row_span = c.coldplate_length_mm * 0.7
    spacing = row_span / (_NUM_POWER_MODULES - 1) if _NUM_POWER_MODULES > 1 else 0.0
    x0 = -row_span / 2.0
    mod_y = -c.coldplate_width_mm * 0.18   # modules sit toward -Y; DC link toward +Y
    for i in range(_NUM_POWER_MODULES):
        cx = x0 + spacing * i
        steps.append(BuildStep(
            id="power_module_%d" % (i + 1), role="power_module", kind="extrude", boolean="create",
            body_name="SiC_Power_Module_%d" % (i + 1), material="power_module", color=COL_MODULE,
            profile=_rect_at(cx, mod_y, mod_len, mod_wid), z0=plate_top, length=mod_hgt))

    # DC-link capacitor block -- a slab beside the module row (toward +Y)
    cap_len = c.coldplate_length_mm * 0.8
    cap_wid = c.coldplate_width_mm * 0.35
    cap_hgt = 30.0
    cap_y = c.coldplate_width_mm * 0.28
    steps.append(BuildStep(
        id="dc_link_cap", role="dc_link", kind="extrude", boolean="create",
        body_name="DC_Link_Capacitor", material="film_cap", color=COL_CAP,
        profile=_rect_at(0.0, cap_y, cap_len, cap_wid), z0=plate_top, length=cap_hgt))
    return steps


def build_steps(p: InverterParams) -> List[BuildStep]:
    steps = enclosure_steps(p)
    steps.extend(internal_steps(p))
    return steps


def generate(p: InverterParams = None) -> Dict[str, Any]:
    """Full inverter blueprint dict (NX-independent). Same schema as motor_nx so the
    same run_journal builder consumes it."""
    if p is None:
        p = InverterParams()
    g = engineering.derive(p)
    steps = build_steps(p)
    return {
        "schema": "inverter_nx.blueprint/1",
        "name": p.name,
        "units": "mm",
        "axis": "Z",
        "stack_length": p.enclosure.height_mm,   # advisory; the enclosure height
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
