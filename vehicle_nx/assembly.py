"""NX-independent assembly math: turn the vehicle layout into a PLAN -- one entry
per component with its source part file and its placement (origin + 3x3 orientation
matrix) in vehicle coordinates. The NX journal consumes this plan verbatim.

Each subsystem part was modelled in its OWN local frame; this module records the
documented local-frame -> vehicle-frame rotation for each:

  * driveline / motor : local +Z = the rotation axis. In the vehicle the axle runs
                        left-right (along +Y), so local +Z -> vehicle +Y via Rx(-90).
  * inverter          : box in its own frame, mounted upright on the motor (identity).
  * suspension corner : local +X long, +Y outboard, +Z up == vehicle axes. The left
                        corner is identity; the right corner is Rz(180) so its
                        outboard side faces -Y.
  * chassis           : placed as the platform at the origin (identity).

Exact bolt-hole mating is a downstream NX constraint step; this plan positions every
part parametrically so the assembly opens already laid out (the same "representative"
philosophy the subsystem blueprints use).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from .params import VehicleParams

Mat = List[List[float]]
Vec = List[float]


# --------------------------------------------------------------------------- #
# small rotation helpers (3x3 row-major; vehicle_vec = R . local_vec)
# --------------------------------------------------------------------------- #
def identity() -> Mat:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def rot_x(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rot_y(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_z(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def matmul(a: Mat, b: Mat) -> Mat:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _round_mat(m: Mat) -> Mat:
    return [[round(v, 9) for v in row] for row in m]


def _component(name: str, part_file: str, role: str, origin: Vec, orient: Mat) -> Dict[str, Any]:
    return {
        "name": name,
        "part_file": part_file,
        "role": role,
        "origin_mm": [round(v, 3) for v in origin],
        "orientation": _round_mat(orient),
    }


# --------------------------------------------------------------------------- #
# the plan
# --------------------------------------------------------------------------- #
def _driven_axles(layout: str) -> List[str]:
    return {"rear": ["rear"], "front": ["front"], "awd": ["front", "rear"]}.get(layout, ["rear"])


def components(p: VehicleParams) -> List[Dict[str, Any]]:
    """Ordered component list with placements (vehicle frame)."""
    L, e, f = p.layout, p.eaxle, p.parts
    z_hub = L.tyre_radius_mm
    x_rear = -L.wheelbase_mm / 2.0
    x_front = +L.wheelbase_mm / 2.0
    axle_x = {"rear": x_rear, "front": x_front}
    track = {"rear": L.track_rear_mm, "front": L.track_front_mm}

    out: List[Dict[str, Any]] = []

    # 1) chassis -- the platform at the origin
    if f.include_chassis:
        out.append(_component("CHASSIS", f.chassis, "chassis", [0.0, 0.0, 0.0], identity()))

    # 2) e-axle(s): driveline + motor (+ inverter) at each driven axle
    eaxle_rot = rot_x(-90.0)   # local +Z (rotation axis) -> vehicle +Y (axle left-right)
    for ax in _driven_axles(L.drive_layout):
        ax_x = axle_x[ax]
        out.append(_component(
            "DRIVELINE_%s" % ax.upper(), f.driveline, "driveline",
            [ax_x, 0.0, z_hub], eaxle_rot))
        out.append(_component(
            "MOTOR_%s" % ax.upper(), f.motor, "motor",
            [ax_x - e.motor_offset_x_mm, 0.0, z_hub + e.motor_offset_z_mm], eaxle_rot))
        if f.include_inverter:
            out.append(_component(
                "INVERTER_%s" % ax.upper(), f.inverter, "inverter",
                [ax_x - e.motor_offset_x_mm + e.inverter_offset_x_mm, 0.0,
                 z_hub + e.motor_offset_z_mm + e.inverter_offset_z_mm], identity()))

    # 3) suspension corners
    if p.layout.suspension_corners >= 4:
        axles = ["front", "rear"]
    else:
        axles = _driven_axles(L.drive_layout)
    for ax in axles:
        for side, sign in (("L", +1.0), ("R", -1.0)):
            y = sign * track[ax] / 2.0
            orient = identity() if sign > 0 else rot_z(180.0)
            out.append(_component(
                "SUSPENSION_%s%s" % (ax[0].upper(), side), f.suspension, "suspension",
                [axle_x[ax], y, z_hub], orient))
    return out


def build_plan(p: VehicleParams = None) -> Dict[str, Any]:
    if p is None:
        p = VehicleParams()
    comps = components(p)
    return {
        "schema": "vehicle_nx.assembly/1",
        "name": p.name,
        "units": "mm",
        "frame": "ISO8855 (+X fwd, +Y left, +Z up)",
        "parameters": p.to_dict(),
        "validation": validate(p),
        "components": comps,
    }


def to_json(plan: Dict[str, Any], indent: int = 2) -> str:
    import json
    return json.dumps(plan, indent=indent)


# --------------------------------------------------------------------------- #
# checks + report
# --------------------------------------------------------------------------- #
def validate(p: VehicleParams) -> List[str]:
    issues: List[str] = []
    L = p.layout
    if L.drive_layout not in ("rear", "front", "awd"):
        issues.append("layout.drive_layout '%s' unknown (rear|front|awd)" % L.drive_layout)
    if L.suspension_corners not in (2, 4):
        issues.append("layout.suspension_corners must be 2 or 4")
    for nm, v in (("wheelbase_mm", L.wheelbase_mm), ("track_front_mm", L.track_front_mm),
                  ("track_rear_mm", L.track_rear_mm), ("tyre_radius_mm", L.tyre_radius_mm)):
        if v <= 0:
            issues.append("layout.%s must be > 0" % nm)
    # the motor must clear the ground (axle height + offset - half the motor OD-ish)
    if L.tyre_radius_mm + p.eaxle.motor_offset_z_mm <= 0:
        issues.append("motor placed below ground (check eaxle.motor_offset_z_mm)")
    # no two non-chassis components may share an identical origin (overlap)
    seen = {}
    for c in components(p):
        if c["role"] == "chassis":
            continue
        key = tuple(c["origin_mm"])
        if key in seen:
            issues.append("components %s and %s share an origin %s" % (seen[key], c["name"], key))
        else:
            seen[key] = c["name"]
    return issues


def report(p: VehicleParams) -> str:
    plan = build_plan(p)
    L = p.layout
    lines = [
        "Vehicle assembly plan -- %s" % p.name,
        "  drive layout / corners   : %s, %d suspension corners" % (L.drive_layout, L.suspension_corners),
        "  wheelbase / track (f/r)  : %.0f / %.0f / %.0f mm" % (
            L.wheelbase_mm, L.track_front_mm, L.track_rear_mm),
        "  hub-centre height        : %.0f mm (tyre radius)" % L.tyre_radius_mm,
        "  components (%d):" % len(plan["components"]),
    ]
    for c in plan["components"]:
        ox, oy, oz = c["origin_mm"]
        lines.append("    %-16s %-18s @ (%7.1f, %7.1f, %7.1f)" % (
            c["name"], c["part_file"], ox, oy, oz))
    issues = plan["validation"]
    lines.append("  validation: %s" % ("OK (layout is consistent)" if not issues else "%d issue(s)" % len(issues)))
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
