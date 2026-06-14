"""Dependency-free SVG cross-section of a blueprint -- visually verify the
lamination/magnet/winding geometry WITHOUT opening NX.

Projects every build step onto the XY lamination plane (patterns expanded) and
writes an SVG. Steel/air/magnet/copper are colour-coded. Used by
`python -m motor_nx.cli preview`.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from . import blueprint as _bp

_FILL = {
    "stator_steel": "#9aa0a6", "rotor_steel": "#7c828a", "magnet": "#2b2b30",
    "conductor": "#b87333", "shaft": "#c8ccd2", "housing": "#3f6fb0",
}
_AIR = "#ffffff"


def _circle_pts(cx: float, cy: float, r: float, n: int = 96) -> List[List[float]]:
    return [[cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)]
            for i in range(n)]


def _rotate(points, ang_deg):
    a = math.radians(ang_deg)
    c, s = math.cos(a), math.sin(a)
    return [[x * c - y * s, x * s + y * c] for x, y in points]


def _instances(step: Dict[str, Any]) -> List[List[List[float]]]:
    """Return the XY polygon(s) for a step, patterns expanded."""
    count = max(1, step.get("pattern_count", 1))
    angle = step.get("pattern_angle_deg", 0.0)
    polys: List[List[List[float]]] = []

    if step["kind"] == "extrude" and step.get("profile"):
        base = [[p[0], p[1]] for p in step["profile"]]
        for i in range(count):
            polys.append(_rotate(base, i * angle))
    elif step["kind"] == "cylinder":
        base = _circle_pts(step.get("cx", 0.0), step.get("cy", 0.0), step["outer_radius"])
        for i in range(count):
            polys.append(_rotate(base, i * angle))
    elif step["kind"] == "tube":
        polys.append(_circle_pts(0.0, 0.0, step["outer_radius"]))
        polys.append(_circle_pts(0.0, 0.0, step["inner_radius"]))
    elif step["kind"] == "revolve":
        rs = [p[0] for p in step.get("profile", [])]
        if rs:
            polys.append(_circle_pts(0.0, 0.0, max(rs)))
            inner = min(rs)
            if inner > 1e-6:
                polys.append(_circle_pts(0.0, 0.0, inner))
    return polys


def to_svg(blueprint: Dict[str, Any], size: int = 900) -> str:
    steps = blueprint["build_steps"]
    extent = 1.0
    for s in steps:
        if s["role"] == "housing":
            extent = s["outer_radius"] * 1.05
    extent = extent or 120.0
    scale = (size / 2.0) / extent

    def tx(x, y):  # model mm -> SVG px (y flipped, centred)
        return size / 2.0 + x * scale, size / 2.0 - y * scale

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" '
           f'viewBox="0 0 {size} {size}">',
           f'<rect width="{size}" height="{size}" fill="#f4f4f4"/>']

    # draw order: housing, steel, then cuts (air), then magnets/conductors/shaft
    order = ["housing", "cooling_channel_cut", "stator_steel", "stator_slot_cut",
             "rotor_steel", "rotor_hole_cut", "magnet_pocket_cut", "shaft", "magnet", "conductor"]
    role_steps = {}
    for s in steps:
        role_steps.setdefault(s["role"], []).append(s)

    def emit(step):
        role = step["role"]
        is_cut = role.endswith("_cut")
        fill = _AIR if is_cut else _FILL.get(role, "#cccccc")
        stroke = "#333" if not is_cut else "#999"
        for poly in _instances(step):
            d = " ".join(("M" if i == 0 else "L") + "%.2f,%.2f" % tx(x, y)
                         for i, (x, y) in enumerate(poly)) + " Z"
            out.append(f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="0.6"/>')

    for role in order:
        for step in role_steps.get(role, []):
            emit(step)

    label = "%s | %d slots / %d poles | OD %.0f mm" % (
        blueprint.get("name", "motor"),
        blueprint["parameters"]["stator"]["slot_count"],
        blueprint["parameters"]["rotor"]["pole_count"],
        blueprint["parameters"]["stator"]["outer_diameter"])
    out.append(f'<text x="12" y="{size-14}" font-family="monospace" font-size="14" fill="#222">{label}</text>')
    out.append("</svg>")
    return "\n".join(out)
