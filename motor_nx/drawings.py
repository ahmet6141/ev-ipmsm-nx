"""2D manufacturing drawings, generated directly from the model -- NX-independent.

Builds dimensioned engineering drawings (cross-section + detail views, linear &
diameter dimensions, leader callouts, centre-lines, a title block, and a notes
block carrying the key tolerances + BOM summary) and renders each to BOTH:

  * DXF  (R12 ASCII; LINE/CIRCLE/ARC/LWPOLYLINE/TEXT/SOLID on named layers) --
         the manufacturing interchange format, opens in NX / AutoCAD / LibreCAD.
  * SVG  -- for on-screen review.

A small abstract entity list is rendered by two back-ends (DXF, SVG) so the
geometry/dimension logic is written once. Three sheets are produced:
  1. assembly cross-section   (overall diameters, air gap, shaft, BOM + notes)
  2. stator lamination detail (slot/tooth/bore/OD/back-iron)
  3. rotor lamination detail  (V-magnet W x t, V-angle, bridge, shaft bore)

Drawings are 1:1 in model millimetres; the title block states the print scale.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from . import blueprint as _bp
from . import em_design
from . import manufacturing as _mfg
from .params import MotorParams

Point = Tuple[float, float]
_ARROW = 2.2          # arrowhead length (mm)
_TXT = 3.2            # default dimension text height (mm)


# --------------------------------------------------------------------------- #
# abstract drawing: collect entities, render to DXF and SVG
# --------------------------------------------------------------------------- #
class Drawing:
    def __init__(self, name: str):
        self.name = name
        self.ents: List[tuple] = []

    # -- primitives -------------------------------------------------------- #
    def line(self, x1, y1, x2, y2, layer="GEOM"):
        self.ents.append(("line", float(x1), float(y1), float(x2), float(y2), layer))

    def circle(self, cx, cy, r, layer="GEOM"):
        self.ents.append(("circle", float(cx), float(cy), float(r), layer))

    def arc(self, cx, cy, r, a1, a2, layer="GEOM"):
        self.ents.append(("arc", float(cx), float(cy), float(r), float(a1), float(a2), layer))

    def polyline(self, pts: List[Point], layer="GEOM", closed=True):
        self.ents.append(("poly", [(float(x), float(y)) for x, y in pts], layer, closed))

    def solid(self, pts: List[Point], layer="DIM"):
        self.ents.append(("solid", [(float(x), float(y)) for x, y in pts], layer))

    def text(self, x, y, s, h=_TXT, layer="TEXT", anchor="ml", rot=0.0):
        self.ents.append(("text", float(x), float(y), str(s), float(h), layer, anchor, float(rot)))

    # -- composed: arrowhead, dimensions, leaders -------------------------- #
    def _arrowhead(self, tip: Point, towards: Point, layer="DIM"):
        dx, dy = tip[0] - towards[0], tip[1] - towards[1]
        d = math.hypot(dx, dy) or 1.0
        ux, uy = dx / d, dy / d           # points from shaft toward tip
        bx, by = tip[0] - ux * _ARROW, tip[1] - uy * _ARROW
        w = _ARROW * 0.32
        self.solid([tip, (bx - uy * w, by + ux * w), (bx + uy * w, by - ux * w)], layer)

    def dim_h(self, xa, xb, y, label, txt_h=_TXT):
        """Horizontal linear/diameter dimension between xa and xb at height y."""
        self.line(xa, 0, xa, y, "DIM"); self.line(xb, 0, xb, y, "DIM")  # extension lines
        self.line(xa, y, xb, y, "DIM")                                   # dimension line
        self._arrowhead((xa, y), (xb, y)); self._arrowhead((xb, y), (xa, y))
        self.text((xa + xb) / 2.0, y + txt_h * 0.6, label, txt_h, "TEXT", "mm")

    def dim_v(self, ya, yb, x, label, txt_h=_TXT):
        self.line(0, ya, x, ya, "DIM"); self.line(0, yb, x, yb, "DIM")
        self.line(x, ya, x, yb, "DIM")
        self._arrowhead((x, ya), (x, yb)); self._arrowhead((x, yb), (x, ya))
        self.text(x + txt_h * 0.6, (ya + yb) / 2.0, label, txt_h, "TEXT", "ml", 90.0)

    def leader(self, fx, fy, tx, ty, label, txt_h=_TXT):
        """Leader callout: arrow at the feature (fx,fy), text at (tx,ty)."""
        self.line(tx, ty, fx, fy, "DIM")
        self._arrowhead((fx, fy), (tx, ty))
        anchor = "ml" if tx <= fx else "mr"
        ox = txt_h * 0.4 if tx <= fx else -txt_h * 0.4
        self.text(tx + ox, ty, label, txt_h, "TEXT", anchor)

    def centerlines(self, extent, layer="CL"):
        self.line(-extent, 0, extent, 0, layer)
        self.line(0, -extent, 0, extent, layer)

    # -- bounds ------------------------------------------------------------ #
    def bounds(self):
        xs, ys = [], []
        for e in self.ents:
            if e[0] == "line":
                xs += [e[1], e[3]]; ys += [e[2], e[4]]
            elif e[0] == "circle":
                xs += [e[1] - e[3], e[1] + e[3]]; ys += [e[2] - e[3], e[2] + e[3]]
            elif e[0] == "arc":
                xs += [e[1] - e[3], e[1] + e[3]]; ys += [e[2] - e[3], e[2] + e[3]]
            elif e[0] in ("poly", "solid"):
                pts = e[1]
                xs += [p[0] for p in pts]; ys += [p[1] for p in pts]
            elif e[0] == "text":
                xs += [e[1]]; ys += [e[2]]
        if not xs:
            return (-1.0, -1.0, 1.0, 1.0)
        return (min(xs), min(ys), max(xs), max(ys))

    # -- DXF back-end ------------------------------------------------------ #
    def to_dxf(self) -> str:
        lays = {_entity_layer(e) for e in self.ents}
        head = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4",
                "9", "$MEASUREMENT", "70", "1", "0", "ENDSEC"]
        tbl = ["0", "SECTION", "2", "TABLES", "0", "TABLE", "2", "LAYER",
               "70", str(len(lays))]
        for i, ly in enumerate(sorted(lays)):
            tbl += ["0", "LAYER", "2", ly, "70", "0", "62", str(_LAYER_COLOR.get(ly, 7)), "6", "CONTINUOUS"]
        tbl += ["0", "ENDTAB", "0", "ENDSEC"]
        body: List[str] = ["0", "SECTION", "2", "ENTITIES"]
        for e in self.ents:
            body += _dxf_entity(e)
        body += ["0", "ENDSEC", "0", "EOF"]
        return "\n".join(head + tbl + body) + "\n"

    # -- SVG back-end ------------------------------------------------------ #
    def to_svg(self, width: int = 1400, margin: int = 30) -> str:
        x0, y0, x1, y1 = self.bounds()
        w_mm = max(1e-6, x1 - x0); h_mm = max(1e-6, y1 - y0)
        scale = (width - 2 * margin) / w_mm
        height = int(h_mm * scale + 2 * margin)

        def tx(x, y):
            return margin + (x - x0) * scale, height - margin - (y - y0) * scale

        out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
               'viewBox="0 0 %d %d">' % (width, height, width, height),
               '<rect width="%d" height="%d" fill="white"/>' % (width, height)]
        for e in self.ents:
            out.append(_svg_entity(e, tx, scale))
        out.append("</svg>")
        return "\n".join(out)


def _entity_layer(e):
    if e[0] == "poly":
        return e[2]
    if e[0] == "text":
        return e[5]
    return e[-1]


_LAYER_COLOR = {"GEOM": 7, "DIM": 1, "TEXT": 3, "CL": 5, "BORDER": 7, "HATCH": 8}
_SVG_STROKE = {"GEOM": "#111", "DIM": "#c00", "TEXT": "#006", "CL": "#39c",
               "BORDER": "#000", "HATCH": "#bbb"}


# --------------------------------------------------------------------------- #
# DXF / SVG per-entity rendering
# --------------------------------------------------------------------------- #
def _dxf_entity(e) -> List[str]:
    t = e[0]
    if t == "line":
        return ["0", "LINE", "8", e[5], "10", "%.4f" % e[1], "20", "%.4f" % e[2],
                "11", "%.4f" % e[3], "21", "%.4f" % e[4]]
    if t == "circle":
        return ["0", "CIRCLE", "8", e[4], "10", "%.4f" % e[1], "20", "%.4f" % e[2], "40", "%.4f" % e[3]]
    if t == "arc":
        return ["0", "ARC", "8", e[6], "10", "%.4f" % e[1], "20", "%.4f" % e[2],
                "40", "%.4f" % e[3], "50", "%.4f" % e[4], "51", "%.4f" % e[5]]
    if t == "poly":
        pts, layer, closed = e[1], e[2], e[3]
        out = ["0", "LWPOLYLINE", "8", layer, "90", str(len(pts)), "70", "1" if closed else "0"]
        for x, y in pts:
            out += ["10", "%.4f" % x, "20", "%.4f" % y]
        return out
    if t == "solid":
        p = e[1]
        q = p + [p[-1]] * (4 - len(p)) if len(p) < 4 else p[:4]
        return ["0", "SOLID", "8", e[2],
                "10", "%.4f" % q[0][0], "20", "%.4f" % q[0][1],
                "11", "%.4f" % q[1][0], "21", "%.4f" % q[1][1],
                "12", "%.4f" % q[2][0], "22", "%.4f" % q[2][1],
                "13", "%.4f" % q[3][0], "23", "%.4f" % q[3][1]]
    if t == "text":
        _, x, y, s, h, layer, anchor, rot = e
        hj = {"l": 0, "m": 1, "r": 2}[anchor[1] if len(anchor) > 1 else "l"]
        vj = {"t": 3, "m": 2, "b": 1}[anchor[0]]
        return ["0", "TEXT", "8", layer, "10", "%.4f" % x, "20", "%.4f" % y,
                "40", "%.4f" % h, "1", s, "50", "%.4f" % rot,
                "72", str(hj), "73", str(vj), "11", "%.4f" % x, "21", "%.4f" % y]
    return []


def _svg_entity(e, tx, scale) -> str:
    t = e[0]
    col = _SVG_STROKE.get(_entity_layer(e), "#111")
    lw = 1.0 if _entity_layer(e) == "GEOM" else 0.7
    if t == "line":
        x1, y1 = tx(e[1], e[2]); x2, y2 = tx(e[3], e[4])
        dash = ' stroke-dasharray="6 3"' if e[5] == "CL" else ""
        return '<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="%.2f"%s/>' % (
            x1, y1, x2, y2, col, lw, dash)
    if t == "circle":
        cx, cy = tx(e[1], e[2])
        return '<circle cx="%.2f" cy="%.2f" r="%.2f" fill="none" stroke="%s" stroke-width="%.2f"/>' % (
            cx, cy, e[3] * scale, col, lw)
    if t == "arc":
        cx, cy = e[1], e[2]; r = e[3]
        a1, a2 = math.radians(e[4]), math.radians(e[5])
        p1 = tx(cx + r * math.cos(a1), cy + r * math.sin(a1))
        p2 = tx(cx + r * math.cos(a2), cy + r * math.sin(a2))
        large = 1 if (e[5] - e[4]) % 360 > 180 else 0
        return '<path d="M%.2f,%.2f A%.2f,%.2f 0 %d 0 %.2f,%.2f" fill="none" stroke="%s" stroke-width="%.2f"/>' % (
            p1[0], p1[1], r * scale, r * scale, large, p2[0], p2[1], col, lw)
    if t == "poly":
        pts = [tx(x, y) for x, y in e[1]]
        d = " ".join(("M" if i == 0 else "L") + "%.2f,%.2f" % p for i, p in enumerate(pts))
        if e[3]:
            d += " Z"
        return '<path d="%s" fill="none" stroke="%s" stroke-width="%.2f"/>' % (d, col, lw)
    if t == "solid":
        pts = [tx(x, y) for x, y in e[1]]
        d = " ".join("%.2f,%.2f" % p for p in pts)
        return '<polygon points="%s" fill="%s"/>' % (d, col)
    if t == "text":
        _, x, y, s, h, layer, anchor, rot = e
        px, py = tx(x, y)
        ta = {"l": "start", "m": "middle", "r": "end"}[anchor[1] if len(anchor) > 1 else "l"]
        dy = {"t": "0.0em", "m": "0.35em", "b": "0.7em"}[anchor[0]]
        trans = ' transform="rotate(%.1f %.2f %.2f)"' % (-rot, px, py) if rot else ""
        s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        return '<text x="%.2f" y="%.2f" font-family="monospace" font-size="%.1f" fill="%s" text-anchor="%s" dy="%s"%s>%s</text>' % (
            px, py, h * scale, col, ta, dy, trans, s)
    return ""


# --------------------------------------------------------------------------- #
# cross-section geometry (projected build steps, like preview/fea)
# --------------------------------------------------------------------------- #
def _rotate(pts, deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def _draw_cross_section(d: Drawing, bp: Dict[str, Any], roles: Optional[set] = None):
    for st in bp["build_steps"]:
        role = st.get("role", "")
        if role == "end_winding":
            continue
        if roles is not None and role not in roles:
            continue
        kind = st["kind"]
        count = max(1, st.get("pattern_count", 1))
        ang = st.get("pattern_angle_deg", 0.0)
        if kind == "tube":
            d.circle(0, 0, st["outer_radius"]); d.circle(0, 0, st["inner_radius"])
        elif kind == "cylinder":
            for i in range(count):
                cx, cy = _rotate([(st.get("cx", 0.0), st.get("cy", 0.0))], i * ang)[0]
                d.circle(cx, cy, st["outer_radius"])
        elif kind == "extrude" and st.get("profile"):
            base = [(p[0], p[1]) for p in st["profile"]]
            for i in range(count):
                d.polyline(_rotate(base, i * ang))
        elif kind == "revolve" and st.get("profile"):
            rs = [p[0] for p in st["profile"]]
            if rs:
                d.circle(0, 0, max(rs))
                if min(rs) > 1e-6:
                    d.circle(0, 0, min(rs))


def _title_block(d: Drawing, x: float, y: float, w: float, rows: List[Tuple[str, str]]):
    """Simple title block: a boxed key/value table with its top-left at (x, y)."""
    rh = 9.0
    h = rh * len(rows)
    d.line(x, y, x + w, y, "BORDER"); d.line(x, y - h, x + w, y - h, "BORDER")
    d.line(x, y, x, y - h, "BORDER"); d.line(x + w, y, x + w, y - h, "BORDER")
    d.line(x + w * 0.34, y, x + w * 0.34, y - h, "BORDER")
    for i, (k, v) in enumerate(rows):
        yy = y - rh * (i + 0.5)
        if i:
            d.line(x, y - rh * i, x + w, y - rh * i, "BORDER")
        d.text(x + 2, yy, k, 3.0, "TEXT", "ml")
        d.text(x + w * 0.34 + 2, yy, v, 3.4, "TEXT", "ml")


def _notes_block(d: Drawing, x: float, y: float, title: str, lines: List[str], h: float = 3.2):
    d.text(x, y, title, h * 1.3, "TEXT", "tl")
    for i, ln in enumerate(lines):
        d.text(x, y - h * 2.0 - i * h * 1.55, ln, h, "TEXT", "tl")


# --------------------------------------------------------------------------- #
# the three sheets
# --------------------------------------------------------------------------- #
def _common_title_rows(p: MotorParams, sheet: str, scale: str, date: str):
    return [
        ("TITLE", "EV TRACTION IPMSM"),
        ("SHEET", sheet),
        ("PART", p.name),
        ("MATERIAL", "see BOM / notes"),
        ("SCALE", scale),
        ("UNITS", "mm"),
        ("DWG No", "EVM-%s" % sheet.split()[0]),
        ("DATE / REV", "%s / A" % date),
    ]


def assembly_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    g = em_design.derive(p)
    bp = _bp.generate(p)
    bom = _mfg.bill_of_materials(p)
    d = Drawing("assembly")
    _draw_cross_section(d, bp)
    R_house = g.stator_outer_radius + p.cooling.housing_gap + p.cooling.jacket_thickness
    d.centerlines(R_house * 1.15)

    # stacked diameter dimensions above the part
    R_rotor = g.rotor_outer_radius
    y = R_house + 16
    d.dim_h(-R_house, R_house, y + 0,  "%cHOUSING %.1f" % (0xD8, 2 * R_house))
    d.dim_h(-g.stator_outer_radius, g.stator_outer_radius, y + 14, "%cSTATOR OD %.0f h6" % (0xD8, p.stator.outer_diameter))
    d.dim_h(-g.bore_radius, g.bore_radius, y + 28, "%cBORE %.0f H7" % (0xD8, p.stator.bore_diameter))
    d.dim_h(-R_rotor, R_rotor, y + 42, "%cROTOR OD %.1f" % (0xD8, 2 * R_rotor))
    d.dim_h(-g.shaft_radius, g.shaft_radius, y + 56, "%cSHAFT %.0f" % (0xD8, p.shaft.diameter))

    # leader callouts
    d.leader(g.bore_radius, 0, g.bore_radius + 40, -R_house * 0.35,
             "AIR GAP %.2f (radial)" % p.rotor.air_gap)
    d.leader(R_house * 0.71, R_house * 0.5, R_house + 12, R_house * 0.7,
             "WATER JACKET t=%.0f" % p.cooling.jacket_thickness)

    # title block (lower-right) + BOM table (lower-left)
    _title_block(d, R_house + 6, -R_house + 10, 95, _common_title_rows(p, "1 ASSY", "1:2", date))
    bom_lines = ["BILL OF MATERIALS (modelled mass):"]
    for it in bom["line_items"]:
        bom_lines.append("  %-26s %5.2f kg x%d" % (it["component"][:26], it["mass_kg"], it["qty"]))
    bom_lines.append("  %-26s %5.2f kg" % ("TOTAL", bom["total_mass_kg"]))
    _notes_block(d, -R_house, -R_house - 8, "", bom_lines, 3.0)
    return d


def stator_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    g = em_design.derive(p)
    bp = _bp.generate(p)
    d = Drawing("stator")
    _draw_cross_section(d, bp, roles={"stator_steel", "stator_slot_cut"})
    Ro = g.stator_outer_radius
    d.centerlines(Ro * 1.15)
    y = Ro + 14
    d.dim_h(-Ro, Ro, y, "%cOD %.0f h6  (%d SLOTS)" % (0xD8, p.stator.outer_diameter, p.stator.slot_count))
    d.dim_h(-g.bore_radius, g.bore_radius, y + 14, "%cBORE %.0f H7" % (0xD8, p.stator.bore_diameter))
    # radial build-up callouts along +X (slot centre line)
    d.leader(g.bore_radius, 0, -Ro - 10, Ro * 0.30, "SLOT OPENING %.2f x %.1f" %
             (p.stator.slot_opening_width, p.stator.slot_opening_depth))
    d.leader(g.slot_body_outer_radius, 0, Ro + 10, -Ro * 0.20, "SLOT DEPTH %.1f" % g.slot_depth)
    d.leader(g.slot_body_inner_radius + 1, g.slot_width / 2, Ro + 10, Ro * 0.15,
             "SLOT WIDTH %.2f  TOOTH %.1f" % (g.slot_width, p.stator.tooth_width))
    d.leader((Ro + g.slot_body_outer_radius) / 2, 0, -Ro - 10, -Ro * 0.30,
             "BACK IRON %.0f" % p.stator.back_iron_thickness)
    _title_block(d, Ro + 6, -Ro + 6, 95, _common_title_rows(p, "2 STATOR", "1:1.5", date))
    _notes_block(d, -Ro - 10, -Ro - 6, "NOTES:", [
        "1. Laminate: %s." % p.material.electrical_steel,
        "2. Stamped slot/tooth/bore features +/-0.02; slot opening +/-0.02.",
        "3. Bore %cH7; cylindricity 0.015; runout 0.02-0.03 to A." % 0xD8,
        "4. Stack length %.0f +/-0.30; backlack-bond, stacking >= 0.96." % p.stack_length,
    ], 3.0)
    return d


def rotor_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    g = em_design.derive(p)
    bp = _bp.generate(p)
    d = Drawing("rotor")
    _draw_cross_section(d, bp, roles={"rotor_steel", "magnet_pocket_cut", "magnet", "rotor_hole_cut"})
    Ro = g.rotor_outer_radius
    d.centerlines(Ro * 1.2)
    y = Ro + 12
    d.dim_h(-Ro, Ro, y, "%cROTOR OD %.1f  (%d POLES)" % (0xD8, 2 * Ro, p.rotor.pole_count))
    d.dim_h(-g.shaft_radius, g.shaft_radius, y + 13, "%cROTOR BORE (shaft fit) %.0f" % (0xD8, p.shaft.diameter))
    d.leader(Ro - p.rotor.outer_bridge, Ro * 0.05, Ro + 10, Ro * 0.55,
             "OUTER BRIDGE %.1f" % p.rotor.outer_bridge)
    d.leader(Ro * 0.62, Ro * 0.22, -Ro - 10, Ro * 0.55,
             "MAGNET %.0f x %.1f  (N seg=%d)" % (p.rotor.magnet_width, p.rotor.magnet_thickness,
                                                 p.material.magnet_segments_axial))
    d.leader(Ro * 0.66, Ro * 0.12, -Ro - 10, Ro * 0.30, "V-ANGLE %.0f%c" % (p.rotor.v_angle_deg, 0xB0))
    d.leader(g.shaft_radius + p.rotor.vertex_gap, 1.0, Ro + 10, -Ro * 0.2,
             "CENTRE RIB %.1f" % (2 * p.rotor.center_post_halfwidth))
    _title_block(d, Ro + 6, -Ro + 6, 95, _common_title_rows(p, "3 ROTOR", "1:1.5", date))
    _notes_block(d, -Ro - 10, -Ro - 6, "NOTES:", [
        "1. Magnet: sintered NdFeB %s; assemble UNMAGNETIZED, magnetize in place." % p.material.magnet_grade,
        "2. Pocket width +0.05/0; pole position +/-0.1%c (pos 0.10 MMC to B)." % 0xB0,
        "3. Outer bridge %.1f +/-0.05; finish OD on journals (runout 0.02 to A)." % p.rotor.outer_bridge,
        "4. Balance assembled rotor ISO 21940-11 G2.5 (target G1.0) at max speed.",
    ], 3.0)
    return d


def all_sheets(p: MotorParams, date: str = "-------") -> Dict[str, Drawing]:
    return {"1_assembly": assembly_sheet(p, date),
            "2_stator": stator_sheet(p, date),
            "3_rotor": rotor_sheet(p, date)}


def write_drawings(p: MotorParams, out_dir: str = "drawings", date: str = "-------") -> List[str]:
    import os
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for key, d in all_sheets(p, date).items():
        for ext, payload in (("dxf", d.to_dxf()), ("svg", d.to_svg())):
            path = os.path.join(out_dir, "%s.%s" % (key, ext))
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(payload)
            written.append(path)
    return written
