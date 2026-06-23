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

    # -- GD&T: feature-control frame + datum-feature symbol ---------------- #
    def fcf(self, x, y, cells, h=_TXT):
        """Feature-control frame: a row of boxed cells (e.g. ['CYL', '0.015', 'A'])
        with its top-left at (x, y). The first cell is the geometric characteristic."""
        cw = h * 3.4
        hh = h * 1.9
        for i, c in enumerate(cells):
            x0 = x + i * cw
            self.line(x0, y, x0 + cw, y, "DIM")
            self.line(x0, y - hh, x0 + cw, y - hh, "DIM")
            self.line(x0, y, x0, y - hh, "DIM")
            self.text(x0 + cw / 2.0, y - hh / 2.0, str(c), h * 0.85, "TEXT", "mm")
        self.line(x + len(cells) * cw, y, x + len(cells) * cw, y - hh, "DIM")

    def datum(self, x, y, letter, h=_TXT):
        """Datum-feature symbol: a boxed letter (the leader/triangle is implied)."""
        s = h * 1.8
        self.line(x, y, x + s, y, "DIM"); self.line(x, y - s, x + s, y - s, "DIM")
        self.line(x, y, x, y - s, "DIM"); self.line(x + s, y, x + s, y - s, "DIM")
        self.text(x + s / 2.0, y - s / 2.0, str(letter), h, "TEXT", "mm")

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
def _draw_cross_section(d: Drawing, bp: Dict[str, Any], roles: Optional[set] = None):
    for role, shape in _bp.iter_cross_section(bp, roles):
        if shape[0] == "circle":
            d.circle(shape[1], shape[2], shape[3])
        else:  # polygon
            d.polyline(shape[1])


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
    a = p.assembly
    d = Drawing("assembly")
    _draw_cross_section(d, bp)
    R_house = g.stator_outer_radius + p.cooling.housing_gap + p.cooling.jacket_thickness
    flange_R = R_house + (a.housing_flange_od_margin if (a.enabled and a.housing_flange_thickness > 0) else 0.0)
    R_ext = max(R_house, flange_R)
    d.centerlines(R_ext * 1.15)

    # stacked diameter dimensions above the part
    R_rotor = g.rotor_outer_radius
    y = R_ext + 16
    if flange_R > R_house:
        d.dim_h(-flange_R, flange_R, y - 14, "%cFLANGE %.0f" % (0xD8, 2 * flange_R))
    d.dim_h(-R_house, R_house, y + 0,  "%cHOUSING %.1f" % (0xD8, 2 * R_house))
    d.dim_h(-g.stator_outer_radius, g.stator_outer_radius, y + 14, "%cSTATOR OD %.0f h6" % (0xD8, p.stator.outer_diameter))
    d.dim_h(-g.bore_radius, g.bore_radius, y + 28, "%cBORE %.0f H7" % (0xD8, p.stator.bore_diameter))
    d.dim_h(-R_rotor, R_rotor, y + 42, "%cROTOR OD %.1f" % (0xD8, 2 * R_rotor))
    d.dim_h(-g.shaft_radius, g.shaft_radius, y + 56, "%cSHAFT %.0f" % (0xD8, p.shaft.diameter))

    # leader callouts
    d.leader(g.bore_radius, 0, g.bore_radius + 40, -R_ext * 0.35,
             "AIR GAP %.2f (radial)" % p.rotor.air_gap)
    d.leader(R_house * 0.71, R_house * 0.5, R_house + 12, R_ext * 0.7,
             "WATER JACKET t=%.0f" % p.cooling.jacket_thickness)
    # assembly / mounting features on the flange
    if a.enabled and a.housing_flange_thickness > 0:
        jo = R_house
        if a.housing_mount_bolt_count > 0:
            pr = jo + 0.72 * a.housing_flange_od_margin
            d.leader(0, pr, -flange_R - 8, flange_R * 0.55,
                     "%dx MOUNT BOLT %c%.1f @ R%.0f" % (a.housing_mount_bolt_count, 0xD8, a.housing_mount_bolt_diameter, pr))
        if a.housing_endshield_bolt_count > 0:
            pr = jo + 0.30 * a.housing_flange_od_margin
            d.leader(-pr * 0.7, pr * 0.7, -flange_R - 8, flange_R * 0.20,
                     "%dx END-SHIELD BOLT %c%.1f @ R%.0f" % (a.housing_endshield_bolt_count, 0xD8, a.housing_endshield_bolt_diameter, pr))
        if a.housing_coolant_port_diameter > 0:
            d.leader(0, R_house, flange_R + 8, flange_R * 0.85,
                     "COOLANT PORT %c%.0f (in/out)" % (0xD8, a.housing_coolant_port_diameter))
    # end-shield (bearing cap) callout + datum/GD&T on the rotational axis
    if a.enabled and a.endshield_enabled and a.housing_flange_thickness > 0:
        d.leader(g.shaft_radius, -g.shaft_radius, flange_R + 8, -flange_R * 0.45,
                 "END-SHIELD (bearing cap) %cbore %.0f" % (0xD8, a.endshield_bearing_bore))
    d.datum(g.shaft_radius + 3, -3, "A")
    d.fcf(g.bore_radius + 6, -R_ext * 0.5, ["RUNOUT", "0.03", "A"])

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
    a = p.assembly
    bp = _bp.generate(p)
    d = Drawing("stator")
    _draw_cross_section(d, bp, roles={"stator_steel", "stator_slot_cut",
                                      "stator_tie_rod_cut", "stator_key_cut"})
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
    # assembly features: tie-rod ring + OD anti-rotation key
    if a.enabled and a.stator_tie_rod_count > 0:
        pr = a.stator_tie_rod_pitch_radius or 0.5 * (g.slot_body_outer_radius + Ro)
        d.leader(0, pr, Ro + 10, Ro * 0.45,
                 "%dx TIE-ROD %c%.1f @ R%.0f" % (a.stator_tie_rod_count, 0xD8, a.stator_tie_rod_diameter, pr))
    if a.enabled and a.stator_key_count > 0:
        d.leader(Ro - a.stator_key_depth / 2, 0, Ro + 10, Ro * 0.70,
                 "%dx OD KEY %.0fx%.1f" % (a.stator_key_count, a.stator_key_width, a.stator_key_depth))
    d.datum(g.bore_radius + 2, 2, "A")
    d.fcf(Ro * 0.40, -Ro - 16, ["POS", "0.05", "A"])   # slot-pattern position to the bore axis
    _title_block(d, Ro + 6, -Ro + 6, 95, _common_title_rows(p, "2 STATOR", "1:1.5", date))
    _notes_block(d, -Ro - 10, -Ro - 6, "NOTES:", [
        "1. Laminate: %s." % p.material.electrical_steel,
        "2. Stamped slot/tooth/bore features +/-0.02; slot opening +/-0.02.",
        "3. Bore %cH7; cylindricity 0.015; runout 0.02-0.03 to A." % 0xD8,
        "4. Stack length %.0f +/-0.30; backlack-bond, stacking >= 0.96." % p.stack_length,
        "5. Tie-rod holes + OD key clamp/locate the stack (see hardware schedule).",
    ], 3.0)
    return d


def rotor_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    g = em_design.derive(p)
    bp = _bp.generate(p)
    a = p.assembly
    d = Drawing("rotor")
    _draw_cross_section(d, bp, roles={"rotor_steel", "magnet_pocket_cut", "magnet",
                                      "rotor_hole_cut", "rotor_rivet_cut", "rotor_keyway_cut"})
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
    if a.enabled and a.rotor_rivet_count > 0:
        pr = a.rotor_rivet_pitch_radius or (g.shaft_radius + 0.45 * p.rotor.vertex_gap)
        d.leader(0, pr, -Ro - 10, -Ro * 0.55,
                 "%dx RIVET/END-PLATE %c%.1f @ R%.0f" % (a.rotor_rivet_count, 0xD8, a.rotor_rivet_diameter, pr))
    d.datum(g.shaft_radius + 2, 2, "B")
    d.fcf(Ro * 0.40, -Ro - 16, ["POS", "0.10(M)", "B"])   # magnet-pocket position to the bore/d-axis
    _title_block(d, Ro + 6, -Ro + 6, 95, _common_title_rows(p, "3 ROTOR", "1:1.5", date))
    _notes_block(d, -Ro - 10, -Ro - 6, "NOTES:", [
        "1. Magnet: sintered NdFeB %s; assemble UNMAGNETIZED, magnetize in place." % p.material.magnet_grade,
        "2. Pocket width +0.05/0; pole position +/-0.1%c (pos 0.10 MMC to B)." % 0xB0,
        "3. Outer bridge %.1f +/-0.05; finish OD on journals (runout 0.02 to A)." % p.rotor.outer_bridge,
        "4. Balance assembled rotor ISO 21940-11 G2.5 (target G1.0) at max speed.",
        "5. Bore = shaft press/shrink fit (H7); rivet/end-plate holes retain the stack.",
    ], 3.0)
    return d


def shaft_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    """Longitudinal (side) section of the shaft with its manufacturing features:
    stepped journals / bearing seats, the DIN 6885 drive-end keyway, the DIN 471
    retaining-ring groove and the hollow-shaft radial oil cross-holes."""
    g = em_design.derive(p)
    a = p.assembly
    sh = p.shaft
    d = Drawing("shaft")
    # longitudinal outline: plot the (r, z) profile as (z, +/-r) -- a side section
    prof = _bp.shaft_profile(p, g)
    top = [(z, r) for (r, z) in prof]
    d.polyline(top, closed=True)
    d.polyline([(z, -r) for (r, z) in prof], closed=True)
    z_l = -sh.overhang
    z_r = p.stack_length + sh.overhang
    r_main = sh.diameter / 2.0
    r_brg = sh.bearing_seat_diameter / 2.0
    has_stub = sh.drive_stub_length > 0 and 0 < sh.drive_stub_diameter < sh.bearing_seat_diameter
    r_stub = sh.drive_stub_diameter / 2.0
    z_end = z_r + sh.drive_stub_length if has_stub else z_r
    d.line(z_l - 8, 0, z_end + 8, 0, "CL")         # rotation-axis centre-line

    # overall length + key diameters
    yL = r_main + 20
    d.line(z_l, 0, z_l, yL, "DIM"); d.line(z_end, 0, z_end, yL, "DIM")
    d.line(z_l, yL, z_end, yL, "DIM")
    d._arrowhead((z_l, yL), (z_end, yL)); d._arrowhead((z_end, yL), (z_l, yL))
    d.text((z_l + z_end) / 2.0, yL + 2, "OAL %.0f" % (z_end - z_l), _TXT, "TEXT", "bm")
    d.leader(0, r_main, z_l, r_main + 12, "%cJOURNAL %.0f (n6/m6 rotor seat)" % (0xD8, sh.diameter))
    d.leader(z_r - sh.bearing_seat_length / 2, r_brg, z_r - 30, r_brg + 16,
             "%cBEARING SEAT %.0f k5" % (0xD8, sh.bearing_seat_diameter))
    if has_stub:
        d.leader(z_r + sh.drive_stub_length / 2, r_stub, z_end, r_stub + 14,
                 "%cOUTPUT STUB %.0f" % (0xD8, sh.drive_stub_diameter))
    if sh.bore_diameter > 0:
        d.leader(z_l + 4, sh.bore_diameter / 2, z_l, -r_main - 12,
                 "%cHOLLOW BORE %.0f (oil feed)" % (0xD8, sh.bore_diameter))

    # assembly features -- keyway sits on the output stub (if present) else the DE seat
    if a.enabled and a.shaft_keyway_width > 0:
        if has_stub:
            r_surf = r_stub
            kl = min(a.shaft_keyway_length, sh.drive_stub_length)
            zk0 = z_r + (sh.drive_stub_length - kl)
        else:
            r_surf = r_brg
            kl = min(a.shaft_keyway_length, sh.bearing_seat_length)
            zk0 = z_r - kl
        zk1 = zk0 + kl
        d.polyline([(zk0, r_surf - a.shaft_keyway_depth), (zk1, r_surf - a.shaft_keyway_depth),
                    (zk1, r_surf), (zk0, r_surf)], closed=False)
        d.leader(zk0 + kl / 2, r_surf - a.shaft_keyway_depth / 2, zk0, -r_main - 12,
                 "DE KEYWAY %.0fx%.1f L%.0f (DIN 6885)" % (a.shaft_keyway_width, a.shaft_keyway_depth, kl))
    if a.enabled and a.shaft_snap_ring_width > 0:
        zg = z_r - sh.bearing_seat_length - a.shaft_snap_ring_width
        d.polyline([(zg, r_main - a.shaft_snap_ring_depth), (zg + a.shaft_snap_ring_width, r_main - a.shaft_snap_ring_depth),
                    (zg + a.shaft_snap_ring_width, r_main), (zg, r_main)], closed=False)
        d.leader(zg, r_main - a.shaft_snap_ring_depth, zg - 14, r_main + 14,
                 "RETAINING GROOVE (DIN 471)")
    if a.enabled and sh.bore_diameter > 0 and a.shaft_oil_hole_count > 0:
        zc = p.stack_length / 2.0
        d.line(zc, sh.bore_diameter / 2, zc, r_main, "GEOM")
        d.line(zc, -sh.bore_diameter / 2, zc, -r_main, "GEOM")
        d.leader(zc, r_main, zc + 20, r_main + 14,
                 "%dx OIL CROSS-HOLE %c%.0f" % (a.shaft_oil_hole_count, 0xD8, a.shaft_oil_hole_diameter))

    d.datum(z_l + sh.bearing_seat_length / 2, -r_brg - 2, "A")
    d.datum(z_r - sh.bearing_seat_length / 2, -r_brg - 2, "B")
    d.fcf(0, -r_main - 16, ["RUNOUT", "0.01", "A-B"])   # journal runout to the bearing axes
    _title_block(d, z_end + 14, r_main + 8, 95, _common_title_rows(p, "4 SHAFT", "1:2", date))
    _notes_block(d, z_l, -r_main - 26, "NOTES:", [
        "1. Material: alloy steel 42CrMo4 / 4140, hardened journals.",
        "2. Bearing seats k5; journal Ra<=0.4 um ground; coaxiality 0.01 to A-B.",
        "3. Keyway width N9; symmetry 0.02 to axis. Retaining groove per DIN 471.",
        "4. Machine OD after pressing the rotor stack; two-plane balance the assembly.",
    ], 3.0)
    return d


def exploded_sheet(p: MotorParams, date: str = "-------") -> Drawing:
    """Exploded ASSEMBLY layout: each major component as a longitudinal half-section,
    laid out left->right in assembly order with the assembly-path centre-line, so the
    build-up (end-shields / housing+stator / rotor+shaft) reads at a glance."""
    g = em_design.derive(p)
    a = p.assembly
    sh = p.shaft
    bp = _bp.generate(p)
    d = Drawing("exploded")
    housing = next((s for s in bp["build_steps"] if s["role"] == "housing" and s["kind"] == "tube"), None)
    h_len = housing["length"] if housing else p.stack_length + 50.0
    jacket_inner = g.stator_outer_radius + p.cooling.housing_gap
    flange_R = jacket_inner + p.cooling.jacket_thickness + (
        a.housing_flange_od_margin if (a.enabled and a.housing_flange_thickness > 0) else 0.0)
    es_t = a.endshield_thickness if (a.enabled and a.endshield_enabled) else 0.0
    es_bore = a.endshield_bearing_bore / 2.0

    # (label, length, r_in, r_out) in assembly order
    comps = []
    if es_t:
        comps.append(("NDE END-SHIELD", es_t, es_bore, flange_R))
    comps.append(("HOUSING + JACKET", h_len, jacket_inner, flange_R))
    comps.append(("STATOR + WINDING", p.stack_length, g.bore_radius, g.stator_outer_radius))
    comps.append(("ROTOR + MAGNETS", p.stack_length, g.shaft_radius, g.rotor_outer_radius))
    shaft_len = sh.overhang * 2 + p.stack_length + (sh.drive_stub_length if sh.drive_stub_length > 0 else 0.0)
    comps.append(("SHAFT", shaft_len, sh.bore_diameter / 2.0, sh.diameter / 2.0))
    if es_t:
        comps.append(("DE END-SHIELD", es_t, es_bore, flange_R))

    gap = flange_R * 0.55
    x = 0.0
    centres = []
    for label, length, r_in, r_out in comps:
        # full longitudinal section (upper + lower band) of the component envelope
        d.polyline([(x, r_in), (x + length, r_in), (x + length, r_out), (x, r_out)], closed=True)
        if r_in > 1e-6:
            d.polyline([(x, -r_in), (x + length, -r_in), (x + length, -r_out), (x, -r_out)], closed=True)
        else:  # solid (shaft, no bore) -> single band through the axis
            d.polyline([(x, -r_out), (x + length, -r_out), (x + length, r_out), (x, r_out)], closed=True)
        d.text(x + length / 2.0, -flange_R - 8, label, 3.4, "TEXT", "tm")
        d.text(x + length / 2.0, flange_R + 6, "%cmax %.0f" % (0xD8, 2 * r_out), 3.0, "TEXT", "bm")
        centres.append((x + length / 2.0, x, x + length))
        x += length + gap

    total = x - gap
    d.line(-gap, 0, total + gap, 0, "CL")          # assembly-path centre-line
    # assembly-order numbers + flow arrows between components
    for i, (cx, x0, x1) in enumerate(centres):
        d.text(cx, flange_R + 16, "%d" % (i + 1), 4.2, "TEXT", "bm")
        if i + 1 < len(centres):
            nx0 = centres[i + 1][1]
            d._arrowhead((nx0, 0.0), (x1, 0.0))

    _title_block(d, total - 95, -flange_R - 20, 95, _common_title_rows(p, "5 EXPLODED", "1:5", date))
    _notes_block(d, -gap, flange_R + 40, "ASSEMBLY ORDER (see docs/MANUFACTURING.md):", [
        "1-2. Press/bond stacks; shrink stator into the housing jacket.",
        "3.   Press rotor + magnets onto the hollow shaft; magnetize in place; balance.",
        "4.   Insert rotor/shaft into the stator bore (keep the 0.70 mm air gap).",
        "5-6. Bolt the end-shields (bearings) to the flanges; seal; fill coolant; EOL test.",
    ], 3.0)
    return d


def all_sheets(p: MotorParams, date: str = "-------") -> Dict[str, Drawing]:
    return {"1_assembly": assembly_sheet(p, date),
            "2_stator": stator_sheet(p, date),
            "3_rotor": rotor_sheet(p, date),
            "4_shaft": shaft_sheet(p, date),
            "5_exploded": exploded_sheet(p, date)}


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
