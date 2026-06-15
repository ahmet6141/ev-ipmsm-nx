"""Manufacturing hand-off: a model-derived Bill of Materials (mass + count) and a
GD&T / critical-dimension tolerance scheme. NX-independent -- runs under plain
CPython and is unit-tested.

The BOM masses/quantities are computed directly from the geometry in
:mod:`motor_nx.blueprint` (no NX needed): each build step's volume is integrated
(polygon area x length for extrudes, annulus for tubes, Pappus for revolves),
subtract steps are netted against the body they cut, and volumes x material
density give masses. So the BOM always matches whatever the current parameters
produce. Tolerances come from `TOLERANCES` (values grounded in EV-traction
production practice; see docs/MANUFACTURING.md for sources).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from . import blueprint as _bp
from . import em_design
from .params import MotorParams

# material densities (kg/m^3)
DENSITY = {
    "electrical_steel": 7650.0,
    "NdFeB": 7500.0,
    "copper": 8960.0,
    "shaft_steel": 7850.0,
    "aluminium": 2700.0,
}
# materials that are removed (cuts) -- never counted as mass
_VOID = {"air", "coolant"}

# indicative material cost (USD/kg), market-VOLATILE -- NdFeB especially (heavy-rare-
# earth Dy/Tb content). Order-of-magnitude only; replace with a live quote.
COST_PER_KG = {
    "NdFeB": 90.0, "copper": 11.0, "electrical_steel": 2.8,
    "aluminium": 3.5, "shaft_steel": 2.5,
}


# --------------------------------------------------------------------------- #
# volume integration over build steps
# --------------------------------------------------------------------------- #
def _polygon_area(poly: List[List[float]]) -> float:
    """Shoelace area of a closed 2D polygon (mm^2)."""
    a = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) * 0.5


def _revolve_volume(profile: List[List[float]], angle_deg: float) -> float:
    """Volume (mm^3) of an (r, z) profile revolved `angle_deg` about the Z axis
    (Pappus / polygon theorem of Guldinus)."""
    s = 0.0
    n = len(profile)
    for i in range(n):
        r0, z0 = profile[i]
        r1, z1 = profile[(i + 1) % n]
        s += (r0 + r1) * (r0 * z1 - r1 * z0)
    v_full = abs(math.pi / 3.0 * s)
    return v_full * (angle_deg / 360.0)


def _step_unit_volume(step: Dict[str, Any]) -> float:
    """Volume (mm^3) of ONE instance of a build step."""
    kind = step["kind"]
    if kind == "tube":
        return math.pi * (step["outer_radius"] ** 2 - step["inner_radius"] ** 2) * step["length"]
    if kind == "cylinder":
        return math.pi * step["outer_radius"] ** 2 * step["length"]
    if kind == "extrude":
        return _polygon_area(step["profile"]) * step["length"]
    if kind == "revolve":
        return _revolve_volume(step["profile"], step.get("angle_deg", 360.0))
    return 0.0


def step_volumes(p: MotorParams) -> List[Dict[str, Any]]:
    """Per-step volume rows with NET volume for create bodies (subtract steps are
    removed from the body they target)."""
    bp = _bp.generate(p)
    steps = bp["build_steps"]
    raw = {}      # id -> total raw volume (all instances)
    rows = []
    for s in steps:
        vol = _step_unit_volume(s) * max(1, s.get("pattern_count", 1))
        raw[s["id"]] = vol
        rows.append({"id": s["id"], "role": s["role"], "material": s["material"],
                     "boolean": s["boolean"], "target": s.get("target"),
                     "count": max(1, s.get("pattern_count", 1)), "raw_volume": vol})
    # net: subtract cuts from their target create body
    net = dict(raw)
    for s in steps:
        if s["boolean"] == "subtract" and s.get("target") in net:
            net[s["target"]] -= raw[s["id"]]
    for row in rows:
        row["net_volume"] = net.get(row["id"], row["raw_volume"])
    return rows


# --------------------------------------------------------------------------- #
# Bill of Materials
# --------------------------------------------------------------------------- #
# logical BOM groups: role -> (component name, optional stacking-factor flag)
_GROUP = {
    "stator_steel": ("Stator lamination stack", True),
    "rotor_steel": ("Rotor lamination stack", True),
    "magnet": ("Rotor magnets (NdFeB, V)", False),
    "conductor": ("Stator winding - slot bars", False),
    "end_winding": ("Stator winding - end turns", False),
    "shaft": ("Shaft", False),
    "housing": ("Housing / cooling jacket", False),
}


def bill_of_materials(p: MotorParams) -> Dict[str, Any]:
    rows = step_volumes(p)
    g = em_design.derive(p)
    mat = p.material
    groups: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        if row["material"] in _VOID or row["role"] not in _GROUP:
            continue
        name, lam = _GROUP[row["role"]]
        grp = groups.setdefault(name, {
            "component": name, "material": row["material"],
            "count": 0, "volume_mm3": 0.0, "laminated": lam,
        })
        grp["count"] += row["count"]
        grp["volume_mm3"] += max(0.0, row["net_volume"])

    items: List[Dict[str, Any]] = []
    for name, grp in groups.items():
        density = DENSITY.get(grp["material"], 0.0)
        # lamination stacks: effective steel = envelope volume x stacking factor
        factor = mat.stacking_factor if grp["laminated"] else 1.0
        mass = grp["volume_mm3"] * 1e-9 * density * factor
        items.append({
            "component": name,
            "material": _material_spec(grp["material"], p),
            "qty": grp["count"],
            "mass_kg": round(mass, 3),
            "cost_usd": round(mass * COST_PER_KG.get(grp["material"], 0.0), 2),
        })

    # number of laminations (sheets) for the stacks
    sheets = int(round(p.stack_length / max(1e-6, p.stator.lamination_thickness)))
    for it in items:
        if it["component"].startswith(("Stator lamination", "Rotor lamination")):
            it["note"] = "%d sheets @ %.2f mm (stacking %.2f)" % (
                sheets, p.stator.lamination_thickness, mat.stacking_factor)
        elif it["component"].startswith("Stator winding - end"):
            it["note"] = "solid-envelope volume (upper-bound copper; real end-turns are part air)"

    by_material: Dict[str, float] = {}
    for it in items:
        key = it["material"].split(" ")[0]
        by_material[key] = round(by_material.get(key, 0.0) + it["mass_kg"], 3)
    total = round(sum(it["mass_kg"] for it in items), 3)
    total_cost = round(sum(it["cost_usd"] for it in items), 2)
    magnet_cost = next((it["cost_usd"] for it in items if "magnet" in it["component"].lower()), 0.0)

    return {
        "name": p.name,
        "line_items": sorted(items, key=lambda i: -i["mass_kg"]),
        "mass_by_material_kg": by_material,
        "material_cost_usd": total_cost,
        "magnet_cost_share_pct": round(100.0 * magnet_cost / total_cost, 1) if total_cost else 0.0,
        "active_mass_kg": round(sum(i["mass_kg"] for i in items
                                    if i["component"].startswith(("Stator lam", "Rotor lam", "Rotor mag", "Stator wind"))), 3),
        "total_mass_kg": total,
        "magnet_mass_kg": next((i["mass_kg"] for i in items if "magnet" in i["component"].lower()), 0.0),
        "copper_mass_kg": round(sum(i["mass_kg"] for i in items if i["material"].startswith("Copper")), 3),
    }


def _material_spec(material: str, p: MotorParams) -> str:
    m = p.material
    return {
        "electrical_steel": m.electrical_steel,
        "NdFeB": "Sintered NdFeB %s" % m.magnet_grade,
        "copper": "Copper magnet wire (%s)" % m.copper_insulation_class,
        "shaft_steel": "Alloy steel (42CrMo4 / 4140)",
        "aluminium": m.housing_material,
    }.get(material, material)


# --------------------------------------------------------------------------- #
# GD&T / critical-dimension tolerance scheme
# (values grounded in EV-traction production practice; see docs/MANUFACTURING.md)
# --------------------------------------------------------------------------- #
def TOLERANCES(p: MotorParams) -> List[Dict[str, str]]:
    """Critical-to-function GD&T / dimensional scheme. Values web-grounded in EV
    traction production practice and reconciled against an adversarial review (see
    docs/MANUFACTURING.md for sources). Datum A = bearing-journal axis (rotor),
    stator features referenced to the housing register."""
    g = em_design.derive(p)
    pocket_w = p.rotor.magnet_thickness + 2 * p.rotor.pocket_clearance
    st = eccentricity_stackup(p)
    return [
        {"feature": "Air gap (radial) uniformity", "nominal": "%.2f mm" % p.rotor.air_gap, "datum": "A",
         "tolerance": "not directly dimensioned; assembled eccentricity budget <= %.3f mm (~10%% of gap); "
                      "computed RSS of bore+rotor-OD+coaxiality = %.3f mm (%s)"
                      % (st["budget_mm"], st["rss_mm"], "OK" if st["rss_pass"] else "OVER"),
         "gdt": "governed by the stack-up below",
         "rationale": "Bg ~ 1/g; asymmetry -> UMP, 2x-line/pole-passing NVH, cogging"},
        {"feature": "Stator bore diameter", "nominal": "%.1f mm" % p.stator.bore_diameter, "datum": "A (housing register)",
         "tolerance": "IT7 / H7 (+0.040 / 0 mm)", "gdt": "cylindricity 0.015; runout 0.02-0.03 TIR to A",
         "rationale": "one half of the air gap; roundness/runout set static eccentricity"},
        {"feature": "Stator OD (into Al jacket)", "nominal": "%.1f mm" % p.stator.outer_diameter, "datum": "A",
         "tolerance": "h6/js6 shrink fit; diametral interference 0.05-0.10 mm (low end - Al)",
         "gdt": "OD-to-bore concentricity 0.05; ASSEMBLED bore roundness re-checked post-shrink",
         "rationale": "transmit torque + survive Al CTE; do not ovalize the bore (verify after shrink)"},
        {"feature": "Rotor OD diameter", "nominal": "%.1f mm" % (2 * g.rotor_outer_radius), "datum": "A",
         "tolerance": "-0.02 / -0.04 mm (keep gap open); roundness 0.01-0.02",
         "gdt": "runout (TIR) to A <= 0.02 mm; finish-turn/grind on the journals in one clamping",
         "rationale": "other half of the gap; runout = dynamic eccentricity at 18k rpm"},
        {"feature": "Magnet pocket width", "nominal": "%.2f mm" % pocket_w, "datum": "B (rotor bore)",
         "tolerance": "%.2f +0.05/0 mm; magnet ground +/-0.05 -> real clearance 0.10-0.25 mm" % pocket_w,
         "gdt": "wall profile 0.05; stamped to +/-0.02 (die)",
         "rationale": "fit magnet + bond line; position scatter -> cogging, movement demag"},
        {"feature": "Magnet pocket position", "nominal": "%.1f deg pole pitch" % (360.0 / p.rotor.pole_count), "datum": "B",
         "tolerance": "+/-0.1 deg pole-to-pole; radial +/-0.05 mm",
         "gdt": "position 0.10 MMC to B; V-symmetry about d-axis 0.05",
         "rationale": "pole scatter is the prime source of cogging/torque ripple"},
        {"feature": "Outer bridge thickness", "nominal": "%.1f mm" % p.rotor.outer_bridge, "datum": "B",
         "tolerance": "+/-0.05 mm (die +/-0.02; protect lower limit when finishing OD)",
         "gdt": "web profile 0.05; bridge-to-bridge within 0.04",
         "rationale": "centrifugal stress @18k rpm vs d-axis flux leakage (mech FEA)"},
        {"feature": "Slot opening width", "nominal": "%.2f mm" % p.stator.slot_opening_width, "datum": "A",
         "tolerance": "+/-0.02 mm (progressive die); tooth %.1f +/-0.03" % p.stator.tooth_width,
         "gdt": "slot pattern position 0.05 to A; mouth profile 0.04",
         "rationale": "slot-opening permeance sets cogging harmonics + effective fill"},
        {"feature": "Slot body / hairpin fit", "nominal": "%.2f mm" % g.slot_width, "datum": "A",
         "tolerance": "+0.05/0 vs bar column; per-bar clearance 0.40-0.50; depth +/-0.05",
         "gdt": "wall profile 0.05; wall parallelism 0.03",
         "rationale": "guarantee liner+enamel clearance for 8-bar insertion vs slot fill"},
        {"feature": "Bearing journals (DE/NDE)", "nominal": "%.1f mm" % p.shaft.bearing_seat_diameter, "datum": "A-B",
         "tolerance": "k5 (+0.013/+0.002); housing per seat: floating H6, locating J6/K6; Ra <= 0.4 um",
         "gdt": "journal cylindricity 0.004-0.006; two-journal coaxiality 0.01; runout to A-B 0.01",
         "rationale": "light interference stops inner-ring creep; journal axis IS the rotational datum"},
        {"feature": "Rotor seat on shaft (press)", "nominal": "%.1f mm" % p.shaft.diameter, "datum": "A-B",
         "tolerance": "shaft n6/m6 vs stack bore H7; interference 0.02-0.06 mm",
         "gdt": "rotor-seat coaxiality to A-B 0.01; clamp shoulder perpendicularity 0.01",
         "rationale": "transmit traction torque without slip; machine OD after pressing"},
        {"feature": "Lamination stack length", "nominal": "%.0f mm" % p.stack_length, "datum": "A",
         "tolerance": "+/-0.30 mm (sqrt(N)*per-sheet scatter); per-sheet +/-0.01",
         "gdt": "end-face perpendicularity 0.05 to A; parallelism 0.05 over OD",
         "rationale": "active length; end-face squareness keeps the stack from cocking"},
        {"feature": "Rotor balance", "nominal": "two-plane dynamic", "datum": "A-B",
         "tolerance": "ISO 21940-11 G2.5 max @ 18k rpm; DESIGN TARGET G1.0 (NVH)",
         "gdt": "correct at dedicated balance lands; resolution ~1 g-mm/plane",
         "rationale": "1x vibration / bearing life at ~150 m/s tip speed"},
    ]


def general_notes() -> List[str]:
    """Datum strategy, stack-up budget and quality notes (web-grounded)."""
    return [
        "Datum strategy: the bearing-journal axis (A / A-B) is the primary rotational datum; "
        "reference rotor OD and end-face squareness to it. Datum B = the rotor lamination bore / "
        "d-axis (concentric to A via the shaft press-fit); the magnet-pocket pattern is located to B. "
        "Reference stator bore, slot pattern and OD shrink-fit to the housing register.",
        "Eccentricity stack-up: keep combined static+dynamic eccentricity <= 0.07 mm (~10% of the "
        "0.70 mm gap). As RSS of bore TIR (<=0.02), rotor-OD TIR (<=0.02) and journal/housing "
        "coaxiality (0.01 each) this is ~0.03-0.05 mm, comfortably within budget; beyond ~0.07 mm "
        "UMP grows and loads the bearings.",
        "Lamination quality: progressive-die stamping holds slot/tooth/bridge/bore to +/-0.02 mm; "
        "per-sheet 0.27 mm +/-0.01; burr < ~0.02 mm (verify IEC 60404) or eddy loss rises 15-20% "
        "and the stacking factor (target 0.95-0.97) drops. Backlack-bond the stator stack.",
        "Magnet handling: bond UNMAGNETIZED N42SH segments (4 axial, ~0.1 mm gap) into the V pockets "
        "with high-temp epoxy, then magnetize-in-place on the assembled rotor; verify by back-EMF / "
        "surface-flux map. Keep the operating point above the knee at max temperature.",
        "Surface finishes: bearing journals Ra <= 0.4 um (ground); stator bore + rotor OD Ra <= "
        "0.8-1.6 um; shrink/press faces Ra <= 1.6 um for predictable interference.",
        "GD&T framework: ISO 1101 (geometry) + ISO 286 (limits & fits) + ISO 2768-mK (general); "
        "MMC on slot & magnet-pocket position for bonus tolerance.",
        "End-of-line: 100% air-gap/eccentricity (back-EMF symmetry), cogging + no-load loss screen, "
        "and surge/hi-pot on the hairpin insulation.",
    ]


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def bom_report(p: MotorParams) -> str:
    bom = bill_of_materials(p)
    lines = ["Bill of Materials -- %s" % bom["name"],
             "  %-30s %-30s %5s %9s %9s" % ("component", "material", "qty", "mass[kg]", "cost[$]")]
    for it in bom["line_items"]:
        lines.append("  %-30s %-30s %5d %9.3f %9.0f"
                     % (it["component"], it["material"][:30], it["qty"], it["mass_kg"], it["cost_usd"]))
    lines += [
        "  " + "-" * 86,
        "  active material mass : %.2f kg  (steel + copper + magnet)" % bom["active_mass_kg"],
        "  magnet (NdFeB) mass  : %.3f kg" % bom["magnet_mass_kg"],
        "  copper mass          : %.3f kg" % bom["copper_mass_kg"],
        "  TOTAL modelled mass  : %.2f kg" % bom["total_mass_kg"],
        "  material cost (indicative) : $%.0f  (magnet share %.0f%%)"
        % (bom["material_cost_usd"], bom["magnet_cost_share_pct"]),
        "  (geometry-derived mass; cost is market-volatile $/kg, excludes "
        "processing/labour/consumables, fasteners, sensors, connectors)",
    ]
    return "\n".join(lines)


def eccentricity_stackup(p: MotorParams) -> Dict[str, Any]:
    """COMPUTE the air-gap eccentricity stack-up (was hand-asserted prose). The
    budget is ~10% of the mechanical air gap; the achieved estimate is the RSS of
    the independent runout/coaxiality contributors that feed the assembled gap."""
    # per-feature contributors (mm) -- mirror the TOLERANCES GD&T callouts
    contributors = {
        "stator_bore_TIR": 0.02,
        "rotor_OD_TIR": 0.02,
        "journal_coaxiality": 0.01,
        "rotor_seat_coaxiality": 0.01,
    }
    budget = 0.10 * p.rotor.air_gap
    rss = math.sqrt(sum(v * v for v in contributors.values()))
    worst_case = sum(contributors.values())
    return {
        "air_gap_mm": p.rotor.air_gap,
        "budget_mm": round(budget, 4),
        "contributors_mm": contributors,
        "rss_mm": round(rss, 4),
        "worst_case_sum_mm": round(worst_case, 4),
        "rss_pass": rss <= budget + 1e-9,
        "worst_case_pass": worst_case <= budget + 1e-9,
    }


def tolerance_report(p: MotorParams) -> str:
    lines = ["Critical-dimension / GD&T scheme -- %s" % p.name,
             "  %-32s %-14s %-22s %s" % ("feature", "nominal", "tolerance", "GD&T")]
    for t in TOLERANCES(p):
        lines.append("  %-32s %-14s %-22s %s"
                     % (t["feature"][:32], t["nominal"][:14], t["tolerance"][:22], t["gdt"]))
    return "\n".join(lines)
