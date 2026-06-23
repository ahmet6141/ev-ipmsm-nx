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
    if kind == "hole":  # radial / arbitrary-axis cylindrical hole: pi r^2 * depth
        return math.pi * step["outer_radius"] ** 2 * step["length"]
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
    "endshield": ("End-shield / bearing cap", False),
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
        # "unite" rows (e.g. the housing flanges) merge into one casting -- count
        # their mass but not as a separate part.
        if row["boolean"] == "create":
            grp["count"] += row["count"]
        grp["volume_mm3"] += max(0.0, row["net_volume"])

    ew_fraction = _end_winding_copper_fraction(p, g)
    items: List[Dict[str, Any]] = []
    for name, grp in groups.items():
        density = DENSITY.get(grp["material"], 0.0)
        # lamination stacks: effective steel = envelope volume x stacking factor.
        # end-turn envelope is a full slot-band annulus (incl. teeth) -> scale it to
        # the real slot-copper fraction so the BOM isn't a gross over-estimate.
        factor = mat.stacking_factor if grp["laminated"] else 1.0
        if name.startswith("Stator winding - end"):
            factor = ew_fraction
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
            it["note"] = "envelope scaled by slot-copper fraction %.2f (end-turn estimate)" % ew_fraction

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


def _end_winding_copper_fraction(p: MotorParams, g) -> float:
    """Real slot-copper cross-section / end-winding-envelope annulus cross-section.
    The envelope tube spans the whole slot-band annulus (teeth included); only this
    fraction is actually copper, so scaling the envelope mass by it gives a sane
    end-turn copper mass instead of a gross over-estimate."""
    bars = _bp.conductor_polygons(p, g)
    cu_area = sum(_polygon_area(b) for b in bars) * p.stator.slot_count
    annulus = math.pi * (g.slot_body_outer_radius ** 2 - g.slot_body_inner_radius ** 2)
    return min(1.0, cu_area / annulus) if annulus > 0 else 1.0


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
    ] + _assembly_tolerances(p, g)


def _assembly_tolerances(p: MotorParams, g) -> List[Dict[str, str]]:
    """GD&T / fit callouts for the manufacturing-assembly features (mounting flange
    register, bolt circles, shaft keyway + retaining groove, anti-rotation key).
    Skipped when a feature is disabled so the scheme always matches the geometry."""
    a = getattr(p, "assembly", None)
    if a is None or not a.enabled:
        return []
    rows: List[Dict[str, str]] = []
    if a.housing_flange_thickness > 0:
        jacket_outer = g.stator_outer_radius + p.cooling.housing_gap + p.cooling.jacket_thickness
        rows.append({
            "feature": "Housing mounting-flange register (spigot)",
            "nominal": "%.0f mm pilot" % (2 * jacket_outer), "datum": "C (housing register) / A",
            "tolerance": "h7/H7 pilot fit to the gearbox; face square to A",
            "gdt": "register-to-bore concentricity 0.05; flange-face perpendicularity 0.05 to A",
            "rationale": "locates the whole motor to the transmission; sets shaft-to-input coaxiality"})
        if a.housing_mount_bolt_count > 0:
            rows.append({
                "feature": "Flange / end-shield bolt-circle position",
                "nominal": "%d + %d holes" % (a.housing_mount_bolt_count, a.housing_endshield_bolt_count),
                "datum": "C", "tolerance": "true position 0.3 MMC; pitch-circle dia +/-0.2",
                "gdt": "position 0.30 (M) to C|A", "rationale": "bolt pattern must mate the cover / gearbox"})
    if a.shaft_keyway_width > 0:
        rows.append({
            "feature": "Drive-end shaft keyway (DIN 6885-A)",
            "nominal": "%.0f mm wide" % a.shaft_keyway_width, "datum": "A-B",
            "tolerance": "width N9 (-0/-0.036); depth t1 +0.2/0",
            "gdt": "symmetry 0.02 to the shaft axis; parallelism 0.02",
            "rationale": "even key bearing; off-centre key -> fretting + unbalance at 18k rpm"})
    if a.shaft_snap_ring_width > 0:
        rows.append({
            "feature": "Shaft retaining-ring groove (DIN 471)",
            "nominal": "depth %.1f mm" % a.shaft_snap_ring_depth, "datum": "A-B",
            "tolerance": "groove dia per DIN 471 for %.0f mm shaft; width +0.14/0" % p.shaft.diameter,
            "gdt": "groove-bottom runout 0.02 to A-B", "rationale": "axial bearing retention; sharp corners raise stress"})
    if a.stator_key_count > 0:
        rows.append({
            "feature": "Stator OD anti-rotation key-notch",
            "nominal": "%.0f mm wide" % a.stator_key_width, "datum": "A",
            "tolerance": "width +0.05/0; angular position +/-0.2 deg",
            "gdt": "notch profile 0.05; position 0.1 to A", "rationale": "reacts fault/short-circuit torque on the shrink fit"})
    return rows


# --------------------------------------------------------------------------- #
# fastener / hardware schedule  (the procured items the assembly holes accept)
# --------------------------------------------------------------------------- #
# metric clearance-hole diameter (mm) -> nearest standard bolt thread (ISO 273 medium)
_CLEARANCE_TO_THREAD = [(3.4, "M3"), (4.5, "M4"), (5.5, "M5"), (6.6, "M6"),
                        (9.0, "M8"), (11.0, "M10"), (13.5, "M12"), (17.5, "M16")]


def _thread_for_clearance(d: float) -> str:
    """The bolt thread whose ISO 273 medium clearance hole is ~`d` mm."""
    best = _CLEARANCE_TO_THREAD[0][1]
    for hole, thread in _CLEARANCE_TO_THREAD:
        if d >= hole - 0.4:
            best = thread
    return best


def hardware_schedule(p: MotorParams) -> List[Dict[str, Any]]:
    """The procured FASTENERS / hardware the assembly holes + features accept --
    the production line items a real motor needs beyond the modelled solids. Counts
    track the parametric features in :class:`motor_nx.params.AssemblyParams`.
    Sizes follow the referenced standards (DIN 6885 keys, DIN 471 rings, ISO metric
    bolts). Bearings / end-shields / seals are procured to the interfaces modelled
    on the housing + shaft (bolt circles, journals, registers)."""
    a = getattr(p, "assembly", None)
    rows: List[Dict[str, Any]] = []

    def add(item, std, size, qty, note=""):
        rows.append({"item": item, "standard": std, "size": size, "qty": int(qty), "note": note})

    if a is not None and a.enabled:
        if a.stator_tie_rod_count > 0:
            add("Stator clamping / tie rod", "ISO 4762 (or weld stud)",
                _thread_for_clearance(a.stator_tie_rod_diameter), a.stator_tie_rod_count,
                "axial through the yoke; clamp the bonded stack / locate in housing")
        if a.stator_key_count > 0:
            add("Stator anti-rotation key", "parallel key",
                "%.0fx%.0f" % (a.stator_key_width, a.stator_key_depth), a.stator_key_count,
                "engages a matching housing key-slot; reacts fault torque on the shrink fit")
        if a.rotor_rivet_count > 0:
            add("Rotor end-plate rivet / pin", "solid rivet / dowel",
                "%.0f mm" % a.rotor_rivet_diameter, a.rotor_rivet_count,
                "retains the rotor end-plates / aligns the stack during bonding")
        if a.shaft_keyway_width > 0:
            add("Drive-end shaft key", "DIN 6885-A",
                "%.0fx%.0f" % (a.shaft_keyway_width, a.shaft_keyway_width * 0.66), 1,
                "torque transfer to the output coupling / gear")
        if a.shaft_snap_ring_width > 0:
            add("Bearing retaining ring", "DIN 471",
                "%.0f mm shaft" % p.shaft.diameter, 1, "axially locates the DE bearing inner ring")
        if a.shaft_oil_hole_count > 0 and p.shaft.bore_diameter > 0:
            add("Hollow-shaft oil jet (cross-hole)", "machined",
                "%.0f mm" % a.shaft_oil_hole_diameter, a.shaft_oil_hole_count,
                "rotor cooling oil from the shaft bore to the laminations")
        if a.housing_flange_thickness > 0:
            if a.housing_mount_bolt_count > 0:
                add("Housing-to-gearbox mounting bolt", "ISO 4762",
                    _thread_for_clearance(a.housing_mount_bolt_diameter), a.housing_mount_bolt_count,
                    "DE flange to transmission housing")
            if a.housing_endshield_bolt_count > 0:
                add("End-shield / bearing-cap bolt", "ISO 4762",
                    _thread_for_clearance(a.housing_endshield_bolt_diameter),
                    2 * a.housing_endshield_bolt_count, "both ends (DE + NDE)")
        if a.housing_coolant_port_diameter > 0:
            add("Coolant port fitting (in/out)", "BSP/NPT or O-ring boss",
                "%.0f mm bore" % a.housing_coolant_port_diameter, 2, "jacket inlet + outlet")
        if a.housing_terminal_diameter > 0:
            add("Power-terminal / cable gland", "cable gland / sealed boss",
                "%.0f mm" % a.housing_terminal_diameter, 1, "3-phase lead exit; IP-sealed")
        if a.housing_lifting_hole_diameter > 0:
            add("Lifting eyebolt", "DIN 580",
                _thread_for_clearance(a.housing_lifting_hole_diameter), 1, "handling / hoisting")

    # procured assembly hardware the modelled interfaces mate to (not hole-driven)
    add("Bearing (DE / NDE)", "ISO 15 deep-groove or angular",
        "bore %.0f mm" % p.shaft.bearing_seat_diameter, 2,
        "at least one insulated / hybrid-ceramic (PWM EDM-current)")
    add("End-shield / bearing housing", "cast Al, machined", "to flange bolt circle", 2,
        "carries the bearing bore concentric to the stator bore")
    add("Radial shaft seal", "FKM/HNBR lip seal", "shaft %.0f mm" % p.shaft.bearing_seat_diameter, 2,
        "DE + NDE; retains lubricant / excludes contamination")
    add("Temperature sensor", "PT100 / NTC", "-", 3, "stator end-winding hot-spots")
    add("Position sensor", "resolver / encoder", "-", 1, "rotor angle for FOC")
    return rows


def hardware_report(p: MotorParams) -> str:
    lines = ["Fastener / hardware schedule -- %s" % p.name,
             "  %-32s %-26s %-12s %4s" % ("item", "standard", "size", "qty")]
    for h in hardware_schedule(p):
        lines.append("  %-32s %-26s %-12s %4d" % (h["item"][:32], h["standard"][:26], h["size"][:12], h["qty"]))
    lines.append("  (procured items; mate to the assembly holes / journals / bolt circles modelled on each part)")
    return "\n".join(lines)


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
        "Assembly features: the lamination tie-rod / rivet holes and OD anti-rotation key locate and clamp "
        "the bonded stacks; the housing mounting flange (h7 pilot register + bolt circle), end-shield bolt "
        "circles, coolant ports, terminal gland and lifting eye carry the standard interfaces; the shaft "
        "DIN 6885 keyway + DIN 471 retaining groove transfer torque and locate the bearing. See `cli hardware` "
        "for the fastener schedule and project_details/ for the per-part rationale.",
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
        "processing/labour/consumables)",
        "  fasteners / bearings / seals / sensors: see `cli hardware` (hardware_schedule)",
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


def eccentricity_monte_carlo(p: MotorParams, n: int = 20000, seed: int = 12345) -> Dict[str, Any]:
    """DFM Monte Carlo of the assembled air-gap eccentricity. Each runout/coaxiality
    contributor is a radial VECTOR with a random phase and a half-normal magnitude
    (3-sigma = its tolerance); the assembled eccentricity is their vector sum. Reports
    the distribution + the fraction over the ~10%-of-gap budget + a one-sided process
    capability Cpk = (budget - mean) / (3*sigma). Deterministic via `seed`.

    This is the statistical counterpart of :func:`eccentricity_stackup` (which only
    gives the RSS and worst-case sum): random phases rarely align, so the realistic
    spread sits between RSS and worst-case -- the number a yield estimate needs."""
    import random
    st = eccentricity_stackup(p)
    contributors = st["contributors_mm"]
    budget = st["budget_mm"]
    rng = random.Random(seed)
    two_pi = 2.0 * math.pi
    samples: List[float] = []
    for _ in range(max(1, n)):
        sx = sy = 0.0
        for tol in contributors.values():
            mag = abs(rng.gauss(0.0, tol / 3.0))     # 3-sigma == the tolerance
            ang = rng.uniform(0.0, two_pi)
            sx += mag * math.cos(ang)
            sy += mag * math.sin(ang)
        samples.append(math.hypot(sx, sy))
    samples.sort()

    def pct(q):
        return samples[min(len(samples) - 1, int(q * len(samples)))]

    mean = sum(samples) / len(samples)
    var = sum((s - mean) ** 2 for s in samples) / len(samples)
    sigma = math.sqrt(var)
    over = sum(1 for s in samples if s > budget) / len(samples)
    cpk = (budget - mean) / (3.0 * sigma) if sigma > 0 else float("inf")
    return {
        "trials": len(samples), "budget_mm": budget,
        "mean_mm": round(mean, 4), "sigma_mm": round(sigma, 4),
        "p50_mm": round(pct(0.50), 4), "p95_mm": round(pct(0.95), 4),
        "p99_mm": round(pct(0.99), 4), "max_mm": round(samples[-1], 4),
        "fraction_over_budget": round(over, 5),
        "ppm_over_budget": int(round(over * 1e6)),
        "cpk": round(cpk, 3),
        "rss_mm": st["rss_mm"], "worst_case_mm": st["worst_case_sum_mm"],
    }


def dfm_report(p: MotorParams, n: int = 20000) -> str:
    mc = eccentricity_monte_carlo(p, n)
    verdict = "OK" if mc["cpk"] >= 1.33 else ("MARGINAL" if mc["cpk"] >= 1.0 else "LOW")
    return "\n".join([
        "DFM Monte Carlo -- assembled air-gap eccentricity -- %s" % p.name,
        "  trials               : %d (half-normal runouts, random phase, vector sum)" % mc["trials"],
        "  budget (<=10%% gap)    : %.3f mm" % mc["budget_mm"],
        "  mean / sigma         : %.4f / %.4f mm" % (mc["mean_mm"], mc["sigma_mm"]),
        "  p50 / p95 / p99 / max: %.4f / %.4f / %.4f / %.4f mm"
        % (mc["p50_mm"], mc["p95_mm"], mc["p99_mm"], mc["max_mm"]),
        "  over budget          : %.3f%%  (%d ppm)" % (100 * mc["fraction_over_budget"], mc["ppm_over_budget"]),
        "  Cpk (one-sided)      : %.2f  [%s]  (target >= 1.33)" % (mc["cpk"], verdict),
        "  (compare deterministic RSS %.3f / worst-case %.3f mm)" % (mc["rss_mm"], mc["worst_case_mm"]),
    ])


def manufacturing_summary_md(p: MotorParams) -> str:
    """A single human-readable manufacturing hand-off summary (Markdown): BOM,
    fastener schedule, key tolerances, DFM verdict and the assembly order."""
    bom = bill_of_materials(p)
    mc = eccentricity_monte_carlo(p, 5000)
    L = ["# Manufacturing package -- %s" % p.name, "",
         "Model-derived; regenerate with `python -m motor_nx.cli package`.", "",
         "## Bill of materials (modelled mass)", "",
         "| Component | Material | Qty | Mass [kg] |", "|---|---|---:|---:|"]
    for it in bom["line_items"]:
        L.append("| %s | %s | %d | %.3f |" % (it["component"], it["material"], it["qty"], it["mass_kg"]))
    L += ["| **TOTAL** | | | **%.2f** |" % bom["total_mass_kg"], "",
          "Active mass %.2f kg; magnet %.3f kg; copper %.3f kg; material cost ~$%.0f (magnet %.0f%%)."
          % (bom["active_mass_kg"], bom["magnet_mass_kg"], bom["copper_mass_kg"],
             bom["material_cost_usd"], bom["magnet_cost_share_pct"]), "",
          "## Fastener / hardware schedule", "",
          "| Item | Standard | Size | Qty |", "|---|---|---|---:|"]
    for h in hardware_schedule(p):
        L.append("| %s | %s | %s | %d |" % (h["item"], h["standard"], h["size"], h["qty"]))
    L += ["", "## DFM -- assembled air-gap eccentricity (Monte Carlo)", "",
          "Budget %.3f mm; mean %.4f, sigma %.4f, p99 %.4f, max %.4f mm; over budget %d ppm; **Cpk %.2f**."
          % (mc["budget_mm"], mc["mean_mm"], mc["sigma_mm"], mc["p99_mm"], mc["max_mm"],
             mc["ppm_over_budget"], mc["cpk"]), "",
          "## Critical tolerances (GD&T)", "",
          "| Feature | Nominal | Tolerance | Datum |", "|---|---|---|---|"]
    for t in TOLERANCES(p):
        L.append("| %s | %s | %s | %s |" % (t["feature"], t["nominal"],
                                            t["tolerance"][:48], t.get("datum", "")))
    L += ["", "## Process / assembly notes", ""]
    L += ["- " + n for n in general_notes()]
    L += ["", "See `docs/MANUFACTURING.md` (full process + 16-step assembly) and "
          "`project_details/` (per-part spec + assembly-feature catalog)."]
    return "\n".join(L)


def write_manufacturing_package(p: MotorParams, out_dir: str = "manufacturing") -> List[str]:
    """Write the complete manufacturing hand-off to one folder: BOM + hardware +
    tolerances CSVs, the DFM report, the Markdown summary and all 2D drawings."""
    import csv
    import os
    os.makedirs(out_dir, exist_ok=True)
    written: List[str] = []

    def _csv(name, header, rows):
        path = os.path.join(out_dir, name)
        with open(path, "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(header)
            for r in rows:
                wr.writerow(r)
        written.append(path)

    bom = bill_of_materials(p)
    _csv("bom.csv", ["component", "material", "qty", "mass_kg", "cost_usd", "note"],
         [[it["component"], it["material"], it["qty"], it["mass_kg"], it["cost_usd"], it.get("note", "")]
          for it in bom["line_items"]])
    _csv("hardware.csv", ["item", "standard", "size", "qty", "note"],
         [[h["item"], h["standard"], h["size"], h["qty"], h["note"]] for h in hardware_schedule(p)])
    _csv("tolerances.csv", ["feature", "nominal", "datum", "tolerance", "gdt", "rationale"],
         [[t["feature"], t["nominal"], t.get("datum", ""), t["tolerance"], t["gdt"], t["rationale"]]
          for t in TOLERANCES(p)])

    dfm_path = os.path.join(out_dir, "dfm_eccentricity.txt")
    with open(dfm_path, "w", encoding="utf-8") as fh:
        fh.write(dfm_report(p) + "\n")
    written.append(dfm_path)

    summary_path = os.path.join(out_dir, "manufacturing_summary.md")
    with open(summary_path, "w", encoding="utf-8") as fh:
        fh.write(manufacturing_summary_md(p) + "\n")
    written.append(summary_path)

    from . import drawings as _dwg  # lazy: drawings imports manufacturing
    written += _dwg.write_drawings(p, os.path.join(out_dir, "drawings"))
    return written


def tolerance_report(p: MotorParams) -> str:
    lines = ["Critical-dimension / GD&T scheme -- %s" % p.name,
             "  %-32s %-14s %-22s %s" % ("feature", "nominal", "tolerance", "GD&T")]
    for t in TOLERANCES(p):
        lines.append("  %-32s %-14s %-22s %s"
                     % (t["feature"][:32], t["nominal"][:14], t["tolerance"][:22], t["gdt"]))
    return "\n".join(lines)
