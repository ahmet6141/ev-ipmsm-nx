"""Chassis engineering: frame-rail section properties, a first-order mass estimate,
a (very rough) torsional-stiffness estimate, weight distribution / CG assumptions,
and a geometric buildability check. Pure math -- NX-independent and unit-tested
(the analogue of driveline_nx.engineering / motor_nx.em_design).

First-order closed-form estimates only. The torsional stiffness in particular is
an ORDER-OF-MAGNITUDE figure (single thin-walled closed section over the
wheelbase) -- verify the real number with FEA. Lengths are mm, masses kg.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, List

from .params import ChassisParams


def _hollow_rect_area(w: float, h: float, t: float) -> float:
    """Cross-section area of a hollow rectangle (outer w x h, wall t) in mm^2."""
    inner_w = max(0.0, w - 2.0 * t)
    inner_h = max(0.0, h - 2.0 * t)
    return w * h - inner_w * inner_h


def _hollow_rect_I(b: float, d: float, t: float) -> float:
    """Second moment of area of a hollow rectangle about its centroidal axis that
    is parallel to the `b` (width) dimension, i.e. bending in the `d` direction:
    I = (b*d^3 - bi*di^3) / 12,  with bi = b-2t, di = d-2t.  Units mm^4."""
    bi = max(0.0, b - 2.0 * t)
    di = max(0.0, d - 2.0 * t)
    return (b * d ** 3 - bi * di ** 3) / 12.0


@dataclass
class DerivedChassis:
    # frame rail box-beam section properties
    rail_area_mm2: float
    rail_Ix_mm4: float              # about the lateral (Y) axis -> vertical bending
    rail_Iy_mm4: float              # about the vertical (Z) axis -> lateral bending
    rail_Sx_mm3: float              # section modulus (vertical bending) = Ix / (h/2)
    rail_Sy_mm3: float              # section modulus (lateral bending)  = Iy / (w/2)
    # static vertical-bending load case (both rails, simply supported at the axles,
    # carrying the sprung mass + battery as a uniformly distributed load)
    static_payload_kg: float        # assumed sprung load reacted by the rails
    rail_max_bending_stress_mpa: float
    rail_bending_safety_factor: float
    rail_mid_deflection_mm: float
    # mass estimate + breakdown
    rail_mass_kg: float
    crossmember_mass_kg: float
    battery_tray_mass_kg: float
    total_mass_kg: float
    # torsional stiffness (ROUGH order-of-magnitude estimate, Nm/deg)
    torsional_stiffness_nm_per_deg: float
    # weight distribution / CG (rough assumptions)
    front_weight_fraction: float
    rear_weight_fraction: float
    cg_height_mm: float             # MASS-WEIGHTED vehicle CG (battery low + body high)
    battery_cg_height_mm: float     # battery-pack CG alone (the low floor pack)
    # battery-pack energy audit (against the tray internal cavity)
    pack_energy_kwh: float
    tray_cavity_volume_l: float         # gross internal cavity volume
    required_pack_density_wh_per_l: float   # energy / (cavity * usable_fraction)
    # track / wheelbase ratios
    track_avg_mm: float
    wheelbase_track_ratio: float


# 6082-T6 extruded aluminium reference properties (production rail material)
_E_ALU_MPA = 69_000.0      # Young's modulus (N/mm^2 = MPa)
_SIGMA_YIELD_MPA = 260.0   # 0.2% proof stress of 6082-T6 (MPa)
_G_ALU_PA = 26.0e9         # shear modulus (Pa)


def derive(p: ChassisParams) -> DerivedChassis:
    f, b, mat = p.frame, p.battery_tray, p.material
    rho = mat.density_kg_m3                       # kg/m^3
    mm3_to_m3 = 1.0e-9
    g_acc = 9.81                                   # m/s^2

    # ---- frame rail box-beam section properties --------------------------- #
    area = _hollow_rect_area(f.rail_width_mm, f.rail_height_mm, f.rail_wall_mm)   # mm^2
    # Ix: bending about the lateral (Y) axis -> resists vertical (Z) loads. The
    # "depth" in that bending is the rail HEIGHT. Iy: bending about the vertical
    # (Z) axis -> resists lateral (Y) loads; depth is the rail WIDTH.
    Ix = _hollow_rect_I(f.rail_width_mm, f.rail_height_mm, f.rail_wall_mm)        # mm^4
    Iy = _hollow_rect_I(f.rail_height_mm, f.rail_width_mm, f.rail_wall_mm)        # mm^4
    # section moduli S = I / c  (c = extreme-fibre distance). Vertical bending uses
    # the rail HEIGHT half-depth; lateral bending uses the WIDTH half-depth.
    Sx = Ix / (f.rail_height_mm / 2.0) if f.rail_height_mm > 0 else 0.0           # mm^3
    Sy = Iy / (f.rail_width_mm / 2.0) if f.rail_width_mm > 0 else 0.0             # mm^3

    # ---- mass estimate (sum of beam + tray-wall volumes) ------------------ #
    # two longitudinal rails over the overall length
    rail_vol_mm3 = 2.0 * area * f.overall_length_mm
    rail_mass = rail_vol_mm3 * mm3_to_m3 * rho

    # crossmembers span the inner channel between the rails
    cm_len = f.frame_inner_width_mm
    cm_area = _hollow_rect_area(f.crossmember_width_mm, f.crossmember_height_mm,
                                f.crossmember_wall_mm)
    cm_vol_mm3 = max(0, f.crossmember_count) * cm_area * cm_len
    cm_mass = cm_vol_mm3 * mm3_to_m3 * rho

    # battery tray: outer box minus inner cavity (the wall material only) + braces
    if b.enabled:
        outer = b.length_mm * b.width_mm * b.height_mm
        inner = (max(0.0, b.length_mm - 2.0 * b.wall_mm)
                 * max(0.0, b.width_mm - 2.0 * b.wall_mm)
                 * max(0.0, b.height_mm - 2.0 * b.wall_mm))
        tray_wall_vol = outer - inner
        # crossbraces: thin lateral webs spanning the tray width
        brace_vol = (max(0, b.crossbrace_count) * b.wall_mm
                     * max(0.0, b.width_mm - 2.0 * b.wall_mm)
                     * max(0.0, b.height_mm - 2.0 * b.wall_mm))
        tray_mass = (tray_wall_vol + brace_vol) * mm3_to_m3 * rho
    else:
        tray_mass = 0.0

    total_mass = rail_mass + cm_mass + tray_mass

    # ---- static vertical-bending load case -------------------------------- #
    # Treat the platform as two parallel rails, simply supported at the front/rear
    # axle stations (span = wheelbase), carrying the sprung load (vehicle kerb minus
    # unsprung corners, dominated by the floor battery) as a uniformly distributed
    # load w over the wheelbase. A simply-supported UDL beam has:
    #   M_max = w L^2 / 8       at mid-span
    #   delta_max = 5 w L^4 / (384 E I)
    # Split the load equally between the two rails. This is the classic first-order
    # bench check (verify the real distribution + local stresses with FEA).
    #
    # The sprung load now TRACKS THE MODEL (the adversarial-review MEDIUM finding: it
    # used to be a hardcoded 1650 kg independent of every param). If loads.static_
    # payload_kg is set (>0) it is used verbatim; otherwise it is derived as the
    # battery-pack mass (from energy / gravimetric density, only when the tray is
    # fitted) + the body/occupant allowance. Disabling/shrinking the tray now flows
    # through to the load.
    ld = p.loads
    pack_mass_kg = (b.energy_kwh * 1000.0 / ld.pack_gravimetric_wh_per_kg
                    if (b.enabled and ld.pack_gravimetric_wh_per_kg > 0) else 0.0)
    if ld.static_payload_kg > 0.0:
        payload_kg = ld.static_payload_kg
    else:
        payload_kg = pack_mass_kg + ld.body_occupant_mass_kg
    span_mm = max(1.0, f.wheelbase_mm)
    w_total_n_per_mm = payload_kg * g_acc / span_mm     # N/mm over the whole platform
    w_per_rail = w_total_n_per_mm / 2.0                 # N/mm per rail
    M_max_nmm = w_per_rail * span_mm ** 2 / 8.0          # N*mm
    sigma_mpa = (M_max_nmm / Sx) if Sx > 0 else float("inf")   # N/mm^2 = MPa
    sf = (_SIGMA_YIELD_MPA / sigma_mpa) if sigma_mpa > 0 else 0.0
    delta_mm = (5.0 * w_per_rail * span_mm ** 4 / (384.0 * _E_ALU_MPA * Ix)
                if Ix > 0 else float("inf"))

    # ---- torsional stiffness (ROUGH first-order estimate) ----------------- #
    # Treat the platform as a single thin-walled CLOSED rectangular tube of
    # width = frame_inner_width + 2*rail_width and depth = rail_height, with an
    # effective wall = rail_wall, twisted over the wheelbase. Bredt's torsion
    # constant J = 4 A_m^2 / integral(ds/t) = 4 A_m^2 * t / perimeter for a
    # uniform wall. K = G*J/L (Nm/rad), converted to Nm/deg. ORDER OF MAGNITUDE.
    G = _G_ALU_PA                                 # Pa, shear modulus of aluminium
    box_w = (f.frame_inner_width_mm + 2.0 * f.rail_width_mm) * 1e-3   # m
    box_h = f.rail_height_mm * 1e-3                                   # m
    t = f.rail_wall_mm * 1e-3                                         # m
    L = max(1e-3, f.wheelbase_mm * 1e-3)                             # m
    A_m = box_w * box_h                            # enclosed mid-wall area (m^2)
    perim = 2.0 * (box_w + box_h)                  # m
    J = (4.0 * A_m ** 2 * t / perim) if perim > 0 else 0.0   # m^4
    k_nm_per_rad = G * J / L                        # Nm/rad
    k_nm_per_deg = k_nm_per_rad * math.pi / 180.0   # Nm/deg

    # ---- weight distribution / CG (rough assumptions) --------------------- #
    # Skateboard EVs sit close to 50/50; the heavy floor battery keeps the CG low.
    front_frac = 0.50
    rear_frac = 1.0 - front_frac
    # The BATTERY-PACK CG alone sits at the tray mid-height above the ground (cells
    # fill the cavity, sitting low above the road). This is NOT the vehicle CG. The
    # tray bottom is the floor ground-clearance datum (see blueprint._z_layout); we
    # mirror that here to avoid importing the geometry module.
    _GROUND_CLEARANCE_MM = 140.0
    batt_cg_h = ((_GROUND_CLEARANCE_MM + b.height_mm / 2.0) if b.enabled
                 else f.rail_height_mm / 2.0 + 250.0)
    # The VEHICLE CG is the mass-weighted blend of the low battery pack and the much
    # higher body/occupant mass (~600-700 mm) -- a skateboard EV lands ~450-550 mm,
    # NOT the ~115 mm battery-only figure the old code reported under this name.
    body_m = max(0.0, ld.body_occupant_mass_kg)
    if (pack_mass_kg + body_m) > 0.0:
        cg_h = (pack_mass_kg * batt_cg_h + body_m * ld.body_occupant_cg_height_mm) / (pack_mass_kg + body_m)
    else:
        cg_h = batt_cg_h

    # ---- battery-pack energy audit (against the tray internal cavity) ----- #
    # The required GROSS density is energy / gross-cavity volume -- this is the figure
    # compared to the assumed pack-level density (a gross-pack number, ~200-300 Wh/L for
    # prismatic NCM/LFP). The usable_fraction then implies the CELL-level density the
    # active volume must hit (req_gross / usable_fraction), reported for context. With
    # the 130 mm internal height the 75 kWh pack lands at ~249 Wh/L gross (feasible);
    # the old 110 mm height demanded ~273 Wh/L gross, the optimistic edge.
    if b.enabled:
        cavity_mm3 = (max(0.0, b.length_mm - 2.0 * b.wall_mm)
                      * max(0.0, b.width_mm - 2.0 * b.wall_mm)
                      * max(0.0, b.height_mm - 2.0 * b.wall_mm))
        cavity_l = cavity_mm3 * 1e-6
        req_density = (b.energy_kwh * 1000.0 / cavity_l) if cavity_l > 0 else float("inf")
    else:
        cavity_l = 0.0
        req_density = 0.0

    # ---- track / wheelbase ratios ----------------------------------------- #
    track_avg = 0.5 * (f.track_front_mm + f.track_rear_mm)
    wb_track = (f.wheelbase_mm / track_avg) if track_avg > 0 else 0.0

    return DerivedChassis(
        rail_area_mm2=round(area, 1),
        rail_Ix_mm4=round(Ix, 1),
        rail_Iy_mm4=round(Iy, 1),
        rail_Sx_mm3=round(Sx, 1),
        rail_Sy_mm3=round(Sy, 1),
        static_payload_kg=round(payload_kg, 1),
        rail_max_bending_stress_mpa=round(sigma_mpa, 2),
        rail_bending_safety_factor=round(sf, 2),
        rail_mid_deflection_mm=round(delta_mm, 3),
        rail_mass_kg=round(rail_mass, 2),
        crossmember_mass_kg=round(cm_mass, 2),
        battery_tray_mass_kg=round(tray_mass, 2),
        total_mass_kg=round(total_mass, 2),
        torsional_stiffness_nm_per_deg=round(k_nm_per_deg, 1),
        front_weight_fraction=round(front_frac, 3),
        rear_weight_fraction=round(rear_frac, 3),
        cg_height_mm=round(cg_h, 1),
        battery_cg_height_mm=round(batt_cg_h, 1),
        pack_energy_kwh=round(b.energy_kwh if b.enabled else 0.0, 1),
        tray_cavity_volume_l=round(cavity_l, 1),
        required_pack_density_wh_per_l=round(req_density, 1),
        track_avg_mm=round(track_avg, 1),
        wheelbase_track_ratio=round(wb_track, 3),
    )


def mass_breakdown(p: ChassisParams) -> Dict[str, Any]:
    """Itemised mass estimate (kg) + the rough torsional-stiffness figure.
    The masses are bare structural blanks (no fasteners, lid, electronics)."""
    g = derive(p)
    return {
        "rail_mass_kg": g.rail_mass_kg,
        "crossmember_mass_kg": g.crossmember_mass_kg,
        "battery_tray_mass_kg": g.battery_tray_mass_kg,
        "total_structural_mass_kg": g.total_mass_kg,
        "rail_bending_stress_MPa": g.rail_max_bending_stress_mpa,
        "rail_bending_safety_factor": g.rail_bending_safety_factor,
        "rail_mid_deflection_mm": g.rail_mid_deflection_mm,
        "torsional_stiffness_Nm_per_deg_est": g.torsional_stiffness_nm_per_deg,
        "note": "order-of-magnitude structural estimate -- verify mass + stiffness with FEA",
    }


# --------------------------------------------------------------------------- #
# buildability check (geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: ChassisParams) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors driveline_nx.engineering.validate()'s contract."""
    issues: List[str] = []
    f, b, s, bm = p.frame, p.battery_tray, p.subframe, p.body_mount

    # positive principal dimensions
    for name, val in (("wheelbase_mm", f.wheelbase_mm),
                      ("overall_length_mm", f.overall_length_mm),
                      ("track_front_mm", f.track_front_mm),
                      ("track_rear_mm", f.track_rear_mm),
                      ("rail_width_mm", f.rail_width_mm),
                      ("rail_height_mm", f.rail_height_mm),
                      ("frame_inner_width_mm", f.frame_inner_width_mm)):
        if val <= 0:
            issues.append("frame.%s must be positive" % name)

    # rail box-beam wall must leave a hollow core
    if f.rail_wall_mm * 2.0 >= min(f.rail_width_mm, f.rail_height_mm):
        issues.append("frame.rail_wall_mm too thick: leaves no bore in the rail box beam")
    # crossmember wall must leave a hollow core
    if f.crossmember_wall_mm * 2.0 >= min(f.crossmember_width_mm, f.crossmember_height_mm):
        issues.append("frame.crossmember_wall_mm too thick: leaves no bore in the crossmember")

    # rails (inner channel + both rail widths) must fit inside the narrower track
    min_track = min(f.track_front_mm, f.track_rear_mm)
    if f.frame_inner_width_mm + 2.0 * f.rail_width_mm > min_track:
        issues.append("frame_inner_width + 2*rail_width (%.0f) exceeds track (%.0f): rails run wider than the wheels"
                      % (f.frame_inner_width_mm + 2.0 * f.rail_width_mm, min_track))

    # ICD §4.3: the rail spacing must BRACKET the ±track/2 mount pads -- the rail
    # centre-lines (where the subframe pads + hub stations sit) must lie inboard of
    # the wheels (else the pads would be outside the tyre). The subframe pads land on
    # the rail centre-line y = ±(frame_inner_width/2 + rail_width/2).
    rail_cl = f.frame_inner_width_mm / 2.0 + f.rail_width_mm / 2.0
    if rail_cl >= min_track / 2.0:
        issues.append("rail centre-line y=±%.0f does not bracket the ±track/2 (=%.0f) mount pads"
                      % (rail_cl, min_track / 2.0))

    # need at least two crossmembers to tie the rails
    if f.crossmember_count < 2:
        issues.append("frame.crossmember_count must be >= 2 to tie the rails")

    # ICD §4.3: overall length must cover the wheelbase PLUS both crush-can overhangs.
    # With the crush cans extending +X/-X beyond the axles, the minimum overall length
    # is the wheelbase itself; here we require strictly greater so an overhang exists.
    if f.wheelbase_mm >= f.overall_length_mm:
        issues.append("overall_length_mm (%.0f) must exceed wheelbase_mm + overhangs (%.0f)"
                      % (f.overall_length_mm, f.wheelbase_mm))

    # battery tray fit (only if fitted)
    if b.enabled:
        if b.length_mm <= 0 or b.width_mm <= 0 or b.height_mm <= 0:
            issues.append("battery_tray dimensions must be positive when enabled")
        if b.wall_mm * 2.0 >= min(b.width_mm, b.height_mm):
            issues.append("battery_tray.wall_mm too thick: leaves no cavity in the tray")
        # ICD §4.3: the sealed tray drops BETWEEN the rails, so it must fit inside the
        # inner channel with a side clearance on each face (seal / mounting flange).
        clr = max(0.0, getattr(b, "side_clearance_mm", 0.0))
        if b.width_mm + 2.0 * clr > f.frame_inner_width_mm:
            issues.append("battery_tray.width_mm + 2*side_clearance (%.0f) exceeds frame_inner_width (%.0f): "
                          "tray does not fit between the rails"
                          % (b.width_mm + 2.0 * clr, f.frame_inner_width_mm))
        if b.length_mm > f.wheelbase_mm:
            issues.append("battery_tray.length_mm exceeds the wheelbase (tray runs past the axles)")
        # ICD §6 auditability: the pack energy claim must be feasible for the tray
        # cavity. The required GROSS density (energy / usable cavity volume) must not
        # exceed the assumed pack-level density (the adversarial-review MEDIUM finding:
        # 75 kWh in the old 110 mm cavity demanded ~273 Wh/L gross / ~365 Wh/L usable,
        # beyond prismatic NCM/LFP pack-level density).
        g = derive(p)
        if b.energy_kwh > 0 and g.required_pack_density_wh_per_l > b.pack_density_wh_per_l + 1e-6:
            cell_density = g.required_pack_density_wh_per_l / max(1e-6, b.usable_fraction)
            issues.append(
                "battery_tray %.0f kWh needs %.0f Wh/L gross in the %.0f L cavity "
                "(~%.0f Wh/L usable at frac %.2f) -- exceeds the assumed pack density "
                "%.0f Wh/L; raise the tray internal height or lower energy_kwh"
                % (b.energy_kwh, g.required_pack_density_wh_per_l, g.tray_cavity_volume_l,
                   cell_density, b.usable_fraction, b.pack_density_wh_per_l))

    # mount counts
    if bm.body_mount_count < 1:
        issues.append("body_mount.body_mount_count must be >= 1")
    if (s.front_subframe or s.rear_subframe) and s.mount_bolt_count < 1:
        issues.append("subframe.mount_bolt_count must be >= 1 when a subframe is fitted")

    # ICD §3 crash-structure geometry: the crush cans must form the OVERHANGS beyond
    # the wheelbase and BUTT onto the rail ends -- they must NOT interpenetrate the
    # rails (the adversarial-review HIGH finding: previously the cans were fully buried
    # inside full-length rails, two coincident solids per overhang). Read back the
    # ACTUAL built world boxes of the rail + crush-can create-bodies (each is a +X box
    # beam sharing the rail Y/Z section, so any positive-length X overlap is a volume
    # interpenetration). The blueprint import is deferred so this stays a leaf module.
    issues.extend(_crush_can_overlap_issues(p))
    return issues


def _x_extent_of_prism(step) -> tuple:
    """Min/max world X of a +X-axis prism build step (rails, crush cans). Uses the
    step's own origin3 + length so it tracks whatever the blueprint actually built."""
    o = step.origin3
    return (o[0], o[0] + step.length)


def _crush_can_overlap_issues(p: ChassisParams) -> List[str]:
    """Assert no crush-can create-body interpenetrates a rail create-body along X
    (they share the Y/Z section, so an X overlap is a solid-on-solid burial). A
    touch at the shared end face (overlap ~0) is fine."""
    from . import blueprint as _bp           # deferred: avoid an import cycle
    out: List[str] = []
    steps = {s.id: s for s in _bp.build_steps(p) if s.boolean == "create"}
    rails = [steps[k] for k in ("rail_l", "rail_r") if k in steps]
    cans = [(sid, s) for sid, s in steps.items() if s.role == "crush_can"]
    for sid, can in cans:
        c_lo, c_hi = _x_extent_of_prism(can)
        for rail in rails:
            r_lo, r_hi = _x_extent_of_prism(rail)
            ov = min(c_hi, r_hi) - max(c_lo, r_lo)
            if ov > 1e-6:
                out.append(
                    "crush can %s overlaps rail %s along X by %.0f mm (it must butt onto "
                    "the rail end, not bury inside it)" % (sid, rail.id, ov))
                break
    return out


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def report(p: ChassisParams) -> str:
    g = derive(p)
    f, b, s, bm, mat = p.frame, p.battery_tray, p.subframe, p.body_mount, p.material
    issues = validate(p)
    lines = [
        "Chassis design summary -- %s" % p.name,
        "  layout                   : skateboard, wheelbase %.0f mm, track %.0f/%.0f mm (f/r)" % (
            f.wheelbase_mm, f.track_front_mm, f.track_rear_mm),
        "  overall length           : %.0f mm  (wheelbase/track %.2f)" % (
            f.overall_length_mm, g.wheelbase_track_ratio),
        "  frame rail (box beam)    : %.0fx%.0f mm, %.0f mm wall  (inner spacing %.0f mm)" % (
            f.rail_width_mm, f.rail_height_mm, f.rail_wall_mm, f.frame_inner_width_mm),
        "  rail section             : A %.0f mm^2, Ix %.3g mm^4 (vert), Iy %.3g mm^4 (lat)" % (
            g.rail_area_mm2, g.rail_Ix_mm4, g.rail_Iy_mm4),
        "  rail section modulus     : Sx %.3g mm^3 (vert), Sy %.3g mm^3 (lat)" % (
            g.rail_Sx_mm3, g.rail_Sy_mm3),
        "  static bending (%.0f kg) : sigma %.1f MPa, SF %.1f, mid-span deflection %.2f mm" % (
            g.static_payload_kg, g.rail_max_bending_stress_mpa,
            g.rail_bending_safety_factor, g.rail_mid_deflection_mm),
        "                             (sprung load derived from the model; deflection uses rail-only Ix -- a conservative bound)",
        "  crossmembers             : %d x %.0fx%.0f mm, %.0f mm wall" % (
            f.crossmember_count, f.crossmember_width_mm, f.crossmember_height_mm, f.crossmember_wall_mm),
        "  battery tray             : %s" % (
            "%.0f x %.0f x %.0f mm, %.0f mm wall, %d braces%s" % (
                b.length_mm, b.width_mm, b.height_mm, b.wall_mm, b.crossbrace_count,
                " (sealed)" if b.sealed else "") if b.enabled else "none (deleted)"),
        "  subframes                : %s%s, %d bolts Ø%.0f, %d motor mounts" % (
            "front" if s.front_subframe else "", "+rear" if s.rear_subframe else "",
            s.mount_bolt_count, s.mount_bolt_diameter_mm, s.motor_mount_count),
        "  body mounts / crash      : %d holes Ø%.0f, crush cans %s/%s (f/r)" % (
            bm.body_mount_count, bm.body_mount_diameter_mm,
            "yes" if bm.crush_can_front else "no", "yes" if bm.crush_can_rear else "no"),
        "  material                 : %s (%.0f kg/m^3), %s" % (
            mat.rail_material, mat.density_kg_m3, mat.joining),
        "  mass estimate (structure): %.1f kg  (rails %.1f + crossmembers %.1f + tray %.1f)" % (
            g.total_mass_kg, g.rail_mass_kg, g.crossmember_mass_kg, g.battery_tray_mass_kg),
        "  torsional stiffness      : ~%.0f Nm/deg  (ROUGH single-tube estimate -- verify by FEA)" % (
            g.torsional_stiffness_nm_per_deg),
        "  battery pack (energy)    : %s" % (
            "%.0f kWh in %.0f L cavity -> needs %.0f Wh/L gross (pack assumed %.0f Wh/L)" % (
                g.pack_energy_kwh, g.tray_cavity_volume_l, g.required_pack_density_wh_per_l,
                b.pack_density_wh_per_l) if b.enabled else "none"),
        "  weight dist / CG (rough) : %.0f/%.0f f/r, vehicle CG ~%.0f mm (battery-pack CG ~%.0f mm)" % (
            g.front_weight_fraction * 100.0, g.rear_weight_fraction * 100.0,
            g.cg_height_mm, g.battery_cg_height_mm),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
