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
    cg_height_mm: float
    # track / wheelbase ratios
    track_avg_mm: float
    wheelbase_track_ratio: float


def derive(p: ChassisParams) -> DerivedChassis:
    f, b, mat = p.frame, p.battery_tray, p.material
    rho = mat.density_kg_m3                       # kg/m^3
    mm3_to_m3 = 1.0e-9

    # ---- frame rail box-beam section properties --------------------------- #
    area = _hollow_rect_area(f.rail_width_mm, f.rail_height_mm, f.rail_wall_mm)   # mm^2
    # Ix: bending about the lateral (Y) axis -> resists vertical (Z) loads. The
    # "depth" in that bending is the rail HEIGHT. Iy: bending about the vertical
    # (Z) axis -> resists lateral (Y) loads; depth is the rail WIDTH.
    Ix = _hollow_rect_I(f.rail_width_mm, f.rail_height_mm, f.rail_wall_mm)        # mm^4
    Iy = _hollow_rect_I(f.rail_height_mm, f.rail_width_mm, f.rail_wall_mm)        # mm^4

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

    # ---- torsional stiffness (ROUGH first-order estimate) ----------------- #
    # Treat the platform as a single thin-walled CLOSED rectangular tube of
    # width = frame_inner_width + 2*rail_width and depth = rail_height, with an
    # effective wall = rail_wall, twisted over the wheelbase. Bredt's torsion
    # constant J = 4 A_m^2 / integral(ds/t) = 4 A_m^2 * t / perimeter for a
    # uniform wall. K = G*J/L (Nm/rad), converted to Nm/deg. ORDER OF MAGNITUDE.
    G = 26.0e9                                    # Pa, shear modulus of aluminium
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
    # CG height ~ a bit above the battery tray mid-height (battery dominates mass,
    # sits low); a deliberately rough placeholder.
    cg_h = (b.height_mm / 2.0 + 60.0) if b.enabled else (f.rail_height_mm / 2.0 + 250.0)

    # ---- track / wheelbase ratios ----------------------------------------- #
    track_avg = 0.5 * (f.track_front_mm + f.track_rear_mm)
    wb_track = (f.wheelbase_mm / track_avg) if track_avg > 0 else 0.0

    return DerivedChassis(
        rail_area_mm2=round(area, 1),
        rail_Ix_mm4=round(Ix, 1),
        rail_Iy_mm4=round(Iy, 1),
        rail_mass_kg=round(rail_mass, 2),
        crossmember_mass_kg=round(cm_mass, 2),
        battery_tray_mass_kg=round(tray_mass, 2),
        total_mass_kg=round(total_mass, 2),
        torsional_stiffness_nm_per_deg=round(k_nm_per_deg, 1),
        front_weight_fraction=round(front_frac, 3),
        rear_weight_fraction=round(rear_frac, 3),
        cg_height_mm=round(cg_h, 1),
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

    # need at least two crossmembers to tie the rails
    if f.crossmember_count < 2:
        issues.append("frame.crossmember_count must be >= 2 to tie the rails")

    # wheelbase must be shorter than the overall vehicle length
    if f.wheelbase_mm >= f.overall_length_mm:
        issues.append("wheelbase_mm must be less than overall_length_mm")

    # battery tray fit (only if fitted)
    if b.enabled:
        if b.length_mm <= 0 or b.width_mm <= 0 or b.height_mm <= 0:
            issues.append("battery_tray dimensions must be positive when enabled")
        if b.wall_mm * 2.0 >= min(b.width_mm, b.height_mm):
            issues.append("battery_tray.wall_mm too thick: leaves no cavity in the tray")
        if b.width_mm > f.frame_inner_width_mm:
            # not a hard stop -- the tray can overhang the rail tops -- but warn if
            # it would not even fit within the track envelope.
            margin = 60.0
            if b.width_mm > min_track - margin:
                issues.append("battery_tray.width_mm (%.0f) exceeds track-margin (%.0f): tray will not fit"
                              % (b.width_mm, min_track - margin))
        if b.length_mm > f.wheelbase_mm:
            issues.append("battery_tray.length_mm exceeds the wheelbase (tray runs past the axles)")

    # mount counts
    if bm.body_mount_count < 1:
        issues.append("body_mount.body_mount_count must be >= 1")
    if (s.front_subframe or s.rear_subframe) and s.mount_bolt_count < 1:
        issues.append("subframe.mount_bolt_count must be >= 1 when a subframe is fitted")
    return issues


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
        "  weight dist / CG (rough) : %.0f/%.0f f/r, CG height ~%.0f mm (battery low)" % (
            g.front_weight_fraction * 100.0, g.rear_weight_fraction * 100.0, g.cg_height_mm),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
