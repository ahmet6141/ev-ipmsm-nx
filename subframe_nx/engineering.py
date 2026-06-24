"""Subframe (cradle) engineering: cradle load paths + a first-order stiffness
estimate, structural mass, chassis-pad + pickup bolt sizing, and a geometric
buildability check. Pure math -- NX-independent and unit-tested (the analogue of
chassis_nx.engineering / driveline_nx.engineering).

The cradle closes the chassis <-> suspension joint (ICD §7.2): the suspension corner
loads enter at the inboard PICKUP BOSSES, run through the perimeter box beams, and
exit UP through the chassis-pad riser posts into the chassis rails (and partly into
the e-axle mounts). All estimates are first-order closed form -- verify the real load
paths + cradle stiffness with FEA. Lengths mm, masses kg, forces N, stress MPa.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Tuple

from .params import SubframeParams

Vec3 = Tuple[float, float, float]


def _hollow_rect_area(w: float, h: float, t: float) -> float:
    """Cross-section area of a hollow rectangle (outer w x h, wall t) in mm^2."""
    inner_w = max(0.0, w - 2.0 * t)
    inner_h = max(0.0, h - 2.0 * t)
    return w * h - inner_w * inner_h


def _hollow_rect_I(b: float, d: float, t: float) -> float:
    """Second moment of area of a hollow rectangle about the centroidal axis parallel
    to `b` (bending in the `d` direction): I = (b*d^3 - bi*di^3)/12.  Units mm^4."""
    bi = max(0.0, b - 2.0 * t)
    di = max(0.0, d - 2.0 * t)
    return (b * d ** 3 - bi * di ** 3) / 12.0


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# a representative corner vertical load (N) at the wheel used to drive the pickup /
# bolt sizing when params.corner_vertical_load_n is not set. ~600 kg corner * a 2.0
# dynamic factor -> ~12 kN at the contact patch; the lower arm reacts the bulk of it.
_DEFAULT_CORNER_LOAD_N = 12_000.0
_E_ALU_MPA = 69_000.0       # Young's modulus of aluminium (N/mm^2)
# representative wall (mm) for the shock-tower POST in the mass estimate -- the post is an
# envelope blank in the geometry; a real tower is a thin-walled cast/extruded section, so
# the kg figure uses a hollow correction (low-severity review note: tower mass realism).
_TOWER_WALL_MM = 8.0
# proof shear strength of a typical M-class 8.8 bolt (~0.6 * 640 MPa proof) -- used to
# size the chassis-pad + pickup bolts (single-shear), a static screen only.
_BOLT_SHEAR_ALLOW_MPA = 384.0


@dataclass
class DerivedSubframe:
    axle: str
    # perimeter box-beam section properties (the cradle side rails / crossbeams)
    beam_area_mm2: float
    beam_Iz_mm4: float                  # vertical-bending second moment of the side rail
    beam_Sz_mm3: float                  # vertical-bending section modulus
    # cradle load path: the corner vertical load splits between the two chassis-pad
    # posts of a side; each side rail spans pad-to-pad carrying the pickup load.
    corner_load_n: float
    side_rail_span_mm: float            # fore/aft span between the two pads on a side
    side_rail_bending_stress_mpa: float
    side_rail_safety_factor: float
    side_rail_deflection_mm: float      # cradle vertical compliance at the pickup (rough)
    cradle_vertical_stiffness_n_per_mm: float
    # mass estimate + breakdown (one subframe)
    cradle_mass_kg: float
    pad_post_mass_kg: float
    tower_mass_kg: float
    boss_mass_kg: float
    total_mass_kg: float
    # bolt sizing (chassis-pad bolts react the pad reaction; pickup bolts the link load)
    pad_bolt_shear_stress_mpa: float
    pad_bolt_safety_factor: float
    pickup_bolt_shear_stress_mpa: float
    pickup_bolt_safety_factor: float
    # geometry tie facts (ICD §7.4.2) -- read from the hardpoint table / pad accessors
    tower_reach_z_mm: float             # world Z the tower top reaches (must hit damper top)
    damper_top_z_mm: float              # the suspension damper/strut-top hardpoint Z
    n_pickups: int                      # inboard pickup bosses per side
    n_pads: int                         # chassis-pad posts (fore/aft x left/right)


def _corner_load(p: SubframeParams) -> float:
    return p.corner_vertical_load_n if p.corner_vertical_load_n > 0 else _DEFAULT_CORNER_LOAD_N


def derive(p: SubframeParams) -> DerivedSubframe:
    c, pad, boss, tw, ea, mat = (p.cradle, p.pad, p.boss, p.tower, p.eaxle, p.material)
    rho = mat.density_kg_m3
    mm3_to_m3 = 1.0e-9

    # ---- perimeter box-beam section properties (the cradle side rail) ------ #
    area = _hollow_rect_area(c.beam_width_mm, c.beam_height_mm, c.beam_wall_mm)   # mm^2
    # vertical bending of the side rail bends about its lateral axis: depth = beam height
    Iz = _hollow_rect_I(c.beam_width_mm, c.beam_height_mm, c.beam_wall_mm)        # mm^4
    Sz = Iz / (c.beam_height_mm / 2.0) if c.beam_height_mm > 0 else 0.0           # mm^3

    # ---- cradle load path (first-order) ------------------------------------ #
    # The corner vertical load enters the side rail at the pickups and is reacted at the
    # two chassis-pad posts (fore + aft) on that side: model the side rail as a simply
    # supported beam spanning pad-to-pad (= 2*pad_x_local) with the load at mid-span.
    #   M_max = P L / 4,  delta = P L^3 / (48 E I).
    corner = _corner_load(p)
    span = max(1.0, 2.0 * pad.pad_x_local_mm)
    P = corner                                  # the side rail reacts the corner load
    M_max_nmm = P * span / 4.0
    sigma = (M_max_nmm / Sz) if Sz > 0 else float("inf")
    sf = (mat.yield_strength_mpa / sigma) if sigma > 0 else 0.0
    delta = (P * span ** 3 / (48.0 * _E_ALU_MPA * Iz)) if Iz > 0 else float("inf")
    k_vert = (P / delta) if delta > 0 else float("inf")     # N/mm

    # ---- mass estimate (sum of beam + post + tower + boss volumes) --------- #
    # perimeter: two side rails (length side_rail_length) + two crossbeams spanning the
    # pad-to-pad lateral width (2*pad_y). All share the box-beam section.
    side_len = c.side_rail_length_mm
    cross_len = 2.0 * pad.pad_y_mm
    # perimeter = 2 side rails + 2 crossbeams + 2 inboard pickup stringers (review finding 5)
    perimeter_len = 2.0 * side_len + 2.0 * cross_len + 2.0 * side_len
    cradle_mass = area * perimeter_len * mm3_to_m3 * rho

    # chassis-pad riser posts (solid cylinder from base plane to pad) + flange disc
    post_h = max(0.0, pad.pad_z_mm - c.base_plane_z_mm)
    post_vol = math.pi * (pad.post_diameter_mm / 2.0) ** 2 * post_h
    flange_vol = math.pi * (pad.flange_diameter_mm / 2.0) ** 2 * pad.flange_thickness_mm
    n_pads = 4
    pad_mass = n_pads * (post_vol + flange_vol) * mm3_to_m3 * rho

    # towers (post from base plane to damper top + seat plate), one per side. The tower
    # post is modelled as a SOLID envelope blank in the blueprint, but a real cast/extruded
    # tower is a thin-walled section; a solid Ø70x442 bar would dominate the cradle mass
    # (~10 kg of ~25 kg) and overstate it. Apply a representative wall to the post volume in
    # the mass integral (the geometry/stiffness use the full envelope; only the kg figure
    # needs the hollow correction -- low-severity review note).
    tower_h = max(0.0, p.hardpoints_local("l")["damper_top"][2] - c.base_plane_z_mm)
    r_o = tw.post_diameter_mm / 2.0
    r_i = max(0.0, r_o - _TOWER_WALL_MM)
    tower_post_vol = math.pi * (r_o ** 2 - r_i ** 2) * tower_h
    seat_vol = math.pi * (tw.seat_diameter_mm / 2.0) ** 2 * tw.seat_thickness_mm
    tower_mass = 2.0 * (tower_post_vol + seat_vol) * mm3_to_m3 * rho

    # pickup bosses (solid cylinder per hardpoint, both sides) + e-axle mount bosses
    n_pickups = 5                               # lower fore/aft, upper fore/aft, toe
    boss_vol = math.pi * (boss.boss_diameter_mm / 2.0) ** 2 * boss.boss_length_mm
    ea_vol = (ea.mount_count * math.pi * (ea.boss_diameter_mm / 2.0) ** 2 * ea.boss_height_mm
              if ea.enabled else 0.0)
    boss_mass = (2.0 * n_pickups * boss_vol + ea_vol) * mm3_to_m3 * rho

    total_mass = cradle_mass + pad_mass + tower_mass + boss_mass

    # ---- bolt sizing (single-shear static screen) ------------------------- #
    # The four chassis-pad posts share the total reacted load (corner load + a share of
    # the diff/e-axle mass); each pad's bolt group carries pad_reaction / bolt_count in
    # shear. Approximate the total vertical reaction as the corner load (the dominant
    # term for the cradle). tau = F / (n_bolts * A).
    pad_reaction = corner / n_pads
    a_pad = math.pi * (pad.bolt_diameter_mm / 2.0) ** 2
    tau_pad = (pad_reaction / (max(1, pad.bolt_count) * a_pad)) if a_pad > 0 else float("inf")
    sf_pad = (_BOLT_SHEAR_ALLOW_MPA / tau_pad) if tau_pad > 0 else 0.0

    # each pickup bolt reacts the link load. Take the lower-arm pickup as the worst case:
    # the lower arm reacts ~the full corner vertical load across its two pickups.
    pickup_load = corner / 2.0
    a_pick = math.pi * (boss.bore_diameter_mm / 2.0) ** 2
    tau_pick = (pickup_load / a_pick) if a_pick > 0 else float("inf")
    sf_pick = (_BOLT_SHEAR_ALLOW_MPA / tau_pick) if tau_pick > 0 else 0.0

    hp = p.hardpoints_local("l")
    return DerivedSubframe(
        axle=p.axle,
        beam_area_mm2=round(area, 1),
        beam_Iz_mm4=round(Iz, 1),
        beam_Sz_mm3=round(Sz, 1),
        corner_load_n=round(corner, 1),
        side_rail_span_mm=round(span, 1),
        side_rail_bending_stress_mpa=round(sigma, 2),
        side_rail_safety_factor=round(sf, 2),
        side_rail_deflection_mm=round(delta, 3),
        cradle_vertical_stiffness_n_per_mm=round(k_vert, 1),
        cradle_mass_kg=round(cradle_mass, 2),
        pad_post_mass_kg=round(pad_mass, 2),
        tower_mass_kg=round(tower_mass, 2),
        boss_mass_kg=round(boss_mass, 2),
        total_mass_kg=round(total_mass, 2),
        pad_bolt_shear_stress_mpa=round(tau_pad, 2),
        pad_bolt_safety_factor=round(sf_pad, 2),
        pickup_bolt_shear_stress_mpa=round(tau_pick, 2),
        pickup_bolt_safety_factor=round(sf_pick, 2),
        tower_reach_z_mm=round(_tower_top_z(p), 2),
        damper_top_z_mm=round(hp["damper_top"][2], 2),
        n_pickups=n_pickups,
        n_pads=n_pads,
    )


def _tower_top_z(p: SubframeParams) -> float:
    """World Z the tower top reaches -- by construction the seat top sits AT the
    damper-top hardpoint (the blueprint places the seat there), so the tower reach is
    the damper-top Z. Reported so validate() can confirm the build closes the gap."""
    return p.hardpoints_local("l")["damper_top"][2]


def mass_breakdown(p: SubframeParams) -> Dict[str, Any]:
    """Itemised mass estimate (kg) for ONE subframe + the load-path facts. Bare
    structural blanks (no fasteners, bushings, paint)."""
    g = derive(p)
    return {
        "cradle_mass_kg": g.cradle_mass_kg,
        "pad_post_mass_kg": g.pad_post_mass_kg,
        "tower_mass_kg": g.tower_mass_kg,
        "boss_mass_kg": g.boss_mass_kg,
        "total_structural_mass_kg": g.total_mass_kg,
        "side_rail_bending_stress_MPa": g.side_rail_bending_stress_mpa,
        "side_rail_safety_factor": g.side_rail_safety_factor,
        "cradle_vertical_stiffness_N_per_mm_est": g.cradle_vertical_stiffness_n_per_mm,
        "pad_bolt_safety_factor": g.pad_bolt_safety_factor,
        "pickup_bolt_safety_factor": g.pickup_bolt_safety_factor,
        "note": "order-of-magnitude structural estimate -- verify load paths + stiffness with FEA",
    }


# --------------------------------------------------------------------------- #
# buildability check (geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: SubframeParams) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors chassis_nx.engineering.validate()'s contract. Enforces the ICD §7.4.2
    mating ties: pads align to the chassis y=±585 rail tops, the pickup bosses match
    the suspension inboard hardpoints, and the tower reaches the damper top."""
    issues: List[str] = []
    c, pad, boss, tw, ea = p.cradle, p.pad, p.boss, p.tower, p.eaxle

    if p.axle not in ("front", "rear"):
        issues.append("axle '%s' unknown (front|rear)" % p.axle)

    # positive principal dimensions + hollow-section sanity
    for name, val in (("cradle.beam_width_mm", c.beam_width_mm),
                      ("cradle.beam_height_mm", c.beam_height_mm),
                      ("cradle.side_rail_length_mm", c.side_rail_length_mm),
                      ("pad.post_diameter_mm", pad.post_diameter_mm),
                      ("pad.flange_diameter_mm", pad.flange_diameter_mm),
                      ("tower.post_diameter_mm", tw.post_diameter_mm),
                      ("boss.boss_diameter_mm", boss.boss_diameter_mm)):
        if val <= 0:
            issues.append("%s must be positive" % name)
    if c.beam_wall_mm * 2.0 >= min(c.beam_width_mm, c.beam_height_mm):
        issues.append("cradle.beam_wall_mm too thick: leaves no bore in the box beam")
    if boss.bore_diameter_mm >= boss.boss_diameter_mm:
        issues.append("boss.bore_diameter_mm must be smaller than boss.boss_diameter_mm")
    if tw.rod_bore_diameter_mm >= tw.seat_diameter_mm:
        issues.append("tower.rod_bore_diameter_mm must be smaller than the seat OD")
    if pad.flange_diameter_mm <= pad.bolt_diameter_mm * 2.0:
        issues.append("pad.flange_diameter_mm too small for the bolt circle")

    # ICD §7.4.2 (a): chassis pads must align to the chassis rail-top mounts at
    # y = ±585, z ≈ 400 at the axle x-station. The chassis builds Subframe_Boss there
    # (chassis_nx.blueprint.subframe_pad_centre); read it back if available and confirm
    # this subframe's pad_centre_local lands on it. The import is deferred so this stays
    # a leaf module that imports without the chassis package present.
    issues.extend(_chassis_pad_alignment_issues(p))

    # ICD §7.4.2 (b): the pickup bosses must match the suspension inboard hardpoints.
    # The hardpoint table IS the single source of truth, so we confirm the suspension
    # package (if present) reports the SAME inboard pickups (after mapping its hub-frame
    # to the subframe local frame). A drift here means a boss would float.
    issues.extend(_suspension_pickup_match_issues(p))

    # ICD §7.4.2 (c): the tower must reach the damper/strut top so the spring is not
    # floating. The blueprint seats the tower top at damper_top by construction; assert
    # the tower post is long enough (base plane below the damper top) and the damper top
    # is above the upper pickups (a real shock tower rises above the arms).
    hp = p.hardpoints_local("l")
    damper_z = hp["damper_top"][2]
    if c.base_plane_z_mm >= damper_z:
        issues.append("cradle.base_plane_z_mm %.0f is at/above the damper top %.0f: the tower has no height"
                      % (c.base_plane_z_mm, damper_z))
    upper_z = hp["upper_pickup_fore"][2]
    if damper_z <= upper_z:
        issues.append("damper top %.0f must rise above the upper pickups %.0f (a shock tower)"
                      % (damper_z, upper_z))

    # ICD §7.2 / review finding 5: each suspension pickup boss must be CARRIED by the
    # perimeter -- its tie leg must reach a cradle perimeter body (the inboard stringer /
    # a crossbeam / a side rail), not bottom out in empty space. Assert connectivity so a
    # floating boss (the broken chassis<-pickup load path) is flagged.
    issues.extend(_pickup_leg_connectivity_issues(p))

    # the cradle side rails must fore/aft SPAN the pickups + pads they carry (else a
    # boss would sit off the end of the rail with nothing under it).
    xs = [hp[n][0] for n in ("lower_pickup_fore", "lower_pickup_aft", "toe_pickup",
                             "upper_pickup_fore", "upper_pickup_aft")]
    xs += [+pad.pad_x_local_mm * (1.0 if p.axle == "rear" else -1.0),
           -pad.pad_x_local_mm * (1.0 if p.axle == "rear" else -1.0)]
    need_span = max(xs) - min(xs)
    if c.side_rail_length_mm < need_span:
        issues.append("cradle.side_rail_length_mm %.0f does not span the fore/aft pickup+pad spread %.0f mm"
                      % (c.side_rail_length_mm, need_span))

    # e-axle mounts must sit inboard of the cradle perimeter (between the pad Y rails)
    if ea.enabled and ea.mount_y_mm >= pad.pad_y_mm:
        issues.append("eaxle.mount_y_mm %.0f must sit inboard of the pad rails y=±%.0f"
                      % (ea.mount_y_mm, pad.pad_y_mm))
    if ea.enabled and ea.mount_count < 1:
        issues.append("eaxle.mount_count must be >= 1 when the e-axle mounts are enabled")

    # bolt counts
    if pad.bolt_count < 1:
        issues.append("pad.bolt_count must be >= 1")
    if tw.bolt_count < 1:
        issues.append("tower.bolt_count must be >= 1")

    # engineering margins (reported, not hard stops): the cradle is fatigue-loaded, so a
    # >= 1.5 static screen backs the (separate) fatigue check.
    g = derive(p)
    if g.side_rail_safety_factor < 1.5:
        issues.append("cradle side-rail bending SF %.2f < 1.5 (increase beam section)"
                      % g.side_rail_safety_factor)
    if g.pad_bolt_safety_factor < 1.5:
        issues.append("chassis-pad bolt shear SF %.2f < 1.5 (more/larger bolts)"
                      % g.pad_bolt_safety_factor)
    if g.pickup_bolt_safety_factor < 1.5:
        issues.append("pickup bolt shear SF %.2f < 1.5 (larger pickup bore bolt)"
                      % g.pickup_bolt_safety_factor)
    return issues


def _pickup_leg_connectivity_issues(p: SubframeParams) -> List[str]:
    """Each pickup boss's tie leg must overlap a cradle perimeter body at the base plane
    so the boss is carried into the perimeter (review finding 5). The legs descend to the
    base-plane Z band at the pickup X/Y; the inboard pickup stringer runs fore/aft at
    |Y| = stringer_y over the pickup band. Assert every leg's footprint falls within a
    perimeter body's XY footprint at the base plane (NX-free, analytic to the blueprint)."""
    out: List[str] = []
    c, boss = p.cradle, p.boss
    half_len = c.side_rail_length_mm / 2.0
    sec = boss.boss_diameter_mm / 2.0

    # perimeter body XY footprints at the base plane (the leg foot lands in this Z band):
    #   inboard stringer (per side): X in [-half_len, half_len], |Y| in stringer band
    #   front/rear crossbeams:       X in [+/-half_len -/+ embed .. ], full Y
    #   side rails:                  X in [-half_len, half_len], |Y| in rail band
    sy_lo = c.stringer_y_mm - c.stringer_width_mm / 2.0
    sy_hi = c.stringer_y_mm + c.stringer_width_mm / 2.0
    rail_lo = p.pad.pad_y_mm - c.beam_width_mm / 2.0
    rail_hi = p.pad.pad_y_mm + c.beam_width_mm / 2.0

    def _leg_supported(lx: float, ly: float) -> bool:
        ay = abs(ly)
        in_x = (-half_len - 1e-6) <= lx <= (half_len + 1e-6)
        # inboard stringer band OR side-rail band, anywhere along X
        if in_x and (sy_lo - sec <= ay <= sy_hi + sec):
            return True
        if in_x and (rail_lo - sec <= ay <= rail_hi + sec):
            return True
        return False

    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        for nm in _PICKUP_LEG_NAMES:
            lx, ly, _lz = hp[nm]
            if not _leg_supported(lx, ly):
                out.append(
                    "pickup boss %s_%s leg bottoms out in empty space (no cradle perimeter "
                    "body under |Y|=%.0f) -- the boss floats; widen/relocate the inboard "
                    "stringer (review finding 5)" % (nm, side, abs(ly)))
    return out


# the inboard hardpoints whose pickup bosses get a tie leg (mirrors the blueprint).
_PICKUP_LEG_NAMES = ("lower_pickup_fore", "lower_pickup_aft",
                     "upper_pickup_fore", "upper_pickup_aft", "toe_pickup")


def _chassis_pad_alignment_issues(p: SubframeParams) -> List[str]:
    """Confirm this subframe's chassis-pad mating faces land on the chassis subframe
    mount pads (ICD §7.2: rail centre-line y=±585, rail top z≈400 at the axle station).
    Reads chassis_nx if present; otherwise just checks the pad params themselves."""
    out: List[str] = []
    # the subframe's own pad Y must be the rail centre-line (±585); the chassis is the
    # authority, so cross-check it when available.
    try:
        from chassis_nx.blueprint import subframe_pad_centre
        from chassis_nx.params import ChassisParams
        cp = ChassisParams()
        # the chassis pad is at the axle station; in the subframe LOCAL frame the axle
        # station is X=0, so compare the |Y| and Z only (the X offset pad_x_local is the
        # subframe's own fore/aft mounting spread, not a chassis coordinate).
        veh = subframe_pad_centre(cp, p.axle, "l")            # (x, +y, rail_top)
        if abs(abs(veh[1]) - p.pad.pad_y_mm) > 1.0:
            out.append("pad.pad_y_mm %.0f != chassis rail centre-line |Y| %.0f (ICD §7.2)"
                       % (p.pad.pad_y_mm, abs(veh[1])))
        if abs(veh[2] - p.pad.pad_z_mm) > 2.0:
            out.append("pad.pad_z_mm %.0f != chassis rail-top Z %.0f (ICD §7.2)"
                       % (p.pad.pad_z_mm, veh[2]))
    except Exception:
        # chassis package absent -- fall back to the ICD literals (y=585, z≈400)
        if abs(p.pad.pad_y_mm - 585.0) > 1.0:
            out.append("pad.pad_y_mm %.0f != ICD chassis rail centre-line 585 mm" % p.pad.pad_y_mm)
        if abs(p.pad.pad_z_mm - 400.0) > 5.0:
            out.append("pad.pad_z_mm %.0f != ICD chassis rail-top ~400 mm" % p.pad.pad_z_mm)
    return out


def _suspension_pickup_match_issues(p: SubframeParams) -> List[str]:
    """Confirm the subframe pickup bosses coincide with the suspension inboard hardpoints
    on BOTH sides (ICD §7.4.2). The suspension corner is hub-datumed (origin = hub centre
    = vehicle HUB_CENTRE) and the vehicle assembly places it at
    ``HUB_CENTRE(axle, side) + Rz(side).hp_local``; the subframe is placed IDENTITY at the
    axle station, so a suspension point maps into the subframe LOCAL frame as

        subframe_local = Rz(side).hp_local + (0, side_sign*track/2, tyre_radius)

    -- the SAME convention hardpoints_local() builds from. This is the headline 'no
    floating arm' tie. It is independent of the axle (the canonical corner is reused front
    and rear -- NO front X mirror), so a regression to an independently-mirrored table is
    caught on every side. Reads suspension_nx if present; otherwise the assembly enforces
    it."""
    out: List[str] = []
    try:
        from suspension_nx.engineering import hardpoints as s_hardpoints
        from suspension_nx.params import SuspensionParams
        s_hp = s_hardpoints(SuspensionParams())
        names = ("lower_pickup_fore", "lower_pickup_aft",
                 "upper_pickup_fore", "upper_pickup_aft", "toe_pickup")
        half_track = p.track_mm / 2.0
        r = p.tyre_radius_mm
        for side in ("l", "r"):
            side_sign = -1.0 if side == "r" else 1.0
            sub_hp = p.hardpoints_local(side)
            for nm in names:
                if nm not in s_hp:
                    continue
                sx, sy, sz = s_hp[nm]
                rx, ry = (sx, sy) if side_sign > 0 else (-sx, -sy)   # Rz(side)
                mapped = (rx, side_sign * half_track + ry, r + sz)
                d = _dist(mapped, sub_hp[nm])
                if d > 5.0:     # 5 mm tolerance: the boss must land on the suspension pickup
                    out.append(
                        "pickup boss %s_%s is %.0f mm from the suspension inboard hardpoint "
                        "(subframe %s vs suspension-mapped %s) -- the boss would not catch "
                        "the arm" % (nm, side, d, tuple(round(v, 0) for v in sub_hp[nm]),
                                     tuple(round(v, 0) for v in mapped)))
    except Exception:
        pass    # suspension package absent -- the assembly enforces the coincidence
    return out


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def report(p: SubframeParams) -> str:
    g = derive(p)
    c, pad, boss, tw, ea, mat = p.cradle, p.pad, p.boss, p.tower, p.eaxle, p.material
    issues = validate(p)
    hp = p.hardpoints_local("l")
    lines = [
        "Subframe (cradle) design summary -- %s [%s axle]" % (p.name, p.axle),
        "  perimeter box beam       : %.0fx%.0f mm, %.0f mm wall  (side rail %.0f mm)" % (
            c.beam_width_mm, c.beam_height_mm, c.beam_wall_mm, c.side_rail_length_mm),
        "  beam section             : A %.0f mm^2, Iz %.3g mm^4 (vert)" % (
            g.beam_area_mm2, g.beam_Iz_mm4),
        "  cradle load path         : corner load %.0f N over a %.0f mm pad-to-pad span" % (
            g.corner_load_n, g.side_rail_span_mm),
        "  side-rail bending        : sigma %.1f MPa, SF %.2f, deflection %.3f mm" % (
            g.side_rail_bending_stress_mpa, g.side_rail_safety_factor, g.side_rail_deflection_mm),
        "  cradle vertical stiffness: ~%.0f N/mm  (ROUGH single-beam estimate -- verify by FEA)" % (
            g.cradle_vertical_stiffness_n_per_mm),
        "  chassis pads             : 4 posts to y=±%.0f, z=%.0f (rail top); %d bolts Ø%.0f, SF %.2f" % (
            pad.pad_y_mm, pad.pad_z_mm, pad.bolt_count, pad.bolt_diameter_mm, g.pad_bolt_safety_factor),
        "  suspension pickups       : %d bosses/side Ø%.0f, bore Ø%.0f, bolt SF %.2f" % (
            g.n_pickups, boss.boss_diameter_mm, boss.bore_diameter_mm, g.pickup_bolt_safety_factor),
        "    lower fore/aft (local) : %s / %s" % (
            tuple(round(v) for v in hp["lower_pickup_fore"]),
            tuple(round(v) for v in hp["lower_pickup_aft"])),
        "    upper fore/aft (local) : %s / %s" % (
            tuple(round(v) for v in hp["upper_pickup_fore"]),
            tuple(round(v) for v in hp["upper_pickup_aft"])),
        "    toe (local)            : %s" % (tuple(round(v) for v in hp["toe_pickup"]),),
        "  shock tower              : reaches z=%.0f (damper top %.0f), seat Ø%.0f, %d bolts Ø%.0f" % (
            g.tower_reach_z_mm, g.damper_top_z_mm, tw.seat_diameter_mm, tw.bolt_count, tw.bolt_diameter_mm),
        "  e-axle / diff mounts     : %s" % (
            "%d mounts at y=±%.0f, z=%.0f, Ø%.0f bolt Ø%.0f" % (
                ea.mount_count, ea.mount_y_mm, ea.mount_z_mm, ea.boss_diameter_mm, ea.bolt_diameter_mm)
            if ea.enabled else "none"),
        "  material                 : %s (%.0f kg/m^3), %s" % (
            mat.cradle_material, mat.density_kg_m3, mat.joining),
        "  mass estimate (structure): %.1f kg  (cradle %.1f + pads %.1f + towers %.1f + bosses %.1f)" % (
            g.total_mass_kg, g.cradle_mass_kg, g.pad_post_mass_kg, g.tower_mass_kg, g.boss_mass_kg),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
