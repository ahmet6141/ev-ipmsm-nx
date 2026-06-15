"""Derived geometry, winding figures of merit and design-rule validation.

Pure functions over :class:`motor_nx.params.MotorParams`. No NX dependency.

The *derived* dimensions computed here (radii, slot widths, pole pitch, ...)
are the single source of truth consumed by :mod:`motor_nx.blueprint`. Keeping
them in one place means the parametric relationships (e.g. "slot width follows
from the target tooth width") live in exactly one location.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .params import MotorParams

TAU = 2.0 * math.pi

# minimum steel web (mm) between a magnet pocket corner and the shaft bore
ROTOR_INNER_WEB_MIN = 2.0
# angular clearance (deg) kept between a magnet pocket and the q-axis (pole edge)
INTER_POLE_MARGIN_DEG = 1.0
# minimum metal wall (mm) between a cooling channel and a jacket face / neighbour
COOLANT_WALL_MIN = 0.8
# minimum surviving d-axis centre rib (mm) between the two V-pocket arms
ROTOR_CENTER_RIB_MIN = 0.5


@dataclass
class DerivedGeometry:
    # stator
    stator_outer_radius: float
    bore_radius: float
    slot_pitch_deg: float
    slot_body_inner_radius: float   # r1: top of the slot mouth / bottom of slot body
    slot_body_outer_radius: float   # r2: slot bottom (back-iron starts here)
    slot_depth: float
    slot_width: float               # tangential slot-body width (parallel walls, hairpin)
    tooth_width_at_r1: float
    # rotor
    rotor_outer_radius: float
    shaft_radius: float
    pole_pitch_deg: float
    # winding
    slots_per_pole_per_phase: float  # q
    coil_pitch_slots: int
    distribution_factor: float       # kd
    pitch_factor: float              # kp
    winding_factor: float            # kw = kd * kp
    electrical_slot_angle_deg: float
    # ratios / sanity figures
    split_ratio: float               # bore / OD
    aspect_ratio: float              # stack_length / bore_diameter


def derive(p: MotorParams) -> DerivedGeometry:
    s, r, w = p.stator, p.rotor, p.winding

    stator_outer_radius = s.outer_diameter / 2.0
    bore_radius = s.bore_diameter / 2.0
    slot_pitch_deg = 360.0 / s.slot_count

    r1 = bore_radius + s.slot_opening_depth
    r2 = stator_outer_radius - s.back_iron_thickness
    slot_depth = r2 - r1

    # Hairpin slots have parallel walls -> tooth metal grows with radius, so the
    # narrowest tooth sits at r1. Fix the slot width from the target tooth width.
    tooth_pitch_at_r1 = TAU * r1 / s.slot_count
    slot_width = tooth_pitch_at_r1 - s.tooth_width
    tooth_width_at_r1 = tooth_pitch_at_r1 - slot_width  # == s.tooth_width by construction

    rotor_outer_radius = bore_radius - r.air_gap
    shaft_radius = p.shaft.diameter / 2.0
    pole_pitch_deg = 360.0 / r.pole_count

    # winding factors (integer-slot distributed winding)
    q = s.slot_count / (r.pole_count * w.phases)
    pole_pairs = r.pole_count / 2.0
    elec_slot_angle_deg = pole_pairs * slot_pitch_deg            # electrical angle between slots
    gamma = math.radians(elec_slot_angle_deg)
    n = max(1, int(round(q)))
    if abs(math.sin(gamma / 2.0)) < 1e-12:
        kd = 1.0
    else:
        kd = math.sin(n * gamma / 2.0) / (n * math.sin(gamma / 2.0))
    full_pitch_slots = s.slot_count // r.pole_count
    # coil_span_slots == 0 => full pitch; a shorter span chords the winding (kp<1).
    span = getattr(w, "coil_span_slots", 0) or full_pitch_slots
    coil_pitch_slots = max(1, min(int(span), full_pitch_slots))
    kp = math.sin(math.radians(coil_pitch_slots / full_pitch_slots * 90.0))
    kw = abs(kd * kp)

    return DerivedGeometry(
        stator_outer_radius=stator_outer_radius,
        bore_radius=bore_radius,
        slot_pitch_deg=slot_pitch_deg,
        slot_body_inner_radius=r1,
        slot_body_outer_radius=r2,
        slot_depth=slot_depth,
        slot_width=slot_width,
        tooth_width_at_r1=tooth_width_at_r1,
        rotor_outer_radius=rotor_outer_radius,
        shaft_radius=shaft_radius,
        pole_pitch_deg=pole_pitch_deg,
        slots_per_pole_per_phase=q,
        coil_pitch_slots=coil_pitch_slots,
        distribution_factor=kd,
        pitch_factor=kp,
        winding_factor=kw,
        electrical_slot_angle_deg=elec_slot_angle_deg,
        split_ratio=s.bore_diameter / s.outer_diameter,
        aspect_ratio=p.stack_length / s.bore_diameter,
    )


def validate(p: MotorParams) -> List[str]:
    """Return a list of human-readable problems. Empty list => buildable.

    These are *geometric/topological* feasibility checks -- they stop the NX
    builder from attempting an impossible cut (negative thickness, magnets that
    poke through the rotor surface, slots deeper than the available iron, ...).
    """
    issues: List[str] = []
    g = derive(p)
    s, r, w = p.stator, p.rotor, p.winding

    # --- stator ---------------------------------------------------------- #
    if g.bore_radius >= g.stator_outer_radius:
        issues.append("bore_diameter must be smaller than outer_diameter.")
    if g.slot_depth <= 0:
        issues.append(
            f"Slot depth is {g.slot_depth:.2f} mm (<=0): back_iron_thickness "
            f"({s.back_iron_thickness}) leaves no room below the slot mouth."
        )
    if g.slot_width <= 0:
        issues.append(
            f"Slot width is {g.slot_width:.2f} mm (<=0): tooth_width "
            f"({s.tooth_width}) is too wide for {s.slot_count} slots at r1={g.slot_body_inner_radius:.1f}."
        )
    if s.slot_opening_width > g.slot_width:
        issues.append("slot_opening_width is wider than the slot body width.")
    if s.slot_count % (r.pole_count * w.phases) != 0:
        issues.append(
            f"slot_count ({s.slot_count}) is not an integer multiple of "
            f"pole_count*phases ({r.pole_count * w.phases}); fractional-slot winding "
            f"is out of scope for this hairpin generator (q={g.slots_per_pole_per_phase:.3f})."
        )

    # --- air gap / rotor ------------------------------------------------- #
    if r.air_gap <= 0:
        issues.append("air_gap must be > 0.")
    if g.rotor_outer_radius <= g.shaft_radius:
        issues.append("rotor outer radius is not larger than the shaft radius.")
    if r.pole_count % 2 != 0:
        issues.append("pole_count must be even.")
    if r.magnets_per_pole != 2:
        issues.append("this generator models a single V (magnets_per_pole == 2).")

    if r.pocket_clearance < 0 or r.end_barrier < 0:
        issues.append("pocket_clearance and end_barrier must be >= 0 (a negative "
                      "value makes the pocket smaller than the magnet it must hold).")

    # --- V-magnet pocket fits inside the rotor pole ---------------------- #
    pocket = _magnet_pocket_extent(p, g)
    if pocket is not None:
        max_radius, min_radius, max_half_angle_deg, rib_half = pocket
        if max_radius > g.rotor_outer_radius - r.outer_bridge + 1e-6:
            issues.append(
                f"V-magnet outer corner reaches r={max_radius:.2f} mm but the rotor "
                f"surface minus outer_bridge is {g.rotor_outer_radius - r.outer_bridge:.2f} mm. "
                f"Reduce magnet_width/v_angle_deg or outer_bridge."
            )
        if min_radius < g.shaft_radius + ROTOR_INNER_WEB_MIN - 1e-6:
            issues.append(
                f"V-magnet inner corner reaches r={min_radius:.2f} mm, leaving less than the "
                f"{ROTOR_INNER_WEB_MIN:.1f} mm minimum web above the shaft (r={g.shaft_radius:.2f}). "
                f"Reduce magnet_width or increase vertex_gap."
            )
        if max_half_angle_deg > g.pole_pitch_deg / 2.0 - INTER_POLE_MARGIN_DEG:
            issues.append(
                f"V-magnet spans +/-{max_half_angle_deg:.1f} deg but the pole half-pitch (minus a "
                f"{INTER_POLE_MARGIN_DEG:.0f} deg inter-pole bridge) is {g.pole_pitch_deg / 2.0 - INTER_POLE_MARGIN_DEG:.1f} deg; "
                f"adjacent poles would collide. Reduce magnet_width or vertex_gap, or widen the V angle."
            )
        # the rib that actually survives after pocket_clearance erosion (not the
        # input center_post_halfwidth) -- rib_half<=0 means the two pockets cross
        # the d-axis and the NX subtract self-intersects.
        if rib_half < ROTOR_CENTER_RIB_MIN - 1e-6:
            issues.append(
                f"d-axis centre rib is {2 * rib_half:.2f} mm after pocket_clearance erosion "
                f"(min {2 * ROTOR_CENTER_RIB_MIN:.1f} mm; <=0 means the pockets cross y=0 and the "
                f"cut self-intersects). Increase center_post_halfwidth or reduce pocket_clearance/magnet_thickness."
            )
    if r.center_post_halfwidth <= 0:
        issues.append("center_post_halfwidth must be > 0 (the d-axis rib).")
    if r.outer_bridge <= 0:
        issues.append("outer_bridge must be > 0.")
    # magnet solid (sharp rect) must stay inside its rounded pocket -- a large
    # magnet_pocket_fillet relative to end_barrier/pocket_clearance can leave a
    # magnet corner outside the cut, interfering with un-removed steel.
    if _magnet_outside_pocket(p, g):
        issues.append(
            "a magnet corner falls outside its rounded pocket: magnet_pocket_fillet is too large "
            "for end_barrier/pocket_clearance. Reduce magnet_pocket_fillet or increase the clearances."
        )

    # --- shaft -------------------------------------------------------------- #
    if p.shaft.bore_diameter >= min(p.shaft.diameter, p.shaft.bearing_seat_diameter):
        issues.append(
            f"shaft bore_diameter ({p.shaft.bore_diameter}) must be smaller than the shaft "
            f"journal and bearing-seat diameters (min {min(p.shaft.diameter, p.shaft.bearing_seat_diameter)}); "
            f"otherwise the revolved shaft profile self-intersects."
        )

    # --- hairpin conductors fit in the slot ------------------------------ #
    if w.conductors_per_slot < 1:
        issues.append("conductors_per_slot must be >= 1.")
    if w.parallel_paths < 1:
        issues.append("parallel_paths must be >= 1.")
    if w.phases < 1:
        issues.append("phases must be >= 1.")
    # Mirror conductor_polygons() EXACTLY: n bars + (n+1) clearances stack
    # radially AND the tangential bar width must be positive. (A looser check
    # here let validate() pass while the builder silently emitted zero bars.)
    n_cond = max(1, w.conductors_per_slot)
    bar_h = (g.slot_depth - w.bar_clearance * (n_cond + 1)) / n_cond
    if bar_h <= 0:
        issues.append(
            f"conductors_per_slot ({w.conductors_per_slot}) do not fit radially in the "
            f"{g.slot_depth:.2f} mm slot depth."
        )
    bar_w = g.slot_width - 2.0 * w.bar_clearance
    if bar_w <= 0:
        issues.append(
            f"hairpin bar width is {bar_w:.2f} mm (<=0): slot_width {g.slot_width:.2f} mm minus "
            f"2x bar_clearance ({w.bar_clearance}) leaves no copper -- the builder emits zero bars. "
            f"Widen the slot (reduce tooth_width) or reduce bar_clearance."
        )

    # --- cooling jacket / axial channels --------------------------------- #
    c = p.cooling
    if c.jacket_thickness <= 0:
        issues.append(
            f"cooling.jacket_thickness ({c.jacket_thickness}) must be > 0; a zero/negative "
            f"jacket makes an inverted (outer<=inner) housing tube that NX cannot build."
        )
    if c.channel_type == "axial" and c.channel_count > 0:
        jacket_inner = g.stator_outer_radius + c.housing_gap
        jacket_outer = jacket_inner + c.jacket_thickness
        pitch_r = 0.5 * (jacket_inner + jacket_outer)
        chan_r = c.channel_diameter / 2.0
        radial_wall = c.jacket_thickness / 2.0 - chan_r
        if radial_wall < COOLANT_WALL_MIN - 1e-6:
            issues.append(
                f"cooling channel diameter {c.channel_diameter:.1f} mm leaves only "
                f"{radial_wall:.2f} mm wall in the {c.jacket_thickness:.1f} mm jacket "
                f"(min {COOLANT_WALL_MIN:.1f} mm). A channel tangent to a jacket face is a "
                f"zero-wall cut; reduce channel_diameter or thicken the jacket."
            )
        chan_pitch_arc = TAU * pitch_r / c.channel_count
        if c.channel_diameter + COOLANT_WALL_MIN > chan_pitch_arc + 1e-6:
            issues.append(
                f"{c.channel_count} channels of {c.channel_diameter:.1f} mm overlap on the "
                f"{TAU * pitch_r:.0f} mm jacket circumference (pitch {chan_pitch_arc:.1f} mm). "
                f"Reduce channel_count or channel_diameter."
            )
    if c.channel_type not in ("axial", "none"):
        issues.append(
            f"cooling.channel_type '{c.channel_type}' is not implemented -- only 'axial' "
            f"channels are generated (a '{c.channel_type}' setting silently makes none). "
            f"Use 'axial' or 'none'."
        )

    # --- magnet axial segmentation --------------------------------------- #
    n_seg = max(1, int(p.material.magnet_segments_axial))
    if n_seg > 1:
        seg_len = (p.stack_length - p.material.magnet_seg_gap_mm * (n_seg - 1)) / n_seg
        if seg_len < 2.0:
            issues.append(
                f"{n_seg} axial magnet segments leave only {seg_len:.2f} mm each "
                f"(< 2 mm min); reduce magnet_segments_axial or magnet_seg_gap_mm."
            )

    return issues


def _magnet_pocket_extent(p: MotorParams, g: DerivedGeometry):
    """Min/max radius and max half-angle reached by one V-pair's magnet pockets,
    measured in the reference-pole frame. Returns None if geometry is degenerate.
    Mirrors the construction in blueprint.magnet_pocket_polygons().
    """
    from .blueprint import magnet_pocket_polygons  # local import avoids cycle

    polys = magnet_pocket_polygons(p, g)
    if not polys:
        return None
    radii = [math.hypot(x, y) for poly in polys for (x, y) in poly]
    angles = [abs(math.degrees(math.atan2(y, x))) for poly in polys for (x, y) in poly]
    # the +Y pocket's closest approach to the d-axis (y=0); <=0 means the two
    # mirrored pockets overlap across the d-axis -> negative centre rib.
    rib_half = min(y for (x, y) in polys[0])
    return max(radii), min(radii), max(angles), rib_half


def _point_in_polygon(pt, poly) -> bool:
    """Ray-casting point-in-polygon (poly = list of (x, y))."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def _magnet_outside_pocket(p: MotorParams, g: DerivedGeometry) -> bool:
    """True if any magnet-solid corner lies outside its (rounded) pocket cut."""
    from .blueprint import magnet_polygons, magnet_pocket_polygons
    try:
        mags = magnet_polygons(p, g)
        pkts = magnet_pocket_polygons(p, g)
    except Exception:
        return False
    for mag, pkt in zip(mags, pkts):
        for corner in mag:
            if not _point_in_polygon(corner, pkt):
                return True
    return False


def report(p: MotorParams) -> str:
    """One-screen human summary -- printed by the CLI and logged by the builder."""
    g = derive(p)
    lines = [
        f"IPMSM design summary -- {p.name}",
        f"  stack length            : {p.stack_length:.1f} mm",
        f"  stator OD / bore         : {p.stator.outer_diameter:.1f} / {p.stator.bore_diameter:.1f} mm"
        f"  (split ratio {g.split_ratio:.3f})",
        f"  air gap                  : {p.rotor.air_gap:.2f} mm",
        f"  rotor OD                 : {2 * g.rotor_outer_radius:.2f} mm",
        f"  slots / poles            : {p.stator.slot_count} / {p.rotor.pole_count}"
        f"  (q = {g.slots_per_pole_per_phase:.2f})",
        f"  slot width / depth       : {g.slot_width:.2f} / {g.slot_depth:.2f} mm",
        f"  tooth width (min, @r1)   : {g.tooth_width_at_r1:.2f} mm",
        f"  winding factor kw        : {g.winding_factor:.4f}"
        f"  (kd={g.distribution_factor:.4f}, kp={g.pitch_factor:.4f})",
        f"  hairpin bars / slot      : {p.winding.conductors_per_slot}",
        f"  V-magnet  W x t          : {p.rotor.magnet_width:.1f} x {p.rotor.magnet_thickness:.1f} mm"
        f"  @ V-angle {p.rotor.v_angle_deg:.0f} deg",
        f"  aspect ratio L/D_bore    : {g.aspect_ratio:.2f}",
        f"  magnet                   : {p.material.magnet_grade} "
        f"(Br {p.material.magnet_br_t:.2f} T, <= {p.material.magnet_max_service_c:.0f} C, "
        f"{p.material.magnet_segments_axial} axial seg)",
        f"  laminations              : {p.material.electrical_steel}, "
        f"stacking {p.material.stacking_factor:.2f}",
        f"  conductor insulation     : {p.material.copper_insulation_class}",
    ]
    problems = validate(p)
    if problems:
        lines.append("  VALIDATION ISSUES:")
        lines.extend(f"    - {msg}" for msg in problems)
    else:
        lines.append("  validation: OK (geometry is buildable)")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# First-order performance estimate  (analytical sizing -- NOT a substitute for FEA)
# --------------------------------------------------------------------------- #
@dataclass
class EMAssumptions:
    """Operating-point assumptions for first-order sizing of a liquid-cooled
    hairpin EV-traction IPMSM. These are INPUTS (typical values), not results;
    an FEA / Motor-CAD pass refines airgap flux, saturation, losses and saliency."""
    j_cont_a_mm2: float = 12.0       # continuous rms current density in slot copper
    j_peak_a_mm2: float = 26.0       # short-term peak rms current density
    slot_fill: float = 0.60          # NET copper fill of the slot body (matches the modeled
    #                                  8-bar geometry ~0.595; hairpin gross ~0.7, net ~0.6)
    b_g1_peak_t: float = 0.85        # fundamental airgap flux density from the PMs (peak)
    saliency_factor: float = 1.25    # IPM reluctance-torque bonus over PM-only (1.2-1.4)
    dc_bus_v: float = 400.0          # inverter DC-link voltage (400 V class; 800 V via turns)
    modulation_index: float = 1.0    # 1.0 = pure SVPWM linear ceiling; up to ~1.15 toward six-step
    max_speed_rpm: float = 18000.0   # mechanical max-speed design target
    copper_resistivity: float = 2.1e-8   # Ohm*m, copper at ~100 C
    end_turn_factor: float = 1.5     # total end-winding extension as x(pole pitch)


@dataclass
class PerformanceEstimate:
    series_turns_per_phase: float
    slot_copper_area_mm2: float
    conductor_area_mm2: float
    i_phase_cont_a: float
    i_phase_peak_a: float
    electric_loading_cont_ka_m: float
    electric_loading_peak_ka_m: float
    shear_cont_kpa: float
    shear_peak_kpa: float
    torque_cont_nm: float
    torque_peak_nm: float
    ke_ll_v_per_krpm: float
    base_speed_rpm: float
    power_cont_kw: float
    power_peak_kw: float
    copper_loss_cont_w: float
    efficiency_cont_pct: float


def estimate_performance(p: MotorParams, a: "EMAssumptions" = None) -> PerformanceEstimate:
    """First-order analytical performance from geometry + EM loading assumptions.

    Torque via airgap shear stress:  T = (pi/2) * D_bore^2 * L * sigma, with
    sigma = A_rms * Bg1_rms * kw (q-axis, PM-aligned), times an IPM reluctance
    factor for the saliency torque. Good to ~+/-20-30 %; verify with FEA.
    """
    a = a or EMAssumptions()
    g = derive(p)
    s, r, w = p.stator, p.rotor, p.winding
    m = float(w.phases)

    total_cond = s.slot_count * w.conductors_per_slot
    series_turns = total_cond / (2.0 * m * w.parallel_paths)

    slot_area = g.slot_width * g.slot_depth                 # mm^2, hairpin slot body
    copper_area = slot_area * a.slot_fill                   # mm^2 copper per slot
    cond_area = copper_area / w.conductors_per_slot         # mm^2 per bar

    i_cond_cont = a.j_cont_a_mm2 * cond_area                # A rms per bar
    i_cond_peak = a.j_peak_a_mm2 * cond_area
    i_ph_cont = i_cond_cont * w.parallel_paths
    i_ph_peak = i_cond_peak * w.parallel_paths

    D = s.bore_diameter / 1000.0                            # m
    L = p.stack_length / 1000.0
    bore_circ = math.pi * D
    A_cont = s.slot_count * w.conductors_per_slot * i_cond_cont / bore_circ   # A/m
    A_peak = s.slot_count * w.conductors_per_slot * i_cond_peak / bore_circ

    bg1_rms = a.b_g1_peak_t / math.sqrt(2.0)
    sigma_cont = A_cont * bg1_rms * g.winding_factor        # Pa
    sigma_peak = A_peak * bg1_rms * g.winding_factor

    k_rv = (math.pi / 2.0) * D * D * L                      # T = k_rv * sigma
    torque_cont = k_rv * sigma_cont * a.saliency_factor
    torque_peak = k_rv * sigma_peak * a.saliency_factor

    pole_area = math.pi * D * L / r.pole_count
    flux_pole = (2.0 / math.pi) * a.b_g1_peak_t * pole_area
    psi_pm = series_turns * g.winding_factor * flux_pole
    omega_e_1k = (r.pole_count / 2.0) * (2.0 * math.pi * 1000.0 / 60.0)
    e_ph_rms_1k = psi_pm * omega_e_1k / math.sqrt(2.0)
    ke_ll_1k = e_ph_rms_1k * math.sqrt(3.0)

    # Max phase RMS voltage the inverter can synthesise. SVPWM linear-modulation
    # ceiling is Vdc/sqrt(6) (== Vdc/sqrt(2) line-line RMS); modulation_index up
    # to ~1.15 reaches toward six-step. (Vdc/sqrt(3) would demand the FULL DC bus
    # line-to-line, which no PWM inverter can produce.)
    v_ph_max = a.modulation_index * a.dc_bus_v / math.sqrt(6.0)
    base_speed = v_ph_max / e_ph_rms_1k * 1000.0 if e_ph_rms_1k > 1e-9 else 0.0
    omega_base = 2.0 * math.pi * base_speed / 60.0
    power_cont = torque_cont * omega_base
    power_peak = torque_peak * omega_base

    pole_pitch = math.pi * D / r.pole_count
    l_cu = L + a.end_turn_factor * pole_pitch
    v_cu = copper_area * 1e-6 * s.slot_count * l_cu
    p_cu = a.copper_resistivity * (a.j_cont_a_mm2 * 1e6) ** 2 * v_cu
    eff_cont = power_cont / (power_cont + p_cu) * 100.0     # Cu-loss only (optimistic)

    return PerformanceEstimate(
        series_turns_per_phase=series_turns,
        slot_copper_area_mm2=copper_area,
        conductor_area_mm2=cond_area,
        i_phase_cont_a=i_ph_cont,
        i_phase_peak_a=i_ph_peak,
        electric_loading_cont_ka_m=A_cont / 1000.0,
        electric_loading_peak_ka_m=A_peak / 1000.0,
        shear_cont_kpa=sigma_cont / 1000.0,
        shear_peak_kpa=sigma_peak / 1000.0,
        torque_cont_nm=torque_cont,
        torque_peak_nm=torque_peak,
        ke_ll_v_per_krpm=ke_ll_1k,
        base_speed_rpm=base_speed,
        power_cont_kw=power_cont / 1000.0,
        power_peak_kw=power_peak / 1000.0,
        copper_loss_cont_w=p_cu,
        efficiency_cont_pct=eff_cont,
    )


def performance_report(p: MotorParams, a: "EMAssumptions" = None) -> str:
    """One-screen analytical performance summary (printed by the CLI / builder)."""
    a = a or EMAssumptions()
    e = estimate_performance(p, a)
    return "\n".join([
        f"Performance estimate (FIRST-ORDER analytical -- verify with FEA) -- {p.name}",
        f"  assumptions   : J {a.j_cont_a_mm2:.0f}/{a.j_peak_a_mm2:.0f} A/mm^2 cont/peak,"
        f" fill {a.slot_fill:.2f}, Bg1 {a.b_g1_peak_t:.2f} T, saliency x{a.saliency_factor:.2f},"
        f" Vdc {a.dc_bus_v:.0f} V",
        f"  series turns/phase       : {e.series_turns_per_phase:.0f}",
        f"  phase current cont/peak  : {e.i_phase_cont_a:.0f} / {e.i_phase_peak_a:.0f} A rms",
        f"  electric loading         : {e.electric_loading_cont_ka_m:.0f} / {e.electric_loading_peak_ka_m:.0f} kA/m",
        f"  airgap shear stress      : {e.shear_cont_kpa:.1f} / {e.shear_peak_kpa:.1f} kPa",
        f"  TORQUE   cont / peak     : {e.torque_cont_nm:.0f} / {e.torque_peak_nm:.0f} Nm",
        f"  POWER    cont / peak     : {e.power_cont_kw:.0f} / {e.power_peak_kw:.0f} kW",
        f"  base / max speed         : {e.base_speed_rpm:.0f} / {a.max_speed_rpm:.0f} rpm",
        f"  back-EMF const (LL rms)  : {e.ke_ll_v_per_krpm:.1f} V / 1000 rpm",
        f"  copper loss @ cont       : {e.copper_loss_cont_w / 1000.0:.2f} kW (eff ~{e.efficiency_cont_pct:.1f}% Cu-only)",
    ])
