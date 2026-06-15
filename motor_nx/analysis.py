"""First-order engineering analyses that the geometry + performance model did not
yet cover: a multi-component LOSS breakdown, a THERMAL continuous-rating (losses
tied to a winding temperature limit), a DEMAGNETISATION margin, a rotor
CENTRIFUGAL bridge-stress check, and a COGGING/NVH index.

NX-independent, unit-tested. Every number here is a transparent first-order
estimate with its coefficients exposed in :class:`AnalysisAssumptions`; they are
NOT a substitute for FEA (cogging/torque-ripple, true iron+magnet loss, a real
lumped-parameter thermal network and rotor stress FEA) -- see docs/FEA_PREP.md.
The point is a defensible "is this design in the right ballpark, and what limits
it" answer before committing FEA time.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict

from . import em_design
from . import manufacturing as _mfg
from .params import MotorParams

MU0 = 4e-7 * math.pi


@dataclass
class AnalysisAssumptions:
    """Coefficients for the first-order loss / thermal / stress / demag models."""
    # iron loss: p = (k_h*(f/50) + k_e*(f/50)^2) * (B/1.5)^2  [W/kg], calibrated so
    # k_h + k_e = the datasheet 2.3 W/kg @ 1.5 T / 50 Hz (hysteresis-dominant at 50 Hz)
    iron_loss_15t_50hz: float = 2.3
    iron_hyst_fraction: float = 0.6
    stator_b_eff_t: float = 1.5        # representative stator flux density
    rotor_iron_loss_fraction: float = 0.1   # rotor iron sees mostly DC + ripple
    # magnet eddy: p = ref * (f_e/ref_f)^2 / n_seg^2  [W/kg] (order-of-magnitude; PWM-driven,
    # the crudest term -- FEA-replaceable)
    magnet_eddy_wpkg_at_ref: float = 25.0
    magnet_eddy_ref_f_hz: float = 900.0
    # thermal (water-jacket)
    coolant_inlet_c: float = 65.0
    h_conv_w_m2k: float = 3000.0       # effective jacket convection incl. wall (water-glycol)
    slot_liner_thickness_mm: float = 0.3
    slot_liner_k_w_mk: float = 0.25    # Nomex/impregnated liner conductivity
    slot_eff_k_w_mk: float = 3.0       # effective slot-to-iron conductivity for distributed
    #                                    Joule heat (impregnated copper + tooth conduction);
    #                                    DOMINANT internal resistance -- main calibration knob
    rotor_to_coolant_kpw: float = 2.5  # magnet->airgap->stator thermal resistance (poor path)
    winding_temp_margin_c: float = 15.0
    # structural
    overspeed_factor: float = 1.2
    rotor_steel_yield_mpa: float = 450.0   # high-strength electrical steel
    target_structural_sf: float = 1.5
    rotor_steel_density: float = 7650.0
    # demag (temperature coefficients, %/C)
    br_tempco_pct_c: float = -0.12
    hcj_tempco_pct_c: float = -0.55


def _class_temp_limit_c(insulation_class: str) -> float:
    """Parse the continuous temperature limit (deg C) out of a class string like
    'H (180 C) hairpin enamel'; default to class H 180 C."""
    m = re.search(r"(\d{2,3})\s*C", insulation_class or "")
    return float(m.group(1)) if m else 180.0


def _electrical_freq_hz(p: MotorParams, rpm: float) -> float:
    return (p.rotor.pole_count / 2.0) * rpm / 60.0


# --------------------------------------------------------------------------- #
# loss breakdown
# --------------------------------------------------------------------------- #
@dataclass
class LossBreakdown:
    speed_rpm: float
    elec_freq_hz: float
    current_density_a_mm2: float
    p_cu_dc_w: float
    ac_resistance_factor: float
    p_cu_ac_w: float            # the AC ADDER (proximity/skin), above DC
    p_iron_w: float
    p_magnet_w: float
    p_total_w: float


def _copper_volume_m3(p: MotorParams, g, perf, a_em) -> float:
    pole_pitch = math.pi * (p.stator.bore_diameter / 1000.0) / p.rotor.pole_count
    l_cu = p.stack_length / 1000.0 + a_em.end_turn_factor * pole_pitch
    return perf.slot_copper_area_mm2 * 1e-6 * p.stator.slot_count * l_cu


def _ac_resistance_factor(p: MotorParams, g, f_hz: float, rho: float) -> float:
    """Dowell low-frequency proximity factor for m radial conductor layers."""
    if f_hz <= 0:
        return 1.0
    n = max(1, p.winding.conductors_per_slot)
    bar_h_mm = (g.slot_depth - p.winding.bar_clearance * (n + 1)) / n
    if bar_h_mm <= 0:
        return 1.0
    skin_depth_mm = math.sqrt(rho / (math.pi * f_hz * MU0)) * 1000.0
    ratio = bar_h_mm / skin_depth_mm
    return 1.0 + (5.0 * n * n - 1.0) / 45.0 * ratio ** 4


def loss_breakdown(p: MotorParams, rpm: float = None, j_a_mm2: float = None,
                   a_em: "em_design.EMAssumptions" = None,
                   a: AnalysisAssumptions = None) -> LossBreakdown:
    a_em = a_em or em_design.EMAssumptions()
    a = a or AnalysisAssumptions()
    g = em_design.derive(p)
    perf = em_design.estimate_performance(p, a_em)
    rpm = perf.base_speed_rpm if rpm is None else rpm
    j = a_em.j_cont_a_mm2 if j_a_mm2 is None else j_a_mm2
    f_e = _electrical_freq_hz(p, rpm)

    v_cu = _copper_volume_m3(p, g, perf, a_em)
    p_cu_dc = a_em.copper_resistivity * (j * 1e6) ** 2 * v_cu
    f_r = _ac_resistance_factor(p, g, f_e, a_em.copper_resistivity)
    p_cu_ac = p_cu_dc * (f_r - 1.0)

    bom = _mfg.bill_of_materials(p)
    m_stator = next((i["mass_kg"] for i in bom["line_items"] if i["component"].startswith("Stator lam")), 0.0)
    m_rotor = next((i["mass_kg"] for i in bom["line_items"] if i["component"].startswith("Rotor lam")), 0.0)
    m_magnet = bom["magnet_mass_kg"]
    k_h = a.iron_loss_15t_50hz * a.iron_hyst_fraction
    k_e = a.iron_loss_15t_50hz * (1.0 - a.iron_hyst_fraction)
    fr_ratio = f_e / 50.0
    p_iron_density = (k_h * fr_ratio + k_e * fr_ratio ** 2) * (a.stator_b_eff_t / 1.5) ** 2
    p_iron = p_iron_density * (m_stator + a.rotor_iron_loss_fraction * m_rotor)

    n_seg = max(1, int(p.material.magnet_segments_axial))
    p_magnet = a.magnet_eddy_wpkg_at_ref * (f_e / a.magnet_eddy_ref_f_hz) ** 2 * m_magnet / (n_seg ** 2)

    return LossBreakdown(
        speed_rpm=rpm, elec_freq_hz=f_e, current_density_a_mm2=j,
        p_cu_dc_w=p_cu_dc, ac_resistance_factor=f_r, p_cu_ac_w=p_cu_ac,
        p_iron_w=p_iron, p_magnet_w=p_magnet,
        p_total_w=p_cu_dc + p_cu_ac + p_iron + p_magnet,
    )


# --------------------------------------------------------------------------- #
# thermal continuous rating (losses tied to a winding temperature limit)
# --------------------------------------------------------------------------- #
@dataclass
class ThermalRating:
    r_int_winding_to_iron_kpw: float
    r_ext_iron_to_coolant_kpw: float
    winding_temp_limit_c: float
    allowable_rise_c: float
    iron_loss_at_base_w: float
    j_cont_thermal_a_mm2: float
    j_cont_assumed_a_mm2: float
    torque_cont_thermal_nm: float
    magnet_temp_rise_c: float
    thermally_limited: bool


def thermal_rating(p: MotorParams, a_em: "em_design.EMAssumptions" = None,
                   a: AnalysisAssumptions = None) -> ThermalRating:
    """Continuous current density bounded by the winding hotspot. Inverts the
    loss<->temperature relation: instead of ASSUMING j_cont, find the j that holds
    the winding at its insulation limit through a 2-resistance lumped path
    (winding -> slot liner+iron -> jacket -> coolant)."""
    a_em = a_em or em_design.EMAssumptions()
    a = a or AnalysisAssumptions()
    g = em_design.derive(p)
    perf = em_design.estimate_performance(p, a_em)

    # external resistance: jacket convection over the stator OD contact area
    a_jacket = math.pi * (p.stator.outer_diameter / 1000.0) * (p.stack_length / 1000.0)
    r_ext = 1.0 / (a.h_conv_w_m2k * a_jacket)
    # internal resistance = slot-liner conduction + distributed-slot conduction.
    # liner: thin sheet over the slot-wall area (perimeter ~ 2*(depth+width)).
    slot_perim = 2.0 * (g.slot_depth + g.slot_width) / 1000.0
    a_slot_walls = slot_perim * p.stator.slot_count * (p.stack_length / 1000.0)
    r_liner = (a.slot_liner_thickness_mm / 1000.0) / (a.slot_liner_k_w_mk * a_slot_walls)
    # distributed Joule heat reaching the cooled back-iron: equivalent resistance of
    # a uniformly-heated slot column = depth/(3*k_eff*A_radial). This is the DOMINANT
    # internal term (the liner alone hugely under-resists -> unrealistic ratings).
    a_radial = (g.slot_width / 1000.0) * p.stator.slot_count * (p.stack_length / 1000.0)
    r_slot = (g.slot_depth / 1000.0) / (3.0 * a.slot_eff_k_w_mk * a_radial) if a_radial > 0 else 0.0
    r_int = r_liner + r_slot

    t_limit = _class_temp_limit_c(p.material.copper_insulation_class)
    allow = t_limit - a.coolant_inlet_c - a.winding_temp_margin_c

    # iron loss at base speed (independent of current); magnet loss heats the rotor
    loss_base = loss_breakdown(p, perf.base_speed_rpm, a_em.j_cont_a_mm2, a_em, a)
    p_fe = loss_base.p_iron_w
    f_r = loss_base.ac_resistance_factor

    # solve P_cu_total*(R_int+R_ext) + P_fe*R_ext = allow  for P_cu_total
    p_cu_allow = max(0.0, (allow - p_fe * r_ext) / (r_int + r_ext))
    v_cu = _copper_volume_m3(p, g, perf, a_em)
    denom = f_r * a_em.copper_resistivity * v_cu
    j_thermal = math.sqrt(p_cu_allow / denom) / 1e6 if denom > 0 else 0.0   # A/mm^2

    torque_thermal = perf.torque_cont_nm * (j_thermal / a_em.j_cont_a_mm2) if a_em.j_cont_a_mm2 else 0.0
    dt_magnet = loss_base.p_magnet_w * a.rotor_to_coolant_kpw

    return ThermalRating(
        r_int_winding_to_iron_kpw=r_int, r_ext_iron_to_coolant_kpw=r_ext,
        winding_temp_limit_c=t_limit, allowable_rise_c=allow,
        iron_loss_at_base_w=p_fe,
        j_cont_thermal_a_mm2=j_thermal, j_cont_assumed_a_mm2=a_em.j_cont_a_mm2,
        torque_cont_thermal_nm=torque_thermal, magnet_temp_rise_c=dt_magnet,
        thermally_limited=j_thermal < a_em.j_cont_a_mm2,
    )


# --------------------------------------------------------------------------- #
# demagnetisation margin
# --------------------------------------------------------------------------- #
@dataclass
class DemagMargin:
    magnet_temp_c: float
    br_hot_t: float
    hcj_hot_ka_m: float
    h_demag_peak_ka_m: float    # conservative: full peak armature MMF across the magnet
    margin_ratio: float         # Hcj_hot / H_demag (>1 safe)
    safe: bool


def demag_margin(p: MotorParams, a_em: "em_design.EMAssumptions" = None,
                 a: AnalysisAssumptions = None) -> DemagMargin:
    a_em = a_em or em_design.EMAssumptions()
    a = a or AnalysisAssumptions()
    g = em_design.derive(p)
    perf = em_design.estimate_performance(p, a_em)
    m = p.material
    t_hot = m.magnet_max_service_c

    br_hot = m.magnet_br_t * (1.0 + a.br_tempco_pct_c / 100.0 * (t_hot - 20.0))
    hcj_hot = m.magnet_hcj_ka_m * (1.0 + a.hcj_tempco_pct_c / 100.0 * (t_hot - 20.0))

    # peak fundamental armature MMF amplitude per pole (m-phase):
    #   F1 = (m/2)*(4/pi)*(kw*N_series/(2p))*sqrt(2)*I_rms
    n_series = perf.series_turns_per_phase
    i_peak_rms = perf.i_phase_peak_a
    f1_peak = (p.winding.phases / 2.0) * (4.0 / math.pi) * (
        g.winding_factor * n_series / p.rotor.pole_count) * math.sqrt(2.0) * i_peak_rms
    # conservative: assume the entire MMF drops as H across the magnet thickness
    h_demag = f1_peak / (p.rotor.magnet_thickness / 1000.0)   # A/m
    h_demag_ka = h_demag / 1000.0
    margin = hcj_hot / h_demag_ka if h_demag_ka > 0 else float("inf")
    return DemagMargin(
        magnet_temp_c=t_hot, br_hot_t=br_hot, hcj_hot_ka_m=hcj_hot,
        h_demag_peak_ka_m=h_demag_ka, margin_ratio=margin, safe=margin >= 1.5,
    )


# --------------------------------------------------------------------------- #
# rotor centrifugal bridge stress
# --------------------------------------------------------------------------- #
@dataclass
class RotorStress:
    overspeed_rpm: float
    magnet_mass_per_pole_kg: float
    cg_radius_mm: float
    bridge_area_per_pole_mm2: float
    centrifugal_force_per_pole_n: float
    bridge_stress_mpa: float
    safety_factor: float
    safe: bool


def rotor_stress(p: MotorParams, a_em: "em_design.EMAssumptions" = None,
                 a: AnalysisAssumptions = None) -> RotorStress:
    a_em = a_em or em_design.EMAssumptions()
    a = a or AnalysisAssumptions()
    g = em_design.derive(p)
    r = p.rotor
    bom = _mfg.bill_of_materials(p)
    m_mag_per_pole = bom["magnet_mass_kg"] / r.pole_count

    ext = em_design._magnet_pocket_extent(p, g)
    cg_r = (ext[0] + ext[1]) / 2.0 if ext else g.rotor_outer_radius * 0.7   # mm
    omega = a.overspeed_factor * a_em.max_speed_rpm * 2.0 * math.pi / 60.0

    # iron of the pole cap above the magnets also loads the bridges; approximate as
    # the magnet mass again (cap iron ~ magnet mass order) for a conservative load.
    loaded_mass = m_mag_per_pole * 2.0
    f_cf = loaded_mass * omega ** 2 * (cg_r / 1000.0)        # N per pole

    # resisting steel: two outer bridges + the centre rib, each x stack length
    bridge_area = (2.0 * r.outer_bridge + 2.0 * r.center_post_halfwidth) * p.stack_length  # mm^2
    sigma = f_cf / (bridge_area * 1e-6) / 1e6 if bridge_area > 0 else float("inf")  # MPa
    sf = a.rotor_steel_yield_mpa / sigma if sigma > 0 else float("inf")
    return RotorStress(
        overspeed_rpm=a.overspeed_factor * a_em.max_speed_rpm,
        magnet_mass_per_pole_kg=m_mag_per_pole, cg_radius_mm=cg_r,
        bridge_area_per_pole_mm2=bridge_area, centrifugal_force_per_pole_n=f_cf,
        bridge_stress_mpa=sigma, safety_factor=sf, safe=sf >= a.target_structural_sf,
    )


# --------------------------------------------------------------------------- #
# cogging / NVH index
# --------------------------------------------------------------------------- #
@dataclass
class CoggingIndex:
    slots: int
    poles: int
    gcd: int
    lcm: int                 # cogging periods per mechanical revolution
    cogging_per_rev: int
    note: str


def cogging_index(p: MotorParams) -> CoggingIndex:
    z = p.stator.slot_count
    pp = p.rotor.pole_count
    gcd = math.gcd(z, pp)
    lcm = z * pp // gcd
    # higher LCM => more cogging cycles/rev => lower per-cycle amplitude; high GCD
    # (here 6) means strongly-aligned slots/poles -> more cogging than a fractional
    # combo. Integer-slot q=3 is a deliberate trade for low MMF harmonics.
    note = ("integer-slot q=%.0f: LCM %d cogging cycles/rev, GCD %d. Higher GCD = "
            "more cogging energy; mitigate with rotor/stator skew (~1 slot pitch) "
            "or pole-arc/notch shaping. FEA confirms amplitude." %
            (z / (pp * p.winding.phases), lcm, gcd))
    return CoggingIndex(slots=z, poles=pp, gcd=gcd, lcm=lcm, cogging_per_rev=lcm, note=note)


# --------------------------------------------------------------------------- #
# combined report
# --------------------------------------------------------------------------- #
def analysis_report(p: MotorParams, a_em: "em_design.EMAssumptions" = None,
                    a: AnalysisAssumptions = None) -> str:
    a_em = a_em or em_design.EMAssumptions()
    a = a or AnalysisAssumptions()
    perf = em_design.estimate_performance(p, a_em)
    lb_base = loss_breakdown(p, perf.base_speed_rpm, a_em.j_cont_a_mm2, a_em, a)
    lb_max = loss_breakdown(p, a_em.max_speed_rpm, a_em.j_cont_a_mm2, a_em, a)
    th = thermal_rating(p, a_em, a)
    dm = demag_margin(p, a_em, a)
    rs = rotor_stress(p, a_em, a)
    cg = cogging_index(p)

    def kw(x): return x / 1000.0
    return "\n".join([
        f"First-order analyses (NOT FEA -- order-of-magnitude) -- {p.name}",
        f"  LOSSES @ base {lb_base.speed_rpm:.0f} rpm (f_e {lb_base.elec_freq_hz:.0f} Hz, "
        f"J {lb_base.current_density_a_mm2:.0f} A/mm2):",
        f"    Cu DC {kw(lb_base.p_cu_dc_w):.2f} kW + AC {kw(lb_base.p_cu_ac_w):.2f} kW "
        f"(F_R {lb_base.ac_resistance_factor:.2f}) | iron {kw(lb_base.p_iron_w):.2f} kW | "
        f"magnet {lb_base.p_magnet_w:.0f} W | TOTAL {kw(lb_base.p_total_w):.2f} kW",
        f"  LOSSES @ max {lb_max.speed_rpm:.0f} rpm (f_e {lb_max.elec_freq_hz:.0f} Hz): "
        f"Cu-AC F_R {lb_max.ac_resistance_factor:.2f}, iron {kw(lb_max.p_iron_w):.2f} kW, "
        f"magnet {lb_max.p_magnet_w:.0f} W, TOTAL {kw(lb_max.p_total_w):.2f} kW",
        f"  THERMAL (winding limit {th.winding_temp_limit_c:.0f} C, coolant "
        f"{a.coolant_inlet_c:.0f} C, allow rise {th.allowable_rise_c:.0f} K):",
        f"    R_th winding->coolant {th.r_int_winding_to_iron_kpw + th.r_ext_iron_to_coolant_kpw:.3f} K/W "
        f"| continuous J {th.j_cont_thermal_a_mm2:.1f} A/mm2 (assumed {th.j_cont_assumed_a_mm2:.0f}) "
        f"-> cont. torque ~{th.torque_cont_thermal_nm:.0f} Nm",
        f"    magnet eddy temp rise ~{th.magnet_temp_rise_c:.0f} K "
        f"{'(THERMALLY LIMITED below the assumed J)' if th.thermally_limited else '(assumed J is thermally OK)'}",
        f"  DEMAG @ {dm.magnet_temp_c:.0f} C: Br {dm.br_hot_t:.2f} T, Hcj {dm.hcj_hot_ka_m:.0f} kA/m vs "
        f"peak demag field {dm.h_demag_peak_ka_m:.0f} kA/m -> margin x{dm.margin_ratio:.2f} "
        f"{'OK' if dm.safe else 'CHECK (conservative upper bound; verify in FEA)'}",
        f"  ROTOR STRESS @ {rs.overspeed_rpm:.0f} rpm (1.2x max): bridge sigma {rs.bridge_stress_mpa:.0f} MPa, "
        f"SF {rs.safety_factor:.2f} vs yield {a.rotor_steel_yield_mpa:.0f} MPa "
        f"{'OK' if rs.safe else 'CHECK -- thicken bridges/rib or use higher-strength steel (verify in FEA)'}",
        f"  COGGING: {cg.lcm} cycles/rev (GCD {cg.gcd}); {cg.note.split(': ',1)[1]}",
    ])
