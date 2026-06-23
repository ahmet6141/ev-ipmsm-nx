"""Inverter engineering: power-device sizing, field-weakening ratio, DC-link ripple,
regen limit, loss / thermal budget, and a geometric buildability check. Pure math --
NX-independent and unit-tested (the analogue of motor_nx.em_design / driveline_nx.engineering).

All loads are referred from the motor interface (params.motor), the worst case being
the peak phase current at maximum speed. First-order closed-form estimates; verify the
power-loss model, ripple spectrum and thermal network with detailed tools (datasheet
SOA, PLECS, CFD).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import List

from .params import InverterParams

# SVPWM peak line-to-line output as a fraction of Vdc (2/sqrt(3) of the half-bus
# fundamental peak => 0.612 * Vdc rms LL at the linear-modulation limit).
_SVPWM_LL_FRACTION = 0.612
# transient-voltage headroom: the switch blocking class must clear Vdc by this factor.
_VOLTAGE_HEADROOM = 1.8
# switching-frequency rule of thumb: carrier >= this multiple of the max electrical freq.
_FSW_MULTIPLE = 10.0
# assumed peak-power conversion efficiency for the rough loss split.
_PEAK_EFFICIENCY = 0.985
# rough DC-link rms ripple current as a fraction of the peak phase current amplitude.
_RIPPLE_FRACTION = 0.6


@dataclass
class DerivedInverter:
    # power-device sizing
    peak_phase_current_amp_a: float     # peak phase current amplitude (sqrt2 * rms)
    switch_current_rating_a: float      # required device current rating (with margin)
    voltage_headroom_v: float           # switch class - Vdc * headroom (>=0 is OK)
    # electrical frequency / switching
    max_electrical_freq_hz: float
    min_switching_freq_khz: float       # recommended carrier (>= 10x f_elec)
    switching_freq_adequate: bool
    # field weakening
    backemf_max_ll_v: float             # back-EMF LL rms at max speed
    inverter_max_ll_v: float            # SVPWM LL rms output ceiling
    field_weakening_ratio: float        # backemf_max / inverter ceiling (>1 => FW needed)
    # DC link
    dc_ripple_current_arms: float       # estimated rms ripple current
    cap_ripple_rating_a: float          # required cap ripple-current rating (with margin)
    # regen
    effective_regen_power_kw: float     # min(inverter regen, battery charge limit)
    # loss / thermal
    peak_loss_kw: float                 # dissipated at peak power
    coldplate_area_m2: float
    coldplate_heat_flux_w_cm2: float    # peak loss over the cold-plate footprint


def derive(p: InverterParams) -> DerivedInverter:
    b, m, s = p.bus, p.motor, p.power_stage
    d, r, c = p.dc_link, p.regen, p.cooling

    # peak phase current amplitude (rms -> amplitude) and required switch rating
    i_peak_amp = m.peak_phase_current_arms * math.sqrt(2.0)
    i_switch = i_peak_amp * s.current_margin

    # transient voltage headroom: device blocking class vs Vdc * factor
    v_headroom = s.switch_voltage_class_v - b.dc_voltage_v * _VOLTAGE_HEADROOM

    # max electrical frequency = mech freq * pole pairs; carrier should clear 10x it
    f_elec = (m.max_speed_rpm / 60.0) * m.pole_pairs
    f_sw_min_khz = _FSW_MULTIPLE * f_elec / 1000.0
    fsw_ok = (s.switching_freq_khz * 1000.0) >= (_FSW_MULTIPLE * f_elec)

    # field weakening: back-EMF LL at max speed vs the SVPWM LL output ceiling
    backemf_max = m.backemf_v_per_krpm_ll * m.max_speed_rpm / 1000.0
    inv_max_ll = _SVPWM_LL_FRACTION * b.dc_voltage_v
    fw_ratio = backemf_max / inv_max_ll if inv_max_ll > 0 else float("inf")

    # DC-link ripple current (rough) and the required cap ripple rating
    i_ripple = _RIPPLE_FRACTION * i_peak_amp
    cap_ripple = i_ripple * d.ripple_current_margin

    # regen is the lesser of what the inverter allows and what the pack will accept
    regen_eff = min(r.max_regen_power_kw, r.battery_charge_limit_kw) if r.enabled else 0.0

    # rough loss split at peak power and the resulting cold-plate heat flux
    loss_kw = m.peak_power_kw * (1.0 - _PEAK_EFFICIENCY)
    area_m2 = (c.coldplate_length_mm * c.coldplate_width_mm) * 1e-6
    flux_w_cm2 = (loss_kw * 1000.0) / (area_m2 * 1.0e4) if area_m2 > 0 else float("inf")

    return DerivedInverter(
        peak_phase_current_amp_a=round(i_peak_amp, 1),
        switch_current_rating_a=round(i_switch, 1),
        voltage_headroom_v=round(v_headroom, 1),
        max_electrical_freq_hz=round(f_elec, 1),
        min_switching_freq_khz=round(f_sw_min_khz, 2),
        switching_freq_adequate=fsw_ok,
        backemf_max_ll_v=round(backemf_max, 1),
        inverter_max_ll_v=round(inv_max_ll, 1),
        field_weakening_ratio=round(fw_ratio, 3),
        dc_ripple_current_arms=round(i_ripple, 1),
        cap_ripple_rating_a=round(cap_ripple, 1),
        effective_regen_power_kw=round(regen_eff, 1),
        peak_loss_kw=round(loss_kw, 2),
        coldplate_area_m2=round(area_m2, 4),
        coldplate_heat_flux_w_cm2=round(flux_w_cm2, 2),
    )


# --------------------------------------------------------------------------- #
# buildability check (engineering + geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: InverterParams) -> List[str]:
    """Return a list of engineering/geometric problems (empty list => buildable).
    Mirrors driveline_nx.engineering.validate()'s contract."""
    issues: List[str] = []
    b, m, s = p.bus, p.motor, p.power_stage
    r, c, e = p.regen, p.cooling, p.enclosure
    g = derive(p)

    if b.architecture not in ("400V", "800V"):
        issues.append("bus.architecture '%s' unknown (400V|800V)" % b.architecture)
    if s.modulation not in ("SVPWM", "SPWM", "DPWM"):
        issues.append("power_stage.modulation '%s' unknown (SVPWM|SPWM|DPWM)" % s.modulation)

    # switch blocking class must clear Vdc with transient headroom
    if g.voltage_headroom_v < 0.0:
        issues.append("switch_voltage_class_v %.0f below Vdc x %.1f = %.0f V (insufficient headroom)"
                      % (s.switch_voltage_class_v, _VOLTAGE_HEADROOM, b.dc_voltage_v * _VOLTAGE_HEADROOM))
    # switching frequency must clear 10x the max electrical frequency
    if not g.switching_freq_adequate:
        issues.append("switching_freq_khz %.1f below %.1f kHz (>= 10x f_elec %.0f Hz)"
                      % (s.switching_freq_khz, g.min_switching_freq_khz, g.max_electrical_freq_hz))
    # device current margin
    if s.current_margin < 1.2:
        issues.append("power_stage.current_margin %.2f < 1.2 (too little device headroom)" % s.current_margin)
    # regen must not exceed the battery charge-acceptance
    if r.enabled and r.max_regen_power_kw > r.battery_charge_limit_kw:
        issues.append("max_regen_power_kw %.0f exceeds battery_charge_limit_kw %.0f"
                      % (r.max_regen_power_kw, r.battery_charge_limit_kw))

    # geometry: the cold plate must fit inside the enclosure inner cavity
    inner_l = e.length_mm - 2.0 * e.wall_mm
    inner_w = e.width_mm - 2.0 * e.wall_mm
    if c.coldplate_length_mm >= inner_l:
        issues.append("cold plate length %.0f does not fit enclosure inner length %.0f"
                      % (c.coldplate_length_mm, inner_l))
    if c.coldplate_width_mm >= inner_w:
        issues.append("cold plate width %.0f does not fit enclosure inner width %.0f"
                      % (c.coldplate_width_mm, inner_w))
    if c.coldplate_thickness_mm >= e.height_mm - 2.0 * e.wall_mm:
        issues.append("cold plate thickness %.0f does not fit enclosure inner height"
                      % c.coldplate_thickness_mm)

    # positive dimensions
    for name, val in (("enclosure.length_mm", e.length_mm), ("enclosure.width_mm", e.width_mm),
                      ("enclosure.height_mm", e.height_mm), ("enclosure.wall_mm", e.wall_mm),
                      ("cooling.coldplate_length_mm", c.coldplate_length_mm),
                      ("cooling.coldplate_width_mm", c.coldplate_width_mm),
                      ("cooling.coldplate_thickness_mm", c.coldplate_thickness_mm)):
        if val <= 0.0:
            issues.append("%s must be > 0 (got %.1f)" % (name, val))
    if 2.0 * e.wall_mm >= min(e.length_mm, e.width_mm, e.height_mm):
        issues.append("enclosure.wall_mm too thick: leaves no inner cavity")

    return issues


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def report(p: InverterParams) -> str:
    g = derive(p)
    b, m, s = p.bus, p.motor, p.power_stage
    d, r, c, ctl = p.dc_link, p.regen, p.cooling, p.control
    issues = validate(p)
    lines = [
        "Inverter design summary -- %s" % p.name,
        "  DC bus                   : %.0f V (%s, %.0f-%.0f V window)" % (
            b.dc_voltage_v, b.architecture, b.min_voltage_v, b.max_transient_v),
        "  motor (peak)             : %.0f kW / %.0f Nm, %.0f rpm max, %d pole pairs" % (
            m.peak_power_kw, m.peak_torque_nm, m.max_speed_rpm, m.pole_pairs),
        "  power stage              : %s x%d, %.0f V class, %s" % (
            s.device, s.modules_parallel, s.switch_voltage_class_v, s.modulation),
        "  peak phase current       : %.0f A rms -> %.0f A amplitude" % (
            m.peak_phase_current_arms, g.peak_phase_current_amp_a),
        "  switch current rating     : %.0f A  (margin x%.2f)" % (g.switch_current_rating_a, s.current_margin),
        "  voltage headroom         : %+.0f V  (class %.0f vs Vdc x%.1f)" % (
            g.voltage_headroom_v, s.switch_voltage_class_v, _VOLTAGE_HEADROOM),
        "  max electrical freq      : %.0f Hz  (fsw %.1f kHz, recommend >= %.1f kHz) %s" % (
            g.max_electrical_freq_hz, s.switching_freq_khz, g.min_switching_freq_khz,
            "OK" if g.switching_freq_adequate else "LOW"),
        "  field weakening          : back-EMF %.0f V LL @ max vs %.0f V SVPWM ceiling -> ratio %.2f %s" % (
            g.backemf_max_ll_v, g.inverter_max_ll_v, g.field_weakening_ratio,
            "(FW required)" if g.field_weakening_ratio > 1.0 else "(no FW)"),
        "  DC link                  : %.0f uF %s, ripple ~%.0f A rms (cap rating >= %.0f A)" % (
            d.capacitance_uf, d.cap_technology, g.dc_ripple_current_arms, g.cap_ripple_rating_a),
        "  regen                    : %s, effective %.0f kW (battery limit %.0f kW)" % (
            "enabled" if r.enabled else "disabled", g.effective_regen_power_kw, r.battery_charge_limit_kw),
        "  control                  : %s%s%s, %s, %s, %s%s" % (
            ctl.scheme, " +MTPA" if ctl.mtpa else "", " +FW" if ctl.field_weakening else "",
            ctl.position_sensor, ctl.functional_safety,
            "STO" if ctl.sto else "no-STO", " +ASC" if ctl.active_short_circuit else ""),
        "  peak loss / heat flux    : %.2f kW over %.0fx%.0f mm plate -> %.1f W/cm^2" % (
            g.peak_loss_kw, c.coldplate_length_mm, c.coldplate_width_mm, g.coldplate_heat_flux_w_cm2),
        "  enclosure                : %.0f x %.0f x %.0f mm, %.0f mm wall, %d phase connector(s)%s" % (
            p.enclosure.length_mm, p.enclosure.width_mm, p.enclosure.height_mm, p.enclosure.wall_mm,
            p.enclosure.connector_count, " +HV DC" if p.enclosure.hv_connector else ""),
        "  validation: %s" % ("OK (design is sound + buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
