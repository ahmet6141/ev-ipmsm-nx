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
from typing import List, Tuple

from .params import InverterParams

# SVPWM peak line-to-line output as a fraction of Vdc. With space-vector / third-
# harmonic injection the linear-modulation limit is a phase peak of Vdc/sqrt(3), i.e.
# an LL rms of Vdc/sqrt(2) = 0.707 * Vdc -- ~15.5% above plain SPWM's 0.612 * Vdc.
_SVPWM_LL_FRACTION = 1.0 / math.sqrt(2.0)   # 0.707  (SVPWM/THIPWM linear ceiling, LL rms)
_SPWM_LL_FRACTION = math.sqrt(3.0) / 2.0 / math.sqrt(2.0)   # 0.612  (plain SPWM, LL rms)
# steady-state device-voltage derating: keep the bus below ~70% of the blocking class,
# i.e. require Vrated >= Vdc / 0.7. SiC traction practice keeps peak device voltage
# (Vdc + switching overshoot) well under the rated value; 0.7 leaves room for overshoot.
_VOLTAGE_DERATE = 0.70
# switching-frequency rule of thumb: carrier >= this multiple of the max electrical freq.
_FSW_MULTIPLE = 10.0
# assumed peak-power conversion efficiency for the rough loss split. 0.97 is realistic
# for a SiC stage at PEAK current (the earlier 0.985 understated peak loss / heat flux
# ~2x); the Tj-to-coolant path remains the real limit (the low-severity finding).
_PEAK_EFFICIENCY = 0.97
# regulatory DC-link discharge: the bus must fall below this voltage within the time
# limit after disconnection (a passive bleed resistor across the link). ECE R100 /
# common OEM practice: < 60 V within 5 s.
_DISCHARGE_SAFE_V = 60.0
_DISCHARGE_TIME_LIMIT_S = 5.0
# conservative DC-link rms ripple current as a fraction of the peak phase current
# amplitude. The true worst case (Kolar) peaks near ~0.46 * i_amp at m~0.6/cos(phi)=1;
# 0.6 is an intentionally pessimistic bound for first-order capacitor sizing.
_RIPPLE_FRACTION = 0.6
# assumed CONTINUOUS-power conversion efficiency for the steady-state loss split (lower
# than peak: continuous operation runs the devices hotter, so a touch more loss).
_CONT_EFFICIENCY = 0.975
# material densities (kg/m^3) for a first-order packaging mass estimate.
_RHO_ALU = 2700.0       # enclosure + cold-plate aluminium
_RHO_COPPER = 8960.0    # busbars
_RHO_MODULE = 2500.0    # SiC power-module package (encapsulant + substrate + leadframe)
_RHO_CAP = 1600.0       # film-capacitor block (wound film + potting + can)
# six switch positions = three half-bridges.
N_POWER_MODULES = 6
# nominal SiC power-module footprint (the representative blank): length(X) x width(Y) x height(Z).
MODULE_LEN_MM = 26.0
MODULE_WID_MM = 40.0
MODULE_HGT_MM = 14.0
# the module row spans this fraction of the cold-plate length, biased toward -Y so the
# DC-link cap sits toward +Y (cap_y_frac) -- shared by the blueprint and validate().
MODULE_ROW_SPAN_FRAC = 0.70
MODULE_Y_FRAC = -0.18       # module-row centre Y as a fraction of the cold-plate width
CAP_Y_FRAC = 0.28           # DC-link cap centre Y as a fraction of the cold-plate width
CAP_LEN_FRAC = 0.80         # DC-link cap length as a fraction of the cold-plate length
CAP_WID_FRAC = 0.35         # DC-link cap width  as a fraction of the cold-plate width
CAP_HGT_MM = 30.0


# --------------------------------------------------------------------------- #
# packaging layout -- ONE source of truth for component placement, shared by the
# blueprint (geometry) and validate() (fit checks) so they can never disagree.
# All coordinates are the inverter LOCAL frame: +Z = height, base (mounting face) at
# z = 0, footprint centred on (0, 0); X = enclosure length, Y = width.
# --------------------------------------------------------------------------- #
@dataclass
class Layout:
    floor_z: float                  # inner-floor z (top of the enclosure base wall)
    plate_top_z: float              # top of the cold plate (modules + cap sit here)
    inner_l: float                  # enclosure inner cavity length (X)
    inner_w: float                  # enclosure inner cavity width  (Y)
    module_xs: List[float]          # module-row centre X positions (count = N_POWER_MODULES)
    module_y: float                 # module-row centre Y
    module_len: float               # module footprint length (X)
    module_wid: float               # module footprint width  (Y)
    module_hgt: float               # module height (Z)
    cap_cx: float                   # DC-link cap centre X
    cap_cy: float                   # DC-link cap centre Y
    cap_len: float                  # DC-link cap length (X)
    cap_wid: float                  # DC-link cap width  (Y)
    cap_hgt: float                  # DC-link cap height (Z)
    cap_terminal_face_y: float      # cap -Y face (toward the modules) the busbars land on
    busbar_cy: float                # busbar centre Y (both bars share it; they stack in Z)
    mounting_face_xyz: Tuple[float, float, float]   # the datum the assembly sits on the motor


def layout(p: InverterParams) -> Layout:
    """Resolve the packaging layout (cold plate / module row / DC-link cap) in the
    inverter local frame. The single source of truth for both the geometry blueprint
    and the validate() fit checks."""
    e, c = p.enclosure, p.cooling
    floor_z = e.wall_mm
    plate_top_z = floor_z + c.coldplate_thickness_mm
    inner_l = e.length_mm - 2.0 * e.wall_mm
    inner_w = e.width_mm - 2.0 * e.wall_mm

    row_span = c.coldplate_length_mm * MODULE_ROW_SPAN_FRAC
    spacing = row_span / (N_POWER_MODULES - 1) if N_POWER_MODULES > 1 else 0.0
    x0 = -row_span / 2.0
    module_xs = [x0 + spacing * i for i in range(N_POWER_MODULES)]
    module_y = MODULE_Y_FRAC * c.coldplate_width_mm

    cap_cy = CAP_Y_FRAC * c.coldplate_width_mm
    cap_wid = CAP_WID_FRAC * c.coldplate_width_mm
    # the cap -Y face (toward the module row) is the terminal face the busbars land on.
    cap_terminal_face_y = cap_cy - cap_wid / 2.0
    # the laminated busbar pair sits in the gap BETWEEN the module row and the cap, with
    # each bar's +Y edge meeting the cap terminal pad's outer face (proud of the cap face
    # by busbar.terminal_pad_proj_mm) -- a TOUCHING contact, never buried in the cap. The
    # two bars share this Y and stack in Z (see blueprint), so this lands both +Y edges on
    # the pad face. Falls back to the gap mid-line if the bar would otherwise hit the
    # modules (degenerate small-cap geometry); validate() flags the real overhang case.
    busbar_cy = cap_terminal_face_y - p.busbar.terminal_pad_proj_mm - p.busbar.width_mm / 2.0

    return Layout(
        floor_z=floor_z, plate_top_z=plate_top_z, inner_l=inner_l, inner_w=inner_w,
        module_xs=module_xs, module_y=module_y,
        module_len=MODULE_LEN_MM, module_wid=MODULE_WID_MM, module_hgt=MODULE_HGT_MM,
        cap_cx=0.0, cap_cy=cap_cy,
        cap_len=CAP_LEN_FRAC * c.coldplate_length_mm,
        cap_wid=cap_wid, cap_hgt=CAP_HGT_MM,
        cap_terminal_face_y=cap_terminal_face_y, busbar_cy=busbar_cy,
        mounting_face_xyz=(0.0, 0.0, 0.0),
    )


@dataclass
class DerivedInverter:
    # power-device sizing
    peak_phase_current_amp_a: float     # peak phase current amplitude (sqrt2 * rms)
    switch_current_rating_a: float      # required device current rating (with margin)
    min_switch_voltage_class_v: float   # required blocking class = Vdc / derate
    voltage_headroom_v: float           # switch class - required min class (>=0 is OK)
    voltage_derate_ok: bool             # class clears both derate AND max transient
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
    cont_loss_kw: float                 # dissipated at CONTINUOUS power (steady-state)
    coldplate_area_m2: float
    coldplate_heat_flux_w_cm2: float    # peak loss over the cold-plate footprint
    cont_heat_flux_w_cm2: float         # continuous loss over the cold-plate footprint
    cont_dc_current_a: float            # continuous DC-bus current (cont power / Vdc)
    peak_dc_current_a: float            # peak DC-bus current (peak power / Vdc)
    # DC-link energy (from the fitted capacitance, the real packaging driver)
    dc_link_energy_j: float             # 0.5*C*V^2 stored at nominal bus voltage
    dc_link_discharge_time_s: float     # passive bleed time to reach the safe voltage
    dc_link_discharge_ok: bool          # discharges to < safe V within the time limit
    # packaging (from the geometry layout)
    packaging_volume_l: float           # enclosure outer envelope volume (litres)
    estimated_mass_kg: float            # first-order assembly mass (alu+cu+modules+cap)
    power_density_kw_per_l: float       # peak power / packaging volume


def derive(p: InverterParams) -> DerivedInverter:
    b, m, s = p.bus, p.motor, p.power_stage
    d, r, c = p.dc_link, p.regen, p.cooling

    # peak phase current amplitude (rms -> amplitude) and required switch rating
    i_peak_amp = m.peak_phase_current_arms * math.sqrt(2.0)
    i_switch = i_peak_amp * s.current_margin

    # voltage derating: the blocking class must clear Vdc/0.7 (steady-state derate) AND
    # exceed the worst-case transient bus voltage (regen / overshoot at the device).
    v_class_min = b.dc_voltage_v / _VOLTAGE_DERATE
    v_headroom = s.switch_voltage_class_v - v_class_min
    v_derate_ok = (s.switch_voltage_class_v >= v_class_min
                   and s.switch_voltage_class_v > b.max_transient_v)

    # max electrical frequency = mech freq * pole pairs; carrier should clear 10x it
    f_elec = (m.max_speed_rpm / 60.0) * m.pole_pairs
    f_sw_min_khz = _FSW_MULTIPLE * f_elec / 1000.0
    fsw_ok = (s.switching_freq_khz * 1000.0) >= (_FSW_MULTIPLE * f_elec)

    # field weakening: back-EMF LL at max speed vs the inverter LL output ceiling. The
    # ceiling depends on the modulation: SVPWM/DPWM reach 0.707*Vdc LL rms, plain SPWM
    # only 0.612*Vdc.
    backemf_max = m.backemf_v_per_krpm_ll * m.max_speed_rpm / 1000.0
    ll_fraction = _SPWM_LL_FRACTION if s.modulation == "SPWM" else _SVPWM_LL_FRACTION
    inv_max_ll = ll_fraction * b.dc_voltage_v
    fw_ratio = backemf_max / inv_max_ll if inv_max_ll > 0 else float("inf")

    # DC-link ripple current (rough) and the required cap ripple rating
    i_ripple = _RIPPLE_FRACTION * i_peak_amp
    cap_ripple = i_ripple * d.ripple_current_margin

    # regen is the lesser of: the inverter regen setting, the pack charge-acceptance,
    # and the motor's own peak (generating) capability -- it cannot push back more than
    # the machine can produce nor more than the battery will take.
    regen_eff = (min(r.max_regen_power_kw, r.battery_charge_limit_kw, m.peak_power_kw)
                 if r.enabled else 0.0)

    # rough loss split at peak + continuous power and the resulting cold-plate heat flux.
    # The cold plate must reject the CONTINUOUS loss indefinitely; the peak loss is a
    # short transient ridden out by the plate + module thermal mass.
    loss_kw = m.peak_power_kw * (1.0 - _PEAK_EFFICIENCY)
    cont_loss_kw = m.cont_power_kw * (1.0 - _CONT_EFFICIENCY)
    area_m2 = (c.coldplate_length_mm * c.coldplate_width_mm) * 1e-6
    flux_w_cm2 = (loss_kw * 1000.0) / (area_m2 * 1.0e4) if area_m2 > 0 else float("inf")
    cont_flux_w_cm2 = (cont_loss_kw * 1000.0) / (area_m2 * 1.0e4) if area_m2 > 0 else float("inf")

    # DC-bus current (power / bus voltage) -- the busbar + cap + connector current path.
    vdc = b.dc_voltage_v if b.dc_voltage_v > 0 else float("inf")
    cont_dc_a = m.cont_power_kw * 1000.0 / vdc
    peak_dc_a = m.peak_power_kw * 1000.0 / vdc

    # DC-link stored energy from the fitted capacitance at the nominal bus.
    dc_link_energy_j = 0.5 * (d.capacitance_uf * 1e-6) * b.dc_voltage_v ** 2

    # passive bleed discharge: t = R*C*ln(Vdc / Vsafe). Must reach < _DISCHARGE_SAFE_V
    # within _DISCHARGE_TIME_LIMIT_S. An active-discharge path is faster but the
    # passive bleed is the regulatory fallback, so we size against IT.
    cap_f = d.capacitance_uf * 1e-6
    r_bleed = d.bleed_resistor_kohm * 1.0e3
    if r_bleed > 0 and cap_f > 0 and b.dc_voltage_v > _DISCHARGE_SAFE_V:
        discharge_t = r_bleed * cap_f * math.log(b.dc_voltage_v / _DISCHARGE_SAFE_V)
    elif b.dc_voltage_v <= _DISCHARGE_SAFE_V:
        discharge_t = 0.0
    else:
        discharge_t = float("inf")     # no bleed fitted -> never bleeds down passively
    discharge_ok = discharge_t <= _DISCHARGE_TIME_LIMIT_S

    # first-order packaging volume + mass from the geometry layout. Volume = the outer
    # enclosure envelope; mass sums the dominant solids (alu shell+plate, cu busbars,
    # module packages, cap block) -- a sanity figure, not a CAD mass-properties result.
    lay = layout(p)
    env = p.enclosure
    vol_mm3 = env.length_mm * env.width_mm * env.height_mm
    packaging_volume_l = vol_mm3 * 1e-6
    # aluminium = enclosure shell (outer box - inner cavity) + cold-plate slab
    shell_mm3 = vol_mm3 - max(0.0, lay.inner_l) * max(0.0, lay.inner_w) * max(0.0, env.height_mm - 2.0 * env.wall_mm)
    plate_mm3 = c.coldplate_length_mm * c.coldplate_width_mm * c.coldplate_thickness_mm
    modules_mm3 = N_POWER_MODULES * lay.module_len * lay.module_wid * lay.module_hgt
    cap_mm3 = lay.cap_len * lay.cap_wid * lay.cap_hgt
    bus_mm3 = 0.0
    if p.busbar.enabled:
        bus_mm3 = 2.0 * p.busbar.width_mm * p.busbar.thickness_mm * lay.cap_len  # +/- laminated pair
    mass_kg = ((shell_mm3 + plate_mm3) * _RHO_ALU + modules_mm3 * _RHO_MODULE
               + cap_mm3 * _RHO_CAP + bus_mm3 * _RHO_COPPER) * 1e-9
    power_density = m.peak_power_kw / packaging_volume_l if packaging_volume_l > 0 else float("inf")

    return DerivedInverter(
        peak_phase_current_amp_a=round(i_peak_amp, 1),
        switch_current_rating_a=round(i_switch, 1),
        min_switch_voltage_class_v=round(v_class_min, 1),
        voltage_headroom_v=round(v_headroom, 1),
        voltage_derate_ok=v_derate_ok,
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
        cont_loss_kw=round(cont_loss_kw, 2),
        coldplate_area_m2=round(area_m2, 4),
        coldplate_heat_flux_w_cm2=round(flux_w_cm2, 2),
        cont_heat_flux_w_cm2=round(cont_flux_w_cm2, 2),
        cont_dc_current_a=round(cont_dc_a, 1),
        peak_dc_current_a=round(peak_dc_a, 1),
        dc_link_energy_j=round(dc_link_energy_j, 2),
        dc_link_discharge_time_s=round(discharge_t, 2),
        dc_link_discharge_ok=discharge_ok,
        packaging_volume_l=round(packaging_volume_l, 2),
        estimated_mass_kg=round(mass_kg, 2),
        power_density_kw_per_l=round(power_density, 2),
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
    d = p.dc_link
    g = derive(p)

    if b.architecture not in ("400V", "800V"):
        issues.append("bus.architecture '%s' unknown (400V|800V)" % b.architecture)
    if s.modulation not in ("SVPWM", "SPWM", "DPWM"):
        issues.append("power_stage.modulation '%s' unknown (SVPWM|SPWM|DPWM)" % s.modulation)

    # switch blocking class must clear the steady-state derate (Vdc/0.7) AND exceed the
    # worst-case transient bus voltage.
    if not g.voltage_derate_ok:
        issues.append("switch_voltage_class_v %.0f below required %.0f V (Vdc/%.2f) or transient %.0f V"
                      % (s.switch_voltage_class_v, g.min_switch_voltage_class_v,
                         _VOLTAGE_DERATE, b.max_transient_v))
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

    # DC-link discharge: the bus must passively bleed to < safe voltage within the
    # regulatory time after HV disconnect (the low-severity finding: the model stored
    # ~40 J at 400 V with no bleed provision). A passive bleed resistor is required.
    if not g.dc_link_discharge_ok:
        if d.bleed_resistor_kohm <= 0:
            issues.append("no DC-link bleed resistor fitted: the %.0f uF link will not "
                          "discharge to < %.0f V (fit a passive bleed across the link)"
                          % (d.capacitance_uf, _DISCHARGE_SAFE_V))
        else:
            # the largest bleed resistance that still meets the time limit
            import math as _m
            r_max_kohm = (_DISCHARGE_TIME_LIMIT_S
                          / (d.capacitance_uf * 1e-6 * _m.log(b.dc_voltage_v / _DISCHARGE_SAFE_V))
                          / 1.0e3)
            issues.append("DC-link discharge %.1f s > %.0f s limit: lower bleed_resistor_kohm "
                          "to <= %.0f kOhm (or add active discharge)"
                          % (g.dc_link_discharge_time_s, _DISCHARGE_TIME_LIMIT_S, r_max_kohm))

    # geometry: the cold plate must fit inside the enclosure inner cavity
    lay = layout(p)
    inner_l, inner_w = lay.inner_l, lay.inner_w
    if c.coldplate_length_mm >= inner_l:
        issues.append("cold plate length %.0f does not fit enclosure inner length %.0f"
                      % (c.coldplate_length_mm, inner_l))
    if c.coldplate_width_mm >= inner_w:
        issues.append("cold plate width %.0f does not fit enclosure inner width %.0f"
                      % (c.coldplate_width_mm, inner_w))

    # the stacked internals (cold plate + the taller of module / cap, + busbar standoff)
    # plus the lid flange lip must all fit under the enclosure height (the lid seals on
    # top of the wall, so the cavity available to the internals is height - wall).
    stack_z = max(lay.module_hgt, lay.cap_hgt)
    if p.busbar.enabled:
        stack_z = max(stack_z, p.busbar.height_mm + p.busbar.thickness_mm)
    internals_top = lay.plate_top_z + stack_z
    if internals_top >= e.height_mm - e.wall_mm:
        issues.append("internals top %.0f mm exceeds enclosure inner height %.0f mm "
                      "(cold plate + tallest component does not clear the lid)"
                      % (internals_top, e.height_mm - e.wall_mm))

    # the module row + DC-link cap must sit ON the cold plate (footprint within the plate)
    hx, hy = c.coldplate_length_mm / 2.0, c.coldplate_width_mm / 2.0
    mod_x_max = max(abs(x) for x in lay.module_xs) + lay.module_len / 2.0
    mod_y_max = abs(lay.module_y) + lay.module_wid / 2.0
    if mod_x_max > hx or mod_y_max > hy:
        issues.append("power-module row (%.0f x %.0f mm extent) overhangs the cold plate"
                      % (2.0 * mod_x_max, 2.0 * mod_y_max))
    cap_x_max = abs(lay.cap_cx) + lay.cap_len / 2.0
    cap_y_max = abs(lay.cap_cy) + lay.cap_wid / 2.0
    if cap_x_max > hx or cap_y_max > hy:
        issues.append("DC-link cap (%.0f x %.0f mm) overhangs the cold plate"
                      % (lay.cap_len, lay.cap_wid))
    # module row and the DC-link cap must not collide in Y (modules toward -Y, cap +Y)
    gap_y = (lay.cap_cy - lay.cap_wid / 2.0) - (lay.module_y + lay.module_wid / 2.0)
    if gap_y < 0.0:
        issues.append("DC-link cap overlaps the power-module row in Y (gap %.1f mm < 0)" % gap_y)

    # coolant ports: the inlet/outlet pair must sit within the cold-plate -X end face
    if c.port_diameter_mm > 0.0:
        port_y_max = c.port_pitch_mm / 2.0 + c.port_diameter_mm / 2.0
        if port_y_max > hy:
            issues.append("coolant ports (pitch %.0f, dia %.0f) fall outside the cold-plate width"
                          % (c.port_pitch_mm, c.port_diameter_mm))
        if c.port_diameter_mm >= c.coldplate_thickness_mm:
            issues.append("coolant port dia %.0f >= cold-plate thickness %.0f (no metal around the bore)"
                          % (c.port_diameter_mm, c.coldplate_thickness_mm))

    # lid bolt pattern must fit on the top sealing flange (inset within the flange width)
    if e.lid_flange_mm > 0.0 and e.lid_bolt_count > 0:
        if e.lid_bolt_inset_mm + e.lid_bolt_diameter_mm / 2.0 > e.lid_flange_mm:
            issues.append("lid bolts (inset %.1f, dia %.1f) do not fit the %.1f mm lid flange"
                          % (e.lid_bolt_inset_mm, e.lid_bolt_diameter_mm, e.lid_flange_mm))
        if e.lid_bolt_count % 2 != 0:
            issues.append("lid_bolt_count %d should be even (symmetric perimeter pattern)" % e.lid_bolt_count)

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
        "  voltage derating         : class %.0f V vs req %.0f V (Vdc/%.2f), transient %.0f V -> %s" % (
            s.switch_voltage_class_v, g.min_switch_voltage_class_v, _VOLTAGE_DERATE,
            b.max_transient_v, "OK" if g.voltage_derate_ok else "INSUFFICIENT"),
        "  max electrical freq      : %.0f Hz  (fsw %.1f kHz, recommend >= %.1f kHz) %s" % (
            g.max_electrical_freq_hz, s.switching_freq_khz, g.min_switching_freq_khz,
            "OK" if g.switching_freq_adequate else "LOW"),
        "  field weakening          : back-EMF %.0f V LL @ max vs %.0f V %s ceiling -> ratio %.2f %s" % (
            g.backemf_max_ll_v, g.inverter_max_ll_v, s.modulation, g.field_weakening_ratio,
            "(FW required)" if g.field_weakening_ratio > 1.0 else "(no FW)"),
        "  DC link                  : %.0f uF %s, ripple ~%.0f A rms (cap rating >= %.0f A)" % (
            d.capacitance_uf, d.cap_technology, g.dc_ripple_current_arms, g.cap_ripple_rating_a),
        "  regen                    : %s, effective %.0f kW (battery limit %.0f kW)" % (
            "enabled" if r.enabled else "disabled", g.effective_regen_power_kw, r.battery_charge_limit_kw),
        "  control                  : %s%s%s, %s, %s, %s%s" % (
            ctl.scheme, " +MTPA" if ctl.mtpa else "", " +FW" if ctl.field_weakening else "",
            ctl.position_sensor, ctl.functional_safety,
            "STO" if ctl.sto else "no-STO", " +ASC" if ctl.active_short_circuit else ""),
        "  DC-bus current           : %.0f A cont / %.0f A peak (at %.0f V)" % (
            g.cont_dc_current_a, g.peak_dc_current_a, b.dc_voltage_v),
        "  DC-link energy           : %.2f J stored (%.0f uF @ %.0f V)" % (
            g.dc_link_energy_j, d.capacitance_uf, b.dc_voltage_v),
        "  DC-link discharge        : %.1f s to < %.0f V via %.0f kOhm bleed%s -> %s" % (
            g.dc_link_discharge_time_s, _DISCHARGE_SAFE_V, d.bleed_resistor_kohm,
            " (+active)" if d.active_discharge else "",
            "OK" if g.dc_link_discharge_ok else "TOO SLOW"),
        "  peak loss / heat flux    : %.2f kW over %.0fx%.0f mm plate -> %.1f W/cm^2 (Tj-coolant path is the real limit)" % (
            g.peak_loss_kw, c.coldplate_length_mm, c.coldplate_width_mm, g.coldplate_heat_flux_w_cm2),
        "  continuous loss / flux   : %.2f kW -> %.1f W/cm^2 (steady-state cold-plate duty)" % (
            g.cont_loss_kw, g.cont_heat_flux_w_cm2),
        "  packaging                : %.1f L, ~%.1f kg -> %.1f kW/L (peak)" % (
            g.packaging_volume_l, g.estimated_mass_kg, g.power_density_kw_per_l),
        "  enclosure                : %.0f x %.0f x %.0f mm, %.0f mm wall, %d phase connector(s)%s%s" % (
            p.enclosure.length_mm, p.enclosure.width_mm, p.enclosure.height_mm, p.enclosure.wall_mm,
            p.enclosure.connector_count, " +HV DC" if p.enclosure.hv_connector else "",
            " +LV" if p.enclosure.lv_connector else ""),
        "  mounting-face datum      : (%.0f, %.0f, %.0f) mm local -- enclosure base centre (sits on the motor)" % (
            layout(p).mounting_face_xyz),
        "  validation: %s" % ("OK (design is sound + buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
