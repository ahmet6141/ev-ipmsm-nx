"""Driveline engineering: gear ratios, output torque, half-shaft torsional sizing,
CV-joint duty, wheel-hub bearing life, and a geometric buildability check. Pure
math -- NX-independent and unit-tested (the analogue of motor_nx.em_design).

All loads are referred from the motor's peak torque (params.motor_peak_torque_nm)
through the single-speed final drive. First-order closed-form estimates; verify
gear ratings (ISO 6336) and bearing life (ISO 281) with detailed tools.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, List

from .params import DrivelineParams

# worst-case single-wheel torque share by differential type (for half-shaft sizing)
_TORQUE_BIAS = {"open": 0.5, "elsd": 0.6, "torque_vectoring": 0.6, "spool": 1.0}


@dataclass
class DerivedDriveline:
    final_drive_ratio: float
    input_torque_nm: float          # motor peak at the diff input
    ring_gear_torque_nm: float      # after the final drive (total to the diff case)
    per_wheel_torque_nm: float      # open diff splits evenly
    wheel_max_speed_rpm: float
    # half-shaft torsion
    halfshaft_shear_stress_mpa: float
    halfshaft_safety_factor: float
    halfshaft_polar_modulus_mm3: float
    # gearing
    input_pinion_pitch_diameter: float
    ring_pinion_ratio: float
    pitch_line_velocity_mps: float  # at the ring gear, at max speed
    # geometry envelope
    total_track_length_mm: float    # wheel-flange face to wheel-flange face (both sides)
    sides_modelled: int


def derive(p: DrivelineParams) -> DerivedDriveline:
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    ratio = d.final_drive_ratio

    input_torque = p.motor_peak_torque_nm
    ring_torque = input_torque * ratio
    n_sides = 2 if p.sides == "both" else 1
    # per-wheel design torque = ring torque x a worst-case bias factor: an open diff
    # splits 50/50; an e-LSD / torque-vectoring eDiff transiently biases more to one
    # wheel; a spool (locked) can put the whole axle torque through one shaft.
    bias = _TORQUE_BIAS.get(d.type, 0.5)
    per_wheel = ring_torque * bias

    wheel_max_rpm = p.motor_max_speed_rpm / ratio

    # half-shaft torsion: tau = T / Z_p,  Z_p = pi/16 * (D^4 - d^4) / D
    do = h.diameter
    di = h.bore_diameter if 0.0 < h.bore_diameter < h.diameter else 0.0
    zp = math.pi / 16.0 * (do ** 4 - di ** 4) / do if do > 0 else 0.0   # mm^3
    t_nmm = per_wheel * 1.0e3
    tau = (t_nmm / zp) if zp > 0 else float("inf")                       # MPa (N/mm^2)
    sf = (p.material.halfshaft_shear_allow_mpa / tau) if tau > 0 else float("inf")

    # gearing geometry / kinematics
    pinion_pd = d.input_pinion_pitch_diameter
    ring_pinion_ratio = d.ring_gear_pitch_diameter / pinion_pd if pinion_pd > 0 else 0.0
    # ring-gear pitch-line velocity at max wheel speed
    plv = math.pi * (d.ring_gear_pitch_diameter * 1e-3) * (wheel_max_rpm / 60.0)

    # total modelled track length (wheel-flange outer faces, both sides)
    half = (d.carrier_length / 2.0 + 5.0 + h.inboard_bell_length + h.length
            + h.outboard_bell_length + w.bearing_width + w.hub_flange_thickness)
    total = half * n_sides

    return DerivedDriveline(
        final_drive_ratio=ratio,
        input_torque_nm=round(input_torque, 2),
        ring_gear_torque_nm=round(ring_torque, 1),
        per_wheel_torque_nm=round(per_wheel, 1),
        wheel_max_speed_rpm=round(wheel_max_rpm, 1),
        halfshaft_shear_stress_mpa=round(tau, 1),
        halfshaft_safety_factor=round(sf, 2),
        halfshaft_polar_modulus_mm3=round(zp, 1),
        input_pinion_pitch_diameter=pinion_pd,
        ring_pinion_ratio=round(ring_pinion_ratio, 3),
        pitch_line_velocity_mps=round(plv, 2),
        total_track_length_mm=round(total, 1),
        sides_modelled=n_sides,
    )


def bearing_life_estimate(p: DrivelineParams, radial_load_n: float = 6000.0) -> Dict[str, Any]:
    """Very rough L10 wheel-hub bearing life (ISO 281, ball-bearing exponent 3).
    Dynamic capacity C is estimated from the bearing envelope; replace with the
    catalogue C for a real part. radial_load_n defaults to a ~600 kg corner load."""
    w = p.wheel_hub
    g = derive(p)
    # crude C ~ k * d_m^1.4 (catalogue-fit constant) for a medium ball-bearing unit;
    # k chosen so an ~84 mm-OD Gen-3 hub unit lands near a realistic ~35 kN C.
    d_m = 0.5 * (w.bearing_outer_diameter + w.bearing_bore_diameter)
    c_dyn = 100.0 * d_m ** 1.4    # N (order-of-magnitude only -- use the catalogue C)
    rpm = max(1e-6, g.wheel_max_speed_rpm * 0.15)   # ~15% of max as a duty-average
    l10_rev = (c_dyn / max(1.0, radial_load_n)) ** 3 * 1e6
    l10_hours = l10_rev / (rpm * 60.0)
    return {
        "dynamic_capacity_N_est": int(c_dyn),
        "assumed_radial_load_N": radial_load_n,
        "duty_speed_rpm": round(rpm, 1),
        "L10_million_rev": round(l10_rev / 1e6, 1),
        "L10_hours_est": int(l10_hours),
    }


# --------------------------------------------------------------------------- #
# buildability check (geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: DrivelineParams) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors motor_nx.em_design.validate()'s contract."""
    issues: List[str] = []
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    g = derive(p)

    if d.type not in ("open", "elsd", "torque_vectoring", "spool"):
        issues.append("differential.type '%s' unknown (open|elsd|torque_vectoring|spool)" % d.type)
    if p.sides not in ("both", "left", "right"):
        issues.append("sides '%s' unknown (both|left|right)" % p.sides)
    if d.final_drive_ratio <= 1.0:
        issues.append("final_drive_ratio %.2f must be > 1 for a reduction" % d.final_drive_ratio)

    # carrier wall must leave a bore
    if d.carrier_wall * 2.0 >= d.carrier_outer_diameter:
        issues.append("carrier_wall too thick: leaves no bore in the carrier")
    # ring gear must be bigger than the carrier it bolts to
    if d.ring_gear_pitch_diameter <= d.carrier_outer_diameter:
        issues.append("ring_gear_pitch_diameter must exceed carrier_outer_diameter")
    # half-shaft must pass through the side gear bore region
    if h.diameter >= d.side_gear_diameter:
        issues.append("halfshaft.diameter must be smaller than side_gear_diameter")
    # hollow shaft wall
    if h.bore_diameter and h.bore_diameter >= h.diameter - 4.0:
        issues.append("halfshaft.bore_diameter leaves < 2 mm wall (use a smaller bore)")
    # motor coupling bore must fit in the input flange
    if d.input_bore_diameter + 12.0 >= d.input_flange_diameter:
        issues.append("input_bore_diameter too large for input_flange_diameter")

    # wheel-mount: lug circle must fit on the flange and clear the pilot bore
    if not w.single_centre_nut:
        if w.lug_count < 3:
            issues.append("wheel_hub.lug_count must be >= 3")
        r_out = w.lug_pcd / 2.0 + w.lug_hole_diameter / 2.0
        if 2.0 * r_out >= w.hub_flange_diameter:
            issues.append("lug bolt circle + hole runs off the hub flange OD")
        r_in = w.lug_pcd / 2.0 - w.lug_hole_diameter / 2.0
        if 2.0 * r_in <= w.pilot_bore_diameter:
            issues.append("lug bolt circle overlaps the centre pilot bore")
    else:
        if w.centre_nut_diameter <= h.spline_diameter:
            issues.append("centre_nut_diameter must exceed the hub spline diameter")
    if w.pilot_bore_diameter >= w.hub_flange_diameter:
        issues.append("pilot_bore_diameter must be smaller than the hub flange OD")
    if w.bearing_bore_diameter >= w.bearing_outer_diameter:
        issues.append("bearing bore must be smaller than the bearing OD")

    # engineering margins (warnings, not hard stops, but reported)
    if g.halfshaft_safety_factor < 1.2:
        issues.append("halfshaft shear safety factor %.2f < 1.2 (increase diameter or reduce bore)"
                      % g.halfshaft_safety_factor)
    return issues


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def report(p: DrivelineParams) -> str:
    g = derive(p)
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    issues = validate(p)
    lines = [
        "Driveline design summary -- %s" % p.name,
        "  differential type        : %s%s" % (d.type, "  (+disconnect)" if d.disconnect else ""),
        "  final drive ratio        : %.2f : 1" % g.final_drive_ratio,
        "  input torque (motor peak): %.0f Nm @ <= %.0f rpm" % (g.input_torque_nm, p.motor_max_speed_rpm),
        "  ring-gear torque (total) : %.0f Nm" % g.ring_gear_torque_nm,
        "  per-wheel torque         : %.0f Nm  (%s, bias %.2f)" % (
            g.per_wheel_torque_nm,
            {"open": "open-diff 50/50 split", "spool": "locked -- full axle torque",
             "elsd": "e-LSD worst-case bias", "torque_vectoring": "TV worst-case bias"}.get(d.type, "bias"),
            _TORQUE_BIAS.get(d.type, 0.5)),
        "  wheel max speed          : %.0f rpm" % g.wheel_max_speed_rpm,
        "  ring : pinion ratio      : %.2f  (ring PD %.0f / pinion PD %.0f mm)" % (
            g.ring_pinion_ratio, d.ring_gear_pitch_diameter, g.input_pinion_pitch_diameter),
        "  ring pitch-line velocity : %.1f m/s @ max speed" % g.pitch_line_velocity_mps,
        "  half-shaft               : %.0f mm OD%s, %.0f mm long" % (
            h.diameter, (" / %.0f mm bore (hollow)" % h.bore_diameter) if h.bore_diameter else " (solid)", h.length),
        "  half-shaft shear stress  : %.0f MPa  (allow %.0f -> SF %.2f)" % (
            g.halfshaft_shear_stress_mpa, p.material.halfshaft_shear_allow_mpa, g.halfshaft_safety_factor),
        "  CV joints                : %s (in) / %s (out, %.0f deg max)" % (
            h.inboard_joint, h.outboard_joint, h.max_articulation_deg),
        "  wheel mount              : %s" % (
            "centre-lock single nut Ø%.0f" % w.centre_nut_diameter if w.single_centre_nut
            else "%dx PCD %.1f, Ø%.0f studs" % (w.lug_count, w.lug_pcd, w.lug_hole_diameter)),
        "  wheel-hub bearing        : Gen-%d unit, OD %.0f x W %.0f mm" % (
            w.bearing_generation, w.bearing_outer_diameter, w.bearing_width),
        "  modelled track length    : %.0f mm (%d side%s)" % (
            g.total_track_length_mm, g.sides_modelled, "s" if g.sides_modelled > 1 else ""),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
