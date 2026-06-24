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

# worst-case single-wheel torque share by differential type (for half-shaft sizing).
#   open  : a passive bevel diff splits 50/50.
#   elsd  : an e-LSD clutch can lock some bias to one wheel but cannot exceed the
#           input axle torque; ~0.6 is a representative transient single-wheel share.
#   torque_vectoring : a twin-clutch ACTIVE eDiff (params.py) routes torque
#           INDEPENDENTLY left/right -- transiently it can put effectively the WHOLE
#           axle torque through one half-shaft (a launch / single-wheel-grip event),
#           i.e. the same worst case as a locked spool. Sizing the fatigue-critical
#           half-shaft for 0.6 understates the duty the chosen diff can produce, so
#           the TV bias is the full 1.0 (the adversarial-review HIGH finding).
#   spool : locked/welded -> the full axle torque can pass through one shaft.
_TORQUE_BIAS = {"open": 0.5, "elsd": 0.6, "torque_vectoring": 1.0, "spool": 1.0}

# floor on the inboard plunge clearance (carrier face -> inboard CV bell): a real
# tripod joint always needs a few mm of axial standoff + plunge travel, so the
# solved gap is never driven below this.
_MIN_INBOARD_CLEARANCE_MM = 5.0


def _fixed_half_chain_mm(p: DrivelineParams) -> float:
    """Per-side built length from the differential CENTRE (z = 0) to the wheel-hub
    flange OUTER face, EXCLUDING the inboard plunge clearance -- i.e. the sum of the
    catalogue component lengths that the ICD ties to the track:

        diff_half + cv_inboard + halfshaft + cv_outboard + hub_bearing + hub_flange

    The inboard clearance (solved separately) is the one slack term that makes the
    total equal target_track/2 without rescaling any real part."""
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    return (d.carrier_length / 2.0
            + h.inboard_bell_length + h.length + h.outboard_bell_length
            + w.bearing_width + w.hub_flange_thickness)


def inboard_clearance(p: DrivelineParams) -> float:
    """Axial gap (mm) placed between the differential carrier face and the inboard
    CV-joint bell so the wheel-hub flange face lands at local z = target_track/2.

    Solved as  clearance = target_track/2 - fixed_half_chain, then clamped to a
    realistic minimum plunge standoff. With the default catalogue dimensions and
    T = 1580 this is ~19 mm (a sensible tripod plunge gap), landing both flange
    faces exactly on the shared HUB_CENTRE. The blueprint reads THIS value (not a
    literal), so the geometry and the engineering track stay in lock-step."""
    return max(_MIN_INBOARD_CLEARANCE_MM,
               p.target_track_mm / 2.0 - _fixed_half_chain_mm(p))


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
    # geometry envelope / ICD track tie (ICD §3, §4)
    inboard_clearance_mm: float     # solved carrier->inboard-CV gap (lands the face on T/2)
    flange_face_z_mm: float         # local |z| of each wheel-hub flange OUTER face
    total_track_length_mm: float    # wheel-flange face to wheel-flange face (both sides)
    track_target_mm: float          # the vehicle track the build is tied to (ICD)
    track_error_pct: float          # 100*(built - target)/target; |.| must be <= 2 %
    sides_modelled: int


def _gearbox_output(p: DrivelineParams):
    """The upstream gearbox OUTPUT torque (Nm) + speed (rpm) the differential is driven
    by, read from gearbox_nx (the reduction lives there, ICD §7.1 / review finding 3).
    NX-free deferred import; returns None if the gearbox package is absent so the
    driveline still derives stand-alone (falling back to the raw motor peak)."""
    try:
        from gearbox_nx.engineering import derive as g_derive
        from gearbox_nx.params import GearboxParams
        g = g_derive(GearboxParams())
        return float(g.output_torque_nm), float(g.output_speed_rpm)
    except Exception:
        return None


def derive(p: DrivelineParams) -> DerivedDriveline:
    d, h, w = p.differential, p.halfshaft, p.wheel_hub
    ratio = d.final_drive_ratio

    # The differential INPUT is the GEARBOX OUTPUT (the reduction is upstream in the
    # gearbox, ICD §7.1): drive the half-shaft sizing from the real ~4.1 kNm gearbox
    # output, not the raw motor peak. Falls back to the motor peak if the gearbox package
    # is unavailable (stand-alone driveline). This removes the old double-count where the
    # diff re-multiplied the motor peak by its own 9:1 (review finding 3).
    gb = _gearbox_output(p)
    input_torque = gb[0] if gb is not None else p.motor_peak_torque_nm
    input_speed_rpm = gb[1] if gb is not None else p.motor_max_speed_rpm
    ring_torque = input_torque * ratio
    n_sides = 2 if p.sides == "both" else 1
    # per-wheel design torque = ring torque x a worst-case bias factor: an open diff
    # splits 50/50; an e-LSD / torque-vectoring eDiff transiently biases more to one
    # wheel; a spool (locked) can put the whole axle torque through one shaft.
    bias = _TORQUE_BIAS.get(d.type, 0.5)
    per_wheel = ring_torque * bias

    # wheel speed = the differential INPUT speed / the diff ratio. The input speed is the
    # GEARBOX OUTPUT speed (already reduced ~9.4:1 upstream); the diff itself is 1:1, so
    # the wheel turns at the gearbox output speed (review finding 3 -- before, this divided
    # the motor speed by the diff's own 9:1 a SECOND time).
    wheel_max_rpm = input_speed_rpm / ratio

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

    # ICD track tie: the inboard plunge clearance is SOLVED so the wheel-hub flange
    # OUTER face lands at local z = target_track/2 (= the shared HUB_CENTRE after the
    # assembly Rx(-90)). half = flange-face |z|; the full built track is the symmetric
    # span across both sides (always 2 x half, even when only one side is modelled --
    # the vehicle has two corners regardless of how many we draw here).
    clearance = inboard_clearance(p)
    half = clearance + _fixed_half_chain_mm(p)             # = flange-face |z|
    total = 2.0 * half                                     # full track both sides span
    target = p.target_track_mm
    err_pct = 100.0 * (total - target) / target if target > 0 else float("inf")

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
        inboard_clearance_mm=round(clearance, 2),
        flange_face_z_mm=round(half, 2),
        total_track_length_mm=round(total, 1),
        track_target_mm=round(target, 1),
        track_error_pct=round(err_pct, 3),
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
    # The differential is a TRUE differential (the reduction is in the gearbox, ICD §7.1):
    # final_drive_ratio is 1.0 for an open diff (no reduction). A ratio < 1 (overdrive) is
    # nonsensical for this driveline; a ratio > 1 means the diff still carries its own
    # reduction (a stand-alone driveline without the gearbox), which is allowed but the
    # vehicle assembly's total-ratio check guards against double-counting it.
    if d.final_drive_ratio < 1.0:
        issues.append("final_drive_ratio %.2f < 1 (the differential cannot overdrive)" % d.final_drive_ratio)

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

    # ICD §4.1 dimensional-consistency tie: the built track (both wheel-hub flange
    # faces) must match the vehicle track within +-2 %, so each flange lands on the
    # shared HUB_CENTRE. The inboard plunge clearance is solved to hit this exactly;
    # it only FAILS here if the catalogue chain is already longer than target_track/2
    # (clearance clamped to its floor) -- i.e. components must shrink, not the gap grow.
    if p.target_track_mm <= 0:
        issues.append("target_track_mm must be > 0 (it is the vehicle track to span)")
    elif abs(g.track_error_pct) > 2.0:
        issues.append(
            "built track %.0f mm is %.1f%% off target %.0f mm (> 2%%): the catalogue "
            "half-chain already exceeds target_track/2 -- shorten halfshaft/bells or "
            "raise target_track_mm" % (g.total_track_length_mm, g.track_error_pct, p.target_track_mm))

    # engineering margins (warnings, not hard stops, but reported). Half-shafts are
    # fatigue-critical, so this static screen carries a >= 1.5 target (not just > yield);
    # the governing fatigue check is separate.
    if g.halfshaft_safety_factor < 1.5:
        issues.append("halfshaft static shear SF %.2f < 1.5 (fatigue-critical part; increase diameter or reduce bore)"
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
        "  diff ratio (open=1:1)    : %.2f : 1  (the ~9.4:1 reduction is in gearbox_nx, ICD §7.1)" % g.final_drive_ratio,
        "  input torque (gearbox out): %.0f Nm @ <= %.0f rpm  (diff input = gearbox output)" % (
            g.input_torque_nm, g.wheel_max_speed_rpm * g.final_drive_ratio),
        "  axle torque (to the diff): %.0f Nm" % g.ring_gear_torque_nm,
        "  per-wheel torque         : %.0f Nm  (%s, bias %.2f)" % (
            g.per_wheel_torque_nm,
            {"open": "open-diff 50/50 split", "spool": "locked -- full axle torque",
             "elsd": "e-LSD worst-case bias", "torque_vectoring": "TV worst-case bias"}.get(d.type, "bias"),
            _TORQUE_BIAS.get(d.type, 0.5)),
        "  wheel max speed          : %.0f rpm  (= gearbox output speed; the diff is 1:1)" % g.wheel_max_speed_rpm,
        "  ring / pinion geometry   : ring PD %.0f / pinion PD %.0f mm (diff bevel set, 1:1 functionally)" % (
            d.ring_gear_pitch_diameter, g.input_pinion_pitch_diameter),
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
        "  modelled side(s)         : %d (chain drawn per side)" % g.sides_modelled,
        "  inboard plunge clearance : %.1f mm (solved to tie the track to the vehicle)" % g.inboard_clearance_mm,
        "  wheel-hub flange face     : local z = +-%.1f mm (each lands on HUB_CENTRE)" % g.flange_face_z_mm,
        "  built track length       : %.0f mm  vs target %.0f mm  (%+.2f%%, ICD +-2%%)" % (
            g.total_track_length_mm, g.track_target_mm, g.track_error_pct),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
