"""Corner-suspension engineering: wheel rate, ride frequency, roll stiffness,
damping ratio, wheel-hop frequency, and a geometric buildability check. Pure math
-- NX-independent and unit-tested (the analogue of driveline_nx.engineering /
motor_nx.em_design).

First-order closed-form estimates only; the roll-stiffness, damping and wheel-hop
expressions are deliberately ROUGH (labelled below) -- verify ride/handling with a
full multibody (ADAMS/Car) model. Spring + damper rates are referred to the wheel
through the motion ratio.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import List

from .params import SuspensionParams

# representative vertical tyre stiffness used for the wheel-hop estimate (N/mm)
_TYRE_RATE_N_PER_MM = 200.0
# sane ride-frequency band for a passenger EV (Hz); outside => warning
_RIDE_FREQ_MIN_HZ = 0.8
_RIDE_FREQ_MAX_HZ = 2.0


@dataclass
class DerivedSuspension:
    linkage_type: str
    corners_modelled: int
    # spring / ride
    wheel_rate_n_per_mm: float          # spring rate referred to the wheel
    ride_frequency_hz: float            # sprung-mass ride natural frequency
    spring_deflection_mm: float         # static deflection at the ride load
    # roll
    roll_stiffness_nm_per_deg: float    # arb + spring contribution (rough)
    # damping
    damping_ratio: float                # bump damping vs critical (rough)
    # unsprung / wheel hop
    wheel_hop_frequency_hz: float       # unsprung-mass natural frequency (rough)
    # geometry envelope
    corner_envelope_height_mm: float    # representative local-Z build envelope


def derive(p: SuspensionParams) -> DerivedSuspension:
    g, s, d = p.geometry, p.spring, p.damper
    a, m = p.antiroll, p.mass
    n_corners = 2 if p.corners == "axle" else 1

    # wheel rate: spring rate seen at the wheel scales with motion_ratio^2
    wheel_rate = s.spring_rate_n_per_mm * (s.motion_ratio ** 2)   # N/mm
    wheel_rate_n_per_m = wheel_rate * 1.0e3

    # ride (sprung) natural frequency: f = 1/(2pi) * sqrt(k / m)
    sprung = max(1e-6, m.sprung_corner_mass_kg)
    ride_freq = (1.0 / (2.0 * math.pi)) * math.sqrt(wheel_rate_n_per_m / sprung)

    # static spring deflection at the ride-height load
    defl = (s.ride_height_load_n / s.spring_rate_n_per_mm) if s.spring_rate_n_per_mm > 0 else float("inf")

    # roll stiffness (ROUGH, first-order): the anti-roll bar contributes directly,
    # plus the two springs acting across the track resist roll. For a roll angle
    # theta the outer/inner wheels move +/- (track/2)*theta, so the spring-pair roll
    # rate ~ wheel_rate * track^2 / 2 (per radian) -> /(180/pi) per degree.
    track_m = g.track_width_mm * 1.0e-3
    spring_roll_nm_per_rad = wheel_rate_n_per_m * (track_m ** 2) / 2.0
    spring_roll_nm_per_deg = spring_roll_nm_per_rad * math.pi / 180.0
    arb_nm_per_deg = a.rate_nm_per_deg if a.enabled else 0.0
    roll_stiffness = arb_nm_per_deg + spring_roll_nm_per_deg

    # damping ratio (ROUGH): zeta = c / (2 * sqrt(k * m)), using the bump rate.
    crit = 2.0 * math.sqrt(wheel_rate_n_per_m * sprung)
    zeta = (d.bump_rate_ns_per_m / crit) if crit > 0 else float("inf")

    # wheel-hop (unsprung) natural frequency (ROUGH): the tyre and the wheel rate act
    # in series on the unsprung mass.
    k_series = (_TYRE_RATE_N_PER_MM * wheel_rate) / max(1e-9, (_TYRE_RATE_N_PER_MM + wheel_rate))
    unsprung = max(1e-6, m.unsprung_corner_mass_kg)
    hop_freq = (1.0 / (2.0 * math.pi)) * math.sqrt((k_series * 1.0e3) / unsprung)

    # representative local-Z build envelope: knuckle height + spring/damper stack
    envelope = max(p.knuckle.height_mm, s.free_length_mm, d.damper_length_mm) + g.ride_height_mm

    return DerivedSuspension(
        linkage_type=g.type,
        corners_modelled=n_corners,
        wheel_rate_n_per_mm=round(wheel_rate, 3),
        ride_frequency_hz=round(ride_freq, 3),
        spring_deflection_mm=round(defl, 1),
        roll_stiffness_nm_per_deg=round(roll_stiffness, 1),
        damping_ratio=round(zeta, 3),
        wheel_hop_frequency_hz=round(hop_freq, 2),
        corner_envelope_height_mm=round(envelope, 1),
    )


# --------------------------------------------------------------------------- #
# buildability / sanity check (geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: SuspensionParams) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors driveline_nx.engineering.validate()'s contract."""
    issues: List[str] = []
    g, s, d = p.geometry, p.spring, p.damper
    a, k = p.antiroll, p.knuckle
    der = derive(p)

    if g.type not in ("multilink", "double_wishbone", "macpherson"):
        issues.append("geometry.type '%s' unknown (multilink|double_wishbone|macpherson)" % g.type)
    if p.corners not in ("one", "axle"):
        issues.append("corners '%s' unknown (one|axle)" % p.corners)

    # arm lengths must be positive and fit inside the half-track
    half_track = g.track_width_mm / 2.0
    for nm, val in (("lower_arm_length_mm", g.lower_arm_length_mm),
                    ("upper_arm_length_mm", g.upper_arm_length_mm),
                    ("toe_link_length_mm", g.toe_link_length_mm)):
        if val <= 0:
            issues.append("geometry.%s must be > 0" % nm)
        elif val >= half_track:
            issues.append("geometry.%s %.0f must be < track/2 (%.0f mm)" % (nm, val, half_track))

    # spring: static deflection must stay within the free length
    if der.spring_deflection_mm >= s.free_length_mm:
        issues.append("spring static deflection %.0f mm >= free_length %.0f mm (coil binds)"
                      % (der.spring_deflection_mm, s.free_length_mm))
    # motion ratio physically in (0, 1.2]
    if not (0.0 < s.motion_ratio <= 1.2):
        issues.append("spring.motion_ratio %.2f out of range (0, 1.2]" % s.motion_ratio)

    # knuckle hub bore must be a sane bearing OD and fit inside the knuckle body
    if k.hub_bore_diameter_mm <= 0:
        issues.append("knuckle.hub_bore_diameter_mm must be > 0 (match the hub-bearing OD)")
    elif k.hub_bore_diameter_mm >= min(k.height_mm, k.width_mm):
        issues.append("knuckle.hub_bore_diameter_mm too large for the knuckle block")

    # damper body must fit inside a sane corner envelope
    if d.damper_length_mm <= 0:
        issues.append("damper.damper_length_mm must be > 0")
    elif d.damper_length_mm >= 800.0:
        issues.append("damper.damper_length_mm %.0f exceeds the corner envelope (800 mm)" % d.damper_length_mm)

    # anti-roll bar diameter must be positive when the bar is fitted
    if a.enabled and a.bar_diameter_mm <= 0:
        issues.append("antiroll.bar_diameter_mm must be > 0 when the anti-roll bar is enabled")

    # arm wall must leave a bore for a hollow arm
    if p.arm.arm_wall_mm and p.arm.arm_wall_mm * 2.0 >= p.arm.arm_diameter_mm:
        issues.append("arm.arm_wall_mm too thick: leaves no bore in the arm")

    # ride frequency band (warning, but reported)
    if not (_RIDE_FREQ_MIN_HZ <= der.ride_frequency_hz <= _RIDE_FREQ_MAX_HZ):
        issues.append("ride frequency %.2f Hz outside the %.1f-%.1f Hz comfort band"
                      % (der.ride_frequency_hz, _RIDE_FREQ_MIN_HZ, _RIDE_FREQ_MAX_HZ))
    return issues


# --------------------------------------------------------------------------- #
# reports
# --------------------------------------------------------------------------- #
def report(p: SuspensionParams) -> str:
    der = derive(p)
    g, s, d = p.geometry, p.spring, p.damper
    a, k, m = p.antiroll, p.knuckle, p.mass
    issues = validate(p)
    lines = [
        "Suspension corner design summary -- %s" % p.name,
        "  linkage type             : %s  (%d corner%s)" % (
            g.type, der.corners_modelled, "s" if der.corners_modelled > 1 else ""),
        "  track / ride height      : %.0f mm track, %.0f mm ride height" % (
            g.track_width_mm, g.ride_height_mm),
        "  steering-axis geometry   : KPI %.1f deg, caster %.1f deg, camber %.1f deg, scrub %.0f mm" % (
            g.kingpin_inclination_deg, g.caster_deg, g.camber_deg, g.scrub_radius_mm),
        "  arms (lower/upper/toe)   : %.0f / %.0f / %.0f mm" % (
            g.lower_arm_length_mm, g.upper_arm_length_mm, g.toe_link_length_mm),
        "  spring rate / motion rat.: %.1f N/mm, MR %.2f" % (
            s.spring_rate_n_per_mm, s.motion_ratio),
        "  wheel rate               : %.2f N/mm  (= rate x MR^2)" % der.wheel_rate_n_per_mm,
        "  ride frequency           : %.2f Hz  (sprung %.0f kg)" % (
            der.ride_frequency_hz, m.sprung_corner_mass_kg),
        "  static spring deflection : %.0f mm  (load %.0f N, free %.0f mm)" % (
            der.spring_deflection_mm, s.ride_height_load_n, s.free_length_mm),
        "  damper                   : Ø%.0f x %.0f mm%s, bump %.0f / rebound %.0f N.s/m" % (
            d.damper_diameter_mm, d.damper_length_mm, " (adaptive/CDC)" if d.adaptive else "",
            d.bump_rate_ns_per_m, d.rebound_rate_ns_per_m),
        "  damping ratio (bump)     : %.2f  (rough)" % der.damping_ratio,
        "  anti-roll bar            : %s" % (
            "Ø%.0f, %.0f Nm/deg" % (a.bar_diameter_mm, a.rate_nm_per_deg) if a.enabled else "none"),
        "  roll stiffness           : %.0f Nm/deg  (arb + spring, rough)" % der.roll_stiffness_nm_per_deg,
        "  wheel-hop frequency      : %.1f Hz  (unsprung %.0f kg, rough)" % (
            der.wheel_hop_frequency_hz, m.unsprung_corner_mass_kg),
        "  knuckle / hub bore       : %.0f x %.0f x %.0f mm, hub bore Ø%.0f%s" % (
            k.height_mm, k.width_mm, k.thickness_mm, k.hub_bore_diameter_mm,
            " (+caliper mount)" if k.brake_caliper_mount else ""),
        "  corner build envelope    : %.0f mm (local Z)" % der.corner_envelope_height_mm,
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
