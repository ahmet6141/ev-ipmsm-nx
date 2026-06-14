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
    coil_pitch_slots = full_pitch_slots                          # full-pitch assumption
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

    # --- V-magnet pocket fits inside the rotor pole ---------------------- #
    pocket = _magnet_pocket_extent(p, g)
    if pocket is not None:
        max_radius, min_radius, max_half_angle_deg = pocket
        if max_radius > g.rotor_outer_radius - r.outer_bridge + 1e-6:
            issues.append(
                f"V-magnet outer corner reaches r={max_radius:.2f} mm but the rotor "
                f"surface minus outer_bridge is {g.rotor_outer_radius - r.outer_bridge:.2f} mm. "
                f"Reduce magnet_width/magnet_tilt_deg or outer_bridge."
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
    if r.center_post_halfwidth <= 0:
        issues.append("center_post_halfwidth must be > 0 (the d-axis rib).")
    if r.outer_bridge <= 0:
        issues.append("outer_bridge must be > 0.")

    # --- hairpin conductors fit in the slot ------------------------------ #
    usable_h = g.slot_depth - 2 * w.bar_clearance
    bar_h = usable_h / max(1, w.conductors_per_slot)
    if bar_h <= 0:
        issues.append(
            f"conductors_per_slot ({w.conductors_per_slot}) do not fit in the "
            f"{g.slot_depth:.2f} mm slot depth."
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
    return max(radii), min(radii), max(angles)


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
    ]
    problems = validate(p)
    if problems:
        lines.append("  VALIDATION ISSUES:")
        lines.extend(f"    - {msg}" for msg in problems)
    else:
        lines.append("  validation: OK (geometry is buildable)")
    return "\n".join(lines)
