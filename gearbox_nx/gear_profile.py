"""Involute spur-gear TOOTH PROFILE generator (pure math, NX-independent).

Produces the closed 2D outline (a single simple, non-self-intersecting polygon) of a
standard involute gear in the transverse plane, so the NX builder can extrude it into a
REAL toothed gear (kind="extrude"/"prism") instead of a smooth pitch-diameter blank.

Standard (ISO 53 / DIN 867 basic rack, 20deg involute):
    pitch radius      r  = m*z/2
    base radius       rb = r*cos(alpha)
    addendum radius   ra = r + ha*m            (ha = addendum coeff, 1.0)
    root radius       rf = r - hf*m            (hf = dedendum coeff, 1.25)
    involute fn       inv(a) = tan(a) - a
    half tooth angle at the base circle  psi = pi/(2z) + inv(alpha) + x*2*tan(alpha)/z
        (x = profile-shift coefficient; the flank angle at radius rho is
         alpha_rho = acos(rb/rho), and the flank polar angle = psi - inv(alpha_rho),
         so at the pitch circle the half tooth angle is pi/(2z) — a standard tooth.)

The outline is traced CCW with monotonically increasing polar angle around the gear:
per tooth = left flank (root->tip) + tip arc + right flank (tip->root) + root arc to the
next tooth. Flanks below the base circle (when rf < rb, i.e. z < ~41 for x=0) drop
radially to the root as a straight fillet approximation — keeps the loop simple.
"""
from __future__ import annotations

import math
from typing import List, Tuple

Point = Tuple[float, float]


def _inv(a: float) -> float:
    return math.tan(a) - a


def gear_outline(module_mm: float, teeth: int, pressure_angle_deg: float = 20.0,
                 addendum_coeff: float = 1.0, dedendum_coeff: float = 1.25,
                 profile_shift: float = 0.0, flank_pts: int = 4, tip_pts: int = 2,
                 root_pts: int = 2, rotate_deg: float = 0.0) -> List[Point]:
    """Closed CCW outline (list of (x, y), mm) of an involute spur gear centred on the
    origin, tooth 0 centred on +X. `flank_pts` points per involute flank (the involute
    is gentle; 4 reads cleanly). Total vertices ~= teeth * (2*flank_pts + tip_pts + root_pts).

    `rotate_deg` rigidly rotates the whole outline about the origin (CCW, degrees) -- used
    to PHASE a meshing pair: orient the driver's tooth toward the mate along the line of
    centres and the driven gear's GAP toward the driver (a half angular pitch offset), so
    the two solids interlock at the line of centres instead of clashing."""
    if teeth < 5 or module_mm <= 0:
        raise ValueError("teeth>=5 and module>0 required")
    a = math.radians(pressure_angle_deg)
    r = module_mm * teeth / 2.0
    rb = r * math.cos(a)
    ra = r + (addendum_coeff + profile_shift) * module_mm
    rf = r - (dedendum_coeff - profile_shift) * module_mm
    rf = max(rf, 0.2 * module_mm)                    # never collapse to the centre
    # half angular tooth thickness at the base circle (profile shift widens the tooth)
    psi = math.pi / (2.0 * teeth) + _inv(a) + 2.0 * profile_shift * math.tan(a) / teeth
    pitch = 2.0 * math.pi / teeth
    rho_start = max(rb, rf)                           # flank exists only for rho >= rb

    def flank_angle(rho: float) -> float:
        """Polar half-angle of the flank point at radius rho (measured from tooth centre)."""
        ar = math.acos(max(-1.0, min(1.0, rb / rho)))
        return psi - _inv(ar)

    # one tooth's flank radii (root_eff -> tip), shared by both flanks
    radii = [rho_start + (ra - rho_start) * i / flank_pts for i in range(flank_pts + 1)]

    pts: List[Point] = []
    rot = math.radians(rotate_deg)

    def add(rho: float, ang: float):
        pts.append((rho * math.cos(ang + rot), rho * math.sin(ang + rot)))

    for k in range(teeth):
        phi = k * pitch                               # tooth centre angle
        # if the usable flank starts above the root, drop radially to the root first
        if rho_start > rf + 1e-9:
            a_lo = flank_angle(rho_start)
            add(rf, phi - psi)                        # root point (left)
            add(rho_start, phi - a_lo)                # base of left flank
        # left flank: root_eff -> tip (angle increases toward the tooth centre)
        for rho in radii:
            add(rho, phi - flank_angle(rho))
        # tip arc across the top at ra
        a_tip = flank_angle(ra)
        for i in range(1, tip_pts):
            t = i / tip_pts
            add(ra, (phi - a_tip) + (2.0 * a_tip) * t)
        # right flank: tip -> root_eff (angle keeps increasing)
        for rho in reversed(radii):
            add(rho, phi + flank_angle(rho))
        if rho_start > rf + 1e-9:
            add(rf, phi + psi)                        # root point (right)
        # root arc to the next tooth's left root
        next_phi = (k + 1) * pitch
        a0, a1 = phi + psi, next_phi - psi
        for i in range(1, root_pts + 1):
            t = i / (root_pts + 1)
            add(rf, a0 + (a1 - a0) * t)
    return pts


def gear_metrics(module_mm: float, teeth: int, pressure_angle_deg: float = 20.0,
                 addendum_coeff: float = 1.0, dedendum_coeff: float = 1.25,
                 profile_shift: float = 0.0) -> dict:
    """Key radii (mm) of the gear — for placement, bores and clearance checks."""
    a = math.radians(pressure_angle_deg)
    r = module_mm * teeth / 2.0
    return {
        "pitch_radius": r,
        "base_radius": r * math.cos(a),
        "tip_radius": r + (addendum_coeff + profile_shift) * module_mm,
        "root_radius": max(0.2 * module_mm, r - (dedendum_coeff - profile_shift) * module_mm),
    }
