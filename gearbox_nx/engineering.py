"""Gearbox engineering: reduction ratio, the two centre distances, gear pitch-line
velocity, housing wall / oil-sump sizing, mass, and the geometric buildability +
ICD §7.1 connector checks. Pure math -- NX-independent and unit-tested (the analogue
of motor_nx.em_design and driveline_nx.engineering).

The headline job of this module is to PIN the motor<->differential offset so the two
subsystems stop interpenetrating (ICD §7.1): the offset is DERIVED as the sum of the
two gear-stage centre distances, then validated to clear the motor/ring-gear envelopes
(centre distance >= motor_OD/2 + ring_gear_pitch/2 + clearance). It also reconciles the
motor-mounting flange with the real motor DE flange (read from motor_nx) and the output
coupling with the driveline diff input flange (read from driveline_nx).

First-order closed-form estimates; verify gear ratings (ISO 6336) and bearing life
(ISO 281) with detailed tools.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

from .params import GearboxParams

# ICD §7.1 motor<->diff clearance margin (mm) on top of the two envelope radii
_ICD_CLEARANCE_MM = 15.0
# minimum metal a bolt circle / wall must leave (mm)
_WALL_MIN = 1.5


# --------------------------------------------------------------------------- #
# mating-interface dimensions read back from the neighbouring subsystems
# (NX-FREE: these import only the pure-Python params / em_design layers). Deferred
# imports keep gearbox_nx importable even if a sibling package is absent.
# --------------------------------------------------------------------------- #
def motor_de_flange(motor_params=None) -> Dict[str, float]:
    """The motor's DRIVE-END mounting flange the gearbox bolts to, computed from
    motor_nx (params + em_design) EXACTLY as motor_nx.blueprint builds it:

        jacket_inner = stator_outer_radius + cooling.housing_gap         (pilot spigot r)
        jacket_outer = jacket_inner + cooling.jacket_thickness
        flange_outer = jacket_outer + assembly.housing_flange_od_margin  (flange OD/2)
        mount bolt circle pitch r = jacket_outer + 0.72 * od_margin

    Returns a dict of DIAMETERS (mm) + bolt count so the gearbox motor flange matches
    diameter, bolt circle (PCD) and pilot. NX-free."""
    from motor_nx.em_design import derive as m_derive
    from motor_nx.params import MotorParams

    mp = motor_params or MotorParams()
    g = m_derive(mp)
    c, a = mp.cooling, mp.assembly
    jacket_inner = g.stator_outer_radius + c.housing_gap
    jacket_outer = jacket_inner + c.jacket_thickness
    flange_outer = jacket_outer + a.housing_flange_od_margin
    mount_pr = jacket_outer + 0.72 * a.housing_flange_od_margin
    return {
        "flange_diameter_mm": 2.0 * flange_outer,
        "bolt_circle_diameter_mm": 2.0 * mount_pr,
        "bolt_count": int(a.housing_mount_bolt_count),
        "bolt_diameter_mm": float(a.housing_mount_bolt_diameter),
        # the motor DE spigot (pilot) is the jacket bore; the gearbox flange bores a
        # spigot pilot to it (a snug locating fit at the jacket OD register).
        "pilot_diameter_mm": 2.0 * jacket_inner,
        "envelope_radius_mm": float(flange_outer),       # motor housing flange OD/2 (the big envelope)
        "stator_envelope_radius_mm": float(g.stator_outer_radius),  # OD 225 -> r 112.5 (ICD clearance term)
    }


def diff_input_interface(driveline_params=None) -> Dict[str, float]:
    """The differential INPUT flange/pinion the gearbox OUTPUT couples to, read from
    driveline_nx (the gearbox output->diff interface, ICD §7.1). Returns the coupling
    flange diameter / bore / bolt circle + the ring-gear pitch + carrier OD used for
    the clearance and diff-mount checks. NX-free."""
    from driveline_nx.params import DrivelineParams

    dp = driveline_params or DrivelineParams()
    d = dp.differential
    return {
        "input_flange_diameter_mm": float(d.input_flange_diameter),
        "input_bore_diameter_mm": float(d.input_bore_diameter),
        "input_flange_bolt_count": int(d.input_flange_bolt_count),
        "input_flange_bolt_diameter_mm": float(d.input_flange_bolt_diameter),
        "ring_gear_pitch_diameter_mm": float(d.ring_gear_pitch_diameter),
        "carrier_outer_diameter_mm": float(d.carrier_outer_diameter),
        # the diff input pinion is parallel-axis, offset from the wheel axis by the
        # ring+pinion centre distance -- the gearbox output sits coaxial with it.
        "input_pinion_offset_mm": 0.5 * (d.ring_gear_pitch_diameter + d.input_pinion_pitch_diameter),
    }


# --------------------------------------------------------------------------- #
# resolved gearbox geometry (AUTO motor-flange fields filled from motor_nx)
# --------------------------------------------------------------------------- #
def resolve_motor_flange(p: GearboxParams, motor_params=None) -> Dict[str, float]:
    """The motor-mounting flange the gearbox actually builds: each housing.motor_*
    field, with 0.0 meaning AUTO -> take the real motor DE flange value (so the
    gearbox bolts straight onto the motor). A non-zero param overrides."""
    h = p.housing
    m = motor_de_flange(motor_params)
    return {
        "flange_diameter_mm": h.motor_flange_diameter_mm or m["flange_diameter_mm"],
        "thickness_mm": h.motor_flange_thickness_mm,
        "bolt_circle_diameter_mm": h.motor_bolt_circle_diameter_mm or m["bolt_circle_diameter_mm"],
        "bolt_count": h.motor_bolt_count or m["bolt_count"],
        "bolt_diameter_mm": h.motor_bolt_diameter_mm or m["bolt_diameter_mm"],
        "pilot_diameter_mm": h.motor_pilot_diameter_mm or m["pilot_diameter_mm"],
    }


def _stage_pitch(stage) -> Tuple[float, float, float, float]:
    """(pinion_pd, gear_pd, centre_distance, ratio) for a stage from module * teeth."""
    pin = stage.module_mm * stage.pinion_teeth
    gear = stage.module_mm * stage.gear_teeth
    cd = 0.5 * (pin + gear)
    ratio = stage.gear_teeth / stage.pinion_teeth if stage.pinion_teeth > 0 else 0.0
    return pin, gear, cd, ratio


# axis positions (local XY, mm): diff at origin, layshaft and motor along motor_dir.
def axis_positions(p: GearboxParams) -> Dict[str, Tuple[float, float]]:
    """Local-XY centres of the three parallel gear axes. The differential axis is the
    datum (origin). With ``layshaft_collinear`` the layshaft sits on the diff->motor
    line at centre_distance_2 from the diff, so C1 + C2 IS the motor<->diff distance
    (inline reduction). The motor direction (toward the vehicle centre-plane + up,
    ICD §7.1) is ``motor_dir_angle_deg`` in the local XY plane."""
    _, _, c1, _ = _stage_pitch(p.stage1)
    _, _, c2, _ = _stage_pitch(p.stage2)
    a = math.radians(p.motor_dir_angle_deg)
    ux, uy = math.cos(a), math.sin(a)
    diff = (0.0, 0.0)
    if p.layshaft_collinear:
        layshaft = (c2 * ux, c2 * uy)
        motor = ((c1 + c2) * ux, (c1 + c2) * uy)
    else:  # general (non-collinear) placement: layshaft offset perpendicular is out
        layshaft = (c2 * ux, c2 * uy)
        motor = ((c1 + c2) * ux, (c1 + c2) * uy)
    return {"diff": diff, "layshaft": layshaft, "motor": motor}


def axial_bands(p: GearboxParams) -> Dict[str, Tuple[float, float]]:
    """Local-Z (axis-aligned) bands each MESH occupies. An inline layshaft reduction
    separates the two meshes AXIALLY on the layshaft: stage-1 mesh (motor pinion <->
    layshaft gear) sits in band 1, stage-2 mesh (layshaft pinion <-> output gear) in
    band 2, axially offset by the stage-1 face + the inter-gear gap. This is WHY the
    big stage-1 gear and the big output gear -- whose XY pitch circles overlap in an
    inline layout -- never physically collide: they live in disjoint Z bands."""
    fw1 = p.stage1.face_width_mm
    fw2 = p.stage2.face_width_mm
    gap = p.layshaft.inter_gear_gap_mm
    band1 = (0.0, fw1)
    band2 = (fw1 + gap, fw1 + gap + fw2)
    return {"stage1": band1, "stage2": band2}


def _bands_overlap(a: Tuple[float, float], b: Tuple[float, float], tol: float = 1e-6) -> bool:
    return not (a[1] <= b[0] + tol or b[1] <= a[0] + tol)


# segments for the support-function hull sampling (matches blueprint._stadium_hull)
_HULL_SEG = 64


def gear_envelopes(p: GearboxParams, g: "DerivedGearbox", pos: Dict[str, Tuple[float, float]],
                   add: float = 4.0) -> Tuple[List[Tuple[float, float]], List[float]]:
    """(centres, tip radii) of the four gear envelopes in the local XY plane -- the same
    disc set the blueprint hulls for the cast shell footprint (tip = pitch r + addendum).
    Shared by derive() and blueprint.housing_steps so the reported envelope and the built
    shell are bounded by the SAME oval (ICD §7 fidelity, review finding 6)."""
    centres = [pos["diff"], pos["layshaft"], pos["layshaft"], pos["motor"]]
    radii = [g.output_gear_pd_mm / 2.0 + add, g.layshaft_gear_pd_mm / 2.0 + add,
             g.layshaft_pinion_pd_mm / 2.0 + add, g.motor_pinion_pd_mm / 2.0 + add]
    return centres, radii


def _hull_extents(centres: List[Tuple[float, float]], radii: List[float],
                  segs: int = _HULL_SEG) -> Dict[str, float]:
    """Per-direction support extents of a disc set (the stadium/oval hull the cast shell
    follows): the min/max u and v of the support points + the long/short diameters. This
    is the TRUE oval bound, not a single circumscribed radius -- so the reported envelope,
    the oil-sump low point and the mass reflect the real ~379x390 oval, not a phantom
    Ø562 ring (review finding 6)."""
    u_lo = v_lo = math.inf
    u_hi = v_hi = -math.inf
    for i in range(segs):
        th = 2.0 * math.pi * i / segs
        cx, sy = math.cos(th), math.sin(th)
        best = max(c[0] * cx + c[1] * sy + r for c, r in zip(centres, radii))
        x, y = best * cx, best * sy
        u_lo, u_hi = min(u_lo, x), max(u_hi, x)
        v_lo, v_hi = min(v_lo, y), max(v_hi, y)
    return {"u_lo": u_lo, "u_hi": u_hi, "v_lo": v_lo, "v_hi": v_hi,
            "long_mm": max(u_hi - u_lo, v_hi - v_lo),
            "short_mm": min(u_hi - u_lo, v_hi - v_lo)}


@dataclass
class DerivedGearbox:
    # kinematics
    stage1_ratio: float
    stage2_ratio: float
    total_ratio: float
    # centre distances (the connector spine -- they SUM to the motor<->diff offset)
    centre_distance_1_mm: float        # motor pinion <-> layshaft gear
    centre_distance_2_mm: float        # layshaft pinion <-> output gear
    motor_offset_mm: float             # = C1 + C2 (the wheel-axis-to-motor-axis distance)
    motor_offset_dx_mm: float          # local-X component (toward vehicle centre)
    motor_offset_dz_mm: float          # local-Y component (the "up" offset; +Z in vehicle after Rx(-90))
    # gear pitch diameters
    motor_pinion_pd_mm: float
    layshaft_gear_pd_mm: float
    layshaft_pinion_pd_mm: float
    output_gear_pd_mm: float
    # torque / kinematics
    input_torque_nm: float
    output_torque_nm: float
    output_speed_rpm: float
    stage1_pitch_line_velocity_mps: float   # at max input speed
    stage2_pitch_line_velocity_mps: float
    # housing
    housing_inner_radius_mm: float     # required cavity radius to clear the biggest gear + clearance
    housing_envelope_radius_mm: float  # bounding (circumscribed) shell radius -- a worst-case
    #                                    single number; the real shell is the oval below
    housing_envelope_long_mm: float    # the cast shell oval LONG outer diameter (true bound)
    housing_envelope_short_mm: float   # the cast shell oval SHORT outer diameter (true bound)
    housing_sump_v_low_mm: float       # local v (XY) of the shell inner-wall LOW point (sump seats here)
    housing_axial_length_mm: float
    gear_train_axial_length_mm: float  # stage-1 face -> stage-2 far face (both meshes)
    oil_volume_l: float
    mass_estimate_kg: float
    # ICD §7.1 clearance
    icd_min_centre_distance_mm: float  # motor_OD/2 + ring/2 + clearance
    centre_distance_margin_mm: float   # motor_offset - icd_min (must be >= 0)


def derive(p: GearboxParams, motor_params=None, driveline_params=None) -> DerivedGearbox:
    p1_pd, g1_pd, c1, r1 = _stage_pitch(p.stage1)
    p2_pd, g2_pd, c2, r2 = _stage_pitch(p.stage2)
    total_ratio = r1 * r2

    a = math.radians(p.motor_dir_angle_deg)
    motor_offset = c1 + c2
    dx = motor_offset * math.cos(a)
    dz = motor_offset * math.sin(a)

    input_torque = p.motor_peak_torque_nm
    output_torque = input_torque * total_ratio
    output_speed = p.motor_max_speed_rpm / total_ratio if total_ratio > 0 else 0.0

    # pitch-line velocity v = pi * d * n / 60 (d in m, n in rev/s)
    n_in = p.motor_max_speed_rpm / 60.0
    n_lay = n_in / r1 if r1 > 0 else 0.0
    plv1 = math.pi * (p1_pd * 1e-3) * n_in
    plv2 = math.pi * (p2_pd * 1e-3) * n_lay

    # housing: the biggest rotating envelope is the output gear (g2_pd) on the diff
    # axis and the stage-1 gear (g1_pd) on the layshaft. The cavity must clear the
    # FARTHEST gear tip from the diff axis (the layshaft gear reaches its centre
    # distance from the diff plus its own tip radius) -- size the envelope to that.
    pos = axis_positions(p)
    h = p.housing
    tips = [
        ("diff_ring", 0.0, max(g2_pd, _ring_pd(driveline_params)) / 2.0),
        ("layshaft_gear", _dist(pos["layshaft"]), g1_pd / 2.0),
        ("motor_pinion", _dist(pos["motor"]), p1_pd / 2.0),
    ]
    reach = max(d + r for (_n, d, r) in tips)             # farthest gear tip from the diff axis
    housing_inner_r = reach + h.radial_clearance_mm
    housing_env_r = housing_inner_r + h.wall_thickness_mm
    # the gear train spans both axially-separated meshes (stage-1 band -> stage-2 band)
    bands = axial_bands(p)
    train_axial = max(bands["stage1"][1], bands["stage2"][1])
    housing_axial = max(h.axial_length_mm, train_axial)

    # The cast shell footprint is the OVAL (stadium) hull of the gear envelopes, NOT a
    # single circumscribed circle. Bound the housing by the per-direction hull extents so
    # the reported envelope, the oil sump and the mass reflect the real ~379x390 oval (the
    # single radius over-bounds the short direction by ~40-60 %, review finding 6).
    add = 4.0   # representative tip addendum beyond pitch radius (matches blueprint)
    centres = [pos["diff"], pos["layshaft"], pos["layshaft"], pos["motor"]]
    radii = [g2_pd / 2.0 + add, g1_pd / 2.0 + add, p2_pd / 2.0 + add, p1_pd / 2.0 + add]
    outer_ext = _hull_extents(centres, [r + h.radial_clearance_mm + h.wall_thickness_mm
                                        for r in radii])
    inner_ext = _hull_extents(centres, [r + h.radial_clearance_mm for r in radii])
    env_long = outer_ext["long_mm"]
    env_short = outer_ext["short_mm"]
    sump_v_low = inner_ext["v_lo"]            # the shell inner-wall LOW point (sump seats here)

    # oil sump volume: a representative box sump along the LOW inner wall of the OVAL
    # cavity (not a 2*env_r-wide slab). Width = the cavity span at the low side ~ the
    # inner short diameter; depth x axial length, filled to oil_fill_fraction.
    sump_width = inner_ext["u_hi"] - inner_ext["u_lo"]
    sump_vol_mm3 = sump_width * h.oil_sump_depth_mm * housing_axial
    oil_l = sump_vol_mm3 * h.oil_fill_fraction * 1e-6     # mm^3 -> litres (1e-6 L/mm^3)

    mass = _mass_estimate(p, g1_pd, g2_pd, p1_pd, p2_pd, env_long, env_short, housing_axial,
                          driveline_params)

    icd_min = _icd_min_centre_distance(motor_params, driveline_params)

    return DerivedGearbox(
        stage1_ratio=round(r1, 4),
        stage2_ratio=round(r2, 4),
        total_ratio=round(total_ratio, 4),
        centre_distance_1_mm=round(c1, 3),
        centre_distance_2_mm=round(c2, 3),
        motor_offset_mm=round(motor_offset, 3),
        motor_offset_dx_mm=round(dx, 3),
        motor_offset_dz_mm=round(dz, 3),
        motor_pinion_pd_mm=round(p1_pd, 3),
        layshaft_gear_pd_mm=round(g1_pd, 3),
        layshaft_pinion_pd_mm=round(p2_pd, 3),
        output_gear_pd_mm=round(g2_pd, 3),
        input_torque_nm=round(input_torque, 2),
        output_torque_nm=round(output_torque, 1),
        output_speed_rpm=round(output_speed, 1),
        stage1_pitch_line_velocity_mps=round(plv1, 2),
        stage2_pitch_line_velocity_mps=round(plv2, 2),
        housing_inner_radius_mm=round(housing_inner_r, 2),
        housing_envelope_radius_mm=round(housing_env_r, 2),
        housing_envelope_long_mm=round(env_long, 2),
        housing_envelope_short_mm=round(env_short, 2),
        housing_sump_v_low_mm=round(sump_v_low, 2),
        housing_axial_length_mm=round(housing_axial, 2),
        gear_train_axial_length_mm=round(train_axial, 2),
        oil_volume_l=round(oil_l, 3),
        mass_estimate_kg=round(mass, 2),
        icd_min_centre_distance_mm=round(icd_min, 2),
        centre_distance_margin_mm=round(motor_offset - icd_min, 2),
    )


def _dist(xy: Tuple[float, float]) -> float:
    return math.hypot(xy[0], xy[1])


def _ring_pd(driveline_params=None) -> float:
    try:
        return diff_input_interface(driveline_params)["ring_gear_pitch_diameter_mm"]
    except Exception:
        return 208.0


def _icd_min_centre_distance(motor_params=None, driveline_params=None) -> float:
    """ICD §7.1 clearance rule: motor-to-diff centre distance must be at least
    motor_OD/2 + ring_gear_pitch/2 + 15 mm. motor_OD/2 = the motor stator envelope
    radius (OD 225 -> 112.5); ring_gear_pitch/2 from the driveline (Ø208 -> 104)."""
    try:
        m = motor_de_flange(motor_params)
        motor_r = m["stator_envelope_radius_mm"]
    except Exception:
        motor_r = 112.5
    ring_r = _ring_pd(driveline_params) / 2.0
    return motor_r + ring_r + _ICD_CLEARANCE_MM


def _mass_estimate(p: GearboxParams, g1_pd, g2_pd, p1_pd, p2_pd,
                   env_long, env_short, housing_axial, driveline_params=None) -> float:
    """First-order mass (kg): the cast-aluminium shell (a thin-walled OVAL of
    wall_thickness over the axial length, plus two end covers) + the steel gear blanks
    + the layshaft. The shell is bounded by the real ~379x390 oval (long/short diameters),
    not a phantom Ø562 circumscribed ring (review finding 6). Order-of-magnitude only."""
    m = p.material
    h = p.housing
    rho_al = m.housing_density
    rho_st = m.gear_steel_density
    # shell: an oval ring (ellipse semi-axes a, b outer; a-wall, b-wall inner) x axial.
    # A_ellipse = pi*a*b; the wall ring area is the outer minus the inner ellipse.
    a_o, b_o = env_long / 2.0, env_short / 2.0
    a_i, b_i = max(0.0, a_o - h.wall_thickness_mm), max(0.0, b_o - h.wall_thickness_mm)
    ring_area = math.pi * (a_o * b_o - a_i * b_i)                     # mm^2
    shell_v = ring_area * housing_axial * 1e-9                        # mm^3 -> m^3
    cover_v = 2.0 * math.pi * a_o * b_o * h.end_cover_thickness_mm * 1e-9
    housing_kg = (shell_v + cover_v) * rho_al
    # gear blanks (solid discs) + layshaft (solid bar)
    def disc(pd, fw):
        return math.pi * (pd / 2.0) ** 2 * fw * 1e-9
    gears_v = (disc(p1_pd, p.stage1.face_width_mm) + disc(g1_pd, p.stage1.face_width_mm)
               + disc(p2_pd, p.stage2.face_width_mm) + disc(g2_pd, p.stage2.face_width_mm))
    lay_len = (p.stage1.face_width_mm + p.stage2.face_width_mm
               + p.layshaft.inter_gear_gap_mm + 2.0 * p.layshaft.bearing_seat_length_mm)
    lay_v = math.pi * (p.layshaft.shaft_diameter_mm / 2.0) ** 2 * lay_len * 1e-9
    gears_kg = (gears_v + lay_v) * rho_st
    return housing_kg + gears_kg


# --------------------------------------------------------------------------- #
# buildability + ICD §7.1 connector checks
# --------------------------------------------------------------------------- #
def validate(p: GearboxParams, motor_params=None, driveline_params=None) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors motor_nx.em_design.validate() / driveline_nx.engineering.validate()."""
    issues: List[str] = []
    g = derive(p, motor_params, driveline_params)

    # --- gear tooth counts / modules ------------------------------------- #
    for tag, st in (("stage1", p.stage1), ("stage2", p.stage2)):
        if st.pinion_teeth < 10:
            issues.append("%s.pinion_teeth %d < 10 (undercut / weak pinion)" % (tag, st.pinion_teeth))
        if st.gear_teeth <= st.pinion_teeth:
            issues.append("%s.gear_teeth must exceed pinion_teeth for a reduction" % tag)
        if st.module_mm <= 0:
            issues.append("%s.module_mm must be > 0" % tag)
        if st.face_width_mm <= 0:
            issues.append("%s.face_width_mm must be > 0" % tag)

    # --- total ratio in the EV reduction band ---------------------------- #
    if g.total_ratio <= 1.0:
        issues.append("total_ratio %.2f must be > 1 for a reduction" % g.total_ratio)
    elif not (8.0 <= g.total_ratio <= 11.0):
        issues.append("total_ratio %.2f is outside the ~9-10:1 single-speed EV band "
                      "(8-11 tolerated); retooth stage1/stage2" % g.total_ratio)

    # --- ICD §7.1 clearance: the centre distances must reach far enough that the
    #     motor and the diff ring gear do NOT interpenetrate ----------------- #
    if g.centre_distance_margin_mm < 0:
        issues.append(
            "motor<->diff offset %.1f mm (= C1 %.1f + C2 %.1f) is below the ICD §7.1 "
            "minimum %.1f mm (motor_OD/2 + ring/2 + %.0f): the motor and differential "
            "would interpenetrate -- raise a stage centre distance (module / teeth)"
            % (g.motor_offset_mm, g.centre_distance_1_mm, g.centre_distance_2_mm,
               g.icd_min_centre_distance_mm, _ICD_CLEARANCE_MM))

    # --- the two centre distances must SUM to the placed motor offset ------ #
    # (this is true by construction in derive(); assert it so a future refactor that
    # breaks the tie is caught here rather than silently letting the train miss.)
    if abs((g.centre_distance_1_mm + g.centre_distance_2_mm) - g.motor_offset_mm) > 1e-6:
        issues.append("centre distances %.2f + %.2f do not sum to motor_offset %.2f"
                      % (g.centre_distance_1_mm, g.centre_distance_2_mm, g.motor_offset_mm))

    # --- layshaft gear axial layout -------------------------------------- #
    if p.layshaft.inter_gear_gap_mm < 0:
        issues.append("layshaft.inter_gear_gap_mm < 0: the stage-1 gear and stage-2 "
                      "pinion axially overlap on the layshaft (and the big gears collide)")

    # --- gears clear the housing inner wall ------------------------------- #
    h = p.housing
    if h.wall_thickness_mm <= 0:
        issues.append("housing.wall_thickness_mm must be > 0")
    if h.radial_clearance_mm <= 0:
        issues.append("housing.radial_clearance_mm must be > 0 (gear tip -> wall gap)")
    if g.housing_inner_radius_mm <= 0:
        issues.append("housing inner radius collapses to <= 0 (gears do not fit)")
    # the housing cavity must hold the whole gear train (both axially-separated meshes)
    if h.axial_length_mm < g.gear_train_axial_length_mm - 1e-6:
        issues.append("housing.axial_length_mm %.0f < gear-train axial length %.0f mm "
                      "(stage-1 + gap + stage-2 faces)"
                      % (h.axial_length_mm, g.gear_train_axial_length_mm))

    # --- non-meshing gears must not interfere ----------------------------- #
    # In an inline layshaft reduction the big stage-1 gear (layshaft) and the big output
    # gear (diff) have OVERLAPPING xy pitch circles -- that is expected and harmless
    # BECAUSE the two meshes sit in DISJOINT axial bands on the layshaft (stage-1 mesh in
    # band 1, stage-2 mesh in band 2). A clash is real only when two gears on DIFFERENT
    # axes share an axial band AND their pitch circles overlap beyond a proper mesh.
    bands = axial_bands(p)
    pos = axis_positions(p)
    # the only same-band non-meshing pair to police is within a band where a third
    # gear could intrude. Check every non-meshing gear pair that shares an axial band.
    gears = [  # (name, axis_xy, tip_radius, band)
        ("motor_pinion", pos["motor"], g.motor_pinion_pd_mm / 2.0, bands["stage1"]),
        ("layshaft_gear", pos["layshaft"], g.layshaft_gear_pd_mm / 2.0, bands["stage1"]),
        ("layshaft_pinion", pos["layshaft"], g.layshaft_pinion_pd_mm / 2.0, bands["stage2"]),
        ("output_gear", pos["diff"], g.output_gear_pd_mm / 2.0, bands["stage2"]),
    ]
    mesh_pairs = {("motor_pinion", "layshaft_gear"), ("layshaft_pinion", "output_gear")}
    for i in range(len(gears)):
        for j in range(i + 1, len(gears)):
            ni, (xi, yi), ri, bi = gears[i]
            nj, (xj, yj), rj, bj = gears[j]
            if (ni, nj) in mesh_pairs or (nj, ni) in mesh_pairs:
                continue                                   # a proper mesh -- circles touch by design
            if (xi, yi) == (xj, yj):
                continue                                   # coaxial (same shaft, axially stacked)
            if not _bands_overlap(bi, bj):
                continue                                   # axially disjoint -> cannot collide
            sep = math.hypot(xi - xj, yi - yj)
            clash = ri + rj - sep
            if clash > _WALL_MIN:
                issues.append(
                    "%s (r %.0f) and %s (r %.0f) overlap by %.1f mm and share an axial band "
                    "-- they collide; retooth a stage or add inter-gear axial gap"
                    % (ni, ri, nj, rj, clash))

    # --- motor-mounting flange matches the real motor DE flange ------------ #
    try:
        mf = resolve_motor_flange(p, motor_params)
        motor = motor_de_flange(motor_params)
        if abs(mf["flange_diameter_mm"] - motor["flange_diameter_mm"]) > 1.0:
            issues.append(
                "gearbox motor flange Ø%.0f != motor DE flange Ø%.0f (it will not bolt on); "
                "set housing.motor_flange_diameter_mm = 0 for AUTO"
                % (mf["flange_diameter_mm"], motor["flange_diameter_mm"]))
        if abs(mf["bolt_circle_diameter_mm"] - motor["bolt_circle_diameter_mm"]) > 1.0:
            issues.append(
                "gearbox motor bolt circle Ø%.0f != motor DE bolt circle Ø%.0f"
                % (mf["bolt_circle_diameter_mm"], motor["bolt_circle_diameter_mm"]))
        if mf["bolt_count"] != motor["bolt_count"]:
            issues.append("gearbox motor bolt count %d != motor DE bolt count %d"
                          % (mf["bolt_count"], motor["bolt_count"]))
        # bolt circle must sit inside the flange OD with wall
        if mf["bolt_circle_diameter_mm"] / 2.0 + mf["bolt_diameter_mm"] / 2.0 > mf["flange_diameter_mm"] / 2.0 - _WALL_MIN:
            issues.append("motor bolt circle runs off the motor-mounting flange OD")
    except Exception as exc:                                       # pragma: no cover
        issues.append("could not read the motor DE flange from motor_nx: %s" % exc)

    # --- output coupling matches the driveline diff input flange ----------- #
    try:
        di = diff_input_interface(driveline_params)
        o = p.output
        if abs(o.flange_diameter_mm - di["input_flange_diameter_mm"]) > 1.0:
            issues.append(
                "gearbox output flange Ø%.0f != driveline diff input flange Ø%.0f "
                "(the output->diff coupling will not mate)"
                % (o.flange_diameter_mm, di["input_flange_diameter_mm"]))
        if abs(o.bore_diameter_mm - di["input_bore_diameter_mm"]) > 1.0:
            issues.append("gearbox output bore Ø%.0f != driveline input bore Ø%.0f"
                          % (o.bore_diameter_mm, di["input_bore_diameter_mm"]))
        # the diff carrier must fit the diff-mount bore
        if h.diff_carrier_diameter_mm < di["carrier_outer_diameter_mm"] - 1e-6:
            issues.append(
                "housing.diff_carrier_diameter_mm %.0f < driveline carrier OD %.0f "
                "(the carrier will not seat)" % (h.diff_carrier_diameter_mm, di["carrier_outer_diameter_mm"]))
        if h.diff_mount_flange_diameter_mm <= h.diff_carrier_diameter_mm:
            issues.append("diff_mount_flange_diameter must exceed the carrier diameter")
    except Exception as exc:                                       # pragma: no cover
        issues.append("could not read the diff input interface from driveline_nx: %s" % exc)

    # --- oil sump sanity --------------------------------------------------- #
    if not (0.0 < h.oil_fill_fraction < 1.0):
        issues.append("housing.oil_fill_fraction must be in (0, 1)")
    if h.oil_sump_depth_mm <= 0:
        issues.append("housing.oil_sump_depth_mm must be > 0")

    # --- pitch-line velocity warning (helical EV gears run high; flag extreme) #
    if g.stage1_pitch_line_velocity_mps > 60.0:
        issues.append("stage-1 pitch-line velocity %.0f m/s is very high (> 60 m/s); "
                      "raise pinion teeth / module or cap input speed" % g.stage1_pitch_line_velocity_mps)
    return issues


# --------------------------------------------------------------------------- #
# report
# --------------------------------------------------------------------------- #
def report(p: GearboxParams, motor_params=None, driveline_params=None) -> str:
    g = derive(p, motor_params, driveline_params)
    issues = validate(p, motor_params, driveline_params)
    mf = resolve_motor_flange(p, motor_params)
    lines = [
        "Reduction gearbox design summary -- %s" % p.name,
        "  layout                   : parallel-axis 2-stage (motor -> layshaft -> diff)",
        "  stage 1 ratio            : %.3f:1  (z %d/%d, m %.2f, PD %.0f/%.0f mm)" % (
            g.stage1_ratio, p.stage1.pinion_teeth, p.stage1.gear_teeth, p.stage1.module_mm,
            g.motor_pinion_pd_mm, g.layshaft_gear_pd_mm),
        "  stage 2 ratio            : %.3f:1  (z %d/%d, m %.2f, PD %.0f/%.0f mm)" % (
            g.stage2_ratio, p.stage2.pinion_teeth, p.stage2.gear_teeth, p.stage2.module_mm,
            g.layshaft_pinion_pd_mm, g.output_gear_pd_mm),
        "  TOTAL ratio              : %.3f:1" % g.total_ratio,
        "  centre distance 1 / 2    : %.1f / %.1f mm  (sum = motor offset %.1f mm)" % (
            g.centre_distance_1_mm, g.centre_distance_2_mm, g.motor_offset_mm),
        "  motor offset (dx, dz)    : (%.0f, %.0f) mm @ %.1f deg from the diff axis" % (
            g.motor_offset_dx_mm, g.motor_offset_dz_mm, p.motor_dir_angle_deg),
        "  ICD §7.1 clearance       : offset %.1f mm vs min %.1f mm -> margin %.1f mm %s" % (
            g.motor_offset_mm, g.icd_min_centre_distance_mm, g.centre_distance_margin_mm,
            "(CLEARS)" if g.centre_distance_margin_mm >= 0 else "(OVERLAP!)"),
        "  input / output torque    : %.0f -> %.0f Nm" % (g.input_torque_nm, g.output_torque_nm),
        "  output speed @ max        : %.0f rpm" % g.output_speed_rpm,
        "  pitch-line vel (s1/s2)   : %.1f / %.1f m/s @ max input" % (
            g.stage1_pitch_line_velocity_mps, g.stage2_pitch_line_velocity_mps),
        "  housing oval (long x short): %.0f x %.0f mm (env r %.0f circ.), axial %.0f mm" % (
            g.housing_envelope_long_mm, g.housing_envelope_short_mm,
            g.housing_envelope_radius_mm, g.housing_axial_length_mm),
        "  oil volume (splash)      : %.2f L (%.0f%% of sump)" % (
            g.oil_volume_l, 100.0 * p.housing.oil_fill_fraction),
        "  mass estimate            : %.1f kg (cast-Al shell + steel gears)" % g.mass_estimate_kg,
        "  motor mount flange       : Ø%.0f, %d x Ø%.1f on PCD %.0f, pilot Ø%.0f (matches motor DE)" % (
            mf["flange_diameter_mm"], mf["bolt_count"], mf["bolt_diameter_mm"],
            mf["bolt_circle_diameter_mm"], mf["pilot_diameter_mm"]),
        "  diff carrier mount       : Ø%.0f bore, %d x Ø%.1f bolts on Ø%.0f flange" % (
            p.housing.diff_carrier_diameter_mm, p.housing.diff_mount_bolt_count,
            p.housing.diff_mount_bolt_diameter_mm, p.housing.diff_mount_flange_diameter_mm),
        "  validation: %s" % ("OK (geometry is buildable)" if not issues else "%d issue(s)" % len(issues)),
    ]
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
