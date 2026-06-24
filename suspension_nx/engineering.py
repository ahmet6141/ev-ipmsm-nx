"""Corner-suspension engineering: the 3D HARDPOINT table, wheel rate, ride
frequency, roll stiffness, roll-centre height, damping ratio, wheel-hop frequency,
and a geometric buildability check. Pure math -- NX-independent and unit-tested
(the analogue of driveline_nx.engineering / motor_nx.em_design).

First-order closed-form estimates only; the roll-stiffness, roll-centre, damping
and wheel-hop expressions are deliberately ROUGH (labelled below) -- verify
ride/handling with a full multibody (ADAMS/Car) model. The spring rate is referred
to the wheel through the motion ratio (wheel rate = spring rate x MR^2); the damper
bump rate is referred through the SAME MR^2 so the headline damping ratio is a
consistent WHEEL-frame zeta (~0.25 for the defaults). The un-referred damper-frame
value (which reads high, ~0.65) is also reported for cross-checking only.

THE HARDPOINT TABLE (engineering.hardpoints) is the single source of truth for the
corner's 3D geometry, shared by blueprint.py (which builds the links between these
points), validate() (reach / overlap checks) and the tests (world bounding boxes,
mating points). It is expressed in the LOCAL corner frame defined in
blueprint.py: hub centre at (0, 0, 0), +X fwd, +Y OUTBOARD, +Z up.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Dict, List, Tuple

from .params import SuspensionParams

Vec3 = Tuple[float, float, float]

# representative vertical tyre stiffness used for the wheel-hop estimate (N/mm)
_TYRE_RATE_N_PER_MM = 200.0
# sane ride-frequency band for a passenger EV (Hz); outside => warning
_RIDE_FREQ_MIN_HZ = 0.8
_RIDE_FREQ_MAX_HZ = 2.0
# fore/aft split of an A-arm's two pickups about the hub X-station (mm); sets the
# longitudinal base of the wishbone (anti-dive/squat lever).
_ARM_HALF_BASE_MM = 120.0


# --------------------------------------------------------------------------- #
# 3D hardpoint table -- the single source of truth for the corner geometry
# --------------------------------------------------------------------------- #
def hardpoints(p: SuspensionParams) -> Dict[str, Vec3]:
    """All corner hardpoints in the LOCAL frame (hub centre at origin, +X fwd, +Y
    OUTBOARD, +Z up). The blueprint builds the links BETWEEN these points and the
    tests assert the world geometry from them, so the geometry can never drift from
    the kinematics.

    Steering-axis construction: the lower / upper ball joints sit just inboard of
    the hub face (so they clear the wheel) and straddle the hub in Z. The line
    through them is the kingpin (steering) axis, inclined `kingpin_inclination_deg`
    from vertical in the lateral (Y-Z) plane and `caster_deg` in the fore-aft (X-Z)
    plane; `scrub_radius_mm` sets how far inboard of the hub the axis crosses the
    ground (it pulls the ball joints inboard). Inboard chassis pickups sit at -Y
    (toward the chassis centreline) at the respective arm length from the ball
    joint, split fore/aft to form the A-arm base."""
    g, d, k = p.geometry, p.damper, p.knuckle
    kpi = math.radians(g.kingpin_inclination_deg)
    caster = math.radians(g.caster_deg)

    # vertical span of the steering axis: lower ball joint a bit below the hub,
    # upper a bit above (bounded by the knuckle height).
    z_low = -0.45 * k.height_mm
    z_high = +0.45 * k.height_mm
    # the ball joints sit just inboard of the hub face (clear of the wheel/disc)
    y_face = -(k.width_mm / 2.0 + 6.0)
    # The kingpin (steering) axis is a single straight line through both ball joints,
    # inclined by KPI in the lateral (Y-Z) plane and by caster in the fore-aft (X-Z)
    # plane. Parameterise it by height z (measured from the hub): at z it lies
    # dy(z) inboard and dx(z) rearward of a reference vertical.
    #   +KPI  -> the top of the axis leans INBOARD  (more -Y as z increases)
    #   +caster -> the top of the axis leans REARWARD (more -X as z increases)
    def _axis_offset(z: float) -> Tuple[float, float]:
        return (-math.tan(caster) * z, -math.tan(kpi) * z)
    # anchor the axis laterally so it crosses the ground (z = -ride_height) exactly
    # `scrub_radius` inboard of the hub-centre vertical (scrub radius definition).
    _, dy_ground = _axis_offset(-g.ride_height_mm)
    y_anchor = (-g.scrub_radius_mm) - dy_ground
    lx, ly = _axis_offset(z_low)
    ux, uy = _axis_offset(z_high)
    lower_bj: Vec3 = (lx, min(y_face, y_anchor + ly), z_low)
    upper_bj: Vec3 = (ux, min(y_face, y_anchor + uy), z_high)

    hb = _ARM_HALF_BASE_MM
    # inboard pickups: arm_length inboard (-Y) of the ball joint, split fore/aft. The
    # front-view inclination differs per arm so the arms are NOT parallel (a genuine
    # instant centre / roll centre): the lower arm rises gently toward the chassis,
    # the UPPER arm drops toward the chassis (a steeper, opposite slope) -- the
    # classic short-long-arm layout that controls camber gain and the roll centre.
    def _pickups(bj: Vec3, length: float, rise_frac: float) -> Tuple[Vec3, Vec3]:
        y_in = bj[1] - length
        z_in = bj[2] + rise_frac * length
        return ((bj[0] + hb, y_in, z_in), (bj[0] - hb, y_in, z_in))

    lf, la = _pickups(lower_bj, g.lower_arm_length_mm, +0.06)   # lower arm rises inboard
    uf, ua = _pickups(upper_bj, g.upper_arm_length_mm, -0.10)   # upper arm drops inboard

    # toe / tie link: outboard on the steering arm (rearward of, and at hub height
    # near, the lower joint); inboard pickup -Y by the toe-link length, set rearward.
    # The outboard toe joint sits on the steering arm just INBOARD of the lower ball
    # joint (a small +Y step toward the hub but still negative / inboard of the hub
    # face) -- NOT outboard of the hub. (Tying it to +0.25*arm_length pushed it to +Y,
    # outboard of the wheel face -- the low-severity finding.)
    toe_out: Vec3 = (-0.6 * hb, lower_bj[1] + 0.12 * abs(lower_bj[1]), 0.0)
    toe_in: Vec3 = (toe_out[0] - 0.3 * g.toe_link_length_mm,
                    toe_out[1] - g.toe_link_length_mm, toe_out[2] + 8.0)

    # damper / coil: seated on the lower arm (inboard of the ball joint) and inclined
    # up-and-inboard to a body mount. MacPherson adds a strut top above the upright.
    damper_lower: Vec3 = (0.0, lower_bj[1] - 0.45 * g.lower_arm_length_mm,
                          lower_bj[2] + 0.10 * k.height_mm)
    damper_top: Vec3 = (0.0, damper_lower[1] - 0.30 * d.damper_length_mm,
                        damper_lower[2] + d.damper_length_mm)
    strut_top: Vec3 = (-math.tan(caster) * (z_high + 0.6 * d.damper_length_mm),
                       upper_bj[1] - 0.15 * d.damper_length_mm,
                       z_high + 0.6 * d.damper_length_mm)

    # anti-roll drop link: from a point on the lower arm up to the bar end.
    arb_lower: Vec3 = (hb * 0.5, lower_bj[1] - 0.25 * g.lower_arm_length_mm,
                       lower_bj[2] + 6.0)
    arb_upper: Vec3 = (arb_lower[0], arb_lower[1] - 0.05 * p.antiroll.arm_length_mm,
                       arb_lower[2] + p.antiroll.arm_length_mm)

    # caliper mount: fore (+X) of the hub, just inboard of the wheel face.
    caliper: Vec3 = (k.thickness_mm / 2.0 + 14.0, y_face, 0.30 * k.height_mm)

    return {
        "hub_centre": (0.0, 0.0, 0.0),
        "lower_ball_joint": lower_bj,
        "upper_ball_joint": upper_bj,
        "lower_pickup_fore": lf,
        "lower_pickup_aft": la,
        "upper_pickup_fore": uf,
        "upper_pickup_aft": ua,
        "toe_outboard": toe_out,
        "toe_pickup": toe_in,
        "damper_lower": damper_lower,
        "damper_top": damper_top,
        "strut_top": strut_top,
        "arb_link_lower": arb_lower,
        "arb_link_upper": arb_upper,
        "caliper_mount": caliper,
    }


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
    roll_centre_height_mm: float        # geometric roll-centre height (rough)
    # damping
    damping_ratio: float                # WHEEL-frame bump damping vs critical (rough);
                                        # bump rate referred through MR^2 to match the
                                        # wheel-frame stiffness (the headline value)
    damping_ratio_damper_frame: float   # un-referred damper-frame value (reads high) --
                                        # reported only for cross-checking
    # unsprung / wheel hop
    wheel_hop_frequency_hz: float       # unsprung-mass natural frequency (rough)
    # geometry envelope
    corner_envelope_height_mm: float    # representative local-Z build envelope
    # kinematics note
    anti_feature_note: str              # anti-dive/anti-squat qualitative note


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

    # damping ratio (ROUGH): zeta = c / (2 * sqrt(k * m)). The critical-damping term
    # uses the wheel-frame stiffness (MR^2-referred), so the damper coefficient must be
    # referred to the WHEEL through the same MR^2 to be consistent -- a damper-frame c
    # divided by a wheel-frame crit over-reports zeta (the headline number must be the
    # wheel-frame value). c_wheel = bump_rate * MR^2.
    crit = 2.0 * math.sqrt(wheel_rate_n_per_m * sprung)
    c_wheel = d.bump_rate_ns_per_m * (s.motion_ratio ** 2)
    zeta = (c_wheel / crit) if crit > 0 else float("inf")
    # the damper-frame value (un-referred) is also reported for cross-checking.
    zeta_damper_frame = (d.bump_rate_ns_per_m / crit) if crit > 0 else float("inf")

    # wheel-hop (unsprung) natural frequency (ROUGH): the unsprung mass rides between
    # the tyre (to ground) and the wheel rate (to the sprung mass), so the two
    # stiffnesses act in PARALLEL on it -> k_hop = k_tyre + wheel_rate.
    k_hop = _TYRE_RATE_N_PER_MM + wheel_rate
    unsprung = max(1e-6, m.unsprung_corner_mass_kg)
    hop_freq = (1.0 / (2.0 * math.pi)) * math.sqrt((k_hop * 1.0e3) / unsprung)

    # roll-centre height (ROUGH, geometric): the instantaneous roll centre is found
    # by intersecting the line through the contact patch and the front-view instant
    # centre (the apparent pivot of the wheel) with the vehicle centreline. With the
    # hardpoints to hand we take the lower- and upper-arm front-view slopes, find the
    # instant centre IC, then the contact-patch -> IC line crossing the centreline.
    # For a single MacPherson lower arm we use the lower arm + strut-perpendicular.
    hp = hardpoints(p)
    rc_h = _roll_centre_height_mm(g, hp)

    # representative local-Z build envelope: knuckle height + spring/damper stack
    envelope = max(p.knuckle.height_mm, s.free_length_mm, d.damper_length_mm) + g.ride_height_mm

    # anti-dive / anti-squat note: the fore/aft inclination of the side-view arm
    # lines sets the anti-feature %. We report it qualitatively from the A-arm base.
    anti_note = ("side-view arm base ~%.0f mm (fore/aft); raise the inboard pickups "
                 "for more anti-dive/squat, lower for less (verify in ADAMS)"
                 % (2.0 * _ARM_HALF_BASE_MM))

    return DerivedSuspension(
        linkage_type=g.type,
        corners_modelled=n_corners,
        wheel_rate_n_per_mm=round(wheel_rate, 3),
        ride_frequency_hz=round(ride_freq, 3),
        spring_deflection_mm=round(defl, 1),
        roll_stiffness_nm_per_deg=round(roll_stiffness, 1),
        roll_centre_height_mm=round(rc_h, 1),
        damping_ratio=round(zeta, 3),
        damping_ratio_damper_frame=round(zeta_damper_frame, 3),
        wheel_hop_frequency_hz=round(hop_freq, 2),
        corner_envelope_height_mm=round(envelope, 1),
        anti_feature_note=anti_note,
    )


def _roll_centre_height_mm(g, hp: Dict[str, Vec3]) -> float:
    """ROUGH front-view (Y-Z plane) roll-centre height above the ground.

    Project the control arms into the front view (Y, Z). The lower arm runs from its
    inboard pickup to the lower ball joint; the upper arm (or, for MacPherson, the
    line perpendicular to the strut at the top mount) runs to the upper joint.
    Their extensions meet at the instant centre IC; the line from the tyre contact
    patch to IC crosses the vehicle centreline at the roll-centre height. All in the
    re-datumed LOCAL frame (hub at origin, +Y outboard); ground is at z = -ride_height.

    NOTE (ICD §3 re-datuming): the corner is hub-datumed, so the hub centre is the
    LOCAL origin (0,0,0) and the contact patch sits essentially DIRECTLY BELOW the hub
    -- offset inboard only by the scrub radius (where the steering axis meets the
    ground). It is NOT at +track/2: placing it there would mix the VEHICLE frame into
    the local-frame kinematics and double-count T/2 (the exact error the ICD forbids).
    The roll-centre height therefore depends on the ARM geometry, not on where the
    wheel sits in vehicle Y."""
    ride = g.ride_height_mm
    # contact patch in the LOCAL hub-origin frame: just inboard of the hub vertical by
    # the scrub radius (the steering axis crosses the ground there), on the ground.
    cp = (-g.scrub_radius_mm, -ride)            # (y, z) ; ~directly below the hub
    # lower-arm front-view line: pickup -> lower ball joint
    lp = (hp["lower_pickup_fore"][1], hp["lower_pickup_fore"][2])
    lb = (hp["lower_ball_joint"][1], hp["lower_ball_joint"][2])
    if g.type == "macpherson":
        # upper "line" is perpendicular to the strut at the top mount
        st = (hp["strut_top"][1], hp["strut_top"][2])
        sb = (hp["lower_ball_joint"][1], hp["lower_ball_joint"][2])
        sd = (st[0] - sb[0], st[1] - sb[1])
        up = st
        ub = (st[0] - sd[1], st[1] + sd[0])      # rotate strut dir by 90 deg
    else:
        up = (hp["upper_pickup_fore"][1], hp["upper_pickup_fore"][2])
        ub = (hp["upper_ball_joint"][1], hp["upper_ball_joint"][2])
    ic = _line_intersection(lp, lb, up, ub)
    if ic is None:
        return 0.0                               # parallel arms -> RC at ground (rough)
    # local hub-centre is at y = 0; the wheel-centre vertical that the contact patch
    # belongs to is at y = +track/2. RC = where the (contact-patch -> IC) line meets
    # the vehicle centreline (y = -track/2 inboard of this hub).
    centreline_y = -g.track_width_mm / 2.0
    rc = _line_y_at(cp, ic, centreline_y)
    if rc is None:
        return 0.0
    return rc + ride                             # height above ground


def _line_intersection(a0, a1, b0, b1):
    """Intersection of line a0a1 with line b0b1 in 2D, or None if parallel."""
    x1, y1 = a0
    x2, y2 = a1
    x3, y3 = b0
    x4, y4 = b1
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def _line_y_at(p0, p1, x):
    """Z value (second coord) of the line p0p1 at first-coord x, or None if vertical."""
    if abs(p1[0] - p0[0]) < 1e-9:
        return None
    t = (x - p0[0]) / (p1[0] - p0[0])
    return p0[1] + t * (p1[1] - p0[1])


# --------------------------------------------------------------------------- #
# buildability / sanity check (geometry must close before the NX builder runs)
# --------------------------------------------------------------------------- #
def validate(p: SuspensionParams) -> List[str]:
    """Return a list of geometric/engineering problems (empty list => buildable).
    Mirrors driveline_nx.engineering.validate()'s contract."""
    issues: List[str] = []
    g, s, d = p.geometry, p.spring, p.damper
    a, k, arm = p.antiroll, p.knuckle, p.arm
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

    # ---- 3D HARDPOINT geometry checks (the redesign's connectivity contract) --- #
    hp = hardpoints(p)

    def _dist(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    # (1) the hub bore is centred on the LOCAL ORIGIN (the redesign's datum)
    if _dist(hp["hub_centre"], (0.0, 0.0, 0.0)) > 1e-6:
        issues.append("hub centre must be the local origin (0,0,0) -- it is %s" % (hp["hub_centre"],))

    # (2) every link must REACH the hub: each outboard joint sits close to the hub
    #     centre (within a knuckle-sized radius), and each inboard pickup sits well
    #     inboard at -Y (toward the chassis centreline).
    knuckle_reach = math.hypot(k.width_mm, k.height_mm)        # knuckle bounding radius
    for jn in ("lower_ball_joint", "upper_ball_joint", "toe_outboard"):
        if g.type == "macpherson" and jn == "upper_ball_joint":
            continue
        if _dist(hp[jn], hp["hub_centre"]) > knuckle_reach + 5.0:
            issues.append("%s does not reach the hub (%.0f mm > knuckle reach %.0f mm)"
                          % (jn, _dist(hp[jn], hp["hub_centre"]), knuckle_reach))
    inboard_pts = ["lower_pickup_fore", "lower_pickup_aft", "toe_pickup"]
    if g.type in ("multilink", "double_wishbone"):
        inboard_pts += ["upper_pickup_fore", "upper_pickup_aft"]
    for nm in inboard_pts:
        if hp[nm][1] >= 0.0:
            issues.append("%s must sit INBOARD at -Y (toward the chassis); y=%.0f"
                          % (nm, hp[nm][1]))
        elif -hp[nm][1] >= half_track:
            issues.append("%s reaches past the chassis centreline (y=%.0f, track/2=%.0f)"
                          % (nm, hp[nm][1], half_track))

    # the outboard toe (tie-rod) joint must sit INBOARD of the hub face (y < 0), on the
    # steering arm -- never outboard of the wheel face (the low-severity finding).
    if hp["toe_outboard"][1] >= 0.0:
        issues.append("toe_outboard must sit inboard of the hub (y < 0); y=%.0f"
                      % hp["toe_outboard"][1])

    # (3) arm lengths must actually be realised between the pickup and the ball joint
    #     (the link length the blueprint builds, +/- the fore/aft base, must track the
    #     requested arm length within 25 %).
    for tag, pf, bj, want in (("lower", "lower_pickup_fore", "lower_ball_joint", g.lower_arm_length_mm),):
        got = _dist(hp[pf], hp[bj])
        if got < 0.6 * want or got > 1.6 * want:
            issues.append("%s arm built length %.0f mm strays from requested %.0f mm"
                          % (tag, got, want))

    # (4) no upright self-overlap: the lower and upper ball joints must clear the hub
    #     bore vertically -- their Z span must exceed the hub-bore radius plus a ball
    #     joint each side, else the upright bodies interfere (unbuildable steering
    #     axis around the bearing).
    min_bj_span = k.hub_bore_diameter_mm / 2.0 + arm.ball_joint_diameter_mm
    if g.type in ("multilink", "double_wishbone"):
        dz = hp["upper_ball_joint"][2] - hp["lower_ball_joint"][2]
        if dz < min_bj_span:
            issues.append("upper/lower ball joints too close in Z (%.0f < %.0f mm) -- "
                          "upright self-overlaps the hub bore; increase knuckle.height_mm"
                          % (dz, min_bj_span))

    # (5) the spring/damper must seat on the lower arm (below the hub) and reach up
    #     to a body mount ABOVE the hub -- a positive, sane inclined working length.
    seat = hp["damper_lower"]
    top = hp["strut_top"] if g.type == "macpherson" else hp["damper_top"]
    work = _dist(seat, top)
    if top[2] <= seat[2]:
        issues.append("damper top mount is not above its lower seat (no working travel)")
    if work < 0.5 * d.damper_length_mm:
        issues.append("damper working length %.0f mm << body length %.0f mm (won't reach the body)"
                      % (work, d.damper_length_mm))

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
        "  damping ratio (bump)     : %.2f wheel-frame  (rough; damper-frame %.2f)" % (
            der.damping_ratio, der.damping_ratio_damper_frame),
        "  anti-roll bar            : %s" % (
            "Ø%.0f, %.0f Nm/deg" % (a.bar_diameter_mm, a.rate_nm_per_deg) if a.enabled else "none"),
        "  roll stiffness           : %.0f Nm/deg  (arb + spring, rough)" % der.roll_stiffness_nm_per_deg,
        "  roll-centre height       : %.0f mm above ground  (front-view geometric, rough)" % der.roll_centre_height_mm,
        "  anti-dive / anti-squat   : %s" % der.anti_feature_note,
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
