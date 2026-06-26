"""NX-independent assembly math: turn the vehicle layout into a PLAN -- one entry
per component with its source part file and its placement (origin + 3x3 orientation
matrix) in vehicle coordinates. The NX journal consumes this plan verbatim.

THE SHARED DATUM (ICD §2). Every corner has one mating point all subsystems agree
on -- the wheel-hub centre::

    HUB_CENTRE(axle, side) = ( +-wheelbase/2 ,  +-track/2 ,  tyre_radius )

The driveline wheel-hub FLANGE FACE, the suspension upright HUB BORE and the wheel
all coincide there. Each subsystem is modelled in its OWN local frame; this module
records the documented local->vehicle placement (ICD §3) so every hub feature lands
on its HUB_CENTRE after the transform:

  * chassis           : built in TRUE vehicle coordinates -> placed with the IDENTITY
                        transform at the origin (0,0,0). Every chassis coordinate is
                        already a vehicle coordinate; its subframe pads sit at the
                        axle x-stations on the rail tops.
  * driveline / motor : local +Z = the rotation axis. In the vehicle the axle runs
                        left-right (along +Y), so local +Z -> vehicle +Y via Rx(-90),
                        origin at the differential centre [axle_x, 0, r]. The driveline
                        is BUILT to span the vehicle track (flange faces at local
                        z = +-T/2), so each flange lands at vehicle y = +-T/2 with NO
                        extra +-T/2 offset added here.
  * suspension corner : RE-DATUMED at the hub centre -- the upright bore is on the
                        LOCAL ORIGIN (0,0,0), +X fwd, +Y outboard, +Z up. The corner
                        origin therefore maps STRAIGHT to HUB_CENTRE(axle, side); left
                        is identity, right is Rz(180) (local +Y outboard -> vehicle -Y).
                        Because the part is hub-relative we do NOT pre-offset by T/2
                        (doing so would double-count the track) -- the placement origin
                        IS the +-T/2 hub station.
  * inverter          : box in its own frame, base (z=0) = mounting face, mounted
                        upright on the motor top (identity rotation).
  * gearbox           : the reduction-gearbox CONNECTOR (ICD §7.1), local +Z = the gear
                        axes and the LOCAL ORIGIN = the differential axis. Placed with
                        Rx(-90).Rz(180) at the diff station [axle_x, 0, r] (the SAME
                        station as the driveline -- they are coaxial-by-design). The
                        extra Rz(180) bridges motor<->diff: it lands the gearbox motor
                        flange on the motor DE flange (the motor sits at axle_x - dx, up
                        by dz) and couples the gearbox output to the driveline diff input.
  * subframe          : the suspension/e-axle SUBFRAME cradle (ICD §7.2), built in TRUE
                        vehicle coordinates offset by the axle station -> placed IDENTITY
                        per axle at [axle_x, 0, 0]. It bolts UP to the chassis pads and
                        presents the suspension inboard pickup bosses + e-axle mounts. Its
                        pickup bosses are DERIVED from the suspension hardpoint table with
                        the SAME placement convention the suspension corner uses (left =
                        identity, right = Rz(180), the SAME canonical corner front and rear
                        -- NO front X mirror), so the bosses coincide with the suspension
                        pickups on ALL FOUR corners. Front/rear are distinct part files
                        (same boss geometry, placed at +-wheelbase/2).

Exact bolt-hole mating is a downstream NX constraint step; this plan positions every
part parametrically so the assembly opens already laid out (the same "representative"
philosophy the subsystem blueprints use).

ICD §7 INTEGRATION (this module's headline job): the OLD motor offset (60/110 mm) drove
the motor into the differential. The reduction gearbox now bridges motor<->diff and the
motor is placed at the gearbox-derived final-drive centre distance (~156/176 mm ->
~235 mm centre distance, clearing the ICD §7.1 minimum). `validate()` enforces the §7
acceptance: NO two non-chassis component solids interpenetrate (the sampled-solid
overlap test in `vehicle_nx.clearance`) and the real matings hold (gearbox<->motor,
gearbox<->diff, subframe<->chassis pads, subframe bosses<->suspension pickups, tower
top<->damper top).

`validate()` enforces the ICD §4 dimensional-consistency rules against the ACTUAL
subsystem geometry (their NX-free `engineering.derive()` / blueprint accessors): the
driveline built track == vehicle track, the suspension hub-centre local Y == T/2, the
chassis length/rail-spacing bracket the wheelbase/track, plus the shared-origin and
ground-clearance checks. Those imports stay pure CPython (no NXOpen), so the plan and
its validation generate in plain Python.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence

from . import clearance
from .params import VehicleParams

Mat = List[List[float]]
Vec = List[float]

# ICD §4 consistency tolerances
_TRACK_TOL_PCT = 2.0      # driveline built track vs vehicle track (ICD §4.1)
_HUB_Y_TOL_MM = 1.0       # suspension hub-centre local Y vs T/2 (ICD §4.2)

# ICD §7 integration tolerances
_TOUCH_TOL_MM = clearance.TOUCH_TOL_MM   # AABB overlap below this is a mating touch, not a clash
_MATE_TOL_MM = 30.0       # mating-point coincidence tolerance (representative hardpoints)
# subframe boss <-> suspension pickup coincidence (ICD §7.4.2). The subframe now DERIVES
# its bosses from the suspension hardpoint table with the SAME placement convention, so
# they coincide essentially exactly; assert a tight 5 mm so a convention regression on ANY
# of the four corners is caught (the bug this guards against was 225-890 mm off).
_SUBFRAME_COINCIDENCE_TOL_MM = 5.0


# --------------------------------------------------------------------------- #
# small rotation helpers (3x3 row-major; vehicle_vec = R . local_vec)
# --------------------------------------------------------------------------- #
def identity() -> Mat:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def rot_x(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rot_y(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_z(deg: float) -> Mat:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def matmul(a: Mat, b: Mat) -> Mat:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _round_mat(m: Mat) -> Mat:
    return [[round(v, 9) for v in row] for row in m]


def _component(name: str, part_file: str, role: str, origin: Vec, orient: Mat,
               variant: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """One placed component. ``variant`` carries the build-time parameter overrides the
    assembler must apply to the role's blueprint before building this part (e.g. the
    subframe ``axle`` so the front/rear X-mirror variants build into distinct files)."""
    comp: Dict[str, Any] = {
        "name": name,
        "part_file": part_file,
        "role": role,
        "origin_mm": [round(v, 3) for v in origin],
        "orientation": _round_mat(orient),
    }
    if variant:
        comp["variant"] = variant
    return comp


def _subframe_part_file(base: str, axle: str) -> str:
    """Per-axle subframe part file: subframe_out.prt -> subframe_front_out.prt /
    subframe_rear_out.prt. The front/rear variants share the same suspension-derived
    boss/tower geometry (no X mirror); they are built and saved separately so each is
    placed at its own axle x-station."""
    stem, dot, ext = base.rpartition(".")
    stem = stem or base
    ext = ext if dot else "prt"
    return "%s_%s.%s" % (stem, axle, ext)


# --------------------------------------------------------------------------- #
# the plan
# --------------------------------------------------------------------------- #
def _driven_axles(layout: str) -> List[str]:
    return {"rear": ["rear"], "front": ["front"], "awd": ["front", "rear"]}.get(layout, ["rear"])


def _axle_x(p: VehicleParams, axle: str) -> float:
    """Vehicle x-station of an axle centre: +wheelbase/2 front, -wheelbase/2 rear."""
    return (+1.0 if axle == "front" else -1.0) * p.layout.wheelbase_mm / 2.0


def _track(p: VehicleParams, axle: str) -> float:
    return p.layout.track_front_mm if axle == "front" else p.layout.track_rear_mm


# --------------------------------------------------------------------------- #
# the e-axle motor offset -- ICD §7.1 (the cause of, and the fix for, the overlap)
# --------------------------------------------------------------------------- #
def _gearbox_motor_offset_mm() -> Optional[Dict[str, float]]:
    """The reduction-gearbox final-drive centre distance, split into the toward-centre
    (dx) and up (dz) components, from gearbox_nx.engineering. NX-free deferred import so
    the assembly still works if the gearbox package is absent (returns None).

    These two numbers ARE the motor<->differential centre distance (= C1 + C2 of the
    two reduction stages); placing the motor at this offset clears the ICD §7.1 minimum
    (~231 mm) so the motor and differential no longer interpenetrate."""
    try:
        from gearbox_nx.engineering import derive as g_derive
        from gearbox_nx.params import GearboxParams
        g = g_derive(GearboxParams())
        return {"dx": float(g.motor_offset_dx_mm), "dz": float(g.motor_offset_dz_mm)}
    except Exception:
        return None


def motor_offset(p: VehicleParams) -> Dict[str, float]:
    """Resolve the motor's (dx, dz) offset from the wheel/diff axis (mm). A non-zero
    eaxle.motor_offset_* overrides; 0.0 means AUTO -> take the gearbox-derived centre
    distance (ICD §7.1). The AUTO fallback (if gearbox_nx is unavailable) is the legacy
    cleared estimate dx=155, dz=176 so the motor never falls back onto the old 60/110
    overlap."""
    e = p.eaxle
    g = _gearbox_motor_offset_mm() or {"dx": 155.881, "dz": 176.192}
    dx = e.motor_offset_x_mm if e.motor_offset_x_mm else g["dx"]
    dz = e.motor_offset_z_mm if e.motor_offset_z_mm else g["dz"]
    return {"dx": float(dx), "dz": float(dz)}


def hub_centre(p: VehicleParams, axle: str, side_sign: float) -> Vec:
    """THE shared datum (ICD §2): the wheel-hub centre in vehicle coordinates,

        HUB_CENTRE(axle, side) = ( +-wheelbase/2 , +-track/2 , tyre_radius ).

    `side_sign` = +1 for the LEFT (+Y) wheel, -1 for the RIGHT (-Y). The driveline
    flange face, the suspension upright bore and the wheel all coincide here, so this
    is also the suspension corner's placement origin (the corner is hub-datumed)."""
    return [_axle_x(p, axle), side_sign * _track(p, axle) / 2.0, p.layout.tyre_radius_mm]


def gearbox_orientation() -> Mat:
    """The reduction-gearbox placement orientation (ICD §3/§7.1): Rx(-90) . Rz(180).

    The gearbox local frame has +Z = the parallel gear axes (like the motor/driveline),
    so the gear axis must map to the vehicle +Y axle -- that is the shared Rx(-90). The
    EXTRA Rz(180) is what makes the connector actually bridge motor<->diff: the gearbox
    builds its motor axis toward LOCAL +X (toward the vehicle centre) and +Y (up); the
    motor sits at vehicle ``axle_x - dx`` (AWAY from centre on a rear axle) and +Z (up),
    matching the established e-axle convention (the old motor_offset_x subtracts from
    the axle x). Composing Rz(180) flips local +X -> vehicle -X and local +Y -> vehicle
    +Z, so the gearbox motor-mounting flange lands exactly on the motor DE flange and the
    'up' gear offset becomes vehicle-up. (Verified: with the default gearbox the gearbox
    motor FLANGE FACE maps to the same vehicle point as the motor DE flange face -- the
    full-3D mating check in validate() asserts this.)

    CONVENTION NOTE (review finding 7). The motor sits at ``axle_x - dx`` (AWAY from the
    vehicle centre on a rear axle). The ICD §7.1 *text* prefers the motor toward the
    centre-plane, but that is NOT achievable for BOTH axles with a single proper-rotation
    gearbox placement that ALSO keeps the gear axis along +Y (required so the gearbox
    output couples to the +Y-built driveline diff input). Mapping the gearbox's local +X
    motor offset to vehicle +X (toward centre, rear) while keeping +Z->+Y and +Y->+Z is an
    improper (mirror) transform; a toward-centre motor on both axles would need per-axle
    gearbox geometry variants (the motor built toward local -X). The away-from-centre
    placement is geometrically buildable, mass-centralisation-neutral on the diff axis, and
    keeps the gear axis coupling correct, so it is retained; the toward-centre flip is
    deferred to a per-axle gearbox variant (documented, not silently diverging from the
    ICD). What review finding 7's sibling defect -- the motor not actually MEETING the
    gearbox -- IS fixed: ``motor_axial_y`` now butts the DE flange to the gearbox flange."""
    return matmul(rot_x(-90.0), rot_z(180.0))


def components(p: VehicleParams) -> List[Dict[str, Any]]:
    """Ordered component list with placements (vehicle frame)."""
    L, e, f = p.layout, p.eaxle, p.parts
    z_hub = L.tyre_radius_mm
    axle_x = {"rear": _axle_x(p, "rear"), "front": _axle_x(p, "front")}

    out: List[Dict[str, Any]] = []

    # 1) chassis -- the platform at the origin
    if f.include_chassis:
        out.append(_component("CHASSIS", f.chassis, "chassis", [0.0, 0.0, 0.0], identity()))

    # 2) e-axle(s): driveline + gearbox + motor (+ inverter) at each driven axle.
    #    The driveline and gearbox are both DIFF-AXIS datumed -> same diff station
    #    origin; the gearbox bridges the motor (offset by the gearbox centre distance)
    #    to the differential, so the motor no longer interpenetrates the diff (ICD §7.1).
    eaxle_rot = rot_x(-90.0)        # local +Z (rotation axis) -> vehicle +Y (axle left-right)
    gbox_rot = gearbox_orientation()   # Rx(-90).Rz(180): bridges motor<->diff (see above)
    for ax in _driven_axles(L.drive_layout):
        ax_x = axle_x[ax]
        out.append(_component(
            "DRIVELINE_%s" % ax.upper(), f.driveline, "driveline",
            [ax_x, 0.0, z_hub], eaxle_rot))
        # the reduction gearbox: datumed on the DIFF axis (= the driveline origin), so it
        # shares the diff station; its output couples to the driveline diff input and its
        # motor flange receives the motor.
        if f.include_gearbox:
            out.append(_component(
                "GEARBOX_%s" % ax.upper(), f.gearbox, "gearbox",
                [ax_x, 0.0, z_hub], gbox_rot))
        # the motor, at the gearbox-derived final-drive offset in X/Z (away from centre =
        # -X on the rear axle per the established convention, and +Z up) and shifted
        # AXIALLY in Y so its DE flange butts the gearbox motor flange with no gap (review
        # finding 1). This REPLACES the old 60/110 offset that drove the motor into the diff
        # AND closes the 187 mm gap that left the gearbox not meeting the motor.
        m_org = motor_origin(p, ax)
        out.append(_component(
            "MOTOR_%s" % ax.upper(), f.motor, "motor", m_org, eaxle_rot))
        if f.include_inverter:
            out.append(_component(
                "INVERTER_%s" % ax.upper(), f.inverter, "inverter",
                [m_org[0] + e.inverter_offset_x_mm, m_org[1],
                 m_org[2] + e.inverter_offset_z_mm], identity()))

    # 3) subframe (cradle) per axle: built in TRUE vehicle coordinates offset by the axle
    #    station, so it is placed with IDENTITY at the axle x-station (ICD §3/§7.2). It
    #    bolts UP to the chassis subframe pads and presents the suspension inboard pickup
    #    bosses + e-axle mounts. The boss geometry is DERIVED from the suspension hardpoint
    #    table with the suspension's own placement convention and is the SAME front and rear
    #    (NO X mirror -- the suspension reuses one canonical corner); we just place each
    #    variant at its axle x-station.
    if f.include_subframe:
        sub_axles = ["front", "rear"] if L.suspension_corners >= 4 else _driven_axles(L.drive_layout)
        for ax in sub_axles:
            out.append(_component(
                "SUBFRAME_%s" % ax.upper(), _subframe_part_file(f.subframe, ax), "subframe",
                [axle_x[ax], 0.0, 0.0], identity(), variant={"axle": ax}))

    # 4) suspension corners. The corner is RE-DATUMED at the hub centre, so its
    #    placement origin IS the shared HUB_CENTRE(axle, side) -- (axle_x, +-T/2, r).
    #    NO extra +-T/2 offset is added: the part is hub-relative, and local +Y
    #    outboard maps to vehicle +Y on the left (identity) and vehicle -Y on the
    #    right (Rz(180)), so the upright bore lands exactly on the wheel.
    if p.layout.suspension_corners >= 4:
        axles = ["front", "rear"]
    else:
        axles = _driven_axles(L.drive_layout)
    for ax in axles:
        for side, sign in (("L", +1.0), ("R", -1.0)):
            orient = identity() if sign > 0 else rot_z(180.0)
            out.append(_component(
                "SUSPENSION_%s%s" % (ax[0].upper(), side), f.suspension, "suspension",
                list(hub_centre(p, ax, sign)), orient))
    return out


def build_plan(p: VehicleParams = None) -> Dict[str, Any]:
    if p is None:
        p = VehicleParams()
    comps = components(p)
    return {
        "schema": "vehicle_nx.assembly/1",
        "name": p.name,
        "units": "mm",
        "frame": "ISO8855 (+X fwd, +Y left, +Z up)",
        "parameters": p.to_dict(),
        "validation": validate(p),
        "components": comps,
    }


def to_json(plan: Dict[str, Any], indent: int = 2) -> str:
    import json
    return json.dumps(plan, indent=indent)


# --------------------------------------------------------------------------- #
# per-role blueprint + vehicle-frame AABB (NX-FREE) -- the ICD §7 clearance check
#
# Each subsystem ships a pure-Python blueprint.generate() that emits the same
# motor_nx.blueprint.BuildStep list the NX builder consumes. We build it here (no
# NXOpen), bound it with vehicle_nx.clearance.local_bbox, and transform it into the
# vehicle frame by the component's recorded orientation + origin. Imports are deferred
# so the module imports even if a subsystem package is absent.
# --------------------------------------------------------------------------- #
# cache of (role, variant-key) -> blueprint dict. The default-params blueprints are
# deterministic and several are heavy (the motor especially), so caching keeps the
# repeated clearance/mating probes in validate()/report()/tests cheap. The cached dicts
# are read-only here (the clearance helpers never mutate them).
_BLUEPRINT_CACHE: Dict[tuple, Optional[Dict[str, Any]]] = {}


def _role_blueprint(role: str, variant: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
    """The NX-free blueprint dict for a component role, with any ``variant`` overrides
    applied to its params. Returns None if the package is unavailable. (The connector
    packages -- gearbox_nx / subframe_nx -- expose ``blueprint.generate(params)`` exactly
    like the original subsystems, and importing their blueprint/params is NX-safe; their
    nx_builder is NOT imported, which would auto-run.) Cached per (role, variant)."""
    variant = variant or {}
    key = (role, tuple(sorted(variant.items())))
    if key in _BLUEPRINT_CACHE:
        return _BLUEPRINT_CACHE[key]
    blue = _role_blueprint_uncached(role, variant)
    _BLUEPRINT_CACHE[key] = blue
    return blue


def _role_blueprint_uncached(role: str, variant: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    try:
        if role == "motor":
            from motor_nx.blueprint import generate
            from motor_nx.params import MotorParams
            return generate(MotorParams())
        if role == "driveline":
            from driveline_nx.blueprint import generate
            from driveline_nx.params import DrivelineParams
            return generate(DrivelineParams())
        if role == "inverter":
            from inverter_nx.blueprint import generate
            from inverter_nx.params import InverterParams
            return generate(InverterParams())
        if role == "suspension":
            from suspension_nx.blueprint import generate
            from suspension_nx.params import SuspensionParams
            return generate(SuspensionParams())
        if role == "chassis":
            from chassis_nx.blueprint import generate
            from chassis_nx.params import ChassisParams
            return generate(ChassisParams())
        if role == "gearbox":
            from gearbox_nx.blueprint import generate
            from gearbox_nx.params import GearboxParams
            return generate(GearboxParams())
        if role == "subframe":
            from subframe_nx.blueprint import generate
            from subframe_nx.params import SubframeParams
            sp = SubframeParams()
            if variant.get("axle"):
                sp = sp.overridden(**{"axle": variant["axle"]})
            return generate(sp)
    except Exception:
        return None
    return None


def component_world_aabb(c: Dict[str, Any]) -> Optional[tuple]:
    """The vehicle-frame AABB (min_xyz, max_xyz) of a placed component, or None if its
    blueprint is unavailable / has no solid body. Built from the component's role
    blueprint placed at its recorded orientation + origin (ICD §7.6)."""
    blue = _role_blueprint(c["role"], c.get("variant"))
    if blue is None:
        return None
    return clearance.world_aabb(blue, c["orientation"], c["origin_mm"])


def _component_solids(c: Dict[str, Any]) -> Optional[List[Dict[str, Any]]]:
    """The oriented vehicle-frame solid primitives of a placed component (NX-free), or
    None if the blueprint is unavailable."""
    blue = _role_blueprint(c["role"], c.get("variant"))
    if blue is None:
        return None
    return clearance.part_solids(blue, c["orientation"], c["origin_mm"])


def _component_voids(c: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The oriented vehicle-frame VOID primitives of a placed component (subtract bodies:
    bores, bolt holes, hollow-box cavities and -- the crucial one -- the chassis axle
    NOTCH relief windows). The void-aware clash test (clearance.solids_clash) treats a
    neighbour point that passes THROUGH one of these voids as clearance, not a clash, so
    a half-shaft / control arm sweeping through the rail notch is not falsely flagged."""
    blue = _role_blueprint(c["role"], c.get("variant"))
    if blue is None:
        return []
    return clearance.part_voids(blue, c["orientation"], c["origin_mm"])


def interpenetration_pairs(p: VehicleParams,
                           touch_tol: float = _TOUCH_TOL_MM) -> List[Dict[str, Any]]:
    """Every NON-CHASSIS component pair whose SOLIDS interpenetrate by more than
    ``touch_tol`` (ICD §7.1/§7.4.1 headline acceptance check).

    Uses the sampled-solid test (vehicle_nx.clearance.solids_interpenetrate): each body
    is bounded by its true oriented primitive (finite cylinder / swept polygon / revolve
    disc) and a clash exists only if a sampled surface point of one part lies INSIDE a
    body of the other. This is the ICD §7.6 "(better: sampled solid)" variant -- a plain
    whole-part AABB squares off a ROUND body, so two parallel cylinders offset diagonally
    (the motor vs the differential) would report a spurious box overlap even when the
    round solids clear; the sampled-solid test resolves that correctly.

    The chassis is the reference body that everything mounts into (the rails wrap the
    battery + reach the axles), so its solid legitimately envelops other parts -- the
    rail-vs-corner / rail-vs-halfshaft piercing is policed separately by the dedicated
    notch-aware rail check, and the chassis is excluded here. Mating NEIGHBOURS that are
    MEANT to bolt/seat together (the motor and its own gearbox, the suspension corner and
    its own subframe at the same axle, ...) are skipped: their solids touch at the joint
    by design. CRITICALLY motor<->driveline is NOT a mating pair (the gearbox bridges
    them), so a motor pushed back into the differential is caught here. Returns a list of
    ``{a, b, penetration_mm}`` records (empty => nothing interpenetrates)."""
    comps = [c for c in components(p) if c["role"] != "chassis"]
    solids = {c["name"]: _component_solids(c) for c in comps}
    out: List[Dict[str, Any]] = []
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            ca, cb = comps[i], comps[j]
            if _are_mating_neighbours(ca, cb):
                continue
            sa, sb = solids[ca["name"]], solids[cb["name"]]
            if sa is None or sb is None:
                continue
            pen = clearance.solids_interpenetrate(sa, sb, touch_tol)
            if pen is not None:
                out.append({"a": ca["name"], "b": cb["name"], "penetration_mm": pen})
    return out


def comprehensive_interpenetration_pairs(
        p: VehicleParams, touch_tol: float = _TOUCH_TOL_MM) -> List[Dict[str, Any]]:
    """HONEST whole-vehicle clash check (ICD §7.1/§7.4.1): the VOID-AWARE sampled-solid
    overlap test run between EVERY component pair INCLUDING the chassis, with NO
    "mating-neighbour" skip -- except a NARROW, documented allowlist for the bolted
    integrated e-axle / hub unit.

    Why this exists. The older :func:`interpenetration_pairs` is a FALSE PASS: it EXCLUDES
    the chassis outright and SKIPS every "mating neighbour" pair, which hid five real
    clashes the user confirmed in NX (half-shaft/diff piercing the rail beyond the notch,
    the subframe cradle overlapping the control arms, the cradle/e-axle overlap). This
    check fixes both blind spots:

      * The CHASSIS is included. Its rail axle-NOTCH relief and every hollow-box cavity /
        bolt bore are real VOIDS (clearance.part_voids), so a half-shaft or control arm
        passing THROUGH the notch is clearance, not a clash -- only an overlap with the
        rail's REMAINING solid is flagged.
      * Every pair is checked. Intended bolt/press-fit/butt contacts touch only to
        ``touch_tol`` (a few mm) and so do not register; a deeper solid overlap does.

    The ONLY skipped pairs are the bolted INTEGRATED E-AXLE / WHEEL-HUB unit
    (``_EAXLE_UNIT_PAIRS``): gearbox<->motor (DE-flange bolted), gearbox<->differential
    (carrier seats inside the housing) and driveline<->its-own-suspension-hub (the wheel
    hub flange in the upright bore). Those are genuinely one assembled unit whose solids
    seat together by design. EVERY OTHER pair -- notably subframe<->suspension,
    subframe<->chassis, driveline<->chassis, suspension<->chassis, subframe<->driveline,
    subframe<->gearbox -- must be clash-free.

    Returns a list of ``{a, b, penetration_mm}`` records (empty => the vehicle is clean)."""
    comps = components(p)
    solids = {c["name"]: _component_solids(c) for c in comps}
    voids = {c["name"]: _component_voids(c) for c in comps}
    out: List[Dict[str, Any]] = []
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            ca, cb = comps[i], comps[j]
            if _is_eaxle_unit_pair(ca, cb):
                continue
            sa, sb = solids[ca["name"]], solids[cb["name"]]
            if sa is None or sb is None:
                continue
            pen = clearance.solids_clash(sa, sb, voids[ca["name"]], voids[cb["name"]],
                                         touch_tol)
            if pen is not None:
                out.append({"a": ca["name"], "b": cb["name"], "penetration_mm": pen})
    return out


# The bolted INTEGRATED E-AXLE / WHEEL-HUB unit -- the ONLY pairs the honest whole-vehicle
# check (comprehensive_interpenetration_pairs) allows to overlap. These are genuinely one
# assembled unit whose solids seat together by design:
#   * gearbox<->motor       : the motor DE flange BOLTS to the gearbox motor flange and the
#                             representative pinion/bearing envelopes seat inside the bell
#                             housing (the established e-axle blank philosophy).
#   * gearbox<->differential: the diff carrier seats INSIDE the gearbox housing (the gearbox
#                             encloses + mounts the carrier, ICD §7.1).
#   * driveline<->suspension: the wheel-hub flange seats in the upright HUB BORE -- the
#                             driveline's OWN suspension hub at the same corner (the shared
#                             ICD §2 datum). (Cross-axle driveline<->suspension is still
#                             checked: a different-axle pair is never this unit.)
# EVERY OTHER pair must be clash-free.
_EAXLE_UNIT_PAIRS = frozenset(frozenset(pair) for pair in (
    ("motor", "gearbox"),
    ("gearbox", "differential"),   # role name placeholder; the diff lives inside driveline
    ("gearbox", "driveline"),      # the gearbox output couples to / encloses the diff carrier
    ("driveline", "suspension"),   # wheel-hub flange <-> upright hub bore (same corner only)
))


def _is_eaxle_unit_pair(ca: Dict[str, Any], cb: Dict[str, Any]) -> bool:
    """True iff two components are the bolted INTEGRATED E-AXLE / WHEEL-HUB unit (the only
    by-design solid overlap the honest check allows). Only SAME-AXLE parts qualify; a
    cross-axle pair is never the same unit and is always checked."""
    ax_a, ax_b = _axle_suffix(ca["name"]), _axle_suffix(cb["name"])
    if ax_a and ax_b and ax_a != ax_b:
        return False
    return frozenset((ca["role"], cb["role"])) in _EAXLE_UNIT_PAIRS


def _axle_suffix(name: str) -> str:
    """The axle a component name belongs to: 'FRONT' or 'REAR', '' if none. Handles BOTH
    the axle-suffixed names (DRIVELINE_REAR, GEARBOX_FRONT, SUBFRAME_REAR) AND the
    suspension CORNER names (SUSPENSION_FL/FR -> FRONT, SUSPENSION_RL/RR -> REAR) so a
    cross-axle pair (e.g. DRIVELINE_REAR vs SUSPENSION_FL) is correctly recognised as
    different axles and never wrongly allowlisted as the same e-axle/hub unit."""
    for ax in ("FRONT", "REAR"):
        if name.endswith("_" + ax):
            return ax
    for suffix, ax in (("_FL", "FRONT"), ("_FR", "FRONT"), ("_RL", "REAR"), ("_RR", "REAR")):
        if name.endswith(suffix):
            return ax
    return ""


# Designed bolted/mating role pairs (UNORDERED): an AABB overlap between two parts in a
# pair is an intended joint, not a clash, so it is excluded from the headline solid-overlap
# check. CRITICALLY, motor<->driveline is NOT here -- the reduction gearbox bridges them
# (motor->gearbox->diff), so they must NOT touch; that pair is exactly what fires when a
# bad motor offset drives the motor back into the differential (ICD §7.1).
#
# The blanket skip is NOT all-or-nothing for every pair: the gearbox<->motor BUTT joint is
# additionally policed by a BOUNDED axial-engagement check (_mating_engagement_issues) so a
# connector that has slid OVER its neighbour (the old 149 mm gearbox-into-motor burial) is
# still flagged even though the pair is a designed mating neighbour (review finding 4). The
# truly ENVELOPED pairs (the diff carrier seats INSIDE the gearbox; the suspension corner
# pickups seat ON the subframe bosses; parts CARRIED by the subframe cradle) legitimately
# overlap by design and stay skipped.
_MATING_ROLE_PAIRS = frozenset(frozenset(pair) for pair in (
    ("motor", "gearbox"),       # motor DE flange  <-> gearbox motor-mounting flange
    ("motor", "inverter"),      # inverter sits on the motor top
    ("gearbox", "inverter"),    # inverter housing spans the motor + gearbox top
    ("gearbox", "driveline"),   # gearbox output coupling <-> driveline diff input
    ("driveline", "suspension"),# wheel-hub flange <-> upright hub bore (the shared datum)
    ("subframe", "suspension"), # cradle pickup bosses carry the corner inboard pickups
    ("subframe", "gearbox"),    # cradle e-axle mounts carry the gearbox/diff carrier
    ("subframe", "driveline"),  # cradle e-axle mounts carry the diff
    ("subframe", "motor"),      # the motor sits in/over the cradle (carried via the e-axle)
    ("subframe", "inverter"),   # the inverter sits over the cradle (carried via the e-axle)
))


def _are_mating_neighbours(ca: Dict[str, Any], cb: Dict[str, Any]) -> bool:
    """True if two components are DESIGNED to bolt/seat together (so an AABB overlap at
    their joint is expected, not a clash). Only SAME-AXLE parts in a documented mating
    role pair are skipped; cross-axle parts and non-mating role pairs (notably
    motor<->driveline, bridged by the gearbox) are always checked."""
    ax_a, ax_b = _axle_suffix(ca["name"]), _axle_suffix(cb["name"])
    if ax_a and ax_b and ax_a != ax_b:
        return False                               # different axles never mate
    return frozenset((ca["role"], cb["role"])) in _MATING_ROLE_PAIRS


# --------------------------------------------------------------------------- #
# subsystem geometry probes (NX-FREE) for the ICD §4 consistency checks
#
# Each subsystem ships a pure-Python engineering.derive() / blueprint accessor that
# reports the dimensions it actually BUILT. We read those back and reconcile them
# against the vehicle master dimensions. Imports are deferred (inside the helper) so
# the assembly module imports even if a subsystem package is absent, and stay NX-free.
# --------------------------------------------------------------------------- #
def _driveline_built_track_mm() -> float:
    """The track length the default driveline part actually spans (wheel-hub flange
    face to flange face), from driveline_nx.engineering. The driveline solves its
    inboard plunge clearance so each flange face lands at local |z| = T/2."""
    from driveline_nx.engineering import derive as d_derive
    from driveline_nx.params import DrivelineParams
    return float(d_derive(DrivelineParams()).total_track_length_mm)


def _driveline_hub_bore_od_mm() -> float:
    from driveline_nx.params import DrivelineParams
    return float(DrivelineParams().wheel_hub.bearing_outer_diameter)


def _suspension_hub_local_y_mm() -> float:
    """The suspension corner's hub-centre LOCAL Y. The corner is re-datumed onto the
    hub bore, so the hub centre sits on the local origin -> local Y = 0. The corner's
    outboard reach to the wheel is the full +track/2 (the contact patch in the local
    frame), which engineering reports as track_width_mm/2."""
    from suspension_nx.engineering import hardpoints
    from suspension_nx.params import SuspensionParams
    return float(hardpoints(SuspensionParams())["hub_centre"][1])


def _suspension_built_half_track_mm() -> float:
    """Half the full track the suspension corner is parameterised for (its outboard
    contact-patch Y). Must equal T/2 so the corner spans to the wheel."""
    from suspension_nx.params import SuspensionParams
    return float(SuspensionParams().geometry.track_width_mm) / 2.0


def _suspension_hub_bore_od_mm() -> float:
    from suspension_nx.params import SuspensionParams
    return float(SuspensionParams().knuckle.hub_bore_diameter_mm)


def _motor_envelope_radius_mm() -> float:
    """Outer radius of the motor housing envelope (cooling jacket OD grown by the
    mounting-flange OD margin), from motor_nx.em_design -- used for the ICD §4.5
    ground-clearance check. NX-free."""
    from motor_nx.em_design import derive as m_derive
    from motor_nx.params import MotorParams
    mp = MotorParams()
    g = m_derive(mp)
    c, a = mp.cooling, mp.assembly
    jacket_outer = g.stator_outer_radius + c.housing_gap + c.jacket_thickness
    return float(jacket_outer + getattr(a, "housing_flange_od_margin", 0.0))


def _norm3(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def _step_world_points(s) -> List[List[float]]:
    """Corner points of a build-step body in its OWN local frame (NX-free), via the
    pure-math twins -- a prism's section at both ends, or a cylinder/tube's end-centre
    radial extent. Mirrors the suspension test helper so the clearance check sees the
    same envelope the geometry builds."""
    from motor_nx.blueprint import profile_to_world
    kind = s["kind"]
    if kind == "prism" and s.get("profile") is not None and s.get("origin3") is not None:
        near = profile_to_world([tuple(p) for p in s["profile"]], tuple(s["origin3"]),
                                tuple(s["axis"]), tuple(s.get("u_dir", (1.0, 0.0, 0.0))))
        w = _norm3(s["axis"])
        L = s["length"]
        far = [[x + w[0] * L, y + w[1] * L, z + w[2] * L] for (x, y, z) in near]
        return [list(p) for p in near] + far
    if kind in ("cylinder", "tube") and s.get("origin3") is not None:
        base = tuple(s["origin3"])
        w = _norm3(s["axis"])
        L = s["length"]
        r = s.get("outer_radius", 0.0)
        helper = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
        u = _norm3([w[1] * helper[2] - w[2] * helper[1],
                    w[2] * helper[0] - w[0] * helper[2],
                    w[0] * helper[1] - w[1] * helper[0]])
        v = [w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0]]
        pts = []
        for t in (0.0, L):
            c = [base[0] + w[0] * t, base[1] + w[1] * t, base[2] + w[2] * t]
            for d in (u, v):
                pts.append([c[0] + r * d[0], c[1] + r * d[1], c[2] + r * d[2]])
                pts.append([c[0] - r * d[0], c[1] - r * d[1], c[2] - r * d[2]])
        return pts
    return []


def _xform(R: Mat, o: Vec, p) -> List[float]:
    return [o[i] + sum(R[i][k] * p[k] for k in range(3)) for i in range(3)]


def _suspension_corner_world_points(p: VehicleParams, axle: str, side_sign: float) -> List[List[float]]:
    """The suspension corner's body point cloud transformed into VEHICLE coordinates
    for a given corner (origin = HUB_CENTRE, left = identity, right = Rz(180))."""
    from suspension_nx.blueprint import generate as s_generate
    from suspension_nx.params import SuspensionParams
    blue = s_generate(SuspensionParams())
    R = identity() if side_sign > 0 else rot_z(180.0)
    o = hub_centre(p, axle, side_sign)
    out: List[List[float]] = []
    for s in blue["build_steps"]:
        for pt in _step_world_points(s):
            out.append(_xform(R, o, pt))
    return out


def _driveline_world_points(p: VehicleParams, axle: str) -> List[List[float]]:
    """The driveline body point cloud (half-shafts, CV joints, hubs) transformed into
    VEHICLE coordinates: placed at [axle_x, 0, r] with Rx(-90) (local +Z -> +Y)."""
    from driveline_nx.blueprint import generate as d_generate
    from driveline_nx.params import DrivelineParams
    blue = d_generate(DrivelineParams())
    R = rot_x(-90.0)
    o = [_axle_x(p, axle), 0.0, p.layout.tyre_radius_mm]
    out: List[List[float]] = []
    for s in blue["build_steps"]:
        if s["kind"] in ("cylinder", "tube") and s.get("origin3") is None:
            # legacy +Z column (cx, cy, z0): the body runs ALONG local +Z (-> vehicle
            # +Y after Rx(-90)). Sample its radial extent at several stations ALONG the
            # length, not just the end caps -- the half-shaft crosses the rail Y band at
            # its mid-length, so end-only sampling would miss the interference.
            r = s.get("outer_radius", 0.0)
            cx, cy, z0, L = s.get("cx", 0.0), s.get("cy", 0.0), s.get("z0", 0.0), s.get("length", 0.0)
            n = 16
            for i in range(n + 1):
                zz = z0 + L * i / n
                for dx, dy in ((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r), (0.0, 0.0)):
                    out.append(_xform(R, o, [cx + dx, cy + dy, zz]))
        else:
            for pt in _step_world_points(s):
                out.append(_xform(R, o, pt))
    return out


# --------------------------------------------------------------------------- #
# mating-point coordinates (vehicle frame) -- the ICD §7.4.2 coincidence checks
#
# Each interface point is the documented LOCAL coordinate of a feature, transformed by
# the component's vehicle placement (orientation + origin). Two features MATE when their
# vehicle points coincide within tolerance. All NX-free.
# --------------------------------------------------------------------------- #
# gearbox interface points in the gearbox LOCAL frame (read from gearbox_nx so they track
# the real built geometry; the gearbox is diff-axis datumed, +Z = gear axes). Fallback to
# the documented default-gearbox literals if the package is unavailable.
_GEARBOX_IFACE_FALLBACK = {
    "motor_flange_face": (155.881, 176.192, -28.0),  # -Z motor-mount face, on the motor axis
    "motor_axis": (155.881, 176.192, 0.0),           # motor input axis
    "output_coupling_face": (0.0, 0.0, 76.0),        # +Z output coupling -> driveline diff input
    "diff_mount_face": (0.0, 0.0, 122.0),            # +Z diff-carrier mount face
}


def _gearbox_iface_local() -> Dict[str, tuple]:
    """The gearbox interface points in the gearbox LOCAL frame, read from gearbox_nx's
    derived geometry (motor axis position, motor-flange face Z, output coupling + diff
    mount Z) so the assembly's mating coordinates track what the gearbox actually builds
    -- not stale literals. NX-free; falls back to the documented defaults if the package
    is absent."""
    try:
        from gearbox_nx.engineering import derive as g_derive, axis_positions, axial_bands
        from gearbox_nx.params import GearboxParams
        gp = GearboxParams()
        g = g_derive(gp)
        pos = axis_positions(gp)
        bands = axial_bands(gp)
        mx, my = pos["motor"]
        h = gp.housing
        # the housing diff-side end cover face (z0) is where the motor flange stands proud
        z0 = bands["stage1"][0] - h.end_cover_thickness_mm
        motor_face_z = z0 - h.motor_flange_thickness_mm     # -Z motor-mount face
        # the output coupling flange spans bands["stage2"][1] .. + flange thickness; the
        # face the driveline diff input bolts to is the OUTER (+Z) face.
        out_z = bands["stage2"][1] + gp.output.flange_thickness_mm
        diff_z = z0 + (g.housing_axial_length_mm + 2.0 * h.end_cover_thickness_mm)  # +Z diff mount face
        # the diff mount face is at the +Z end cover plus the diff-mount flange thickness
        diff_z += h.diff_mount_thickness_mm
        return {
            "motor_flange_face": (mx, my, motor_face_z),
            "motor_axis": (mx, my, 0.0),
            "output_coupling_face": (0.0, 0.0, out_z),
            "diff_mount_face": (0.0, 0.0, diff_z),
        }
    except Exception:
        return dict(_GEARBOX_IFACE_FALLBACK)
# the motor DE (drive-end) mounting flange face in the MOTOR local frame: the +Z housing
# face, on the rotation axis (the gearbox bolts here). z = stack_length + end_margin.
def _motor_de_flange_face_local() -> Optional[List[float]]:
    """The motor DE flange face centre in the motor LOCAL frame (on the rotation axis,
    +Z). Read from motor_nx.blueprint so it tracks the real geometry."""
    try:
        from motor_nx.blueprint import generate as m_generate
        from motor_nx.params import MotorParams
        blue = m_generate(MotorParams())
        for s in blue["build_steps"]:
            if s.get("id") == "housing_flange_de_disk":
                return [0.0, 0.0, float(s.get("z0", 0.0)) + float(s.get("length", 0.0))]
        # fall back to the stack-length advisory if the flange step is absent
        return [0.0, 0.0, float(blue.get("stack_length", 0.0))]
    except Exception:
        return None


def gearbox_iface_world(p: VehicleParams, axle: str, name: str) -> List[float]:
    """A gearbox interface point in VEHICLE coordinates (gearbox placed Rx(-90).Rz(180)
    at the diff station)."""
    R = gearbox_orientation()
    o = [_axle_x(p, axle), 0.0, p.layout.tyre_radius_mm]
    return _xform(R, o, list(_gearbox_iface_local()[name]))


def motor_axial_y(p: VehicleParams, axle: str) -> float:
    """The motor's AXIAL (vehicle Y) placement so its DE flange face BUTTS the gearbox
    motor-mounting flange face with no gap (ICD §7.1/§7.4.2, review finding 1).

    The motor is placed with Rx(-90), so its DE flange (motor local +Z = z_de) maps to
    vehicle Y = motor_origin_y + z_de. The gearbox motor flange face sits at a fixed
    vehicle Y (gearbox_iface 'motor_flange_face'); set the motor origin Y so the two
    faces coincide:  motor_origin_y = gearbox_motor_flange_Y - z_de.

    Before this fix the motor sat at Y=0 with its DE flange 187 mm away from (and facing
    the opposite end of) the gearbox motor flange -- the connector did not actually meet
    the motor. Falls back to 0.0 if motor_nx is unavailable (no DE flange to butt)."""
    face = _motor_de_flange_face_local()
    if face is None:
        return 0.0
    z_de = face[2]
    gb_flange = gearbox_iface_world(p, axle, "motor_flange_face")
    return gb_flange[1] - z_de


def motor_origin(p: VehicleParams, axle: str) -> List[float]:
    """The motor placement origin (vehicle frame): offset toward the wheel/diff axis in
    X/Z by the gearbox-derived final-drive centre distance, and shifted AXIALLY in Y so
    the DE flange butts the gearbox motor flange (review finding 1)."""
    off = motor_offset(p)
    return [_axle_x(p, axle) - off["dx"], motor_axial_y(p, axle),
            p.layout.tyre_radius_mm + off["dz"]]


def motor_de_flange_world(p: VehicleParams, axle: str) -> Optional[List[float]]:
    """The motor DE flange face in VEHICLE coordinates (motor placed Rx(-90) at the
    gearbox-derived offset + axial-Y butt). None if motor_nx is unavailable."""
    face = _motor_de_flange_face_local()
    if face is None:
        return None
    R = rot_x(-90.0)
    return _xform(R, motor_origin(p, axle), face)


def _driveline_diff_input_flange_local() -> Optional[List[float]]:
    """The driveline DIFF INPUT flange face centre in the driveline LOCAL frame (where the
    gearbox output coupling bolts). Read from the driveline blueprint so it tracks the real
    geometry: with the diff demoted to a true 1:1 differential the input is COAXIAL with
    the diff axis (cx=0), so this lands on the diff axis and the gearbox output (also on
    the diff axis) couples to it (review findings 2/3). None if driveline_nx is absent."""
    try:
        from driveline_nx.blueprint import generate as d_generate
        from driveline_nx.params import DrivelineParams
        blue = d_generate(DrivelineParams())
        for s in blue["build_steps"]:
            if s.get("id") == "diff_input_flange":
                cx = float(s.get("cx", 0.0))
                cy = float(s.get("cy", 0.0))
                z = float(s.get("z0", 0.0)) + float(s.get("length", 0.0))
                return [cx, cy, z]
        return None
    except Exception:
        return None


def driveline_diff_input_world(p: VehicleParams, axle: str) -> Optional[List[float]]:
    """The driveline diff INPUT flange face in VEHICLE coordinates (driveline placed
    Rx(-90) at the diff station). The gearbox OUTPUT coupling must coincide with this."""
    face = _driveline_diff_input_flange_local()
    if face is None:
        return None
    R = rot_x(-90.0)
    o = [_axle_x(p, axle), 0.0, p.layout.tyre_radius_mm]
    return _xform(R, o, face)


def suspension_hardpoint_world(p: VehicleParams, axle: str, side_sign: float,
                               name: str) -> Optional[List[float]]:
    """A suspension inboard hardpoint in VEHICLE coordinates (corner placed at
    HUB_CENTRE; left = identity, right = Rz(180))."""
    try:
        from suspension_nx.engineering import hardpoints as s_hardpoints
        from suspension_nx.params import SuspensionParams
        hp = s_hardpoints(SuspensionParams())
    except Exception:
        return None
    if name not in hp:
        return None
    R = identity() if side_sign > 0 else rot_z(180.0)
    o = hub_centre(p, axle, side_sign)
    return _xform(R, o, list(hp[name]))


def subframe_point_world(p: VehicleParams, axle: str, kind: str, name: str,
                         side: str) -> Optional[List[float]]:
    """A subframe feature centre in VEHICLE coordinates. ``kind`` in {"pickup","pad",
    "tower"}: the pickup bosses + tower come from the subframe hardpoint table, the
    chassis pads from pad_centre_local. The subframe is placed IDENTITY at the axle
    x-station, so vehicle = local + (axle_x, 0, 0)."""
    try:
        from subframe_nx.params import SubframeParams
        sp = SubframeParams().overridden(**{"axle": axle})
    except Exception:
        return None
    o = [_axle_x(p, axle), 0.0, 0.0]
    s = side.lower()            # subframe accessors key on lowercase 'l'/'r'
    if kind == "pad":
        loc = sp.pad_centre_local(name, s)
    else:
        loc = sp.hardpoints_local(s).get(name)
        if loc is None:
            return None
    return [o[i] + loc[i] for i in range(3)]


def chassis_pad_world(axle: str, fore_aft: str, side: str) -> Optional[List[float]]:
    """The chassis subframe mount-pad centre (one of the FOUR per axle) in VEHICLE
    coordinates (the chassis is identity at the origin, so this is already a vehicle
    coordinate)."""
    try:
        from chassis_nx.blueprint import subframe_pad_centre
        from chassis_nx.params import ChassisParams
        return list(subframe_pad_centre(ChassisParams(), axle, fore_aft, side))
    except Exception:
        return None


def _dist3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _chassis_rail_boxes() -> List[Dict[str, Any]]:
    """The chassis rail create-bodies as vehicle-frame boxes, each with the list of
    its axle-notch relief windows (also boxes). A point clears the rail if it is
    OUTSIDE every rail box OR inside one of that rail's notch windows."""
    from chassis_nx.blueprint import build_steps
    from chassis_nx.params import ChassisParams
    steps = build_steps(ChassisParams())

    def _prism_box(s) -> Dict[str, float]:
        pts = _step_world_points(s.as_dict())
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
        return {"x": (min(xs), max(xs)), "y": (min(ys), max(ys)), "z": (min(zs), max(zs))}

    rails = {s.id: {"box": _prism_box(s), "notches": []}
             for s in steps if s.boolean == "create" and s.role == "frame_rail"}
    for s in steps:
        if s.role == "axle_notch_cut" and s.target in rails:
            rails[s.target]["notches"].append(_prism_box(s))
    return list(rails.values())


def _point_collides_rail(pt: List[float], rails: List[Dict[str, Any]], pad: float = 0.0) -> bool:
    """True if `pt` is inside a rail box and NOT inside one of that rail's notch
    windows. `pad` shrinks the box slightly so a body merely TOUCHING a face (the
    butt/seat case) is not flagged as interference."""
    x, y, z = pt

    def _in(box, slack):
        return (box["x"][0] + slack <= x <= box["x"][1] - slack
                and box["y"][0] + slack <= y <= box["y"][1] - slack
                and box["z"][0] + slack <= z <= box["z"][1] - slack)
    for rail in rails:
        if _in(rail["box"], pad):
            if any(_in(n, -1e-6) for n in rail["notches"]):
                continue
            return True
    return False


def _chassis_dims() -> Dict[str, float]:
    """Chassis overall length, frame inner width, battery-tray width + clearance and
    the rail centre-line Y -- read from chassis_nx so the fit/bracket checks use the
    geometry the chassis actually builds."""
    from chassis_nx.blueprint import rail_centreline_y
    from chassis_nx.params import ChassisParams
    cp = ChassisParams()
    fr, bt = cp.frame, cp.battery_tray
    return {
        "overall_length_mm": float(fr.overall_length_mm),
        "wheelbase_mm": float(fr.wheelbase_mm),
        "frame_inner_width_mm": float(fr.frame_inner_width_mm),
        "tray_width_mm": float(bt.width_mm) if bt.enabled else 0.0,
        "tray_side_clearance_mm": float(bt.side_clearance_mm),
        "rail_centreline_y_mm": float(rail_centreline_y(cp)),
    }


# --------------------------------------------------------------------------- #
# checks + report
# --------------------------------------------------------------------------- #
def validate(p: VehicleParams) -> List[str]:
    """Layout sanity + the ICD §4 dimensional-consistency rules.

    Rules (ICD §4):
      1. driveline built track == LayoutParams.track_* (+-2 %).
      2. suspension hub-centre local Y == 0 (re-datumed on the bore) and the corner's
         built half-track == T/2; the hub-bore OD matches the driveline bearing OD.
      3. chassis overall length >= wheelbase + overhangs; frame_inner_width >= battery
         tray width + clearance; rail spacing brackets the +-T/2 hub/mount stations.
      4. no two non-chassis components share an origin.
      5. motor + inverter clear the ground.

    Plus the ICD §7 INTEGRATION acceptance checks:
      7a. HEADLINE: no two non-chassis component SOLIDS interpenetrate (the sampled-solid
          AABB-tightened overlap test). The reduction gearbox bridging motor<->diff and
          the cleared motor offset make the motor no longer pierce the differential.
      7b. MATING coincidences: gearbox motor-flange FACE == motor DE flange FACE (FULL 3D,
          incl. the axial Y -- review finding 1), gearbox output coupling == the driveline
          diff INPUT flange (3D -- review finding 2), gearbox diff mount on the diff axis,
          the gearbox<->motor axial engagement within a flange/pilot budget (no burial --
          review finding 4), subframe pads == chassis pads, subframe pickup bosses ==
          suspension inboard pickups, subframe tower top == damper/strut top.
      7c. END-TO-END REDUCTION: gearbox total_ratio x driveline diff ratio in ~9-10:1
          (the reduction lives once, in the gearbox -- review finding 3).
    """
    issues: List[str] = []
    L = p.layout
    if L.drive_layout not in ("rear", "front", "awd"):
        issues.append("layout.drive_layout '%s' unknown (rear|front|awd)" % L.drive_layout)
    if L.suspension_corners not in (2, 4):
        issues.append("layout.suspension_corners must be 2 or 4")
    for nm, v in (("wheelbase_mm", L.wheelbase_mm), ("track_front_mm", L.track_front_mm),
                  ("track_rear_mm", L.track_rear_mm), ("tyre_radius_mm", L.tyre_radius_mm)):
        if v <= 0:
            issues.append("layout.%s must be > 0" % nm)
    if issues:
        # bad primitives make the geometry probes meaningless; bail before them
        return issues

    driven = _driven_axles(L.drive_layout)
    tracks = {ax: _track(p, ax) for ax in driven}

    # --- ICD §4.1: driveline built track ~= vehicle track (+-2 %) ---------- #
    try:
        built = _driveline_built_track_mm()
        for ax in driven:
            tgt = tracks[ax]
            err = 100.0 * (built - tgt) / tgt if tgt else 0.0
            if abs(err) > _TRACK_TOL_PCT:
                issues.append(
                    "driveline built track %.0f mm is %.1f%% off the %s vehicle track "
                    "%.0f mm (> %.0f%%); the flange faces will not land on +-T/2 -- retarget "
                    "DrivelineParams.target_track_mm" % (built, err, ax, tgt, _TRACK_TOL_PCT))
    except Exception as exc:                                   # pragma: no cover
        issues.append("could not read driveline built track for ICD §4.1 check: %s" % exc)

    # --- ICD §4.2: suspension hub centre on the local origin; corner spans T/2 - #
    try:
        hub_y = _suspension_hub_local_y_mm()
        if abs(hub_y) > _HUB_Y_TOL_MM:
            issues.append(
                "suspension hub-centre local Y %.1f mm != 0; the corner is no longer "
                "datumed on the hub bore, so placing the origin at HUB_CENTRE would "
                "double-count the offset" % hub_y)
        half = _suspension_built_half_track_mm()
        for ax in (["front", "rear"] if L.suspension_corners >= 4 else driven):
            tgt_half = _track(p, ax) / 2.0
            if abs(half - tgt_half) > max(_HUB_Y_TOL_MM, _TRACK_TOL_PCT / 100.0 * tgt_half):
                issues.append(
                    "suspension built half-track %.0f mm != %s T/2 %.0f mm; the corner "
                    "envelope will not reach the wheel" % (half, ax, tgt_half))
        # the hub bore must match the driveline hub-bearing OD (they share the wheel hub)
        s_bore, d_bore = _suspension_hub_bore_od_mm(), _driveline_hub_bore_od_mm()
        if abs(s_bore - d_bore) > 1e-6:
            issues.append(
                "suspension hub-bore OD %.1f mm != driveline hub-bearing OD %.1f mm; the "
                "upright cannot carry the driveline hub" % (s_bore, d_bore))
    except Exception as exc:                                   # pragma: no cover
        issues.append("could not read suspension geometry for ICD §4.2 check: %s" % exc)

    # --- ICD §4.3: chassis length / tray fit / rail spacing brackets +-T/2 --- #
    if p.parts.include_chassis:
        try:
            ch = _chassis_dims()
            # chassis must at least span the wheelbase + a representative overhang each end
            if ch["overall_length_mm"] < L.wheelbase_mm:
                issues.append(
                    "chassis overall length %.0f mm < wheelbase %.0f mm (cannot reach both "
                    "axle stations)" % (ch["overall_length_mm"], L.wheelbase_mm))
            if abs(ch["wheelbase_mm"] - L.wheelbase_mm) > 1.0:
                issues.append(
                    "chassis wheelbase %.0f mm != vehicle wheelbase %.0f mm; the subframe "
                    "pads will not sit at the axle stations" % (ch["wheelbase_mm"], L.wheelbase_mm))
            # the battery tray must fit the inner rail channel with side clearance
            need = ch["tray_width_mm"] + 2.0 * ch["tray_side_clearance_mm"]
            if ch["tray_width_mm"] > 0 and need > ch["frame_inner_width_mm"]:
                issues.append(
                    "battery tray width %.0f mm + 2x%.0f clearance = %.0f mm exceeds the "
                    "frame inner channel %.0f mm" % (ch["tray_width_mm"],
                    ch["tray_side_clearance_mm"], need, ch["frame_inner_width_mm"]))
            # the rails (and their subframe pads) must sit INBOARD of the +-T/2 hubs so
            # the wheel/suspension corner envelope is clear -- 0 < rail_cy < T/2.
            rail_cy = ch["rail_centreline_y_mm"]
            min_half_track = min(L.track_front_mm, L.track_rear_mm) / 2.0
            if not (0.0 < rail_cy < min_half_track):
                issues.append(
                    "rail centre-line Y %.0f mm does not bracket the hub track (need "
                    "0 < y < T/2 = %.0f mm)" % (rail_cy, min_half_track))
        except Exception as exc:                               # pragma: no cover
            issues.append("could not read chassis geometry for ICD §4.3 check: %s" % exc)

    # --- ICD §4.2/§4.3: suspension + driveline envelopes clear the chassis rail --- #
    # The corner links and the half-shaft sweep from inboard of the rail out to the
    # wheel, so they cross the rail's Y band at the hub-centre height. Transform the
    # actual subsystem point clouds into vehicle coords per corner and assert they
    # clear the rail box (the rail is relieved by the axle notch at each axle station).
    if p.parts.include_chassis:
        try:
            rails = _chassis_rail_boxes()
            corner_axles = ["front", "rear"] if L.suspension_corners >= 4 else driven
            for ax in corner_axles:
                for side, sign in (("L", +1.0), ("R", -1.0)):
                    hits = sum(1 for pt in _suspension_corner_world_points(p, ax, sign)
                               if _point_collides_rail(pt, rails, pad=1.0))
                    if hits:
                        issues.append(
                            "suspension corner %s%s envelope pierces the chassis rail at the "
                            "%s axle (%d sampled points inside the rail box) -- relieve the rail "
                            "(axle notch / kick-up) or move the pickups inboard of the rail"
                            % (ax[0].upper(), side, ax, hits))
            for ax in driven:
                hits = sum(1 for pt in _driveline_world_points(p, ax)
                           if _point_collides_rail(pt, rails, pad=1.0))
                if hits:
                    issues.append(
                        "driveline half-shaft envelope pierces the chassis rail at the %s axle "
                        "(%d sampled points inside the rail box) -- relieve the rail (axle notch) "
                        "so the half-shaft passes through" % (ax, hits))
        except Exception as exc:                                   # pragma: no cover
            issues.append("could not run the rail-clearance check (ICD §4.2/§4.3): %s" % exc)

    # --- ICD §4.4: no two non-chassis components share an origin ------------ #
    # The driveline and the gearbox are BOTH datumed on the differential axis (ICD §7.1:
    # the gearbox local origin IS the diff axis), so they intentionally share the diff
    # station origin -- the gearbox output couples directly to the driveline diff input.
    # That coaxial-by-design pair is exempt; any OTHER coincident origin is still a bug.
    _coaxial_diff = {"driveline", "gearbox"}
    seen: Dict[tuple, str] = {}
    seen_role: Dict[tuple, str] = {}
    for c in components(p):
        if c["role"] == "chassis":
            continue
        key = tuple(c["origin_mm"])
        if key in seen:
            if not ({c["role"], seen_role[key]} <= _coaxial_diff
                    and _axle_suffix(c["name"]) == _axle_suffix(seen[key])):
                issues.append("components %s and %s share an origin %s"
                              % (seen[key], c["name"], key))
        else:
            seen[key] = c["name"]
            seen_role[key] = c["role"]

    # --- ICD §4.5: motor + inverter clear the ground ------------------------ #
    off = motor_offset(p)              # resolved (gearbox-derived AUTO) e-axle offset
    motor_axis_z = L.tyre_radius_mm + off["dz"]
    try:
        env_r = _motor_envelope_radius_mm()
        if motor_axis_z - env_r <= 0.0:
            issues.append(
                "motor envelope reaches the ground: axle z %.0f - envelope radius %.0f "
                "<= 0 (raise eaxle.motor_offset_z_mm)" % (motor_axis_z, env_r))
    except Exception:                                          # pragma: no cover
        # fall back to the axle-height check if the motor package is unavailable
        if motor_axis_z <= 0:
            issues.append("motor placed below ground (check eaxle.motor_offset_z_mm)")
    if p.parts.include_inverter:
        inverter_base_z = motor_axis_z + p.eaxle.inverter_offset_z_mm
        # the inverter is modelled with its base at local z=0 (all geometry z>=0), so
        # the placement origin is the contact face -- it clears ground iff base_z > 0.
        if inverter_base_z <= 0.0:
            issues.append(
                "inverter base z %.0f <= 0 (it would sit at/under ground); raise the "
                "motor/inverter z offsets" % inverter_base_z)

    # ===================================================================== #
    # ICD §7 -- INTEGRATION acceptance: no interpenetration + real matings   #
    # ===================================================================== #
    # (a) HEADLINE (HONEST): the void-aware sampled-solid clash test run between EVERY
    #     component pair INCLUDING the chassis (its axle notch + bores are real voids),
    #     with only the bolted integrated e-axle/hub unit on the allowlist. This replaces
    #     the older chassis-excluding / mating-skipping check that FALSE-PASSED while five
    #     real clashes (half-shaft/diff into the rail, the subframe cradle into the control
    #     arms, the cradle into the e-axle) sat in the assembled vehicle.
    try:
        for hit in comprehensive_interpenetration_pairs(p):
            pen = hit.get("penetration_mm", [0.0])
            issues.append(
                "ICD §7.1 INTERPENETRATION: %s and %s overlap by %.1f mm solid -- the "
                "components clash (relieve the rail notch / reshape the subframe / raise "
                "the e-axle offset)" % (hit["a"], hit["b"], pen[0] if pen else 0.0))
    except Exception as exc:                                   # pragma: no cover
        issues.append("could not run the ICD §7 interpenetration check: %s" % exc)

    # (c) END-TO-END REDUCTION: the motor->wheel ratio = gearbox total_ratio x driveline
    #     diff ratio must land in the ~9-10:1 single-speed EV band. The gearbox PROVIDES
    #     the reduction (ICD §7.1) and the differential is a TRUE 1:1 diff -- this guards
    #     against the two packages double-counting the final drive (review finding 3).
    issues.extend(_total_ratio_issues(p))

    # (b) MATING coincidences (ICD §7.4.2). Each within _MATE_TOL_MM.
    issues.extend(_mating_issues(p, driven))
    return issues


# ICD §7.1 end-to-end reduction band (motor -> wheel). A passenger EV is single-speed
# ~9-10:1; we tolerate 8-11:1 (the same band gearbox_nx.engineering enforces on its own
# total_ratio). Outside this band the gearbox + differential are double-counting (or
# under-providing) the final drive.
_TOTAL_RATIO_BAND = (8.0, 11.0)


def vehicle_total_ratio(p: VehicleParams) -> Optional[float]:
    """The end-to-end motor->wheel reduction: gearbox total_ratio x driveline diff ratio.
    None if either package is unavailable (NX-free deferred import)."""
    try:
        from gearbox_nx.engineering import derive as g_derive
        from gearbox_nx.params import GearboxParams
        from driveline_nx.params import DrivelineParams
        g_ratio = float(g_derive(GearboxParams()).total_ratio)
        diff_ratio = float(DrivelineParams().differential.final_drive_ratio)
        return g_ratio * diff_ratio
    except Exception:
        return None


def _total_ratio_issues(p: VehicleParams) -> List[str]:
    if not p.parts.include_gearbox:
        return []
    total = vehicle_total_ratio(p)
    if total is None:
        return []
    lo, hi = _TOTAL_RATIO_BAND
    if not (lo <= total <= hi):
        return ["ICD §7.1 REDUCTION: end-to-end motor->wheel ratio %.2f:1 is outside the "
                "~9-10:1 single-speed band (%.0f-%.0f tolerated) -- the gearbox reduction "
                "and the differential final drive are double-counting (set "
                "DrivelineParams.differential.final_drive_ratio = 1.0 so the diff is a "
                "true differential and the gearbox alone provides the reduction)"
                % (total, lo, hi)]
    return []


# the gearbox<->motor joint is a bolted flange BUTT + a pilot spigot register. The two
# flanges (motor DE flange + gearbox motor flange) seat face-to-face and the spigot pilots
# a few mm; the motor's DE housing-flange disc also seats. Allow the axial engagement up to
# this budget (mm) -- beyond it the gearbox has slid OVER the motor barrel (burial).
_MATING_ENGAGEMENT_BUDGET_MM = 70.0


def _mating_engagement_issues(p: VehicleParams, axle: str) -> List[str]:
    """Bounded mating-engagement check (review finding 4) for the gearbox<->motor butt
    joint: the axial (Y, the e-axle axis) extent over which the two parts' solids overlap
    must stay within a flange/pilot seating budget. A clean butt is a flange+pilot depth;
    a connector that has buried itself over the neighbour spans its whole barrel."""
    gb = next((c for c in components(p)
               if c["role"] == "gearbox" and _axle_suffix(c["name"]) == axle.upper()), None)
    mot = next((c for c in components(p)
                if c["role"] == "motor" and _axle_suffix(c["name"]) == axle.upper()), None)
    if gb is None or mot is None:
        return []
    sg, sm = _component_solids(gb), _component_solids(mot)
    if sg is None or sm is None:
        return []
    eng = clearance.solids_axial_engagement(sg, sm, axis_index=1)   # Y = the axle axis
    if eng is not None and eng > _MATING_ENGAGEMENT_BUDGET_MM:
        return ["ICD §7.1 MATING BURIAL: gearbox %s housing engages the motor by %.0f mm "
                "axially (> %.0f mm flange/pilot budget) -- the gearbox has slid OVER the "
                "motor barrel instead of butting its DE flange (raise the e-axle offset / "
                "axial standoff)" % (axle, eng, _MATING_ENGAGEMENT_BUDGET_MM)]
    return []


def _mating_issues(p: VehicleParams, driven: List[str]) -> List[str]:
    """The ICD §7.4.2 mating-coincidence checks: gearbox motor flange <-> motor DE
    flange (coaxial), gearbox output/diff mount on the diff axis, subframe pads <->
    chassis pads, subframe pickup bosses <-> suspension inboard pickups, subframe tower
    top <-> suspension damper/strut top. NX-free; tolerances reflect the representative
    placement (the real ties bolt in a downstream NX constraint step)."""
    issues: List[str] = []
    L = p.layout
    f = p.parts

    # --- gearbox <-> motor + gearbox <-> diff (the e-axle spine) ------------ #
    if f.include_gearbox:
        for ax in driven:
            try:
                # (1) the gearbox motor-mounting FLANGE FACE must COINCIDE with the motor
                # DE flange face in FULL 3D -- not merely share the X/Z axis. The earlier
                # check compared only X/Z of the axis and skipped Y, so a 187 mm axial gap
                # (the gearbox flange facing the motor's NON-drive end) passed silently
                # (review finding 1). Assert all three axes now.
                gb_face = gearbox_iface_world(p, ax, "motor_flange_face")
                m_face = motor_de_flange_world(p, ax)
                if m_face is not None:
                    d3 = _dist3(gb_face, m_face)
                    if d3 > _MATE_TOL_MM:
                        issues.append(
                            "ICD §7.4.2: gearbox %s motor flange face is %.0f mm (3D) from "
                            "the motor DE flange face -- the gearbox does not meet the motor "
                            "(gap / wrong-end orientation)" % (ax, d3))
                # (2) the gearbox OUTPUT coupling must COINCIDE (3D) with the driveline diff
                # INPUT flange the driveline actually models -- not merely 'on the diff
                # axis'. Before, the output sat on the diff axis while the driveline input
                # was a 127 mm off-axis input pinion, so they never coupled (review finding
                # 2). The driveline diff input is now coaxial (1:1 diff), so they meet.
                di_face = driveline_diff_input_world(p, ax)
                oc_face = gearbox_iface_world(p, ax, "output_coupling_face")
                if di_face is not None:
                    d3 = _dist3(oc_face, di_face)
                    if d3 > _MATE_TOL_MM:
                        issues.append(
                            "ICD §7.4.2: gearbox %s output coupling is %.0f mm (3D) from the "
                            "driveline diff input flange -- the gearbox output does not couple "
                            "to the diff" % (ax, d3))
                # the diff-carrier mount must still sit ON the diff axis (X/Z) -- it bores
                # the carrier OD coaxially.
                ax_x, z_hub = _axle_x(p, ax), L.tyre_radius_mm
                dm = gearbox_iface_world(p, ax, "diff_mount_face")
                d = math.hypot(dm[0] - ax_x, dm[2] - z_hub)
                if d > _MATE_TOL_MM:
                    issues.append(
                        "ICD §7.4.2: gearbox %s diff-carrier mount is %.0f mm off the "
                        "differential axis (the carrier will not seat)" % (ax, d))
                # (3) BOUNDED MATING ENGAGEMENT (review finding 4): the gearbox must BUTT
                # the motor at the flange, not slide its housing OVER the motor barrel. The
                # blanket mating skip hid a 149 mm coaxial burial; assert the axial (Y)
                # engagement of the gearbox<->motor solids stays within a flange/pilot
                # seating budget.
                issues.extend(_mating_engagement_issues(p, ax))
            except Exception as exc:                           # pragma: no cover
                issues.append("could not run the gearbox mating check at %s: %s" % (ax, exc))

    # --- subframe <-> chassis pads + bosses <-> suspension + tower <-> damper - #
    if f.include_subframe:
        sub_axles = ["front", "rear"] if L.suspension_corners >= 4 else driven
        for ax in sub_axles:
            # subframe chassis pads coincide with the chassis subframe mount pads. The
            # the chassis now builds FOUR pads per axle (fore + aft, l + r) that COINCIDE
            # with the subframe's four pad flanges. Verify each subframe pad lands on a
            # chassis pad in full X + |Y| + Z (the l/r label is flipped between the two
            # packages -- chassis rail 'l' is -Y, subframe 'l' is +Y -- so match on |Y|).
            if f.include_chassis:
                try:
                    ch_pads = [chassis_pad_world(ax, fa, s)
                               for fa in ("fore", "aft") for s in ("l", "r")]
                    ch_pads = [c for c in ch_pads if c is not None]
                    for s in ("l", "r"):
                        for fa in ("fore", "aft"):
                            sub = subframe_point_world(p, ax, "pad", fa, s)
                            if sub is None or not ch_pads:
                                continue
                            landed = any(
                                abs(c[0] - sub[0]) <= _MATE_TOL_MM
                                and abs(abs(c[1]) - abs(sub[1])) <= _MATE_TOL_MM
                                and abs(c[2] - sub[2]) <= _MATE_TOL_MM
                                for c in ch_pads)
                            if not landed:
                                issues.append(
                                    "ICD §7.4.2: subframe %s %s/%s pad (%.0f, %.0f, %.0f) "
                                    "does not land on any chassis rail-top mount pad -- the "
                                    "cradle would not bolt to the chassis"
                                    % (ax, fa, s, sub[0], sub[1], sub[2]))
                except Exception as exc:                       # pragma: no cover
                    issues.append("could not run the subframe<->chassis pad check at %s: %s" % (ax, exc))

            # subframe pickup bosses coincide with the suspension inboard hardpoints, and
            # the subframe tower top coincides with the suspension damper/strut top --
            # asserted on ALL FOUR CORNERS by EXACT hardpoint NAME (no nearest-pickup
            # fudge, no representative-corner-only short-cut).
            #
            # The earlier check asserted ONE 'convention-compatible' corner per axle and
            # matched via nearest pickup, which had no teeth: the subframe built its bosses
            # from its OWN independently-mirrored table that only AGREED with the suspension
            # on the rear-left corner, so three of the four corners were ~225-890 mm off and
            # still passed. The subframe now DERIVES its bosses from the suspension hardpoint
            # table using the SAME placement convention (subframe_nx sources
            # suspension_nx.engineering.hardpoints), so every named boss lands exactly on its
            # suspension pickup; assert that per corner so a future drift is caught.
            sus_names = ("lower_pickup_fore", "lower_pickup_aft",
                         "upper_pickup_fore", "upper_pickup_aft", "toe_pickup")
            for side, sign in (("l", +1.0), ("r", -1.0)):
                for nm in sus_names:
                    sub = subframe_point_world(p, ax, "pickup", nm, side)
                    sus = suspension_hardpoint_world(p, ax, sign, nm)
                    if sub is None or sus is None:
                        continue
                    d = _dist3(sub, sus)
                    if d > _SUBFRAME_COINCIDENCE_TOL_MM:
                        issues.append(
                            "ICD §7.4.2: subframe %s %s boss %s is %.1f mm from the "
                            "suspension inboard pickup -- the arm would float"
                            % (ax, side.upper(), nm, d))
                # damper / strut top <-> shock-tower boss (same corner)
                sub_t = subframe_point_world(p, ax, "tower", "damper_top", side)
                sus_t = suspension_hardpoint_world(p, ax, sign, "damper_top")
                if sub_t is not None and sus_t is not None:
                    d = _dist3(sub_t, sus_t)
                    if d > _SUBFRAME_COINCIDENCE_TOL_MM:
                        issues.append(
                            "ICD §7.4.2: subframe %s %s shock-tower top is %.1f mm from "
                            "the suspension damper/strut top -- the spring would float"
                            % (ax, side.upper(), d))
    return issues


def report(p: VehicleParams) -> str:
    plan = build_plan(p)
    L = p.layout
    lines = [
        "Vehicle assembly plan -- %s" % p.name,
        "  drive layout / corners   : %s, %d suspension corners" % (L.drive_layout, L.suspension_corners),
        "  wheelbase / track (f/r)  : %.0f / %.0f / %.0f mm" % (
            L.wheelbase_mm, L.track_front_mm, L.track_rear_mm),
        "  hub-centre height        : %.0f mm (tyre radius)" % L.tyre_radius_mm,
    ]
    # the shared hub stations all subsystems converge on (ICD §2)
    driven = _driven_axles(L.drive_layout)
    corner_axles = ["front", "rear"] if L.suspension_corners >= 4 else driven
    lines.append("  HUB_CENTRE stations      :")
    for ax in corner_axles:
        hl = hub_centre(p, ax, +1.0)
        hr = hub_centre(p, ax, -1.0)
        lines.append("    %-5s L (%7.1f,%7.1f,%7.1f) | R (%7.1f,%7.1f,%7.1f)" % (
            ax, hl[0], hl[1], hl[2], hr[0], hr[1], hr[2]))

    # ICD §4 consistency facts (best-effort; missing packages just omit a line)
    cons: List[str] = []
    try:
        built = _driveline_built_track_mm()
        cons.append("driveline built track    : %.0f mm (vehicle track %.0f mm)" % (
            built, _track(p, driven[0])))
    except Exception:
        pass
    try:
        cons.append("suspension hub local Y   : %.1f mm (0 => datumed on the bore); "
                    "half-track %.0f mm" % (_suspension_hub_local_y_mm(),
                                            _suspension_built_half_track_mm()))
        cons.append("hub bore OD (susp/drv)   : %.1f / %.1f mm" % (
            _suspension_hub_bore_od_mm(), _driveline_hub_bore_od_mm()))
    except Exception:
        pass
    off = motor_offset(p)
    try:
        env_r = _motor_envelope_radius_mm()
        cons.append("motor axis z / clearance : %.0f mm / %.0f mm to ground (env r %.0f)" % (
            L.tyre_radius_mm + off["dz"],
            L.tyre_radius_mm + off["dz"] - env_r, env_r))
    except Exception:
        pass
    if cons:
        lines.append("  ICD §4 consistency:")
        lines.extend("    %s" % c for c in cons)

    # ICD §7 integration facts: the cleared e-axle offset + the interpenetration result
    lines.append("  ICD §7 integration:")
    lines.append("    motor offset (dx,dz)     : (%.1f, %.1f) mm from the wheel/diff axis %s" % (
        off["dx"], off["dz"],
        "(gearbox-derived AUTO)" if not (p.eaxle.motor_offset_x_mm and p.eaxle.motor_offset_z_mm)
        else "(override)"))
    try:
        from gearbox_nx.engineering import derive as _gd
        from gearbox_nx.params import GearboxParams as _GP
        g = _gd(_GP())
        lines.append("    e-axle centre distance   : %.1f mm vs ICD min %.1f mm -> margin %.1f mm %s" % (
            g.motor_offset_mm, g.icd_min_centre_distance_mm, g.centre_distance_margin_mm,
            "(CLEARS)" if g.centre_distance_margin_mm >= 0 else "(OVERLAP!)"))
    except Exception:
        pass
    try:
        hits = comprehensive_interpenetration_pairs(p)
        if hits:
            lines.append("    interpenetration         : %d pair(s) CLASH" % len(hits))
            for h in hits:
                pen = h.get("penetration_mm", [0.0])
                lines.append("      %s <-> %s : %.1f mm" % (h["a"], h["b"], pen[0] if pen else 0.0))
        else:
            lines.append("    interpenetration         : none (no two component solids overlap)")
    except Exception:
        pass

    lines.append("  components (%d):" % len(plan["components"]))
    for c in plan["components"]:
        ox, oy, oz = c["origin_mm"]
        lines.append("    %-16s %-18s @ (%7.1f, %7.1f, %7.1f)" % (
            c["name"], c["part_file"], ox, oy, oz))
    issues = plan["validation"]
    lines.append("  validation: %s" % ("OK (layout is consistent)" if not issues else "%d issue(s)" % len(issues)))
    for it in issues:
        lines.append("    - %s" % it)
    return "\n".join(lines)
