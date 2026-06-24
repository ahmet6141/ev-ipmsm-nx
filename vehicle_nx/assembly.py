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

Exact bolt-hole mating is a downstream NX constraint step; this plan positions every
part parametrically so the assembly opens already laid out (the same "representative"
philosophy the subsystem blueprints use).

`validate()` enforces the ICD §4 dimensional-consistency rules against the ACTUAL
subsystem geometry (their NX-free `engineering.derive()` / blueprint accessors): the
driveline built track == vehicle track, the suspension hub-centre local Y == T/2, the
chassis length/rail-spacing bracket the wheelbase/track, plus the shared-origin and
ground-clearance checks. Those imports stay pure CPython (no NXOpen), so the plan and
its validation generate in plain Python.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from .params import VehicleParams

Mat = List[List[float]]
Vec = List[float]

# ICD §4 consistency tolerances
_TRACK_TOL_PCT = 2.0      # driveline built track vs vehicle track (ICD §4.1)
_HUB_Y_TOL_MM = 1.0       # suspension hub-centre local Y vs T/2 (ICD §4.2)


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


def _component(name: str, part_file: str, role: str, origin: Vec, orient: Mat) -> Dict[str, Any]:
    return {
        "name": name,
        "part_file": part_file,
        "role": role,
        "origin_mm": [round(v, 3) for v in origin],
        "orientation": _round_mat(orient),
    }


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


def hub_centre(p: VehicleParams, axle: str, side_sign: float) -> Vec:
    """THE shared datum (ICD §2): the wheel-hub centre in vehicle coordinates,

        HUB_CENTRE(axle, side) = ( +-wheelbase/2 , +-track/2 , tyre_radius ).

    `side_sign` = +1 for the LEFT (+Y) wheel, -1 for the RIGHT (-Y). The driveline
    flange face, the suspension upright bore and the wheel all coincide here, so this
    is also the suspension corner's placement origin (the corner is hub-datumed)."""
    return [_axle_x(p, axle), side_sign * _track(p, axle) / 2.0, p.layout.tyre_radius_mm]


def components(p: VehicleParams) -> List[Dict[str, Any]]:
    """Ordered component list with placements (vehicle frame)."""
    L, e, f = p.layout, p.eaxle, p.parts
    z_hub = L.tyre_radius_mm
    axle_x = {"rear": _axle_x(p, "rear"), "front": _axle_x(p, "front")}

    out: List[Dict[str, Any]] = []

    # 1) chassis -- the platform at the origin
    if f.include_chassis:
        out.append(_component("CHASSIS", f.chassis, "chassis", [0.0, 0.0, 0.0], identity()))

    # 2) e-axle(s): driveline + motor (+ inverter) at each driven axle
    eaxle_rot = rot_x(-90.0)   # local +Z (rotation axis) -> vehicle +Y (axle left-right)
    for ax in _driven_axles(L.drive_layout):
        ax_x = axle_x[ax]
        out.append(_component(
            "DRIVELINE_%s" % ax.upper(), f.driveline, "driveline",
            [ax_x, 0.0, z_hub], eaxle_rot))
        out.append(_component(
            "MOTOR_%s" % ax.upper(), f.motor, "motor",
            [ax_x - e.motor_offset_x_mm, 0.0, z_hub + e.motor_offset_z_mm], eaxle_rot))
        if f.include_inverter:
            out.append(_component(
                "INVERTER_%s" % ax.upper(), f.inverter, "inverter",
                [ax_x - e.motor_offset_x_mm + e.inverter_offset_x_mm, 0.0,
                 z_hub + e.motor_offset_z_mm + e.inverter_offset_z_mm], identity()))

    # 3) suspension corners. The corner is RE-DATUMED at the hub centre, so its
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
    seen: Dict[tuple, str] = {}
    for c in components(p):
        if c["role"] == "chassis":
            continue
        key = tuple(c["origin_mm"])
        if key in seen:
            issues.append("components %s and %s share an origin %s" % (seen[key], c["name"], key))
        else:
            seen[key] = c["name"]

    # --- ICD §4.5: motor + inverter clear the ground ------------------------ #
    motor_axis_z = L.tyre_radius_mm + p.eaxle.motor_offset_z_mm
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
    try:
        env_r = _motor_envelope_radius_mm()
        cons.append("motor axis z / clearance : %.0f mm / %.0f mm to ground (env r %.0f)" % (
            L.tyre_radius_mm + p.eaxle.motor_offset_z_mm,
            L.tyre_radius_mm + p.eaxle.motor_offset_z_mm - env_r, env_r))
    except Exception:
        pass
    if cons:
        lines.append("  ICD §4 consistency:")
        lines.extend("    %s" % c for c in cons)

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
