"""NX-FREE vehicle-frame clearance check: axis-aligned bounding-box (AABB) overlap
between placed components, the headline ICD §7.1/§7.4.1 acceptance test ("no two
component solids may interpenetrate").

The check works entirely off each subsystem's pure-math blueprint (the ordered
:class:`motor_nx.blueprint.BuildStep` list) and the vehicle-frame placement (a 3x3
orientation ``R`` + ``origin`` vector) the assembly plan records. It never imports
NXOpen, so it runs under plain CPython and is unit-tested alongside the assembly.

Algorithm (ICD §7.6 reference, extended to EVERY build-step kind)
    local_bbox(blueprint): the min/max over the create + unite bodies in the part's
    OWN local frame. ``subtract`` bodies (bores, cavities, bolt holes, notches) only
    REMOVE material, so they never extend the solid envelope and are skipped. Per kind:

      * cylinder / tube : a capped cylinder. Honours ``origin3`` + ``axis`` (a body
        coaxial with an arbitrary axis) OR the legacy ``(cx, cy, z0)`` +Z column. The
        radial extent is sampled at both end caps along two axis-perpendicular
        directions (the same envelope the assembly's point-cloud helpers use).
      * hole           : a cylindrical body on an arbitrary axis (base ``(cx, cy, z0)``,
        direction ``axis``). Almost always a subtract; the rare ``unite`` boss (e.g. the
        motor terminal boss) is bounded like a cylinder so it is not missed.
      * extrude        : a closed XY polygon swept +Z from ``z0`` by ``length``.
      * prism          : a closed local-(u, v) polygon placed at ``origin3`` and swept
        along ``axis`` by ``length`` -- mapped to world via
        :func:`motor_nx.blueprint.profile_to_world` (the pure-math twin of the NX frame).
      * revolve        : a closed (r, z) polygon revolved 360 deg about Z; the radial
        extent is ``max|r|`` and the axial extent the profile's z-range.

    veh_aabb = transform the 8 corners of the local bbox by the component orientation
    ``R`` and add ``origin``, then take the min/max -- the smallest AABB in VEHICLE
    coordinates that contains the rotated local box (a conservative envelope: it can
    only over-report overlap, never miss a real one).

    overlap(a, b): the boxes overlap on an axis iff their intervals intersect; the
    per-axis overlap amount is ``min(a.hi, b.hi) - max(a.lo, b.lo)`` (negative = a gap).
    The boxes interpenetrate iff ALL three axes overlap by MORE than ``touch_tol`` --
    bolt-flange faces and pilots that merely touch (a few mm) are an intended mating
    contact, not a clash, so a small tolerance (~2 mm, ICD §7.6) is allowed.

This is intentionally conservative: an AABB over-bounds an oriented/round body, so a
PASS (no AABB overlap) is a hard guarantee of no solid interpenetration, while a small
reported AABB overlap between two NEIGHBOURING mating parts (gearbox flange touching
the motor face, a pad touching a chassis boss) can be a bounding-box artefact rather
than real metal overlap. The assembly's ``validate()`` therefore applies the check
between component PAIRS that should not touch at all (motor vs diff, motor vs subframe,
gearbox vs suspension, ...) and treats the chassis as the reference body.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

Vec = List[float]
Mat = List[List[float]]
Interval = Tuple[float, float]

# ICD §7.6: solid overlap beyond this (mm) is interpenetration; below it is a mating
# touch (bolt-flange contact, pilot register, butt joint) and is allowed.
TOUCH_TOL_MM = 2.0


# --------------------------------------------------------------------------- #
# small vector helpers (kept local so this module imports with zero deps)
# --------------------------------------------------------------------------- #
def _norm3(v: Sequence[float]) -> List[float]:
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def _rotate2d(points, angle_deg: float):
    """Rotate 2D (u, v) points about the local origin by angle_deg (CCW). Used to place a
    loft_twist section at its start / top rotation -- the pure-math twin of the NX builder's
    _rotate2d so the helical solid's envelope can be bounded NX-free."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return [(x * c - y * s, x * s + y * c) for (x, y) in points]


def _perp_frame(w: Sequence[float]) -> Tuple[List[float], List[float]]:
    """Two unit vectors perpendicular to the (already-normalised) axis ``w`` -- the
    radial sampling directions for a cylinder/tube/hole end cap."""
    helper = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = _norm3([w[1] * helper[2] - w[2] * helper[1],
                w[2] * helper[0] - w[0] * helper[2],
                w[0] * helper[1] - w[1] * helper[0]])
    v = [w[1] * u[2] - w[2] * u[1],
         w[2] * u[0] - w[0] * u[2],
         w[0] * u[1] - w[1] * u[0]]
    return u, v


# --------------------------------------------------------------------------- #
# per-build-step world (local-frame) point cloud
# --------------------------------------------------------------------------- #
def step_points(s: Dict[str, Any]) -> List[List[float]]:
    """Bounding points of ONE build-step body in the part's LOCAL frame.

    Returns the extreme points that bound the body's envelope (cylinder end-cap radial
    extremes, prism section corners at both ends, extrude/revolve profile extremes).
    A ``subtract`` body returns ``[]`` -- it removes material and never grows the solid.
    """
    if s.get("boolean") == "subtract":
        return []

    # import here so the module stays importable even if motor_nx is unavailable; it is
    # always present in this repo (the shared engine).
    from motor_nx.blueprint import profile_to_world

    kind = s.get("kind")
    pts: List[List[float]] = []

    if kind == "prism" and s.get("profile") is not None and s.get("origin3") is not None:
        near = profile_to_world([tuple(p) for p in s["profile"]],
                                tuple(s["origin3"]), tuple(s["axis"]),
                                tuple(s.get("u_dir", (1.0, 0.0, 0.0))))
        w = _norm3(s["axis"])
        L = s.get("length", 0.0)
        far = [[x + w[0] * L, y + w[1] * L, z + w[2] * L] for (x, y, z) in near]
        return [list(p) for p in near] + far

    if kind == "loft_twist" and s.get("profile") is not None and s.get("origin3") is not None:
        # a TRUE twisted solid: the LOCAL (u, v) profile, rotated start_twist_deg at the
        # base section and start_twist_deg + twist_deg at the top, swept along axis. Bound
        # it by the two end sections (rotation does not change the radial envelope, so the
        # bbox is the union of the two rotated sections placed at origin3 and origin3+axis*L).
        prof = [tuple(p) for p in s["profile"]]
        w = _norm3(s["axis"])
        L = s.get("length", 0.0)
        ud = tuple(s.get("u_dir", (1.0, 0.0, 0.0)))
        o3 = tuple(s["origin3"])
        top = (o3[0] + w[0] * L, o3[1] + w[1] * L, o3[2] + w[2] * L)
        start = s.get("start_twist_deg", 0.0)
        twist = s.get("twist_deg", 0.0)
        base = profile_to_world(_rotate2d(prof, start), o3, s["axis"], ud)
        far = profile_to_world(_rotate2d(prof, start + twist), top, s["axis"], ud)
        return [list(p) for p in base] + [list(p) for p in far]

    if kind in ("cylinder", "tube", "hole") and s.get("origin3") is not None:
        base = tuple(s["origin3"])
        w = _norm3(s["axis"])
        L = s.get("length", 0.0)
        r = s.get("outer_radius", 0.0)
        u, v = _perp_frame(w)
        for t in (0.0, L):
            c = [base[0] + w[0] * t, base[1] + w[1] * t, base[2] + w[2] * t]
            for d in (u, v):
                pts.append([c[0] + r * d[0], c[1] + r * d[1], c[2] + r * d[2]])
                pts.append([c[0] - r * d[0], c[1] - r * d[1], c[2] - r * d[2]])
        return pts

    if kind in ("cylinder", "tube"):
        # legacy +Z column at (cx, cy, z0)
        r = s.get("outer_radius", 0.0)
        cx, cy, z0, L = s.get("cx", 0.0), s.get("cy", 0.0), s.get("z0", 0.0), s.get("length", 0.0)
        for zz in (z0, z0 + L):
            for dx, dy in ((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)):
                pts.append([cx + dx, cy + dy, zz])
        return pts

    if kind == "hole":
        # arbitrary-axis hole given without origin3: base = (cx, cy, z0), dir = axis
        r = s.get("outer_radius", 0.0)
        base = (s.get("cx", 0.0), s.get("cy", 0.0), s.get("z0", 0.0))
        w = _norm3(s["axis"])
        L = s.get("length", 0.0)
        u, v = _perp_frame(w)
        for t in (0.0, L):
            c = [base[0] + w[0] * t, base[1] + w[1] * t, base[2] + w[2] * t]
            for d in (u, v):
                pts.append([c[0] + r * d[0], c[1] + r * d[1], c[2] + r * d[2]])
                pts.append([c[0] - r * d[0], c[1] - r * d[1], c[2] - r * d[2]])
        return pts

    if kind == "extrude" and s.get("profile") is not None:
        z0, L = s.get("z0", 0.0), s.get("length", 0.0)
        for (x, y) in s["profile"]:
            pts.append([x, y, z0])
            pts.append([x, y, z0 + L])
        return pts

    if kind == "revolve" and s.get("profile") is not None:
        rmax = max((abs(pt[0]) for pt in s["profile"]), default=0.0)
        zs = [pt[1] for pt in s["profile"]]
        z_lo, z_hi = (min(zs), max(zs)) if zs else (0.0, 0.0)
        for zz in (z_lo, z_hi):
            for dx, dy in ((rmax, 0.0), (-rmax, 0.0), (0.0, rmax), (0.0, -rmax)):
                pts.append([dx, dy, zz])
        return pts

    return pts


# --------------------------------------------------------------------------- #
# local bounding box of a whole blueprint
# --------------------------------------------------------------------------- #
def local_bbox(blueprint: Dict[str, Any]) -> Optional[Tuple[Vec, Vec]]:
    """(min_xyz, max_xyz) over every create/unite body of a blueprint, in the part's
    LOCAL frame. Returns ``None`` if the blueprint has no solid (only subtract) bodies."""
    lo = [math.inf, math.inf, math.inf]
    hi = [-math.inf, -math.inf, -math.inf]
    seen = False
    for s in blueprint.get("build_steps", []):
        for p in step_points(s):
            seen = True
            for i in range(3):
                if p[i] < lo[i]:
                    lo[i] = p[i]
                if p[i] > hi[i]:
                    hi[i] = p[i]
    if not seen:
        return None
    return lo, hi


# --------------------------------------------------------------------------- #
# transform a local bbox into vehicle coordinates
# --------------------------------------------------------------------------- #
def _xform(R: Mat, o: Vec, p: Sequence[float]) -> List[float]:
    return [o[i] + sum(R[i][k] * p[k] for k in range(3)) for i in range(3)]


def world_aabb(blueprint: Dict[str, Any], R: Mat, origin: Vec) -> Optional[Tuple[Vec, Vec]]:
    """Vehicle-frame AABB of a blueprint placed at ``origin`` with orientation ``R``.

    The 8 corners of the LOCAL bbox are transformed by ``R`` (vehicle = R . local) and
    offset by ``origin``; the AABB is the min/max of those 8 world corners. This is the
    smallest axis-aligned box that contains the rotated local box -- conservative
    (over-bounds the true oriented solid), so it can only over-report overlap."""
    lb = local_bbox(blueprint)
    if lb is None:
        return None
    (lx0, ly0, lz0), (lx1, ly1, lz1) = lb
    corners = [(x, y, z) for x in (lx0, lx1) for y in (ly0, ly1) for z in (lz0, lz1)]
    wlo = [math.inf, math.inf, math.inf]
    whi = [-math.inf, -math.inf, -math.inf]
    for c in corners:
        w = _xform(R, origin, c)
        for i in range(3):
            if w[i] < wlo[i]:
                wlo[i] = w[i]
            if w[i] > whi[i]:
                whi[i] = w[i]
    return wlo, whi


# --------------------------------------------------------------------------- #
# AABB overlap
# --------------------------------------------------------------------------- #
def overlap(a: Tuple[Vec, Vec], b: Tuple[Vec, Vec]) -> Optional[List[float]]:
    """Per-axis overlap amount of two vehicle-frame AABBs (each ``(min_xyz, max_xyz)``).

    Returns ``[dx, dy, dz]`` where ``d_i = min(a.hi, b.hi) - max(a.lo, b.lo)`` on axis
    ``i``. A positive value is solid overlap on that axis; a negative value is the gap.
    Returns ``None`` if either box is missing (a part with no solid body)."""
    if a is None or b is None:
        return None
    (alo, ahi), (blo, bhi) = a, b
    return [min(ahi[i], bhi[i]) - max(alo[i], blo[i]) for i in range(3)]


def interpenetrates(a: Tuple[Vec, Vec], b: Tuple[Vec, Vec],
                    touch_tol: float = TOUCH_TOL_MM) -> bool:
    """True iff two AABBs overlap on ALL three axes by MORE than ``touch_tol`` (mm).

    A bolt-flange mating contact touches over a few mm of bounding-box slop; only an
    overlap exceeding the tolerance on every axis is a genuine solid interpenetration."""
    ov = overlap(a, b)
    if ov is None:
        return False
    return all(d > touch_tol for d in ov)


# --------------------------------------------------------------------------- #
# SAMPLED-SOLID overlap -- the tighter ICD §7.6 "(better: sampled solid)" variant
#
# A pure whole-part (or even per-body) AABB squares off a ROUND body, so two parallel
# cylinders whose centres are offset DIAGONALLY (the motor vs the differential: a 235 mm
# centre distance at ~48 deg) report a spurious box overlap even when the round solids
# clear by ~18 mm. The sampled-solid test bounds each create/unite body by its true
# oriented PRIMITIVE (a finite cylinder, a swept prism/extrude polygon, a revolve disc)
# and asks whether any sampled surface point of one part falls INSIDE a body of the
# other (and vice versa) -- the physically correct interpenetration test, still NX-free.
# --------------------------------------------------------------------------- #
# surface-sampling density for the solid test (axial stations x angular spokes / polygon
# vertices x axial stations). Dense enough that a neighbouring body's surface points fall
# inside this one when they truly overlap, cheap enough for the whole vehicle.
_AXIAL_STATIONS = 9       # along the body length (incl. both ends)
_ANGULAR_SPOKES = 16      # around a round cross-section


def _world_body(s: Dict[str, Any], R: Mat, origin: Vec,
                include_subtract: bool = False) -> Optional[Dict[str, Any]]:
    """A placed build-step body as an oriented solid PRIMITIVE in vehicle coordinates,
    plus a DENSE list of sampled surface points (lateral surface at several axial
    stations + both end caps).

    By default returns None for ``subtract`` bodies (they remove material, so they are
    not part of the solid envelope). Pass ``include_subtract=True`` to build the
    primitive for a subtract body too -- the VOID-aware clash test (``part_voids`` /
    ``solids_clash``) needs the bore / notch / cavity geometry so a neighbour passing
    THROUGH a real subtracted void (the chassis axle notch, a bolt clearance bore) is
    not falsely flagged as a clash.

    Primitive kinds:
      * "cyl"     : finite cylinder -- base point, unit axis, radius, length.
      * "poly"    : swept closed polygon -- base point, unit axis, length, and the
                    polygon vertices in the section's local (u, v) frame + that frame.
    """
    if s.get("boolean") == "subtract" and not include_subtract:
        return None

    kind = s.get("kind")

    def _cyl(base_local, axis_local, r, L):
        base = _xform(R, origin, base_local)
        aw = _norm3([sum(R[i][k] * axis_local[k] for k in range(3)) for i in range(3)])
        # dense surface sample: spokes around the wall at several axial stations + the
        # two end-cap centres (so a short fat disc is covered too).
        u, v = _perp_frame(aw)
        sample: List[List[float]] = []
        for si in range(_AXIAL_STATIONS):
            t = L * si / (_AXIAL_STATIONS - 1) if _AXIAL_STATIONS > 1 else 0.0
            c = [base[i] + aw[i] * t for i in range(3)]
            sample.append(list(c))                         # axis point (for thin bodies)
            for k in range(_ANGULAR_SPOKES):
                a = 2.0 * math.pi * k / _ANGULAR_SPOKES
                d = [math.cos(a) * u[i] + math.sin(a) * v[i] for i in range(3)]
                sample.append([c[i] + r * d[i] for i in range(3)])
        return {"type": "cyl", "base": base, "axis": aw, "r": float(r),
                "L": float(L), "sample": sample}

    if kind in ("cylinder", "tube", "hole"):
        r = s.get("outer_radius", 0.0)
        L = s.get("length", 0.0)
        if s.get("origin3") is not None:
            return _cyl(list(s["origin3"]), list(s["axis"]), r, L)
        base_local = [s.get("cx", 0.0), s.get("cy", 0.0), s.get("z0", 0.0)]
        return _cyl(base_local, list(s.get("axis", (0.0, 0.0, 1.0))), r, L)

    if kind == "extrude" and s.get("profile") is not None:
        base_local = [0.0, 0.0, s.get("z0", 0.0)]
        return _poly_body([tuple(pt) for pt in s["profile"]], base_local,
                          (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), s.get("length", 0.0),
                          R, origin)

    if kind == "prism" and s.get("profile") is not None and s.get("origin3") is not None:
        return _poly_body([tuple(pt) for pt in s["profile"]], list(s["origin3"]),
                          tuple(s["axis"]), tuple(s.get("u_dir", (1.0, 0.0, 0.0))),
                          s.get("length", 0.0), R, origin)

    if kind == "loft_twist" and s.get("profile") is not None and s.get("origin3") is not None:
        return _twist_body([tuple(pt) for pt in s["profile"]], list(s["origin3"]),
                           tuple(s["axis"]), tuple(s.get("u_dir", (1.0, 0.0, 0.0))),
                           s.get("length", 0.0), s.get("start_twist_deg", 0.0),
                           s.get("twist_deg", 0.0), R, origin)

    if kind == "revolve" and s.get("profile") is not None:
        rmax = max((abs(pt[0]) for pt in s["profile"]), default=0.0)
        zs = [pt[1] for pt in s["profile"]]
        z0 = min(zs) if zs else 0.0
        L = (max(zs) - min(zs)) if zs else 0.0
        return _cyl([0.0, 0.0, z0], (0.0, 0.0, 1.0), rmax, L)

    return None


def _poly_body(profile_uv, origin3, axis, u_dir, length, R, origin):
    """A swept-polygon (prism/extrude) solid in vehicle coordinates: store the base
    point, the world axis, the length, and the section's world (u, v) basis so a point
    can be tested as (in the 2D polygon) AND (axial param in [0, L]). The surface sample
    is the polygon vertices + edge midpoints at several axial stations + both faces."""
    from motor_nx.blueprint import prism_frame
    u, v, w = prism_frame(tuple(axis), tuple(u_dir))
    base = _xform(R, origin, list(origin3))
    uw = [sum(R[i][k] * u[k] for k in range(3)) for i in range(3)]
    vw = [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]
    ww = _norm3([sum(R[i][k] * w[k] for k in range(3)) for i in range(3)])
    poly = [(float(a), float(b)) for (a, b) in profile_uv]
    # vertices + edge midpoints in the (u, v) section
    ring: List[Tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        ring.append(poly[i])
        nx = poly[(i + 1) % n]
        ring.append(((poly[i][0] + nx[0]) / 2.0, (poly[i][1] + nx[1]) / 2.0))
    sample: List[List[float]] = []
    for si in range(_AXIAL_STATIONS):
        t = length * si / (_AXIAL_STATIONS - 1) if _AXIAL_STATIONS > 1 else 0.0
        for (pu, pv) in ring:
            sample.append([base[i] + pu * uw[i] + pv * vw[i] + t * ww[i] for i in range(3)])
    return {"type": "poly", "base": base, "u": uw, "v": vw, "w": ww,
            "L": float(length), "poly": poly, "sample": sample}


def _twist_body(profile_uv, origin3, axis, u_dir, length, start_deg, twist_deg, R, origin):
    """A TRUE twisted (loft_twist / helical) solid in vehicle coordinates: the LOCAL (u, v)
    polygon rotated `start_deg` at the base and `start_deg + twist_deg` at the top, swept
    along the axis with the section rotating LINEARLY. Stored like a poly body but with the
    per-station rotation so a point test un-rotates the point's (u, v) by the section angle at
    its axial parameter before the polygon test. The surface sample is the rotated section
    ring at several axial stations + both faces (dense enough that a neighbouring gear's
    teeth fall inside when they truly overlap)."""
    from motor_nx.blueprint import prism_frame
    u, v, w = prism_frame(tuple(axis), tuple(u_dir))
    base = _xform(R, origin, list(origin3))
    uw = [sum(R[i][k] * u[k] for k in range(3)) for i in range(3)]
    vw = [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]
    ww = _norm3([sum(R[i][k] * w[k] for k in range(3)) for i in range(3)])
    poly = [(float(a), float(b)) for (a, b) in profile_uv]
    ring: List[Tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        ring.append(poly[i])
        nx = poly[(i + 1) % n]
        ring.append(((poly[i][0] + nx[0]) / 2.0, (poly[i][1] + nx[1]) / 2.0))
    # denser axial sampling for a twisting section (the tip helix sweeps tangentially)
    stations = max(_AXIAL_STATIONS, 13)
    sample: List[List[float]] = []
    for si in range(stations):
        f = si / (stations - 1) if stations > 1 else 0.0
        t = length * f
        ang = math.radians(start_deg + twist_deg * f)
        c, sn = math.cos(ang), math.sin(ang)
        for (pu, pv) in ring:
            ru, rv = pu * c - pv * sn, pu * sn + pv * c
            sample.append([base[i] + ru * uw[i] + rv * vw[i] + t * ww[i] for i in range(3)])
    return {"type": "twist", "base": base, "u": uw, "v": vw, "w": ww,
            "L": float(length), "poly": poly, "sample": sample,
            "start": math.radians(start_deg), "twist": math.radians(twist_deg)}


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _point_in_body(pt, body, tol: float) -> bool:
    """Is world point ``pt`` inside the oriented solid ``body`` by more than ``tol`` (so
    a mere surface touch is not flagged)?"""
    rel = [pt[i] - body["base"][i] for i in range(3)]
    if body["type"] == "cyl":
        t = _dot(rel, body["axis"])
        if t < tol or t > body["L"] - tol:
            return False
        radial = [rel[i] - t * body["axis"][i] for i in range(3)]
        return math.sqrt(_dot(radial, radial)) < body["r"] - tol
    if body["type"] == "twist":
        # twisted solid: the section at axial param t is the base polygon rotated by
        # start + twist*(t/L). Un-rotate the point's (u, v) by that angle, then test the
        # (un-twisted) base polygon.
        t = _dot(rel, body["w"])
        if t < tol or t > body["L"] - tol:
            return False
        f = t / body["L"] if body["L"] else 0.0
        ang = body["start"] + body["twist"] * f
        pu = _dot(rel, body["u"])
        pv = _dot(rel, body["v"])
        c, sn = math.cos(-ang), math.sin(-ang)         # rotate the point BACK by -ang
        return _point_in_polygon(pu * c - pv * sn, pu * sn + pv * c, body["poly"], tol)
    # poly (prism/extrude)
    t = _dot(rel, body["w"])
    if t < tol or t > body["L"] - tol:
        return False
    pu = _dot(rel, body["u"])
    pv = _dot(rel, body["v"])
    return _point_in_polygon(pu, pv, body["poly"], tol)


def _point_in_polygon(x: float, y: float, poly, tol: float) -> bool:
    """Ray-cast point-in-polygon (even-odd). ``tol`` is not applied to the polygon edge
    here (the axial tol already gives the touch allowance); the polygon test is exact."""
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < xint:
                inside = not inside
        j = i
    return inside


def part_solids(blueprint: Dict[str, Any], R: Mat, origin: Vec) -> List[Dict[str, Any]]:
    """Every create/unite body of a placed blueprint as an oriented vehicle-frame solid
    primitive (with sampled surface points). Used by :func:`solids_interpenetrate`."""
    out: List[Dict[str, Any]] = []
    for s in blueprint.get("build_steps", []):
        b = _world_body(s, R, origin)
        if b is not None:
            out.append(b)
    return out


def part_voids(blueprint: Dict[str, Any], R: Mat, origin: Vec) -> List[Dict[str, Any]]:
    """Every SUBTRACT body of a placed blueprint as an oriented vehicle-frame primitive
    (a real material VOID: a bore, a bolt clearance hole, the chassis axle NOTCH, a
    hollow-box cavity). The void-aware clash test (:func:`solids_clash`) treats a point
    that lies inside a part's solid AND inside one of its voids as NOT solid -- so a
    neighbour that passes THROUGH a relief notch / clearance bore (an intended path, not
    metal) is not falsely flagged as interpenetration.

    A primitive is bounded by its outer envelope, so a void over-removes only at a
    rounded corner of a rectangular cut -- conservative for a clash test (it can only
    UNDER-report a clash near a void edge, never invent clearance where there is metal
    beyond the cut envelope)."""
    out: List[Dict[str, Any]] = []
    for s in blueprint.get("build_steps", []):
        if s.get("boolean") != "subtract":
            continue
        b = _world_body(s, R, origin, include_subtract=True)
        if b is not None:
            out.append(b)
    return out


def solids_interpenetrate(solids_a: List[Dict[str, Any]],
                          solids_b: List[Dict[str, Any]],
                          touch_tol: float = TOUCH_TOL_MM) -> Optional[List[float]]:
    """True interpenetration test between two parts' oriented solids: a clash exists if
    any sampled surface point of one part's body lies INSIDE the other part's body by
    more than ``touch_tol``. Returns the deepest penetration as a 1-element list
    ``[depth_mm]`` (a positive number) for reporting, or None if the parts clear.

    A cheap whole-part AABB pre-filter (inflated by ``touch_tol``) skips the expensive
    point loop for the overwhelmingly common case of parts nowhere near each other."""
    if not solids_a or not solids_b:
        return None
    # AABB pre-filter over the sampled points
    def _aabb(solids):
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for s in solids:
            for p in s["sample"]:
                for i in range(3):
                    lo[i] = min(lo[i], p[i])
                    hi[i] = max(hi[i], p[i])
        return lo, hi
    aa, bb = _aabb(solids_a), _aabb(solids_b)
    if any(min(aa[1][i], bb[1][i]) - max(aa[0][i], bb[0][i]) < -touch_tol for i in range(3)):
        return None

    # per-body AABB (cached) so the point loop only tests body pairs that can overlap
    def _body_aabb(body):
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for p in body["sample"]:
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
        return lo, hi
    box_a = [_body_aabb(s) for s in solids_a]
    box_b = [_body_aabb(s) for s in solids_b]

    def _box_overlap(ba, bb_):
        return all(min(ba[1][i], bb_[1][i]) - max(ba[0][i], bb_[0][i]) >= -touch_tol
                   for i in range(3))

    worst = 0.0
    found = False
    for src, sboxes, dst, dboxes in ((solids_a, box_a, solids_b, box_b),
                                     (solids_b, box_b, solids_a, box_a)):
        for s, sbox in zip(src, sboxes):
            # which destination bodies can this source body's points possibly enter?
            cand = [body for body, dbox in zip(dst, dboxes) if _box_overlap(sbox, dbox)]
            if not cand:
                continue
            for pt in s["sample"]:
                for body in cand:
                    if _point_in_body(pt, body, touch_tol):
                        found = True
                        depth = _penetration_depth(pt, body)
                        if depth > worst:
                            worst = depth
    return [round(worst, 2)] if found else None


def _point_in_any(pt, bodies, tol: float) -> bool:
    return any(_point_in_body(pt, b, tol) for b in bodies)


def solids_clash(solids_a: List[Dict[str, Any]],
                 solids_b: List[Dict[str, Any]],
                 voids_a: Optional[List[Dict[str, Any]]] = None,
                 voids_b: Optional[List[Dict[str, Any]]] = None,
                 touch_tol: float = TOUCH_TOL_MM) -> Optional[List[float]]:
    """VOID-AWARE interpenetration test between two parts. Like
    :func:`solids_interpenetrate`, but a sampled surface point of part A is counted as a
    real clash with part B ONLY if it lies inside a SOLID body of B by more than
    ``touch_tol`` AND NOT inside any VOID of B (a bore / cavity / the chassis axle notch)
    -- and the point itself must be real metal of A (inside no void of A). So a half-shaft
    or control arm that passes THROUGH the rail's notch window, or a bolt sitting in a
    clearance bore, is correctly NOT a clash.

    Returns the deepest penetration as a 1-element list ``[depth_mm]`` for reporting, or
    None if the parts clear. The ``voids_*`` default to empty (then this is exactly
    :func:`solids_interpenetrate`)."""
    if not solids_a or not solids_b:
        return None
    va = voids_a or []
    vb = voids_b or []

    # whole-part AABB pre-filter over the sampled SOLID points
    def _aabb(solids):
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for s in solids:
            for p in s["sample"]:
                for i in range(3):
                    lo[i] = min(lo[i], p[i])
                    hi[i] = max(hi[i], p[i])
        return lo, hi
    aa, bb = _aabb(solids_a), _aabb(solids_b)
    if any(min(aa[1][i], bb[1][i]) - max(aa[0][i], bb[0][i]) < -touch_tol for i in range(3)):
        return None

    def _body_aabb(body):
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for p in body["sample"]:
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
        return lo, hi
    box_a = [_body_aabb(s) for s in solids_a]
    box_b = [_body_aabb(s) for s in solids_b]

    def _box_overlap(ba, bb_):
        return all(min(ba[1][i], bb_[1][i]) - max(ba[0][i], bb_[0][i]) >= -touch_tol
                   for i in range(3))

    worst = 0.0
    found = False
    for src, sboxes, src_voids, dst, dboxes, dst_voids in (
            (solids_a, box_a, va, solids_b, box_b, vb),
            (solids_b, box_b, vb, solids_a, box_a, va)):
        for s, sbox in zip(src, sboxes):
            cand = [body for body, dbox in zip(dst, dboxes) if _box_overlap(sbox, dbox)]
            if not cand:
                continue
            for pt in s["sample"]:
                # the source point must be real metal of its own part (not in a void of A)
                if src_voids and _point_in_any(pt, src_voids, -touch_tol):
                    continue
                for body in cand:
                    if not _point_in_body(pt, body, touch_tol):
                        continue
                    # the destination point must be real metal of B (not inside a B void)
                    if dst_voids and _point_in_any(pt, dst_voids, -touch_tol):
                        continue
                    found = True
                    depth = _penetration_depth(pt, body)
                    if depth > worst:
                        worst = depth
    return [round(worst, 2)] if found else None


def solids_axial_engagement(solids_a: List[Dict[str, Any]],
                            solids_b: List[Dict[str, Any]],
                            axis_index: int,
                            touch_tol: float = TOUCH_TOL_MM) -> Optional[float]:
    """The AXIAL engagement (mm) of two parts that are MEANT to butt/seat along a mating
    axis (``axis_index`` 0=X, 1=Y, 2=Z): the extent, ALONG that axis, over which sampled
    surface points of one part lie INSIDE the other part's solids.

    For a flange-face butt + pilot spigot this is small (a flange/pilot thickness); for a
    connector that has slid OVER its neighbour (the gearbox burying the motor barrel) it
    spans the buried length. Used to bound the allowed overlap of a mating pair instead of
    skipping it entirely (review finding 4). Returns None if the parts do not overlap at
    all (a clean gap)."""
    if not solids_a or not solids_b:
        return None
    coords: List[float] = []
    for src, dst in ((solids_a, solids_b), (solids_b, solids_a)):
        for s in src:
            for pt in s["sample"]:
                for body in dst:
                    if _point_in_body(pt, body, touch_tol):
                        coords.append(pt[axis_index])
                        break
    if not coords:
        return None
    return max(coords) - min(coords)


def _penetration_depth(pt, body) -> float:
    """A rough penetration depth (mm) of a point inside a body -- the distance to the
    nearest lateral surface (cylinder wall / polygon-extruded section is approximated by
    the radial clearance). Used only for human-readable reporting."""
    rel = [pt[i] - body["base"][i] for i in range(3)]
    if body["type"] == "cyl":
        t = _dot(rel, body["axis"])
        radial = [rel[i] - t * body["axis"][i] for i in range(3)]
        return max(0.0, body["r"] - math.sqrt(_dot(radial, radial)))
    return 0.0
