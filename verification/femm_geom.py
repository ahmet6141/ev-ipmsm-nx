"""Pure-math FEMM geometry for the IPMSM -- emits nodes/segments/arcs/block-labels
as plain data, with NO ``femm`` dependency.  Runnable + unit-testable under plain
CPython, exactly like :mod:`motor_nx.blueprint` (the project's "pure-math blueprint
-> thin CAD emitter" split).  :mod:`verification.femm_emag` is the thin emitter that
replays this data into FEMM via ``mi_*`` calls.

WHY A FULL (NOT 1-POLE) MODEL
    fea_spec advertises a 1-pole anti-periodic sector, but FEMM is fast enough to
    solve the whole 54-slot / 6-pole cross-section in well under a second, and the
    full model sidesteps every anti-periodic-boundary + AGE-sector pitfall.  The
    moving rotor is one FEMM *group* (GROUP_ROTOR); the cogging / torque / ripple
    stages rotate that group and re-solve.

GEOMETRY SOURCE OF TRUTH
    Radii / slot widths come from :func:`motor_nx.em_design.derive`; the slot,
    magnet-pocket and magnet polygons come from :mod:`motor_nx.blueprint`; the
    block-label recipe (copper circuits + magnet magnetisation angles) comes from
    :func:`motor_nx.fea.femm_label_recipe`.  This module only stitches those into a
    planar straight-line graph FEMM can mesh (every region bounded, no crossing
    segments: the stator bore is broken into tooth-tip arcs with a gap at each slot
    mouth so the slot opening stays continuous with the air gap).

UNITS: millimetres, degrees.  Angles are measured CCW from +X.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# allow "python verification/femm_geom.py" from the repo root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import blueprint as _bp          # noqa: E402
from motor_nx import em_design                  # noqa: E402
from motor_nx import fea as _fea                 # noqa: E402
from motor_nx.params import MotorParams          # noqa: E402

Point = Tuple[float, float]

GROUP_STATOR = 0          # fixed: stator steel, windings, outer boundary, bore tooth tips
GROUP_ROTOR = 1           # rotates for cogging/torque/ripple: rotor steel, magnets, pockets

# FEMM material names this geometry references (defined by the emitter)
MAT_AIR = "Air"
MAT_STEEL = "LamSteel"
MAT_COPPER = "Coil"
MAT_MAGNET = "Magnet"


# --------------------------------------------------------------------------- #
# data records
# --------------------------------------------------------------------------- #
@dataclass
class Seg:
    x1: float; y1: float; x2: float; y2: float
    bdry: Optional[str] = None
    group: int = 0


@dataclass
class Arc:
    x1: float; y1: float; x2: float; y2: float
    angle_deg: float            # swept CCW from (x1,y1) to (x2,y2)
    maxseg_deg: float = 1.0
    bdry: Optional[str] = None
    group: int = 0


@dataclass
class Label:
    x: float; y: float
    material: str
    group: int = 0
    circuit: str = ""
    turns: int = 0
    magdir_deg: float = 0.0
    meshsize: float = 0.0       # 0 => automesh
    note: str = ""


@dataclass
class FemmModel:
    name: str
    depth_mm: float                              # axial stack length -> mi_probdef depth
    segs: List[Seg] = field(default_factory=list)
    arcs: List[Arc] = field(default_factory=list)
    labels: List[Label] = field(default_factory=list)
    circuits: List[str] = field(default_factory=list)
    radii: Dict[str, float] = field(default_factory=dict)
    outer_boundary: str = "A0"

    # convenience for the emitter / tests
    def label_tally(self) -> Dict[str, int]:
        t: Dict[str, int] = {}
        for lb in self.labels:
            t[lb.material] = t.get(lb.material, 0) + 1
        return t


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _rot(x: float, y: float, deg: float) -> Point:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x * c - y * s, x * s + y * c)


def _rot_poly(poly: List[Point], deg: float) -> List[Point]:
    return [_rot(x, y, deg) for (x, y) in poly]


def _poly_centroid(poly: List[Point]) -> Point:
    n = len(poly)
    return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)


def _add_closed_poly(model: FemmModel, poly: List[Point], group: int,
                     bdry: Optional[str] = None) -> None:
    """Add the closed boundary of a polygon as a loop of segments."""
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9:
            continue            # skip a degenerate zero-length edge
        model.segs.append(Seg(x1, y1, x2, y2, bdry, group))


def _circle_arcs(model: FemmModel, radius: float, group: int,
                 bdry: Optional[str], maxseg_deg: float = 1.0) -> None:
    """A full circle as two CCW semicircle arcs (FEMM needs >=2 arcs per circle)."""
    p0 = (radius, 0.0)
    p1 = (-radius, 0.0)
    model.arcs.append(Arc(p0[0], p0[1], p1[0], p1[1], 180.0, maxseg_deg, bdry, group))
    model.arcs.append(Arc(p1[0], p1[1], p0[0], p0[1], 180.0, maxseg_deg, bdry, group))


# --------------------------------------------------------------------------- #
# stator
# --------------------------------------------------------------------------- #
def _build_stator(model: FemmModel, p: MotorParams, g) -> None:
    s = p.stator
    R_so = g.stator_outer_radius
    R_sb = g.bore_radius
    r1 = g.slot_body_inner_radius        # slot-body inner (near bore)
    r2 = g.slot_body_outer_radius        # slot bottom (near yoke)
    sw = g.slot_width / 2.0              # tangential half-width of the slot body
    ow = s.slot_opening_width / 2.0      # tangential half-width of the slot opening
    n_slot = s.slot_count
    pitch = 360.0 / n_slot

    # outer boundary (A = 0)
    _circle_arcs(model, R_so, GROUP_STATOR, model.outer_boundary, maxseg_deg=1.0)

    # mouth points sit ON the bore circle: y = +/-ow, x = sqrt(R_sb^2 - ow^2)
    xb = math.sqrt(max(R_sb * R_sb - ow * ow, 0.0))
    phi = math.degrees(math.asin(min(1.0, ow / R_sb)))   # half-angular span of a slot mouth

    for k in range(n_slot):
        th = k * pitch
        # reference-frame slot wall points (centred on +X), then rotate to th
        C = _rot(r1, +sw, th)     # body shoulder +
        D = _rot(r2, +sw, th)     # slot bottom +
        E = _rot(r2, -sw, th)     # slot bottom -
        F = _rot(r1, -sw, th)     # body shoulder -
        Bb = _rot(r1, +ow, th)    # opening bottom +  (body/opening interface end)
        Gb = _rot(r1, -ow, th)    # opening bottom -
        Bt = _rot(xb, +ow, th)    # bore mouth +  (on R_sb circle)
        Gt = _rot(xb, -ow, th)    # bore mouth -
        # slot-body walls + bottom
        model.segs += [
            Seg(*C, *D, None, GROUP_STATOR),     # + body wall
            Seg(*D, *E, None, GROUP_STATOR),     # slot bottom
            Seg(*E, *F, None, GROUP_STATOR),     # - body wall
            Seg(*C, *Bb, None, GROUP_STATOR),    # + tooth shoulder (overhang underside)
            Seg(*F, *Gb, None, GROUP_STATOR),    # - tooth shoulder
            Seg(*Bb, *Gb, None, GROUP_STATOR),   # copper-body top <-> opening-air interface (at r1)
            Seg(*Bb, *Bt, None, GROUP_STATOR),   # + opening wall up to the bore
            Seg(*Gb, *Gt, None, GROUP_STATOR),   # - opening wall up to the bore
        ]
        # copper block label (one homogenised winding region per slot)
        # NOTE: turns/circuit are filled later from the femm_label_recipe.

    # bore broken into tooth-tip arcs: from slot k's + mouth to slot (k+1)'s - mouth,
    # leaving a gap at each slot mouth so the opening air stays continuous with the gap.
    for k in range(n_slot):
        th = k * pitch
        start = _rot(xb, +ow, th)                 # this slot's + mouth (angle th+phi)
        end = _rot(xb, -ow, (k + 1) * pitch)      # next slot's - mouth (angle th+pitch-phi)
        span = pitch - 2.0 * phi                  # tooth-tip arc span
        model.arcs.append(Arc(start[0], start[1], end[0], end[1],
                              span, 0.5, None, GROUP_STATOR))


# --------------------------------------------------------------------------- #
# rotor (steel + V magnets + flux-barrier pockets)
# --------------------------------------------------------------------------- #
def _build_rotor(model: FemmModel, p: MotorParams, g) -> None:
    r = p.rotor
    R_ro = g.rotor_outer_radius
    R_sh = g.shaft_radius
    n_pole = r.pole_count
    pole_pitch = 360.0 / n_pole

    # rotor surface + shaft bore (both rotate with the rotor group)
    _circle_arcs(model, R_ro, GROUP_ROTOR, None, maxseg_deg=0.5)
    _circle_arcs(model, R_sh, GROUP_ROTOR, None, maxseg_deg=2.0)

    # reference-pole polygons (centred on +X)
    pockets_ref = _bp.magnet_pocket_polygons(p, g)     # [+Y arm, -Y arm]
    magnets_ref = _bp.magnet_polygons(p, g)            # [+Y arm, -Y arm]

    for k in range(n_pole):
        ang = k * pole_pitch
        for arm in range(len(pockets_ref)):
            pocket = _rot_poly(pockets_ref[arm], ang)
            magnet = _rot_poly(magnets_ref[arm], ang)
            _add_closed_poly(model, pocket, GROUP_ROTOR, None)   # air flux barrier wall
            _add_closed_poly(model, magnet, GROUP_ROTOR, None)   # magnet outline
            # pocket-air label: a point inside the pocket but OUTSIDE the magnet,
            # in the end-barrier region beyond the magnet's outer (air-gap-side) end.
            for air_pt in _pocket_air_points(p, g, magnets_ref[arm], pockets_ref[arm], ang):
                model.labels.append(Label(air_pt[0], air_pt[1], MAT_AIR,
                                          group=GROUP_ROTOR, note="pocket flux barrier"))


def _pocket_air_points(p: MotorParams, g, magnet_ref: List[Point],
                       pocket_ref: List[Point], ang: float) -> List[Point]:
    """Points in the end-barrier air (inside pocket, outside magnet) for one arm.

    The pocket is the magnet grown by ``end_barrier`` at BOTH ends, so there are TWO
    disconnected air barriers: one just beyond the magnet's OUTER (air-gap-side) end face
    and one just beyond its INNER (centre-post-side) end face. Each is its own FEMM region
    and needs its own block label -- labelling only one left the other region without a
    material ("Material properties have not been defined for all regions")."""
    r = p.rotor
    if r.end_barrier <= 1e-6:
        return []
    # Derive the end-barrier points from the MAGNET'S OWN polygon so BOTH V arms are
    # handled. The previous version recomputed a single arm's axis from _v_magnet_axes
    # (ignoring the arm), so the second magnet of every V kept its barriers unlabelled.
    cx = sum(v[0] for v in magnet_ref) / len(magnet_ref)
    cy = sum(v[1] for v in magnet_ref) / len(magnet_ref)
    best, ux, uy = 0.0, 1.0, 0.0          # length axis = the magnet's longest edge
    n = len(magnet_ref)
    for i in range(n):
        x1, y1 = magnet_ref[i]
        x2, y2 = magnet_ref[(i + 1) % n]
        d = math.hypot(x2 - x1, y2 - y1)
        if d > best:
            best, ux, uy = d, (x2 - x1) / d, (y2 - y1) / d
    step = r.magnet_width / 2.0 + 0.5 * r.end_barrier      # just beyond each end face
    outer = (cx + step * ux, cy + step * uy)
    inner = (cx - step * ux, cy - step * uy)
    return [_rot(outer[0], outer[1], ang), _rot(inner[0], inner[1], ang)]


# --------------------------------------------------------------------------- #
# block labels (air gap, shaft, rotor steel, copper slots, magnets)
# --------------------------------------------------------------------------- #
def _build_labels(model: FemmModel, p: MotorParams, g) -> None:
    r = p.rotor
    R_so = g.stator_outer_radius
    R_sb = g.bore_radius
    R_ro = g.rotor_outer_radius
    R_sh = g.shaft_radius
    r1 = g.slot_body_inner_radius
    r2 = g.slot_body_outer_radius

    # --- air gap, split by a mid-gap circle at R_ag --------------------------
    # The INNER ring (rotor surface .. R_ag) is a clean annulus around the rotor
    # (the slot openings sit at r > bore > R_ag, so they never touch it) -- it is
    # the integration region for the weighted-stress-tensor torque.  The OUTER ring
    # (R_ag .. bore) stays continuous with the slot-opening air through the mouths.
    R_ag = (R_ro + R_sb) / 2.0
    _circle_arcs(model, R_ag, GROUP_STATOR, None, maxseg_deg=0.5)
    gap_mesh = max(0.12, r.air_gap / 4.0)
    model.labels.append(Label((R_ro + R_ag) / 2.0, 0.0, MAT_AIR, group=GROUP_STATOR,
                              meshsize=gap_mesh, note="air gap (inner ring, torque contour)"))
    model.labels.append(Label((R_ag + R_sb) / 2.0, 0.0, MAT_AIR, group=GROUP_STATOR,
                              meshsize=gap_mesh, note="air gap (outer ring + slot mouths)"))

    # --- shaft bore: non-magnetic in the EM model (per fea_spec note) ------ #
    if R_sh > 1.0:
        model.labels.append(Label(R_sh / 2.0, 0.0, MAT_AIR, group=GROUP_ROTOR,
                                  note="shaft bore (non-magnetic EM)"))

    # --- stator yoke steel ------------------------------------------------- #
    model.labels.append(Label((r2 + R_so) / 2.0, 0.0, MAT_STEEL,
                              group=GROUP_STATOR, note="stator yoke"))

    # --- rotor steel: a central post label + one per pole cap (same material;
    #     redundant labels guard against a pole island being left unmeshed) -- #
    ext = em_design._magnet_pocket_extent(p, g)
    r_pocket_min = ext[1] if ext else (R_sh + 5.0)
    model.labels.append(Label((R_sh + r_pocket_min) / 2.0, 0.0, MAT_STEEL,
                              group=GROUP_ROTOR, note="rotor core (centre)"))
    pole_pitch = 360.0 / r.pole_count
    r_cap = 0.5 * ((ext[0] if ext else R_ro) + R_ro)    # between pocket outer corner and surface
    for k in range(r.pole_count):
        # pole-cap steel sits on the d-axis (pole centre) just inside the surface
        x, y = _rot(r_cap, 0.0, k * pole_pitch)
        model.labels.append(Label(x, y, MAT_STEEL, group=GROUP_ROTOR,
                                  note="rotor pole cap %d" % k))

    # --- copper slots + magnets come straight from the recipe -------------- #
    recipe = _fea.femm_label_recipe(p)
    for row in recipe:
        mat = row["material"]
        if mat == "Copper":
            model.labels.append(Label(
                row["x"], row["y"], MAT_COPPER, group=GROUP_STATOR,
                circuit=str(row["circuit"]),
                turns=int(row["turns"]) if row["turns"] != "" else 0,
                meshsize=0.0, note=row["note"]))
        elif mat == "NdFeB":
            model.labels.append(Label(
                row["x"], row["y"], MAT_MAGNET, group=GROUP_ROTOR,
                magdir_deg=float(row["magdir"]) if row["magdir"] != "" else 0.0,
                meshsize=max(0.3, p.rotor.magnet_thickness / 4.0),
                note=row["note"]))
        # Steel / Air rows from the recipe are superseded by the explicit,
        # group-correct labels added above (the recipe is group-agnostic).


# --------------------------------------------------------------------------- #
# top-level build
# --------------------------------------------------------------------------- #
def build(p: Optional[MotorParams] = None) -> FemmModel:
    if p is None:
        p = MotorParams()
    g = em_design.derive(p)
    model = FemmModel(
        name=p.name, depth_mm=p.stack_length,
        circuits=["A", "B", "C"],
        radii={
            "stator_OD": g.stator_outer_radius, "bore": g.bore_radius,
            "rotor_OD": g.rotor_outer_radius, "shaft": g.shaft_radius,
            "slot_body_inner": g.slot_body_inner_radius,
            "slot_body_outer": g.slot_body_outer_radius,
            "air_gap_mid": (g.rotor_outer_radius + g.bore_radius) / 2.0,
            # torque-integration point: middle of the clean inner gap ring
            # (rotor surface .. mid-gap circle), on +X
            "torque_ring_r": (g.rotor_outer_radius + (g.rotor_outer_radius + g.bore_radius) / 2.0) / 2.0,
            "air_gap": p.rotor.air_gap,
        },
    )
    _build_stator(model, p, g)
    _build_rotor(model, p, g)
    _build_labels(model, p, g)
    return model


# --------------------------------------------------------------------------- #
# standalone sanity check (NO FEMM required) -- run to validate the geometry data
# --------------------------------------------------------------------------- #
def _sanity(model: FemmModel) -> List[str]:
    problems: List[str] = []
    R = model.radii
    order = [R["shaft"], R["rotor_OD"], R["bore"], R["slot_body_inner"],
             R["slot_body_outer"], R["stator_OD"]]
    names = ["shaft", "rotor_OD", "bore", "slot_body_inner", "slot_body_outer", "stator_OD"]
    for i in range(len(order) - 1):
        if not (order[i] < order[i + 1] + 1e-9):
            problems.append("radius order violated: %s(%.2f) !< %s(%.2f)"
                            % (names[i], order[i], names[i + 1], order[i + 1]))
    if R["rotor_OD"] >= R["bore"]:
        problems.append("rotor_OD >= bore (no air gap)")
    tally = model.label_tally()
    if tally.get(MAT_COPPER, 0) == 0:
        problems.append("no copper labels")
    if tally.get(MAT_MAGNET, 0) == 0:
        problems.append("no magnet labels")
    # every copper label must name a circuit and carry turns
    for lb in model.labels:
        if lb.material == MAT_COPPER and (not lb.circuit or lb.turns == 0):
            problems.append("copper label at (%.1f,%.1f) missing circuit/turns" % (lb.x, lb.y))
            break
    # magnet count == 2 arms * poles
    return problems


def main(argv=None) -> int:
    p = MotorParams()
    model = build(p)
    g = em_design.derive(p)
    print("=== FEMM geometry sanity: %s ===" % model.name)
    print("depth (stack)      : %.1f mm" % model.depth_mm)
    print("radii (mm)         : " + ", ".join("%s=%.2f" % (k, v) for k, v in model.radii.items()))
    print("segments           : %d" % len(model.segs))
    print("arcs               : %d" % len(model.arcs))
    print("block labels       : %d  %s" % (len(model.labels), model.label_tally()))
    print("circuits           : %s" % ", ".join(model.circuits))
    # turns balance per phase (sum of signed turns should cancel over the machine)
    bal: Dict[str, int] = {}
    for lb in model.labels:
        if lb.material == MAT_COPPER:
            bal[lb.circuit] = bal.get(lb.circuit, 0) + lb.turns
    print("signed-turns/phase : %s  (each should be 0 for a balanced winding)" % bal)
    mags = [lb for lb in model.labels if lb.material == MAT_MAGNET]
    print("magnet magdirs     : " + ", ".join("%.0f" % m.magdir_deg for m in mags))
    problems = _sanity(model)
    if problems:
        print("\nSANITY PROBLEMS:")
        for pr in problems:
            print("  - " + pr)
        return 1
    print("\nsanity: OK (radius order, labels, circuits, turns balance)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
