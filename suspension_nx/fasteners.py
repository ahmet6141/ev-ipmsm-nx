"""Parametric FASTENERS and JOINT primitives for the corner suspension, emitted as
the same ordered :class:`motor_nx.blueprint.BuildStep` list the NX builder consumes.

Why this module exists
    The first NX build of the corner had 32 body pairs that genuinely
    INTERPENETRATED (arm hubs buried in the knuckle, the ball joint overlapping both
    the arm and the knuckle, eyes overlapping their own link, sleeves buried in
    bushings).  The fix is to model every joint as a REAL bolted/pressed joint so no
    two distinct solids ever share volume -- "no interpenetration BY CONSTRUCTION".

How hollow bodies are made (and why NOT a single annulus profile)
    A hollow ring/eye/can/sleeve/perch/barrel/nut is built the way the whole repo
    (the ``tube`` kind, motor_nx) makes hollows that BUILD CLEANLY IN NX: an OUTER
    solid (create) with a concentric INNER solid SUBTRACTED out for the bore.  NX's
    Section rejects a single profile that contains both an outer and an inner loop as
    "self intersecting", so a one-profile annulus prism does NOT build -- it must be
    two simple loops (outer create + inner subtract).  ``ring_body`` therefore emits a
    ``kind="tube"`` step (origin3 + axis), which the NX builder turns into exactly that
    outer-extrude + inner-subtract on an arbitrary axis.

    A bolt shank / ball stud / bushing bolt is a SOLID round ``cylinder`` (create)
    that sits inside the SUBTRACTED bore of the rings it passes through -- the bore is
    a real void in NX, so the shank fills it without overlapping the ring material.  A
    press fit (sleeve OD == can bore, ball housing OD == eye bore) is a touching
    contact -- the inner part fills the host's subtracted void exactly, sharing only a
    face, which NX point-in-solid containment scores as ~0 interior points.

    Distinct members that are NOT a fit are kept spatially disjoint (stacked AXIALLY
    along the joint pin -- a real clevis/lap), so their solids never share volume.

NOTE on clearance.py:  vehicle_nx/clearance.py is BLIND to ``subtract`` voids and
    treats a ``tube`` as a solid disc, so it gives false readings on bored parts.  It
    is used only for gross solid-member overlap it can actually see; the definitive
    no-interpenetration arbiter is verification/nx_inspect.py (real NX point-in-solid
    containment, which DOES see the subtracted bores).  The geometry here is correct
    BY CONSTRUCTION, not by satisfying clearance.py.

Everything is pure math (no NX import) and oriented in the LOCAL corner frame via
``origin3`` + ``axis`` exactly like suspension_nx.blueprint.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from motor_nx.blueprint import BuildStep

Vec3 = Tuple[float, float, float]

# fastener / joint colours (RGB 0-255)
COL_BOLT = (60, 62, 68)         # dark steel bolt / stud
COL_NUT = (80, 82, 90)
COL_SLEEVE = (150, 152, 160)    # bright inner sleeve / ball housing
COL_BUSH = (45, 45, 50)         # rubber bushing can

# RADIAL clearance (mm) at every press/clearance fit.  A touching OD==ID fit can sample
# as interference in NX point-in-solid containment, so the inner part is always made a
# touch SMALLER than the host bore: inner_OD = host_bore - 2*FIT_CLEARANCE.  >= 0.3 mm
# radial, per the redesign brief (priority 4).
FIT_CLEARANCE = 0.4


# --------------------------------------------------------------------------- #
# small 3D vector helpers (local, zero-dependency)
# --------------------------------------------------------------------------- #
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v: Vec3) -> Vec3:
    n = _norm(v) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


# --------------------------------------------------------------------------- #
# a hollow RING (eye / bushing-can / sleeve / perch / nut / hub barrel) as a
# kind="tube": outer solid (create) - inner solid (subtract), built on an
# arbitrary axis.  This BUILDS in NX (two simple loops), unlike an annulus prism.
# --------------------------------------------------------------------------- #
def ring_body(step_id: str, role: str, body_name: str, center: Vec3, axis: Vec3,
              outer_d: float, bore_d: float, length: float, color, material: str,
              boolean: str = "create", target: Optional[str] = None) -> BuildStep:
    """A hollow ring of outer diameter ``outer_d`` and through-bore ``bore_d``, of axial
    ``length``, CENTRED on ``center`` and coaxial with ``axis``.  Emitted as a
    ``kind="tube"`` step (origin3 + axis); the NX builder makes it an outer cylinder
    (create) minus a concentric inner cylinder (subtract), so the bore is a real void
    a shank/sleeve passes through.  Spans +/- length/2 about ``center`` along the axis.

    A ring is its own create body (a ``tube`` is created then bored in one step), so it
    is not used as a unite/subtract child; ``boolean`` defaults to "create"."""
    w = _unit(axis)
    base = _add(center, _scale(w, -0.5 * length))
    bore_d = max(1.0, min(bore_d, outer_d - 1.0))
    return BuildStep(
        id=step_id, role=role, kind="tube", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=outer_d / 2.0, inner_radius=bore_d / 2.0,
        origin3=base, axis=w, length=length, target=target)


def solid_disc(step_id: str, role: str, body_name: str, center: Vec3, axis: Vec3,
               diameter: float, length: float, color, material: str,
               boolean: str = "create", target: Optional[str] = None) -> BuildStep:
    """A SOLID short cylinder (a bolt head / nut blank / boss) centred on ``center``
    coaxial with ``axis`` -- a single simple-loop circle, so it builds in NX and can be
    UNITED into a member (unlike a tube)."""
    w = _unit(axis)
    base = _add(center, _scale(w, -0.5 * length))
    return BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=diameter / 2.0, origin3=base, axis=w, length=length,
        target=target)


def united_eye(steps: List[BuildStep], step_id: str, role: str, body_name: str,
               target: str, center: Vec3, axis: Vec3, outer_d: float, bore_d: float,
               length: float, color, material: str) -> float:
    """An EYE that is PART OF a member (e.g. a toe-link eye, a knuckle socket, a spring
    perch): a SOLID boss disc UNITED into ``target`` plus a concentric through-BORE
    SUBTRACTED from ``target``.  This is how a bored feature joins a member and still
    builds in NX -- a solid create/unite loop plus a separate subtract loop, never a
    single annulus.

    CONTRACT (priority 1): the boss MUST physically overlap ``target``'s existing solid,
    so the unite makes ONE connected lump and the coaxial bore then cuts it.  If the
    boss is disjoint from the target, NX's unite leaves a separate lump and the bore can
    land "completely outside target body" and FAIL -- leaving the eye solid (a clash).
    Callers therefore route the member's shank/leg up to (and through) the eye centre.
    The bore is concentric with the boss (same centre + axis) and spans the full boss +
    a hair each side so it cuts cleanly.  Returns the bore diameter."""
    w = _unit(axis)
    bore_d = max(1.0, min(bore_d, outer_d - 1.0))
    boss_base = _add(center, _scale(w, -0.5 * length))
    steps.append(BuildStep(
        id=step_id, role=role, kind="cylinder", boolean="unite", target=target,
        body_name=body_name, material=material, color=color,
        outer_radius=outer_d / 2.0, origin3=boss_base, axis=w, length=length))
    bore_len = length + 4.0
    bore_base = _add(center, _scale(w, -0.5 * bore_len))
    steps.append(BuildStep(
        id=step_id + "_bore", role=role, kind="cylinder", boolean="subtract",
        target=target, body_name=body_name + "_Bore", material="air", color=(0, 0, 0),
        outer_radius=bore_d / 2.0, origin3=bore_base, axis=w, length=bore_len))
    return bore_d


def rod_body(step_id: str, role: str, body_name: str, p0: Vec3, p1: Vec3,
             diameter: float, color, material: str,
             boolean: str = "create", target: Optional[str] = None) -> BuildStep:
    """A solid round rod/shank/stud from ``p0`` to ``p1`` (a bolt shank, a ball stud).
    Its radius is < the bore radius of every ring it threads, so it fills the
    subtracted voids without overlapping the ring material."""
    axis = _sub(p1, p0)
    return BuildStep(
        id=step_id, role=role, kind="cylinder", boolean=boolean,
        body_name=body_name, material=material, color=color,
        outer_radius=diameter / 2.0, origin3=p0, axis=axis, length=_norm(axis),
        target=target)


# --------------------------------------------------------------------------- #
# a complete BOLT (hex head + shank) + NUT through a set of coaxial eye bores
# --------------------------------------------------------------------------- #
def bolt_clearance(bore_d: float) -> float:
    """Shank diameter for a bore: a clear radial gap so the shank fills the bore void
    but never reaches the ring solid (so NX point-in-solid reads the joint as clean)."""
    return max(4.0, bore_d - 4.0)


def bolt_assembly(steps: List[BuildStep], tag: str, U: str, name: str,
                  center: Vec3, axis: Vec3, span: float, bore_d: float,
                  head_d: Optional[float] = None) -> None:
    """Emit a dedicated BOLT (hex head + shank) + NUT for a pin joint whose stacked
    eyes are CENTRED on ``center``, coaxial with ``axis``, occupying ``span`` mm along
    the axis (the total stacked eye thickness).  The shank runs the eye stack; the head
    and nut are SOLID discs just clear of each outer eye face, so they sit OUTSIDE the
    eyes (no solid overlap) while the thin shank fills the (clear) bores.

      head  | eye stack (span) | nut
      ====[##|=================|##]====   (shank, thin, clears every bore)"""
    w = _unit(axis)
    head_d = head_d or max(bore_d * 1.7, bore_d + 8.0)
    head_t = max(5.0, 0.5 * bore_d)             # head / nut axial thickness
    shank_d = bolt_clearance(bore_d)
    half = 0.5 * span
    stand = 4.0                                  # axial gap between an eye face + head/nut
    grip = half + stand + 0.5
    p_head_inner = _add(center, _scale(w, -grip))           # shank start (under head)
    p_nut_outer = _add(center, _scale(w, +grip))            # shank end (past nut)
    steps.append(rod_body(
        "%s_shank_%s" % (name, tag), "fastener", "Bolt_Shank_%s_%s" % (name.title(), U),
        p_head_inner, p_nut_outer, shank_d, COL_BOLT, "bolt_steel"))
    # hex HEAD: a solid disc clear OUTSIDE the eye stack on the -axis side (a standoff
    # gap so it never samples as touching the eye it clamps).
    head_c = _add(center, _scale(w, -(half + stand + 0.5 * head_t)))
    steps.append(solid_disc(
        "%s_head_%s" % (name, tag), "fastener", "Bolt_Head_%s_%s" % (name.title(), U),
        head_c, w, head_d, head_t, COL_BOLT, "bolt_steel"))
    # NUT: a solid disc clear OUTSIDE the eye stack on the +axis side.
    nut_c = _add(center, _scale(w, +(half + stand + 0.5 * head_t)))
    steps.append(solid_disc(
        "%s_nut_%s" % (name, tag), "fastener", "Bolt_Nut_%s_%s" % (name.title(), U),
        nut_c, w, head_d, head_t, COL_NUT, "nut_steel"))


# --------------------------------------------------------------------------- #
# a compliance BUSHING: outer can (ring) + inner steel sleeve (ring) -- touching fit
# --------------------------------------------------------------------------- #
def bushing(steps: List[BuildStep], tag: str, U: str, name: str, center: Vec3,
            axis: Vec3, can_d: float, length: float, bolt_bore_d: float) -> Tuple[float, str]:
    """A press-in compliance BUSHING centred on ``center`` coaxial with ``axis``:
      * an outer rubber CAN (ring) of OD ``can_d`` with a bore the steel sleeve sits in;
      * an inner steel SLEEVE (ring) whose OD is the can bore MINUS a clearance (so the
        fit reads clean, not as interference) and whose bore takes the joint bolt.
    STRICT diameter nesting with >= FIT_CLEARANCE radial gap at each step:
        can OD ``can_d``  >  can bore == sleeve OD + 2*clr  >  sleeve OD
        sleeve OD  >  sleeve bore == bolt clearance.
    Both are tube rings (outer-create + inner-subtract), so the sleeve sits in the can's
    real void with a clear gap.  Returns ``(sleeve_bore_d, can_create_id)`` so the caller
    can run the joint bolt through the sleeve bore."""
    can_bore = can_d * 0.62
    sleeve_od = can_bore - 2.0 * FIT_CLEARANCE                 # clear gap inside the can
    sleeve_bore = max(bolt_bore_d, sleeve_od * 0.45)
    can_id = "%s_can_%s" % (name, tag)
    steps.append(ring_body(
        can_id, "bushing", "Bushing_%s_%s" % (name.title(), U), center, axis,
        can_d, can_bore, length, COL_BUSH, "rubber"))
    steps.append(ring_body(
        "%s_sleeve_%s" % (name, tag), "bushing", "Bushing_Sleeve_%s_%s" % (name.title(), U),
        center, axis, sleeve_od, sleeve_bore, length * 0.9, COL_SLEEVE, "joint_steel"))
    return sleeve_bore, can_id
