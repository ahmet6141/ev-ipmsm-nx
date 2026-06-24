"""Parametric inputs for the suspension / e-axle SUBFRAME (cradle) -- the connector
part that closes the chassis <-> suspension joint (ICD §7.2).

Mirrors :mod:`chassis_nx.params` / :mod:`driveline_nx.params`: plain dataclasses so
one :class:`SubframeParams` fully describes a variant, variants load from / save to
JSON, and every numeric dimension is pushed into Siemens NX as a named *expression*
so the built body stays editable.

Units: millimetres (mm) unless noted. The subframe is built in the VEHICLE FRAME
(like the chassis), so the assembler places it at IDENTITY per axle (ICD §3, §7.2).

THE LOCAL DATUM (read this before changing any hardpoint)
    The subframe's local origin (0, 0, 0) represents the AXLE CENTRE ON THE GROUND,
    i.e. vehicle point (axle_x, 0, 0) where axle_x = ±wheelbase/2. So:

        local_coord = vehicle_coord - (axle_x, 0, 0)

    The assembler restores absolute vehicle coordinates by placing the part with the
    identity orientation at origin = (axle_x, 0, 0). The suspension is ONE canonical
    corner reused front and rear (the front axle is NOT X-mirrored -- only axle_x changes
    sign), so the front/rear subframe PICKUP geometry is IDENTICAL; only the placement
    origin's X differs (applied by the assembler). The front/rear remain distinct part
    files for the assembly, but their boss/tower table does not X-mirror.

THE HARDPOINT TABLE -- DERIVED from the suspension, the single source of truth
    (ICD §7.2 / §7.3 / §7.4.2)
    The suspension corner is datumed on the HUB CENTRE; its inboard pickups land
    ~160-240 mm inboard of the chassis rails with nothing to attach to, and the
    spring/damper top reaches z≈690 with no body mount.

    CRITICAL CONVENTION (the fix for the four-corner pickup mismatch). The subframe
    boss positions are NOT an independently-mirrored copy of the rear-left hardpoints
    -- they are DERIVED from the suspension hardpoint table verbatim, applying the
    EXACT placement convention the vehicle assembly uses for the suspension corner, so
    the two can never drift again. The suspension is ONE canonical corner reused at all
    four wheels, placed by the assembly as::

        suspension_world(axle, side) = HUB_CENTRE(axle, side) + Rz(side) . hp_local
            HUB_CENTRE(axle, side) = (axle_x, side_sign * T/2, tyre_radius)
            Rz(LEFT) = identity ,  Rz(RIGHT) = rot_z(180)   (negates local x AND y)
            side_sign = +1 (LEFT, +Y) , -1 (RIGHT, -Y)
            the SAME canonical corner front and rear (the front is NOT X-mirrored --
            only axle_x changes sign).

    The subframe is built in the vehicle frame and placed IDENTITY at the axle station
    (axle_x, 0, 0), so for its boss to land on the suspension pickup we need::

        boss_local(side) = Rz(side) . hp_local + (0, side_sign * T/2, tyre_radius)

    i.e. the SAME Rz(side) and the SAME track/tyre-radius offset, with NO front X
    mirror. ``hardpoints_local()`` computes exactly this from
    ``suspension_nx.engineering.hardpoints`` (a leaf, NX-free import). A frozen literal
    fallback (the rear-left ICD values) is used only if the suspension package is
    absent. For reference, the resulting rear-LEFT VEHICLE coordinates are:

        lower_pickup_fore  (-1310, 344, 277)   upper_pickup_fore  (-1324, 424, 386)
        lower_pickup_aft   (-1550, 344, 277)   upper_pickup_aft   (-1564, 424, 386)
        toe_pickup         (-1606, 412, 343)   damper/strut top   (-1437, 427, 692)

Defaults target the same passenger-EV rear-drive-unit class as the rest of the
platform (~Tesla Model 3 RDU). engineering.validate() flags impossible combinations
(pads off the chassis y=±585, bosses off the suspension hardpoints, tower not
reaching the damper top) before a build.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# The subframe hardpoints whose bosses/tower the cradle presents (the inboard control-
# arm + toe pickups, plus the spring/damper top). These names are read straight from the
# suspension hardpoint table -- the single source of truth -- so the two can never drift.
_SUBFRAME_HARDPOINT_NAMES: Tuple[str, ...] = (
    "lower_pickup_fore", "lower_pickup_aft",
    "upper_pickup_fore", "upper_pickup_aft",
    "toe_pickup", "damper_top",
)

# FROZEN FALLBACK -- the suspension-derived LEFT-corner subframe-local hardpoints for the
# platform defaults (T = 1580, tyre_radius = 335). Used ONLY when suspension_nx is not
# importable; the live path derives these from suspension_nx.engineering.hardpoints so the
# coincidence holds on all four corners (see hardpoints_local). Kept as the documented
# rear-left ICD §7.2 contract values for reference / NX-free standalone builds.
_LEFT_HARDPOINTS_LOCAL_FALLBACK: Dict[str, Tuple[float, float, float]] = {
    "lower_pickup_fore": (127.1, 344.0, 276.8),   # veh (-1310, 344, 277)
    "lower_pickup_aft":  (-112.9, 344.0, 276.8),  # veh (-1550, 344, 277)
    "upper_pickup_fore": (112.9, 424.0, 386.0),   # veh (-1324, 424, 386)
    "upper_pickup_aft":  (-127.1, 424.0, 386.0),  # veh (-1564, 424, 386)
    "toe_pickup":        (-168.0, 411.9, 343.0),  # veh (-1606, 412, 343)
    "damper_top":        (0.0, 427.0, 692.0),     # veh (-1437, 427, 692) strut/damper top
}

# Back-compat alias: the canonical LEFT-corner local hardpoint NAMES live here. Consumers
# that only iterate the keys (tests, blueprint) use this; the per-side, suspension-derived
# values come from SubframeParams.hardpoints_local().
_LEFT_HARDPOINTS_LOCAL = _LEFT_HARDPOINTS_LOCAL_FALLBACK


def _suspension_hardpoints_local() -> Dict[str, Tuple[float, float, float]]:
    """The LEFT suspension corner's inboard hardpoints in the suspension LOCAL frame
    (hub centre at origin, +X fwd, +Y OUTBOARD, +Z up), read from
    ``suspension_nx.engineering.hardpoints`` -- the single source of truth. Deferred,
    NX-free import; returns None if the suspension package is unavailable."""
    try:
        from suspension_nx.engineering import hardpoints as s_hardpoints
        from suspension_nx.params import SuspensionParams
        hp = s_hardpoints(SuspensionParams())
        return {nm: tuple(float(v) for v in hp[nm])
                for nm in _SUBFRAME_HARDPOINT_NAMES if nm in hp}
    except Exception:
        return None


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class CradleParams:
    """The perimeter cradle -- a closed loop of hollow box beams (kind="prism") that
    bolts UP to the chassis subframe pads and reaches inboard/down to carry the
    suspension pickups + e-axle. The cradle is a rectangular ladder: two side rails
    running fore/aft (along ±X) at each chassis-pad Y, tied by a front and a rear
    crossbeam running laterally (along ±Y). The perimeter sits at the cradle base
    plane (low, near the pickup Z band); the chassis pads are reached UP by separate
    riser posts (see PadParams) so the cradle bolts to the rail tops at z≈400."""
    beam_width_mm: float = 60.0            # box-beam section width (across the beam)
    beam_height_mm: float = 70.0           # box-beam section height
    beam_wall_mm: float = 4.0              # box-beam wall thickness (0 < 2*wall < min(w,h))
    # fore/aft length of the cradle side rails (X span). Sized to bracket the fore/aft
    # spread of the pickups + pads with margin; validate() checks it actually spans them.
    side_rail_length_mm: float = 360.0
    # the cradle base-plane height (world Z) -- the perimeter sits at this Z, just below
    # the lower pickups so the pickup bosses rise a little and the e-axle hangs under it.
    base_plane_z_mm: float = 250.0
    # INBOARD PICKUP STRINGER: a fore/aft (±X) box beam per side at the pickup |Y| band,
    # tying the front and rear crossbeams, that the suspension pickup-boss legs land on so
    # the bosses are CARRIED by the perimeter (no mid-air boss). Without it the pickup legs
    # bottom out in empty space inboard of the side rails (review finding 5). Its |Y| is
    # set to the pickup band centre and its width spans the pickup Y spread.
    stringer_y_mm: float = 384.0           # |Y| centre of the inboard pickup stringer
    stringer_width_mm: float = 130.0       # lateral (Y) width -- spans the 344..424 pickup band


@dataclass
class PadParams:
    """The chassis-pad interface: vertical riser posts that carry the cradle UP from
    its base plane to the chassis subframe mount pads on the rail tops (ICD §7.2:
    y = ±585, z ≈ 400 at the axle x-station). Each post tops out in a bolt-flange that
    matches the chassis Subframe_Boss (chassis_nx mount_steps). Four pads (fore/aft ×
    left/right) bolt the cradle to the chassis."""
    # chassis subframe-pad mating coordinates (vehicle frame, ICD §7.2). The chassis
    # builds these at rail centre-line y = ±585 on the rail top z ≈ 400 at the axle
    # x-station; local X = pad_x_local (mirrored fore/aft about the axle station).
    pad_y_mm: float = 585.0                # rail centre-line |Y| (= chassis rail_centreline_y)
    # rail-top mating plane (world Z). The chassis builds its rail top at z=390 and a
    # subframe boss 14 mm above it; the subframe pad flange mates to that rail-top plane
    # (the ICD §7.2 "z≈400" is approximate -- 390 is what chassis_nx actually builds).
    pad_z_mm: float = 390.0
    pad_x_local_mm: float = 150.0          # fore/aft pad offset from the axle station (±X local)
    post_diameter_mm: float = 56.0         # riser-post OD (carries the cradle up to the pad)
    flange_diameter_mm: float = 90.0       # pad bolt-flange OD (= chassis Subframe_Boss OD)
    flange_thickness_mm: float = 14.0
    bolt_count: int = 4                    # bolts per pad (= chassis subframe.mount_bolt_count)
    bolt_diameter_mm: float = 14.0         # M14 (= chassis subframe.mount_bolt_diameter_mm)


@dataclass
class BossParams:
    """The suspension inboard PICKUP BOSSES: a cylindrical boss with a cross bore at
    each suspension inboard hardpoint, so the control-arm / toe-link inboard ends bolt
    to the subframe instead of floating (ICD §7.2 / §7.4.2). The boss axis is fore/aft
    (±X) -- the natural pin/bolt axis for a control-arm bushing. Coordinates come from
    the hardpoint table in this module (the single source of truth)."""
    boss_diameter_mm: float = 36.0         # boss OD around the pickup pin
    boss_length_mm: float = 44.0           # axial length along the pin axis (X)
    bore_diameter_mm: float = 16.0         # pin / bushing-bolt clearance bore (M16 class)


@dataclass
class TowerParams:
    """The body / SHOCK-TOWER boss that supports the spring/damper (strut) TOP so it
    is not floating (ICD §7.2). A vertical (+Z) post rising from the cradle to the
    damper-top hardpoint, capped by a seat plate with a central damper-rod bore and a
    bolt circle. One tower per side, reaching damper_top (z≈692 local)."""
    post_diameter_mm: float = 70.0         # tower post OD
    seat_diameter_mm: float = 120.0        # top seat-plate OD (the upper spring seat)
    seat_thickness_mm: float = 16.0
    rod_bore_diameter_mm: float = 24.0     # damper-rod / top-mount clearance bore
    bolt_count: int = 3                    # top-mount bolts
    bolt_diameter_mm: float = 10.0         # M10 top-mount studs


@dataclass
class EAxleMountParams:
    """The E-AXLE / DIFF mounts that carry the gearbox + differential (ICD §7.1/§7.2).
    The reduction housing + diff sit on the wheel axis (vehicle (axle_x, 0, r) along
    Y); the subframe presents bushed mount bosses under the cradle for the diff/gearbox
    carrier to bolt down to. Modelled as a pair of mount bosses straddling the centre
    plane at the diff carrier height."""
    enabled: bool = True
    mount_count: int = 2                   # diff/gearbox mounts (e.g. fore + aft of the diff)
    mount_y_mm: float = 110.0              # |Y| of each mount from the centre plane
    mount_z_mm: float = 300.0             # world Z of the diff/gearbox carrier mount face
    boss_diameter_mm: float = 60.0
    boss_height_mm: float = 40.0
    bolt_diameter_mm: float = 12.0         # M12 carrier bolt


@dataclass
class MaterialParams:
    """Production material spec (metadata; does not change geometry). A cast / welded
    aluminium cradle in the chassis-rail material family."""
    cradle_material: str = "A356-T6 cast aluminium (or 6082-T6 welded)"
    density_kg_m3: float = 2700.0
    # 0.2% proof stress of the cradle alloy (MPa) -- used for the load-path SF screen.
    yield_strength_mpa: float = 200.0
    joining: str = "cast nodes + MIG-welded extruded beams"


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class SubframeParams:
    name: str = "EV_subframe"
    axle: str = "rear"                     # rear | front (which axle variant to build)
    # The vehicle TRACK and loaded TYRE RADIUS (mm). The suspension corner is hub-datumed,
    # so a subframe boss must sit at HUB_CENTRE + Rz(side).hp_local, and HUB_CENTRE =
    # (axle_x, side_sign*track/2, tyre_radius). These mirror vehicle_nx.LayoutParams (the
    # platform single source of truth) so hardpoints_local() can place the bosses on the
    # suspension pickups for BOTH sides. (Defaults = platform 1580 / 335.)
    track_mm: float = 1580.0
    tyre_radius_mm: float = 335.0
    # half the inboard load reacted by the cradle's two side rails (N). <=0 => derive
    # from a representative corner load through the lower pickups (see engineering).
    corner_vertical_load_n: float = 0.0
    cradle: CradleParams = field(default_factory=CradleParams)
    pad: PadParams = field(default_factory=PadParams)
    boss: BossParams = field(default_factory=BossParams)
    tower: TowerParams = field(default_factory=TowerParams)
    eaxle: EAxleMountParams = field(default_factory=EAxleMountParams)
    material: MaterialParams = field(default_factory=MaterialParams)

    # -- serialisation (identical contract to chassis_nx.params) ----------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SubframeParams":
        kwargs: Dict[str, Any] = {}
        for f in fields(cls):
            if f.name not in data:
                continue
            value = data[f.name]
            if f.name in _GROUP_TYPES and isinstance(value, dict):
                kwargs[f.name] = _merge_group(_GROUP_TYPES[f.name], value)
            else:
                kwargs[f.name] = value
        return cls(**kwargs)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    @classmethod
    def from_json(cls, text: str) -> "SubframeParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "SubframeParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "SubframeParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"axle": "front", "tower.post_diameter_mm": 80.0})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return SubframeParams.from_dict(data)

    # -- hardpoint accessor (DERIVED from the suspension, the single source of truth) -- #
    def hardpoints_local(self, side: str = "l") -> Dict[str, Tuple[float, float, float]]:
        """Suspension inboard hardpoints + damper top in the subframe LOCAL frame for one
        side, DERIVED from the suspension hardpoint table so the subframe bosses land on
        the suspension pickups on ALL FOUR corners.

        The suspension corner is placed by the vehicle assembly at
        ``HUB_CENTRE(axle, side) + Rz(side) . hp_local``; the subframe is placed IDENTITY
        at the axle station ``(axle_x, 0, 0)``. Equating the two world points gives the
        subframe-local boss::

            boss_local(side) = Rz(side) . hp_local + (0, side_sign * track/2, tyre_radius)

        with ``Rz(LEFT)=identity``, ``Rz(RIGHT)=rot_z(180)`` (negates local x AND y) and
        ``side_sign = +1 (LEFT) / -1 (RIGHT)``. The SAME canonical corner is used front
        and rear -- there is NO front X mirror (the suspension is not X-mirrored on the
        front axle; only ``axle_x`` changes, which the assembler applies via the placement
        origin). ``side`` in {"l","r"}. Returns a name -> (x, y, z) map in mm.

        Sources the suspension hardpoints from ``suspension_nx`` (NX-free); falls back to
        the frozen platform-default LEFT table only if that package is unavailable."""
        side_sign = -1.0 if side == "r" else 1.0
        half_track = self.track_mm / 2.0
        r = self.tyre_radius_mm
        s_hp = _suspension_hardpoints_local()
        if s_hp is not None:
            out: Dict[str, Tuple[float, float, float]] = {}
            for name, (hx, hy, hz) in s_hp.items():
                # Rz(side): identity for LEFT, rot_z(180) negates x,y for RIGHT
                rx, ry = (hx, hy) if side_sign > 0 else (-hx, -hy)
                out[name] = (rx, side_sign * half_track + ry, r + hz)
            return out
        # fallback: the suspension package is absent -- use the frozen LEFT subframe-local
        # table. The right corner is Rz(180) of the same canonical corner, which (after the
        # +-track/2 offset is folded into the stored LEFT values) negates BOTH local x and y.
        sgn = side_sign
        return {
            name: (sgn * x, sgn * y, z)
            for name, (x, y, z) in _LEFT_HARDPOINTS_LOCAL_FALLBACK.items()
        }

    def pad_centre_local(self, fore_aft: str, side: str) -> Tuple[float, float, float]:
        """Chassis-pad mating-face centre (subframe LOCAL frame), matching the chassis
        Subframe_Boss on the rail top. `fore_aft` in {"fore","aft"} -> ±X; `side` in
        {"l","r"} -> ±Y. The front axle mirrors X like the hardpoints."""
        x_sign = -1.0 if self.axle == "front" else 1.0
        fa_sign = 1.0 if fore_aft == "fore" else -1.0
        y_sign = -1.0 if side == "r" else 1.0
        return (x_sign * fa_sign * self.pad.pad_x_local_mm,
                y_sign * self.pad.pad_y_mm,
                self.pad.pad_z_mm)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Names are NX-expression safe (group_field).
        Only the geometric groups carry dimensions; material metadata is skipped."""
        out: List[Tuple[str, float, str]] = [
            ("corner_vertical_load_n", float(self.corner_vertical_load_n), ""),
        ]
        for group_name in ("cradle", "pad", "boss", "tower", "eaxle"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif f.name.endswith("_n"):
                    unit = ""    # a force, not a length
                elif isinstance(val, int):
                    unit = ""    # dimensionless count
                else:
                    unit = "mm"
                out.append(("%s_%s" % (group_name, f.name), float(val), unit))
        return out


_GROUP_TYPES = {
    "cradle": CradleParams,
    "pad": PadParams,
    "boss": BossParams,
    "tower": TowerParams,
    "eaxle": EAxleMountParams,
    "material": MaterialParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> SubframeParams:
    """The reference rear EV subframe variant (carries the default suspension corner +
    e-axle, bolts to the default chassis_nx rear subframe pads)."""
    return SubframeParams()
