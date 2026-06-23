"""Parametric inputs for one EV corner suspension (knuckle + control arms + coil
spring + damper + anti-roll bar) that carries the Gen-3 wheel hub from
:mod:`driveline_nx`.

Mirrors :mod:`driveline_nx.params`: plain dataclasses so one
:class:`SuspensionParams` fully describes a variant, variants load from / save to
JSON, and every numeric dimension is pushed into Siemens NX as a named
*expression* so the built body stays editable.

Units: millimetres (mm) and degrees (deg) unless a field comment notes SI. The
corner is modelled in a LOCAL frame: X = vehicle longitudinal, Y = lateral
(outboard = +Y toward the wheel), Z = vertical (up). The knuckle / upright sits
outboard; control-arm chassis pickups sit inboard.

Defaults target the same passenger-EV rear-drive-unit class as the motor +
driveline (~Tesla Model 3 RDU class): a rear multi-link corner, ~140 mm ride
height, semi-active (CDC) damper. engineering.validate() flags impossible /
out-of-band combinations before a build.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class GeometryParams:
    """Corner kinematics + hardpoint geometry. `type` selects the linkage:
      multilink        -- rear multi-link (lower + upper arm + separate toe link)
      double_wishbone  -- upper + lower A-arms (toe via a tie/toe link)
      macpherson       -- strut + single lower arm (the damper is the upper link)
    Lengths are nominal arm bar lengths from the inboard pickup to the knuckle;
    the angles set the steering-axis geometry of the corner."""
    type: str = "multilink"             # multilink | double_wishbone | macpherson
    track_width_mm: float = 1580.0      # full axle track (wheel-centre to wheel-centre)
    lower_arm_length_mm: float = 380.0
    upper_arm_length_mm: float = 300.0
    toe_link_length_mm: float = 320.0
    ride_height_mm: float = 140.0       # static wheel-centre height (local Z reference)
    kingpin_inclination_deg: float = 8.0
    caster_deg: float = 5.0
    camber_deg: float = -1.5
    scrub_radius_mm: float = 15.0


@dataclass
class SpringParams:
    """Main coil spring. The motion ratio maps wheel travel to spring travel
    (spring travel = motion_ratio x wheel travel); the wheel rate scales with
    motion_ratio^2."""
    spring_rate_n_per_mm: float = 45.0
    free_length_mm: float = 300.0
    coil_outer_diameter_mm: float = 140.0
    motion_ratio: float = 0.62          # spring travel : wheel travel (dimensionless)
    ride_height_load_n: float = 4500.0  # static corner spring load at ride height (N)


@dataclass
class DamperParams:
    """Telescopic damper. Rates are in N.s/m (SI) at the damper. `adaptive` flags a
    semi-active / continuously-variable (CDC) valve."""
    damper_diameter_mm: float = 46.0
    damper_length_mm: float = 420.0     # extended body length (envelope)
    bump_rate_ns_per_m: float = 3500.0  # compression damping coefficient (N.s/m)
    rebound_rate_ns_per_m: float = 5200.0  # extension damping coefficient (N.s/m)
    adaptive: bool = True               # semi-active / CDC valve


@dataclass
class ArmParams:
    """Control-arm bar + its mount features. Hollow arms (wall > 0) are lighter;
    set arm_wall_mm = 0 for a solid bar."""
    arm_diameter_mm: float = 28.0
    arm_wall_mm: float = 4.0            # bar wall (0 => solid)
    bushing_diameter_mm: float = 40.0   # inboard compliance-bushing OD
    ball_joint_diameter_mm: float = 22.0  # outboard ball-joint / link-end OD


@dataclass
class AntiRollBarParams:
    """Anti-roll (stabiliser) bar. `rate_nm_per_deg` is the bar's roll rate
    contribution (Nm per degree of body roll)."""
    enabled: bool = True
    bar_diameter_mm: float = 24.0
    arm_length_mm: float = 180.0        # lever arm from the bar to the drop-link
    rate_nm_per_deg: float = 480.0      # roll stiffness contribution (Nm/deg)


@dataclass
class KnuckleParams:
    """Knuckle / upright that carries the wheel-hub bearing. hub_bore_diameter_mm
    must match the Gen-3 hub bearing OD from driveline_nx
    (wheel_hub.bearing_outer_diameter)."""
    hub_bore_diameter_mm: float = 84.0  # = driveline_nx wheel_hub.bearing_outer_diameter
    height_mm: float = 180.0
    width_mm: float = 120.0
    thickness_mm: float = 40.0
    brake_caliper_mount: bool = True


@dataclass
class MassParams:
    """Corner masses (SI, kg). Sprung = body share at the corner; unsprung = wheel,
    tyre, hub, brake + the moving suspension share."""
    sprung_corner_mass_kg: float = 420.0
    unsprung_corner_mass_kg: float = 42.0


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class SuspensionParams:
    name: str = "EV_suspension_corner"
    corners: str = "one"                # one | axle (axle => model a mirrored pair)
    geometry: GeometryParams = field(default_factory=GeometryParams)
    spring: SpringParams = field(default_factory=SpringParams)
    damper: DamperParams = field(default_factory=DamperParams)
    arm: ArmParams = field(default_factory=ArmParams)
    antiroll: AntiRollBarParams = field(default_factory=AntiRollBarParams)
    knuckle: KnuckleParams = field(default_factory=KnuckleParams)
    mass: MassParams = field(default_factory=MassParams)

    # -- serialisation (identical contract to driveline_nx.params) --------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SuspensionParams":
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
    def from_json(cls, text: str) -> "SuspensionParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "SuspensionParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "SuspensionParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"spring.spring_rate_n_per_mm": 50.0, "geometry.type": "macpherson"})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return SuspensionParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Names are NX-expression safe (group_field).
        Iterates the numeric mm/deg/dimensionless fields of the geometric groups."""
        out: List[Tuple[str, float, str]] = []
        for group_name in ("geometry", "spring", "damper", "arm", "antiroll", "knuckle"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif f.name.endswith("_mm"):
                    unit = "mm"
                else:
                    unit = ""   # ratio / rate / dimensionless
                out.append(("%s_%s" % (group_name, f.name), float(val), unit))
        return out


_GROUP_TYPES = {
    "geometry": GeometryParams,
    "spring": SpringParams,
    "damper": DamperParams,
    "arm": ArmParams,
    "antiroll": AntiRollBarParams,
    "knuckle": KnuckleParams,
    "mass": MassParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> SuspensionParams:
    """The reference EV corner-suspension variant (carries the default driveline hub)."""
    return SuspensionParams()
