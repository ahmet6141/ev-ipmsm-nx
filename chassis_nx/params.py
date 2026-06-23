"""Parametric inputs for the EV "skateboard" chassis (frame rails + crossmembers
+ battery tray + suspension/e-axle subframe mounts + body mounts).

Mirrors :mod:`driveline_nx.params`: plain dataclasses so one :class:`ChassisParams`
fully describes a variant, variants load from / save to JSON, and every numeric
dimension is pushed into Siemens NX as a named *expression* so the built body
stays editable.

Units: millimetres (mm) unless noted. Vehicle frame: X = longitudinal (front +X),
Y = lateral, Z = vertical (up). The two longitudinal rails run along X spaced
frame_inner_width apart (inner faces); crossmembers tie them along Y; the battery
tray sits low between the rails. engineering.validate() flags impossible
combinations before a build.

Defaults target a mid-size passenger-EV skateboard platform (~Tesla Model 3 / VW
MEB class): ~2875 mm wheelbase, ~1580 mm track, a ~75 kWh floor battery.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class FrameParams:
    """The structural backbone: two longitudinal rails (hollow box beams running
    along X) tied by a series of lateral crossmembers (hollow box beams along Y).
    `frame_inner_width` is the lateral spacing between the inner faces of the two
    rails -- the clear channel the battery tray drops into."""
    wheelbase_mm: float = 2875.0
    track_front_mm: float = 1580.0
    track_rear_mm: float = 1580.0
    overall_length_mm: float = 4690.0

    # longitudinal rail box-beam section
    rail_width_mm: float = 70.0
    rail_height_mm: float = 120.0
    rail_wall_mm: float = 4.0

    # lateral crossmembers
    crossmember_count: int = 5
    crossmember_width_mm: float = 60.0
    crossmember_height_mm: float = 90.0
    crossmember_wall_mm: float = 3.0

    # lateral spacing between the inner faces of the two longitudinal rails
    frame_inner_width_mm: float = 1100.0


@dataclass
class BatteryTrayParams:
    """The sealed structural battery tray that drops between the rails (the floor
    of the skateboard). Modelled as a hollow box with a few lateral crossbraces.
    `sealed` flags an IP-rated enclosure (a lid is fitted; not modelled here)."""
    enabled: bool = True
    length_mm: float = 2400.0
    width_mm: float = 1450.0
    height_mm: float = 110.0
    wall_mm: float = 4.0
    crossbrace_count: int = 6
    sealed: bool = True


@dataclass
class SubframeParams:
    """Front and rear suspension/e-axle subframe interfaces. Each subframe bolts to
    the rails through `mount_bolt_count` bosses; the rear (and optionally front)
    subframe also carries `motor_mount_count` e-axle (motor + driveline) mounts."""
    front_subframe: bool = True
    rear_subframe: bool = True
    mount_bolt_count: int = 4
    mount_bolt_diameter_mm: float = 14.0
    motor_mount_count: int = 3


@dataclass
class BodyMountParams:
    """Body-in-white attachment + crash structure. `body_mount_count` holes locate
    the cabin/body above the platform; the front/rear crush cans are the
    energy-absorbing crash structure ahead of / behind the wheelbase."""
    body_mount_count: int = 10
    body_mount_diameter_mm: float = 12.0
    crush_can_front: bool = True
    crush_can_rear: bool = True


@dataclass
class MaterialParams:
    """Production material spec (metadata; does not change geometry)."""
    rail_material: str = "6082-T6 extruded aluminium"
    density_kg_m3: float = 2700.0
    joining: str = "MIG + structural adhesive + FDS"


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class ChassisParams:
    name: str = "EV_skateboard_chassis"
    frame: FrameParams = field(default_factory=FrameParams)
    battery_tray: BatteryTrayParams = field(default_factory=BatteryTrayParams)
    subframe: SubframeParams = field(default_factory=SubframeParams)
    body_mount: BodyMountParams = field(default_factory=BodyMountParams)
    material: MaterialParams = field(default_factory=MaterialParams)

    # -- serialisation (identical contract to driveline_nx.params) --------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ChassisParams":
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
    def from_json(cls, text: str) -> "ChassisParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "ChassisParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "ChassisParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"frame.rail_height_mm": 140.0, "battery_tray.enabled": False})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return ChassisParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Names are NX-expression safe (group_field).
        Only the geometric groups carry dimensions; material metadata is skipped."""
        out: List[Tuple[str, float, str]] = []
        for group_name in ("frame", "battery_tray", "subframe", "body_mount"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif f.name.endswith("_ratio"):
                    unit = ""
                elif isinstance(val, int):
                    unit = ""   # dimensionless count
                else:
                    unit = "mm"
                out.append(("%s_%s" % (group_name, f.name), float(val), unit))
        return out


_GROUP_TYPES = {
    "frame": FrameParams,
    "battery_tray": BatteryTrayParams,
    "subframe": SubframeParams,
    "body_mount": BodyMountParams,
    "material": MaterialParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> ChassisParams:
    """The reference EV skateboard-chassis variant (houses the default motor_nx
    motor + driveline_nx driveline)."""
    return ChassisParams()
