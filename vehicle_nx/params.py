"""Vehicle-assembly parameters: the overall layout (wheelbase / track / ride) and
where each subsystem part sits. Plain dataclasses, JSON-serialisable, mirroring the
style of the subsystem `params.py` modules.

Units: millimetres (mm) and degrees (deg). Vehicle frame: +X forward, +Y left,
+Z up (ISO 8855); origin at the chassis centre, ground at z = 0.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List


@dataclass
class LayoutParams:
    wheelbase_mm: float = 2875.0
    track_front_mm: float = 1580.0
    track_rear_mm: float = 1580.0
    tyre_radius_mm: float = 335.0      # loaded radius (e.g. 235/45R18) => hub-centre height
    drive_layout: str = "rear"         # rear | front | awd (which axle(s) carry the e-axle)
    suspension_corners: int = 4        # 4 = model all corners; 2 = driven axle only


@dataclass
class EAxlePlacement:
    """Where the motor + driveline (e-axle) and the inverter mount, relative to the
    driven axle centre. The motor sits parallel to and offset from the wheel axis by
    the reduction-gearbox final-drive CENTRE DISTANCE; the inverter mounts on top of
    the motor.

    The motor offsets DEFAULT TO AUTO (0.0): the assembler reads the real centre
    distance from ``gearbox_nx.engineering.derive()`` (``motor_offset_dx_mm`` toward the
    vehicle centre, ``motor_offset_dz_mm`` up) so the motor lands on the gearbox's
    motor-mounting flange and clears the differential (ICD §7.1). Setting a non-zero
    value overrides the gearbox-derived offset for a one-off layout.

    The OLD hard defaults (60 / 110 mm) put the motor only ~125 mm from the diff axis,
    well under the ICD §7.1 minimum centre distance (~231 mm = motor_OD/2 + ring/2 +
    15), which is exactly what made the motor and differential interpenetrate. The
    gearbox-derived offset (~156 / ~176 -> ~235 mm centre distance) clears it."""
    motor_offset_x_mm: float = 0.0     # 0 => AUTO from gearbox C1+C2 (toward vehicle centre)
    motor_offset_z_mm: float = 0.0     # 0 => AUTO from gearbox C1+C2 (up); final-drive centre dist.
    inverter_offset_z_mm: float = 180.0  # inverter above the motor axis
    inverter_offset_x_mm: float = 0.0


@dataclass
class PartFiles:
    """The per-subsystem .prt file names the assembler adds as components. The
    assembler builds these first (from each package's blueprint) unless they exist."""
    motor: str = "motor_out.prt"
    driveline: str = "driveline_out.prt"
    inverter: str = "inverter_out.prt"
    suspension: str = "suspension_out.prt"
    chassis: str = "chassis_out.prt"
    # connector / realism parts (ICD §7): the reduction gearbox that bridges
    # motor<->differential, and the suspension/e-axle subframe (cradle) that closes the
    # chassis<->suspension joint. Built front/rear from the SAME gearbox/subframe
    # blueprint (the subframe's suspension-derived bosses are identical front/rear -- the
    # suspension reuses one canonical corner, so there is NO front X mirror).
    gearbox: str = "gearbox_out.prt"
    subframe: str = "subframe_out.prt"
    include_inverter: bool = True
    include_chassis: bool = True
    include_gearbox: bool = True       # ICD §7.1 e-axle reduction-gearbox connector
    include_subframe: bool = True      # ICD §7.2 suspension/e-axle subframe cradle


@dataclass
class VehicleParams:
    name: str = "EV_vehicle"
    layout: LayoutParams = field(default_factory=LayoutParams)
    eaxle: EAxlePlacement = field(default_factory=EAxlePlacement)
    parts: PartFiles = field(default_factory=PartFiles)

    # -- serialisation (identical contract to the subsystem params) -------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VehicleParams":
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
    def from_json(cls, text: str) -> "VehicleParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "VehicleParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "VehicleParams":
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return VehicleParams.from_dict(data)


_GROUP_TYPES = {
    "layout": LayoutParams,
    "eaxle": EAxlePlacement,
    "parts": PartFiles,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> VehicleParams:
    return VehicleParams()
