"""Parametric inputs for the EV driveline (differential + half-shafts + wheel hubs).

Mirrors :mod:`motor_nx.params`: plain dataclasses so one :class:`DrivelineParams`
fully describes a variant, variants load from / save to JSON, and every numeric
dimension is pushed into Siemens NX as a named *expression* so the built body
stays editable.

Units: millimetres (mm) and degrees (deg) unless noted. The driveline rotation
axis is +Z; the differential centre is z = 0 and the two half-shafts exit along
-Z (left) and +Z (right). The motor's keyed output stub feeds the input pinion
flange (offset, parallel-axis final drive).

Defaults target the same passenger-EV rear-drive-unit class as the motor
(~Tesla Model 3 RDU): a ~9:1 single-speed final drive carrying the motor's
440 Nm peak. engineering.validate() flags impossible combinations before a build.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class DifferentialParams:
    """Final-drive + differential. `type` selects the next-gen differential:
      open            -- conventional bevel-gear open differential
      elsd            -- electronically-controlled limited-slip (multi-plate clutch)
      torque_vectoring-- twin-clutch active eDiff (independent left/right torque)
      spool           -- locked / welded (track use)
    A `disconnect` decoupler can physically de-clutch the driveline for drag-free
    coasting (range), independent of the diff type."""
    type: str = "torque_vectoring"     # open | elsd | torque_vectoring | spool
    final_drive_ratio: float = 9.0     # motor rev : wheel rev (single-speed EV)
    disconnect: bool = True            # dog/clutch decoupler for free-wheel coasting

    # carrier / case (modelled as a representative tube + side bosses)
    carrier_outer_diameter: float = 150.0
    carrier_length: float = 92.0       # axial length of the diff case
    carrier_wall: float = 12.0         # case wall (outer - bore)

    # ring gear (modelled as a BLANK at pitch diameter -- teeth cut by hobbing)
    ring_gear_pitch_diameter: float = 208.0
    ring_gear_face_width: float = 30.0
    ring_gear_bolt_count: int = 12     # ring-gear-to-carrier bolt circle
    ring_gear_bolt_diameter: float = 10.0   # M10 clearance

    # side gears / output bosses to the half-shaft splines
    side_gear_diameter: float = 62.0
    side_gear_length: float = 30.0

    # input pinion + motor-coupling flange (parallel-axis, offset from the wheel axis)
    input_pinion_pitch_diameter: float = 46.0
    input_flange_diameter: float = 96.0
    input_flange_thickness: float = 16.0
    input_flange_bolt_count: int = 8
    input_flange_bolt_diameter: float = 9.0     # M8 clearance
    input_bore_diameter: float = 32.0           # = motor drive-stub diameter (keyed)
    input_keyway_width: float = 10.0            # matches the motor DIN 6885-A key
    input_keyway_depth: float = 4.0


@dataclass
class HalfshaftParams:
    """The two drive (half) shafts, each with an inboard tripod (plunging) joint
    and an outboard Rzeppa (fixed) joint. Hollow shafts are lighter + raise the
    first torsional mode (NVH); set bore_diameter = 0 for a solid shaft."""
    diameter: float = 44.0              # sized so the WORST-CASE wheel torque keeps static
                                        # SF >= 1.5. The default diff is a torque-vectoring
                                        # eDiff (bias 1.0 -> the full 3960 Nm axle torque can
                                        # pass through one shaft); at 44 mm OD / 18 mm bore the
                                        # static shear SF is ~1.7. Half-shafts are
                                        # fatigue-critical, so this >1.5 static margin backstops
                                        # the separate fatigue check.
    bore_diameter: float = 18.0         # hollow-shaft bore (0 => solid)
    length: float = 520.0               # bar length between the two CV-joint bells
    spline_diameter: float = 28.0       # stub spline into the wheel hub

    inboard_joint: str = "tripod"       # plunging joint (accommodates suspension travel)
    inboard_bell_diameter: float = 90.0
    inboard_bell_length: float = 70.0

    outboard_joint: str = "rzeppa"      # fixed CV joint (high articulation at the wheel)
    outboard_bell_diameter: float = 96.0
    outboard_bell_length: float = 76.0
    max_articulation_deg: float = 47.0  # outboard fixed-joint max steer/articulation


@dataclass
class WheelHubParams:
    """Gen-3 wheel-hub bearing unit + the wheel-mounting interface (the "wheel
    connection elements"): bolt circle (PCD) of lug studs, the centre pilot bore,
    the brake-disc pilot and the ABS/wheel-speed encoder ring."""
    bearing_generation: int = 3         # Gen-3 flanged hub-bearing unit (info)
    bearing_outer_diameter: float = 84.0
    bearing_width: float = 39.0
    bearing_bore_diameter: float = 45.0

    hub_flange_diameter: float = 150.0
    hub_flange_thickness: float = 20.0
    pilot_bore_diameter: float = 64.1   # wheel centre (hub-centric) bore
    brake_pilot_diameter: float = 160.0 # brake-disc centring pilot face (0 => none)
    brake_pilot_height: float = 8.0

    # wheel fastening -- studs on a bolt circle, OR a single motorsport centre nut
    single_centre_nut: bool = False     # True => centre-lock (one big nut), no PCD studs
    centre_nut_diameter: float = 52.0
    lug_count: int = 5
    lug_pcd: float = 114.3              # 5x114.3 passenger-EV pattern
    lug_hole_diameter: float = 14.0     # M12 stud clearance / press-in stud seat

    abs_encoder_ring: bool = True       # magnetic wheel-speed encoder ring on the hub
    encoder_ring_diameter: float = 70.0
    encoder_ring_width: float = 6.0


@dataclass
class MaterialParams:
    """Production material spec (metadata; does not change geometry)."""
    halfshaft_steel: str = "induction-hardened 4140 / 36MnVS6 (hollow swaged)"
    gear_steel: str = "case-carburised 18CrNiMo7-6 (ground teeth)"
    carrier_material: str = "spheroidal-graphite cast iron (GJS-500) / forged"
    hub_material: str = "forged steel hub + cast-Al knuckle interface"
    # allowable torsional shear for induction-hardened shaft steel. This is a STATIC
    # (peak-torque) allowable; half-shafts are fatigue-critical, so the SF computed
    # against it is a static screen only -- a full design must also pass a fatigue
    # (e.g. ISO 1099 / load-spectrum) check, which typically governs.
    halfshaft_shear_allow_mpa: float = 420.0
    gear_steel_density: float = 7850.0


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class DrivelineParams:
    name: str = "EV_driveline"
    sides: str = "both"                 # both | left | right (which wheel ends to model)
    motor_peak_torque_nm: float = 440.0 # from motor_nx em_design (peak); drives sizing
    motor_max_speed_rpm: float = 18000.0
    # Vehicle TRACK the built driveline must span (ICD §3/§4). The two wheel-hub
    # flange faces are placed at local z = +-target_track_mm/2 so that, after the
    # assembly transform Rx(-90) about the differential centre, each flange lands on
    # the shared HUB_CENTRE at vehicle y = +-T/2. The inboard plunge clearance is
    # solved (engineering.inboard_clearance) to absorb the residual length so the
    # face hits T/2 exactly while every real component keeps its catalogue size;
    # validate() asserts the achieved track is within +-2 % of this target.
    target_track_mm: float = 1580.0     # = vehicle_nx LayoutParams.track_* (single SoT)
    differential: DifferentialParams = field(default_factory=DifferentialParams)
    halfshaft: HalfshaftParams = field(default_factory=HalfshaftParams)
    wheel_hub: WheelHubParams = field(default_factory=WheelHubParams)
    material: MaterialParams = field(default_factory=MaterialParams)

    # -- serialisation (identical contract to motor_nx.params) ------------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DrivelineParams":
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
    def from_json(cls, text: str) -> "DrivelineParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "DrivelineParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "DrivelineParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"halfshaft.diameter": 32.0, "differential.type": "elsd"})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return DrivelineParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Names are NX-expression safe (group_field)."""
        out: List[Tuple[str, float, str]] = [
            ("final_drive_ratio", self.differential.final_drive_ratio, ""),
            ("target_track_mm", float(self.target_track_mm), "mm"),
        ]
        for group_name in ("differential", "halfshaft", "wheel_hub"):
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
    "differential": DifferentialParams,
    "halfshaft": HalfshaftParams,
    "wheel_hub": WheelHubParams,
    "material": MaterialParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> DrivelineParams:
    """The reference EV driveline variant (mates to the default motor_nx motor)."""
    return DrivelineParams()
