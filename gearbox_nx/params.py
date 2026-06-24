"""Parametric inputs for the EV reduction gearbox (the motor<->differential
connector, ICD §7.1).

Mirrors :mod:`motor_nx.params` / :mod:`driveline_nx.params`: plain dataclasses so one
:class:`GearboxParams` fully describes a variant, variants load from / save to JSON,
and every numeric dimension is pushed into Siemens NX as a named *expression* so the
built body stays editable.

Units: millimetres (mm) and degrees (deg) unless noted. The gear axes are +Z; the
DIFFERENTIAL axis is the local origin (x = y = 0), the LAYSHAFT and the MOTOR axes are
parallel to it, offset in the local XY plane (see :mod:`gearbox_nx.engineering`).

Coordinate / kinematic abstraction
    * gears are represented as BLANKS at pitch diameter (teeth cut later by hobbing) --
      the same "envelope" philosophy motor_nx uses for end-windings and driveline_nx
      uses for its ring/side/pinion gears; the production interfaces (the motor flange,
      bolt circles, pilot, the diff coupling) are modelled exactly.
    * the two stage centre distances are the SOURCE OF TRUTH for the motor<->diff
      offset: engineering derives ``motor_offset = centre_distance_1 + centre_distance_2``
      and the vehicle assembler places the motor at that offset (ICD §7.1), so the gear
      train always physically reaches and the motor/diff envelopes always clear.

Defaults target the same passenger-EV rear-drive-unit class as the motor + driveline
(~Tesla Model 3 RDU): a ~9.4:1 two-stage reduction carrying the motor's 440 Nm peak.
engineering.validate() flags impossible combinations before a build is attempted.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class GearStageParams:
    """One parallel-axis reduction stage as a pinion + gear BLANK pair at pitch
    diameter. The pitch diameter of a spur/helical gear is ``module * teeth``; the
    centre distance is ``(pinion_pd + gear_pd) / 2 = module * (z_pinion + z_gear) / 2``.

    Stage 1 (motor -> layshaft) runs at high speed / low torque, so a smaller module
    and narrower face; stage 2 (layshaft -> output) carries the multiplied torque, so
    a larger module + wider face. Tooth counts are chosen to give the target overall
    ratio while the two centre distances sum to the motor<->diff offset."""
    module_mm: float = 2.5            # gear module m (pitch diameter = m * teeth)
    pinion_teeth: int = 19            # driving (smaller) gear tooth count z_p
    gear_teeth: int = 53              # driven (larger) gear tooth count z_g
    face_width_mm: float = 30.0       # axial gear face width (blank length)
    pressure_angle_deg: float = 20.0  # standard involute pressure angle (info / rating)
    helix_angle_deg: float = 25.0     # helical (quiet EV gear); 0 => spur (info / rating)


@dataclass
class LayshaftParams:
    """The intermediate (counter) shaft carrying stage-1 GEAR + stage-2 PINION. It
    sits BETWEEN the motor and the differential axes (ICD §7.1) so the train walks
    motor -> layshaft -> diff. Modelled as a journalled shaft blank spanning both
    gear faces plus bearing seats at each end."""
    shaft_diameter_mm: float = 38.0   # layshaft journal diameter
    bore_diameter_mm: float = 0.0     # hollow layshaft bore (0 => solid)
    bearing_seat_diameter_mm: float = 35.0
    bearing_seat_length_mm: float = 20.0
    # axial gap between the stage-1 gear face and the stage-2 pinion face on the shaft
    inter_gear_gap_mm: float = 8.0


@dataclass
class HousingParams:
    """The cast reduction housing (representative shell). It is the structural
    connector: it BOLTS to the motor DE flange on one side and ENCLOSES / MOUNTS the
    differential carrier on the other, bridging motor<->diff with no gap and no
    overlap. Modelled as a prism shell (an outer cast wall with the gear cavity left
    hollow) spanning the gear-axis length, plus the two mounting flanges.

    The motor-mounting flange dimensions DEFAULT to AUTO (0.0): engineering reads the
    real motor DE flange (diameter / bolt circle / pilot) from motor_nx so the gearbox
    bolts straight onto it (ICD §7.1). Set a non-zero value to override."""
    wall_thickness_mm: float = 10.0      # cast wall thickness of the shell
    axial_length_mm: float = 96.0        # gear-axis length of the housing cavity (>= widest gear face)
    end_cover_thickness_mm: float = 12.0 # bolted end-cover plate thickness at each axial end
    radial_clearance_mm: float = 12.0    # min radial gap from a gear tip to the inner wall (oil + tolerance)

    # --- motor-mounting flange (matches the motor DE housing flange) -------- #
    # 0.0 => AUTO: take the value from motor_nx (motor DE flange OD, mount PCD, mount
    # bolt count/diameter, pilot diameter). A non-zero value overrides for a variant.
    motor_flange_diameter_mm: float = 0.0
    motor_flange_thickness_mm: float = 16.0
    motor_bolt_circle_diameter_mm: float = 0.0
    motor_bolt_count: int = 0
    motor_bolt_diameter_mm: float = 0.0
    motor_pilot_diameter_mm: float = 0.0   # spigot bore the motor DE spigot pilots into

    # --- differential-carrier mounting interface ---------------------------- #
    diff_carrier_diameter_mm: float = 150.0  # = driveline carrier OD (the bore that mounts it)
    diff_mount_flange_diameter_mm: float = 200.0
    diff_mount_thickness_mm: float = 14.0
    diff_mount_bolt_count: int = 8
    diff_mount_bolt_diameter_mm: float = 11.0  # M10 clearance

    # --- oil sump --------------------------------------------------------- #
    oil_sump_depth_mm: float = 35.0      # cast sump depth below the lowest gear (oil bath)
    oil_fill_fraction: float = 0.35      # fraction of the sump volume filled (splash lubrication)


@dataclass
class OutputCouplingParams:
    """The gearbox OUTPUT -> differential coupling. The output GEAR (stage-2 driven
    gear) is coaxial with the differential axis; its hub presents the coupling flange
    that the driveline's existing ``diff_input_pinion`` / ``input_flange`` mates to.
    Sized to the driveline input flange (read from driveline_nx, do not edit it)."""
    flange_diameter_mm: float = 96.0     # = driveline DifferentialParams.input_flange_diameter
    flange_thickness_mm: float = 16.0
    bore_diameter_mm: float = 32.0       # = driveline input_bore_diameter (keyed)
    bolt_count: int = 8
    bolt_diameter_mm: float = 9.0        # M8 clearance


@dataclass
class MaterialParams:
    """Production material spec (metadata; does not change geometry)."""
    housing_material: str = "high-pressure die-cast aluminium (AlSi9Cu3) e-axle housing"
    housing_density: float = 2700.0      # kg/m^3 (cast aluminium)
    gear_steel: str = "case-carburised 18CrNiMo7-6 (ground helical teeth)"
    gear_steel_density: float = 7850.0
    shaft_steel: str = "case-hardened 20MnCr5"
    # allowable contact / bending stress (info; verify with ISO 6336)
    gear_contact_allow_mpa: float = 1500.0
    gear_bending_allow_mpa: float = 430.0
    oil_grade: str = "ATF / 75W e-axle gear oil (splash + channel)"


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class GearboxParams:
    name: str = "EV_reduction_gearbox"
    motor_peak_torque_nm: float = 440.0   # from motor_nx em_design (peak); drives gear sizing
    motor_max_speed_rpm: float = 18000.0
    # The layshaft sits between the motor and the diff. `layshaft_collinear` places the
    # three axes (diff -> layshaft -> motor) on one line so centre_distance_1 +
    # centre_distance_2 IS exactly the motor<->diff offset (the inline reduction layout
    # that guarantees the train reaches). The motor direction from the diff axis (toward
    # the vehicle centre-plane + up, ICD §7.1) is set by motor_dir_angle_deg.
    layshaft_collinear: bool = True
    motor_dir_angle_deg: float = 48.5     # direction of the motor axis from the diff axis in
    #                                       local XY (~atan2(dz,dx) for dx~155, dz~175 -> clears)
    stage1: GearStageParams = field(default_factory=lambda: GearStageParams(
        module_mm=2.5, pinion_teeth=19, gear_teeth=53, face_width_mm=30.0))
    stage2: GearStageParams = field(default_factory=lambda: GearStageParams(
        module_mm=3.5, pinion_teeth=19, gear_teeth=64, face_width_mm=38.0))
    layshaft: LayshaftParams = field(default_factory=LayshaftParams)
    housing: HousingParams = field(default_factory=HousingParams)
    output: OutputCouplingParams = field(default_factory=OutputCouplingParams)
    material: MaterialParams = field(default_factory=MaterialParams)

    # -- serialisation (identical contract to motor_nx.params) ------------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "GearboxParams":
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
    def from_json(cls, text: str) -> "GearboxParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "GearboxParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "GearboxParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"stage2.gear_teeth": 60, "housing.wall_thickness_mm": 12})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return GearboxParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Names are NX-expression safe (group_field)."""
        out: List[Tuple[str, float, str]] = [
            ("motor_peak_torque_nm", float(self.motor_peak_torque_nm), ""),
            ("motor_dir_angle_deg", float(self.motor_dir_angle_deg), "deg"),
        ]
        for group_name in ("stage1", "stage2", "layshaft", "housing", "output"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif f.name.endswith("_fraction") or f.name.endswith("_teeth"):
                    unit = ""
                elif isinstance(val, int):
                    unit = ""   # dimensionless count
                else:
                    unit = "mm"
                out.append(("%s_%s" % (group_name, f.name), float(val), unit))
        return out


_GROUP_TYPES = {
    "stage1": GearStageParams,
    "stage2": GearStageParams,
    "layshaft": LayshaftParams,
    "housing": HousingParams,
    "output": OutputCouplingParams,
    "material": MaterialParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> GearboxParams:
    """The reference EV reduction gearbox variant (connects the default motor_nx motor
    to the default driveline_nx differential)."""
    return GearboxParams()
