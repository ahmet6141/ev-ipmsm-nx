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
    # profile-shift (addendum-modification) coefficients x. A positive shift on the
    # pinion avoids undercut on low tooth counts and balances the two centre-distance-
    # preserving shifts; the built tooth outline + ISO 6336 rating both use them.
    pinion_profile_shift: float = 0.0
    gear_profile_shift: float = 0.0


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
    # The layshaft carries the stage-1 GEAR + the stage-2 PINION keyed/pressed onto it: a
    # gear blank and the shaft it sits on are TWO solids that would interpenetrate if both
    # were modelled as overlapping discs (ICD §7.6 forbids it). cluster_gears UNITES the
    # two layshaft-mounted blanks INTO the shaft as one rotating cluster body (they turn
    # together) -- the physically-correct, overlap-free representation. Set False to bore
    # each blank to the shaft OD (a separate press-fit ring) instead.
    cluster_gears: bool = True


@dataclass
class BearingParams:
    """One representative rolling bearing seated at a shaft journal, modelled as
    concentric race rings (ICD §7.6: press fits, no shared solid). The bore seats on
    the shaft OD (inner-race bore = shaft OD) and the OD seats in the housing bearing
    bore; a thin rolling-element ring sits between the races. The load ratings drive the
    ISO 281 L10 life: a deep-groove BALL bearing has exponent p = 3, a tapered/cylindrical
    ROLLER bearing p = 10/3. Default sizes are catalogue values for the relevant bore.

    `bore_d_mm` / `od_d_mm` / `width_mm` are the boundary dimensions (d, D, B). The
    rolling-element ring is a representative torus-band of `ball_ring_thickness_mm`
    centred on the pitch diameter (d+D)/2. `dynamic_load_rating_c_n` is the basic
    dynamic load rating C (N) from the catalogue; `static_load_rating_c0_n` is C0.
    """
    name: str = "6008"                      # catalogue designation (info)
    kind: str = "ball"                      # "ball" (p=3) or "roller" (p=10/3)
    bore_d_mm: float = 40.0                 # inner-race bore d = shaft OD (press fit)
    od_d_mm: float = 68.0                   # outer-race OD D = housing bearing bore
    width_mm: float = 15.0                  # bearing width B (axial)
    dynamic_load_rating_c_n: float = 30700.0   # basic dynamic load rating C (N)
    static_load_rating_c0_n: float = 19000.0   # basic static load rating C0 (N)
    ball_ring_thickness_mm: float = 6.0     # representative rolling-element ring radial thickness


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
    # Each shaft passes THROUGH the cast end covers via a bearing bore: a clearance hole
    # subtracted where the shaft crosses the cover wall, so the shaft sits in the bore and
    # never shares solid with the housing (ICD §7.6). Radial clearance added to the shaft
    # radius for that bore (the bearing-seat / oil-seal register stand-off).
    bearing_bore_clearance_mm: float = 3.0

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
    # output GEAR hub keyway (DIN 6885-A) that keys the toothed gear to the diff input
    # shaft in its press-fit bore -- the torque connection.
    keyway_width_mm: float = 10.0        # parallel key width (b) for a ~32 mm bore
    keyway_depth_mm: float = 3.3         # keyway depth into the BORE wall (t2)


@dataclass
class TorqueShaftKeyParams:
    """A DIN 6885-A parallel keyway cut into a gear's press-fit BORE (the torque path
    from an external shaft into the toothed gear hub). Width/depth sized to the bore."""
    width_mm: float = 12.0               # key width b
    depth_mm: float = 3.8                # keyway depth into the bore wall (t2)


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
    # ISO 6336 / ISO 281 rating torque: an EV is rated on its CONTINUOUS (thermal) duty,
    # with peak torque only intermittent. The equivalent continuous torque for a passenger
    # EV final drive (load-spectrum / Miner-equivalent) is ~a third of peak. The gear
    # strength + bearing life are screened at motor_peak_torque_nm * this fraction; the
    # geometry (centre distances, gear sizes) is unchanged.
    continuous_torque_fraction: float = 0.33
    # the continuous speed the bearing L10h is integrated at (the duty-equivalent input
    # speed, well below the 18 000 rpm peak); EV cruise sits far below max rpm.
    continuous_speed_rpm: float = 9000.0
    # The layshaft sits between the motor and the diff. `layshaft_collinear` places the
    # three axes (diff -> layshaft -> motor) on one line so centre_distance_1 +
    # centre_distance_2 IS exactly the motor<->diff offset (the inline reduction layout
    # that guarantees the train reaches). The motor direction from the diff axis (toward
    # the vehicle centre-plane + up, ICD §7.1) is set by motor_dir_angle_deg.
    layshaft_collinear: bool = True
    motor_dir_angle_deg: float = 48.5     # direction of the motor axis from the diff axis in
    #                                       local XY (~atan2(dz,dx) for dx~155, dz~175 -> clears)
    # The motor pinion is keyed/pressed onto the MOTOR ROTOR SHAFT (from motor_nx, Ø45),
    # so its blank is a TUBE bored to that shaft OD -- a press-fit ring, not a solid disc
    # that would interpenetrate the shaft (ICD §7.6). 0 => solid (no external shaft).
    motor_pinion_bore_diameter_mm: float = 45.0
    # DIN 6885 keyway keying the motor pinion to the motor rotor shaft in its bore.
    motor_pinion_key: TorqueShaftKeyParams = field(default_factory=TorqueShaftKeyParams)
    stage1: GearStageParams = field(default_factory=lambda: GearStageParams(
        module_mm=2.5, pinion_teeth=19, gear_teeth=53, face_width_mm=32.0))
    stage2: GearStageParams = field(default_factory=lambda: GearStageParams(
        module_mm=3.5, pinion_teeth=19, gear_teeth=64, face_width_mm=44.0))
    layshaft: LayshaftParams = field(default_factory=LayshaftParams)
    housing: HousingParams = field(default_factory=HousingParams)
    output: OutputCouplingParams = field(default_factory=OutputCouplingParams)
    material: MaterialParams = field(default_factory=MaterialParams)
    # Rolling bearings at the journalled shafts (ISO 281 life). Each bearing's BORE = the
    # journal it presses onto (press fit, no shared solid): the LAYSHAFT (the only shaft
    # fully internal to the gearbox) on its Ø35 bearing seat -- two bearings in the end
    # covers; the OUTPUT shaft on its Ø32 journal -- one bearing in the +Z cover OUTBOARD of
    # the output coupling. The layshaft is the highest-loaded, fastest journal, so it runs a
    # CYLINDRICAL ROLLER bearing (NJ207, p = 10/3) for the L10h life; the output runs a
    # deep-groove ball. (The motor-pinion shaft has NO gearbox bearing: the pinion is
    # integral with the motor ROTOR, journalled by the motor's OWN DE/NDE bearings.)
    layshaft_bearing: BearingParams = field(default_factory=lambda: BearingParams(
        name="NJ207", kind="roller", bore_d_mm=35.0, od_d_mm=72.0, width_mm=17.0,
        dynamic_load_rating_c_n=35500.0, static_load_rating_c0_n=29000.0))
    output_shaft_bearing: BearingParams = field(default_factory=lambda: BearingParams(
        name="6006", kind="ball", bore_d_mm=32.0, od_d_mm=55.0, width_mm=13.0,
        dynamic_load_rating_c_n=20300.0, static_load_rating_c0_n=11200.0))

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
            ("motor_pinion_bore_diameter_mm", float(self.motor_pinion_bore_diameter_mm), "mm"),
        ]
        for group_name in ("stage1", "stage2", "layshaft", "housing", "output",
                            "motor_pinion_key", "layshaft_bearing", "output_shaft_bearing"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif (f.name.endswith("_fraction") or f.name.endswith("_teeth")
                      or f.name.endswith("_shift") or f.name.endswith("_n")):
                    unit = ""   # ratio / count / load rating in N (not a length)
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
    "motor_pinion_key": TorqueShaftKeyParams,
    "layshaft_bearing": BearingParams,
    "output_shaft_bearing": BearingParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> GearboxParams:
    """The reference EV reduction gearbox variant (connects the default motor_nx motor
    to the default driveline_nx differential)."""
    return GearboxParams()
