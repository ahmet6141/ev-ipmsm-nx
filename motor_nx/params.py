"""Parametric inputs for the EV traction IPMSM.

Everything here is a plain dataclass so that:
  * a single :class:`MotorParams` instance fully describes one motor variant,
  * variants can be created/overridden in code or loaded from JSON (parameter
    sweeps for the headless batch driver), and
  * the values can be pushed into Siemens NX as named *expressions* so the
    resulting body stays editable inside NX (see :meth:`MotorParams.expressions`).

Units: millimetres (mm) and degrees (deg) unless noted. The motor axis is +Z.
The active stack spans z = 0 .. stack_length.

NOTE: the default numbers target a passenger-EV rear-drive-unit class IPMSM
(~Tesla Model 3 RDU scale). They are reconciled with the research workflow's
verified defaults in DESIGN.md; tune freely -- em_design.validate() will flag
geometrically impossible combinations before a build is attempted.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class StatorParams:
    outer_diameter: float = 225.0       # OD of the stator lamination (Model 3 RDU class)
    bore_diameter: float = 161.0        # inner bore (faces the air gap); split ratio ~0.715
    slot_count: int = 54                # Q -- 54/6 hairpin distributed winding (q=3)
    tooth_width: float = 5.6            # min metal tooth width (at slot-body inner radius)
    slot_opening_width: float = 1.9     # tangential width of the semi-closed slot mouth
    slot_opening_depth: float = 1.0     # radial depth of the slot mouth / tang
    back_iron_thickness: float = 13.0   # stator yoke radial thickness (OD -> slot bottom)
    slot_bottom_fillet: float = 1.0     # fillet radius at the slot bottom corners
    lamination_thickness: float = 0.27  # electrical-steel sheet thickness (M250-27 class)


@dataclass
class WindingParams:
    phases: int = 3
    conductors_per_slot: int = 8        # hairpin bars stacked radially in each slot
    parallel_paths: int = 2
    bar_clearance: float = 0.45         # gap (slot wall <-> bar): insulation + tolerance
    bar_corner_radius: float = 0.8      # rounded corner of a rectangular hairpin bar
    model_endwindings: bool = True      # add a simplified end-winding ENVELOPE ring at each stack end
    end_winding_height: float = 22.0    # axial extension of the end-winding envelope beyond each end (mm)
    end_winding_style: str = "envelope"  # "envelope" (toroidal ring) | "hairpin" (per-slot crown arcs, experimental)


@dataclass
class RotorParams:
    air_gap: float = 0.7                 # radial mechanical air gap (EV traction 0.5-0.8)
    pole_count: int = 6                  # 2p -- 6 poles (3 pole pairs), Model 3 RDU
    magnets_per_pole: int = 2            # "2" => single V per pole
    magnet_width: float = 26.0           # Lm -- leg length of one magnet block
    magnet_thickness: float = 4.5        # t  -- short (magnetisation) dimension
    v_angle_deg: float = 145.0           # included opening angle of the V (between the two legs)
    center_post_halfwidth: float = 1.0   # half-width of the d-axis rib (center rib ~2.0 mm)
    outer_bridge: float = 1.0            # steel bridge between magnet pocket and rotor surface
    end_barrier: float = 1.5             # flux-barrier air length added at each magnet end
    pocket_clearance: float = 0.15       # pocket-vs-magnet clearance (glue/tolerance)
    magnet_pocket_fillet: float = 0.5    # corner radius on the pocket / flux-barrier (rotor stress relief)
    vertex_gap: float = 38.0             # radial gap from shaft surface to the V apex (inner ends)
    lightening_holes: int = 0            # optional circular lightening/cooling holes (0 = none)
    lightening_hole_diameter: float = 10.0
    lightening_hole_pitch_radius: float = 45.0


@dataclass
class ShaftParams:
    diameter: float = 45.0              # main journal diameter (rotor bore = this)
    bore_diameter: float = 14.0         # hollow-shaft inner bore (0 => solid shaft); oil-fed
    overhang: float = 35.0              # shaft length added beyond the stack at EACH end
    bearing_seat_diameter: float = 40.0
    bearing_seat_length: float = 22.0


@dataclass
class CoolingParams:
    jacket_thickness: float = 6.0       # radial thickness of the water jacket sleeve
    housing_gap: float = 0.5            # shrink-fit gap stator OD <-> jacket bore
    channel_type: str = "axial"         # "axial" | "spiral" | "none"
    channel_count: int = 12             # number of axial channels (channel_type == "axial")
    channel_diameter: float = 4.0       # MUST stay below jacket_thickness with wall margin: a Ø=jacket channel is tangent to both jacket faces (zero-wall, NX subtract fails). Ø4 in a 6 mm jacket -> 1 mm walls.
    spiral_pitch: float = 18.0          # axial advance per turn (channel_type == "spiral")
    spiral_channel_width: float = 6.0
    spiral_channel_depth: float = 5.0


@dataclass
class MaterialParams:
    """Production material specification. Metadata only -- does not change the
    generated geometry, but is the data a manufacturing BOM / FEA needs."""
    magnet_grade: str = "N42SH"               # sintered NdFeB; 'SH' = 150 C max service
    magnet_br_t: float = 1.28                 # remanence Br @ 20 C
    magnet_hcj_ka_m: float = 1592.0           # intrinsic coercivity Hcj (>= 20 kOe)
    magnet_max_service_c: float = 150.0       # max continuous magnet temperature
    magnet_mu_recoil: float = 1.05            # recoil permeability (2nd-quadrant line slope)
    magnet_segments_axial: int = 4            # axial magnet segments (eddy-loss control); 1 = solid block
    magnet_seg_gap_mm: float = 0.1            # insulation/adhesive gap between axial magnet segments
    electrical_steel: str = "0.27 mm NO silicon steel (M250-27 class)"
    stacking_factor: float = 0.96             # iron fill of the lamination stack
    copper_insulation_class: str = "H (180 C) hairpin enamel"
    housing_material: str = "cast aluminium (water jacket)"


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class MotorParams:
    name: str = "EV_IPMSM_traction"
    stack_length: float = 134.0         # active axial length of the laminations (Model 3 class)
    stator: StatorParams = field(default_factory=StatorParams)
    winding: WindingParams = field(default_factory=WindingParams)
    rotor: RotorParams = field(default_factory=RotorParams)
    shaft: ShaftParams = field(default_factory=ShaftParams)
    cooling: CoolingParams = field(default_factory=CoolingParams)
    material: MaterialParams = field(default_factory=MaterialParams)

    # -- serialisation ----------------------------------------------------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorParams":
        """Build from a (possibly partial) nested dict; missing keys keep defaults."""
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
    def from_json(cls, text: str) -> "MotorParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "MotorParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "MotorParams":
        """Return a copy with dotted-path overrides applied, e.g.

            params.overridden(**{"rotor.magnet_width": 21.0, "stack_length": 150})

        Handy for parameter sweeps in the batch driver.
        """
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return MotorParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so
        the generated body remains editable. Names are NX-expression safe
        (``group_field``)."""
        out: List[Tuple[str, float, str]] = [("stack_length", self.stack_length, "mm")]
        for group_name in ("stator", "winding", "rotor", "shaft", "cooling"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                # every dimension is a float (-> mm, or deg for *_deg); every
                # count is an int (-> dimensionless). bools were skipped above.
                if f.name.endswith("_deg"):
                    unit = "deg"
                elif isinstance(val, int):
                    unit = ""    # dimensionless count (poles, slots, paths, ...)
                else:
                    unit = "mm"
                out.append((f"{group_name}_{f.name}", float(val), unit))
        return out


_GROUP_TYPES = {
    "stator": StatorParams,
    "winding": WindingParams,
    "rotor": RotorParams,
    "shaft": ShaftParams,
    "cooling": CoolingParams,
    "material": MaterialParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> MotorParams:
    """The reference EV traction IPMSM variant."""
    return MotorParams()
