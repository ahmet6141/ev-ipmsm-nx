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

    # AXLE NOTCH / relief: at each axle x-station the suspension control arms, the toe
    # link, the anti-roll link/damper AND the driveline half-shaft all sweep through the
    # rail's Y band as they run from the inboard pickups out to the wheel hub. Measured
    # against the actual swept envelope, those members cross the rail over X = axle ±~145
    # and reach the FULL rail height (z up to the rail top ~390) and into the abutting
    # crush-can overhang. So the relief is a FULL-SECTION clearance WINDOW centred on the
    # axle x-station, relieving the rail (and the crush-can end) all the way through its
    # height over that X band -- the rail is structurally CONTINUOUS through the battery
    # tray + crossmembers and the subframe pads sit just inboard of the window; the open
    # axle bay is where the corner + half-shaft pass. (Resolves the assembled-vehicle
    # rail/arm + rail/half-shaft + crush-can/arm collisions the NX inspection found.)
    #
    # ``axle_notch_half_width_mm`` is the half-X-extent of the window each side of the
    # axle station; ``axle_notch_top_mm`` is the world Z the relief reaches (>= the rail
    # top so the FULL section is relieved -- the swept arms/anti-roll reach the rail top).
    axle_notch: bool = True
    axle_notch_half_width_mm: float = 165.0   # half X-extent of the relief window each side of the axle
    axle_notch_top_mm: float = 400.0          # sanity ceiling (the band is now the rail interior, see below)
    # The relief is a MIDDLE-BAND window: it leaves a `flange`-thick TOP and BOTTOM flange
    # on the rail so the rail stays ONE continuous body through the axle station (a full-
    # height cut severs it, orphaning the outboard subframe-mount pad). The half-shaft
    # runs at the hub height (mid-rail) so the band clears it; the arms cross clear above /
    # below the retained flanges. Keep < ~33 mm so the band still clears the half-shaft.
    axle_notch_flange_mm: float = 25.0
    # the retained flange is only the INBOARD-top corner of the rail width (the suspension
    # upright/arm envelope hugs the rail's OUTBOARD edge, so the outboard-top is cut away
    # too). `axle_notch_keep_width_mm` is how much of the rail width (from the inner face)
    # the top flange keeps. The corridor it leaves -- inboard-top, above the half-shaft and
    # below the upper arm, inboard of the upright -- is the clear bridge that keeps the
    # rail one body across the axle relief.
    axle_notch_keep_width_mm: float = 50.0


@dataclass
class BatteryTrayParams:
    """The sealed structural battery tray that drops between the rails (the floor
    of the skateboard). Modelled as a hollow box with a few lateral crossbraces.
    `sealed` flags an IP-rated enclosure (a lid is fitted; not modelled here).

    The pack energy is audited against the tray's INTERNAL cavity volume: the gross
    cavity must hold `energy_kwh` at `pack_density_wh_per_l` once the `usable_fraction`
    (walls, cold plate, module housings, gaps) is removed. With the default 130 mm
    internal height the 75 kWh pack sits at a realistic ~245 Wh/L gross
    (~327 Wh/L on the usable cavity)."""
    enabled: bool = True
    length_mm: float = 2400.0
    # the tray drops INTO the inner channel between the rails, so its width must be
    # < frame_inner_width (1100) with a side clearance for the seal / mounting flange.
    width_mm: float = 1040.0
    # internal height raised to 130 mm so prismatic cells + a cooling plate + the
    # module enclosure fit (110 mm was the binding constraint that forced an
    # optimistic ~273 Wh/L gross for 75 kWh -- the adversarial-review MEDIUM finding).
    height_mm: float = 130.0
    wall_mm: float = 4.0
    crossbrace_count: int = 6
    # side clearance between the tray wall and each rail inner face (seal/flange gap)
    side_clearance_mm: float = 20.0
    sealed: bool = True
    # pack-energy audit (does not change geometry; checked in engineering.validate())
    energy_kwh: float = 75.0              # target usable pack energy
    pack_density_wh_per_l: float = 250.0  # assumed pack-level volumetric density (gross)
    usable_fraction: float = 0.75         # cavity fraction that is active cell volume


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
    # Each subframe bolts up to the rails through FOUR pads per axle (fore + aft x
    # left + right), straddling the axle relief window. `pad_reach_mm` is the fore/aft
    # offset of each pad from the axle x-station and MUST equal subframe_nx
    # PadParams.pad_x_local_mm so the chassis bolt pattern coincides with the subframe
    # flange (a coincidence test enforces it). The outboard pad sits past the axle, so
    # the rails extend `mount_zone` (pad_reach + a bolt-circle margin) beyond each axle
    # to carry it on the solid MAIN rail (not the sacrificial crush can); the crush can
    # then butts onto the extended rail end.
    pad_reach_mm: float = 175.0
    mount_pad_margin_mm: float = 55.0       # solid rail kept outboard of the outboard pad bolts
    # bolt-flange OD the subframe presents to each pad (= subframe_nx PadParams
    # .flange_diameter_mm). The chassis drills its pad bolt circle on the SAME PCD the
    # subframe derives from this (PCD_r = max(bolt_d, flange_d/2 - max(bolt_d, 6))), so
    # the two bolt patterns coincide hole-for-hole.
    mount_flange_d_mm: float = 72.0


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
class LoadsParams:
    """Static load-case + mass-distribution assumptions for the first-order bench
    checks (bending stress / deflection, vehicle CG). These DRIVE the engineering
    derive() so the load tracks the model instead of a hardcoded literal (the
    adversarial-review MEDIUM finding)."""
    # sprung mass reacted by the rails over the wheelbase. <=0 => derive it from the
    # battery-pack mass (energy/density) + the body/occupant allowance below.
    static_payload_kg: float = 0.0
    # mass + height of the body-in-white + occupants share, sitting well above the
    # low battery floor -- used for the mass-weighted vehicle CG (not the battery CG).
    body_occupant_mass_kg: float = 700.0
    body_occupant_cg_height_mm: float = 650.0
    # gravimetric pack density used to estimate the battery mass from energy_kwh.
    pack_gravimetric_wh_per_kg: float = 160.0


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
    loads: LoadsParams = field(default_factory=LoadsParams)
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
        # battery-tray pack-energy fields are an engineering audit, NOT geometry, so
        # they must not be emitted as mm NX expressions.
        _NON_GEOMETRIC = {"energy_kwh", "pack_density_wh_per_l", "usable_fraction"}
        out: List[Tuple[str, float, str]] = []
        for group_name in ("frame", "battery_tray", "subframe", "body_mount"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if f.name in _NON_GEOMETRIC:
                    continue  # not a geometric dimension
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
    "loads": LoadsParams,
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
