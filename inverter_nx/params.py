"""Parametric inputs for the EV traction inverter ("sürücü" / motor drive).

Mirrors :mod:`motor_nx.params` / :mod:`driveline_nx.params`: plain dataclasses so one
:class:`InverterParams` fully describes a variant, variants load from / save to JSON,
and every numeric geometric dimension is pushed into Siemens NX as a named *expression*
so the built body stays editable.

Units: millimetres (mm) unless noted (electrical quantities carry their own unit in the
field name: _v, _kw, _nm, _arms, _khz, _us, _uf, _rpm). The inverter local frame stacks
along +Z (height axis): the cold plate is the base, the power modules sit on it, the DC-
link capacitor block beside them, all inside the HV enclosure box.

Defaults target the same 400 V passenger-EV traction class as the motor (167 kW / 440 Nm
peak): a single SiC-MOSFET power stage feeding the 6-pole IPMSM. engineering.validate()
flags impossible combinations before a build, and a default InverterParams() is sound.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Tuple


# --------------------------------------------------------------------------- #
# Sub-parameter groups
# --------------------------------------------------------------------------- #
@dataclass
class BusParams:
    """The high-voltage DC bus the inverter hangs off (battery side). `architecture`
    selects the pack class; min/max bound the operating + transient voltage window."""
    dc_voltage_v: float = 400.0        # nominal pack voltage at the DC link
    architecture: str = "400V"         # 400V | 800V
    min_voltage_v: float = 280.0       # low SOC / cold cut-off
    max_transient_v: float = 470.0     # worst-case regen / transient overshoot


@dataclass
class MotorInterfaceParams:
    """The motor the inverter drives -- numbers referred from motor_nx em_design (peak).
    These set the current/voltage/frequency the power stage must deliver."""
    peak_power_kw: float = 167.0
    peak_torque_nm: float = 440.0
    cont_power_kw: float = 77.0
    peak_phase_current_arms: float = 273.0     # peak phase current (rms)
    cont_phase_current_arms: float = 126.0     # continuous phase current (rms)
    base_speed_rpm: float = 3623.0
    max_speed_rpm: float = 18000.0
    pole_pairs: int = 3                        # 6-pole machine
    backemf_v_per_krpm_ll: float = 78.1        # back-EMF constant (LL rms per 1000 rpm)


@dataclass
class PowerStageParams:
    """The three-phase (6-switch) power bridge. `device` selects the semiconductor;
    `current_margin` is the design headroom of the switch rating over the peak motor
    current; modules can be paralleled to share current."""
    device: str = "SiC_MOSFET"          # SiC_MOSFET | Si_IGBT
    switch_voltage_class_v: float = 750.0   # blocking-voltage class of the device
    modules_parallel: int = 1           # devices paralleled per switch position
    switching_freq_khz: float = 12.0    # PWM carrier frequency
    deadtime_us: float = 0.5            # complementary-switch dead time
    current_margin: float = 1.3         # switch current rating / peak motor current
    modulation: str = "SVPWM"           # SVPWM | SPWM | DPWM


@dataclass
class DcLinkParams:
    """The DC-link capacitor bank that buffers the switching ripple between bus + bridge.
    Film caps dominate traction inverters (self-healing, high ripple rating).

    A DISCHARGE provision drains the stored energy after HV disconnect: a passive bleed
    resistor across the link (and/or an active-discharge path through the bridge). The
    bus must reach < 60 V within the regulatory time (~5 s); engineering.validate()
    checks the passive RC time constant against that limit and reports the required
    bleed resistance."""
    capacitance_uf: float = 500.0
    cap_technology: str = "film"        # film | ceramic | electrolytic
    ripple_current_margin: float = 1.2  # cap ripple-current rating / estimated ripple
    bleed_resistor_kohm: float = 4.7    # passive bleed across the link (0 => none fitted);
                                        # sized so the 500 uF link reaches < 60 V in < 5 s
                                        # (~4.5 s at 400 V) independent of active discharge
    active_discharge: bool = True       # active discharge path (bridge) in addition


@dataclass
class RegenParams:
    """Regenerative braking: the inverter pushes power back to the pack. Limited by the
    battery's charge-acceptance; blended braking coordinates friction + regen."""
    enabled: bool = True
    max_regen_power_kw: float = 70.0
    blended_braking: bool = True
    battery_charge_limit_kw: float = 70.0   # pack charge-acceptance ceiling


@dataclass
class ControlParams:
    """Motor-control + functional-safety configuration. FOC with MTPA below base speed
    and field weakening above it; a resolver gives rotor position with a sensorless
    fallback. STO (Safe Torque Off) + active short circuit are the ASIL safe states."""
    scheme: str = "FOC"                 # FOC | DTC | scalar
    mtpa: bool = True                   # max-torque-per-amp below base speed
    field_weakening: bool = True        # flux weakening above base speed
    sensorless_fallback: bool = True    # observer fallback if the sensor fails
    position_sensor: str = "resolver"   # resolver | encoder | hall
    current_loop_khz: float = 10.0      # current-control loop rate
    functional_safety: str = "ASIL-C"   # ISO 26262 integrity level
    sto: bool = True                    # Safe Torque Off
    active_short_circuit: bool = True   # ASC safe state at high speed
    comms: str = "CAN-FD"               # vehicle bus


@dataclass
class CoolingParams:
    """Liquid cold plate the power modules + DC link sit on (single coolant loop with
    the motor). Sized so the peak switching loss stays within the heat-flux budget.

    The inlet / outlet ports are bored radially through the -X end wall into the cold
    plate (G/SAE coolant fittings); `port_diameter_mm` is the bore (fitting thread
    minor), `port_pitch_mm` the inlet-to-outlet centre spacing along +Y."""
    type: str = "liquid_cold_plate"     # liquid_cold_plate | pin_fin | air
    coldplate_length_mm: float = 220.0
    coldplate_width_mm: float = 180.0
    coldplate_thickness_mm: float = 12.0
    coolant: str = "WEG 50/50"          # 50/50 water/ethylene-glycol
    port_diameter_mm: float = 8.0       # coolant inlet/outlet bore (0 => no ports; G1/8 ~ 8 mm)
    port_pitch_mm: float = 60.0         # inlet-to-outlet centre spacing (along +Y)


@dataclass
class EnclosureParams:
    """The HV enclosure (housing) -- a sealed box that carries the cold plate + bridge +
    DC link, with HV interlock and the 3-phase + DC connectors. The base (z=0) face is
    the MOUNTING-FACE DATUM the vehicle assembly sits on the motor (see blueprint).

    A peripheral LID FLANGE (a raised lip at the top of the wall) carries the lid bolt
    pattern: `lid_bolt_count` bolts on a rectangular pattern inset `lid_bolt_inset_mm`
    from the outer wall, drilled `lid_bolt_diameter_mm`."""
    length_mm: float = 260.0
    width_mm: float = 200.0
    height_mm: float = 90.0
    wall_mm: float = 6.0
    connector_count: int = 3            # 3-phase motor connectors
    hv_connector: bool = True           # HV DC inlet connector + interlock
    lv_connector: bool = True           # LV signal / control connector (CAN, gate, sensor)
    # lid bolt pattern on the top sealing flange
    lid_flange_mm: float = 8.0          # radial width of the raised top sealing lip (0 => none)
    lid_flange_thickness_mm: float = 6.0  # axial height of the raised lip above the wall top
    lid_bolt_count: int = 8             # lid bolts (even, distributed around the perimeter)
    lid_bolt_diameter_mm: float = 5.0   # lid bolt tapped-hole diameter (M5 ~ 4.2 minor)
    lid_bolt_inset_mm: float = 4.0      # bolt-circle inset from the outer wall edge


@dataclass
class BusbarParams:
    """Representative DC-bus + AC-phase busbars (laminated copper) tying the SiC modules
    to the DC-link cap and out to the phase connectors. A first-order packaging BLANK
    (the cap-to-module link), not a routed conductor model.

    The bars LAND on the DC-link cap's -Y terminal face: each bar bolts to a short
    terminal PAD that is united onto the cap (so it is one solid with the cap), and the
    bar's +Y edge meets that pad's outer face -- a touching mating contact, never a bar
    buried inside the cap body (`terminal_pad_proj_mm` is how far the pad stands proud of
    the cap face; the bar edge stops there)."""
    enabled: bool = True
    thickness_mm: float = 3.0           # copper bar thickness (stacked +/-)
    width_mm: float = 24.0              # bar width (current-carrying cross-section)
    height_mm: float = 8.0              # standoff height above the module tops
    terminal_pad_proj_mm: float = 3.0   # how far the cap terminal pad stands proud of the
                                        # cap -Y face (the bar +Y edge lands on it)
    terminal_pad_width_mm: float = 16.0  # terminal-pad extent in X (centred on the bar)


# --------------------------------------------------------------------------- #
# Top-level container
# --------------------------------------------------------------------------- #
@dataclass
class InverterParams:
    name: str = "EV_traction_inverter"
    bus: BusParams = field(default_factory=BusParams)
    motor: MotorInterfaceParams = field(default_factory=MotorInterfaceParams)
    power_stage: PowerStageParams = field(default_factory=PowerStageParams)
    dc_link: DcLinkParams = field(default_factory=DcLinkParams)
    regen: RegenParams = field(default_factory=RegenParams)
    control: ControlParams = field(default_factory=ControlParams)
    cooling: CoolingParams = field(default_factory=CoolingParams)
    enclosure: EnclosureParams = field(default_factory=EnclosureParams)
    busbar: BusbarParams = field(default_factory=BusbarParams)

    # -- serialisation (identical contract to motor_nx.params) ------------- #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "InverterParams":
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
    def from_json(cls, text: str) -> "InverterParams":
        return cls.from_dict(json.loads(text))

    def save_json(self, path: str, indent: int = 2) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json(indent=indent))

    @classmethod
    def load_json(cls, path: str) -> "InverterParams":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(fh.read())

    def overridden(self, **overrides: Any) -> "InverterParams":
        """Copy with dotted-path overrides, e.g.
        params.overridden(**{"power_stage.switching_freq_khz": 16.0, "bus.architecture": "800V"})."""
        data = self.to_dict()
        for dotted, value in overrides.items():
            target = data
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return InverterParams.from_dict(data)

    # -- NX expression table ---------------------------------------------- #
    def expressions(self) -> List[Tuple[str, float, str]]:
        """Flat (name, value, unit) list pushed into NX as named expressions so the
        generated body stays editable. Only the GEOMETRIC groups (cold plate, enclosure)
        carry mm dimensions; electrical params live in the blueprint metadata, not NX
        expressions. Names are NX-expression safe (group_field)."""
        out: List[Tuple[str, float, str]] = []
        for group_name in ("cooling", "enclosure", "busbar"):
            group = getattr(self, group_name)
            for f in fields(group):
                val = getattr(group, f.name)
                if isinstance(val, bool) or isinstance(val, str):
                    continue  # expressions carry numeric dimensions only
                if isinstance(val, int):
                    unit = ""   # dimensionless count
                else:
                    unit = "mm"
                out.append(("%s_%s" % (group_name, f.name), float(val), unit))
        return out


_GROUP_TYPES = {
    "bus": BusParams,
    "motor": MotorInterfaceParams,
    "power_stage": PowerStageParams,
    "dc_link": DcLinkParams,
    "regen": RegenParams,
    "control": ControlParams,
    "cooling": CoolingParams,
    "enclosure": EnclosureParams,
    "busbar": BusbarParams,
}


def _merge_group(group_cls, value: Dict[str, Any]):
    base = {f.name: getattr(group_cls(), f.name) for f in fields(group_cls)}
    base.update({k: v for k, v in value.items() if k in base})
    return group_cls(**base)


def default_params() -> InverterParams:
    """The reference EV traction inverter variant (drives the default motor_nx motor)."""
    return InverterParams()
