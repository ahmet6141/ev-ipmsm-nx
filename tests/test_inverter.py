"""Tests for inverter_nx (NX-independent layers): params round-trip, engineering
sizing + validation, and the geometry blueprint's structural integrity."""

import json

import pytest

from inverter_nx import blueprint as bp
from inverter_nx import engineering as eng
from inverter_nx.params import InverterParams


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = InverterParams()
    p2 = InverterParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = InverterParams.from_dict({"power_stage": {"switching_freq_khz": 16.0}})
    assert p.power_stage.switching_freq_khz == 16.0
    assert p.power_stage.device == InverterParams().power_stage.device  # untouched
    assert p.bus.dc_voltage_v == 400.0


def test_overridden_dotted():
    p = InverterParams().overridden(**{"bus.architecture": "800V",
                                       "power_stage.modules_parallel": 2})
    assert p.bus.architecture == "800V"
    assert p.power_stage.modules_parallel == 2


def test_expressions_are_numeric_and_unit_tagged():
    exprs = InverterParams().expressions()
    assert exprs, "expected geometric expressions"
    for name, val, unit in exprs:
        assert isinstance(val, float)
        assert unit in ("", "mm")


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_peak_amplitude_is_sqrt2_of_rms():
    import math
    p = InverterParams()
    g = eng.derive(p)
    assert g.peak_phase_current_amp_a == pytest.approx(
        p.motor.peak_phase_current_arms * math.sqrt(2.0), rel=1e-3)


def test_field_weakening_ratio_above_one():
    """This 6-pole machine spins to 18000 rpm: its back-EMF far exceeds the SVPWM
    ceiling, so field weakening is mandatory (ratio > 1)."""
    g = eng.derive(InverterParams())
    assert g.field_weakening_ratio > 1.0


def test_switching_freq_adequate_default():
    g = eng.derive(InverterParams())
    assert g.switching_freq_adequate
    assert g.max_electrical_freq_hz == pytest.approx(18000.0 / 60.0 * 3, rel=1e-6)


def test_regen_clamped_to_battery_limit():
    p = InverterParams().overridden(**{"regen.max_regen_power_kw": 120.0,
                                       "regen.battery_charge_limit_kw": 70.0})
    g = eng.derive(p)
    assert g.effective_regen_power_kw == pytest.approx(70.0)


def test_loss_and_heat_flux_positive():
    g = eng.derive(InverterParams())
    assert g.peak_loss_kw > 0.0
    assert g.coldplate_heat_flux_w_cm2 > 0.0


def test_default_design_is_buildable():
    assert eng.validate(InverterParams()) == []


def test_validate_catches_low_voltage_class():
    p = InverterParams().overridden(**{"power_stage.switch_voltage_class_v": 400.0})
    assert any("switch_voltage_class_v" in i for i in eng.validate(p))


def test_voltage_derate_default_ok():
    """The default 750 V SiC class clears Vdc/0.7 (571 V) and the 470 V transient."""
    g = eng.derive(InverterParams())
    assert g.voltage_derate_ok
    assert g.min_switch_voltage_class_v == pytest.approx(400.0 / 0.70, rel=1e-3)


def test_svpwm_ceiling_is_0707_vdc():
    """SVPWM (default) reaches 0.707*Vdc LL rms, not the 0.612*Vdc of plain SPWM."""
    import math
    g = eng.derive(InverterParams())
    assert g.inverter_max_ll_v == pytest.approx(400.0 / math.sqrt(2.0), rel=1e-3)


def test_regen_clamped_to_motor_capability():
    """Regen cannot exceed the motor's peak generating capability even if both the
    setting and the battery limit are raised above it."""
    p = InverterParams().overridden(**{"regen.max_regen_power_kw": 500.0,
                                       "regen.battery_charge_limit_kw": 500.0})
    g = eng.derive(p)
    assert g.effective_regen_power_kw == pytest.approx(p.motor.peak_power_kw)


def test_validate_catches_low_switching_freq():
    p = InverterParams().overridden(**{"power_stage.switching_freq_khz": 1.0})
    assert any("switching_freq" in i for i in eng.validate(p))


# --------------------------------------------------------------------------- #
# blueprint
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(InverterParams())
    assert blue["schema"] == "inverter_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "Z"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 8


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(InverterParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_six_power_modules_present():
    blue = bp.generate(InverterParams())
    mods = [s for s in blue["build_steps"] if s["role"] == "power_module"]
    assert len(mods) == 6


def test_dc_link_and_cold_plate_present():
    blue = bp.generate(InverterParams())
    roles = {s["role"] for s in blue["build_steps"]}
    assert "dc_link" in roles and "cold_plate" in roles
    assert "enclosure_cavity_cut" in roles


# --------------------------------------------------------------------------- #
# mounting-face datum (the interface the vehicle assembly sits on the motor)
# --------------------------------------------------------------------------- #
def test_mounting_face_datum_is_base_centre():
    """The mounting-face datum is the enclosure base centre at the local origin --
    every body grows in +Z from this face (ICD section 3)."""
    p = InverterParams()
    lay = eng.layout(p)
    assert lay.mounting_face_xyz == (0.0, 0.0, 0.0)
    blue = bp.generate(p)
    assert blue["mounting_face_xyz"] == [0.0, 0.0, 0.0]


def test_all_geometry_sits_at_or_above_the_mounting_face():
    """No body dips below z=0 (the mounting face); the inverter sits ENTIRELY on the
    +Z side of the datum so the assembly translation never buries it in the motor."""
    blue = bp.generate(InverterParams())
    for s in blue["build_steps"]:
        if s["boolean"] != "create":
            continue  # cuts (cavity, ports, bolts) may start a hair outside for a clean cut
        z0 = s.get("z0", 0.0)
        assert z0 >= -1e-9, "%s starts below the mounting face (z0=%.2f)" % (s["id"], z0)


def test_enclosure_encloses_the_internals_world_bbox():
    """The internals (cold plate, modules, DC-link cap, busbars) all lie inside the
    enclosure inner cavity footprint and below the lid (world bounding box check)."""
    p = InverterParams()
    blue = bp.generate(p)
    e = p.enclosure
    inner_hx = (e.length_mm - 2.0 * e.wall_mm) / 2.0
    inner_hy = (e.width_mm - 2.0 * e.wall_mm) / 2.0
    lid_z = e.height_mm - e.wall_mm
    internal_roles = {"cold_plate", "power_module", "dc_link", "busbar"}
    for s in blue["build_steps"]:
        if s["role"] not in internal_roles or s["boolean"] != "create":
            continue
        xs = [pt[0] for pt in s["profile"]]
        ys = [pt[1] for pt in s["profile"]]
        assert max(xs) <= inner_hx + 1e-6 and min(xs) >= -inner_hx - 1e-6, s["id"]
        assert max(ys) <= inner_hy + 1e-6 and min(ys) >= -inner_hy - 1e-6, s["id"]
        top_z = s.get("z0", 0.0) + s.get("length", 0.0)
        assert top_z <= lid_z + 1e-6, "%s top %.1f exceeds lid plane %.1f" % (s["id"], top_z, lid_z)


# --------------------------------------------------------------------------- #
# coolant ports on the cold plate
# --------------------------------------------------------------------------- #
def test_two_coolant_ports_present_and_axis_placed():
    blue = bp.generate(InverterParams())
    ports = [s for s in blue["build_steps"] if s["role"] == "coolant_port_cut"]
    assert len(ports) == 2
    for s in ports:
        assert s["kind"] == "hole" and s["boolean"] == "subtract"
        assert s["target"] == "cold_plate"
        # bored along +X into the -X end face of the plate
        assert tuple(s["axis"]) == (1.0, 0.0, 0.0)


def test_coolant_ports_clear_the_plate_and_split_in_y():
    """Inlet/outlet straddle the plate centre by +/- pitch/2 in Y, at the plate mid-
    thickness, and lie within the cold-plate width."""
    p = InverterParams()
    blue = bp.generate(p)
    c = p.cooling
    lay = eng.layout(p)
    z_mid = lay.floor_z + c.coldplate_thickness_mm / 2.0
    ports = sorted((s for s in blue["build_steps"] if s["role"] == "coolant_port_cut"),
                   key=lambda s: s["cy"])
    assert ports[0]["cy"] == pytest.approx(-c.port_pitch_mm / 2.0)
    assert ports[1]["cy"] == pytest.approx(+c.port_pitch_mm / 2.0)
    for s in ports:
        assert s["z0"] == pytest.approx(z_mid)
        assert abs(s["cy"]) + c.port_diameter_mm / 2.0 <= c.coldplate_width_mm / 2.0


def test_port_dia_exceeding_plate_thickness_is_caught():
    p = InverterParams().overridden(**{"cooling.port_diameter_mm": 20.0,
                                       "cooling.coldplate_thickness_mm": 12.0})
    assert any("coolant port" in i for i in eng.validate(p))


# --------------------------------------------------------------------------- #
# lid bolt pattern on the sealing flange
# --------------------------------------------------------------------------- #
def test_lid_bolt_pattern_count_and_axis():
    p = InverterParams()
    blue = bp.generate(p)
    bolts = [s for s in blue["build_steps"] if s["role"] == "lid_bolt_cut"]
    assert len(bolts) == p.enclosure.lid_bolt_count
    for s in bolts:
        assert s["kind"] == "hole" and s["boolean"] == "subtract"
        assert s["target"] == "enclosure"
        assert tuple(s["axis"]) == (0.0, 0.0, -1.0)   # drilled DOWN through the flange


def test_lid_bolts_lie_on_the_flange_centre_line():
    """Every lid bolt sits on the rectangular bolt line inset from the outer wall, i.e.
    on the raised sealing flange (not in the cavity, not off the part)."""
    p = InverterParams()
    e = p.enclosure
    blue = bp.generate(p)
    bx = e.length_mm / 2.0 - e.lid_bolt_inset_mm
    by = e.width_mm / 2.0 - e.lid_bolt_inset_mm
    for s in (s for s in blue["build_steps"] if s["role"] == "lid_bolt_cut"):
        on_x_edge = abs(abs(s["cx"]) - bx) < 1e-6
        on_y_edge = abs(abs(s["cy"]) - by) < 1e-6
        assert on_x_edge or on_y_edge, "bolt (%.1f,%.1f) off the flange line" % (s["cx"], s["cy"])
        assert abs(s["cx"]) <= bx + 1e-6 and abs(s["cy"]) <= by + 1e-6


def test_lid_bolts_inside_outer_footprint():
    p = InverterParams()
    e = p.enclosure
    for s in (s for s in bp.generate(p)["build_steps"] if s["role"] == "lid_bolt_cut"):
        assert abs(s["cx"]) <= e.length_mm / 2.0
        assert abs(s["cy"]) <= e.width_mm / 2.0


def test_lid_bolts_not_fitting_flange_is_caught():
    p = InverterParams().overridden(**{"enclosure.lid_flange_mm": 4.0,
                                       "enclosure.lid_bolt_inset_mm": 4.0,
                                       "enclosure.lid_bolt_diameter_mm": 6.0})
    assert any("lid bolts" in i for i in eng.validate(p))


# --------------------------------------------------------------------------- #
# busbars + LV connector
# --------------------------------------------------------------------------- #
def test_busbar_pair_present_above_modules():
    p = InverterParams()
    blue = bp.generate(p)
    lay = eng.layout(p)
    # the +/- pair are the two CREATE busbar bodies (the terminal pads are unite steps).
    bars = [s for s in blue["build_steps"] if s["role"] == "busbar" and s["boolean"] == "create"]
    assert len(bars) == 2
    module_top = lay.plate_top_z + lay.module_hgt
    for s in bars:
        assert s["z0"] >= module_top - 1e-6   # standoff above the module tops


def test_busbar_lands_on_cap_terminal_face_not_buried():
    """The fix for the inverter_nx interpenetration ERROR: the busbar +Y edge meets the
    DC-link cap terminal face (offset by the terminal pad it bolts to) -- it must NOT cross
    that face into the cap body. The bar also clears the power-module row on its -Y side."""
    p = InverterParams()
    lay = eng.layout(p)
    b = p.busbar
    bars = [s for s in bp.generate(p)["build_steps"]
            if s["role"] == "busbar" and s["boolean"] == "create"]
    pad_face_y = lay.cap_terminal_face_y - b.terminal_pad_proj_mm   # outer face of the pad
    module_edge_y = lay.module_y + lay.module_wid / 2.0
    for s in bars:
        ys = [pt[1] for pt in s["profile"]]
        bar_pos_edge, bar_neg_edge = max(ys), min(ys)
        # +Y edge lands ON the pad face (touching), never past it into the cap body
        assert bar_pos_edge <= lay.cap_terminal_face_y + 1e-6, (
            "%s +Y edge %.2f buries past the cap face %.2f" % (s["id"], bar_pos_edge, lay.cap_terminal_face_y))
        assert bar_pos_edge == pytest.approx(pad_face_y)  # meets the pad it bolts to
        # -Y edge clears the module row (a real gap, not an overlap)
        assert bar_neg_edge >= module_edge_y - 1e-6, (
            "%s -Y edge %.2f overlaps the module row %.2f" % (s["id"], bar_neg_edge, module_edge_y))


def test_busbar_does_not_interpenetrate_the_cap_solid():
    """Structural assert via the NX-free clearance engine: NO sampled surface point of a
    busbar lies inside the DC-link CAP body (the terminal pads are united into the cap and
    are part of that solid -- the bar may only TOUCH that combined solid's face, never bury
    in it). This is the exact relationship nx_inspect flagged as the ERROR."""
    from vehicle_nx import clearance as cl
    blue = bp.generate(InverterParams())
    R, o = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0]

    def solids_named(name):
        out = []
        for s in blue["build_steps"]:
            if s.get("body_name") == name:
                wb = cl._world_body(s, R, o)
                if wb is not None:
                    out.append(wb)
        return out

    cap_only = solids_named("DC_Link_Capacitor")
    assert cap_only, "expected a DC_Link_Capacitor body"
    for bar in ("DC_Busbar_Pos", "DC_Busbar_Neg"):
        bar_solids = solids_named(bar)
        assert bar_solids, "expected a %s body" % bar
        # the bar must NOT bury into the cap body (a touch at the face is fine, but the
        # bar's broad side must never cross the cap face -> no point inside the cap solid).
        clash = cl.solids_interpenetrate(bar_solids, cap_only)
        assert clash is None, "%s interpenetrates the DC-link cap (depth %s)" % (bar, clash)


def test_lv_connector_present_and_separated_from_hv():
    blue = bp.generate(InverterParams())
    conns = {s["id"]: s for s in blue["build_steps"] if s["role"] == "connector"}
    assert "lv_connector" in conns and "hv_connector" in conns
    # HV/LV separation: opposite sides of the -X end wall in Y
    assert conns["lv_connector"]["cy"] * conns["hv_connector"]["cy"] < 0.0


# --------------------------------------------------------------------------- #
# engineering extras (continuous current/flux, cap energy, packaging, mass)
# --------------------------------------------------------------------------- #
def test_continuous_dc_current_from_power_and_voltage():
    p = InverterParams()
    g = eng.derive(p)
    assert g.cont_dc_current_a == pytest.approx(
        p.motor.cont_power_kw * 1000.0 / p.bus.dc_voltage_v, rel=1e-3)
    assert g.peak_dc_current_a > g.cont_dc_current_a


def test_continuous_flux_below_peak_flux():
    g = eng.derive(InverterParams())
    assert 0.0 < g.cont_heat_flux_w_cm2 < g.coldplate_heat_flux_w_cm2


def test_dc_link_energy_half_c_v_squared():
    p = InverterParams()
    g = eng.derive(p)
    expected = 0.5 * (p.dc_link.capacitance_uf * 1e-6) * p.bus.dc_voltage_v ** 2
    assert g.dc_link_energy_j == pytest.approx(expected, rel=1e-3)


def test_packaging_volume_and_mass_positive():
    p = InverterParams()
    g = eng.derive(p)
    expected_vol_l = (p.enclosure.length_mm * p.enclosure.width_mm
                      * p.enclosure.height_mm) * 1e-6
    assert g.packaging_volume_l == pytest.approx(expected_vol_l, rel=1e-3)
    assert g.estimated_mass_kg > 0.0
    assert g.power_density_kw_per_l > 0.0


def test_validate_catches_module_row_overhang():
    """A tiny cold plate cannot host the fixed-size module row -> caught."""
    p = InverterParams().overridden(**{"cooling.coldplate_length_mm": 60.0,
                                       "cooling.coldplate_width_mm": 60.0,
                                       "enclosure.length_mm": 90.0,
                                       "enclosure.width_mm": 90.0})
    assert any("overhang" in i for i in eng.validate(p))


def test_validate_catches_internals_too_tall():
    """A shallow enclosure cannot close its lid over the cold plate + cap stack."""
    p = InverterParams().overridden(**{"enclosure.height_mm": 40.0})
    assert any("internals top" in i for i in eng.validate(p))
