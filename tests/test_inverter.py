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
    assert any("headroom" in i for i in eng.validate(p))


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
