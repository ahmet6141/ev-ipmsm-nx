"""NX-independent tests for em_design performance + the hardened validate() checks."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import em_design  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_performance_sanity_bounds():
    e = em_design.estimate_performance(MotorParams())
    assert 0 < e.torque_cont_nm < e.torque_peak_nm
    assert e.i_phase_peak_a > e.i_phase_cont_a > 0
    assert 0 < e.efficiency_cont_pct <= 100
    assert 1500 < e.base_speed_rpm < 8000      # Model-3-class first-order band


def test_series_turns_closed_form():
    p = MotorParams()
    e = em_design.estimate_performance(p)
    expected = p.stator.slot_count * p.winding.conductors_per_slot / (
        2.0 * p.winding.phases * p.winding.parallel_paths)
    assert abs(e.series_turns_per_phase - expected) < 1e-9


def test_voltage_ceiling_is_svpwm():
    # base speed must use Vdc/sqrt(6) (SVPWM), i.e. back-EMF LL rms at base < Vdc.
    p = MotorParams()
    a = em_design.EMAssumptions()
    e = em_design.estimate_performance(p, a)
    e_ll_at_base = e.ke_ll_v_per_krpm * e.base_speed_rpm / 1000.0
    assert e_ll_at_base < a.dc_bus_v          # real inverters cannot reach full DC bus LL
    assert e_ll_at_base < a.dc_bus_v / math.sqrt(2) + 1.0   # ~SVPWM LL ceiling


def test_chorded_winding_drops_kp():
    p = MotorParams(); p.winding.coil_span_slots = 8   # full pitch = 9 for 54/6
    g = em_design.derive(p)
    assert g.pitch_factor < 1.0
    assert em_design.derive(MotorParams()).pitch_factor == 1.0


def test_validate_default_clean():
    assert em_design.validate(MotorParams()) == []


def test_validate_catches_center_rib_overlap():
    p = MotorParams(); p.rotor.center_post_halfwidth = 0.01; p.rotor.pocket_clearance = 2.0
    assert any("rib" in m.lower() for m in em_design.validate(p))


def test_validate_catches_zero_conductors_and_paths():
    p = MotorParams(); p.winding.conductors_per_slot = 0
    assert any("conductors_per_slot" in m for m in em_design.validate(p))
    p = MotorParams(); p.winding.parallel_paths = 0
    assert any("parallel_paths" in m for m in em_design.validate(p))


def test_validate_catches_shaft_bore_too_big():
    p = MotorParams(); p.shaft.bore_diameter = 50.0
    assert any("bore_diameter" in m for m in em_design.validate(p))


def test_validate_catches_zero_jacket():
    p = MotorParams(); p.cooling.jacket_thickness = 0.0
    assert any("jacket_thickness" in m for m in em_design.validate(p))


# --- airgap flux density derived from Br (refined model) ------------------- #
def test_airgap_flux_density_derived_in_band():
    # default assumptions DERIVE Bg from Br; result must be physical and reproduce
    # the ~0.85 T that was previously hand-assumed for this reference design.
    _flat, bg1 = em_design.airgap_flux_density(MotorParams())
    assert 0.60 < bg1 < 1.00
    assert abs(bg1 - 0.85) < 0.08          # calibrated to the legacy reference value
    e = em_design.estimate_performance(MotorParams())
    assert abs(e.airgap_flux_density_t - bg1) < 1e-9


def test_bg_scales_with_remanence():
    weak = MotorParams(); weak.material.magnet_br_t = 1.10
    strong = MotorParams(); strong.material.magnet_br_t = 1.40
    assert (em_design.airgap_flux_density(strong)[1]
            > em_design.airgap_flux_density(weak)[1])


def test_bg_scales_with_magnet_width():
    thin = MotorParams(); thin.rotor.magnet_width = 20.0
    wide = MotorParams(); wide.rotor.magnet_width = 30.0
    assert em_design.airgap_flux_density(wide)[1] > em_design.airgap_flux_density(thin)[1]


def test_bg_override_is_respected():
    a = em_design.EMAssumptions(b_g1_peak_t=0.90)
    e = em_design.estimate_performance(MotorParams(), a)
    assert abs(e.airgap_flux_density_t - 0.90) < 1e-9


def test_carter_factor_ge_one():
    kc = em_design.carter_factor(MotorParams())
    assert 1.0 <= kc < 1.5                  # slot opening slightly widens the effective gap


def test_stacking_factor_reduces_torque():
    full = MotorParams(); full.material.stacking_factor = 1.00
    real = MotorParams(); real.material.stacking_factor = 0.96
    tf = em_design.estimate_performance(full).torque_peak_nm
    tr = em_design.estimate_performance(real).torque_peak_nm
    assert tr < tf
    assert abs(tr / tf - 0.96) < 1e-3       # torque scales with effective stack


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn(); print("PASS ", fn.__name__)
        except Exception:
            failed += 1; print("FAIL ", fn.__name__); traceback.print_exc()
    print("\n%d/%d passed" % (len(fns) - failed, len(fns)))
    sys.exit(1 if failed else 0)
