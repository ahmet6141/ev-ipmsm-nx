"""NX-independent tests for the first-order analysis module."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import analysis  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_loss_breakdown_components_positive_and_sum():
    lb = analysis.loss_breakdown(MotorParams())
    assert lb.p_cu_dc_w > 0 and lb.p_iron_w > 0
    assert lb.ac_resistance_factor >= 1.0
    assert abs(lb.p_total_w - (lb.p_cu_dc_w + lb.p_cu_ac_w + lb.p_iron_w + lb.p_magnet_w)) < 1e-6


def test_ac_factor_and_iron_rise_with_speed():
    p = MotorParams()
    lo = analysis.loss_breakdown(p, rpm=2000)
    hi = analysis.loss_breakdown(p, rpm=18000)
    assert hi.ac_resistance_factor > lo.ac_resistance_factor   # proximity grows with f
    assert hi.p_iron_w > lo.p_iron_w                           # iron loss grows with f


def test_magnet_segmentation_cuts_eddy_loss():
    p1 = MotorParams(); p1.material.magnet_segments_axial = 1
    p4 = MotorParams(); p4.material.magnet_segments_axial = 4
    e1 = analysis.loss_breakdown(p1, rpm=18000).p_magnet_w
    e4 = analysis.loss_breakdown(p4, rpm=18000).p_magnet_w
    assert e1 > 0 and e4 > 0
    assert abs(e1 / e4 - 16.0) < 0.5   # eddy ~ 1/n_seg^2 -> 4^2 = 16x


def test_thermal_rating_bounds_continuous():
    t = analysis.thermal_rating(MotorParams())
    assert 0.01 < (t.r_int_winding_to_iron_kpw + t.r_ext_iron_to_coolant_kpw) < 1.0
    assert 3.0 < t.j_cont_thermal_a_mm2 < 25.0   # plausible water-jacket continuous J
    assert t.torque_cont_thermal_nm > 0


def test_demag_margin_derates_with_temperature():
    dm = analysis.demag_margin(MotorParams())
    assert dm.br_hot_t < MotorParams().material.magnet_br_t      # hot Br lower than 20 C
    assert dm.hcj_hot_ka_m < MotorParams().material.magnet_hcj_ka_m
    assert dm.h_demag_peak_ka_m > 0 and dm.margin_ratio > 0


def test_rotor_stress_positive_and_sf():
    rs = analysis.rotor_stress(MotorParams())
    assert rs.bridge_stress_mpa > 0
    assert rs.overspeed_rpm == 1.2 * analysis.em_design.EMAssumptions().max_speed_rpm
    assert rs.safety_factor > 0


def test_cogging_index_lcm_gcd():
    cg = analysis.cogging_index(MotorParams())   # 54 slots / 6 poles
    assert cg.gcd == 6 and cg.lcm == 54
    assert cg.cogging_per_rev == 54


def test_report_runs():
    txt = analysis.analysis_report(MotorParams())
    assert "LOSSES" in txt and "THERMAL" in txt and "DEMAG" in txt and "ROTOR STRESS" in txt


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
