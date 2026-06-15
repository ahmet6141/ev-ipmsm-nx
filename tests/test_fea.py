"""NX-independent tests for the FEA hand-off (winding map, spec, DXF)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import fea  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_winding_layout_balanced():
    p = MotorParams()
    layout = fea.winding_layout(p)
    assert len(layout) == p.stator.slot_count
    phases = [ph for ph, _ in layout]
    for ph in ("A", "B", "C"):
        assert phases.count(ph) == p.stator.slot_count // 3      # equal slots per phase
    # net sign per phase is zero (balanced +/- belts)
    for ph in ("A", "B", "C"):
        net = sum(sg for p2, sg in layout if p2 == ph)
        assert net == 0


def test_fea_spec_keys_and_consistency():
    p = MotorParams()
    spec = fea.fea_spec(p)
    for key in ("geometry_mm", "symmetry_and_bc", "materials", "winding",
                "excitation", "operating_points", "mesh", "analyses", "acceptance_targets"):
        assert key in spec
    # Hcb is the derived recoil value (not the old hardcoded 915)
    hcb = spec["materials"]["magnet"]["Hcb_kA_per_m"]
    expected = p.material.magnet_br_t / (4e-7 * math.pi * p.material.magnet_mu_recoil) / 1000.0
    assert abs(hcb - expected) < 1.0
    # winding series turns match the performance model
    from motor_nx import em_design
    assert abs(spec["winding"]["series_turns_per_phase"]
               - round(em_design.estimate_performance(p).series_turns_per_phase)) < 1


def test_slot_phase_map_matches_layout():
    p = MotorParams()
    spec = fea.fea_spec(p)
    layout = fea.winding_layout(p)
    assert len(spec["winding"]["slot_phase_map"]) == p.stator.slot_count
    first = spec["winding"]["slot_phase_map"][0]
    assert first["phase"] == layout[0][0]


def test_to_dxf_well_formed():
    from motor_nx import blueprint
    dxf = fea.to_dxf(blueprint.generate(MotorParams()))
    assert dxf.startswith("0\nSECTION") and dxf.rstrip().endswith("EOF")
    assert "\nCIRCLE\n" in dxf and "\nLWPOLYLINE\n" in dxf


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
