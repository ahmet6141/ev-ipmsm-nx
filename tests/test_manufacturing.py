"""NX-independent tests for the model-derived BOM and tolerance scheme."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import manufacturing as mfg  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_polygon_area_unit_square():
    assert abs(mfg._polygon_area([[0, 0], [2, 0], [2, 3], [0, 3]]) - 6.0) < 1e-9


def test_revolve_volume_matches_tube():
    # rectangle r in [10,12], z in [0,5] revolved 360 deg = tube vol pi(12^2-10^2)*5
    prof = [[10, 0], [12, 0], [12, 5], [10, 5]]
    expected = math.pi * (12 ** 2 - 10 ** 2) * 5
    assert abs(mfg._revolve_volume(prof, 360.0) - expected) / expected < 1e-9
    # half revolve = half the volume
    assert abs(mfg._revolve_volume(prof, 180.0) - expected / 2) / expected < 1e-9


def test_net_volume_below_raw_for_cut_bodies():
    rows = mfg.step_volumes(MotorParams())
    stator = next(r for r in rows if r["id"] == "stator_steel")
    # slots are subtracted -> net steel must be less than the raw tube
    assert stator["net_volume"] < stator["raw_volume"]
    assert stator["net_volume"] > 0


def test_bom_structure_and_magnitudes():
    bom = mfg.bill_of_materials(MotorParams())
    assert bom["line_items"], "BOM should not be empty"
    for it in bom["line_items"]:
        assert it["mass_kg"] > 0
        assert {"component", "material", "qty", "mass_kg"} <= set(it)
    # Model-3-class active mass is tens of kg, total a bit more
    assert 25 < bom["total_mass_kg"] < 70, bom["total_mass_kg"]
    assert bom["magnet_mass_kg"] > 0
    assert bom["copper_mass_kg"] > 0


def test_magnet_count_matches_segmentation():
    p = MotorParams()
    bom = mfg.bill_of_materials(p)
    mag = next(i for i in bom["line_items"] if "magnet" in i["component"].lower())
    # 2 V-arms x n_seg axial segments x pole_count
    expected = 2 * max(1, p.material.magnet_segments_axial) * p.rotor.pole_count
    assert mag["qty"] == expected


def test_solid_magnet_when_one_segment():
    p = MotorParams()
    p.material.magnet_segments_axial = 1
    bom = mfg.bill_of_materials(p)
    mag = next(i for i in bom["line_items"] if "magnet" in i["component"].lower())
    assert mag["qty"] == 2 * p.rotor.pole_count


def test_tolerances_complete():
    tol = mfg.TOLERANCES(MotorParams())
    assert len(tol) >= 8
    for t in tol:
        assert {"feature", "nominal", "tolerance", "gdt", "rationale"} <= set(t)


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print("PASS ", fn.__name__)
        except Exception:
            failed += 1
            print("FAIL ", fn.__name__)
            traceback.print_exc()
    print("\n%d/%d passed" % (len(fns) - failed, len(fns)))
    sys.exit(1 if failed else 0)
