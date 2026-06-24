"""FEMM-independent tests for the pure-geometry layer (verification/femm_geom.py).

These validate the planar straight-line graph the FEMM emitter replays -- radius
ordering, label coverage, winding turns balance, magnet count/magnetisation and
the slot-mouth break in the bore -- without needing FEMM installed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from verification import femm_geom as fg  # noqa: E402
from motor_nx import em_design  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_build_and_sanity_clean():
    model = fg.build()
    assert fg._sanity(model) == []          # the standalone sanity check passes


def test_radius_ordering():
    R = fg.build().radii
    assert R["shaft"] < R["rotor_OD"] < R["bore"] < R["slot_body_inner"] < \
        R["slot_body_outer"] < R["stator_OD"]
    # the torque ring sits inside the clean (rotor-surface .. mid-gap) annulus
    assert R["rotor_OD"] < R["torque_ring_r"] < R["air_gap_mid"] < R["bore"]


def test_label_tally_matches_machine():
    p = MotorParams()
    model = fg.build(p)
    tally = model.label_tally()
    assert tally[fg.MAT_COPPER] == p.stator.slot_count            # one coil region per slot
    assert tally[fg.MAT_MAGNET] == p.rotor.pole_count * p.rotor.magnets_per_pole
    assert tally[fg.MAT_AIR] >= 2                                 # 2 gap rings + shaft + pockets
    assert tally[fg.MAT_STEEL] >= 2                               # yoke + rotor core


def test_winding_turns_balanced_and_present():
    model = fg.build()
    bal = {}
    for lb in model.labels:
        if lb.material == fg.MAT_COPPER:
            assert lb.circuit in ("A", "B", "C")
            assert lb.turns != 0
            bal[lb.circuit] = bal.get(lb.circuit, 0) + lb.turns
    assert set(bal) == {"A", "B", "C"}
    for ph in bal:
        assert bal[ph] == 0                                       # signed turns cancel


def test_magnet_groups_and_magdirs():
    model = fg.build()
    mags = [lb for lb in model.labels if lb.material == fg.MAT_MAGNET]
    assert all(lb.group == fg.GROUP_ROTOR for lb in mags)         # magnets rotate
    # N/S poles alternate -> magdirs come in two sets 180 deg apart per arm
    assert len(mags) == 12
    assert all(0.0 <= lb.magdir_deg < 360.0 for lb in mags)


def test_stator_and_airgap_in_fixed_group():
    model = fg.build()
    for lb in model.labels:
        if lb.material == fg.MAT_COPPER or "air gap" in lb.note:
            assert lb.group == fg.GROUP_STATOR
    # the rotor steel / shaft / magnets / pockets all rotate
    rotor_notes = ("rotor", "shaft", "pocket")
    for lb in model.labels:
        if any(k in lb.note for k in rotor_notes) or lb.material == fg.MAT_MAGNET:
            assert lb.group == fg.GROUP_ROTOR


def test_bore_arcs_break_at_each_slot_mouth():
    p = MotorParams()
    model = fg.build(p)
    g = em_design.derive(p)
    R_sb = g.bore_radius
    # count arcs that lie on the bore radius (tooth tips) -- exactly one per slot
    tol = 1e-3
    bore_arcs = [a for a in model.arcs
                 if abs(math.hypot(a.x1, a.y1) - R_sb) < tol
                 and abs(math.hypot(a.x2, a.y2) - R_sb) < tol]
    assert len(bore_arcs) == p.stator.slot_count
    # each tooth-tip arc spans less than a full slot pitch (a mouth gap remains)
    pitch = 360.0 / p.stator.slot_count
    assert all(0.0 < a.angle_deg < pitch for a in bore_arcs)


def test_segments_have_no_zero_length():
    model = fg.build()
    for s in model.segs:
        assert math.hypot(s.x2 - s.x1, s.y2 - s.y1) > 1e-6


def test_circuits_declared():
    model = fg.build()
    assert model.circuits == ["A", "B", "C"]
    assert model.depth_mm == MotorParams().stack_length


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
