"""NX-independent geometry tests. Run with:  python -m pytest tests/  (or run directly).

These validate the *math* of the parametric model so a bad variant is caught
before any Siemens NX session is launched.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import blueprint, em_design  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def _radius(pt):
    return math.hypot(pt[0], pt[1])


def test_default_params_validate_clean():
    p = MotorParams()
    issues = em_design.validate(p)
    assert issues == [], "default motor should be buildable:\n" + "\n".join(issues)


def test_derived_geometry_consistent():
    p = MotorParams()
    g = em_design.derive(p)
    assert g.rotor_outer_radius == p.stator.bore_diameter / 2 - p.rotor.air_gap
    assert g.slot_depth > 0 and g.slot_width > 0
    # tooth width at r1 must match the requested design tooth width
    assert abs(g.tooth_width_at_r1 - p.stator.tooth_width) < 1e-6
    # q = 54 / (6*3) = 3
    assert abs(g.slots_per_pole_per_phase - 3.0) < 1e-9
    # 54s/6p integer-slot winding factor ~ 0.96
    assert 0.95 < g.winding_factor < 0.97


def test_magnets_inside_rotor_pole():
    p = MotorParams()
    g = em_design.derive(p)
    pockets = blueprint.magnet_pocket_polygons(p, g)
    half_pole = g.pole_pitch_deg / 2.0
    for poly in pockets:
        for pt in poly:
            r = _radius(pt)
            assert r <= g.rotor_outer_radius - p.rotor.outer_bridge + 1e-6, \
                f"pocket corner r={r:.2f} breaks the outer bridge"
            assert r >= g.shaft_radius - 1e-6, "pocket dips below the shaft"
            ang = abs(math.degrees(math.atan2(pt[1], pt[0])))
            assert ang <= half_pole + 1e-6, f"pocket angle {ang:.1f} exceeds half pole {half_pole:.1f}"
    # the +Y pocket must stay in +Y (does not cross the d-axis rib)
    assert min(pt[1] for pt in pockets[0]) > 0.0


def test_magnet_fits_in_pocket():
    p = MotorParams()
    g = em_design.derive(p)
    mags = blueprint.magnet_polygons(p, g)
    pockets = blueprint.magnet_pocket_polygons(p, g)
    # magnet corners must be within the pocket bounding radius
    for mag, pocket in zip(mags, pockets):
        mr = max(_radius(pt) for pt in mag)
        pr = max(_radius(pt) for pt in pocket)
        assert mr <= pr + 1e-9


def test_conductors_fit_slot():
    p = MotorParams()
    g = em_design.derive(p)
    bars = blueprint.conductor_polygons(p, g)
    assert len(bars) == p.winding.conductors_per_slot
    for bar in bars:
        rs = [_radius(pt) for pt in bar]
        assert min(rs) >= g.slot_body_inner_radius - 1e-6
        assert max(rs) <= g.slot_body_outer_radius + 1e-6


def test_blueprint_serialises_and_counts():
    p = MotorParams()
    bp = blueprint.generate(p)
    text = blueprint.to_json(bp)
    assert '"schema": "motor_nx.blueprint/1"' in text
    roles = [s["role"] for s in bp["build_steps"]]
    assert "stator_steel" in roles and "rotor_steel" in roles and "shaft" in roles
    assert "magnet" in roles and "conductor" in roles and "housing" in roles
    # one slot step patterned over all slots
    slot_step = next(s for s in bp["build_steps"] if s["role"] == "stator_slot_cut")
    assert slot_step["pattern_count"] == p.stator.slot_count


def test_invalid_variant_is_flagged():
    p = MotorParams()
    p.stator.tooth_width = 50.0  # absurd -> no room for the slot
    issues = em_design.validate(p)
    assert any("Slot width" in m for m in issues)


def test_instance_expansion():
    p = MotorParams()
    g = em_design.derive(p)
    steps = blueprint.build_steps(p, g)
    slots = next(s for s in steps if s.role == "stator_slot_cut")
    inst = blueprint.expand_step_instances(slots)
    assert len(inst) == p.stator.slot_count
    # instance k is rotated k * slot_pitch -> its polygon centroid sits at that angle
    k = p.stator.slot_count // 4
    expected = k * (360.0 / p.stator.slot_count)
    mid = inst[k]
    cx = sum(x for x, _ in mid["profile"]) / len(mid["profile"])
    cy = sum(y for _, y in mid["profile"]) / len(mid["profile"])
    assert abs(math.degrees(math.atan2(cy, cx)) - expected) < 0.5


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception:
            failed += 1
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
