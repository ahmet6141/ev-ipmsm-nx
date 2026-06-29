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


def test_drive_with_stack_flags():
    # active-stack bodies bind the NX stack_length expression; discrete magnets must NOT
    steps = blueprint.build_steps(MotorParams(), em_design.derive(MotorParams()))
    by_role = {}
    for s in steps:
        by_role.setdefault(s.role, s)
    assert by_role["stator_steel"].drive_with_stack is True
    assert by_role["rotor_steel"].drive_with_stack is True
    assert by_role["conductor"].drive_with_stack is True
    assert by_role["magnet"].drive_with_stack is False
    assert by_role["housing"].drive_with_stack is False


def test_housing_axially_covers_end_windings():
    p = MotorParams()
    bp = blueprint.generate(p)
    housing = next(s for s in bp["build_steps"] if s["role"] == "housing")
    ews = [s for s in bp["build_steps"] if s["role"] == "end_winding"]
    if ews:  # default models end-windings
        front = min(s["z0"] for s in ews)
        assert housing["z0"] <= front + 1e-6   # jacket overshadows the end-turns


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


# --------------------------------------------------------------------------- #
# rotor step-skew (cogging/ripple mitigation)
# --------------------------------------------------------------------------- #
def _steps_by_role(p, role):
    return [s for s in blueprint.generate(p)["build_steps"] if s["role"] == role]


def test_skew_off_by_default_matches_full_length_pockets():
    """skew_segments == 1 must reproduce the original single full-length pockets."""
    p = MotorParams()
    assert p.rotor.skew_segments == 1
    pockets = _steps_by_role(p, "magnet_pocket_cut")
    assert pockets and all("_sk" not in s["id"] for s in pockets)
    assert all(s["length"] == p.stack_length and s["drive_with_stack"] for s in pockets)


def test_skew_slices_pockets_and_magnets():
    K = 4
    p = MotorParams.from_dict({"rotor": {"skew_segments": K}})
    pockets = _steps_by_role(p, "magnet_pocket_cut")
    # every pocket arm is split into K rotated slices (ids carry _sk0.._sk{K-1})
    assert pockets and all("_sk" in s["id"] for s in pockets)
    assert len(pockets) % K == 0
    # the K slices of one arm tile the stack length with no overlap
    arm0 = sorted((s for s in pockets if s["id"].startswith("magnet_pocket_0_sk")),
                  key=lambda s: s["z0"])
    assert len(arm0) == K
    seg = p.stack_length / K
    for k, s in enumerate(arm0):
        assert abs(s["z0"] - k * seg) < 1e-6
        assert abs(s["length"] - seg) < 1e-6
        assert not s["drive_with_stack"]          # sliced features use literal length
    # magnets are segmented at least K-fold and skewed too
    magnets = _steps_by_role(p, "magnet")
    assert len(magnets) >= K


def test_skew_auto_angle_is_one_slot_pitch():
    """skew_angle_deg == 0 with skew_segments > 1 must auto-target one slot pitch."""
    K = 4
    p = MotorParams.from_dict({"rotor": {"skew_segments": K}})
    pockets = sorted((s for s in _steps_by_role(p, "magnet_pocket_cut")
                      if s["id"].startswith("magnet_pocket_0_sk")), key=lambda s: s["z0"])
    # rotation between consecutive slices = (360/slots)/K; recover it from the
    # centroid angle of each slice's profile
    def centroid_angle(prof):
        cx = sum(x for x, y in prof) / len(prof)
        cy = sum(y for x, y in prof) / len(prof)
        return math.degrees(math.atan2(cy, cx))
    step = (360.0 / p.stator.slot_count) / K
    a0 = centroid_angle(pockets[0]["profile"])
    a1 = centroid_angle(pockets[1]["profile"])
    assert abs((a1 - a0) - step) < 0.3   # ~1.667 deg per slice for 54 slots, K=4


def test_improved_v2_config_is_buildable():
    import json
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                       "configs", "improved_v2.json")))
    p = MotorParams.from_dict(cfg["base"])
    assert em_design.validate(p) == []
    assert p.rotor.skew_segments == 4 and p.material.magnet_grade == "N42EH"
