"""NX-independent tests for the manufacturing / assembly features (fastening
holes, keyways, mounting flange, coolant ports, terminal, lifting eye).

These guard the geometry + validation of the production details layered on top of
the electromagnetically-active solid -- so a bad assembly feature is caught before
any Siemens NX session is launched.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import blueprint, em_design, manufacturing as mfg  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


# expected assembly-feature roles in the default (all-enabled) build
_ASSEMBLY_ROLES = {
    "stator_tie_rod_cut", "stator_key_cut", "rotor_rivet_cut",
    "shaft_keyway_cut", "shaft_groove_cut", "shaft_oil_cut",
    "housing_bolt_cut", "housing_port_cut", "housing_terminal_cut",
    "housing_lifting_cut",
}


def test_default_assembly_validates_clean():
    p = MotorParams()
    assert p.assembly.enabled is True
    assert em_design.validate(p) == []


def test_assembly_roles_present_by_default():
    roles = {s["role"] for s in blueprint.generate(MotorParams())["build_steps"]}
    missing = _ASSEMBLY_ROLES - roles
    assert not missing, "missing assembly roles: %s" % missing
    # the housing flange is united in (role 'housing'), not a stray body
    flange = [s for s in blueprint.generate(MotorParams())["build_steps"]
              if s["id"].startswith("housing_flange") and s["boolean"] == "unite"]
    assert len(flange) == 2  # DE + NDE


def test_assembly_disabled_is_pure_em_solid():
    p = MotorParams.from_dict({"assembly": {"enabled": False}})
    roles = {s["role"] for s in blueprint.generate(p)["build_steps"]}
    assert not (_ASSEMBLY_ROLES & roles), "no assembly cuts when disabled"
    assert em_design.validate(p) == []


def test_zeroed_feature_is_skipped():
    p = MotorParams.from_dict({"assembly": {"stator_tie_rod_count": 0,
                                            "housing_lifting_hole_diameter": 0.0}})
    roles = {s["role"] for s in blueprint.generate(p)["build_steps"]}
    assert "stator_tie_rod_cut" not in roles
    assert "housing_lifting_cut" not in roles
    # other features still present
    assert "shaft_keyway_cut" in roles


def test_radial_hole_steps_carry_a_radial_axis():
    steps = blueprint.build_steps(MotorParams(), em_design.derive(MotorParams()))
    holes = [s for s in steps if s.kind == "hole"]
    assert holes, "expected radial 'hole' steps (ports/oil/terminal/lifting)"
    for h in holes:
        ax, ay, az = h.axis
        assert abs(az) < 1e-9, "radial holes point in the XY plane (no Z component)"
        assert math.hypot(ax, ay) > 0.5, "radial hole must have a real in-plane direction"


def test_hole_pattern_rotates_axis_and_base():
    # the shaft oil holes are a ring of radial holes -> expansion rotates base + axis
    steps = blueprint.build_steps(MotorParams(), em_design.derive(MotorParams()))
    oil = next(s for s in steps if s.role == "shaft_oil_cut")
    inst = blueprint.expand_step_instances(oil)
    assert len(inst) == MotorParams().assembly.shaft_oil_hole_count
    # instance k base sits at k * (360/count) about Z
    k = 1
    expected = k * (360.0 / len(inst))
    bx, by = inst[k]["cx"], inst[k]["cy"]
    assert abs((math.degrees(math.atan2(by, bx)) - expected + 180) % 360 - 180) < 0.5
    assert "axis" in inst[k]


def test_validation_flags_oversized_tie_rod():
    p = MotorParams.from_dict({"assembly": {"stator_tie_rod_diameter": 40.0}})
    assert any("tie-rod" in m for m in em_design.validate(p))


def test_validation_flags_deep_shaft_keyway():
    p = MotorParams.from_dict({"assembly": {"shaft_keyway_depth": 18.0}})
    assert any("keyway" in m for m in em_design.validate(p))


def test_validation_flags_rivet_into_pocket():
    p = MotorParams.from_dict({"assembly": {"rotor_rivet_pitch_radius": 56.0}})
    assert any("rivet" in m for m in em_design.validate(p))


def test_validation_flags_oil_hole_without_bore():
    p = MotorParams.from_dict({"shaft": {"bore_diameter": 0.0},
                               "assembly": {"shaft_oil_hole_count": 4}})
    assert any("oil cross-holes" in m for m in em_design.validate(p))


def test_bom_includes_flange_and_stays_in_range():
    bom = mfg.bill_of_materials(MotorParams())
    assert 25 < bom["total_mass_kg"] < 70, bom["total_mass_kg"]
    housing = next(i for i in bom["line_items"] if i["component"].startswith("Housing"))
    assert housing["qty"] == 1, "flanges merge into the one housing casting"
    # the flange lip adds aluminium -> housing is heavier than a bare jacket
    bare = mfg.bill_of_materials(MotorParams.from_dict({"assembly": {"enabled": False}}))
    bare_h = next(i for i in bare["line_items"] if i["component"].startswith("Housing"))
    assert housing["mass_kg"] > bare_h["mass_kg"] + 0.5


def test_holes_reduce_steel_mass():
    full = mfg.bill_of_materials(MotorParams())
    none = mfg.bill_of_materials(MotorParams.from_dict({"assembly": {"enabled": False}}))
    f_st = next(i for i in full["line_items"] if i["component"].startswith("Stator lam"))
    n_st = next(i for i in none["line_items"] if i["component"].startswith("Stator lam"))
    assert f_st["mass_kg"] < n_st["mass_kg"], "tie-rod/key cuts remove stator steel"


def test_hardware_schedule_tracks_features():
    rows = mfg.hardware_schedule(MotorParams())
    items = {r["item"] for r in rows}
    assert any("shaft key" in i.lower() for i in items)
    assert any("mounting bolt" in i.lower() for i in items)
    assert any("retaining ring" in i.lower() for i in items)
    for r in rows:
        assert r["qty"] >= 1 and {"item", "standard", "size", "qty"} <= set(r)


def test_assembly_tolerances_present():
    feats = [t["feature"] for t in mfg.TOLERANCES(MotorParams())]
    assert any("keyway" in f.lower() for f in feats)
    assert any("flange" in f.lower() for f in feats)


# --- output stub / end-shields / boss / DFM / package -------------------- #
def test_shaft_output_stub_extends_profile_and_carries_keyway():
    p = MotorParams()
    g = em_design.derive(p)
    prof = blueprint.shaft_profile(p, g)
    z_r = p.stack_length + p.shaft.overhang
    z_max = max(z for _, z in prof)
    assert abs(z_max - (z_r + p.shaft.drive_stub_length)) < 1e-6, "stub extends the shaft past the DE seat"
    # the smallest non-bore radius equals the stub radius
    radii = sorted({round(r, 3) for r, _ in prof if r > p.shaft.bore_diameter / 2.0 + 1e-6})
    assert abs(radii[0] - p.shaft.drive_stub_diameter / 2.0) < 1e-6
    # keyway sits on the stub surface (its outer corner ~ stub radius)
    kw = next(s for s in blueprint.build_steps(p, g) if s.role == "shaft_keyway_cut")
    assert abs(max(x for x, _ in kw.profile) - (p.shaft.drive_stub_diameter / 2.0 + 0.5)) < 1e-6


def test_endshields_are_two_real_bodies():
    p = MotorParams()
    steps = blueprint.build_steps(p, em_design.derive(p))
    es = [s for s in steps if s.role == "endshield" and s.boolean == "create"]
    assert len(es) == 2, "DE + NDE end-shields are real create bodies"
    bom = mfg.bill_of_materials(p)
    cap = next(i for i in bom["line_items"] if i["component"].startswith("End-shield"))
    assert cap["qty"] == 2 and cap["mass_kg"] > 0


def test_endshields_disabled_flag():
    p = MotorParams.from_dict({"assembly": {"endshield_enabled": False}})
    roles = {s["role"] for s in blueprint.generate(p)["build_steps"]}
    assert "endshield" not in roles
    assert em_design.validate(p) == []


def test_terminal_boss_unites_when_enabled():
    steps = blueprint.build_steps(MotorParams(), em_design.derive(MotorParams()))
    boss = [s for s in steps if s.id == "housing_terminal_boss"]
    assert boss and boss[0].boolean == "unite" and boss[0].kind == "hole"


def test_validation_flags_stub_not_stepping_down():
    p = MotorParams.from_dict({"shaft": {"drive_stub_diameter": 50.0}})
    assert any("stub" in m for m in em_design.validate(p))


def test_validation_flags_small_endshield_bore():
    p = MotorParams.from_dict({"assembly": {"endshield_bearing_bore": 38.0}})
    assert any("bearing bore" in m for m in em_design.validate(p))


def test_dfm_monte_carlo_deterministic_and_capable():
    a = mfg.eccentricity_monte_carlo(MotorParams(), n=4000)
    b = mfg.eccentricity_monte_carlo(MotorParams(), n=4000)
    assert a == b, "seeded Monte Carlo must be deterministic"
    assert 0.0 <= a["fraction_over_budget"] <= 1.0
    assert a["cpk"] > 0 and a["mean_mm"] > 0
    # random phases rarely align -> mean below the deterministic worst-case sum
    assert a["mean_mm"] < a["worst_case_mm"]


def test_manufacturing_package_writes_everything(tmp_path=None):
    import tempfile
    out = tempfile.mkdtemp()
    written = mfg.write_manufacturing_package(MotorParams(), out)
    names = {os.path.basename(p) for p in written}
    assert {"bom.csv", "hardware.csv", "tolerances.csv",
            "dfm_eccentricity.txt", "manufacturing_summary.md"} <= names
    assert sum(p.endswith(".dxf") for p in written) == 5  # all 5 drawing sheets
    assert all(os.path.getsize(p) > 0 for p in written)


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
