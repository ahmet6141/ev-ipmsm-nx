"""Validate every config in configs/ the same way `batch_build --dry-run` does:
expand variants, run em_design.validate(), and generate each blueprint. This locks
the config schema + the no-NX dry-run path so a bad config is caught by the test
suite rather than at NX launch time. NX-independent."""

import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import batch_build  # noqa: E402  (root-level driver; importing it does not launch NX)
from motor_nx import blueprint as bp  # noqa: E402
from motor_nx import em_design  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402

CONFIG_DIR = os.path.join(ROOT, "configs")


def _configs():
    return sorted(glob.glob(os.path.join(CONFIG_DIR, "*.json")))


def test_configs_present():
    files = _configs()
    assert files, "no configs/*.json found"
    # the two documented configs must exist
    names = {os.path.basename(f) for f in files}
    assert "default.json" in names and "sweep_example.json" in names


def test_every_config_loads_and_expands():
    for path in _configs():
        with open(path, "r", encoding="utf-8") as fh:
            config = json.load(fh)
        variants = list(batch_build.expand_variants(config))
        assert variants, "%s expanded to zero variants" % os.path.basename(path)
        for name, params in variants:
            assert isinstance(params, MotorParams)
            issues = em_design.validate(params)          # list (empty => buildable)
            blueprint = bp.generate(params)              # must never crash, even if invalid
            assert isinstance(blueprint, dict) and "build_steps" in blueprint
            # a clean variant must emit real geometry and serialise to blueprint JSON
            if not issues:
                assert blueprint["build_steps"]
                assert bp.to_json(blueprint).startswith("{")


def test_default_config_is_valid_clean():
    with open(os.path.join(CONFIG_DIR, "default.json"), "r", encoding="utf-8") as fh:
        config = json.load(fh)
    variants = list(batch_build.expand_variants(config))
    assert len(variants) == 1                            # single build, no sweep/variants
    _name, params = variants[0]
    assert em_design.validate(params) == []              # the reference design is buildable


def test_sweep_expands_cartesian_plus_named_variant():
    with open(os.path.join(CONFIG_DIR, "sweep_example.json"), "r", encoding="utf-8") as fh:
        config = json.load(fh)
    variants = list(batch_build.expand_variants(config))
    names = [n for n, _ in variants]
    # 3 stack lengths x 3 magnet widths = 9 sweep points, plus the explicit 8-pole variant
    assert len(variants) == 3 * 3 + 1
    assert any("8pole_48slot" in n for n in names)
    # the 8-pole/48-slot point keeps an integer-slot winding (q=2) and must at least
    # generate a blueprint (batch_build auto-skips it only if validate() flags it).
    eight = next(params for n, params in variants if "8pole_48slot" in n)
    assert eight.rotor.pole_count == 8 and eight.stator.slot_count == 48
    assert eight.stator.slot_count % (eight.rotor.pole_count * eight.winding.phases) == 0
    assert bp.generate(eight)["build_steps"]


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
