#!/usr/bin/env python
"""Headless batch driver: turn a sweep config into a set of parametric IPMSM
variants, generate each blueprint, and drive Siemens NX (run_journal.exe) to
build + export every one. Plain CPython -- it orchestrates NX, it does not import
NXOpen itself.

    python batch_build.py configs/default.json
    python batch_build.py configs/sweep_example.json
    python batch_build.py configs/sweep_example.json --dry-run   # no NX, just blueprints+validation

Config schema (all keys optional except as noted):
    {
      "name":      "ev_rdu",                 # base name for outputs
      "base":      { ...MotorParams partial... },   # overrides on the default design
      "output_dir":"build",
      "export":    "both",                   # step | parasolid | both | none
      "nx_root":   "C:/Program Files/Siemens/NX2306/NXBIN",  # else $UGII_ROOT_DIR
      "sweep":     { "stack_length":[120,134,150], "rotor.magnet_width":[24,26] },
      "variants":  [ { "name":"hi_pole", "overrides": { "rotor.pole_count":8 } } ]
    }

`sweep` is expanded as a full cartesian product of its dotted-key value lists;
`variants` adds explicit named designs. Invalid variants are reported and skipped
(never sent to NX) so a bad point in a sweep cannot corrupt a batch run.
"""

import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys

from motor_nx import blueprint as bp
from motor_nx import em_design
from motor_nx.params import MotorParams

HERE = os.path.dirname(os.path.abspath(__file__))
NX_BUILDER = os.path.join(HERE, "motor_nx", "nx_builder.py")


# --------------------------------------------------------------------------- #
# variant expansion
# --------------------------------------------------------------------------- #
def _safe(name):
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in str(name))


def expand_variants(config):
    """Yield (variant_name, MotorParams) for every design in the config."""
    base = MotorParams.from_dict(config.get("base", {})) if config.get("base") else MotorParams()
    base_name = config.get("name", base.name)

    sweep = config.get("sweep", {})
    if sweep:
        keys = list(sweep.keys())
        for combo in itertools.product(*(sweep[k] for k in keys)):
            overrides = dict(zip(keys, combo))
            tag = "__".join("%s-%s" % (_safe(k.split(".")[-1]), _safe(v)) for k, v in overrides.items())
            params = base.overridden(**overrides)
            params.name = "%s__%s" % (base_name, tag)
            yield params.name, params

    for variant in config.get("variants", []):
        params = base.overridden(**variant.get("overrides", {}))
        params.name = "%s__%s" % (base_name, _safe(variant.get("name", "variant")))
        yield params.name, params

    if not sweep and not config.get("variants"):
        base.name = base_name
        yield base_name, base


# --------------------------------------------------------------------------- #
# NX invocation
# --------------------------------------------------------------------------- #
def find_run_journal(config):
    nx_root = config.get("nx_root") or os.environ.get("UGII_ROOT_DIR")
    candidates = []
    if nx_root:
        candidates.append(os.path.join(nx_root, "run_journal.exe"))
    on_path = shutil.which("run_journal") or shutil.which("run_journal.exe")
    if on_path:
        candidates.append(on_path)
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return None


def run_nx(run_journal, blueprint_path, out_prt, export_mode, log_path):
    cmd = [run_journal, NX_BUILDER, "-args", blueprint_path, out_prt, export_mode]
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
    return proc.returncode


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main(argv=None):
    parser = argparse.ArgumentParser(description="Headless parametric IPMSM batch builder for Siemens NX")
    parser.add_argument("config", help="sweep/variant config JSON")
    parser.add_argument("--dry-run", action="store_true",
                        help="generate blueprints + validation only; do not launch NX")
    args = parser.parse_args(argv)

    with open(args.config, "r", encoding="utf-8") as fh:
        config = json.load(fh)

    out_dir = os.path.abspath(config.get("output_dir", "build"))  # absolute: NX writes here
    export_mode = config.get("export", "both")
    os.makedirs(out_dir, exist_ok=True)

    run_journal = None if args.dry_run else find_run_journal(config)
    if not args.dry_run and run_journal is None:
        print("run_journal.exe not found (set 'nx_root' in the config or $UGII_ROOT_DIR).")
        print("Falling back to --dry-run: blueprints are still generated.\n")
        args.dry_run = True

    built = skipped = failed = 0
    manifest = []
    for name, params in expand_variants(config):
        issues = em_design.validate(params)
        blueprint = bp.generate(params)
        bp_path = os.path.join(out_dir, name + ".blueprint.json")
        with open(bp_path, "w", encoding="utf-8") as fh:
            fh.write(bp.to_json(blueprint))

        record = {"name": name, "blueprint": bp_path, "valid": not issues}
        if issues:
            print("SKIP  %-40s INVALID: %s" % (name, issues[0]))
            skipped += 1
            record["issues"] = issues
            manifest.append(record)
            continue

        if args.dry_run:
            print("DRY   %-40s blueprint -> %s" % (name, os.path.basename(bp_path)))
            manifest.append(record)
            continue

        out_prt = os.path.join(out_dir, name + ".prt")
        log_path = os.path.join(out_dir, name + ".log")
        rc = run_nx(run_journal, bp_path, out_prt, export_mode, log_path)
        record.update({"prt": out_prt, "log": log_path, "returncode": rc})
        manifest.append(record)
        if rc == 0:
            print("BUILD %-40s -> %s (rc=0)" % (name, os.path.basename(out_prt)))
            built += 1
        else:
            print("ERROR %-40s rc=%s (see %s)" % (name, rc, os.path.basename(log_path)))
            failed += 1

    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)

    total = built + skipped + failed + sum(1 for r in manifest if args.dry_run and r["valid"])
    print("\n%d variant(s): %d built, %d skipped(invalid), %d failed%s"
          % (len(manifest), built, skipped, failed, "  [dry-run]" if args.dry_run else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
