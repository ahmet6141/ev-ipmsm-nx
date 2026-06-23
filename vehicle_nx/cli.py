"""NX-independent command line for the vehicle assembly: compute and inspect the
placement plan the NX assembler consumes -- without Siemens NX.

    python -m vehicle_nx.cli report   [config.json]              layout + placements
    python -m vehicle_nx.cli validate [config.json]              consistency (exit code)
    python -m vehicle_nx.cli plan      [config.json] [-o p.json]  assembly plan JSON

`config.json` is a (possibly partial) VehicleParams dict; omit it for the default EV
layout. The plan JSON is the exact input the NX assembler (run_journal
vehicle_nx/nx_assembler.py) consumes.
"""

import argparse
import sys

from . import assembly as asm
from .params import VehicleParams


def _load_params(path):
    return VehicleParams.load_json(path) if path else VehicleParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="vehicle_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print the vehicle layout + placements")
    p_report.add_argument("config", nargs="?")

    p_val = sub.add_parser("validate", help="check layout consistency (exit 1 if not)")
    p_val.add_argument("config", nargs="?")

    p_plan = sub.add_parser("plan", help="emit the assembly plan JSON")
    p_plan.add_argument("config", nargs="?")
    p_plan.add_argument("-o", "--out", help="write JSON here (default stdout)")

    args = parser.parse_args(argv)
    p = _load_params(getattr(args, "config", None))

    if args.cmd == "report":
        print(asm.report(p))
        return 0

    if args.cmd == "validate":
        issues = asm.validate(p)
        if not issues:
            print("OK: vehicle layout is consistent")
            return 0
        print("INCONSISTENT -- %d issue(s):" % len(issues))
        for it in issues:
            print("  - %s" % it)
        return 1

    if args.cmd == "plan":
        plan = asm.build_plan(p)
        text = asm.to_json(plan)
        if args.out:
            with open(args.out, "w", encoding="utf-8") as fh:
                fh.write(text)
            print("wrote %s (%d components)" % (args.out, len(plan["components"])))
        else:
            print(text)
        return 0

    parser.error("unknown command")


if __name__ == "__main__":
    sys.exit(main())
