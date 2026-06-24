"""NX-independent command line for the subframe: inspect a design and emit the
geometry blueprint the NX builder consumes -- all without Siemens NX.

    python -m subframe_nx.cli report     [config.json]              design + load paths
    python -m subframe_nx.cli validate   [config.json]              buildability (exit code)
    python -m subframe_nx.cli blueprint  [config.json] [-o b.json]  geometry build steps
    python -m subframe_nx.cli mass       [config.json]              mass + stiffness + bolts

`config.json` is a (possibly partial) SubframeParams dict; omit it for the default
REAR EV subframe. The blueprint JSON is the exact input the NX builder
(run_journal subframe_nx/nx_builder.py) consumes.
"""

import argparse
import sys

from . import blueprint as bp
from . import engineering
from .params import SubframeParams


def _load_params(path):
    return SubframeParams.load_json(path) if path else SubframeParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="subframe_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print a design summary + load paths")
    p_report.add_argument("config", nargs="?")

    p_val = sub.add_parser("validate", help="check buildability (exit 1 if not)")
    p_val.add_argument("config", nargs="?")

    p_bp = sub.add_parser("blueprint", help="emit the NX build-step JSON")
    p_bp.add_argument("config", nargs="?")
    p_bp.add_argument("-o", "--out", help="write JSON here (default stdout)")

    p_mass = sub.add_parser("mass", help="mass + stiffness + bolt sizing estimate")
    p_mass.add_argument("config", nargs="?")

    args = parser.parse_args(argv)
    p = _load_params(getattr(args, "config", None))

    if args.cmd == "report":
        print(engineering.report(p))
        return 0

    if args.cmd == "validate":
        issues = engineering.validate(p)
        if not issues:
            print("OK: subframe geometry is buildable")
            return 0
        print("NOT buildable -- %d issue(s):" % len(issues))
        for it in issues:
            print("  - %s" % it)
        return 1

    if args.cmd == "blueprint":
        blue = bp.generate(p)
        text = bp.to_json(blue)
        if args.out:
            with open(args.out, "w", encoding="utf-8") as fh:
                fh.write(text)
            print("wrote %s (%d build steps)" % (args.out, len(blue["build_steps"])))
        else:
            print(text)
        return 0

    if args.cmd == "mass":
        est = engineering.mass_breakdown(p)
        print("Subframe mass + stiffness + bolt estimate (rough) -- %s [%s]" % (p.name, p.axle))
        for k, v in est.items():
            print("  %-40s : %s" % (k, v))
        return 0

    parser.error("unknown command")


if __name__ == "__main__":
    sys.exit(main())
