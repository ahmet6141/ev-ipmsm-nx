"""NX-independent command line for the driveline: inspect a design and emit the
geometry blueprint the NX builder consumes -- all without Siemens NX.

    python -m driveline_nx.cli report     [config.json]              design + ratings
    python -m driveline_nx.cli validate   [config.json]              buildability (exit code)
    python -m driveline_nx.cli blueprint  [config.json] [-o b.json]  geometry build steps
    python -m driveline_nx.cli bearing    [config.json] [--load N]   wheel-hub L10 life estimate

`config.json` is a (possibly partial) DrivelineParams dict; omit it for the default
EV driveline (mates to the default motor_nx motor). The blueprint JSON is the exact
input the NX builder (run_journal driveline_nx/nx_builder.py) consumes.
"""

import argparse
import sys

from . import blueprint as bp
from . import engineering
from .params import DrivelineParams


def _load_params(path):
    return DrivelineParams.load_json(path) if path else DrivelineParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="driveline_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print a design summary + ratings")
    p_report.add_argument("config", nargs="?")

    p_val = sub.add_parser("validate", help="check buildability (exit 1 if not)")
    p_val.add_argument("config", nargs="?")

    p_bp = sub.add_parser("blueprint", help="emit the NX build-step JSON")
    p_bp.add_argument("config", nargs="?")
    p_bp.add_argument("-o", "--out", help="write JSON here (default stdout)")

    p_brg = sub.add_parser("bearing", help="wheel-hub bearing L10 life estimate")
    p_brg.add_argument("config", nargs="?")
    p_brg.add_argument("--load", type=float, default=6000.0, help="radial load (N)")

    args = parser.parse_args(argv)
    p = _load_params(getattr(args, "config", None))

    if args.cmd == "report":
        print(engineering.report(p))
        return 0

    if args.cmd == "validate":
        issues = engineering.validate(p)
        if not issues:
            print("OK: driveline geometry is buildable")
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

    if args.cmd == "bearing":
        est = engineering.bearing_life_estimate(p, args.load)
        print("Wheel-hub bearing L10 (rough, ISO 281) -- %s" % p.name)
        for k, v in est.items():
            print("  %-24s : %s" % (k, v))
        return 0

    parser.error("unknown command")


if __name__ == "__main__":
    sys.exit(main())
