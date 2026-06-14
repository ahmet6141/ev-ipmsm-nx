"""NX-independent command line: inspect a design and emit its blueprint JSON.

    python -m motor_nx.cli report     [config.json]
    python -m motor_nx.cli validate   [config.json]
    python -m motor_nx.cli blueprint  [config.json] [-o blueprint.json]

`config.json` is a (possibly partial) MotorParams dict; omit it for the default
EV traction variant. The emitted blueprint.json is the exact input the NX builder
(run_journal nx_builder.py) consumes.
"""

import argparse
import sys

from . import blueprint as bp
from . import em_design
from .params import MotorParams


def _load_params(path):
    return MotorParams.load_json(path) if path else MotorParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="motor_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print a design summary + validation")
    p_report.add_argument("config", nargs="?")

    p_valid = sub.add_parser("validate", help="exit non-zero if the design is not buildable")
    p_valid.add_argument("config", nargs="?")

    p_bp = sub.add_parser("blueprint", help="write the geometry blueprint JSON")
    p_bp.add_argument("config", nargs="?")
    p_bp.add_argument("-o", "--out", default="blueprint.json")

    p_prev = sub.add_parser("preview", help="write an SVG cross-section (no NX needed)")
    p_prev.add_argument("config", nargs="?")
    p_prev.add_argument("-o", "--out", default="preview.svg")

    args = parser.parse_args(argv)
    params = _load_params(args.config)

    if args.cmd == "report":
        print(em_design.report(params))
        return 0

    if args.cmd == "validate":
        issues = em_design.validate(params)
        if issues:
            print("INVALID design:")
            for m in issues:
                print("  -", m)
            return 1
        print("OK: design is buildable")
        return 0

    if args.cmd == "blueprint":
        blueprint = bp.generate(params)
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(bp.to_json(blueprint))
        n = len(blueprint["build_steps"])
        print("wrote %s  (%d build steps, %d validation issue(s))"
              % (args.out, n, len(blueprint["validation"])))
        return 0

    if args.cmd == "preview":
        from . import preview
        blueprint = bp.generate(params)
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(preview.to_svg(blueprint))
        print("wrote %s" % args.out)
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
