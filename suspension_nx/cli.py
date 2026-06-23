"""NX-independent command line for the suspension corner: inspect a design and emit
the geometry blueprint the NX builder consumes -- all without Siemens NX.

    python -m suspension_nx.cli report     [config.json]              design + ratings
    python -m suspension_nx.cli validate   [config.json]              buildability (exit code)
    python -m suspension_nx.cli blueprint  [config.json] [-o b.json]  geometry build steps
    python -m suspension_nx.cli rates      [config.json]              wheel rate / ride freq / roll

`config.json` is a (possibly partial) SuspensionParams dict; omit it for the default
EV corner (carries the default driveline_nx wheel hub). The blueprint JSON is the
exact input the NX builder (run_journal suspension_nx/nx_builder.py) consumes.
"""

import argparse
import sys

from . import blueprint as bp
from . import engineering
from .params import SuspensionParams


def _load_params(path):
    return SuspensionParams.load_json(path) if path else SuspensionParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="suspension_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print a design summary + ratings")
    p_report.add_argument("config", nargs="?")

    p_val = sub.add_parser("validate", help="check buildability (exit 1 if not)")
    p_val.add_argument("config", nargs="?")

    p_bp = sub.add_parser("blueprint", help="emit the NX build-step JSON")
    p_bp.add_argument("config", nargs="?")
    p_bp.add_argument("-o", "--out", help="write JSON here (default stdout)")

    p_rates = sub.add_parser("rates", help="wheel rate / ride frequency / roll stiffness")
    p_rates.add_argument("config", nargs="?")

    args = parser.parse_args(argv)
    p = _load_params(getattr(args, "config", None))

    if args.cmd == "report":
        print(engineering.report(p))
        return 0

    if args.cmd == "validate":
        issues = engineering.validate(p)
        if not issues:
            print("OK: suspension geometry is buildable")
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

    if args.cmd == "rates":
        g = engineering.derive(p)
        print("Suspension rates -- %s" % p.name)
        print("  %-24s : %.2f N/mm" % ("wheel rate", g.wheel_rate_n_per_mm))
        print("  %-24s : %.2f Hz" % ("ride frequency", g.ride_frequency_hz))
        print("  %-24s : %.0f Nm/deg" % ("roll stiffness", g.roll_stiffness_nm_per_deg))
        print("  %-24s : %.2f" % ("damping ratio (bump)", g.damping_ratio))
        print("  %-24s : %.1f Hz" % ("wheel-hop frequency", g.wheel_hop_frequency_hz))
        return 0

    parser.error("unknown command")


if __name__ == "__main__":
    sys.exit(main())
