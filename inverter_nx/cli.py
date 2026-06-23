"""NX-independent command line for the inverter: inspect a design and emit the geometry
blueprint the NX builder consumes -- all without Siemens NX.

    python -m inverter_nx.cli report     [config.json]              design + ratings
    python -m inverter_nx.cli validate   [config.json]              buildability (exit code)
    python -m inverter_nx.cli blueprint  [config.json] [-o b.json]  geometry build steps
    python -m inverter_nx.cli thermal    [config.json]              loss + cold-plate heat flux

`config.json` is a (possibly partial) InverterParams dict; omit it for the default EV
traction inverter (drives the default motor_nx motor). The blueprint JSON is the exact
input the NX builder (run_journal inverter_nx/nx_builder.py) consumes.
"""

import argparse
import sys

from . import blueprint as bp
from . import engineering
from .params import InverterParams


def _load_params(path):
    return InverterParams.load_json(path) if path else InverterParams()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="inverter_nx.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_report = sub.add_parser("report", help="print a design summary + ratings")
    p_report.add_argument("config", nargs="?")

    p_val = sub.add_parser("validate", help="check buildability (exit 1 if not)")
    p_val.add_argument("config", nargs="?")

    p_bp = sub.add_parser("blueprint", help="emit the NX build-step JSON")
    p_bp.add_argument("config", nargs="?")
    p_bp.add_argument("-o", "--out", help="write JSON here (default stdout)")

    p_thr = sub.add_parser("thermal", help="loss + cold-plate heat-flux summary")
    p_thr.add_argument("config", nargs="?")

    args = parser.parse_args(argv)
    p = _load_params(getattr(args, "config", None))

    if args.cmd == "report":
        print(engineering.report(p))
        return 0

    if args.cmd == "validate":
        issues = engineering.validate(p)
        if not issues:
            print("OK: inverter design is sound + buildable")
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

    if args.cmd == "thermal":
        g = engineering.derive(p)
        print("Inverter thermal summary -- %s" % p.name)
        print("  peak power               : %.0f kW" % p.motor.peak_power_kw)
        print("  peak inverter loss       : %.2f kW" % g.peak_loss_kw)
        print("  cold-plate footprint     : %.0f x %.0f mm  (%.4f m^2)" % (
            p.cooling.coldplate_length_mm, p.cooling.coldplate_width_mm, g.coldplate_area_m2))
        print("  cold-plate heat flux     : %.1f W/cm^2  (coolant %s)" % (
            g.coldplate_heat_flux_w_cm2, p.cooling.coolant))
        return 0

    parser.error("unknown command")


if __name__ == "__main__":
    sys.exit(main())
