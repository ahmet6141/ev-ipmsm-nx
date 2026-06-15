"""NX-independent command line: inspect a design, emit its blueprint, and produce
manufacturing/FEA hand-off data -- all without Siemens NX.

    python -m motor_nx.cli report      [config.json]              design + performance
    python -m motor_nx.cli validate    [config.json]              buildability (exit code)
    python -m motor_nx.cli blueprint   [config.json] [-o b.json]  geometry build steps
    python -m motor_nx.cli preview      [config.json] [-o p.svg]  SVG cross-section
    python -m motor_nx.cli fea          [config.json] [-o fea/]   FEA hand-off package
    python -m motor_nx.cli bom          [config.json] [--csv b.csv]  Bill of Materials
    python -m motor_nx.cli tolerances   [config.json] [--csv t.csv]  GD&T scheme
    python -m motor_nx.cli drawings     [config.json] [-o drawings/] 2D drawings (DXF+SVG)

`config.json` is a (possibly partial) MotorParams dict; omit it for the default
EV traction variant. The blueprint JSON is the exact input the NX builder
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

    p_fea = sub.add_parser("fea", help="write the FEA hand-off package (spec JSON + DXF + winding map)")
    p_fea.add_argument("config", nargs="?")
    p_fea.add_argument("-o", "--out", default="fea")

    p_bom = sub.add_parser("bom", help="geometry-derived Bill of Materials (mass + count)")
    p_bom.add_argument("config", nargs="?")
    p_bom.add_argument("--csv", help="also write the BOM to this CSV path")

    p_tol = sub.add_parser("tolerances", help="critical-dimension / GD&T scheme")
    p_tol.add_argument("config", nargs="?")
    p_tol.add_argument("--csv", help="also write the tolerance table to this CSV path")

    p_dwg = sub.add_parser("drawings", help="2D manufacturing drawings (DXF + SVG): assembly, stator, rotor")
    p_dwg.add_argument("config", nargs="?")
    p_dwg.add_argument("-o", "--out", default="drawings")

    args = parser.parse_args(argv)
    params = _load_params(args.config)

    if args.cmd == "report":
        print(em_design.report(params))
        print()
        print(em_design.performance_report(params))
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

    if args.cmd == "fea":
        from . import fea
        written = fea.write_package(params, args.out)
        print("wrote FEA hand-off package:")
        for path in written:
            print("  ", path)
        return 0

    if args.cmd == "bom":
        from . import manufacturing as mfg
        print(mfg.bom_report(params))
        if args.csv:
            import csv
            bom = mfg.bill_of_materials(params)
            with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                wr = csv.writer(fh)
                wr.writerow(["component", "material", "qty", "mass_kg", "note"])
                for it in bom["line_items"]:
                    wr.writerow([it["component"], it["material"], it["qty"],
                                 it["mass_kg"], it.get("note", "")])
                wr.writerow([])
                wr.writerow(["TOTAL", "", "", bom["total_mass_kg"], "modelled mass"])
            print("\nwrote %s" % args.csv)
        return 0

    if args.cmd == "drawings":
        import datetime
        from . import drawings
        written = drawings.write_drawings(params, args.out, datetime.date.today().isoformat())
        print("wrote %d 2D drawing files to %s/:" % (len(written), args.out))
        for path in written:
            print("  ", path)
        return 0

    if args.cmd == "tolerances":
        from . import manufacturing as mfg
        print(mfg.tolerance_report(params))
        if args.csv:
            import csv
            with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                wr = csv.writer(fh)
                wr.writerow(["feature", "nominal", "datum", "tolerance", "gdt", "rationale"])
                for t in mfg.TOLERANCES(params):
                    wr.writerow([t["feature"], t["nominal"], t.get("datum", ""),
                                 t["tolerance"], t["gdt"], t["rationale"]])
            print("\nwrote %s" % args.csv)
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
