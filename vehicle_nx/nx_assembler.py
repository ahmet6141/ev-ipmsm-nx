"""Siemens NX ASSEMBLY journal. RUN INSIDE NX (headless) via run_journal:

    "%UGII_ROOT_DIR%\\run_journal.exe" vehicle_nx\\nx_assembler.py -args [plan.json] [vehicle.prt] [build|nobuild]

Two stages:
  1. BUILD each subsystem part (motor / driveline / inverter / suspension / chassis)
     from its own blueprint, reusing motor_nx's hardened NXOpen engine -- each saved
     as its own .prt. (Skip with "nobuild" if the .prt files already exist.)
  2. ASSEMBLE: create the top vehicle part and Assemblies.AddComponent every part at
     the origin + orientation from the vehicle_nx.assembly PLAN.

No new geometry calls are introduced here; only Assemblies.AddComponent is added on
top of the proven motor_nx engine. AddComponent member names/overloads drift across
the NX 1900..2506 API, so the call is wrapped defensively (several known signatures)
-- CONFIRM on the next in-NX smoke run, like the manufacturing-assembly features.
"""

import json
import os
import sys
import traceback

import NXOpen
import NXOpen.Assemblies

# motor_nx.nx_builder imports NXOpen at load -> only importable inside NX (intended).
from motor_nx import nx_builder as eng

_SESSION = NXOpen.Session.GetSession()


# --------------------------------------------------------------------------- #
# stage 1 -- build each subsystem part from its blueprint
# --------------------------------------------------------------------------- #
def _refresh(pkg):
    """Drop a package's cached submodules so edits are picked up between runs (NX
    keeps one interpreter per session) -- mirrors motor_nx.nx_builder._default_blueprint."""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for _m in [m for m in list(sys.modules) if m == pkg or m.startswith(pkg + ".")]:
        del sys.modules[_m]


def _subsystem_blueprints():
    """(role -> (default-part-file-key, blueprint dict)) for every subsystem. Each
    blueprint layer is pure CPython (no NXOpen), so this runs in-session safely."""
    _refresh("motor_nx")
    _refresh("driveline_nx")
    _refresh("inverter_nx")
    _refresh("suspension_nx")
    _refresh("chassis_nx")
    from motor_nx import blueprint as m_bp
    from motor_nx.params import MotorParams
    from driveline_nx import blueprint as d_bp
    from driveline_nx.params import DrivelineParams
    from inverter_nx import blueprint as i_bp
    from inverter_nx.params import InverterParams
    from suspension_nx import blueprint as s_bp
    from suspension_nx.params import SuspensionParams
    from chassis_nx import blueprint as c_bp
    from chassis_nx.params import ChassisParams
    return {
        "motor": m_bp.generate(MotorParams()),
        "driveline": d_bp.generate(DrivelineParams()),
        "inverter": i_bp.generate(InverterParams()),
        "suspension": s_bp.generate(SuspensionParams()),
        "chassis": c_bp.generate(ChassisParams()),
    }


def _build_part(blueprint, out_path, log):
    """Build one subsystem blueprint into a fresh mm part, save and close it."""
    part = eng.new_mm_part(out_path, make_displayed=True)
    builder = eng.MotorBuilder(part, blueprint.get("stack_length", 200.0))
    builder.log("=== vehicle_nx subassembly: %s ===" % blueprint.get("name", "part"))
    builder.build(blueprint)
    eng._save(part)
    log("built subsystem part: %s (%d step error(s))" % (out_path, len(builder.errors)))
    try:
        part.Close(NXOpen.BasePart.CloseWholeTree.TrueValue,
                   NXOpen.BasePart.CloseModified.CloseModified, None)
    except Exception:
        pass  # leaving it open is harmless; AddComponent loads from the saved file
    return out_path


# --------------------------------------------------------------------------- #
# stage 2 -- create the assembly and add each component
# --------------------------------------------------------------------------- #
def _matrix3x3(orient):
    """NXOpen.Matrix3x3 from our row-major R (vehicle = R . local). The matrix's
    X/Y/Z axis vectors are the COLUMNS of R (image of each local axis)."""
    m = NXOpen.Matrix3x3()
    m.Xx, m.Xy, m.Xz = orient[0][0], orient[1][0], orient[2][0]
    m.Yx, m.Yy, m.Yz = orient[0][1], orient[1][1], orient[2][1]
    m.Zx, m.Zy, m.Zz = orient[0][2], orient[1][2], orient[2][2]
    return m


def _add_component(asm, part_path, name, origin, orient, layer, log):
    """Assemblies.ComponentAssembly.AddComponent with defensive handling of the
    signature drift across NX 1900..2506 (basePoint+Matrix3x3 [+ optional load
    status]). Returns the Component or None (non-fatal -- one missing part must not
    abort the whole vehicle)."""
    base = NXOpen.Point3d(float(origin[0]), float(origin[1]), float(origin[2]))
    mat = _matrix3x3(orient)
    last = None
    for args in (
        (part_path, "MODEL", name, base, mat, layer),                 # common 6-arg form
        (part_path, "Entire Part", name, base, mat, layer),           # alt reference set
    ):
        try:
            res = asm.AddComponent(*args)
            comp = res[0] if isinstance(res, tuple) else res
            log("added component %-16s <- %s" % (name, os.path.basename(part_path)))
            return comp
        except Exception as exc:
            last = exc
    log("SKIP component %s (%s): %s" % (name, os.path.basename(part_path), last))
    return None


def assemble(plan, out_prt, log):
    asm = eng._unpack_part(_SESSION.Parts.NewDisplay(out_prt, _mm_units()))
    work = _SESSION.Parts.Work
    ca = work.ComponentAssembly
    added = 0
    for c in plan["components"]:
        part_path = _resolve_part(c["part_file"])
        if not os.path.exists(part_path):
            log("MISSING part for %s: %s" % (c["name"], part_path))
            continue
        comp = _add_component(ca, part_path, c["name"], c["origin_mm"], c["orientation"], 1, log)
        if comp is not None:
            added += 1
    _SESSION.UpdateManager.DoUpdate(
        _SESSION.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "assemble"))
    eng._save(work)
    log("=== vehicle assembly: %d/%d components added -> %s ===" % (added, len(plan["components"]), out_prt))


def _mm_units():
    vals = eng._mm_unit_values()
    if not vals:
        raise RuntimeError("Millimeters part-units enum not available")
    return vals[0]


def _resolve_part(name):
    """Resolve a plan part_file to an absolute path next to this package."""
    if os.path.isabs(name):
        return name
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, name)


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def _log_window():
    lw = _SESSION.ListingWindow
    lw.Open()
    return lw


def main():
    plan = None
    out_prt = None
    do_build = True
    for a in sys.argv[1:]:
        al = a.lower()
        if al == "build":
            do_build = True
        elif al == "nobuild":
            do_build = False
        elif al.endswith(".json"):
            with open(a, "r") as fh:
                plan = json.load(fh)
        elif al.endswith(".prt"):
            out_prt = a
        else:
            out_prt = a + ".prt"
    here = os.path.dirname(os.path.abspath(__file__))
    if out_prt is None:
        out_prt = os.path.join(here, "vehicle_out.prt")
    if plan is None:
        _refresh("vehicle_nx")
        from vehicle_nx import assembly as _asm
        from vehicle_nx.params import VehicleParams
        plan = _asm.build_plan(VehicleParams())

    lw = _log_window()
    lw.WriteLine("=== vehicle_nx assembler ===")
    for issue in plan.get("validation", []):
        lw.WriteLine("VALIDATION: %s" % issue)

    # stage 1 -- build the subsystem parts (into this package dir, matching the plan)
    if do_build:
        try:
            blues = _subsystem_blueprints()
            wanted = {c["role"] for c in plan["components"]}
            role_to_file = {c["role"]: _resolve_part(c["part_file"]) for c in plan["components"]}
            for role, blue in blues.items():
                if role in wanted:
                    _build_part(blue, role_to_file[role], lw.WriteLine)
        except Exception:
            lw.WriteLine("SUBSYSTEM BUILD FAILED:\n" + traceback.format_exc())

    # stage 2 -- assemble
    try:
        assemble(plan, out_prt, lw.WriteLine)
    except Exception:
        lw.WriteLine("ASSEMBLY FAILED:\n" + traceback.format_exc())


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    main()
