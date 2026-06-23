"""Siemens NX builder for the chassis. RUN INSIDE NX (headless) via run_journal:

    "%UGII_ROOT_DIR%\\run_journal.exe" chassis_nx\\nx_builder.py -args [blueprint.json] [out.prt] [step|parasolid|both|none] [parts]

It REUSES motor_nx's hardened NXOpen engine (the only module that imports NXOpen):
the geometry vocabulary (tube/cylinder/extrude/revolve/hole) is identical, so this
journal just supplies the chassis blueprint and chassis body-naming / per-part
grouping. With NO blueprint argument the default EV skateboard chassis is built.

Export DEFAULTS to "step" (AP242, reliable); "parts" also writes each component
(Frame_Rails, Crossmembers, Battery_Tray, Subframe_Mounts, Crash_Structure) as its
own STEP for piece-by-piece production hand-off. See motor_nx/nx_builder.py for the
per-call NXOpen rationale; this file adds no new NXOpen calls.
"""

import json
import os
import sys
import traceback

# motor_nx.nx_builder imports NXOpen at module load, so importing it here only
# succeeds inside NX (run_journal) -- exactly as intended for this journal.
from motor_nx import nx_builder as eng


# chassis material-role DISPLAY NAMES (set via Body.SetName so the FEA / CAM can
# select bodies by role). Keyed by the component grouping below.
_CHASSIS_ROLE_NAME = {
    "Frame_Rails": "FRAME_RAIL",
    "Crossmembers": "CROSSMEMBER",
    "Battery_Tray": "BATTERY_TRAY",
    "Subframe_Mounts": "SUBFRAME_MOUNT",
    "Crash_Structure": "CRUSH_CAN",
}


def _chassis_component_of(step_id):
    """Group a build-step id into a manufacturable component (per-part STEP export
    + body naming). Mirrors driveline_nx.nx_builder._driveline_component_of."""
    base = step_id.split("#")[0]
    if base.startswith("rail_"):
        return "Frame_Rails"
    if base.startswith("crossmember_"):
        return "Crossmembers"
    if base.startswith("battery_"):
        return "Battery_Tray"
    if base.startswith("subframe_"):
        return "Subframe_Mounts"
    if base.startswith("body_mount_"):
        return "Frame_Rails"     # body-mount holes live on the rails
    if base.startswith("crush_"):
        return "Crash_Structure"
    return None


def _default_blueprint():
    """Generate the default EV chassis blueprint in-process (no NXOpen needed for
    the blueprint layer), refreshing the cached submodules like driveline_nx does."""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for _m in [m for m in list(sys.modules)
               if m == "chassis_nx" or m.startswith("chassis_nx.")]:
        del sys.modules[_m]
    from chassis_nx import blueprint as _bp
    from chassis_nx.params import ChassisParams
    return _bp.generate(ChassisParams())


def main():
    blueprint = None
    out_prt = None
    export_mode = "step"
    use_nx_patterns = False
    for a in sys.argv[1:]:
        al = a.lower()
        if al in ("step", "parasolid", "both", "none"):
            export_mode = al
        elif al in ("nxpatterns", "patterns"):
            use_nx_patterns = True
        elif al.endswith(".json"):
            with open(a, "r") as fh:
                blueprint = json.load(fh)
        elif al.endswith(".prt"):
            out_prt = a
        else:
            out_prt = a + ".prt"
    per_part = any(a.lower() == "parts" for a in sys.argv[1:])
    if blueprint is None:
        blueprint = _default_blueprint()
    if out_prt is None:
        out_prt = os.path.join(os.path.dirname(os.path.abspath(__file__)), "chassis_out.prt")

    # Point the reused engine's role-name + component grouping at the chassis maps
    # (the methods read these as module globals at call time, so a clean swap).
    eng._ROLE_NAME = _CHASSIS_ROLE_NAME
    eng._component_of = _chassis_component_of

    part = eng.new_mm_part(out_prt, make_displayed=True)
    builder = eng.MotorBuilder(part, blueprint.get("stack_length", 4690.0),
                               use_nx_patterns=use_nx_patterns)
    builder.log("=== chassis_nx build: %s ===" % blueprint.get("name", "chassis"))
    for issue in blueprint.get("validation", []):
        builder.log("VALIDATION: %s" % issue)

    try:
        builder.build(blueprint)
    except Exception:
        builder.log("BUILD ABORTED:\n" + traceback.format_exc())

    base = os.path.splitext(out_prt)[0]
    try:
        if export_mode in ("step", "both"):
            eng.export_step(part, base + "_ap242.stp", "ap242")
            builder.log("exported %s_ap242.stp" % base)
        if export_mode == "none":
            eng._save(part)
    except Exception:
        builder.log("STEP EXPORT FAILED:\n" + traceback.format_exc())
    if export_mode in ("parasolid", "both"):
        try:
            eng.export_parasolid(part, base + ".x_t")
            builder.log("exported %s.x_t" % base)
        except Exception as _pexc:
            builder.log("Parasolid (.x_t) export SKIPPED: %s" % _pexc)
            builder.log("  -> use the STEP (.stp) file; if NX threw a modeler fault, RESTART NX.")
    try:
        if per_part:
            builder.log("--- per-part (piece-by-piece) STEP export ---")
            builder.export_parts(base)
    except Exception:
        builder.log("PER-PART EXPORT FAILED:\n" + traceback.format_exc())

    builder.log("=== done (%d errors) ===" % len(builder.errors))


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    main()
