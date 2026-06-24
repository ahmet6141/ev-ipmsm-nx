"""Siemens NX builder for the suspension corner. RUN INSIDE NX (headless) via
run_journal:

    "%UGII_ROOT_DIR%\\run_journal.exe" suspension_nx\\nx_builder.py -args [blueprint.json] [out.prt] [step|parasolid|both|none] [parts]

It REUSES motor_nx's hardened NXOpen engine (the only module that imports NXOpen):
the geometry vocabulary (tube/cylinder/extrude/revolve/hole) is identical, so this
journal just supplies the suspension blueprint and suspension body-naming / per-part
grouping. With NO blueprint argument the default EV corner is built.

Export DEFAULTS to "step" (AP242, reliable); "parts" also writes each component
(Knuckle, Lower_Arm, Upper_Arm, Toe_Link, Spring, Damper, Anti_Roll, Mounts) as
its own STEP for piece-by-piece production hand-off. See motor_nx/nx_builder.py
for the per-call NXOpen rationale; this file adds no new NXOpen calls.
"""

import json
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# motor_nx.nx_builder imports NXOpen at module load, so importing it here only
# succeeds inside NX (run_journal) -- exactly as intended for this journal.
from motor_nx import nx_builder as eng


# suspension material-role DISPLAY NAMES (set via Body.SetName so the FEA / CAM can
# select bodies by role). Keyed by the component grouping below.  EVERY component maps
# to a role so EVERY create body is named (the inspector needs names to read its report).
_SUSPENSION_ROLE_NAME = {
    "Knuckle_R": "KNUCKLE_R",
    "Knuckle_L": "KNUCKLE_L",
    "Lower_Arm": "LOWER_ARM",
    "Upper_Arm": "UPPER_ARM",
    "Toe_Link": "TOE_LINK",
    "Spring": "COIL_SPRING",
    "Damper": "DAMPER",
    "Anti_Roll": "ANTI_ROLL_BAR",
    "Ball_Joint": "BALL_JOINT",
    "Bushing": "BUSHING",
    "Bolt": "BOLT",
    "Mounts": "MOUNT",
}


def _suspension_component_of(step_id):
    """Group a build-step id into a manufacturable component (per-part STEP export +
    body NAMING -- every create body must get a name so the NX inspector can read the
    interference report).  Grouping is by id PREFIX; the corner tag (_l / _r) is the
    LAST token (so `lower_arm_hub_l` is left, `lower_arm_hub_r` right) -- using a bare
    "l in tokens" test wrongly tagged every id (e.g. 'lower') as left.

    Every redesign id is covered:
      * knuckle* / caliper_mount*                      -> Knuckle_<side>
      * lower_arm* / lower_fore* / lower_aft*           -> Lower_Arm  (arm + its pickup
        eyes; the inboard bushings/bolts are split out below by feature)
      * upper_arm* / upper_fore* / upper_aft*           -> Upper_Arm
      * toe_link* / toe_eye* / toe_in* / toe_out*       -> Toe_Link (+ its joint hardware)
      * spring* / damper*                               -> Spring / Damper (coil-over)
      * antiroll*                                       -> Anti_Roll (drop link + bracket)
      * *_bj_housing* / *_bj_stud*                      -> Ball_Joint
      * *_can* / *_sleeve*  (bushing parts)             -> Bushing
      * *_shank* / *_head* / *_nut*  (fastener parts)   -> Bolt"""
    base = step_id.split("#")[0]
    tokens = base.split("_")
    side = "_L" if tokens and tokens[-1] == "l" else "_R"   # corner tag = LAST token
    # feature-level groups first (so a fastener/bushing/ball-joint is named by feature,
    # not swallowed by the member prefix)
    if "_shank_" in base or "_head_" in base or "_nut_" in base:
        return "Bolt"
    if "_can_" in base or "_sleeve_" in base:
        return "Bushing"
    if "_bj_housing_" in base or "_bj_stud_" in base:
        return "Ball_Joint"
    # member groups
    if base.startswith("knuckle") or base.startswith("caliper_mount"):
        return "Knuckle" + side
    if base.startswith("lower_arm") or base.startswith("lower_fore") or base.startswith("lower_aft"):
        return "Lower_Arm"
    if base.startswith("upper_arm") or base.startswith("upper_fore") or base.startswith("upper_aft"):
        return "Upper_Arm"
    if base.startswith("toe"):            # toe_link, toe_eye, toe_in, toe_out
        return "Toe_Link"
    if base.startswith("spring"):
        return "Spring"
    if base.startswith("damper"):
        return "Damper"
    if base.startswith("antiroll"):       # drop link + eyes + arm bracket
        return "Anti_Roll"
    if "bushing" in base or "balljoint" in base:
        return "Mounts"
    return "Mounts"                       # safety net -- never leave a body unnamed


def _default_blueprint():
    """Generate the default EV suspension corner blueprint in-process (no NXOpen
    needed for the blueprint layer), refreshing the cached submodules like
    motor_nx / driveline_nx do."""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for _m in [m for m in list(sys.modules)
               if m == "suspension_nx" or m.startswith("suspension_nx.")]:
        del sys.modules[_m]
    from suspension_nx import blueprint as _bp
    from suspension_nx.params import SuspensionParams
    return _bp.generate(SuspensionParams())


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
        out_prt = os.path.join(os.path.dirname(os.path.abspath(__file__)), "suspension_out.prt")

    # Point the reused engine's role-name + component grouping at the suspension maps
    # (the methods read these as module globals at call time, so a clean swap).
    eng._ROLE_NAME = _SUSPENSION_ROLE_NAME
    eng._component_of = _suspension_component_of

    part = eng.new_mm_part(out_prt, make_displayed=True)
    builder = eng.MotorBuilder(part, blueprint.get("stack_length", 600.0),
                               use_nx_patterns=use_nx_patterns)
    builder.log("=== suspension_nx build: %s ===" % blueprint.get("name", "suspension"))
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
