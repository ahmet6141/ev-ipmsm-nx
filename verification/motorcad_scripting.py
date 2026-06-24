#!/usr/bin/env python
"""Run the P1 EM-FEA matrix from INSIDE Motor-CAD's Scripting tab.

WHY THIS FILE
    The EXTERNAL PyMotorCAD path (motorcad_emag.py, launched from a normal Python)
    returned a license-checkout error (-8 "Invalid (inconsistent)") on this machine.
    Motor-CAD's *internal* Scripting tab runs inside the already-open, GUI-licensed
    instance, so it may use the license through a different path.  This script attaches
    to the CURRENT instance and reuses the exact geometry / analysis / scoring logic
    from verification/motorcad_emag.py -- nothing is duplicated.

HOW TO RUN (inside Motor-CAD)
    1. Open Motor-CAD, load an E-Magnetic model with a single-V interior-PM rotor
       (e.g. template "e9"), or leave SET_TEMPLATE=True below to load it for you.
    2. Open the **Scripting** tab.
    3. Edit REPO below if your repo is not at the default path.
    4. Scripting tab -> **Run** (or "Run Script" and pick this file).
    Results -> fea/motorcad_results.json, scored vs fea_spec.acceptance_targets, and
    a PASS/FAIL summary is printed to the Scripting output.

IF IT STILL FAILS
    If you still see "Unable to check out feature: motorcad / Invalid (inconsistent)",
    the license is being rejected at CHECKOUT (cryptographic signature) -- that is not a
    scripting problem and only a valid .lic from Ansys will fix it.  In that case use the
    free FEMM driver (verification/femm_emag.py) instead.
"""
import os
import sys

# --- edit these if needed --------------------------------------------------- #
REPO = r"C:\Users\ahmet\Desktop\web\motor"     # repo root (contains fea/ and verification/)
SET_TEMPLATE = False                            # True -> load the "e9" V-IPM template first
TEMPLATE = "e9"
STAGES = ["cogging", "back_emf", "torque_angle", "ripple", "demag"]
# ---------------------------------------------------------------------------- #

sys.path.insert(0, REPO)
import json                                              # noqa: E402
import ansys.motorcad.core as pymotorcad                 # noqa: E402
from verification import motorcad_emag as mce            # noqa: E402

SPEC = os.path.join(REPO, "fea", "fea_spec.json")
OUT = os.path.join(REPO, "fea", "motorcad_results.json")


def run():
    with open(SPEC, "r", encoding="utf-8") as fh:
        spec = json.load(fh)
    print("=== Motor-CAD EM verification (INTERNAL scripting): %s ===" % spec.get("name"))
    print("spec: %s\nstages: %s\n" % (SPEC, ", ".join(STAGES)))

    # attach to THIS running, GUI-open (already-licensed) instance -- no new instance
    mc_raw = pymotorcad.MotorCAD()
    mc = mce.MC(mc_raw)
    mc.set("MessageDisplayState", 2)
    mc.call("show_magnetic_context")
    mc.set("Motor_Type", 0, "0 = BPM (brushless PM)")
    if SET_TEMPLATE:
        mce.load_topology(mc, TEMPLATE)

    # --- check the license actually checks out before the long matrix -------- #
    if not mc.set("Slot_Number", spec["geometry_mm"]["slots"], "license probe"):
        print("\n*** Motor-CAD refused the first scripted command (license checkout). ***")
        print("    This is the -8 'Invalid (inconsistent)' license rejection -- not a")
        print("    scripting fault.  Use the free FEMM driver instead:")
        print("      python verification/femm_emag.py --out fea/femm_results.json")
        return

    mce.apply_geometry(mc, spec)
    mce.apply_winding(mc, spec)
    mce.apply_materials(mc, spec)

    results = {"design": spec.get("name"), "spec_path": SPEC, "mode": "internal scripting"}
    mtpa = None
    if "cogging" in STAGES:
        mce.stage_cogging(mc, spec, results)
    if "back_emf" in STAGES:
        mce.stage_back_emf(mc, spec, results)
    if "torque_angle" in STAGES:
        mtpa = mce.stage_torque_angle(mc, spec, results)
    if "ripple" in STAGES:
        mce.stage_ripple(mc, spec, results, mtpa)
    if "demag" in STAGES:
        mce.stage_demag(mc, spec, results)

    checks = mce.evaluate(spec, results)
    results["acceptance"] = checks
    results["warnings"] = mc.warn

    print("\n--- ACCEPTANCE (G1 gate, vs fea_spec.acceptance_targets) ---")
    for c in checks:
        flag = {True: "PASS", False: "FAIL", None: "n/a "}[c["pass"]]
        print("  [%s] %-34s %s" % (flag, c["criterion"], c["detail"]))
    if mc.warn:
        print("\n  %d variable/graph warning(s) -- confirm those names in your version."
              % len(mc.warn))

    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2)
    print("\nresults -> %s" % OUT)


# Motor-CAD's "Run Script" executes the file top-to-bottom; this call drives it.
run()
