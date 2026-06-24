#!/usr/bin/env python
"""P1 EM-FEA verification driver -- Ansys Motor-CAD via PyMotorCAD.

Consumes the hand-off package produced by ``python -m motor_nx.cli fea`` (the
``fea/`` directory: ``fea_spec.json`` + ``winding.csv`` + ``femm_labels.csv``)
and drives a Motor-CAD E-Magnetic study end to end:

    geometry / winding / materials  ->  set_variable / set_winding_coil
    analysis matrix (fea_spec.analyses)  ->  cogging, back-EMF, torque-angle
            (MTPA sweep), torque ripple, demag, (optional) thermal + mechanical
    results  ->  compared against fea_spec.acceptance_targets, written to JSON

WHAT THIS SCRIPT DOES *NOT* DO
    It does not invent physics. Motor-CAD runs the FEA; this script only maps the
    frozen design onto Motor-CAD inputs, runs the matrix headless, reads the
    results back, and reports go / no-go against the acceptance gate. The numeric
    answers are Motor-CAD's, not ours.

PREREQUISITES
    pip install ansys-motorcad-core        # the PyMotorCAD client
    A licensed Motor-CAD install (the client launches / connects to it).

RUN
    # 1) regenerate the hand-off package if params changed:
    python -m motor_nx.cli fea -o fea/
    # 2) run the Motor-CAD study (headless; opens a Motor-CAD instance):
    python verification/motorcad_emag.py --spec fea/fea_spec.json --out fea/motorcad_results.json
    # selected stages only:
    python verification/motorcad_emag.py --stages cogging,back_emf,torque_angle,ripple,demag

IMPORTANT -- VARIABLE-NAME DRIFT (read this before the first run)
    Motor-CAD scripting variable names depend on the rotor TEMPLATE and the
    Motor-CAD VERSION. The slot/airgap/stack names below are stable; the V-magnet
    pocket names (bridge / web / V-angle / magnet bar width) differ between the
    "Interior V" rotor templates. Where a name may differ, this script (a) sets the
    BEST-KNOWN name, (b) logs whether the set/get succeeded, and (c) prints the
    TARGET VALUE from fea_spec so you can finish the field by hand if a name misses.
    To confirm any field's scripting name in your install: right-click the field in
    Motor-CAD -> "Copy variable name". Edit GEOM_VARS / the V-pocket block to match.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

# ansys.motorcad.core is the only external dependency; fail with a clear message.
try:
    import ansys.motorcad.core as pymotorcad
except Exception as _exc:  # pragma: no cover - depends on the user's environment
    print("ERROR: PyMotorCAD is not available (%s).\n"
          "       Install it with:  pip install ansys-motorcad-core\n"
          "       and ensure a licensed Motor-CAD is installed." % _exc)
    sys.exit(2)


# --------------------------------------------------------------------------- #
# small logged set/get wrappers -- one wrong variable name must not kill the run
# --------------------------------------------------------------------------- #
class MC:
    """Thin logged wrapper around the PyMotorCAD client. Every set/get is guarded
    so a single drifted variable name degrades to a warning (collected in .warn)
    instead of aborting the whole study."""

    def __init__(self, mc):
        self.mc = mc
        self.warn = []

    def set(self, name, value, target_note=""):
        try:
            self.mc.set_variable(name, value)
            return True
        except Exception as exc:
            msg = "set %-28s failed (%s)%s" % (
                name, exc, ("  TARGET=%s" % target_note) if target_note != "" else "")
            self.warn.append(msg)
            print("  WARN " + msg)
            return False

    def get(self, name, default=None):
        try:
            return self.mc.get_variable(name)
        except Exception as exc:
            self.warn.append("get %-28s failed (%s)" % (name, exc))
            return default

    def call(self, method, *args, **kwargs):
        """Call an arbitrary mc method by name, guarded."""
        fn = getattr(self.mc, method, None)
        if fn is None:
            self.warn.append("method %s not found in this PyMotorCAD version" % method)
            print("  WARN method %s not found" % method)
            return None
        try:
            return fn(*args, **kwargs)
        except Exception as exc:
            self.warn.append("%s%s failed (%s)" % (method, args, exc))
            print("  WARN %s%s failed (%s)" % (method, args, exc))
            return None


# --------------------------------------------------------------------------- #
# fea_spec.geometry_mm  ->  Motor-CAD scripting variables
#   STABLE names (left) are safe across templates/versions; confirm the V-pocket
#   block separately. value-lambda pulls the number from fea_spec.geometry_mm.
# --------------------------------------------------------------------------- #
GEOM_VARS = [
    # (motorcad_name,        spec_key,          unit/comment)
    ("Slot_Number",          "slots",           "stator slots"),
    ("Pole_Number",          "poles",           "rotor poles"),
    ("Stator_Lam_Dia",       "stator_OD",       "stator OD (mm)"),
    ("Stator_Bore",          "stator_bore",     "stator bore (mm)"),
    ("Airgap",               "air_gap",         "radial airgap (mm)"),
    ("Motor_Length",         "stack_length",    "active stack length (mm)"),
    ("Tooth_Width",          "tooth_width_mm",  "min tooth width (mm)"),
    ("Magnet_Thickness",     None,              "set from V-pocket block"),
    ("Shaft_Dia",            "shaft_OD",        "shaft / rotor-bore dia (mm)"),
]


def connect(open_new, keep_open, exe=None):
    print("Connecting to Motor-CAD ...")
    if exe:
        # PyMotorCAD auto-finds Motor-CAD via the MOTORCAD_ACTIVEX env var / registry;
        # if that is not set (common for a portable/custom install) point it explicitly.
        pymotorcad.set_motorcad_exe(exe)
        print("  using exe: %s" % exe)
    kwargs = {}
    if keep_open:
        kwargs["keep_instance_open"] = True
    if open_new:
        kwargs["open_new_instance"] = True
    mc_raw = pymotorcad.MotorCAD(**kwargs)
    mc = MC(mc_raw)
    # headless: suppress popups (also auto-accepts default dialog actions)
    mc.set("MessageDisplayState", 2)
    mc.call("show_magnetic_context")
    mc.set("Motor_Type", 0, "0 = BPM (brushless PM)")
    mc.call("display_screen", "Scripting")
    return mc


def load_topology(mc, template):
    """Load a single-layer V interior-PM rotor template, then it is reshaped by the
    geometry block. If you maintain your own .mot with the V-IPM topology, pass
    --template "" and --base-mot <file> instead."""
    if template:
        if mc.call("load_template", template) is None:
            print("  NOTE template '%s' did not load; assuming the open model already\n"
                  "       has the single-V interior-PM rotor topology." % template)


def apply_geometry(mc, spec):
    print("Geometry ...")
    geo = spec["geometry_mm"]
    for name, key, comment in GEOM_VARS:
        if key is None:
            continue
        if key in geo:
            mc.set(name, geo[key], "%s = %s" % (comment, geo[key]))

    # ----- V-magnet pocket block (CONFIRM THESE NAMES IN YOUR VERSION) ------- #
    # The single-V interior rotor is defined by: magnet thickness & bar width, the
    # V opening angle, the outer steel bridge and the d-axis web (center post). The
    # motor_nx design carries these as magnet_thickness / magnet_width / v_angle and
    # outer_bridge / center_post_halfwidth. fea_spec exposes bridge + half-web; the
    # magnet W x t and V-angle live in params (echoed here from geometry where present).
    vpkt = {
        "Magnet_Thickness":     _spec_get(spec, ["geometry_mm", "magnet_thickness_mm"], None),
        "Magnet_Bar_Width":     _spec_get(spec, ["geometry_mm", "magnet_width_mm"], None),
        "BridgeThickness":      geo.get("outer_bridge_mm"),
        "WebThickness":         _twice(geo.get("center_post_halfwidth_mm")),
        "PoleVAngle":           _spec_get(spec, ["geometry_mm", "v_angle_deg"], None),
    }
    print("  V-pocket (verify names; TARGET values from the frozen design):")
    for name, val in vpkt.items():
        if val is None:
            print("    %-20s (not in fea_spec; set by hand from params.py)" % name)
            continue
        mc.set(name, val, "%s" % val)


def apply_winding(mc, spec):
    print("Winding ...")
    w = spec["winding"]
    mc.set("MagPhases", w.get("phases", 3))
    mc.set("ParallelPaths", w.get("parallel_paths", 1))
    # hairpin: all bars in a slot share one belt -> set conductors/turns per slot.
    cps = w.get("conductors_per_slot")
    if cps is not None:
        mc.set("MagTurnsConductor", cps, "bars per slot")
    # Let Motor-CAD auto-build the integer-slot wave pattern, then VERIFY it.
    mc.set("MagneticWindingType", 0, "0 = auto wave/lap; switch to Custom only to force the map")
    verify_winding(mc, spec)


def verify_winding(mc, spec):
    """Compare Motor-CAD's auto winding factor to fea_spec's kw, and warn if the
    belt order / parallel-path transposition disagrees with slot_phase_map. The plan
    requires forcing the pattern with set_winding_coil() if it differs."""
    kw_spec = spec["winding"].get("winding_factor_kw")
    kw_mc = mc.get("WindingFactor", None) or mc.get("kw", None)
    if kw_spec is not None and kw_mc is not None:
        ok = abs(float(kw_mc) - float(kw_spec)) < 0.01
        print("  winding factor: Motor-CAD %.4f vs fea_spec %.4f -> %s"
              % (float(kw_mc), float(kw_spec), "OK" if ok else "MISMATCH (force with set_winding_coil)"))
        if not ok:
            mc.warn.append("winding factor mismatch %.4f vs %.4f" % (float(kw_mc), float(kw_spec)))
    else:
        print("  winding factor: could not read Motor-CAD kw; verify the auto pattern\n"
              "    against fea/winding.csv (slot_phase_map) manually before solving.")


def apply_materials(mc, spec):
    print("Materials ...")
    m = spec["materials"]
    lam_grade = "M250-35A"  # nearest standard DB grade to the 0.27 mm M250-27 class
    mc.call("set_component_material", "Stator Lam (Back Iron)", lam_grade)
    mc.call("set_component_material", "Rotor Lam (Back Iron)", lam_grade)
    mag = m.get("magnet", {})
    # Magnet: pick the closest DB grade, or define a custom one with the spec Br/Hc.
    # set_component_material expects a DB name; the SH-class NdFeB Br/Hc + temp coeffs
    # below are the TARGET values to confirm (or create as a custom magnet material).
    print("  magnet TARGET: %s  Br(20C)=%s T  Hcj=%s kA/m  Br_tc=%s %%/C  max=%s C"
          % (mag.get("grade"), mag.get("Br_T_at_20C"), mag.get("Hcj_kA_per_m"),
             mag.get("Br_tempco_pct_per_C"), mag.get("max_service_C")))
    mc.call("set_component_material", "Magnet", str(mag.get("grade", "N42SH")))


# --------------------------------------------------------------------------- #
# operating point + the fea_spec.analyses matrix
# --------------------------------------------------------------------------- #
def set_operating_point(mc, speed_rpm, peak_current_a, phase_adv_deg, dc_bus_v,
                        magnet_temp_c=None, winding_temp_c=None):
    mc.set("Shaft_Speed_[RPM]", speed_rpm)
    mc.set("CurrentDefinition", 0, "0 = Peak")
    mc.set("PeakCurrent", peak_current_a)
    mc.set("DCBusVoltage", dc_bus_v)
    mc.set("PhaseAdvance", phase_adv_deg)
    if magnet_temp_c is not None:
        mc.set("Magnet_Temperature", magnet_temp_c)
    if winding_temp_c is not None:
        mc.set("ArmatureConductor_Temperature", winding_temp_c)


def _only(mc, **flags):
    """Enable exactly the listed performance tests, disable the rest."""
    all_tests = ["TorqueCalculation", "BackEMFCalculation", "CoggingTorqueCalculation",
                 "TorqueSpeedCalculation", "DemagnetizationCalc", "InductanceCalc",
                 "ElectromagneticForcesCalc_OC", "ElectromagneticForcesCalc_Load",
                 "BPMShortCircuitCalc"]
    for t in all_tests:
        mc.set(t, t in flags and flags[t])


def _peak_to_peak(series):
    return (max(series) - min(series)) if series else 0.0


def stage_cogging(mc, spec, results):
    print("[cogging] no-current torque vs rotor angle ...")
    e = spec["excitation"]; op = spec["operating_points"]
    set_operating_point(mc, op.get("base_speed_rpm", 1000), 0.0, 0.0, op.get("dc_bus_V", 400))
    _only(mc, CoggingTorqueCalculation=True)
    mc.set("TorquePointsPerCycle", 60); mc.set("TorqueNumberCycles", 1)
    mc.call("do_magnetic_calculation")
    _, cog = _graph(mc, "CoggingTorque")
    cog_pp = _peak_to_peak(cog)
    results["cogging"] = {"pk_pk_Nm": cog_pp, "n_points": len(cog)}
    print("  cogging pk-pk = %.3f Nm" % cog_pp)


def stage_back_emf(mc, spec, results):
    print("[back_emf] open-circuit flux linkage / back-EMF ...")
    op = spec["operating_points"]
    set_operating_point(mc, op.get("base_speed_rpm", 1000), 0.0, 0.0, op.get("dc_bus_V", 400))
    _only(mc, BackEMFCalculation=True)
    mc.call("do_magnetic_calculation")
    ll_peak = mc.get("PeakLineLineVoltage")
    orders, amps = _harmonics(mc, "BackEMFPh1") or _harmonics(mc, "BackEMF") or ([], [])
    thd = _thd(orders, amps)
    results["back_emf"] = {"line_line_peak_V": ll_peak, "thd_pct": thd}
    print("  back-EMF L-L peak = %s V, THD ~ %s %%"
          % (_fmt(ll_peak), _fmt(thd)))


def stage_torque_angle(mc, spec, results):
    """Sweep current advance (beta) at PEAK current to locate MTPA -> peak torque."""
    print("[torque_angle] MTPA sweep at peak current ...")
    e = spec["excitation"]; op = spec["operating_points"]
    betas = e.get("current_advance_angle_deg_from_q_axis", [0, 10, 20, 30, 40])
    ipk = _peak_amp(e.get("peak_current_A_rms"))
    speed = op.get("base_speed_rpm", 1000)
    _only(mc, TorqueCalculation=True)
    mc.set("TorquePointsPerCycle", 30); mc.set("TorqueNumberCycles", 1)
    locus = []
    for beta in betas:
        set_operating_point(mc, speed, ipk, beta, op.get("dc_bus_V", 400))
        mc.call("do_magnetic_calculation")
        tq = mc.get("ShaftTorque")
        locus.append({"beta_deg": beta, "avg_torque_Nm": tq})
        print("    beta=%-4s  T=%s Nm" % (beta, _fmt(tq)))
    valid = [pt for pt in locus if isinstance(pt["avg_torque_Nm"], (int, float))]
    best = max(valid, key=lambda pt: pt["avg_torque_Nm"]) if valid else None
    results["torque_angle"] = {"peak_current_A_peak": ipk, "locus": locus,
                               "mtpa": best}
    if best:
        print("  MTPA: beta=%s deg -> peak torque %.1f Nm"
              % (best["beta_deg"], best["avg_torque_Nm"]))
    return best


def stage_ripple(mc, spec, results, mtpa):
    print("[ripple] instantaneous torque at MTPA ...")
    op = spec["operating_points"]; e = spec["excitation"]
    beta = (mtpa or {}).get("beta_deg", 15)
    ipk = _peak_amp(e.get("peak_current_A_rms"))
    set_operating_point(mc, op.get("base_speed_rpm", 1000), ipk, beta, op.get("dc_bus_V", 400))
    _only(mc, TorqueCalculation=True)
    mc.set("TorquePointsPerCycle", 90); mc.set("TorqueNumberCycles", 1)
    mc.call("do_magnetic_calculation")
    pos, tq = _graph(mc, "TorqueVW")
    avg = (sum(tq) / len(tq)) if tq else 0.0
    ripple_pct = (_peak_to_peak(tq) / avg * 100.0) if avg else 0.0
    results["ripple"] = {"avg_Nm": avg, "pk_pk_Nm": _peak_to_peak(tq),
                         "ripple_pct": ripple_pct}
    print("  avg %.1f Nm, ripple %.2f %%" % (avg, ripple_pct))


def stage_demag(mc, spec, results):
    """Worst-case demag: peak current opposing the magnets at the hot magnet temp."""
    print("[demag] peak current at hot magnet temperature ...")
    op = spec["operating_points"]; e = spec["excitation"]
    mag = spec["materials"]["magnet"]
    hot = mag.get("max_service_C", 150)
    ipk = _peak_amp(e.get("peak_current_A_rms"))
    # d-axis-opposing: beta = 90 deg from q-axis is the most demagnetising direction.
    set_operating_point(mc, op.get("base_speed_rpm", 1000), ipk, 90.0,
                        op.get("dc_bus_V", 400), magnet_temp_c=hot)
    _only(mc, DemagnetizationCalc=True)
    mc.call("do_magnetic_calculation")
    # Motor-CAD reports a demagnetisation proportion / min working point; read both.
    demag_pct = mc.get("DemagnetisationProportion") or mc.get("Demag_Proportion")
    min_b = mc.get("MinMagnetFluxDensity")
    safe = (demag_pct in (0, 0.0, None)) if demag_pct is not None else None
    results["demag"] = {"magnet_temp_C": hot, "demag_proportion": demag_pct,
                        "min_magnet_B_T": min_b, "irreversible": (not safe) if safe is not None else None}
    print("  hot=%s C, demag proportion=%s, min magnet B=%s T"
          % (hot, _fmt(demag_pct), _fmt(min_b)))


def stage_thermal(mc, spec, results):
    """P2 seed: water-jacket steady-state at the continuous point (optional)."""
    print("[thermal] steady-state continuous (jacket) ...")
    mc.call("show_thermal_context")
    op = spec["operating_points"]
    set_operating_point(mc, op.get("base_speed_rpm", 1000),
                        _peak_amp(spec["excitation"].get("rated_current_A_rms")),
                        15.0, op.get("dc_bus_V", 400))
    ran = mc.call("do_steady_state_analysis") or mc.call("do_thermal_calculation")
    t_wind = mc.get("T_[Winding_Max]") or mc.get("Winding_Temperature_Max")
    t_mag = mc.get("T_[Magnet]") or mc.get("Magnet_Temperature")
    results["thermal"] = {"winding_hotspot_C": t_wind, "magnet_C": t_mag}
    print("  winding hotspot=%s C, magnet=%s C" % (_fmt(t_wind), _fmt(t_mag)))
    mc.call("show_magnetic_context")


def stage_mechanical(mc, spec, results, overspeed_factor=1.2):
    """P3 seed: rotor centrifugal stress at 1.2x max speed (optional)."""
    print("[mechanical] rotor stress at overspeed ...")
    op = spec["operating_points"]
    rpm = overspeed_factor * op.get("max_speed_rpm", 18000)
    mc.call("show_mechanical_context")
    mc.set("ShaftSpeed", rpm); mc.set("Shaft_Speed_[RPM]", rpm)
    mc.call("do_mechanical_calculation")
    smax = mc.get("MaxStress_RotorLam")
    syield = mc.get("YieldStress_RotorLam")
    sf = (float(syield) / float(smax)) if (smax and syield) else None
    results["mechanical"] = {"overspeed_rpm": rpm, "max_stress_MPa": smax,
                             "yield_MPa": syield, "safety_factor": sf}
    print("  %.0f rpm: max von Mises=%s MPa, SF=%s" % (rpm, _fmt(smax), _fmt(sf)))
    mc.call("show_magnetic_context")


# --------------------------------------------------------------------------- #
# acceptance gate
# --------------------------------------------------------------------------- #
def evaluate(spec, results):
    t = spec["acceptance_targets"]
    rated = spec["operating_points"].get("continuous_torque_Nm", 0) or 1
    checks = []

    def chk(name, ok, detail):
        checks.append({"criterion": name, "pass": bool(ok) if ok is not None else None, "detail": detail})

    mtpa = (results.get("torque_angle") or {}).get("mtpa") or {}
    peak_tq = mtpa.get("avg_torque_Nm")
    if peak_tq is not None:
        chk("peak_torque >= %s Nm" % t["peak_torque_Nm_min"],
            peak_tq >= t["peak_torque_Nm_min"], "MTPA peak = %.1f Nm" % peak_tq)

    rip = (results.get("ripple") or {}).get("ripple_pct")
    if rip is not None:
        chk("torque_ripple <= %s %%" % t["torque_ripple_pct_max"],
            rip <= t["torque_ripple_pct_max"], "ripple = %.2f %%" % rip)

    cog = (results.get("cogging") or {}).get("pk_pk_Nm")
    if cog is not None:
        cog_pct = cog / rated * 100.0
        chk("cogging <= %s %% of rated" % t["cogging_pct_of_rated_max"],
            cog_pct <= t["cogging_pct_of_rated_max"],
            "cogging = %.3f Nm = %.2f %% of %.0f Nm" % (cog, cog_pct, rated))

    dem = results.get("demag") or {}
    if dem.get("irreversible") is not None:
        chk("no demag @ peak + %s C" % dem.get("magnet_temp_C"),
            not dem["irreversible"], "demag proportion = %s" % _fmt(dem.get("demag_proportion")))

    mech = results.get("mechanical") or {}
    if mech.get("safety_factor") is not None:
        chk("rotor SF >= %s @ 1.2x max" % t["rotor_vonMises_safety_factor_min_at_1.2x_maxspeed"],
            mech["safety_factor"] >= t["rotor_vonMises_safety_factor_min_at_1.2x_maxspeed"],
            "SF = %.2f" % mech["safety_factor"])

    return checks


# --------------------------------------------------------------------------- #
# graph / harmonic / formatting helpers (tolerant of version differences)
# --------------------------------------------------------------------------- #
def _graph(mc, name):
    """Return (x_list, y_list) for a magnetic graph; tolerant of the two APIs."""
    res = mc.call("get_magnetic_graph", name)
    if isinstance(res, (list, tuple)) and len(res) == 2:
        return list(res[0]), list(res[1])
    # fall back to point-by-point
    xs, ys = [], []
    n = 0
    while n < 4000:
        pt = mc.call("get_magnetic_graph_point", name, n)
        if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
            break
        xs.append(pt[0]); ys.append(pt[1]); n += 1
    return xs, ys


def _harmonics(mc, name):
    res = mc.call("get_magnetic_graph_harmonics", name)
    if isinstance(res, (list, tuple)) and len(res) >= 2:
        return list(res[0]), list(res[1])
    return None


def _thd(orders, amps):
    if not orders or not amps:
        return None
    fund = None
    rest_sq = 0.0
    for o, a in zip(orders, amps):
        if round(o) == 1:
            fund = a
        elif round(o) >= 2:
            rest_sq += a * a
    if not fund:
        return None
    return math.sqrt(rest_sq) / abs(fund) * 100.0


def _peak_amp(rms):
    return (float(rms) * math.sqrt(2.0)) if isinstance(rms, (int, float)) else rms


def _twice(x):
    return (2.0 * x) if isinstance(x, (int, float)) else x


def _spec_get(spec, path, default):
    cur = spec
    for k in path:
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        else:
            return default
    return cur


def _fmt(x):
    return ("%.3f" % x) if isinstance(x, (int, float)) else str(x)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
ALL_STAGES = ["cogging", "back_emf", "torque_angle", "ripple", "demag", "thermal", "mechanical"]


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    ap = argparse.ArgumentParser(description="P1 EM-FEA verification in Motor-CAD (PyMotorCAD)")
    ap.add_argument("--spec", default=os.path.join(repo, "fea", "fea_spec.json"),
                    help="path to fea_spec.json (default: fea/fea_spec.json)")
    ap.add_argument("--out", default=os.path.join(repo, "fea", "motorcad_results.json"))
    ap.add_argument("--stages", default="cogging,back_emf,torque_angle,ripple,demag",
                    help="comma list from: %s" % ",".join(ALL_STAGES))
    ap.add_argument("--template", default="e9",
                    help="Motor-CAD rotor template to start from (\"\" to skip)")
    ap.add_argument("--keep-open", action="store_true", help="keep the Motor-CAD instance open")
    ap.add_argument("--new-instance", action="store_true", help="force a new Motor-CAD instance")
    ap.add_argument("--exe", default=os.environ.get("MOTORCAD_EXE", ""),
                    help="path to the Motor-CAD .exe (or set MOTORCAD_EXE) if PyMotorCAD can't auto-find it")
    args = ap.parse_args(argv)

    with open(args.spec, "r", encoding="utf-8") as fh:
        spec = json.load(fh)
    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    print("=== Motor-CAD EM verification: %s ===" % spec.get("name", "motor"))
    print("spec: %s   stages: %s\n" % (args.spec, ", ".join(stages)))

    t0 = time.time()
    mc = connect(args.new_instance, args.keep_open, args.exe or None)
    load_topology(mc, args.template)
    apply_geometry(mc, spec)
    apply_winding(mc, spec)
    apply_materials(mc, spec)
    mc.call("save_to_file", os.path.join(os.path.dirname(args.out), "motorcad_model.mot"))

    results = {"design": spec.get("name"), "spec_path": args.spec}
    mtpa = None
    if "cogging" in stages:
        stage_cogging(mc, spec, results)
    if "back_emf" in stages:
        stage_back_emf(mc, spec, results)
    if "torque_angle" in stages:
        mtpa = stage_torque_angle(mc, spec, results)
    if "ripple" in stages:
        stage_ripple(mc, spec, results, mtpa)
    if "demag" in stages:
        stage_demag(mc, spec, results)
    if "thermal" in stages:
        stage_thermal(mc, spec, results)
    if "mechanical" in stages:
        stage_mechanical(mc, spec, results)

    checks = evaluate(spec, results)
    results["acceptance"] = checks
    results["warnings"] = mc.warn
    results["elapsed_s"] = round(time.time() - t0, 1)

    print("\n--- ACCEPTANCE (G1 gate, vs fea_spec.acceptance_targets) ---")
    for c in checks:
        flag = {True: "PASS", False: "FAIL", None: "n/a "}[c["pass"]]
        print("  [%s] %-34s %s" % (flag, c["criterion"], c["detail"]))
    if mc.warn:
        print("\n  %d variable/graph warning(s) -- confirm those names in your Motor-CAD version." % len(mc.warn))

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2)
    print("\nresults -> %s   (%.1fs)" % (args.out, results["elapsed_s"]))

    if not args.keep_open:
        mc.call("set_variable", "MessageDisplayState", 0)
        mc.call("quit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
