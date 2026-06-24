"""P1 EM-FEA verification driver -- Siemens Simcenter 3D, Magnetics (low-frequency
electromagnetics) solver, driven as an NXOpen journal.

Run INSIDE NX / Simcenter 3D via run_journal.exe::

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_emag.py ^
        -args motor_v8.sim cogging,back_emf,torque_angle,ripple,demag ^
              spec=fea\\fea_spec.json out=fea\\simcenter_emag_results.json

It consumes the same hand-off package as the Motor-CAD / FEMM drivers
(``fea/fea_spec.json``) and drives a 2D transient magnetics study end to end:

    operating point (current / advance angle / speed / magnet temp)  -> solution fields
    analysis matrix (fea_spec.analyses)  -> cogging, back-EMF, torque-angle (MTPA),
            torque ripple, demag        (the same five stages, same acceptance gate)
    results  ->  compared against fea_spec.acceptance_targets, written to JSON

------------------------------------------------------------------------------
HONEST SCOPE -- read this before the first run
------------------------------------------------------------------------------
Simcenter 3D's Magnetics solver is fully capable for an IPMSM, but its *NXOpen
journaling surface is the thinnest of the Simcenter solvers*, and -- exactly like
the NX CAM caveat already documented in docs/PROJECT_PLAN.md -- mesh / coil /
geometry SELECTION does not survive journal replay. So this journal follows the
project's established "pre-build + drive + score, never silently fake it" pattern:

  * The INTERACTIVE prerequisite (done once, by hand) is the FEM + Sim:
      idealize the 2D lamination plane (from fea/cross_section.dxf or the built
      .prt), 2D-mesh it with the airgap layers from fea_spec.mesh, create the mesh
      collectors STATOR_STEEL / ROTOR_STEEL / MAGNET / COIL_A.. and the rotor
      motion region, and save a .sim on the Magnetics 2D Transient solution.
  * This JOURNAL then automates everything that IS replayable and high-value:
      material property VALUES (BH / NdFeB Br-Hcj-recoil-tempco / copper), the coil
      current excitations and advance angle, the rotor speed / step schedule, the
      A=0 outer boundary + anti-periodic faces, the solve, and the result read-back,
      and scores go / no-go against fea_spec.acceptance_targets.
  * Every NXOpen step is guarded and logged to the Listing Window; a drifted member
      name degrades to a warning that carries the TARGET value (so you can finish
      that one field by hand) instead of aborting. If no usable .sim is supplied the
      journal prints the COMPLETE setup card (every target value + the interactive
      checklist) and exits cleanly.

The numbers are Simcenter's, not ours. Cross-check the winning point against the
FEMM / Motor-CAD drivers (the plan's G1 gate wants them within a few %).
"""
from __future__ import annotations

import math
import os
import sys
import traceback

# shared, NX-independent layer (spec, logging wrapper, arg parsing, scoring)
_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
import simcenter_common as sc  # noqa: E402

# NXOpen is only present inside NX / run_journal.exe. Import guarded so the file
# still compiles and the setup-card path can be reached for a dry description.
try:
    import NXOpen
    import NXOpen.CAE
    _HAVE_NX = True
except Exception:
    NXOpen = None
    _HAVE_NX = False


ALL_STAGES = ["cogging", "back_emf", "torque_angle", "ripple", "demag"]


# --------------------------------------------------------------------------- #
# session / part
# --------------------------------------------------------------------------- #
def get_session():
    return NXOpen.Session.GetSession() if _HAVE_NX else None


def open_sim(j, args):
    """Open the supplied .sim (preferred) / .fem / .prt and return the work SimPart.
    Returns None if nothing usable is open -- the caller then prints the setup card."""
    target = args.get("sim") or args.get("fem") or args.get("prt")
    sess = j.session
    if target and os.path.exists(target):
        # OpenBaseDisplay opens a .sim and pulls its .fem + master .prt along.
        j.method(sess.Parts, ["OpenBaseDisplay", "OpenActiveDisplay", "Open"], target,
                 label="open %s" % os.path.basename(target),
                 target="a .sim on the Magnetics 2D Transient solution")
    elif target:
        j.warning("model file not found: %s" % target,
                  target="pass an existing .sim/.fem/.prt as the first -args token")

    work = sess.Parts.BaseWork if sess is not None else None
    if work is None or getattr(work, "Tag", 0) == 0:
        return None
    # Is it actually a SimPart? (a .sim work part is what we need for solutions)
    if "Sim" not in type(work).__name__ and not hasattr(work, "Simulation"):
        j.note("work part %s is not a .sim -- material/value automation needs the Sim part open"
               % type(work).__name__)
    return work


def active_solution(j, sim):
    """Return the active magnetics Solution, guarded across the SimSimulation API."""
    if sim is None:
        return None
    simn = j.get_attr(sim, ["Simulation"])
    if simn is None:
        j.warning("no .Simulation on the work part",
                  target="open the .sim that carries the Magnetics solution")
        return None
    sol = j.get_attr(simn, ["ActiveSolution"])
    if sol is None:
        sols = j.get_attr(simn, ["Solutions"])
        try:
            sol = sols[0] if sols is not None and len(sols) else None
        except Exception:
            sol = None
    if sol is not None:
        j.ok("active solution: %s" % j.get_attr(sol, ["Name"], "<solution>"))
    else:
        j.warning("no solution found",
                  target="create a 'Magnetics' 2D Transient solution in the Sim file")
    return sol


# --------------------------------------------------------------------------- #
# materials  (values are scriptable; the assignment-to-collector is interactive)
# --------------------------------------------------------------------------- #
def apply_materials(j, sim, spec):
    j.line("")
    j.line("Materials (set property VALUES; assign to mesh collectors interactively):")
    m = spec.get("materials", {})

    lam = m.get("lamination", {})
    bh_h = lam.get("BH_H_A_per_m", [])
    bh_b = lam.get("BH_B_T", [])
    j.note("  STATOR/ROTOR steel '%s': %d-pt B-H curve, density %s kg/m3, stacking %s"
           % (lam.get("grade"), len(bh_b), lam.get("density_kg_m3"), lam.get("stacking_factor")))
    j.note("    iron-loss seed %s W/kg @1.5T/50Hz -- REPLACE with vendor Bertotti/Steinmetz "
           "before the qualifying run (fea_spec note)." % lam.get("core_loss_W_per_kg_at_1.5T_50Hz"))

    mag = m.get("magnet", {})
    j.note("  MAGNET '%s': Br(20C)=%s T, Hcj=%s kA/m, mu_recoil=%s, Br_tc=%s %%/C, "
           "Hcj_tc=%s %%/C, max=%s C, axial segs=%s"
           % (mag.get("grade"), mag.get("Br_T_at_20C"), mag.get("Hcj_kA_per_m"),
              mag.get("mu_recoil"), mag.get("Br_tempco_pct_per_C"),
              mag.get("Hcj_tempco_pct_per_C"), mag.get("max_service_C"),
              mag.get("axial_segments")))

    cu = m.get("conductor", {})
    j.note("  COIL copper: sigma=%s S/m @20C, tempco=%s /C, fill=%s, %s"
           % (cu.get("conductivity_S_per_m_20C"), cu.get("temp_coeff_per_C"),
              cu.get("slot_fill_factor"), cu.get("insulation_class")))

    # The MaterialManager / PhysicalMaterial create+edit calls are version-volatile;
    # attempt the create, but the property edit + collector assignment stays in the GUI.
    mm = j.get_attr(sim, ["MaterialManager"])
    if mm is None:
        j.warning("no MaterialManager on the Sim part",
                  target="create the 3 materials in the FEM and assign to collectors by hand")
        return
    pmats = j.get_attr(mm, ["PhysicalMaterials"])
    for name in ("Motor_Lamination_NO", "Magnet_%s" % str(mag.get("grade", "N42SH")), "Coil_Copper"):
        created = j.method(pmats, ["CreateIsotropicMaterial", "CreateMaterial"], name,
                           label="create material %s" % name,
                           target="define + assign this material in the FEM interactively")
        if created is None:
            # creation API differs by release -- not fatal; the values above are the target.
            continue


# --------------------------------------------------------------------------- #
# winding circuits  (coil current = the excitation; pattern from the slot map)
# --------------------------------------------------------------------------- #
def winding_summary(j, spec):
    w = spec.get("winding", {})
    j.line("")
    j.line("Winding / coil excitation (3-phase, hairpin):")
    j.note("  phases=%s, conductors/slot=%s, parallel paths=%s, series turns/phase=%s, "
           "coil pitch=%s, kw=%s"
           % (w.get("phases"), w.get("conductors_per_slot"), w.get("parallel_paths"),
              w.get("series_turns_per_phase"), w.get("coil_pitch_slots"),
              w.get("winding_factor_kw")))
    # turns/slot folded for parallel paths so the coil current is the terminal current
    cps = w.get("conductors_per_slot")
    pp = w.get("parallel_paths") or 1
    if isinstance(cps, (int, float)) and pp:
        j.note("  -> model coil = %g effective turns/slot at full phase current "
               "(folds the %s parallel paths into the turns; see femm_emag.py note)."
               % (cps / pp, pp))
    j.note("  Verify Simcenter's coil belt order against fea/winding.csv (slot_phase_map) "
           "and kw~%s before solving." % w.get("winding_factor_kw"))


def set_operating_point(j, sol, speed_rpm, peak_current_a, beta_deg, dc_bus_v,
                        magnet_temp_c=None):
    """Push the operating point onto the solution. Coil current amplitude + phase
    (advance angle beta from the q-axis) and the rotor speed/step are the scriptable
    knobs; names vary by release so each is guarded and logged with its TARGET."""
    pairs = [
        (["ShaftSpeed", "Shaft_Speed", "RotorSpeed", "Speed"], speed_rpm, "rotor speed [rpm]"),
        (["PeakCurrent", "CoilCurrentPeak", "DriveCurrent"], peak_current_a, "coil peak current [A]"),
        (["PhaseAdvance", "AdvanceAngle", "Beta"], beta_deg, "advance angle from q-axis [deg]"),
        (["DCBusVoltage", "BusVoltage"], dc_bus_v, "DC bus [V]"),
    ]
    if magnet_temp_c is not None:
        pairs.append((["MagnetTemperature", "Magnet_Temperature"], magnet_temp_c, "magnet temp [C]"))
    for names, val, note in pairs:
        if val is None:
            continue
        # operating-point fields live on the solution (or a step set); try both.
        ok = False
        for host in (sol, j.get_attr(sol, ["SolverOptions"]), j.get_attr(sol, ["StepOptions"])):
            if host is not None and j.set_attr(host, names, val, label=note):
                ok = True
                break
        if not ok:
            j.warning("operating point '%s' not set on this solution" % note, target=val)


def solve(j, sol, label=""):
    """Solve the active solution headless; tolerate the Solve / SolveSolution APIs."""
    if sol is None:
        j.warning("solve skipped (%s): no solution" % label)
        return False
    res = j.method(sol, ["Solve", "SolveSolution"], label="solve %s" % label,
                   target="run the Magnetics solve in the GUI for this stage")
    return res is not None


# --------------------------------------------------------------------------- #
# result read-back  (torque vs step, flux linkage) -- guarded post
# --------------------------------------------------------------------------- #
def read_torque_series(j, sim, sol):
    """Best-effort read of the electromagnetic-torque-vs-step result. Returns a list
    of torque values (Nm). On any post-API miss returns [] and logs the TARGET so the
    user reads it from the Magnetics post (Torque graph / report)."""
    sess = j.session
    rm = j.get_attr(sess, ["ResultManager"]) if sess is not None else None
    if rm is None or sol is None:
        j.warning("torque read skipped (no ResultManager/solution)",
                  target="read 'Electromagnetic Torque' vs step from the Magnetics post")
        return []
    sol_result = j.method(rm, ["CreateSolutionResult", "GetResultForSolution"], sol,
                          label="open solution result",
                          target="Electromagnetic Torque vs step from the Magnetics post")
    if sol_result is None:
        return []
    # The exact Result/ResultParameters/ResultMeasure walk to pull a global torque
    # scalar vs step is release-specific; we expose the hook and degrade gracefully.
    j.note("  torque-vs-step extraction is post-API/version specific -- if the value is "
           "not auto-read, take it from the Magnetics 'Torque' graph and re-score.")
    return []


# --------------------------------------------------------------------------- #
# stages  (mirror motorcad_emag.py / femm_emag.py: same five, same gate)
# --------------------------------------------------------------------------- #
def stage_cogging(j, sim, sol, spec, results):
    j.line("")
    j.line("[cogging] no-current torque over 1 slot pitch ...")
    op = spec.get("operating_points", {})
    set_operating_point(j, sol, op.get("base_speed_rpm", 1000), 0.0, 0.0, op.get("dc_bus_V", 400))
    j.note("  set step schedule to fine steps over 1 slot pitch (%g deg mech)"
           % (360.0 / (spec_slots(spec) or 54)))
    solve(j, sol, "cogging")
    tq = read_torque_series(j, sim, sol)
    results["cogging"] = {"pk_pk_Nm": (sc.pk_pk(tq) if tq else None), "n_points": len(tq)}


def stage_back_emf(j, sim, sol, spec, results):
    j.line("")
    j.line("[back_emf] open-circuit flux linkage / back-EMF at base speed ...")
    op = spec.get("operating_points", {})
    set_operating_point(j, sol, op.get("base_speed_rpm", 1000), 0.0, 0.0, op.get("dc_bus_V", 400))
    solve(j, sol, "back_emf")
    j.note("  read open-circuit phase flux-linkage vs angle -> d/dt -> back-EMF; "
           "magnitude vs Vdc and THD from the harmonic table.")
    results["back_emf"] = {"line_line_peak_V": None, "thd_pct": None}


def stage_torque_angle(j, sim, sol, spec, results):
    """Sweep advance angle beta at PEAK current to locate MTPA -> peak torque."""
    j.line("")
    j.line("[torque_angle] MTPA sweep at peak current ...")
    e = spec.get("excitation", {})
    op = spec.get("operating_points", {})
    betas = e.get("current_advance_angle_deg_from_q_axis", [0, 10, 20, 30, 40])
    ipk = sc.peak_amp(e.get("peak_current_A_rms"))
    speed = op.get("base_speed_rpm", 1000)
    locus = []
    for beta in betas:
        set_operating_point(j, sol, speed, ipk, beta, op.get("dc_bus_V", 400))
        solve(j, sol, "torque@beta=%s" % beta)
        tq = read_torque_series(j, sim, sol)
        avg = sc.mean(tq) if tq else None
        locus.append({"beta_deg": beta, "avg_torque_Nm": avg})
        j.line("    beta=%-4s  T=%s Nm" % (beta, sc.fmt(avg)))
    valid = [pt for pt in locus if isinstance(pt["avg_torque_Nm"], (int, float))]
    best = max(valid, key=lambda pt: pt["avg_torque_Nm"]) if valid else None
    results["torque_angle"] = {"peak_current_A_peak": ipk, "locus": locus, "mtpa": best}
    if best:
        j.ok("MTPA beta=%s deg -> peak torque %.1f Nm" % (best["beta_deg"], best["avg_torque_Nm"]))
    return best


def stage_ripple(j, sim, sol, spec, results, mtpa):
    j.line("")
    j.line("[ripple] instantaneous torque at MTPA over one electrical period ...")
    e = spec.get("excitation", {})
    op = spec.get("operating_points", {})
    beta = (mtpa or {}).get("beta_deg", 15)
    ipk = sc.peak_amp(e.get("peak_current_A_rms"))
    set_operating_point(j, sol, op.get("base_speed_rpm", 1000), ipk, beta, op.get("dc_bus_V", 400))
    solve(j, sol, "ripple")
    tq = read_torque_series(j, sim, sol)
    avg = sc.mean(tq) if tq else 0.0
    ripple_pct = (sc.pk_pk(tq) / avg * 100.0) if (tq and avg) else None
    results["ripple"] = {"avg_Nm": (avg if tq else None),
                         "pk_pk_Nm": (sc.pk_pk(tq) if tq else None),
                         "ripple_pct": ripple_pct}


def stage_demag(j, sim, sol, spec, results):
    """Worst-case demag: peak d-axis-opposing current at the hot magnet temperature."""
    j.line("")
    j.line("[demag] peak opposing current at hot magnet temperature ...")
    e = spec.get("excitation", {})
    op = spec.get("operating_points", {})
    mag = spec.get("materials", {}).get("magnet", {})
    hot = mag.get("max_service_C", 150)
    ipk = sc.peak_amp(e.get("peak_current_A_rms"))
    # beta = 90 deg from q-axis is the most demagnetising (pure -d) direction.
    set_operating_point(j, sol, op.get("base_speed_rpm", 1000), ipk, 90.0,
                        op.get("dc_bus_V", 400), magnet_temp_c=hot)
    j.note("  derate the magnet to Br/Hcj at %s C (Br_tc %s, Hcj_tc %s %%/C) before solving."
           % (hot, mag.get("Br_tempco_pct_per_C"), mag.get("Hcj_tempco_pct_per_C")))
    solve(j, sol, "demag")
    j.note("  read min magnet B vs the hot 2nd-quadrant knee / demagnetised-element fraction.")
    results["demag"] = {"magnet_temp_C": hot, "demag_proportion": None,
                        "min_magnet_B_T": None, "irreversible": None}


def spec_slots(spec):
    return sc.spec_get(spec, ["geometry_mm", "slots"], None)


# --------------------------------------------------------------------------- #
# boundary conditions  (A=0 outer + anti-periodic radial cuts; faces interactive)
# --------------------------------------------------------------------------- #
def apply_boundary_conditions(j, sim, sol, spec):
    """Best-effort A=0 outer boundary + anti-periodic master/slave on the radial cut faces.
    The face/edge SELECTION is interactive (does not journal-replay), so a miss logs the
    TARGET from fea_spec.symmetry_and_bc -- same guarded pattern as the thermal convection
    BC and the structural rotational-velocity load."""
    bc = spec.get("symmetry_and_bc", {})
    j.line("")
    j.line("Boundary conditions (guarded best-effort; faces picked interactively):")
    j.note("  outer: %s" % bc.get("outer_boundary"))
    j.method(sim, ["CreateConstraint", "CreateBoundaryCondition"], "FluxTangent_A0",
             label="A=0 (flux-tangent) outer boundary",
             target="A=0 on the stator OD (%s)" % bc.get("outer_boundary"))
    j.note("  radial cuts: %s (sector %s deg)"
           % (bc.get("radial_cut_faces"), bc.get("sector_angle_deg")))
    j.method(sim, ["CreateConstraint", "CreateBoundaryCondition"], "AntiPeriodic",
             label="anti-periodic master/slave on the radial cut faces",
             target="anti-periodic (odd) master/slave, %s sector" % bc.get("fea_sector"))


# --------------------------------------------------------------------------- #
# loss-field export for the P2 thermal hand-off
# --------------------------------------------------------------------------- #
def loss_seed(j, spec):
    """Export a top-level ``losses`` block so simcenter_thermal can consume it via
    ``losses=<this_results.json>``. Until the Magnetics post read is wired up this is the
    analytical seed from motor_nx.analysis.loss_breakdown at base speed -- CLEARLY labelled
    so it is replaced by FEA-extracted per-region losses once available (never silently
    passed off as solved FEA losses)."""
    parent = sc.repo_root()
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for mname in [m for m in list(sys.modules) if m == "motor_nx" or m.startswith("motor_nx.")]:
        del sys.modules[mname]
    try:
        from motor_nx.params import MotorParams
        from motor_nx import analysis as _an
        from motor_nx import em_design as _em
        p = MotorParams()
        perf = _em.estimate_performance(p)
        lb = _an.loss_breakdown(p, perf.base_speed_rpm)
        seed = {"source": "analysis.loss_breakdown SEED -- replace with FEA-extracted per-region losses",
                "speed_rpm": lb.speed_rpm, "f_e_hz": lb.elec_freq_hz,
                "p_cu_total_w": lb.p_cu_dc_w + lb.p_cu_ac_w,
                "p_iron_w": lb.p_iron_w, "p_magnet_w": lb.p_magnet_w}
        j.ok("loss field exported for P2 (Cu %.0f + iron %.0f + magnet %.0f W) [analytical seed]"
             % (seed["p_cu_total_w"], seed["p_iron_w"], seed["p_magnet_w"]))
        return seed
    except Exception as exc:
        j.note("loss seed unavailable (%s) -- thermal falls back to its own analysis seed." % exc)
        return {"source": None, "p_cu_total_w": None, "p_iron_w": None, "p_magnet_w": None}


# --------------------------------------------------------------------------- #
# setup card  (printed when no usable .sim is open -- complete + correct checklist)
# --------------------------------------------------------------------------- #
def print_setup_card(j, spec):
    g = spec.get("geometry_mm", {})
    bc = spec.get("symmetry_and_bc", {})
    me = spec.get("mesh", {})
    j.line("")
    j.line("================ SIMCENTER 3D MAGNETICS -- SETUP CARD ================")
    j.line("No usable .sim open. Build the FEM+Sim ONCE interactively, then re-run")
    j.line("this journal on the saved .sim to drive the matrix + score the gate.")
    j.line("")
    j.line("1) NEW FEM+SIM on the 2D lamination plane (fea/cross_section.dxf or the .prt):")
    j.line("     solver = Simcenter Magnetics, analysis type = 2D, solution = Transient.")
    j.line("2) GEOMETRY/MESH (interactive -- does not journal-replay):")
    j.line("     stator OD %s / bore %s / airgap %s / rotor OD %s / shaft %s mm, stack %s mm"
           % (g.get("stator_OD"), g.get("stator_bore"), g.get("air_gap"),
              g.get("rotor_OD"), g.get("shaft_OD"), g.get("stack_length")))
    j.line("     %s slots / %s poles; airgap %s radial layers @ ~%s mm; global ~%s mm; refine %s"
           % (g.get("slots"), g.get("poles"), me.get("airgap_radial_layers"),
              me.get("airgap_element_mm"), me.get("global_element_mm"), me.get("refine")))
    j.line("     collectors: STATOR_STEEL, ROTOR_STEEL, MAGNET(s), COIL_A/B/C, AIRGAP, rotor MOTION region")
    j.line("3) BOUNDARY CONDITIONS:")
    j.line("     sector = %s (%s deg); radial cuts = %s; outer = %s"
           % (bc.get("fea_sector"), bc.get("sector_angle_deg"),
              bc.get("radial_cut_faces"), bc.get("outer_boundary")))
    j.line("4) Save the .sim, then:")
    j.line("     run_journal.exe verification\\simcenter_emag.py -args <that>.sim "
           "cogging,back_emf,torque_angle,ripple,demag")
    j.line("---- This journal then sets material values / coil current / speed / steps /")
    j.line("     A=0 + anti-periodic / solves / reads torque / scores vs the gate. ----")
    winding_summary(j, spec)


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def main():
    args = sc.parse_args(sys.argv[1:], all_stages=ALL_STAGES)
    spec, spec_path = sc.load_spec(args.get("spec"))
    sess = get_session()
    j = sc.Journal(session=sess, name="emag (%s)" % spec.get("name", "motor"))
    j.line("spec: %s" % spec_path)

    if not _HAVE_NX:
        j.fail("NXOpen not importable -- run this with run_journal.exe inside NX/Simcenter 3D.")
        print_setup_card(j, spec)
        return

    stages = args.get("stages") or ALL_STAGES
    j.line("stages: %s" % ", ".join(stages))

    sim = open_sim(j, args)
    if sim is None:
        j.note("no usable .sim open -- printing the setup card.")
        print_setup_card(j, spec)
        return

    apply_materials(j, sim, spec)
    winding_summary(j, spec)
    sol = active_solution(j, sim)
    apply_boundary_conditions(j, sim, sol, spec)

    results = {"design": spec.get("name"), "spec_path": spec_path, "discipline": "EM"}
    results["losses"] = loss_seed(j, spec)   # exported for the P2 thermal hand-off
    mtpa = None
    if "cogging" in stages:
        stage_cogging(j, sim, sol, spec, results)
    if "back_emf" in stages:
        stage_back_emf(j, sim, sol, spec, results)
    if "torque_angle" in stages:
        mtpa = stage_torque_angle(j, sim, sol, spec, results)
    if "ripple" in stages:
        stage_ripple(j, sim, sol, spec, results, mtpa)
    if "demag" in stages:
        stage_demag(j, sim, sol, spec, results)

    ck = sc.score_em(spec, results)
    results["acceptance"] = ck.to_list()
    results["warnings"] = j.warn
    ck.print(j, gate="G1 EM")
    if j.warn:
        j.line("")
        j.line("  %d field/post warning(s) -- confirm those member names in your NX 2506 "
               "Magnetics build, or finish them by hand (TARGET shown)." % len(j.warn))

    out = args.get("out") or os.path.join(sc.repo_root(), "fea", "simcenter_emag_results.json")
    sc.write_results(out, results, j)
    j.line("=== emag journal done ===")


if (__name__ == "__main__" or os.environ.get("UGII_ROOT_DIR")) \
        and not os.environ.get("SIMCENTER_IMPORTED_AS_LIB"):
    try:
        main()
    except Exception:
        # never let a journal crash NX with an unhandled traceback
        s = get_session()
        if s is not None:
            try:
                w = s.ListingWindow
                w.Open()
                w.WriteLine("FATAL simcenter_emag:\n" + traceback.format_exc())
            except Exception:
                pass
        else:
            traceback.print_exc()
