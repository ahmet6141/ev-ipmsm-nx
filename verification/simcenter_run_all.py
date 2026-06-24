"""Run the three Simcenter 3D verification journals in the project's discipline order
(EM -> Thermal -> Structural) and write one combined acceptance report.

Run INSIDE NX / Simcenter 3D via run_journal.exe::

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_run_all.py ^
        -args emag=motor_emag.sim thermal=motor_thermal.sim structural=motor_struct.sim ^
              spec=fea\\fea_spec.json out=fea\\simcenter_all_results.json

Discipline order is load-bearing (docs/PROJECT_PLAN.md): EM runs FIRST and sets the loss
field; Thermal consumes it for the continuous rating; Structural can veto the thin
bridges. A single .sim rarely carries all three solvers, so each journal takes its OWN
.sim via ``emag=`` / ``thermal=`` / ``structural=``. Any discipline whose .sim is missing
is skipped (its gate is reported n/a) rather than aborting the others.

This is a thin coordinator: rather than each journal's ``main()`` (which parses its own
argv), it calls the journals' stage functions directly so the three results dicts can be
merged into one report. Each NXOpen step inside the journals stays guarded + logged.
"""
from __future__ import annotations

import os
import sys
import traceback

# Importing the discipline journals must NOT trigger their run_journal auto-run guard
# (which also fires on UGII_ROOT_DIR inside NX). Set this BEFORE importing them.
os.environ["SIMCENTER_IMPORTED_AS_LIB"] = "1"

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
import simcenter_common as sc  # noqa: E402
import simcenter_emag as emag  # noqa: E402
import simcenter_thermal as thermal  # noqa: E402
import simcenter_structural as structural  # noqa: E402


def _parse(argv):
    """Per-discipline .sim selection: emag=..., thermal=..., structural=..., plus
    spec=... and out=...."""
    out = {"emag": None, "thermal": None, "structural": None, "spec": None, "out": None,
           "losses": None}
    for a in argv:
        al = a.lower()
        for key in ("emag", "thermal", "structural", "spec", "out", "losses"):
            if al.startswith(key + "="):
                out[key] = a.split("=", 1)[1]
    return out


def main():
    sel = _parse(sys.argv[1:])
    spec, spec_path = sc.load_spec(sel.get("spec"))
    sess = emag.get_session()
    j = sc.Journal(session=sess, name="run_all (%s)" % spec.get("name", "motor"))
    j.line("spec: %s" % spec_path)
    j.line("discipline order: EM -> Thermal -> Structural (PROJECT_PLAN P1->P2->P3)")

    combined = {"design": spec.get("name"), "spec_path": spec_path, "disciplines": {}}
    all_checks = []

    # ---- P1 EM ----------------------------------------------------------- #
    j.line("")
    j.line("########## P1 EM (Magnetics) ##########")
    em_res = {"design": spec.get("name"), "discipline": "EM"}
    if not emag._HAVE_NX:
        j.fail("NXOpen unavailable -- run under run_journal.exe.")
    elif sel.get("emag"):
        try:
            args = {"sim": sel["emag"], "spec": sel.get("spec")}
            sim = emag.open_sim(j, args)
            if sim is not None:
                emag.apply_materials(j, sim, spec)
                emag.winding_summary(j, spec)
                sol = emag.active_solution(j, sim)
                emag.apply_boundary_conditions(j, sim, sol, spec)
                em_res["losses"] = emag.loss_seed(j, spec)
                emag.stage_cogging(j, sim, sol, spec, em_res)
                emag.stage_back_emf(j, sim, sol, spec, em_res)
                mtpa = emag.stage_torque_angle(j, sim, sol, spec, em_res)
                emag.stage_ripple(j, sim, sol, spec, em_res, mtpa)
                emag.stage_demag(j, sim, sol, spec, em_res)
            else:
                emag.print_setup_card(j, spec)
        except Exception:
            j.fail("EM stage raised:\n" + traceback.format_exc())
    else:
        j.note("no emag=<.sim> given -- EM gate reported n/a.")
    ck_em = sc.score_em(spec, em_res)
    em_res["acceptance"] = ck_em.to_list()
    combined["disciplines"]["EM"] = em_res
    all_checks += ck_em.to_list()
    ck_em.print(j, gate="G1 EM")

    # carry the EM loss field (if any) to thermal (losses=<emag_results.json>)
    losses_path = sel.get("losses")

    # ---- P2 Thermal ------------------------------------------------------ #
    j.line("")
    j.line("########## P2 Thermal ##########")
    th_res = {"design": spec.get("name"), "discipline": "Thermal"}
    if not thermal._HAVE_NX:
        j.fail("NXOpen unavailable -- run under run_journal.exe.")
    elif sel.get("thermal"):
        try:
            args = {"sim": sel["thermal"], "spec": sel.get("spec"), "losses": losses_path,
                    "extra": []}
            # prefer the EM stage's in-memory loss field when no explicit losses=<file> given
            em_losses = em_res.get("losses")
            if not losses_path and isinstance(em_losses, dict) \
                    and em_losses.get("p_cu_total_w") is not None:
                args["losses_block"] = em_losses
            losses = thermal.loss_field(j, spec, args)
            inlet_c, h = thermal.coolant_params(j, spec)
            th_res.update({"loss_field": losses, "coolant_inlet_C": inlet_c, "jacket_h_W_m2K": h})
            sim = thermal.open_sim(j, args)
            if sim is not None:
                sol = thermal.active_solution(j, sim)
                thermal.continuous_rating(j, sim, sol, spec, losses, inlet_c, h, th_res)
            else:
                thermal.print_setup_card(j, spec, losses, inlet_c, h)
        except Exception:
            j.fail("Thermal stage raised:\n" + traceback.format_exc())
    else:
        j.note("no thermal=<.sim> given -- Thermal gate reported n/a.")
    ck_th = sc.score_thermal(spec, th_res)
    th_res["acceptance"] = ck_th.to_list()
    combined["disciplines"]["Thermal"] = th_res
    all_checks += ck_th.to_list()
    ck_th.print(j, gate="G2 thermal")

    # ---- P3 Structural --------------------------------------------------- #
    j.line("")
    j.line("########## P3 Structural ##########")
    st_res = {"design": spec.get("name"), "discipline": "Structural"}
    if not structural._HAVE_NX:
        j.fail("NXOpen unavailable -- run under run_journal.exe.")
    elif sel.get("structural"):
        try:
            args = {"sim": sel["structural"], "spec": sel.get("spec")}
            overspeed, yield_mpa, density = structural.struct_params(j, spec)
            sim = structural.open_sim(j, args)
            if sim is not None:
                sols = structural.list_solutions(j, sim)
                structural.stage_rotor_stress(j, sim, sols, spec, st_res,
                                               overspeed, yield_mpa, density)
                structural.stage_rotordynamics(j, sim, sols, spec, st_res)
            else:
                structural.print_setup_card(j, spec, overspeed, yield_mpa, density)
        except Exception:
            j.fail("Structural stage raised:\n" + traceback.format_exc())
    else:
        j.note("no structural=<.sim> given -- Structural gate reported n/a.")
    ck_st = sc.score_structural(spec, st_res)
    st_res["acceptance"] = ck_st.to_list()
    combined["disciplines"]["Structural"] = st_res
    all_checks += ck_st.to_list()
    ck_st.print(j, gate="G3 structural")

    # ---- combined verdict ------------------------------------------------ #
    measured = [c for c in all_checks if c["pass"] is not None]
    passed = [c for c in measured if c["pass"]]
    combined["acceptance"] = all_checks
    combined["summary"] = {"measured": len(measured), "passed": len(passed),
                           "all_pass": (len(measured) > 0 and len(passed) == len(measured))}
    combined["warnings"] = j.warn

    j.line("")
    j.line("====================== COMBINED VERDICT ======================")
    j.line("  %d/%d measured criteria pass%s"
           % (len(passed), len(measured),
              "  -> ALL PASS (design freeze candidate)" if combined["summary"]["all_pass"]
              else "  (open gates remain -- see per-discipline FAIL/n.a above)"))

    out = sel.get("out") or os.path.join(sc.repo_root(), "fea", "simcenter_all_results.json")
    sc.write_results(out, combined, j)
    j.line("=== run_all done ===")


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        s = emag.get_session()
        if s is not None:
            try:
                w = s.ListingWindow
                w.Open()
                w.WriteLine("FATAL simcenter_run_all:\n" + traceback.format_exc())
            except Exception:
                pass
        else:
            traceback.print_exc()
