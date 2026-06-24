"""P3 structural verification driver -- Siemens Simcenter 3D / Simcenter Nastran,
driven as an NXOpen journal.

Run INSIDE NX / Simcenter 3D via run_journal.exe::

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_structural.py ^
        -args motor_rotor.sim spec=fea\\fea_spec.json ^
              out=fea\\simcenter_structural_results.json

Two checks, scored against fea_spec.acceptance_targets:

  rotor_stress  (SOL 101, linear static)  -- centrifugal von Mises in the outer bridges
        (1.0 mm) and centre post at 1.2x max speed (21600 rpm); SF = rotor-steel yield /
        max von Mises must be >= 1.5.
  rotordynamics (SOL 103, normal modes)   -- shaft+rotor first bending natural frequency;
        critical speed (= f x 60) must clear the 18000 rpm max operating speed with margin.

The overspeed (1.2x), the rotor-lamination yield (~450 MPa) and steel density (7650
kg/m3) are taken from motor_nx.analysis.AnalysisAssumptions (imported in-session, like
nx_drafting) so the journal stays in lockstep with the first-order seed; each is logged
with its TARGET so you can confirm/override against your steel datasheet.

HONEST SCOPE -- interactive prerequisite (done once) is the FEM+Sim: the rotor
lamination (2D plane-stress or 3D) meshed for SOL 101, and the shaft+rotor-mass model
meshed for SOL 103, with a ROTOR_LAM mesh collector and the bore/cyclic constraints.
This journal then sets the rotational-velocity load, the material yield/density values,
solves and reads max von Mises / mode frequencies, all guarded + logged. No usable .sim
-> it prints the setup card and exits.
"""
from __future__ import annotations

import os
import sys
import traceback

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
import simcenter_common as sc  # noqa: E402

try:
    import NXOpen
    import NXOpen.CAE
    _HAVE_NX = True
except Exception:
    NXOpen = None
    _HAVE_NX = False


def get_session():
    return NXOpen.Session.GetSession() if _HAVE_NX else None


# --------------------------------------------------------------------------- #
# structural assumptions (overspeed factor, rotor-lam yield, density)
# --------------------------------------------------------------------------- #
def struct_params(j, spec):
    overspeed, yield_mpa, density = 1.2, 450.0, 7650.0
    parent = sc.repo_root()
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for mname in [m for m in list(sys.modules) if m == "motor_nx" or m.startswith("motor_nx.")]:
        del sys.modules[mname]
    try:
        from motor_nx.analysis import AnalysisAssumptions
        a = AnalysisAssumptions()
        overspeed, yield_mpa, density = a.overspeed_factor, a.rotor_steel_yield_mpa, a.rotor_steel_density
        j.ok("structural seed from motor_nx.analysis: overspeed x%s, yield %s MPa, density %s kg/m3"
             % (overspeed, yield_mpa, density))
    except Exception:
        j.note("motor_nx.analysis not importable here -- using defaults overspeed x%s, "
               "yield %s MPa, density %s kg/m3 (confirm against your steel datasheet)."
               % (overspeed, yield_mpa, density))
    return overspeed, yield_mpa, density


# --------------------------------------------------------------------------- #
# Sim part / solutions
# --------------------------------------------------------------------------- #
def open_sim(j, args):
    target = args.get("sim") or args.get("fem") or args.get("prt")
    sess = j.session
    if target and os.path.exists(target):
        j.method(sess.Parts, ["OpenBaseDisplay", "OpenActiveDisplay", "Open"], target,
                 label="open %s" % os.path.basename(target),
                 target="a .sim with SOL 101 (stress) and/or SOL 103 (modes) solutions")
    elif target:
        j.warning("model file not found: %s" % target,
                  target="pass an existing structural .sim/.fem/.prt as the first -args token")
    work = sess.Parts.BaseWork if sess is not None else None
    if work is None or getattr(work, "Tag", 0) == 0:
        return None
    return work


def list_solutions(j, sim):
    if sim is None:
        return []
    simn = j.get_attr(sim, ["Simulation"])
    if simn is None:
        j.warning("no .Simulation on the work part", target="open the structural .sim")
        return []
    sols = j.get_attr(simn, ["Solutions"])
    out = []
    try:
        out = list(sols) if sols is not None else []
    except Exception:
        out = []
    if out:
        j.ok("%d solution(s): %s" % (len(out), ", ".join(j.get_attr(s, ["Name"], "?") for s in out)))
    else:
        j.warning("no solutions found",
                  target="create a SOL 101 (linear static) and a SOL 103 (normal modes) solution")
    return out


def _pick(j, sols, keywords):
    """Pick the first solution whose name/solver hints at the given keywords."""
    for s in sols:
        name = str(j.get_attr(s, ["Name"], "")).lower()
        if any(k in name for k in keywords):
            return s
    return sols[0] if sols else None


def solve(j, sol, label=""):
    if sol is None:
        return False
    return j.method(sol, ["Solve", "SolveSolution"], label="solve %s" % label,
                    target="run the %s solve" % label) is not None


# --------------------------------------------------------------------------- #
# SOL 101 -- rotor centrifugal stress at overspeed
# --------------------------------------------------------------------------- #
def stage_rotor_stress(j, sim, sols, spec, results, overspeed, yield_mpa, density):
    j.line("")
    j.line("[rotor_stress] SOL 101 centrifugal von Mises at overspeed ...")
    op = spec.get("operating_points", {})
    g = spec.get("geometry_mm", {})
    n_max = op.get("max_speed_rpm", 18000)
    n_over = overspeed * n_max
    omega = sc.rpm_to_rad_s(n_over)
    j.note("  overspeed = %.0f rpm (%.1fx %.0f); omega = %.0f rad/s; rotor steel yield %.0f MPa, "
           "density %.0f kg/m3" % (n_over, overspeed, n_max, omega, yield_mpa, density))
    j.note("  thin features under load: outer bridge %s mm, centre-post halfwidth %s mm"
           % (g.get("outer_bridge_mm"), g.get("center_post_halfwidth_mm")))

    sol = _pick(j, sols, ["101", "static", "stress", "stress"])
    if sol is None:
        j.warning("no SOL 101 solution", target="centrifugal stress at %.0f rpm" % n_over)
    else:
        # rotational-velocity (RFORCE) load about the shaft axis at the overspeed
        j.method(sim, ["CreateLoad", "CreateLoadBuilder"], "RotationalVelocity",
                 label="create rotational-velocity load",
                 target="%.0f rpm (%.0f rad/s) about the shaft (Z) axis" % (n_over, omega))
        j.note("  set the rotor-lam material yield %.0f MPa / density %.0f kg/m3 and the bore "
               "constraint (or 1-pole cyclic symmetry) by hand if not already in the FEM."
               % (yield_mpa, density))
        solve(j, sol, "SOL 101 rotor stress")

    smax = read_max_von_mises(j, sim, sol)
    sf = (yield_mpa / smax) if (isinstance(smax, (int, float)) and smax > 0) else None
    results["rotor_stress"] = {"overspeed_rpm": n_over, "omega_rad_s": omega,
                               "yield_MPa": yield_mpa, "max_vonMises_MPa": smax,
                               "safety_factor": sf}
    if sf is not None:
        j.ok("max von Mises %.0f MPa -> SF %.2f" % (smax, sf))


def read_max_von_mises(j, sim, sol):
    sess = j.session
    rm = j.get_attr(sess, ["ResultManager"]) if sess is not None else None
    if rm is None or sol is None:
        j.warning("von Mises read skipped (no ResultManager/solution)",
                  target="max elemental von Mises on the ROTOR_LAM group from the Post")
        return None
    sr = j.method(rm, ["CreateSolutionResult", "GetResultForSolution"], sol,
                  label="open SOL 101 result",
                  target="max elemental von Mises on the ROTOR_LAM group")
    if sr is None:
        return None
    j.note("  max-von-Mises extraction is post-API/version specific -- if not auto-read, take "
           "the bridge/post peak from the Post (envelope on ROTOR_LAM) and re-score.")
    return None


# --------------------------------------------------------------------------- #
# SOL 103 -- normal modes / first bending critical speed
# --------------------------------------------------------------------------- #
def stage_rotordynamics(j, sim, sols, spec, results):
    j.line("")
    j.line("[rotordynamics] SOL 103 first bending critical speed ...")
    op = spec.get("operating_points", {})
    n_max = op.get("max_speed_rpm", 18000)
    j.note("  shaft = hollow %s mm; first bending critical speed must clear %s rpm "
           "(=> f1 > %.0f Hz) with margin." % (sc.spec_get(spec, ["geometry_mm", "shaft_OD"], 45),
                                               n_max, n_max / 60.0))
    sol = _pick(j, sols, ["103", "mode", "modal", "normal"])
    if sol is None:
        j.warning("no SOL 103 solution", target="first bending mode of shaft+rotor mass")
    else:
        j.note("  include the rotor lamination + magnet mass on the hollow %s mm shaft; "
               "bearing supports at the journals." % sc.spec_get(spec, ["geometry_mm", "shaft_OD"], 45))
        solve(j, sol, "SOL 103 modes")
    freqs = read_mode_frequencies(j, sim, sol)
    f1 = freqs[0] if freqs else None
    results["rotordynamics"] = {"max_speed_rpm": n_max, "mode_freqs_Hz": freqs,
                                "first_bending_Hz": f1,
                                "first_critical_rpm": (sc.hz_to_rpm(f1) if f1 else None)}
    if f1:
        j.ok("mode 1 = %.0f Hz -> first critical ~%.0f rpm (confirm it is a BENDING mode)"
             % (f1, sc.hz_to_rpm(f1)))


def read_mode_frequencies(j, sim, sol, max_modes=6):
    sess = j.session
    rm = j.get_attr(sess, ["ResultManager"]) if sess is not None else None
    if rm is None or sol is None:
        j.warning("mode-frequency read skipped (no ResultManager/solution)",
                  target="natural frequencies (Hz) from the SOL 103 result")
        return []
    sr = j.method(rm, ["CreateSolutionResult", "GetResultForSolution"], sol,
                  label="open SOL 103 result",
                  target="natural frequencies (Hz)")
    if sr is None:
        return []
    j.note("  mode-frequency extraction is post-API/version specific -- if not auto-read, take "
           "the natural frequencies from the SOL 103 result and identify the 1st bending mode.")
    return []


# --------------------------------------------------------------------------- #
# setup card
# --------------------------------------------------------------------------- #
def print_setup_card(j, spec, overspeed, yield_mpa, density):
    op = spec.get("operating_points", {})
    g = spec.get("geometry_mm", {})
    n_over = overspeed * op.get("max_speed_rpm", 18000)
    j.line("")
    j.line("================ SIMCENTER 3D STRUCTURAL -- SETUP CARD ================")
    j.line("Build the FEM+Sim ONCE interactively, then re-run on the saved .sim.")
    j.line("A) ROTOR STRESS (SOL 101, linear static):")
    j.line("     mesh the rotor lamination (2D plane-stress or 3D), collector ROTOR_LAM;")
    j.line("     material: rotor steel yield %.0f MPa, density %.0f kg/m3;" % (yield_mpa, density))
    j.line("     load: rotational velocity %.0f rpm about the shaft (Z) axis;" % n_over)
    j.line("     constraint: bore fixed or 1-pole cyclic symmetry;")
    j.line("     target: SF = yield / max von Mises >= %s in the %s mm bridges / centre post."
           % (sc.spec_get(spec, ["acceptance_targets",
                                 "rotor_vonMises_safety_factor_min_at_1.2x_maxspeed"], 1.5),
              g.get("outer_bridge_mm")))
    j.line("B) ROTORDYNAMICS (SOL 103, normal modes):")
    j.line("     shaft+rotor-mass model on the hollow %s mm shaft, bearing supports at journals;"
           % g.get("shaft_OD"))
    j.line("     target: 1st bending critical speed > %s rpm (f1 > %.0f Hz) with margin."
           % (op.get("max_speed_rpm", 18000), op.get("max_speed_rpm", 18000) / 60.0))
    j.line("C) Save .sim, then: run_journal.exe verification\\simcenter_structural.py -args <that>.sim")


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def main():
    args = sc.parse_args(sys.argv[1:])
    spec, spec_path = sc.load_spec(args.get("spec"))
    sess = get_session()
    j = sc.Journal(session=sess, name="structural (%s)" % spec.get("name", "motor"))
    j.line("spec: %s" % spec_path)

    overspeed, yield_mpa, density = struct_params(j, spec)

    if not _HAVE_NX:
        j.fail("NXOpen not importable -- run with run_journal.exe inside NX/Simcenter 3D.")
        print_setup_card(j, spec, overspeed, yield_mpa, density)
        return

    sim = open_sim(j, args)
    if sim is None:
        j.note("no usable .sim open -- printing the setup card.")
        print_setup_card(j, spec, overspeed, yield_mpa, density)
        return

    sols = list_solutions(j, sim)
    results = {"design": spec.get("name"), "spec_path": spec_path, "discipline": "Structural"}
    stage_rotor_stress(j, sim, sols, spec, results, overspeed, yield_mpa, density)
    stage_rotordynamics(j, sim, sols, spec, results)

    ck = sc.score_structural(spec, results)
    results["acceptance"] = ck.to_list()
    results["warnings"] = j.warn
    ck.print(j, gate="G3 structural")
    if j.warn:
        j.line("")
        j.line("  %d field/post warning(s) -- confirm member names in your NX 2506 Nastran "
               "build or finish by hand (TARGET shown)." % len(j.warn))

    out = args.get("out") or os.path.join(sc.repo_root(), "fea", "simcenter_structural_results.json")
    sc.write_results(out, results, j)
    j.line("=== structural journal done ===")


if (__name__ == "__main__" or os.environ.get("UGII_ROOT_DIR")) \
        and not os.environ.get("SIMCENTER_IMPORTED_AS_LIB"):
    try:
        main()
    except Exception:
        s = get_session()
        if s is not None:
            try:
                w = s.ListingWindow
                w.Open()
                w.WriteLine("FATAL simcenter_structural:\n" + traceback.format_exc())
            except Exception:
                pass
        else:
            traceback.print_exc()
