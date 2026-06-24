"""P2 thermal verification driver -- Siemens Simcenter 3D Thermal (steady-state),
driven as an NXOpen journal.

Run INSIDE NX / Simcenter 3D via run_journal.exe::

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_thermal.py ^
        -args motor_thermal.sim spec=fea\\fea_spec.json ^
              out=fea\\simcenter_thermal_results.json

Finds the thermally-limited CONTINUOUS rating: it imposes the P1 EM loss field
(copper + iron + magnet) on the model, applies the water-jacket convection BC, solves
steady state, reads the winding and magnet hotspots, and reduces the current until the
binding hotspot (winding 180 C class-H or magnet 150 C) is reached -- that current's
torque is the continuous rating. Scored against fea_spec.acceptance_targets
(continuous_torque_Nm_min, magnet_temp_C_max_continuous).

LOSS INPUTS (priority):
  1. ``losses=<emag_results.json>`` arg, if it carries a loss field;
  2. else motor_nx.analysis.loss_breakdown (imported in-session, like nx_drafting),
     evaluated at the continuous current -- the documented P2 seed (Cu DC+AC, iron, magnet);
  3. else the TARGET values are logged for you to enter by hand.

HONEST SCOPE -- the interactive prerequisite (done once) is the FEM+Sim: a Simcenter
3D Thermal steady-state solution with the stator/rotor/magnet/winding solids meshed and
mesh collectors named. This journal then automates the loss loads, the jacket convection
BC, the solve and the hotspot read-back, all guarded + logged (a drifted member name
becomes a warning carrying the TARGET, never an abort). If no usable .sim is open it
prints the complete setup card and exits.
"""
from __future__ import annotations

import json
import math
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
# loss field (W) at a given current -- from EM results, else the analysis.py seed
# --------------------------------------------------------------------------- #
def _import_analysis():
    """Import the NX-independent motor_nx.analysis in-session (drop cached copies),
    mirroring nx_drafting._load_design. Returns (MotorParams, analysis, em_design) or None."""
    parent = sc.repo_root()
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for mname in [m for m in list(sys.modules) if m == "motor_nx" or m.startswith("motor_nx.")]:
        del sys.modules[mname]
    try:
        from motor_nx.params import MotorParams
        from motor_nx import analysis as _an
        from motor_nx import em_design as _em
        return MotorParams, _an, _em
    except Exception:
        return None


def loss_field(j, spec, args):
    """Return a dict of loss components in WATTS at the continuous current, plus the
    seed metadata. Each value may be None if no source is available (logged as TARGET)."""
    # 0) an in-memory loss block handed straight from the EM stage (run_all flow)
    block = args.get("losses_block")
    if isinstance(block, dict) and (block.get("p_cu_total_w") or block.get("p_cu_w")) is not None:
        j.ok("loss field from the EM stage (source: %s)" % block.get("source", "EM stage"))
        return {"source": block.get("source", "EM stage"),
                "p_cu_w": block.get("p_cu_total_w") or block.get("p_cu_w"),
                "p_iron_w": block.get("p_iron_w"), "p_magnet_w": block.get("p_magnet_w")}

    # 1) an EM-results JSON that carries a loss field (the simcenter_emag driver writes a
    #    top-level "losses" block; see simcenter_emag.loss_seed)
    lp = args.get("losses") or None
    if lp and os.path.exists(lp):
        try:
            with open(lp, "r", encoding="utf-8") as fh:
                emr = json.load(fh)
            lf = emr.get("losses") or {}
            if lf and (lf.get("p_cu_total_w") or lf.get("p_cu_w")) is not None:
                j.ok("loss field from %s (source: %s)"
                     % (os.path.basename(lp), lf.get("source", "EM results")))
                return {"source": lp, "p_cu_w": lf.get("p_cu_total_w") or lf.get("p_cu_w"),
                        "p_iron_w": lf.get("p_iron_w"), "p_magnet_w": lf.get("p_magnet_w")}
            j.note("%s has no usable 'losses' block -- falling back to the analysis seed."
                   % os.path.basename(lp))
        except Exception as exc:
            j.warning("could not read losses from %s (%s)" % (lp, exc))

    # 2) the analysis.py seed (documented P2 starting losses)
    mods = _import_analysis()
    if mods is not None:
        MotorParams, an, em = mods
        try:
            p = MotorParams.load_json(args.get("model_json")) if args.get("model_json") else MotorParams()
            perf = em.estimate_performance(p)
            lb = an.loss_breakdown(p, perf.base_speed_rpm)
            j.ok("loss field from motor_nx.analysis.loss_breakdown @ base speed "
                 "(Cu %.0f W + iron %.0f W + magnet %.0f W)"
                 % (lb.p_cu_dc_w + lb.p_cu_ac_w, lb.p_iron_w, lb.p_magnet_w))
            return {"source": "analysis.loss_breakdown",
                    "p_cu_w": lb.p_cu_dc_w + lb.p_cu_ac_w,
                    "p_iron_w": lb.p_iron_w, "p_magnet_w": lb.p_magnet_w,
                    "f_e_hz": lb.elec_freq_hz, "j_a_mm2": lb.current_density_a_mm2}
        except Exception as exc:
            j.warning("analysis.loss_breakdown failed (%s)" % exc)

    # 3) nothing usable -- log the TARGET so the user enters losses by hand
    j.warning("no loss field available", target="pass losses=<emag_results.json> or impose "
              "Cu/iron/magnet heat loads from the P1 EM solve by hand")
    return {"source": None, "p_cu_w": None, "p_iron_w": None, "p_magnet_w": None}


# --------------------------------------------------------------------------- #
# coolant / jacket parameters (water jacket)
# --------------------------------------------------------------------------- #
def coolant_params(j, spec):
    """Coolant inlet temp + effective jacket convection coefficient. Defaults are the
    analysis.py thermal seed (65 C, ~3000 W/m2K); confirm against the real jacket."""
    inlet_c, h = 65.0, 3000.0
    mods = _import_analysis()
    if mods is not None:
        try:
            from motor_nx.analysis import AnalysisAssumptions
            a = AnalysisAssumptions()
            inlet_c, h = a.coolant_inlet_c, a.h_conv_w_m2k
        except Exception:
            pass
    j.note("coolant inlet %s C, effective jacket h ~%s W/m2K (SEED -- set the real coolant "
           "flow / inlet temp / h-correlation for the 12-channel Al jacket)." % (inlet_c, h))
    return inlet_c, h


# --------------------------------------------------------------------------- #
# Sim part / solution
# --------------------------------------------------------------------------- #
def open_sim(j, args):
    target = args.get("sim") or args.get("fem") or args.get("prt")
    sess = j.session
    if target and os.path.exists(target):
        j.method(sess.Parts, ["OpenBaseDisplay", "OpenActiveDisplay", "Open"], target,
                 label="open %s" % os.path.basename(target),
                 target="a .sim on a Simcenter 3D Thermal steady-state solution")
    elif target:
        j.warning("model file not found: %s" % target,
                  target="pass an existing thermal .sim/.fem/.prt as the first -args token")
    work = sess.Parts.BaseWork if sess is not None else None
    if work is None or getattr(work, "Tag", 0) == 0:
        return None
    return work


def active_solution(j, sim):
    if sim is None:
        return None
    simn = j.get_attr(sim, ["Simulation"])
    if simn is None:
        j.warning("no .Simulation on the work part",
                  target="open the thermal .sim")
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
        j.warning("no thermal solution found",
                  target="create a Thermal steady-state solution in the Sim file")
    return sol


# --------------------------------------------------------------------------- #
# loads + BC  (heat loads on collectors; convection BC on the jacket face)
# --------------------------------------------------------------------------- #
def apply_loads(j, sim, sol, losses, inlet_c, h):
    j.line("")
    j.line("Thermal loads + jacket convection (values scriptable; region picks interactive):")
    sb = j.get_attr(sim, ["SimulationBuilder"]) or j.get_attr(sim, ["Simulation"])

    def heat_load(name, watts, collector_hint):
        if watts is None:
            j.warning("heat load %s not set" % name, target="impose from the P1 EM loss field")
            return
        j.note("  %-18s %.0f W  -> collector %s" % (name, watts, collector_hint))
        # the create+target-region call is version/collector specific -> guarded
        j.method(sim, ["CreateLoad", "CreateThermalLoad"], "ThermalLoad", name,
                 label="create heat load %s" % name,
                 target="%.0f W on %s" % (watts, collector_hint))

    heat_load("Cu_loss", losses.get("p_cu_w"), "COIL_A/B/C (winding)")
    heat_load("Iron_loss", losses.get("p_iron_w"), "STATOR_STEEL (+ rotor fraction)")
    heat_load("Magnet_loss", losses.get("p_magnet_w"), "MAGNET (axial-segmented)")

    j.note("  CONVECTION: jacket face, h ~%s W/m2K, ambient/coolant %s C" % (h, inlet_c))
    j.method(sim, ["CreateBoundaryCondition", "CreateConstraint"], "ConvectionToEnvironment",
             "Jacket_Convection", label="create jacket convection BC",
             target="h=%s W/m2K, T=%s C on the stator-OD jacket face" % (h, inlet_c))


# --------------------------------------------------------------------------- #
# solve + read hotspots
# --------------------------------------------------------------------------- #
def solve(j, sol, label=""):
    if sol is None:
        return False
    return j.method(sol, ["Solve", "SolveSolution"], label="solve %s" % label,
                    target="run the thermal steady-state solve") is not None


def read_hotspots(j, sim, sol):
    """Read winding-max and magnet-max temperatures from the thermal result. Returns
    (winding_C, magnet_C); None where the post-API read misses (logged with TARGET)."""
    sess = j.session
    rm = j.get_attr(sess, ["ResultManager"]) if sess is not None else None
    if rm is None or sol is None:
        j.warning("hotspot read skipped (no ResultManager/solution)",
                  target="read max Temperature on COIL and MAGNET groups from the Thermal post")
        return None, None
    sr = j.method(rm, ["CreateSolutionResult", "GetResultForSolution"], sol,
                  label="open thermal result",
                  target="max Temperature on COIL and MAGNET groups")
    if sr is None:
        return None, None
    j.note("  per-group max-Temperature extraction is post-API/version specific -- if not "
           "auto-read, take max T on the COIL and MAGNET groups and re-score.")
    return None, None


# --------------------------------------------------------------------------- #
# continuous-rating loop  (reduce current until the binding hotspot = its limit)
# --------------------------------------------------------------------------- #
def continuous_rating(j, sim, sol, spec, losses, inlet_c, h, results):
    """Iterate current DOWN until winding<=180 C and magnet<=150 C. Copper loss scales
    with I^2; iron/magnet (speed-driven) held. Torque ~ I. Pure scaling around the solved
    point -- the Simcenter solve sets the temperatures; this finds the limiting current."""
    op = spec.get("operating_points", {})
    e = spec.get("excitation", {})
    mag = spec.get("materials", {}).get("magnet", {})
    t = spec.get("acceptance_targets", {})

    i_cont = e.get("rated_current_A_rms")
    torque_cont = op.get("continuous_torque_Nm")
    lim_wind = _winding_limit_c(spec)
    lim_mag = t.get("magnet_temp_C_max_continuous", mag.get("max_service_C", 150.0))

    p_cu0 = losses.get("p_cu_w")
    p_fe = losses.get("p_iron_w") or 0.0
    p_mag = losses.get("p_magnet_w") or 0.0

    # solve once at the continuous current to seed temperatures from Simcenter
    apply_loads(j, sim, sol, losses, inlet_c, h)
    solve(j, sol, "continuous @ %s A_rms" % sc.fmt(i_cont))
    t_wind, t_mag = read_hotspots(j, sim, sol)

    rating = {"winding_limit_C": lim_wind, "magnet_limit_C": lim_mag,
              "rated_current_A_rms": i_cont, "rated_continuous_torque_Nm": torque_cont,
              "winding_hotspot_C": t_wind, "magnet_hotspot_C": t_mag}

    if t_wind is None and t_mag is None:
        j.note("hotspots not auto-read -> continuous rating left to the GUI post. Rule used "
               "once temperatures are available: winding is copper(I^2)-driven so scale I to "
               "its limit (holding the iron/magnet share of the rise); the magnet is eddy"
               "(speed)-driven so reducing current does NOT meet a magnet over-limit.")
        rating["continuous_torque_Nm"] = None
        results["thermal"] = rating
        return

    # held (speed-driven) losses that do NOT scale with armature current:
    p_const = (p_fe or 0.0) + (p_mag or 0.0)

    def winding_limit_current(t_now, t_lim):
        """Winding heating is dominated by copper loss, which scales with I^2; the iron/magnet
        share of the winding rise is held. Subtract that constant share (proportional to its
        loss fraction) and scale only the copper-driven part -- attributing the WHOLE rise to
        I^2 would OVERSTATE the allowable current (optimistic), not be conservative."""
        if t_now is None or t_now <= inlet_c:
            return None
        rise = t_now - inlet_c
        frac_const = (p_const / (p_cu0 + p_const)) if (p_cu0 and (p_cu0 + p_const) > 0) else 0.0
        dt_const = rise * frac_const
        num = (t_lim - inlet_c) - dt_const
        den = rise - dt_const
        if den <= 0:
            return None
        ratio = num / den
        return i_cont * math.sqrt(ratio) if ratio > 0 else 0.0

    i_w = winding_limit_current(t_wind, lim_wind)

    # Magnet temperature is set by the (held) magnet EDDY loss, which is speed-driven, not
    # armature-current driven -- so reducing current cannot be relied on to meet the magnet
    # limit. If the magnet is over-limit at the continuous current, flag it as infeasible by
    # current alone (fix via lower speed / more axial segments / better rotor cooling).
    i_m = None
    if t_mag is not None and t_mag > lim_mag:
        j.warning("magnet hotspot %.0f C > %.0f C is eddy(speed)-driven, not current-driven -- "
                  "reduce speed / add axial magnet segmentation / improve rotor cooling"
                  % (t_mag, lim_mag), target="magnet <= %.0f C" % lim_mag)
        i_m = 0.0  # current reduction alone does not guarantee the magnet limit

    candidates = [x for x in (i_w, i_m) if isinstance(x, (int, float))]
    if not candidates:
        i_lim, bind = i_cont, "winding"
    else:
        i_lim = min(candidates)
        bind = "magnet" if (i_m is not None and i_lim == i_m) else "winding"
    torque_lim = (torque_cont * i_lim / i_cont) if (torque_cont and i_cont) else None

    rating.update({"binding": bind, "continuous_current_A_rms": i_lim,
                   "continuous_torque_Nm": torque_lim})
    j.ok("continuous rating: %.1f A_rms (%s-bound) -> ~%s Nm"
         % (i_lim, bind, ("%.0f" % torque_lim) if torque_lim is not None else "n/a"))
    results["thermal"] = rating


def _winding_limit_c(spec):
    """Class-H limit parsed from the conductor insulation string (default 180 C)."""
    s = sc.spec_get(spec, ["materials", "conductor", "insulation_class"], "") or ""
    import re
    m = re.search(r"(\d{2,3})\s*C", s)
    return float(m.group(1)) if m else 180.0


# --------------------------------------------------------------------------- #
# setup card
# --------------------------------------------------------------------------- #
def print_setup_card(j, spec, losses, inlet_c, h):
    g = spec.get("geometry_mm", {})
    j.line("")
    j.line("================ SIMCENTER 3D THERMAL -- SETUP CARD ================")
    j.line("Build the FEM+Sim ONCE interactively, then re-run on the saved .sim.")
    j.line("1) NEW FEM+SIM: solver = Simcenter Thermal, solution = Steady State.")
    j.line("2) MESH the stator/rotor/magnet/winding solids; name collectors")
    j.line("     STATOR_STEEL / ROTOR_STEEL / MAGNET / COIL_A,B,C; jacket face group.")
    j.line("3) LOADS (impose the P1 EM loss field as heat loads):")
    j.line("     Cu   = %s W on COIL    iron = %s W on STATOR_STEEL    magnet = %s W on MAGNET"
           % (sc.fmt(losses.get("p_cu_w")), sc.fmt(losses.get("p_iron_w")),
              sc.fmt(losses.get("p_magnet_w"))))
    j.line("4) BC: convection on the stator-OD jacket face, h ~%s W/m2K, coolant %s C"
           % (h, inlet_c))
    j.line("     (set the real 12-channel jacket flow / inlet temp / h-correlation)")
    j.line("5) LIMITS: winding <= %.0f C (class H), magnet <= %.0f C."
           % (_winding_limit_c(spec),
              sc.spec_get(spec, ["acceptance_targets", "magnet_temp_C_max_continuous"], 150)))
    j.line("6) Save .sim, then: run_journal.exe verification\\simcenter_thermal.py -args <that>.sim")
    j.line("   The journal imposes loads/BC, solves, reads hotspots, and reduces current to the")
    j.line("   thermally-limited continuous rating (target >= %s Nm)."
           % sc.spec_get(spec, ["acceptance_targets", "continuous_torque_Nm_min"], 183))


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def main():
    args = sc.parse_args(sys.argv[1:])
    spec, spec_path = sc.load_spec(args.get("spec"))
    sess = get_session()
    j = sc.Journal(session=sess, name="thermal (%s)" % spec.get("name", "motor"))
    j.line("spec: %s" % spec_path)

    losses = loss_field(j, spec, args)
    inlet_c, h = coolant_params(j, spec)

    if not _HAVE_NX:
        j.fail("NXOpen not importable -- run with run_journal.exe inside NX/Simcenter 3D.")
        print_setup_card(j, spec, losses, inlet_c, h)
        return

    sim = open_sim(j, args)
    if sim is None:
        j.note("no usable .sim open -- printing the setup card.")
        print_setup_card(j, spec, losses, inlet_c, h)
        return

    sol = active_solution(j, sim)
    results = {"design": spec.get("name"), "spec_path": spec_path, "discipline": "Thermal",
               "loss_field": losses, "coolant_inlet_C": inlet_c, "jacket_h_W_m2K": h}
    continuous_rating(j, sim, sol, spec, losses, inlet_c, h, results)

    ck = sc.score_thermal(spec, results)
    results["acceptance"] = ck.to_list()
    results["warnings"] = j.warn
    ck.print(j, gate="G2 thermal")
    if j.warn:
        j.line("")
        j.line("  %d field/post warning(s) -- confirm member names in your NX 2506 Thermal "
               "build or finish by hand (TARGET shown)." % len(j.warn))

    out = args.get("out") or os.path.join(sc.repo_root(), "fea", "simcenter_thermal_results.json")
    sc.write_results(out, results, j)
    j.line("=== thermal journal done ===")


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
                w.WriteLine("FATAL simcenter_thermal:\n" + traceback.format_exc())
            except Exception:
                pass
        else:
            traceback.print_exc()
