#!/usr/bin/env python
"""P1 EM-FEA verification driver -- FEMM (free / open-source) via pyFEMM.

The license-free counterpart of ``verification/motorcad_emag.py``.  It consumes the
same frozen design and writes the same kind of go/no-go report scored against
``fea_spec.acceptance_targets`` -- but the field solution is computed by FEMM 4.2
(David Meeker, Aladdin Free Public License), so no commercial license is involved.

PIPELINE
    motor_nx (params -> blueprint)  ->  verification.femm_geom  (pure geometry data)
        -> THIS driver emits it into FEMM (mi_* calls), defines materials/circuits,
           and runs a full 54-slot / 6-pole magnetostatic model.  The rotor is one
           FEMM group; cogging / torque / ripple rotate it and re-solve.

STAGES (subset selectable with --stages)
    cogging       no-current torque vs rotor angle over one slot pitch -> pk-pk
    back_emf      open-circuit phase flux-linkage vs angle -> fundamental EMF + THD
    torque_angle  average torque vs current-advance beta at peak current -> MTPA
    ripple        instantaneous torque over one electrical period at MTPA -> ripple %
    demag         peak d-axis-opposing current at hot magnet temp -> min magnet B vs knee

WHAT THIS DOES NOT DO
    It does not invent physics.  FEMM runs the FEA; this script maps the frozen
    design onto FEMM inputs, runs the matrix headless, reads results back, and
    reports against the acceptance gate.  Torque uses FEMM's weighted Maxwell stress
    tensor over a clean air-gap annulus; flux linkage uses mo_getcircuitproperties.

PREREQUISITES
    pip install pyfemm                 # the COM client (already in the venv)
    FEMM 4.2 installed                 # https://www.femm.info/wiki/Download
                                       #   or:  winget install DavidMeeker.FEMM

RUN
    python -m motor_nx.cli fea -o fea/                 # (re)generate the hand-off pkg
    python verification/femm_emag.py --out fea/femm_results.json
    python verification/femm_emag.py --stages cogging,back_emf,torque_angle,ripple,demag
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import femm  # pyFEMM COM client
except Exception as _exc:  # pragma: no cover - depends on the user's environment
    print("ERROR: pyFEMM is not available (%s).\n"
          "       Install it with:  pip install pyfemm\n"
          "       and install FEMM 4.2:  winget install DavidMeeker.FEMM\n"
          "       (or https://www.femm.info/wiki/Download )" % _exc)
    sys.exit(2)

from verification import femm_geom as fg               # noqa: E402

MU0 = 4.0e-7 * math.pi


# --------------------------------------------------------------------------- #
# FEMM session + materials + circuits + boundary
# --------------------------------------------------------------------------- #
def open_femm(show: bool):
    femm.openfemm(0 if show else 1)            # 1 = hide the FEMM window (headless)


def setup_model(spec, model, winding, magnet_temp_c):
    """Build a fresh magnetics document: problem def, materials, circuits, boundary,
    and the emitted geometry.  Used once for the EM stages (20 C) and again, hot, for
    the demag stage (so the derated Br/Hc takes effect)."""
    femm.newdocument(0)                        # 0 = magnetics problem
    define_problem(model.depth_mm)
    mat = define_materials(spec, magnet_temp_c=magnet_temp_c)
    define_circuits()
    define_boundary(model.outer_boundary)
    emit_geometry(model, winding)
    femm.mi_zoomnatural()
    # save the built model so it can be opened in the FEMM GUI for inspection
    # (Mesh > Create Mesh flags any region left without a block label)
    try:
        _femdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fea")
        femm.mi_saveas(os.path.join(_femdir, "femm_model.fem"))
        print("  saved model -> fea/femm_model.fem (open in FEMM, Mesh>Create Mesh to inspect)")
    except Exception as _exc:
        print("  (could not save femm_model.fem: %s)" % _exc)
    return mat


def define_problem(depth_mm: float):
    # magnetostatic, mm, planar, precision 1e-8, depth = active stack, min angle 30
    femm.mi_probdef(0, "millimeters", "planar", 1e-8, depth_mm, 30, 0)


def define_materials(spec, magnet_temp_c=20.0):
    m = spec["materials"]
    # Define Air EXPLICITLY instead of mi_getmaterial("Air"): mi_getmaterial pulls from
    # FEMM's matlib and silently no-ops if the library isn't found, leaving every air
    # region undefined ("Material properties have not been defined for all regions").
    femm.mi_addmaterial(fg.MAT_AIR, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0)

    # --- lamination: nonlinear single-valued BH from fea_spec ------------- #
    lam = m["lamination"]
    lam_d = lam.get("thickness_mm", 0.27)
    lam_fill = lam.get("stacking_factor", 0.96)
    femm.mi_addmaterial(fg.MAT_STEEL, 2500, 2500, 0, 0, 0, lam_d, 0, lam_fill, 0, 0, 0, 0, 0)
    H = lam["BH_H_A_per_m"]; B = lam["BH_B_T"]
    for h, b in zip(H, B):
        femm.mi_addbhpoint(fg.MAT_STEEL, float(b), float(h))

    # --- winding copper (stranded coil; conductivity for reference only) -- #
    cu = m["conductor"]
    sigma_cu = cu.get("conductivity_S_per_m_20C", 5.96e7) / 1e6   # MS/m
    femm.mi_addmaterial(fg.MAT_COPPER, 1, 1, 0, 0, sigma_cu, 0, 0, 1, 0, 0, 0, 0, 0)

    # --- NdFeB magnet: linear recoil, Hc derated to the operating temp ---- #
    mag = m["magnet"]
    br20 = mag.get("Br_T_at_20C", 1.28)
    mu_rec = mag.get("mu_recoil", 1.05)
    br_tc = mag.get("Br_tempco_pct_per_C", -0.12) / 100.0
    br_hot = br20 * (1.0 + br_tc * (magnet_temp_c - 20.0))
    hc = br_hot / (MU0 * mu_rec)                       # A/m (coercivity at this temp)
    sigma_mag = 1.0 / (mag.get("resistivity_uOhm_m", 1.4) * 1e-6) / 1e6   # MS/m
    femm.mi_addmaterial(fg.MAT_MAGNET, mu_rec, mu_rec, hc, 0, sigma_mag, 0, 0, 1, 0, 0, 0, 0, 0)
    return {"br_used_T": br_hot, "Hc_used_A_per_m": hc, "magnet_temp_C": magnet_temp_c}


def define_circuits():
    for name in ("A", "B", "C"):
        femm.mi_addcircprop(name, 0.0, 1)     # 1 = series


def define_boundary(name):
    # Prescribed A = 0 on the stator OD (BdryFormat 0)
    femm.mi_addboundprop(name, 0, 0, 0, 0, 0, 0, 0, 0, 0)


# --------------------------------------------------------------------------- #
# geometry emission
# --------------------------------------------------------------------------- #
def emit_geometry(model, winding):
    """Replay the femm_geom model into FEMM and set entity groups / boundaries /
    block properties.  ``winding`` = (turns_scale, current_scale) folds the parallel
    paths into the turns (so the FEMM circuit's flux linkage is the terminal value)."""
    turns_scale, _current_scale = winding

    # Add every unique endpoint NODE first. FEMM's mi_addsegment/mi_addarc bind to the
    # NEAREST existing node; the pure-circle OD / airgap-ring arc endpoints are not shared
    # by any segment, so without a real node there mi_addarc snaps to a distant node and
    # throws "Internal application error". Pre-creating the nodes makes the binding exact.
    _added = set()

    def _node(x, y):
        k = (round(float(x), 5), round(float(y), 5))
        if k in _added:
            return
        _added.add(k)
        try:
            femm.mi_addnode(float(x), float(y))
        except Exception:
            pass

    for s in model.segs:
        _node(s.x1, s.y1)
        _node(s.x2, s.y2)
    for a in model.arcs:
        _node(a.x1, a.y1)
        _node(a.x2, a.y2)

    for s in model.segs:
        femm.mi_addsegment(s.x1, s.y1, s.x2, s.y2)
    arc_fail = 0
    for a in model.arcs:
        try:
            femm.mi_addarc(a.x1, a.y1, a.x2, a.y2, a.angle_deg, a.maxseg_deg)
        except Exception as exc:
            arc_fail += 1
            if arc_fail <= 5:
                print("  WARN arc (%.2f,%.2f)->(%.2f,%.2f) ang=%.2f failed: %s"
                      % (a.x1, a.y1, a.x2, a.y2, a.angle_deg, exc))
    if arc_fail:
        print("  %d/%d arc(s) failed -- geometry may be incomplete" % (arc_fail, len(model.arcs)))

    # set segment groups / boundaries (only where non-default to save COM calls)
    for s in model.segs:
        if s.group == 0 and not s.bdry:
            continue
        mx, my = 0.5 * (s.x1 + s.x2), 0.5 * (s.y1 + s.y2)
        femm.mi_selectsegment(mx, my)
        femm.mi_setsegmentprop(s.bdry or "<None>", 0, 1, 0, s.group)
        femm.mi_clearselected()

    # set arc groups / boundaries
    for a in model.arcs:
        if a.group == 0 and not a.bdry:
            continue
        mid = fg._rot(a.x1, a.y1, a.angle_deg / 2.0)     # all arcs are centred at origin
        femm.mi_selectarcsegment(mid[0], mid[1])
        femm.mi_setarcsegmentprop(a.maxseg_deg, a.bdry or "<None>", 0, a.group)
        femm.mi_clearselected()

    # block labels
    for lb in model.labels:
        femm.mi_addblocklabel(lb.x, lb.y)
        femm.mi_selectlabel(lb.x, lb.y)
        incircuit = lb.circuit if lb.circuit else "<None>"
        turns = int(round(lb.turns * turns_scale)) if lb.material == fg.MAT_COPPER else 0
        automesh = 1 if lb.meshsize <= 0 else 0
        femm.mi_setblockprop(lb.material, automesh, lb.meshsize, incircuit,
                             lb.magdir_deg, lb.group, turns)
        femm.mi_clearselected()


# --------------------------------------------------------------------------- #
# solve helpers
# --------------------------------------------------------------------------- #
class Solver:
    def __init__(self, model, fem_path):
        self.model = model
        self.fem_path = fem_path
        self.rotor_angle = 0.0          # absolute rotor mechanical angle (deg)
        self.solves = 0

    def rotate_to(self, angle_deg):
        d = angle_deg - self.rotor_angle
        if abs(d) > 1e-9:
            femm.mi_selectgroup(fg.GROUP_ROTOR)
            femm.mi_moverotate(0, 0, d)
            femm.mi_clearselected()
            self.rotor_angle = angle_deg

    def set_currents(self, ia, ib, ic):
        femm.mi_setcurrent("A", ia)
        femm.mi_setcurrent("B", ib)
        femm.mi_setcurrent("C", ic)

    def solve(self):
        femm.mi_saveas(self.fem_path)
        femm.mi_analyze(1)              # 1 = no FEMM window during solve
        femm.mi_loadsolution()
        self.solves += 1

    def torque(self):
        """Weighted Maxwell stress tensor torque (N*m) over the clean inner gap ring."""
        r = self.model.radii["torque_ring_r"]
        femm.mo_clearblock()
        femm.mo_selectblock(r, 0.0)
        t = femm.mo_blockintegral(22)
        femm.mo_clearblock()
        return float(t)

    def flux_linkages(self):
        """Terminal phase flux linkages (Wb-turn) from the circuit properties."""
        out = {}
        for name in ("A", "B", "C"):
            props = femm.mo_getcircuitproperties(name)   # (current, volts, flux)
            out[name] = float(props[2])
        return out

    def min_magnet_B(self):
        """Minimum flux density magnitude inside the magnet blocks (demag proxy)."""
        bmin = None
        for lb in self.model.labels:
            if lb.material != fg.MAT_MAGNET:
                continue
            # sample at the (rotated) magnet centroid
            x, y = fg._rot(lb.x, lb.y, self.rotor_angle)
            bx, by = femm.mo_getb(x, y)
            b = math.hypot(bx, by)
            bmin = b if bmin is None else min(bmin, b)
        return bmin


# --------------------------------------------------------------------------- #
# excitation: 3-phase instantaneous currents
# --------------------------------------------------------------------------- #
def phase_currents(i_amp, theta_e_deg, beta_deg):
    """Balanced 3-phase currents for a current vector at (q-axis + beta) electrical.

    theta_e_deg = electrical angle of the rotor d-axis from the phase-A axis
    (= pole_pairs * rotor_mech_angle).  beta = advance from the q-axis (toward -d,
    i.e. field weakening).  Phase-A axis is taken at +X (slot 0)."""
    ang = math.radians(theta_e_deg + 90.0 + beta_deg)
    ia = i_amp * math.cos(ang)
    ib = i_amp * math.cos(ang - 2.0 * math.pi / 3.0)
    ic = i_amp * math.cos(ang + 2.0 * math.pi / 3.0)
    return ia, ib, ic


def _peak_amp(rms):
    return float(rms) * math.sqrt(2.0)


def _fundamental(samples_angle_deg, values):
    """Return (amp1, phase1_rad) of the fundamental (one cycle over the sampled span)."""
    n = len(values)
    if n < 2:
        return 0.0, 0.0
    span = math.radians(samples_angle_deg[-1] - samples_angle_deg[0]) * n / (n - 1)
    a = b = 0.0
    for k, v in enumerate(values):
        th = 2.0 * math.pi * k / n
        a += v * math.cos(th)
        b += v * math.sin(th)
    a *= 2.0 / n; b *= 2.0 / n
    return math.hypot(a, b), math.atan2(b, a)


def _thd(values):
    """THD from the DFT of one period of samples (harmonics 2.. / fundamental)."""
    n = len(values)
    if n < 4:
        return None
    amps = []
    for h in range(1, n // 2):
        a = b = 0.0
        for k, v in enumerate(values):
            th = 2.0 * math.pi * h * k / n
            a += v * math.cos(th); b += v * math.sin(th)
        amps.append(math.hypot(a, b) * 2.0 / n)
    if not amps or amps[0] < 1e-12:
        return None
    rest = math.sqrt(sum(a * a for a in amps[1:]))
    return rest / amps[0] * 100.0


def _peak_to_peak(series):
    return (max(series) - min(series)) if series else 0.0


# --------------------------------------------------------------------------- #
# stages
# --------------------------------------------------------------------------- #
def stage_cogging(slv, spec, results, n_steps=24):
    print("[cogging] no-current torque vs rotor angle (1 slot pitch) ...")
    slot_pitch = 360.0 / spec["geometry_mm"]["slots"]
    slv.set_currents(0.0, 0.0, 0.0)
    angs, tq = [], []
    for i in range(n_steps + 1):
        a = slot_pitch * i / n_steps
        slv.rotate_to(a)
        slv.solve()
        t = slv.torque()
        angs.append(a); tq.append(t)
        print("    rotor=%.3f deg  T_cog=%+.4f Nm" % (a, t))
    slv.rotate_to(0.0)
    pp = _peak_to_peak(tq)
    results["cogging"] = {"pk_pk_Nm": pp, "n_points": len(tq),
                          "angle_deg": angs, "torque_Nm": tq}
    print("  cogging pk-pk = %.4f Nm" % pp)


def stage_back_emf(slv, spec, results, n_steps=36):
    print("[back_emf] open-circuit flux linkage vs rotor angle ...")
    op = spec["operating_points"]
    pole_pairs = spec["geometry_mm"]["poles"] / 2.0
    speed_rpm = op.get("base_speed_rpm", 1000)
    omega_e = 2.0 * math.pi * speed_rpm / 60.0 * pole_pairs   # elec rad/s
    slv.set_currents(0.0, 0.0, 0.0)
    elec_period_mech = 360.0 / pole_pairs                     # one electrical period in mech deg
    angs, la, lb, lc = [], [], [], []
    for i in range(n_steps):
        a = elec_period_mech * i / n_steps
        slv.rotate_to(a)
        slv.solve()
        fl = slv.flux_linkages()
        angs.append(a); la.append(fl["A"]); lb.append(fl["B"]); lc.append(fl["C"])
    slv.rotate_to(0.0)
    amp1, _ = _fundamental(angs, la)
    e_ph_peak = omega_e * amp1                                # V (peak phase EMF)
    e_ll_peak = math.sqrt(3.0) * e_ph_peak
    thd = _thd(la)
    results["back_emf"] = {
        "speed_rpm": speed_rpm, "lambda_pk_Wb": amp1,
        "phase_emf_peak_V": e_ph_peak, "line_line_peak_V": e_ll_peak,
        "thd_pct": thd, "flux_linkage_A_Wb": la, "angle_deg": angs,
    }
    print("  lambda1=%.4f Wb -> phase EMF peak=%.1f V, L-L peak=%.1f V, THD=%s %%"
          % (amp1, e_ph_peak, e_ll_peak, _fmt(thd)))


def stage_torque_angle(slv, spec, results, i_amp, n_pos=8):
    print("[torque_angle] average torque vs beta at peak current (MTPA) ...")
    e = spec["excitation"]
    pole_pairs = spec["geometry_mm"]["poles"] / 2.0
    betas = e.get("current_advance_angle_deg_from_q_axis", [0, 10, 20, 30, 40])
    elec_period_mech = 360.0 / pole_pairs
    locus = []
    for beta in betas:
        tqs = []
        for j in range(n_pos):
            a = elec_period_mech * j / n_pos
            slv.rotate_to(a)
            theta_e = pole_pairs * a
            ia, ib, ic = phase_currents(i_amp, theta_e, beta)
            slv.set_currents(ia, ib, ic)
            slv.solve()
            tqs.append(slv.torque())
        avg = sum(tqs) / len(tqs)
        locus.append({"beta_deg": beta, "avg_torque_Nm": avg,
                      "ripple_Nm": _peak_to_peak(tqs)})
        print("    beta=%-4s  T_avg=%+.1f Nm  (ripple %.1f Nm)" % (beta, avg, _peak_to_peak(tqs)))
    slv.rotate_to(0.0); slv.set_currents(0, 0, 0)
    valid = [pt for pt in locus if isinstance(pt["avg_torque_Nm"], (int, float))]
    best = max(valid, key=lambda pt: abs(pt["avg_torque_Nm"])) if valid else None
    results["torque_angle"] = {"peak_current_A": i_amp, "locus": locus, "mtpa": best}
    if best:
        print("  MTPA: beta=%s deg -> peak torque %.1f Nm"
              % (best["beta_deg"], best["avg_torque_Nm"]))
    return best


def stage_ripple(slv, spec, results, i_amp, mtpa, n_steps=24):
    print("[ripple] instantaneous torque over 1 electrical period at MTPA ...")
    pole_pairs = spec["geometry_mm"]["poles"] / 2.0
    beta = (mtpa or {}).get("beta_deg", 15)
    elec_period_mech = 360.0 / pole_pairs
    angs, tq = [], []
    for i in range(n_steps):
        a = elec_period_mech * i / n_steps
        slv.rotate_to(a)
        theta_e = pole_pairs * a
        ia, ib, ic = phase_currents(i_amp, theta_e, beta)
        slv.set_currents(ia, ib, ic)
        slv.solve()
        tq.append(slv.torque()); angs.append(a)
    slv.rotate_to(0.0); slv.set_currents(0, 0, 0)
    avg = sum(tq) / len(tq) if tq else 0.0
    pp = _peak_to_peak(tq)
    ripple_pct = (pp / abs(avg) * 100.0) if avg else 0.0
    results["ripple"] = {"beta_deg": beta, "avg_Nm": avg, "pk_pk_Nm": pp,
                         "ripple_pct": ripple_pct, "torque_Nm": tq, "angle_deg": angs}
    print("  avg %.1f Nm, ripple pk-pk %.2f Nm = %.2f %%" % (avg, pp, ripple_pct))


def stage_demag(slv, spec, results, i_amp_peak, magnet_temp_c):
    """Worst-case demag: full peak current on the d-axis OPPOSING the magnets, at the
    hot magnet temperature (materials already rebuilt with the derated Br/Hc).  Report
    the minimum magnet flux density vs an estimated knee."""
    print("[demag] peak d-axis-opposing current at %.0f C ..." % magnet_temp_c)
    pole_pairs = spec["geometry_mm"]["poles"] / 2.0
    slv.rotate_to(0.0)                       # d-axis of reference pole on +X
    theta_e = 0.0
    # beta = 90 deg from q-axis  => pure -d current (most demagnetising)
    ia, ib, ic = phase_currents(i_amp_peak, theta_e, 90.0)
    slv.set_currents(ia, ib, ic)
    slv.solve()
    bmin = slv.min_magnet_B()
    slv.set_currents(0, 0, 0)
    mag = spec["materials"]["magnet"]
    # crude knee estimate: SH-class NdFeB stays linear to a low B at 150 C; use a
    # conservative 0.20 T knee unless the spec carries one. (A rigorous knee needs the
    # nonlinear 2nd-quadrant BH at temperature -- noted as a limitation.)
    b_knee = mag.get("demag_knee_T_at_hot", 0.20)
    safe = (bmin is not None and bmin > b_knee)
    results["demag"] = {"magnet_temp_C": magnet_temp_c, "min_magnet_B_T": bmin,
                        "knee_T_assumed": b_knee, "safe": safe,
                        "current_A_peak": i_amp_peak}
    print("  min magnet B = %s T  (knee ~%.2f T)  -> %s"
          % (_fmt(bmin), b_knee, "SAFE" if safe else "RISK"))


# --------------------------------------------------------------------------- #
# acceptance gate (same structure as motorcad_emag.evaluate)
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
            abs(peak_tq) >= t["peak_torque_Nm_min"], "MTPA peak = %.1f Nm" % abs(peak_tq))

    rip = (results.get("ripple") or {}).get("ripple_pct")
    if rip is not None:
        chk("torque_ripple <= %s %%" % t["torque_ripple_pct_max"],
            rip <= t["torque_ripple_pct_max"], "ripple = %.2f %%" % rip)

    cog = (results.get("cogging") or {}).get("pk_pk_Nm")
    if cog is not None:
        cog_pct = cog / rated * 100.0
        chk("cogging <= %s %% of rated" % t["cogging_pct_of_rated_max"],
            cog_pct <= t["cogging_pct_of_rated_max"],
            "cogging = %.4f Nm = %.3f %% of %.0f Nm" % (cog, cog_pct, rated))

    dem = results.get("demag") or {}
    if dem.get("safe") is not None:
        chk("no demag @ peak + %s C" % dem.get("magnet_temp_C"),
            dem["safe"], "min magnet B = %s T (knee ~%s T)"
            % (_fmt(dem.get("min_magnet_B_T")), _fmt(dem.get("knee_T_assumed"))))

    bemf = results.get("back_emf") or {}
    if bemf.get("line_line_peak_V") is not None:
        vdc = spec["operating_points"].get("dc_bus_V", 400)
        chk("back-EMF L-L peak <= Vdc (%s V) at base speed" % vdc,
            bemf["line_line_peak_V"] <= vdc * 1.02,
            "L-L peak = %.0f V at %s rpm" % (bemf["line_line_peak_V"], bemf.get("speed_rpm")))

    return checks


# --------------------------------------------------------------------------- #
def _fmt(x):
    return ("%.3f" % x) if isinstance(x, (int, float)) else str(x)


ALL_STAGES = ["cogging", "back_emf", "torque_angle", "ripple", "demag"]


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    ap = argparse.ArgumentParser(description="P1 EM-FEA verification in FEMM (pyFEMM)")
    ap.add_argument("--spec", default=os.path.join(repo, "fea", "fea_spec.json"))
    ap.add_argument("--out", default=os.path.join(repo, "fea", "femm_results.json"))
    ap.add_argument("--stages", default="cogging,back_emf,torque_angle,ripple,demag",
                    help="comma list from: %s" % ",".join(ALL_STAGES))
    ap.add_argument("--show", action="store_true", help="show the FEMM window (debug)")
    ap.add_argument("--coarse", action="store_true",
                    help="fewer rotor steps (quick smoke run)")
    args = ap.parse_args(argv)

    with open(args.spec, "r", encoding="utf-8") as fh:
        spec = json.load(fh)
    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    print("=== FEMM EM verification: %s ===" % spec.get("name", "motor"))
    print("spec: %s   stages: %s\n" % (args.spec, ", ".join(stages)))

    # winding parallel-path folding: keep the FEMM circuit's flux linkage = terminal
    w = spec["winding"]
    cps = int(w.get("conductors_per_slot", 8))
    par = int(w.get("parallel_paths", 1))
    if cps % par == 0:
        turns_scale, current_scale = 1.0 / par, 1.0   # turns/=par, I = full phase current
    else:
        turns_scale, current_scale = 1.0, 1.0 / par   # turns kept, I = phase/par
    print("winding: %d bars/slot, %d parallel paths -> turns_scale=%.3f current_scale=%.3f"
          % (cps, par, turns_scale, current_scale))

    t0 = time.time()
    model = fg.build()
    fem_path = os.path.join(os.path.dirname(args.out), "femm_model.fem")
    winding = (turns_scale, current_scale)

    # demag rebuilds materials hot; everything else is at 20 C ambient for EM
    open_femm(args.show)
    setup_model(spec, model, winding, magnet_temp_c=20.0)
    print("geometry emitted: %d segs, %d arcs, %d labels"
          % (len(model.segs), len(model.arcs), len(model.labels)))

    slv = Solver(model, fem_path)
    results = {"design": spec.get("name"), "tool": "FEMM 4.2 (pyFEMM)", "spec_path": args.spec}

    i_peak = _peak_amp(spec["excitation"].get("peak_current_A_rms", 273)) * current_scale
    nrot = 12 if args.coarse else 24

    mtpa = None
    if "cogging" in stages:
        stage_cogging(slv, spec, results, n_steps=max(12, nrot))
    if "back_emf" in stages:
        stage_back_emf(slv, spec, results, n_steps=max(12, nrot + 12))
    if "torque_angle" in stages:
        mtpa = stage_torque_angle(slv, spec, results, i_peak, n_pos=6 if args.coarse else 8)
    if "ripple" in stages:
        stage_ripple(slv, spec, results, i_peak, mtpa, n_steps=max(12, nrot))
    if "demag" in stages:
        # rebuild magnet material hot, then re-emit?  Material change is global; rebuild
        # the whole model so the derated Br/Hc takes effect.
        hot = spec["materials"]["magnet"].get("max_service_C", 150.0)
        femm.mi_close()
        setup_model(spec, model, winding, magnet_temp_c=hot)
        slv = Solver(model, fem_path)
        stage_demag(slv, spec, results, i_peak, hot)

    checks = evaluate(spec, results)
    results["acceptance"] = checks
    results["solves"] = slv.solves
    results["elapsed_s"] = round(time.time() - t0, 1)

    print("\n--- ACCEPTANCE (G1 gate, vs fea_spec.acceptance_targets) ---")
    for c in checks:
        flag = {True: "PASS", False: "FAIL", None: "n/a "}[c["pass"]]
        print("  [%s] %-40s %s" % (flag, c["criterion"], c["detail"]))

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2)
    print("\nresults -> %s   (%d solves, %.1fs)" % (args.out, slv.solves, results["elapsed_s"]))

    if not args.show:
        femm.closefemm()
    return 0


if __name__ == "__main__":
    sys.exit(main())
