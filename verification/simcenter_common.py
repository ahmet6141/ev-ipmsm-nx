"""Shared, NX-independent layer for the Simcenter 3D verification journals.

This module holds everything the Simcenter 3D journals need that does NOT depend
on NXOpen, so it can be imported and unit-checked without an NX license:

  * ``load_spec``      -- read the ``fea/fea_spec.json`` hand-off package.
  * ``Journal``        -- the guarded, logged call wrapper every journal uses.
                          It writes to the NX Listing Window when run inside NX
                          (via run_journal.exe) and falls back to stdout otherwise;
                          a single drifted NXOpen member name degrades to a warning
                          (collected in ``.warn``) instead of aborting the study --
                          the same hardening as nx_builder.py / motorcad_emag.py.
  * ``parse_args``     -- the run_journal.exe argument convention shared by all
                          journals (``.sim`` / ``.fem`` / ``.prt`` model file,
                          ``spec=...`` / a bare ``.json`` spec, ``out=...`` result
                          file, and a bare comma list of stages).
  * ``score_em`` / ``score_thermal`` / ``score_structural`` -- the acceptance
                          gates, scored against ``fea_spec.acceptance_targets``.
                          Pure functions so the orchestrator and the unit self-test
                          can reuse them with no NX in the loop.
  * small math/format helpers (``peak_amp``, ``thd``, ``pk_pk``, ``spec_get`` ...).

Run it directly for a license-free self-check of the pure logic::

    python verification/simcenter_common.py

WHAT THE JOURNALS DO *NOT* DO
    They do not invent physics. Simcenter 3D runs the FEA; the journals map the
    frozen design (fea_spec.json) onto Simcenter inputs, drive the solve, read the
    results back and report go / no-go against the acceptance gate. The numbers are
    Simcenter's, not ours.
"""
from __future__ import annotations

import json
import math
import os

# --------------------------------------------------------------------------- #
# paths / spec loading
# --------------------------------------------------------------------------- #
def repo_root():
    """Repository root (parent of this verification/ directory)."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_spec_path():
    return os.path.join(repo_root(), "fea", "fea_spec.json")


def load_spec(path=None):
    """Load fea_spec.json (defaults to fea/fea_spec.json)."""
    path = path or default_spec_path()
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh), path


# --------------------------------------------------------------------------- #
# guarded / logged journal wrapper
#   Models the MC wrapper in motorcad_emag.py and the ListingWindow logging in
#   nx_drafting.py: every NXOpen step is attempted, the outcome is logged, and a
#   failure becomes a collected warning (with the TARGET value, so a missed field
#   can be finished by hand) rather than an exception that aborts the run.
# --------------------------------------------------------------------------- #
class Journal:
    def __init__(self, session=None, name="simcenter"):
        self.session = session
        self.name = name
        self.warn = []
        self.lw = None
        if session is not None:
            try:
                self.lw = session.ListingWindow
                self.lw.Open()
            except Exception:
                self.lw = None
        self.line("=== motor_nx Simcenter 3D journal: %s ===" % name)

    # ---- logging --------------------------------------------------------- #
    def line(self, msg):
        if self.lw is not None:
            try:
                self.lw.WriteLine(msg)
                return
            except Exception:
                pass
        print(msg)

    def ok(self, msg):
        self.line("OK   " + msg)

    def note(self, msg):
        self.line("NOTE " + msg)

    def fail(self, msg):
        self.line("FAIL " + msg)

    def warning(self, msg, target=None):
        m = msg + (("  TARGET=%s" % (target,)) if target is not None else "")
        self.warn.append(m)
        self.line("WARN " + m)

    # ---- guarded calls --------------------------------------------------- #
    def do(self, label, fn, *args, **kwargs):
        """Call ``fn(*args, **kwargs)`` guarded. Logs OK/WARN and returns the
        result, or None on failure. ``target=`` (kw-only) is logged on failure."""
        target = kwargs.pop("target", None)
        if fn is None:
            self.warning("%s skipped (callable not available in this NX version)" % label,
                         target=target)
            return None
        try:
            res = fn(*args, **kwargs)
            self.ok(label)
            return res
        except Exception as exc:
            self.warning("%s failed (%s)" % (label, _short(exc)), target=target)
            return None

    def method(self, obj, names, *args, **kwargs):
        """Call the first method in ``names`` that exists on ``obj``, guarded.
        Tolerates NXOpen method renames across releases."""
        label = kwargs.pop("label", None)
        target = kwargs.pop("target", None)
        if isinstance(names, str):
            names = [names]
        for nm in names:
            fn = getattr(obj, nm, None)
            if callable(fn):
                return self.do(label or nm, fn, *args, target=target, **kwargs)
        self.warning("%s skipped (none of %s on %s)"
                     % (label or names[0], names, type(obj).__name__), target=target)
        return None

    def set_attr(self, obj, names, value, label=None, target=None):
        """Set the first settable attribute in ``names`` on ``obj`` to ``value``.
        Tolerates builder-property renames (the setattr-loop pattern in
        nx_drafting.add_sheet). Logs the TARGET value on a total miss."""
        if isinstance(names, str):
            names = [names]
        for nm in names:
            try:
                setattr(obj, nm, value)
                self.ok("%s = %s" % (label or nm, _fmt(value)))
                return True
            except Exception:
                continue
        self.warning("set %s failed (no settable name in %s)" % (label or names[0], names),
                     target=target if target is not None else value)
        return False

    def get_attr(self, obj, names, default=None):
        """Read the first attribute/zero-arg getter in ``names`` from ``obj``."""
        if isinstance(names, str):
            names = [names]
        for nm in names:
            try:
                v = getattr(obj, nm)
            except Exception:
                continue
            try:
                return v() if callable(v) else v
            except Exception:
                continue
        return default

    def commit(self, builder, label=""):
        """Commit a builder guarded, then Destroy it in a finally (the
        create -> set -> Commit -> Destroy pattern used throughout nx_drafting)."""
        obj = None
        if builder is None:
            self.warning("%s commit skipped (builder is None)" % label)
            return None
        try:
            obj = builder.Commit()
            self.ok("%s committed" % label)
        except Exception as exc:
            self.warning("%s commit failed (%s)" % (label, _short(exc)))
        finally:
            try:
                builder.Destroy()
            except Exception:
                pass
        return obj


def _short(exc, limit=160):
    """First line / clipped form of an exception (Motor-CAD/Ansys errors are huge)."""
    s = str(exc).strip().splitlines()
    s = s[0] if s else ""
    return s if len(s) <= limit else s[:limit] + " ..."


# --------------------------------------------------------------------------- #
# run_journal.exe argument convention (shared by every journal)
#   "%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_xxx.py \
#       -args motor_v8.sim cogging,demag spec=fea\fea_spec.json out=fea\sc_xxx.json
# --------------------------------------------------------------------------- #
def parse_args(argv, all_stages=None):
    """Parse the bare-token argument list run_journal passes after -args.

      *.sim / *.fem / *.prt   -> the model file to open (sim preferred)
      spec=<path>             -> the fea_spec path (default fea/fea_spec.json)
      losses=<path>           -> an EM-results JSON carrying a loss field (thermal)
      out=<path>              -> the results JSON to write
      stages=a,b,c | a bare comma list | any token equal to a known stage name
                              -> the stage subset to run
      a bare .json            -> the fea_spec path (unless it looks like a result/out file)
    """
    all_stages = all_stages or []
    out = {"sim": None, "fem": None, "prt": None, "spec": None, "out": None,
           "losses": None, "stages": None, "extra": []}
    for a in argv:
        al = a.lower()
        if al.endswith(".sim"):
            out["sim"] = a
        elif al.endswith(".fem"):
            out["fem"] = a
        elif al.endswith(".prt"):
            out["prt"] = a
        # key=value tokens are matched BEFORE the bare-".json" branch so that e.g.
        # losses=foo_results.json is not mis-captured as the spec/out file.
        elif al.startswith("spec="):
            out["spec"] = a.split("=", 1)[1]
        elif al.startswith("losses="):
            out["losses"] = a.split("=", 1)[1]
        elif al.startswith("out="):
            out["out"] = a.split("=", 1)[1]
        elif al.startswith("stages="):
            out["stages"] = [s.strip() for s in a.split("=", 1)[1].split(",") if s.strip()]
        elif al.endswith(".json"):
            # a bare .json is the spec unless it looks like a result/out file
            if out["spec"] is None and "result" not in al and "out" not in al:
                out["spec"] = a
            elif out["out"] is None:
                out["out"] = a
        elif "," in a or a.strip() in all_stages:
            out["stages"] = [s.strip() for s in a.split(",") if s.strip()]
        else:
            out["extra"].append(a)
    return out


# --------------------------------------------------------------------------- #
# math / format helpers
# --------------------------------------------------------------------------- #
def peak_amp(rms):
    """RMS -> peak amplitude (sqrt2). Pass-through for non-numbers."""
    return (float(rms) * math.sqrt(2.0)) if isinstance(rms, (int, float)) else rms


def pk_pk(series):
    return (max(series) - min(series)) if series else 0.0


def mean(series):
    return (sum(series) / len(series)) if series else 0.0


def thd(orders, amps):
    """Total harmonic distortion (%) from harmonic (order, amplitude) lists,
    fundamental = order 1."""
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


def rpm_to_rad_s(rpm):
    return float(rpm) * 2.0 * math.pi / 60.0


def hz_to_rpm(freq_hz):
    return float(freq_hz) * 60.0


def electrical_freq_hz(poles, rpm):
    return (poles / 2.0) * float(rpm) / 60.0


def spec_get(spec, path, default=None):
    """Nested ``spec[a][b][c]`` lookup that never raises."""
    cur = spec
    for k in path:
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        else:
            return default
    return cur


def _fmt(x):
    if isinstance(x, float):
        return ("%.4g" % x)
    return str(x)


def fmt(x):
    return _fmt(x) if isinstance(x, (int, float)) else str(x)


# --------------------------------------------------------------------------- #
# acceptance scoring against fea_spec.acceptance_targets
#   Each journal fills a results dict; these turn it into a list of
#   {criterion, pass, detail}. pass=None means "not measured this run".
# --------------------------------------------------------------------------- #
class Checks:
    def __init__(self):
        self.items = []

    def add(self, criterion, passed, detail):
        self.items.append({
            "criterion": criterion,
            "pass": (bool(passed) if passed is not None else None),
            "detail": detail,
        })

    def to_list(self):
        return self.items

    def print(self, journal, gate=""):
        journal.line("")
        journal.line("--- ACCEPTANCE %s(vs fea_spec.acceptance_targets) ---"
                     % (("(%s) " % gate) if gate else ""))
        for c in self.items:
            flag = {True: "PASS", False: "FAIL", None: "n/a "}[c["pass"]]
            journal.line("  [%s] %-44s %s" % (flag, c["criterion"], c["detail"]))


def score_em(spec, results):
    """G1 EM gate: peak torque, ripple, cogging, demag, efficiency."""
    t = spec.get("acceptance_targets", {})
    rated = spec_get(spec, ["operating_points", "continuous_torque_Nm"], 0) or 1
    ck = Checks()

    mtpa = (results.get("torque_angle") or {}).get("mtpa") or {}
    peak_tq = mtpa.get("avg_torque_Nm")
    if peak_tq is not None and "peak_torque_Nm_min" in t:
        ck.add("peak torque >= %s Nm" % t["peak_torque_Nm_min"],
               peak_tq >= t["peak_torque_Nm_min"], "MTPA peak = %.1f Nm" % peak_tq)

    rip = (results.get("ripple") or {}).get("ripple_pct")
    if rip is not None and "torque_ripple_pct_max" in t:
        ck.add("torque ripple <= %s %%" % t["torque_ripple_pct_max"],
               rip <= t["torque_ripple_pct_max"], "ripple = %.2f %%" % rip)

    cog = (results.get("cogging") or {}).get("pk_pk_Nm")
    if cog is not None and "cogging_pct_of_rated_max" in t:
        cog_pct = cog / rated * 100.0
        ck.add("cogging <= %s %% of rated" % t["cogging_pct_of_rated_max"],
               cog_pct <= t["cogging_pct_of_rated_max"],
               "cogging = %.3f Nm = %.2f %% of %.0f Nm" % (cog, cog_pct, rated))

    dem = results.get("demag") or {}
    if dem.get("irreversible") is not None:
        ck.add("no demag @ peak + %s C" % dem.get("magnet_temp_C"),
               not dem["irreversible"],
               "demag proportion = %s, min B = %s T"
               % (fmt(dem.get("demag_proportion")), fmt(dem.get("min_magnet_B_T"))))

    eff = (results.get("efficiency") or {}).get("peak_pct")
    if eff is not None and "peak_efficiency_pct_min" in t:
        ck.add("peak efficiency >= %s %%" % t["peak_efficiency_pct_min"],
               eff >= t["peak_efficiency_pct_min"], "peak eff = %.1f %%" % eff)
    return ck


def score_thermal(spec, results):
    """G2 thermal gate: continuous torque, magnet <=150 C, winding <=180 C."""
    t = spec.get("acceptance_targets", {})
    ck = Checks()
    th = results.get("thermal") or {}

    cont = th.get("continuous_torque_Nm")
    if cont is not None and "continuous_torque_Nm_min" in t:
        ck.add("continuous torque >= %s Nm" % t["continuous_torque_Nm_min"],
               cont >= t["continuous_torque_Nm_min"], "continuous = %.1f Nm" % cont)

    t_mag = th.get("magnet_hotspot_C")
    lim_mag = t.get("magnet_temp_C_max_continuous")
    if t_mag is not None and lim_mag is not None:
        ck.add("magnet hotspot <= %s C" % lim_mag,
               t_mag <= lim_mag, "magnet = %.1f C" % t_mag)

    t_wind = th.get("winding_hotspot_C")
    lim_wind = th.get("winding_limit_C")
    if t_wind is not None and lim_wind is not None:
        ck.add("winding hotspot <= %s C (class limit)" % lim_wind,
               t_wind <= lim_wind, "winding = %.1f C" % t_wind)
    return ck


def score_structural(spec, results):
    """G3 structural gate: rotor von Mises SF >=1.5 @1.2x; 1st bending > max speed."""
    t = spec.get("acceptance_targets", {})
    ck = Checks()

    mech = results.get("rotor_stress") or {}
    sf = mech.get("safety_factor")
    sf_min = t.get("rotor_vonMises_safety_factor_min_at_1.2x_maxspeed")
    if sf is not None and sf_min is not None:
        ck.add("rotor SF >= %s @ %.0f rpm" % (sf_min, mech.get("overspeed_rpm", 0)),
               sf >= sf_min, "SF = %.2f (max von Mises %.0f MPa)"
               % (sf, mech.get("max_vonMises_MPa", float("nan"))))

    rd = results.get("rotordynamics") or {}
    f1 = rd.get("first_bending_Hz")
    nmax = spec_get(spec, ["operating_points", "max_speed_rpm"], None)
    if f1 is not None and nmax is not None:
        crit_rpm = hz_to_rpm(f1)
        ck.add("1st bending critical speed > %s rpm" % nmax,
               crit_rpm > nmax, "1st bending %.0f Hz = %.0f rpm" % (f1, crit_rpm))
    return ck


# --------------------------------------------------------------------------- #
# results IO
# --------------------------------------------------------------------------- #
def write_results(path, results, journal=None):
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    except Exception:
        pass
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2)
    if journal is not None:
        journal.line("results -> %s" % path)


# --------------------------------------------------------------------------- #
# license-free self-check
# --------------------------------------------------------------------------- #
def _selftest():
    spec, path = load_spec()
    print("loaded %s : %s" % (path, spec.get("name")))
    print("operating points :", spec.get("operating_points"))
    print("acceptance gate  :", json.dumps(spec.get("acceptance_targets"), indent=2))
    print("analysis matrix  :", [a["id"] for a in spec.get("analyses", [])])

    j = Journal(session=None, name="selftest")
    # exercise the guarded wrappers with a dummy object
    class Dummy:
        pass
    d = Dummy()
    j.set_attr(d, ["NoSuchName", "AlsoMissing"], 42, label="missing field", target=42)
    j.set_attr(d, ["Value"], 7, label="Value")
    j.do("intentional failure", lambda: 1 / 0, target="should be logged")
    print("collected warnings:", len(j.warn))

    # exercise the scorers with a synthetic results dict
    em = {"torque_angle": {"mtpa": {"avg_torque_Nm": 441.0, "beta_deg": 20}},
          "ripple": {"ripple_pct": 4.2}, "cogging": {"pk_pk_Nm": 1.1},
          "demag": {"irreversible": False, "magnet_temp_C": 150, "demag_proportion": 0.0,
                    "min_magnet_B_T": 0.31},
          "efficiency": {"peak_pct": 96.1}}
    th = {"thermal": {"continuous_torque_Nm": 205.0, "magnet_hotspot_C": 142.0,
                      "winding_hotspot_C": 171.0, "winding_limit_C": 180.0}}
    st = {"rotor_stress": {"safety_factor": 1.8, "max_vonMises_MPa": 250.0,
                           "overspeed_rpm": 21600},
          "rotordynamics": {"first_bending_Hz": 360.0}}
    score_em(spec, em).print(j, gate="G1 EM")
    score_thermal(spec, th).print(j, gate="G2 thermal")
    score_structural(spec, st).print(j, gate="G3 structural")
    print("\nself-check OK")


if __name__ == "__main__":
    _selftest()
