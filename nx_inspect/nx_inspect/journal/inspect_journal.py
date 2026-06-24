"""nx_inspect — NX-side model-quality inspector journal (runs INSIDE Siemens NX).

Run headless via run_journal, or from the open part:

    "%UGII_ROOT_DIR%\\run_journal.exe" inspect_journal.py -args <part.prt> [out=report.json]
        [grid=N] [tol=MM3] [tiny=MM3] [dup=MM] [checks=interference,tiny,...]

It reads the ACTUAL NX geometry (no CPython approximation) and emits a single JSON
report (schema below) that the pure-Python `nx_inspect` CLI renders to console / HTML.
It is GENERIC: no assumptions about the part — works on any NX solid-body part.

Checks (v1, all on APIs verified on NX 2506 hardware):
  interference   — pairs of solid bodies whose solids interpenetrate (grid point-in-both
                   sampling of the bbox-overlap region, using NX's true point containment;
                   touching faces / press fits score ~0 and are NOT flagged). ERROR.
  zero_volume    — a "solid" body with ~0 or negative volume (degenerate). ERROR.
  tiny_body      — a sliver body below `tiny` mm^3 (often an unintended fragment). WARNING.
  duplicate_body — two bodies with the same volume + coincident centroid (a double-build). WARNING.
  unnamed_body   — a body with no display name (hurts FEA/CAM/per-part export). INFO.
  duplicate_name — the same name on multiple bodies (ambiguous selection). INFO.

Verified NX 2506 API: UF.ModlGeneral.AskBoundingBox, UF.Modeling.AskPointContainment
(1=inside / 2=outside), MeasureManager.NewMassProperties (.Volume/.Area/.Mass/.Centroid).

JSON schema (v1):
{
  "tool":"nx_inspect","schema":"1","part":str,"path":str,"units":"mm","is_assembly":bool,
  "config":{"grid":int,"tol_mm3":float,"tiny_mm3":float,"dup_mm":float,"checks":[str]},
  "summary":{"n_bodies":int,"errors":int,"warnings":int,"info":int,"checks_run":[str]},
  "components":[{"name":str,"part":str,"origin":[x,y,z]}],          # assemblies only
  "bodies":[{"id":int,"name":str,"volume_mm3":float|None,"area_mm2":float|None,
             "mass_kg":float|None,"centroid":[x,y,z]|None,"bbox":[x0,y0,z0,x1,y1,z1]}],
  "findings":[{"check":str,"severity":"error|warning|info","title":str,"detail":str,
               "bodies":[str],"location":[x,y,z]|None,"metric":{..},"suggestion":str}]
}
Read-only: the part is never modified.
"""
import json
import os
import sys
import traceback

import NXOpen

try:
    import NXOpen.UF
    _UF = NXOpen.UF.UFSession.GetUFSession()
except Exception:  # pragma: no cover
    _UF = None

SCHEMA = "1"
DEFAULTS = {"grid": 10, "tol": 50.0, "tiny": 30.0, "dup": 1.0}
ALL_CHECKS = ["interference", "zero_volume", "tiny_body", "duplicate_body",
              "unnamed_body", "duplicate_name"]


# --------------------------------------------------------------------------- #
# args / session
# --------------------------------------------------------------------------- #
def _lw():
    s = NXOpen.Session.GetSession()
    w = s.ListingWindow
    w.Open()
    return w


def _arg(key, default):
    for a in sys.argv[1:]:
        if a.lower().startswith(key + "="):
            val = a.split("=", 1)[1]
            if isinstance(default, (int, float)):
                try:
                    return type(default)(val)
                except ValueError:
                    return default
            return val
    return default


def _open_part(s, lw):
    for a in sys.argv[1:]:
        if a.lower().endswith(".prt") and os.path.exists(a):
            try:
                (getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open)(os.path.abspath(a))
                lw.WriteLine("opened %s" % a)
            except Exception as exc:
                lw.WriteLine("WARN could not open %s: %s" % (a, exc))
    return s.Parts.Work


# --------------------------------------------------------------------------- #
# geometry measurement (verified NX 2506 calls)
# --------------------------------------------------------------------------- #
def _bbox(tag):
    return [float(v) for v in _UF.ModlGeneral.AskBoundingBox(tag)]


def _mass_props(part, body):
    """(volume_mm3, area_mm2, mass_kg, centroid[xyz]) via MeasureManager; (None,)*4 on failure."""
    try:
        mp = part.MeasureManager.NewMassProperties(None, 0.99, [body])
        vol = float(mp.Volume)
        area = float(mp.Area)
        mass = float(mp.Mass)
        try:
            c = mp.Centroid
            cen = [float(c.X), float(c.Y), float(c.Z)]
        except Exception:
            cen = None
        try:
            mp.Dispose()
        except Exception:
            pass
        return vol, area, mass, cen
    except Exception:
        return None, None, None, None


def _inside(tag, p):
    try:
        return int(_UF.Modeling.AskPointContainment([float(p[0]), float(p[1]), float(p[2])], tag)) == 1
    except Exception:
        return False


def _boxes_overlap(a, b):
    return all(a[i] < b[i + 3] and b[i] < a[i + 3] for i in range(3))


def _overlap_volume(a_tag, b_tag, box_a, box_b, grid):
    """Estimated interpenetration volume (mm^3) by sampling the box-overlap region."""
    lo = [max(box_a[i], box_b[i]) for i in range(3)]
    hi = [min(box_a[i + 3], box_b[i + 3]) for i in range(3)]
    dims = [hi[i] - lo[i] for i in range(3)]
    if min(dims) <= 0:
        return 0.0, 0, None
    cell = [d / grid for d in dims]
    box_vol = dims[0] * dims[1] * dims[2]
    inside_both = 0
    total = 0
    sx = sy = sz = 0.0
    for i in range(grid):
        x = lo[0] + (i + 0.5) * cell[0]
        for j in range(grid):
            y = lo[1] + (j + 0.5) * cell[1]
            for k in range(grid):
                z = lo[2] + (k + 0.5) * cell[2]
                total += 1
                if _inside(a_tag, (x, y, z)) and _inside(b_tag, (x, y, z)):
                    inside_both += 1
                    sx += x
                    sy += y
                    sz += z
    if not inside_both:
        return 0.0, 0, None
    vol = box_vol * inside_both / float(total)
    centre = [round(sx / inside_both, 2), round(sy / inside_both, 2), round(sz / inside_both, 2)]
    return vol, inside_both, centre


# --------------------------------------------------------------------------- #
# findings
# --------------------------------------------------------------------------- #
def _finding(check, severity, title, detail, bodies=None, location=None, metric=None, suggestion=""):
    return {"check": check, "severity": severity, "title": title, "detail": detail,
            "bodies": bodies or [], "location": location, "metric": metric or {},
            "suggestion": suggestion}


def _collect_bodies(part):
    """List of dicts (tag kept separately) for every solid body in the work part."""
    out = []
    tags = []
    for b in part.Bodies:
        if not b.IsSolidBody:
            continue
        try:
            name = b.Name or ""
        except Exception:
            name = ""
        try:
            box = _bbox(b.Tag)
        except Exception:
            box = None
        vol, area, mass, cen = _mass_props(part, b)
        out.append({"id": len(out), "name": name or "(unnamed)",
                    "volume_mm3": None if vol is None else round(vol, 2),
                    "area_mm2": None if area is None else round(area, 2),
                    "mass_kg": None if mass is None else round(mass, 4),
                    "centroid": None if cen is None else [round(v, 2) for v in cen],
                    "bbox": None if box is None else [round(v, 2) for v in box]})
        tags.append(b.Tag)
    return out, tags


def _components(part):
    """Assembly component list (name, prototype part, origin); [] for a non-assembly."""
    comps = []
    try:
        root = part.ComponentAssembly.RootComponent
        if root is None:
            return comps
        stack = list(root.GetChildren())
        while stack:
            c = stack.pop()
            try:
                kids = list(c.GetChildren())
            except Exception:
                kids = []
            stack.extend(kids)
            if kids:
                continue  # only leaves carry geometry
            try:
                org = c.GetPosition()[0] if hasattr(c, "GetPosition") else None
                origin = [round(org.X, 2), round(org.Y, 2), round(org.Z, 2)] if org else None
            except Exception:
                origin = None
            try:
                proto = os.path.basename(c.Prototype.OwningPart.FullPath) if c.Prototype else ""
            except Exception:
                proto = ""
            comps.append({"name": c.DisplayName, "part": proto, "origin": origin})
    except Exception:
        pass
    return comps


# --------------------------------------------------------------------------- #
# the checks
# --------------------------------------------------------------------------- #
def run_checks(bodies, tags, cfg, lw):
    findings = []
    checks = cfg["checks"]
    name_of = [b["name"] for b in bodies]

    if "interference" in checks:
        boxes = [b["bbox"] for b in bodies]
        n_pairs = 0
        for i in range(len(bodies)):
            if boxes[i] is None:
                continue
            for j in range(i + 1, len(bodies)):
                if boxes[j] is None or not _boxes_overlap(boxes[i], boxes[j]):
                    continue
                n_pairs += 1
                vol, pts, centre = _overlap_volume(tags[i], tags[j], boxes[i], boxes[j], cfg["grid"])
                if vol > cfg["tol"] and pts > 0:
                    findings.append(_finding(
                        "interference", "error",
                        "Bodies interpenetrate (%s & %s)" % (name_of[i], name_of[j]),
                        "Estimated overlap volume ~%.0f mm^3 (%d interior sample points)." % (vol, pts),
                        bodies=[name_of[i], name_of[j]], location=centre,
                        metric={"overlap_volume_mm3": round(vol, 1), "sample_points": pts},
                        suggestion="Unite the two into one body, separate them so they only touch at a "
                                   "face, or seat one in a subtracted bore in the other."))
        lw.WriteLine("interference: checked %d box-overlapping pair(s)" % n_pairs)

    if "zero_volume" in checks:
        for b in bodies:
            v = b["volume_mm3"]
            if v is not None and v <= max(1e-6, cfg["tiny"] * 0.0):
                if v <= 0.0:
                    findings.append(_finding(
                        "zero_volume", "error", "Body has zero/negative volume (%s)" % b["name"],
                        "Measured volume = %s mm^3 - a degenerate or non-solid body." % v,
                        bodies=[b["name"]], location=b["centroid"], metric={"volume_mm3": v},
                        suggestion="Check the build step that created it; it may have failed to form a solid."))

    if "tiny_body" in checks:
        for b in bodies:
            v = b["volume_mm3"]
            if v is not None and 0.0 < v < cfg["tiny"]:
                findings.append(_finding(
                    "tiny_body", "warning", "Sliver/tiny body (%s)" % b["name"],
                    "Volume %.3f mm^3 is below the tiny threshold (%.0f mm^3) - likely an unintended fragment." % (v, cfg["tiny"]),
                    bodies=[b["name"]], location=b["centroid"], metric={"volume_mm3": v},
                    suggestion="Confirm it is intended; a stray sliver often means a boolean left a fragment."))

    if "duplicate_body" in checks:
        dup = cfg["dup"]
        for i in range(len(bodies)):
            bi = bodies[i]
            if bi["volume_mm3"] is None or bi["centroid"] is None:
                continue
            for j in range(i + 1, len(bodies)):
                bj = bodies[j]
                if bj["volume_mm3"] is None or bj["centroid"] is None:
                    continue
                if abs(bi["volume_mm3"] - bj["volume_mm3"]) <= max(1.0, 0.001 * bi["volume_mm3"]) and \
                   all(abs(bi["centroid"][k] - bj["centroid"][k]) <= dup for k in range(3)):
                    findings.append(_finding(
                        "duplicate_body", "warning",
                        "Coincident duplicate bodies (%s, %s)" % (bi["name"], bj["name"]),
                        "Same volume (~%.0f mm^3) and coincident centroid - likely a double-built body." % bi["volume_mm3"],
                        bodies=[bi["name"], bj["name"]], location=bi["centroid"],
                        metric={"volume_mm3": bi["volume_mm3"]},
                        suggestion="Remove the duplicate build step / boolean that created the second copy."))

    if "unnamed_body" in checks:
        n_unnamed = sum(1 for b in bodies if b["name"] == "(unnamed)")
        if n_unnamed:
            findings.append(_finding(
                "unnamed_body", "info", "%d unnamed bod%s" % (n_unnamed, "y" if n_unnamed == 1 else "ies"),
                "Unnamed bodies cannot be selected by role in FEA/CAM or grouped for per-part export.",
                bodies=[b["name"] for b in bodies if b["name"] == "(unnamed)"][:20],
                suggestion="Set body_name (Body.SetName) on every created body."))

    if "duplicate_name" in checks:
        seen = {}
        for b in bodies:
            if b["name"] == "(unnamed)":
                continue
            seen.setdefault(b["name"], 0)
            seen[b["name"]] += 1
        for nm, cnt in sorted(seen.items()):
            if cnt > 1:
                findings.append(_finding(
                    "duplicate_name", "info", "Name '%s' used by %d bodies" % (nm, cnt),
                    "Several bodies share this display name - ambiguous for selection (fine for intended sets).",
                    bodies=[nm], metric={"count": cnt},
                    suggestion="Use a unique suffix per instance (e.g. %s_000, %s_001) if they are distinct parts." % (nm, nm)))
    return findings


def main():
    s = NXOpen.Session.GetSession()
    lw = _lw()
    lw.WriteLine("=== nx_inspect journal (schema %s) ===" % SCHEMA)
    if _UF is None:
        lw.WriteLine("FATAL: UF session unavailable.")
        return
    cfg = {
        "grid": int(_arg("grid", DEFAULTS["grid"])),
        "tol": float(_arg("tol", DEFAULTS["tol"])),
        "tiny": float(_arg("tiny", DEFAULTS["tiny"])),
        "dup": float(_arg("dup", DEFAULTS["dup"])),
    }
    checks_arg = _arg("checks", "")
    cfg["checks"] = [c.strip() for c in checks_arg.split(",") if c.strip()] if checks_arg else list(ALL_CHECKS)

    part = _open_part(s, lw)
    if part is None:
        lw.WriteLine("FATAL: no work part.")
        return
    part_name = os.path.splitext(os.path.basename(part.FullPath))[0]
    comps = _components(part)
    bodies, tags = _collect_bodies(part)
    lw.WriteLine("part %s: %d solid bodies%s" % (
        part_name, len(bodies), (" (assembly: %d leaf components)" % len(comps)) if comps else ""))

    findings = run_checks(bodies, tags, cfg, lw)
    sev = {"error": 0, "warning": 0, "info": 0}
    for f in findings:
        sev[f["severity"]] = sev.get(f["severity"], 0) + 1

    report = {
        "tool": "nx_inspect", "schema": SCHEMA, "part": part_name,
        "path": part.FullPath, "units": "mm", "is_assembly": bool(comps),
        "config": {"grid": cfg["grid"], "tol_mm3": cfg["tol"], "tiny_mm3": cfg["tiny"],
                   "dup_mm": cfg["dup"], "checks": cfg["checks"]},
        "summary": {"n_bodies": len(bodies), "errors": sev["error"],
                    "warnings": sev["warning"], "info": sev["info"], "checks_run": cfg["checks"]},
        "components": comps,
        "bodies": bodies,
        "findings": findings,
    }

    out = _arg("out", "")
    paths = []
    if out:
        paths.append(out if os.path.isabs(out) else os.path.abspath(out))
    try:
        paths.append(os.path.join(os.path.dirname(part.FullPath), part_name + "_inspect.json"))
    except Exception:
        pass
    for p in paths:
        try:
            with open(p, "w") as fh:
                json.dump(report, fh, indent=2)
            lw.WriteLine("wrote %s" % p)
        except Exception as exc:
            lw.WriteLine("WARN could not write %s: %s" % (p, exc))

    lw.WriteLine("SUMMARY: %d bodies | %d error(s), %d warning(s), %d info" % (
        len(bodies), sev["error"], sev["warning"], sev["info"]))
    for f in findings[:40]:
        lw.WriteLine("  [%-7s] %s" % (f["severity"].upper(), f["title"]))
    lw.WriteLine("=== nx_inspect done ===")


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        try:
            NXOpen.Session.GetSession().ListingWindow.WriteLine(
                "nx_inspect ABORTED:\n" + traceback.format_exc())
        except Exception:
            pass
