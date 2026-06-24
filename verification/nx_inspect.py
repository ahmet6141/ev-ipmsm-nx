"""NX SCENE INSPECTOR -- the "NX MCP" read-out: open a built .prt, measure every solid
body in the ACTUAL NX geometry, and find which bodies INTERPENETRATE (not just touch).
Writes a machine-readable JSON report + prints a summary, so the build->inspect->fix
loop runs on NX ground truth instead of CPython bounding-box approximations.

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_inspect.py -args <part.prt> [grid=N] [tol=MM3]

What it reports (per body): name + axis-aligned world bounding box + (best-effort) volume.
Interference: for every body pair whose boxes overlap, it samples a regular grid inside the
box-overlap region and asks NX's TRUE point-in-solid test (UF_MODL_ask_point_containment)
how many sample points fall INSIDE BOTH bodies. The inside-both fraction x the overlap-box
volume estimates the INTERPENETRATION VOLUME. A pair with an estimated overlap volume above
`tol` mm3 (default 50) is flagged INTERPENETRATION; bodies that merely share a face (a bolt
seated in its hole, two parts butting) sample ~0 interior points and are NOT flagged.

Read-only: nothing in the part is modified. The JSON lands next to the part as
<part>_inspect.json (and a copy at verification/nx_inspect_report.json).
"""
import json
import os
import sys
import traceback

import NXOpen

try:
    import NXOpen.UF
    _UF = NXOpen.UF.UFSession.GetUFSession()
except Exception:  # pragma: no cover - UF is effectively always present in NX
    _UF = None


def _lw():
    s = NXOpen.Session.GetSession()
    w = s.ListingWindow
    w.Open()
    return w


def _open_part(s, lw):
    """Use the part passed as -args, else the already-open work part."""
    for a in sys.argv[1:]:
        if a.lower().endswith(".prt") and os.path.exists(a):
            try:
                opener = getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open
                opener(os.path.abspath(a))
                lw.WriteLine("opened %s" % a)
            except Exception as exc:
                lw.WriteLine("WARN could not open %s: %s" % (a, exc))
    return s.Parts.Work


def _arg_val(key, default):
    for a in sys.argv[1:]:
        if a.lower().startswith(key + "="):
            try:
                return type(default)(a.split("=", 1)[1])
            except Exception:
                pass
    return default


def _bbox(body):
    """Axis-aligned world bounding box [xmin,ymin,zmin,xmax,ymax,zmax] of a body.
    NX 2506: UF_MODL_ask_bounding_box is exposed as UF.ModlGeneral.AskBoundingBox and
    returns 6 doubles (min corner, max corner) in absolute coordinates."""
    bb = _UF.ModlGeneral.AskBoundingBox(body.Tag)
    return [float(v) for v in bb]


def _volume(body):
    """Best-effort solid volume (mm^3); None if the mass-props call is unavailable."""
    try:
        acc = [0.999] * 11
        mp = _UF.Modeling.AskMassProps3d([body.Tag], 1, 1, 1, 0.0, 1, acc)
        props = mp[0] if isinstance(mp, (tuple, list)) else mp
        return float(props[3])  # [3] = volume in the UF mass-props array
    except Exception:
        return None


def _boxes_overlap(a, b, pad=0.0):
    return all(a[i] - pad < b[i + 3] and b[i] - pad < a[i + 3] for i in range(3))


def _overlap_box(a, b):
    lo = [max(a[i], b[i]) for i in range(3)]
    hi = [min(a[i + 3], b[i + 3]) for i in range(3)]
    return lo, hi


def _inside(body_tag, p):
    """True if point p is strictly INSIDE the solid. NX 2506 UF.Modeling.
    AskPointContainment returns 1 = inside, 2 = outside (calibrated on real hardware);
    we count only the strict-inside code so two touching faces (a bolt in its hole, two
    parts butting) score ~0 interior points and are NOT mistaken for interpenetration."""
    try:
        st = _UF.Modeling.AskPointContainment([float(p[0]), float(p[1]), float(p[2])], body_tag)
        return int(st) == 1
    except Exception:
        return False


def _interpenetration(a, b, box_a, box_b, grid):
    """Estimate the overlap volume (mm^3) of two bodies by sampling the box-overlap
    region and counting points inside BOTH. Returns (overlap_vol, inside_both, total)."""
    lo, hi = _overlap_box(box_a, box_b)
    dims = [hi[i] - lo[i] for i in range(3)]
    if min(dims) <= 0:
        return 0.0, 0, 0
    cell = [d / grid for d in dims]
    box_vol = dims[0] * dims[1] * dims[2]
    inside_both = 0
    total = 0
    ta, tb = a.Tag, b.Tag
    for i in range(grid):
        x = lo[0] + (i + 0.5) * cell[0]
        for j in range(grid):
            y = lo[1] + (j + 0.5) * cell[1]
            for k in range(grid):
                z = lo[2] + (k + 0.5) * cell[2]
                total += 1
                if _inside(ta, (x, y, z)) and _inside(tb, (x, y, z)):
                    inside_both += 1
    vol = box_vol * inside_both / float(total) if total else 0.0
    return vol, inside_both, total


def main():
    s = NXOpen.Session.GetSession()
    lw = _lw()
    lw.WriteLine("=== nx_inspect: scene interference read-out ===")
    if _UF is None:
        lw.WriteLine("FATAL: UF session unavailable -- cannot measure geometry.")
        return
    grid = int(_arg_val("grid", 12))
    tol = float(_arg_val("tol", 50.0))  # mm^3 overlap above which a pair is flagged

    part = _open_part(s, lw)
    if part is None:
        lw.WriteLine("FATAL: no work part.")
        return
    part_name = os.path.splitext(os.path.basename(part.FullPath))[0]

    bodies = [b for b in part.Bodies if b.IsSolidBody]
    lw.WriteLine("part %s: %d solid bodies (grid=%d, tol=%.0f mm^3)" % (
        part_name, len(bodies), grid, tol))

    report = {"part": part_name, "n_bodies": len(bodies), "bodies": [], "interferences": []}
    boxes = {}
    for b in bodies:
        try:
            name = b.Name or "(unnamed)"
        except Exception:
            name = "(unnamed)"
        try:
            box = _bbox(b)
        except Exception as exc:
            lw.WriteLine("WARN bbox %s: %s" % (name, exc))
            continue
        boxes[b.Tag] = box
        report["bodies"].append({"name": name, "bbox": [round(v, 2) for v in box],
                                 "volume_mm3": _volume(b)})

    # pairwise interference (only for bodies whose boxes already overlap)
    n_pairs = 0
    n_flagged = 0
    for ia in range(len(bodies)):
        a = bodies[ia]
        if a.Tag not in boxes:
            continue
        for ib in range(ia + 1, len(bodies)):
            b = bodies[ib]
            if b.Tag not in boxes:
                continue
            if not _boxes_overlap(boxes[a.Tag], boxes[b.Tag]):
                continue
            n_pairs += 1
            vol, inside_both, total = _interpenetration(a, b, boxes[a.Tag], boxes[b.Tag], grid)
            if vol > tol and inside_both > 0:
                n_flagged += 1
                na = a.Name or "(unnamed)"
                nb = b.Name or "(unnamed)"
                report["interferences"].append({
                    "a": na, "b": nb,
                    "overlap_volume_mm3": round(vol, 1),
                    "inside_both_samples": inside_both, "grid_samples": total})
                lw.WriteLine("INTERFERENCE  %-26s <-> %-26s  ~%.0f mm^3 (%d/%d pts)" % (
                    na, nb, vol, inside_both, total))

    report["n_box_overlap_pairs"] = n_pairs
    report["n_interference_pairs"] = n_flagged
    lw.WriteLine("checked %d box-overlapping pair(s); %d INTERFERENCE(S) above %.0f mm^3"
                 % (n_pairs, n_flagged, tol))

    # write JSON next to the part + a stable copy under verification/
    out_paths = []
    try:
        out_paths.append(os.path.join(os.path.dirname(part.FullPath), part_name + "_inspect.json"))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    out_paths.append(os.path.join(here, "nx_inspect_report.json"))
    for op in out_paths:
        try:
            with open(op, "w") as fh:
                json.dump(report, fh, indent=2)
            lw.WriteLine("wrote %s" % op)
        except Exception as exc:
            lw.WriteLine("WARN could not write %s: %s" % (op, exc))
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
