"""Render a built .prt from several standard views to PNG files, for VISUAL design review.

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_render.py -args <part.prt> [out=DIR] [size=N]

IMPORTANT: NX must have a GRAPHICS DISPLAY. run_journal runs NX in BATCH mode
(session.IsBatch == True), which has NO display, so UF.Disp.CreateImage fails there.
Run this inside an NX GUI session (ugraf), or via a graphics-enabled launch. The journal
writes one shaded PNG per canned view into <part>_views/ (or out=DIR).

Verified NX 2506 API: ModelingViews.WorkView.Orient(View.Canned.*, ScaleAdjustment.Fit);
UF.Disp.CreateImage(path, UF.DispImageFormat.PNG, UF.DispBackgroundColor.WHITE).
"""
import os, sys, traceback, NXOpen, NXOpen.UF

_VIEWS = ["Isometric", "Trimetric", "Top", "Front", "Right", "Back", "Left", "Bottom"]


def _arg(key, default):
    for a in sys.argv[1:]:
        if a.lower().startswith(key + "="):
            v = a.split("=", 1)[1]
            return type(default)(v) if isinstance(default, int) else v
    return default


def main():
    s = NXOpen.Session.GetSession()
    lw = s.ListingWindow; lw.Open()
    uf = NXOpen.UF.UFSession.GetUFSession()
    lw.WriteLine("=== nx_render === (batch=%s)" % s.IsBatch)
    for a in sys.argv[1:]:
        if a.lower().endswith(".prt") and os.path.exists(a):
            (getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open)(os.path.abspath(a))
    part = s.Parts.Work
    if part is None:
        lw.WriteLine("no work part"); return
    name = os.path.splitext(os.path.basename(part.FullPath))[0]
    outdir = _arg("out", "") or os.path.join(os.path.dirname(part.FullPath), name + "_views")
    if not os.path.isdir(outdir):
        os.makedirs(outdir, exist_ok=True)

    if s.IsBatch:
        lw.WriteLine("FATAL: NX is in BATCH mode (no graphics) -- CreateImage cannot render. "
                     "Run this in an NX GUI session.")
        return

    mv = part.ModelingViews.WorkView
    # shaded with edges so flaws (bad proportions, floating/misplaced features) are visible
    try:
        mv.RenderingStyle = NXOpen.View.RenderingStyleType.PartiallyShaded
    except Exception:
        pass
    fmt = uf.Disp.ImageFormat.PNG
    bg = NXOpen.UF.DispBackgroundColor.WHITE
    made = []
    for vn in _VIEWS:
        canned = getattr(NXOpen.View.Canned, vn, None)
        if canned is None:
            continue
        try:
            mv.Orient(canned, NXOpen.View.ScaleAdjustment.Fit)
        except Exception as exc:
            lw.WriteLine("orient %s ERR %s" % (vn, exc)); continue
        path = os.path.join(outdir, "%s_%s.png" % (name, vn.lower()))
        try:
            if os.path.exists(path):
                os.remove(path)
            uf.Disp.CreateImage(path, fmt, bg)
            ok = os.path.exists(path)
            lw.WriteLine("view %-10s -> %s (%s)" % (vn, path, "OK" if ok else "no file"))
            if ok:
                made.append(path)
        except Exception as exc:
            lw.WriteLine("render %s ERR %s" % (vn, exc))
    lw.WriteLine("=== nx_render done: %d image(s) in %s ===" % (len(made), outdir))


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        NXOpen.Session.GetSession().ListingWindow.WriteLine("nx_render ABORTED:\n" + traceback.format_exc())
