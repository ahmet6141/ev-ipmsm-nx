"""Render the EV parts from several standard views to PNG files, for VISUAL design review
-- each subsystem part SEPARATELY *and* the combined vehicle assembly.

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_render.py -args [part.prt ...] [size=N]

RUN MODEL: NX must have a GRAPHICS DISPLAY. run_journal runs NX in BATCH mode
(session.IsBatch == True) which has NO display, so UF.Disp.CreateImage fails there.
Run this INSIDE AN NX GUI session: Tools -> Journal -> Play -> nx_render.py. With no
.prt argument it renders the DEFAULT SET below (every built subsystem part it finds in
vehicle_nx/, PLUS the combined vehicle_out.prt assembly), opening/closing each in turn.
Pass explicit .prt paths to render just those.

Output: <repo>/renders/<partname>/<partname>_<view>.png  -- one folder per part, the
8 canned views each. Point your AI/CAD reviewer at renders/ afterwards.

Confirmed NX 2506 API: ModelingViews.WorkView.Orient(View.Canned.*, ScaleAdjustment.Fit);
UF.Disp.CreateImage(path, UF.DispImageFormat.PNG, UF.DispBackgroundColor.WHITE).
"""
import os, sys, traceback, NXOpen, NXOpen.UF

_VIEWS = ["Isometric", "Trimetric", "Top", "Front", "Right", "Back", "Left", "Bottom"]
# individual subsystem parts first, the COMBINED assembly last
_DEFAULT = ["motor_out.prt", "driveline_out.prt", "gearbox_out.prt", "inverter_out.prt",
            "suspension_out.prt", "chassis_out.prt", "subframe_out_front.prt",
            "subframe_out_rear.prt", "vehicle_out.prt"]


def _repo():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _targets():
    args = [a for a in sys.argv[1:] if a.lower().endswith(".prt")]
    if args:
        return [os.path.abspath(a) for a in args if os.path.exists(a)]
    vdir = os.path.join(_repo(), "vehicle_nx")
    return [os.path.join(vdir, p) for p in _DEFAULT if os.path.exists(os.path.join(vdir, p))]


def _find_loaded(s, prt):
    """The already-loaded Part matching this file path, or None. Subsystem parts are
    loaded as COMPONENTS when the vehicle assembly is open, so OpenDisplay would fail
    'File already exists' -- reuse the loaded part and just SetDisplay it instead."""
    target = os.path.normcase(os.path.abspath(prt))
    for p in s.Parts:
        try:
            if os.path.normcase(p.FullPath) == target:
                return p
        except Exception:
            pass
    return None


def _render_part(s, uf, lw, prt, render_root):
    name = os.path.splitext(os.path.basename(prt))[0]
    opened_here = False
    loaded = _find_loaded(s, prt)
    if loaded is not None:
        try:
            s.Parts.SetDisplay(loaded, False, False)   # display the already-loaded part
        except Exception as exc:
            lw.WriteLine("SetDisplay %s note: %s" % (name, str(exc)[:70]))
    else:
        try:
            r = (getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open)(os.path.abspath(prt))
            r[0] if isinstance(r, tuple) else r
            opened_here = True
        except Exception as exc:
            lw.WriteLine("OPEN FAIL %s: %s" % (name, str(exc)[:70])); return 0
    work = s.Parts.Display or s.Parts.Work     # render the DISPLAYED part
    outdir = os.path.join(render_root, name)
    if not os.path.isdir(outdir):
        os.makedirs(outdir, exist_ok=True)
    mv = work.ModelingViews.WorkView
    # SHADED WITH EDGES (was wireframe before -- PartiallyShaded is not a valid member;
    # ShadedWithEdges is, and is best for spotting bad proportions / surface flaws).
    for style in ("ShadedWithEdges", "Shaded"):
        st = getattr(NXOpen.View.RenderingStyleType, style, None)
        if st is not None:
            try:
                mv.RenderingStyle = st
                break
            except Exception:
                pass
    fmt = uf.Disp.ImageFormat.PNG
    bg = NXOpen.UF.DispBackgroundColor.WHITE
    made = 0
    for vn in _VIEWS:
        canned = getattr(NXOpen.View.Canned, vn, None)
        if canned is None:
            continue
        try:
            mv.Orient(canned, NXOpen.View.ScaleAdjustment.Fit)
        except Exception:
            continue
        path = os.path.join(outdir, "%s_%s.png" % (name, vn.lower()))
        try:
            if os.path.exists(path):
                os.remove(path)
            uf.Disp.CreateImage(path, fmt, bg)
            if os.path.exists(path):
                made += 1
        except Exception as exc:
            lw.WriteLine("  render %s/%s ERR %s" % (name, vn, str(exc)[:70]))
    lw.WriteLine("%-22s -> %d view(s) in %s" % (name, made, outdir))
    # only close parts WE opened here -- never close an already-loaded part (it may be a
    # live component of an open assembly; closing it would break the assembly).
    if opened_here:
        try:
            work.Close(NXOpen.BasePart.CloseWholeTree.TrueValue,
                       NXOpen.BasePart.CloseModified.CloseModified, None)
        except Exception:
            pass
    return made


def main():
    s = NXOpen.Session.GetSession()
    lw = s.ListingWindow; lw.Open()
    uf = NXOpen.UF.UFSession.GetUFSession()
    lw.WriteLine("=== nx_render (batch=%s) ===" % s.IsBatch)
    if s.IsBatch:
        lw.WriteLine("FATAL: NX is in BATCH mode (no graphics). Run this in an NX GUI session "
                     "(Tools -> Journal -> Play). run_journal cannot render.")
        return
    render_root = os.path.join(_repo(), "renders")
    if not os.path.isdir(render_root):
        os.makedirs(render_root, exist_ok=True)
    targets = _targets()
    if not targets:
        lw.WriteLine("no parts found to render (build them first via the subsystem/vehicle builders)")
        return
    lw.WriteLine("rendering %d part(s) (each separately; vehicle_out = combined):" % len(targets))
    total = 0
    for prt in targets:
        total += _render_part(s, uf, lw, prt, render_root)
    lw.WriteLine("=== nx_render done: %d image(s) under %s ===" % (total, render_root))


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        NXOpen.Session.GetSession().ListingWindow.WriteLine("nx_render ABORTED:\n" + traceback.format_exc())
