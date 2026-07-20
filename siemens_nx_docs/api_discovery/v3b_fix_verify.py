"""v3b_fix_verify.py — iteration 2: fix the 6 failures from v3_master_verify.

    "%UGII_ROOT_DIR%\\run_journal.exe" v3b_fix_verify.py

Fixes attempted (with deep introspection logged even on failure):
  T04 draft   -> set AngleTolerance/DistanceTolerance BEFORE commit
  T05 thread  -> log ThreadStandard/Size; try Detailed type; try explicit standards
  T06 shell   -> set Tolerance = 0.01
  T10 pmi     -> dir(b.Text); try every *ext* method; fall back to Annotations.CreateNote
  T11 png     -> variants (no size, UI builder); if all fail => documented GUI-only
  T12 hole    -> SetAllowedEntityTypes(OnlyPoints) before AddToSection
"""

import json
import math
import os
import traceback

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities
import NXOpen.Annotations

SES = NXOpen.Session.GetSession()
HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = {"tests": {}}
LW = SES.ListingWindow
LW.Open()


def log(m):
    try:
        LW.WriteLine(str(m))
    except Exception:
        pass


def p3(x, y, z):
    return NXOpen.Point3d(float(x), float(y), float(z))


def v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))


def run_test(name, fn):
    info = {}
    rec = {"ok": False, "info": info}
    try:
        fn(info)
        rec["ok"] = True
        log("PASS %-24s %s" % (name, json.dumps(info, default=str)[:150]))
    except Exception as exc:
        rec["error"] = str(exc)[:300]
        rec["trace"] = traceback.format_exc()[-500:]
        log("FAIL %-24s %s | info=%s" % (name, str(exc)[:100],
                                         json.dumps(info, default=str)[:200]))
    REPORT["tests"][name] = rec


def section_of(part, curves):
    sec = part.Sections.CreateSection(0.0095, 0.001, 0.5)
    sec.AllowSelfIntersection(False)
    rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)
    sec.AddToSection([rule], curves[0], NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                     p3(0, 0, 0), NXOpen.Section.Mode.Create, False)
    return sec


def extrude(part, curves, length, op="create", target=None):
    B = NXOpen.GeometricUtilities.BooleanOperation.BooleanType
    ops = {"create": B.Create, "unite": B.Unite, "subtract": B.Subtract}
    ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    ext.Section = section_of(part, curves)
    ext.Direction = part.Directions.CreateDirection(
        p3(0, 0, 0), v3(0, 0, 1), NXOpen.SmartObject.UpdateOption.WithinModeling)
    ext.Limits.StartExtend.Value.RightHandSide = "0"
    ext.Limits.EndExtend.Value.RightHandSide = repr(float(length))
    ext.BooleanOperation.Type = ops[op]
    if target is not None and op in ("unite", "subtract"):
        ext.BooleanOperation.SetTargetBodies([target])
    feat = ext.CommitFeature()
    ext.Destroy()
    return feat


def rect_curves(part, cx, cy, w, h, z):
    pts = [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2),
           (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]
    cur = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        cur.append(part.Curves.CreateLine(p3(a[0], a[1], z), p3(b[0], b[1], z)))
    return cur


def circle_curve(part, cx, cy, r, z):
    return part.Curves.CreateArc(p3(cx, cy, z), v3(1, 0, 0), v3(0, 1, 0),
                                 float(r), 0.0, 2.0 * math.pi)


def do_update(tag):
    SES.UpdateManager.DoUpdate(SES.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, tag))


def faces_of(body):
    return list(body.GetFaces())


def horiz_face(body, z_target):
    for f in body.GetFaces():
        zs = [vt.Z for e in f.GetEdges() for vt in e.GetVertices()]
        if zs and max(zs) - min(zs) < 0.001 and abs(zs[0] - z_target) < 0.001:
            return f
    return None


def main():
    out_prt = os.path.join(HERE, "v3b_verify_part.prt")
    if os.path.exists(out_prt):
        try:
            os.remove(out_prt)
        except OSError:
            pass
    part = SES.Parts.NewBaseDisplay(out_prt, NXOpen.BasePart.Units.Millimeters)
    if isinstance(part, tuple):
        part = part[0]

    # base geometry: block A (draft), block B (shell), block C+boss (thread/hole)
    featA = extrude(part, rect_curves(part, 0, 0, 40, 40, 0), 50)
    bodyA = featA.GetBodies()[0]
    bodyA.SetName("DRAFT_BLK")
    featB = extrude(part, rect_curves(part, 100, 0, 40, 40, 0), 40)
    bodyB = featB.GetBodies()[0]
    bodyB.SetName("SHELL_BLK")
    featC = extrude(part, rect_curves(part, 200, 0, 80, 60, 0), 30)
    bodyC = featC.GetBodies()[0]
    bodyC.SetName("HOLE_BLK")
    extrude(part, [circle_curve(part, 200, 0, 10, 30)], 25, "unite", bodyC)
    do_update("base")

    # T04 DRAFT — with tolerances set
    def t04(info):
        bottom = horiz_face(bodyA, 0.0)
        sides = []
        for f in bodyA.GetFaces():
            zs = [vt.Z for e in f.GetEdges() for vt in e.GetVertices()]
            if zs and max(zs) - min(zs) > 0.001:
                sides.append(f)
        info["n_sides"] = len(sides)
        db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
        db.AngleTolerance = 0.5
        db.DistanceTolerance = 0.001
        db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
        db.Direction = part.Directions.CreateDirection(
            p3(0, 0, 0), v3(0, 0, 1), NXOpen.SmartObject.UpdateOption.WithinModeling)
        db.StationaryEntity = bottom
        fsl = db.FaceSetAngleExpressionList
        info["fsl_dir"] = [m for m in dir(fsl) if not m.startswith("_")]
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleFaceDumb(sides)
        col.ReplaceRules([rule], False)
        for meth, args in (("Append", (col, "3")), ("Append", (col,)),
                           ("CreateAndAppend", (col, "3")),
                           ("AddCollectorSet", (col, "3"))):
            try:
                getattr(fsl, meth)(*args)
                info["fsl_add"] = "%s(%d args) OK" % (meth, len(args))
                break
            except Exception as exc:
                info.setdefault("fsl_tries", []).append(
                    "%s/%d: %s" % (meth, len(args), str(exc)[:60]))
        feat = db.CommitFeature()
        db.Destroy()
        do_update("t04")
        info["feature"] = feat.FeatureType
    run_test("T04_draft", t04)

    # T05 THREAD — introspect defaults, try Detailed, then standards
    def t05(info):
        cyl = None
        for f in bodyC.GetFaces():
            try:
                if f.SolidFaceType == NXOpen.Face.FaceType.Cylindrical:
                    cyl = f
                    break
            except Exception:
                pass
        assert cyl is not None
        tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)
        try:
            info["defaults"] = {
                "standard": tb.ThreadStandard, "size": tb.ThreadSize,
                "method": tb.ThreadMethod,
                "input": str(tb.ThreadInput), "type": str(tb.ThreadType)}
        except Exception as exc:
            info["defaults_err"] = str(exc)[:80]
        tb.CylindricalFace.Value = cyl
        try:
            info["after_select"] = {
                "standard": tb.ThreadStandard, "size": tb.ThreadSize,
                "cyl_dia": tb.CylinderDiameter}
        except Exception as exc:
            info["after_select_err"] = str(exc)[:80]
        committed = False
        # A: symbolic with whatever default populated
        try:
            tb.ThreadType = NXOpen.Features.ThreadBuilder.Type.Symbolic
            feat = tb.CommitFeature()
            info["path"] = "symbolic-default"
            committed = True
        except Exception as exc:
            info["symbolic_err"] = str(exc)[:100]
        if not committed:
            # B: detailed
            try:
                tb.ThreadType = NXOpen.Features.ThreadBuilder.Type.Detailed
                feat = tb.CommitFeature()
                info["path"] = "detailed"
                committed = True
            except Exception as exc:
                info["detailed_err"] = str(exc)[:100]
        if not committed:
            # C: symbolic with explicit standards
            for std in ("Metric Coarse", "Metric Fine", "Metric", "Unified UNC"):
                try:
                    tb.ThreadType = NXOpen.Features.ThreadBuilder.Type.Symbolic
                    tb.ThreadStandard = std
                    feat = tb.CommitFeature()
                    info["path"] = "symbolic std=%s" % std
                    committed = True
                    break
                except Exception as exc:
                    info.setdefault("std_tries", []).append("%s: %s" % (std, str(exc)[:60]))
        tb.Destroy()
        assert committed, "no thread path worked"
        do_update("t05")
        info["feature"] = feat.FeatureType
    run_test("T05_thread", t05)

    # T06 SHELL — Tolerance = 0.01
    def t06(info):
        top = horiz_face(bodyB, 40.0)
        assert top is not None
        sb = part.Features.CreateShellBuilder(NXOpen.Features.Feature.Null)
        sb.Tolerance = 0.01
        sb.Body = bodyB
        sb.SetDefaultThickness("2")
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleFaceDumb([top])
        col.ReplaceRules([rule], False)
        sb.RemovedFacesCollector = col
        feat = sb.CommitFeature()
        sb.Destroy()
        do_update("t06")
        info["feature"] = feat.FeatureType
        info["faces_after"] = len(bodyB.GetFaces())
    run_test("T06_shell", t06)

    # T10 PMI NOTE — introspect Text object; fallback CreateNote
    def t10(info):
        b = part.Annotations.CreatePmiNoteBuilder(NXOpen.Annotations.SimpleDraftingAid.Null)
        txt = b.Text
        info["text_type"] = type(txt).__name__
        info["text_dir"] = [m for m in dir(txt) if not m.startswith("_")]
        done = False
        for meth, args in (("SetText", (["SENTINEL NX2506"],)),
                           ("SetContent", (["SENTINEL NX2506"],)),
                           ("SetTextLines", (["SENTINEL NX2506"],))):
            if hasattr(txt, meth):
                try:
                    getattr(txt, meth)(*args)
                    info["text_set"] = meth
                    done = True
                    break
                except Exception as exc:
                    info.setdefault("text_tries", []).append("%s: %s" % (meth, str(exc)[:60]))
        # nested TextBlock?
        if not done:
            for sub in ("TextBlock", "Lettering", "Content"):
                if hasattr(txt, sub):
                    o = getattr(txt, sub)
                    info["sub_%s_dir" % sub] = [m for m in dir(o) if not m.startswith("_")][:30]
                    if hasattr(o, "SetText"):
                        try:
                            o.SetText(["SENTINEL NX2506"])
                            info["text_set"] = sub + ".SetText"
                            done = True
                            break
                        except Exception as exc:
                            info.setdefault("text_tries", []).append(
                                "%s.SetText: %s" % (sub, str(exc)[:60]))
        if done:
            og = b.Origin
            try:
                og.Origin.SetValue(NXOpen.TaggedObject.Null, part.Views.WorkView, p3(0, 0, 80))
                info["origin"] = "SetValue OK"
            except Exception as exc:
                info["origin_err"] = str(exc)[:80]
            note = b.Commit()
            info["note_type"] = type(note).__name__
            b.Destroy()
        else:
            b.Destroy()
            # fallback: direct CreateNote with lettering prefs
            info["fallback"] = "CreateNote"
            lp = part.Annotations.Preferences.GetLetteringPreferences()
            us = part.Annotations.Preferences.GetUserSymbolPreferences()
            note = part.Annotations.CreateNote(
                ["SENTINEL NX2506"], p3(0, 0, 80),
                NXOpen.AxisOrientation.Horizontal, lp, us)
            info["note_type"] = type(note).__name__
        do_update("t10")
    run_test("T10_pmi_note", t10)

    # T11 HEADLESS PNG — 3 variants
    def t11(info):
        png = os.path.join(HERE, "v3b_iso.png")
        if os.path.exists(png):
            os.remove(png)
        # variant 1: Views builder, minimal settings
        try:
            ib = part.Views.CreateImageExportBuilder()
            ib.FileName = png
            ib.FileFormat = NXOpen.Gateway.ImageExportBuilder.FileFormats.Png
            ib.Commit()
            ib.Destroy()
            if os.path.exists(png):
                info["variant"] = "Views-minimal"
                info["size"] = os.path.getsize(png)
                return
        except Exception as exc:
            info["v1_err"] = str(exc)[:100]
        # variant 2: UF display image
        try:
            import NXOpen.UF
            uf = NXOpen.UF.UFSession.GetUFSession()
            uf.Disp.CreateImage(png, NXOpen.UF.Disp.ImageFormat.Png,
                                NXOpen.UF.Disp.BackgroundColor.White)
            if os.path.exists(png):
                info["variant"] = "UF.Disp.CreateImage"
                info["size"] = os.path.getsize(png)
                return
        except Exception as exc:
            info["v2_err"] = str(exc)[:100]
        raise RuntimeError("headless PNG not possible (documented: GUI-only)")
    run_test("T11_headless_png", t11)

    # T12 HOLE PACKAGE — SetAllowedEntityTypes(OnlyPoints) first
    def t12(info):
        before_faces = len(bodyC.GetFaces())
        hp = part.Features.CreateHolePackageBuilder(NXOpen.Features.HolePackage.Null)
        hp.HoleType = NXOpen.Features.HolePackageBuilder.Holetype.Simple
        hp.GeneralSimpleHoleDiameter.SetFormula("8")
        hp.HoleDepthLimitOption = (
            NXOpen.Features.HolePackageBuilder.HoleDepthLimitOptions.ThroughBody)
        sec = hp.HolePosition
        try:
            sec.SetAllowedEntityTypes(NXOpen.Section.AllowTypes.OnlyPoints)
            info["allowed"] = "OnlyPoints OK"
        except Exception as exc:
            info["allowed_err"] = str(exc)[:80]
        try:
            sec.DistanceTolerance = 0.001
            sec.ChainingTolerance = 0.0095
            info["tol"] = "set"
        except Exception as exc:
            info["tol_err"] = str(exc)[:80]
        pt = part.Points.CreatePoint(p3(175, 0, 30))
        rule = part.ScRuleFactory.CreateRuleCurveDumbFromPoints([pt])
        sec.AddToSection([rule], pt, NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                         p3(175, 0, 30), NXOpen.Section.Mode.Create, False)
        info["position"] = "added"
        feat = hp.CommitFeature()
        hp.Destroy()
        do_update("t12")
        info["feature"] = feat.FeatureType
        info["faces_before"] = before_faces
        info["faces_after"] = len(bodyC.GetFaces())
    run_test("T12_hole_package", t12)

    try:
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
                  NXOpen.BasePart.CloseAfterSave.FalseValue)
    except Exception as exc:
        REPORT["save_error"] = str(exc)[:150]

    with open(os.path.join(HERE, "v3b_verify_report.json"), "w", encoding="utf-8") as fh:
        json.dump(REPORT, fh, indent=2, default=str)
    npass = sum(1 for t in REPORT["tests"].values() if t["ok"])
    log("=== v3b: %d/%d PASS ===" % (npass, len(REPORT["tests"])))


main()
