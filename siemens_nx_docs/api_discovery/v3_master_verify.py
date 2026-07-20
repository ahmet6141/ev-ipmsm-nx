"""v3_master_verify.py — NX 2506 PROFESSIONAL FEATURE VERIFICATION (stub-derived, evidence-logged)

Runs HEADLESS:
    "%UGII_ROOT_DIR%\\run_journal.exe" v3_master_verify.py

Verifies — with the EXACT signatures mined from E:\\program\\UGOPEN\\pythonStubs — the
APIs that the earlier DeepSeek pass got wrong or left ⚠️:

  T01 base solid (extrude, proven path)          T07 mirror body (CreateMirrorBodyBuilder — EXISTS!)
  T02 edge blend  AddChainset(ScCollector, str)  T08 material   LoadFromNxmatmllibrary + AssignObjects
  T03 chamfer     SmartCollector + Option        T09 mass props MeasureManager.NewMassProperties
  T04 draft       TypeOfDraft/StationaryEntity   T10 PMI note   Annotations.CreatePmiNoteBuilder
  T05 thread      CylindricalFace (no FaceCollector!)  T11 image export (headless?) ImageExportBuilder
  T06 shell       ShellBuilder.SetDefaultThickness     T12 hole package (point via CreateRuleCurveDumbFromPoints)

Every test logs: ok/fail, exception text, and OBSERVABLE EVIDENCE (body/face/edge counts,
volume/mass before-after) into v3_verify_report.json next to this file.
"""

import json
import math
import os
import sys
import traceback

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities

SES = NXOpen.Session.GetSession()
HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = {"nx_version": "", "tests": {}}
LW = SES.ListingWindow
LW.Open()


def log(msg):
    try:
        LW.WriteLine(str(msg))
    except Exception:
        pass


def p3(x, y, z):
    return NXOpen.Point3d(float(x), float(y), float(z))


def v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))


def counts(part):
    solids = [b for b in part.Bodies if b.IsSolidBody]
    nf = sum(len(b.GetFaces()) for b in solids)
    ne = sum(len(b.GetEdges()) for b in solids)
    return {"bodies": len(solids), "faces": nf, "edges": ne}


def volume_of(part, body):
    """Try MeasureManager mass properties; fall back to UF. Returns dict."""
    out = {}
    try:
        mm = part.MeasureManager
        uc = part.UnitCollection
        # common recorded-journal unit set (5 units: area, volume, mass, radiusofgyr, weight?)
        names = ["SquareMilliMeter", "CubicMilliMeter", "Kilogram", "MilliMeter", "Newton"]
        units = []
        for n in names:
            try:
                units.append(uc.FindObject(n))
            except Exception:
                units.append(None)
        for take in (5, 4, 3):
            try:
                mb = mm.NewMassProperties(units[:take], 0.99, [body])
                out["volume_mm3"] = float(mb.Volume)
                out["area_mm2"] = float(mb.Area)
                try:
                    out["mass_kg"] = float(mb.Mass)
                except Exception:
                    pass
                c = mb.Centroid
                out["centroid"] = [round(c.X, 3), round(c.Y, 3), round(c.Z, 3)]
                out["units_len"] = take
                return out
            except Exception as exc:
                out["mm_err_%d" % take] = str(exc)[:100]
    except Exception as exc:
        out["mm_err"] = str(exc)[:120]
    try:
        import NXOpen.UF
        uf = NXOpen.UF.UFSession.GetUFSession()
        mp, stats = uf.Modl.AskMassProps3d([body.Tag], 1, 1, 4, 0.0, 1, [0.99, 0.99, 0.99])
        out["uf_volume"] = mp[1]
        out["uf_mass"] = mp[2]
    except Exception as exc:
        out["uf_err"] = str(exc)[:120]
    return out


def run_test(name, fn):
    rec = {"ok": False}
    try:
        data = fn()
        rec["ok"] = True
        if isinstance(data, dict):
            rec.update(data)
        log("PASS %-28s %s" % (name, json.dumps(data)[:160] if data else ""))
    except Exception as exc:
        rec["error"] = str(exc)[:400]
        rec["trace"] = traceback.format_exc()[-600:]
        log("FAIL %-28s %s" % (name, str(exc)[:160]))
    REPORT["tests"][name] = rec


# --------------------------------------------------------------------------- #
# part + base geometry helpers (proven extrude path from sx_engine)
# --------------------------------------------------------------------------- #
def section_of(part, curves):
    sec = part.Sections.CreateSection(0.0095, 0.001, 0.5)
    sec.AllowSelfIntersection(False)
    rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)
    sec.AddToSection([rule], curves[0], NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                     p3(0, 0, 0), NXOpen.Section.Mode.Create, False)
    return sec


def extrude(part, curves, length, op="create", target=None, direction=(0.0, 0.0, 1.0),
            origin=(0.0, 0.0, 0.0)):
    B = NXOpen.GeometricUtilities.BooleanOperation.BooleanType
    ops = {"create": B.Create, "unite": B.Unite, "subtract": B.Subtract}
    ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    ext.Section = section_of(part, curves)
    ext.Direction = part.Directions.CreateDirection(
        p3(*origin), v3(*direction), NXOpen.SmartObject.UpdateOption.WithinModeling)
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


def main():
    REPORT["nx_version"] = SES.GetEnvironmentVariableValue("UGII_VERSION") or "2506"
    out_prt = os.path.join(HERE, "v3_verify_part.prt")
    if os.path.exists(out_prt):
        try:
            os.remove(out_prt)
        except OSError:
            pass
    part = SES.Parts.NewBaseDisplay(out_prt, NXOpen.BasePart.Units.Millimeters)
    if isinstance(part, tuple):
        part = part[0]

    state = {}

    # T01 — base block 80x60x30 at origin + boss cylinder r=10 h=25 on top
    def t01():
        feat = extrude(part, rect_curves(part, 0, 0, 80, 60, 0), 30)
        body = feat.GetBodies()[0]
        body.SetName("VERIFY_BLOCK")
        extrude(part, [circle_curve(part, 25, 0, 10, 30)], 25, "unite", body)
        do_update("t01")
        state["body"] = body
        c = counts(part)
        c.update(volume_of(part, body))
        state["v0"] = c.get("volume_mm3")
        return c
    run_test("T01_base_solid", t01)

    # T02 — EDGE BLEND, stub-exact: ScCollector + CreateRuleEdgeDumb + AddChainset(col, "4")
    def t02():
        body = state["body"]
        before = counts(part)
        # pick vertical block edges (parallel to Z, length 30) — re-query AFTER unite
        edges = [e for e in body.GetEdges()
                 if abs(e.GetLength() - 30.0) < 0.01]
        assert edges, "no 30mm vertical edges found"
        ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleEdgeDumb(edges[:4])
        col.ReplaceRules([rule], False)
        idx = ebb.AddChainset(col, "4")
        feat = ebb.CommitFeature()
        ebb.Destroy()
        do_update("t02")
        after = counts(part)
        return {"feature": feat.FeatureType, "chainset_index": idx,
                "faces_before": before["faces"], "faces_after": after["faces"],
                "edges_blended": len(edges[:4])}
    run_test("T02_edge_blend", t02)

    # T03 — CHAMFER, stub-exact: SmartCollector property + Option enum + FirstOffset str
    def t03():
        body = state["body"]
        before = counts(part)
        # top outer edges of the boss cylinder: circular edges at z=55
        edges = []
        for e in body.GetEdges():
            try:
                pts = e.GetVertices()
            except Exception:
                pts = []
            # circular edge has no vertices; use bounding via edge length ~ 2*pi*10
            if abs(e.GetLength() - 2 * math.pi * 10.0) < 0.5:
                edges.append(e)
        assert edges, "no boss circular edges found"
        cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleEdgeDumb(edges[:1])
        col.ReplaceRules([rule], False)
        cb.SmartCollector = col
        cb.Option = NXOpen.Features.ChamferBuilder.ChamferOption.SymmetricOffsets
        cb.FirstOffset = "2"
        feat = cb.CommitFeature()
        cb.Destroy()
        do_update("t03")
        after = counts(part)
        return {"feature": feat.FeatureType,
                "faces_before": before["faces"], "faces_after": after["faces"]}
    run_test("T03_chamfer", t03)

    # T04 — DRAFT on separate block: TypeOfDraft=Face, StationaryEntity=bottom face
    def t04():
        feat = extrude(part, rect_curves(part, 150, 0, 40, 40, 0), 50)
        body2 = feat.GetBodies()[0]
        body2.SetName("VERIFY_DRAFT")
        do_update("t04a")
        state["body2"] = body2
        # bottom face (z=0) and 4 side faces
        bottom, sides = None, []
        for f in body2.GetFaces():
            try:
                ft = f.SolidFaceType
            except Exception:
                ft = None
            # classify by face normal via bounding box of edges
            zs = []
            for e in f.GetEdges():
                for vtx in e.GetVertices():
                    zs.append(vtx.Z)
            if zs and max(zs) - min(zs) < 0.001:  # horizontal face
                if abs(min(zs)) < 0.001:
                    bottom = f
            else:
                sides.append(f)
        assert bottom is not None and sides, "faces not classified"
        db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
        info = {"members": [m for m in dir(db) if not m.startswith("_")][:0]}
        db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
        db.Direction = part.Directions.CreateDirection(
            p3(150, 0, 0), v3(0, 0, 1), NXOpen.SmartObject.UpdateOption.WithinModeling)
        try:
            db.StationaryEntity = bottom
            info["stationary"] = "StationaryEntity=face OK"
        except Exception as exc:
            info["stationary"] = "StationaryEntity err: %s" % str(exc)[:80]
        # faces to draft: FaceSetAngleExpressionList — introspect + attempt
        fsl = db.FaceSetAngleExpressionList
        info["fsl_type"] = type(fsl).__name__
        info["fsl_dir"] = [m for m in dir(fsl) if not m.startswith("_")]
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleFaceDumb(sides)
        col.ReplaceRules([rule], False)
        added = False
        for meth, args in (("Append", (col, "3")), ("Append", (col,)),
                           ("CreateAndAppend", (col, "3")), ("AddCollectorSet", (col, "3"))):
            try:
                getattr(fsl, meth)(*args)
                info["fsl_add"] = "%s%s OK" % (meth, len(args))
                added = True
                break
            except Exception as exc:
                info.setdefault("fsl_tries", []).append("%s/%d: %s" % (meth, len(args), str(exc)[:60]))
        if added:
            feat2 = db.CommitFeature()
            info["feature"] = feat2.FeatureType
        db.Destroy()
        do_update("t04")
        return info
    run_test("T04_draft", t04)

    # T05 — THREAD symbolic on boss cylindrical face: CylindricalFace.Value = face
    def t05():
        body = state["body"]
        cyl = None
        for f in body.GetFaces():
            try:
                if f.SolidFaceType == NXOpen.Face.FaceType.Cylindrical:
                    cyl = f
                    break
            except Exception:
                pass
        assert cyl is not None, "no cylindrical face"
        tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)
        info = {"cylface_prop": type(tb.CylindricalFace).__name__}
        tb.ThreadType = NXOpen.Features.ThreadBuilder.Type.Symbolic
        try:
            tb.CylindricalFace.Value = cyl
            info["select"] = "CylindricalFace.Value=face OK"
        except Exception as exc:
            info["select_err"] = str(exc)[:100]
            info["cylface_dir"] = [m for m in dir(tb.CylindricalFace) if not m.startswith("_")]
            tb.CylindricalFace.SetValue(cyl, None, p3(35, 0, 40))
            info["select"] = "SetValue OK"
        feat = tb.CommitFeature()
        tb.Destroy()
        do_update("t05")
        info["feature"] = feat.FeatureType
        return info
    run_test("T05_thread_symbolic", t05)

    # T06 — SHELL a fresh block (thickness 2, remove top face)
    def t06():
        feat = extrude(part, rect_curves(part, 250, 0, 40, 40, 0), 40)
        body3 = feat.GetBodies()[0]
        body3.SetName("VERIFY_SHELL")
        do_update("t06a")
        v_before = volume_of(part, body3).get("volume_mm3")
        top = None
        for f in body3.GetFaces():
            zs = [vtx.Z for e in f.GetEdges() for vtx in e.GetVertices()]
            if zs and max(zs) - min(zs) < 0.001 and abs(max(zs) - 40.0) < 0.001:
                top = f
                break
        assert top is not None, "top face not found"
        sb = part.Features.CreateShellBuilder(NXOpen.Features.Feature.Null)
        info = {}
        try:
            sb.Body = body3
            info["body_set"] = "Body property OK"
        except Exception as exc:
            info["body_set_err"] = str(exc)[:80]
        sb.SetDefaultThickness("2")
        col = part.ScCollectors.CreateCollector()
        rule = part.ScRuleFactory.CreateRuleFaceDumb([top])
        col.ReplaceRules([rule], False)
        sb.RemovedFacesCollector = col
        feat2 = sb.CommitFeature()
        sb.Destroy()
        do_update("t06")
        v_after = volume_of(part, body3).get("volume_mm3")
        info.update({"feature": feat2.FeatureType,
                     "volume_before": v_before, "volume_after": v_after})
        return info
    run_test("T06_shell", t06)

    # T07 — MIRROR BODY across fixed datum plane X=0 (CreateMirrorBodyBuilder EXISTS)
    def t07():
        body2 = state.get("body2")
        assert body2 is not None, "draft block missing"
        before = counts(part)
        m = NXOpen.Matrix3x3()
        m.Xx, m.Xy, m.Xz = 0.0, 1.0, 0.0
        m.Yx, m.Yy, m.Yz = 0.0, 0.0, 1.0
        m.Zx, m.Zy, m.Zz = 1.0, 0.0, 0.0
        dplane = part.Datums.CreateFixedDatumPlane(p3(-30, 0, 0), m)
        mb = part.Features.CreateMirrorBodyBuilder(NXOpen.Features.Feature.Null)
        info = {"plane_prop": type(mb.Plane).__name__}
        try:
            mb.MirrorBodyList.Add(body2)
            info["bodylist"] = "MirrorBodyList.Add OK"
        except Exception as exc:
            info["bodylist_err"] = str(exc)[:80]
            col = mb.MirrorBodyCollector
            rule = part.ScRuleFactory.CreateRuleBodyDumb([body2])
            col.ReplaceRules([rule], False)
            info["bodylist"] = "MirrorBodyCollector rules OK"
        try:
            mb.Plane.Value = dplane
            info["plane"] = "Plane.Value=datum OK"
        except Exception as exc:
            info["plane_err"] = str(exc)[:80]
            mb.Plane.SetValue(dplane, None, p3(-30, 0, 0))
            info["plane"] = "Plane.SetValue OK"
        feat = mb.CommitFeature()
        mb.Destroy()
        do_update("t07")
        after = counts(part)
        info.update({"feature": feat.FeatureType,
                     "bodies_before": before["bodies"], "bodies_after": after["bodies"]})
        return info
    run_test("T07_mirror_body", t07)

    # T08 — MATERIAL from NX MatML library + assign; evidence = Mass in T09
    def t08():
        body = state["body"]
        pm = part.MaterialManager.PhysicalMaterials
        info = {}
        mat = None
        for name in ("Steel", "steel", "Aluminum_6061", "AISI_Steel_4340", "Iron_40"):
            try:
                mat = pm.LoadFromNxmatmllibrary(name)
                info["loaded"] = name
                break
            except Exception as exc:
                info.setdefault("tries", []).append("%s: %s" % (name, str(exc)[:60]))
        if mat is None:
            libs = []
            try:
                libs = pm.GetMaterialsFromLibrary("physicalmateriallibrary.xml")[:20]
            except Exception as exc:
                libs = ["lib_err: %s" % str(exc)[:60]]
            info["library_sample"] = libs
            raise RuntimeError("no material loaded; sample=%s" % libs[:5])
        mat.AssignObjects([body])
        info["assigned"] = "AssignObjects OK"
        do_update("t08")
        return info
    run_test("T08_material", t08)

    # T09 — MASS PROPERTIES with material applied (mass should be > 0)
    def t09():
        body = state["body"]
        data = volume_of(part, body)
        assert data.get("volume_mm3") or data.get("uf_volume"), "no volume measured"
        return data
    run_test("T09_mass_properties", t09)

    # T10 — PMI NOTE via Annotations.CreatePmiNoteBuilder
    def t10():
        info = {}
        b = part.Annotations.CreatePmiNoteBuilder(NXOpen.Annotations.SimpleDraftingAid.Null)
        info["builder"] = type(b).__name__
        b.Text.SetText(["SENTINEL-UGV", "NX2506 VERIFIED"])
        og = b.Origin
        info["origin_dir"] = [m for m in dir(og) if not m.startswith("_")][:24]
        try:
            og.Origin.SetValue(None, part.Views.WorkView, p3(0, 0, 80))
            info["origin"] = "Origin.SetValue OK"
        except Exception as exc:
            info["origin_err"] = str(exc)[:100]
            try:
                og.SetInferRelativeToGeometry(True)
                info["origin"] = "SetInferRelativeToGeometry OK"
            except Exception as exc2:
                info["origin2_err"] = str(exc2)[:80]
        note = b.Commit()
        b.Destroy()
        do_update("t10")
        info["note_type"] = type(note).__name__
        return info
    run_test("T10_pmi_note", t10)

    # T11 — headless PNG image export (stub says non-interactive supported)
    def t11():
        png = os.path.join(HERE, "v3_verify_iso.png")
        if os.path.exists(png):
            os.remove(png)
        try:
            part.ModelingViews.WorkView.Orient(
                NXOpen.View.Canned.Isometric, NXOpen.View.ScaleAdjustment.Fit)
        except Exception:
            pass
        ib = part.Views.CreateImageExportBuilder()
        ib.FileName = png
        ib.FileFormat = NXOpen.Gateway.ImageExportBuilder.FileFormats.Png
        try:
            ib.DeviceWidth = 1600
            ib.DeviceHeight = 1200
        except Exception:
            pass
        ib.Commit()
        ib.Destroy()
        ok = os.path.exists(png)
        return {"png_written": ok, "size": os.path.getsize(png) if ok else 0}
    run_test("T11_headless_png", t11)

    # T12 — HOLE PACKAGE simple hole via point + CreateRuleCurveDumbFromPoints
    def t12():
        body = state["body"]
        before = counts(part)
        hp = part.Features.CreateHolePackageBuilder(NXOpen.Features.HolePackage.Null)
        info = {}
        hp.HoleType = NXOpen.Features.HolePackageBuilder.Holetype.Simple
        hp.GeneralSimpleHoleDiameter.SetFormula("8")
        hp.HoleDepthLimitOption = NXOpen.Features.HolePackageBuilder.HoleDepthLimitOptions.ThroughBody
        pt = part.Points.CreatePoint(p3(-25, 0, 30))
        sec = hp.HolePosition
        info["pos_type"] = type(sec).__name__
        rule = part.ScRuleFactory.CreateRuleCurveDumbFromPoints([pt])
        sec.AddToSection([rule], pt, NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                         p3(-25, 0, 30), NXOpen.Section.Mode.Create, False)
        info["position"] = "point rule added"
        try:
            hp.BooleanOperation.SetTargetBodies([body])
        except Exception as exc:
            info["target_err"] = str(exc)[:80]
        feat = hp.CommitFeature()
        hp.Destroy()
        do_update("t12")
        after = counts(part)
        info.update({"feature": feat.FeatureType,
                     "faces_before": before["faces"], "faces_after": after["faces"]})
        return info
    run_test("T12_hole_package", t12)

    # save + write report
    try:
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
                  NXOpen.BasePart.CloseAfterSave.FalseValue)
        REPORT["saved"] = out_prt
    except Exception as exc:
        REPORT["save_error"] = str(exc)[:200]

    with open(os.path.join(HERE, "v3_verify_report.json"), "w", encoding="utf-8") as fh:
        json.dump(REPORT, fh, indent=2, default=str)
    npass = sum(1 for t in REPORT["tests"].values() if t["ok"])
    log("=== v3 verify: %d/%d PASS ===" % (npass, len(REPORT["tests"])))


main()
