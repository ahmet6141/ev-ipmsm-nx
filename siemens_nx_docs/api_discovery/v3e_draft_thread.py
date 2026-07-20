"""v3e_draft_thread.py — iteration 5: the two last features.

    "%UGII_ROOT_DIR%\\run_journal.exe" v3e_draft_thread.py

  T04 draft  -> part.CreateExpressionCollectorSet(col, "3", <units>, 0) then
                FaceSetAngleExpressionList.Append(ecs)   (stub NX:48050)
  T05 thread -> Manual input + ShaftPreference=MajorDiameter (+ shaft dia between
                minor/major as fallback)
"""

import json
import math
import os
import traceback

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities

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
        log("PASS %-20s %s" % (name, json.dumps(info, default=str)[:170]))
    except Exception as exc:
        rec["error"] = str(exc)[:300]
        rec["trace"] = traceback.format_exc()[-400:]
        log("FAIL %-20s %s | %s" % (name, str(exc)[:90],
                                    json.dumps(info, default=str)[:180]))
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


def horiz_face(body, z_target):
    for f in body.GetFaces():
        zs = [vt.Z for e in f.GetEdges() for vt in e.GetVertices()]
        if zs and max(zs) - min(zs) < 0.001 and abs(zs[0] - z_target) < 0.001:
            return f
    return None


def main():
    out_prt = os.path.join(HERE, "v3e_verify_part.prt")
    if os.path.exists(out_prt):
        try:
            os.remove(out_prt)
        except OSError:
            pass
    part = SES.Parts.NewBaseDisplay(out_prt, NXOpen.BasePart.Units.Millimeters)
    if isinstance(part, tuple):
        part = part[0]

    featA = extrude(part, rect_curves(part, 0, 0, 40, 40, 0), 50)
    bodyA = featA.GetBodies()[0]
    bodyA.SetName("DRAFT_BLK")
    featC = extrude(part, rect_curves(part, 200, 0, 80, 60, 0), 30)
    bodyC = featC.GetBodies()[0]
    bodyC.SetName("THREAD_BLK")
    extrude(part, [circle_curve(part, 200, 0, 10, 30)], 25, "unite", bodyC)
    do_update("base")

    # T04 DRAFT — via CreateExpressionCollectorSet
    def t04(info):
        bottom = horiz_face(bodyA, 0.0)
        sides = []
        for f in bodyA.GetFaces():
            zs = [vt.Z for e in f.GetEdges() for vt in e.GetVertices()]
            if zs and max(zs) - min(zs) > 0.001:
                sides.append(f)
        db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
        db.AngleTolerance = 0.5
        db.DistanceTolerance = 0.001
        db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
        db.Direction = part.Directions.CreateDirection(
            p3(0, 0, 0), v3(0, 0, 1), NXOpen.SmartObject.UpdateOption.WithinModeling)
        try:
            db.DraftReferencesMethod = (
                NXOpen.Features.DraftBuilder.DraftReferencesMethods.StationaryFace)
            info["refmethod"] = "StationaryFace"
        except Exception as exc:
            info["refmethod_err"] = str(exc)[:60]
        sr = db.StationaryReference
        rule_b = part.ScRuleFactory.CreateRuleFaceDumb([bottom])
        sr.ReplaceRules([rule_b], False)
        col = part.ScCollectors.CreateCollector()
        rule_s = part.ScRuleFactory.CreateRuleFaceDumb(sides)
        col.ReplaceRules([rule_s], False)
        ecs = None
        for units in ("Degrees", "deg", ""):
            try:
                ecs = part.CreateExpressionCollectorSet(col, "3", units, 0)
                info["ecs_units"] = units or "(empty)"
                break
            except Exception as exc:
                info.setdefault("ecs_tries", []).append("%s: %s" % (units, str(exc)[:50]))
        assert ecs is not None, "CreateExpressionCollectorSet failed"
        db.FaceSetAngleExpressionList.Append(ecs)
        info["append"] = "ecs OK"
        feat = db.CommitFeature()
        db.Destroy()
        do_update("t04")
        info["feature"] = feat.FeatureType
    run_test("T04_draft", t04)

    # T05 THREAD — Manual + ShaftPreference
    def t05(info):
        cyl = None
        for f in bodyC.GetFaces():
            try:
                if f.SolidFaceType == NXOpen.Face.FaceType.Cylindrical:
                    cyl = f
            except Exception:
                pass
        top_boss = horiz_face(bodyC, 55.0)
        assert cyl is not None
        tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)
        tb.ThreadType = NXOpen.Features.ThreadBuilder.Type.Symbolic
        tb.ThreadInput = NXOpen.Features.ThreadBuilder.Input.Manual
        tb.ShaftPreference = (
            NXOpen.Features.ThreadBuilder.ShaftSizePreference.MajorDiameter)
        info["shaft_pref"] = "MajorDiameter"
        tb.CylindricalFace.Value = cyl
        if top_boss is not None:
            try:
                tb.StartObject.Value = top_boss
            except Exception:
                pass
        for prop, val in (("MajorDiameterExp", "20"), ("MinorDiameterExp", "17.5"),
                          ("ShaftDiameterExp", "19.0"), ("PitchExp", "2.5"),
                          ("AngleExp", "60"), ("ThreadLength", "20")):
            try:
                getattr(tb, prop).RightHandSide = val
                info.setdefault("set", []).append(prop)
            except Exception as exc:
                info.setdefault("set_err", []).append("%s: %s" % (prop, str(exc)[:40]))
        feat = tb.CommitFeature()
        tb.Destroy()
        do_update("t05")
        info["feature"] = feat.FeatureType
    run_test("T05_thread", t05)

    try:
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
                  NXOpen.BasePart.CloseAfterSave.FalseValue)
    except Exception as exc:
        REPORT["save_error"] = str(exc)[:150]

    with open(os.path.join(HERE, "v3e_verify_report.json"), "w", encoding="utf-8") as fh:
        json.dump(REPORT, fh, indent=2, default=str)
    npass = sum(1 for t in REPORT["tests"].values() if t["ok"])
    log("=== v3e: %d/%d PASS ===" % (npass, len(REPORT["tests"])))


main()
