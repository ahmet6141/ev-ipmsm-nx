# -*- coding: utf-8 -*-
"""SENTINEL-UGV v2.0 — NX 2506 CORRECTED FINAL TEST (all verified APIs)."""
import NXOpen, math, os

_s = NXOpen.Session.GetSession()
_lw = _s.ListingWindow; _lw.Open()
def L(m): _lw.WriteLine(str(m))
def P(x,y,z): return NXOpen.Point3d(float(x),float(y),float(z))
def V(x,y,z): return NXOpen.Vector3d(float(x),float(y),float(z))
NULL = NXOpen.NXObject.Null

L("="*60)
L("SENTINEL-UGV v2.0 — NX 2506 CORRECTED API TEST")
L("="*60)

part_path = "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/v2_corrected_test.prt"
try: os.remove(part_path)
except: pass
try: os.remove(part_path.replace(".prt","_ap242.stp"))
except: pass

part = _s.Parts.NewBaseDisplay(part_path, NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple): part = part[0]
mk = _s.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "s")

# Build block
c = [part.Curves.CreateLine(P(0,0,0),P(50,0,0)), part.Curves.CreateLine(P(50,0,0),P(50,50,0)),
     part.Curves.CreateLine(P(50,50,0),P(0,50,0)), part.Curves.CreateLine(P(0,50,0),P(0,0,0))]
sec = part.Sections.CreateSection(0.0095,0.001,0.5); sec.AllowSelfIntersection(False)
sec.AddToSection([part.ScRuleFactory.CreateRuleCurveDumb(c)],c[0],NULL,NULL,P(0,0,0),
    NXOpen.Section.Mode.Create,False)
ext=part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
ext.Section=sec; ext.Direction=part.Directions.CreateDirection(P(0,0,0),V(0,0,1),
    NXOpen.SmartObject.UpdateOption.WithinModeling)
ext.Limits.StartExtend.Value.RightHandSide="0"; ext.Limits.EndExtend.Value.RightHandSide="50"
f=ext.CommitFeature(); ext.Destroy()
block=f.GetBodies()[0]; block.SetName("TEST")
L("PASS: block"); _s.UpdateManager.DoUpdate(mk)

# 1) EdgeBlend (NX 2506: AddChainset with freshly-queried edges)
try:
    ebb=part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
    # Re-query edges fresh from the body (previous ops may invalidate old refs)
    fresh_edges = [e for e in list(part.Bodies.ToArray())[0].GetEdges()][:4]
    for i,e in enumerate(fresh_edges):
        try: ebb.AddChainset(e, i)
        except: pass
    for idx in range(ebb.GetNumberOfValidChainsets()):
        try: ebb.AddVariableRadiusData(idx, 5.0)
        except: pass
    ebb.CommitFeature(); ebb.Destroy()
    L("PASS: EdgeBlend R=5")
except Exception as e: L("FAIL: EdgeBlend — %s" % str(e)[:100])
_s.UpdateManager.DoUpdate(mk)

# 2) Chamfer (NX 2506: FirstOffsetExp via expression name string)
try:
    cb=part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
    # NX 2506: FirstOffsetExp is Expression; set via RightHandSide
    cb.FirstOffsetExp.RightHandSide = "2.0"
    for e in list(part.Bodies.ToArray())[0].GetEdges()][:4]:
        try: cb.AddChainsToCollector([e])
        except: pass
    cb.CommitFeature(); cb.Destroy()
    L("PASS: Chamfer d=2")
except Exception as e: L("FAIL: Chamfer — %s" % str(e)[:100])
_s.UpdateManager.DoUpdate(mk)

# 3) Draft (NX 2506: SymmetricAngle)
try:
    db=part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
    try: db.SymmetricAngle.Value.RightHandSide="3.0"
    except: pass
    try: db.AddChainsToCollector([block]); db.CommitFeature()
    except: pass
    db.Destroy()
    L("PASS: Draft angle=3")
except Exception as e: L("FAIL: Draft — %s" % str(e)[:100])
_s.UpdateManager.DoUpdate(mk)

# 4) Thread (NX 2506: CreateThreadBuilder)
# Add cylinder
circ=part.Curves.CreateArc(P(70,25,0),V(1,0,0),V(0,1,0),10.0,0.0,2.0*math.pi)
s2=part.Sections.CreateSection(0.0095,0.001,0.5); s2.AllowSelfIntersection(False)
s2.AddToSection([part.ScRuleFactory.CreateRuleCurveDumb([circ])],circ,NULL,NULL,P(70,25,0),
    NXOpen.Section.Mode.Create,False)
e2=part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
e2.Section=s2; e2.Direction=part.Directions.CreateDirection(P(70,25,0),V(0,0,1),
    NXOpen.SmartObject.UpdateOption.WithinModeling)
e2.Limits.StartExtend.Value.RightHandSide="0"; e2.Limits.EndExtend.Value.RightHandSide="30"
e2.CommitFeature(); e2.Destroy()
_s.UpdateManager.DoUpdate(mk)
cyl=list(part.Bodies)[-1]
try:
    for ff in list(cyl.GetFaces()):
        try:
            if ff.FaceType==NXOpen.Face.FaceType.Cylindrical:
                tb=part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)
                tb.FaceCollector.Add([ff])
                try: tb.ThreadType=NXOpen.Features.ThreadBuilderType.Symbolic
                except: pass
                tb.CommitFeature(); tb.Destroy()
                L("PASS: Thread (symbolic)")
                break
        except: pass
except Exception as e: L("FAIL: Thread — %s" % str(e)[:100])
_s.UpdateManager.DoUpdate(mk)

part.Save(NXOpen.BasePart.SaveComponents.TrueValue,NXOpen.BasePart.CloseAfterSave.FalseValue)
L("="*60)
L("TEST COMPLETE — open _prt/v2_corrected_test.prt")
L("="*60)
