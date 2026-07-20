# -*- coding: utf-8 -*-
"""SENTINEL-UGV v2.0 — SIMPLEST POSSIBLE NX 2506 API TEST."""
import NXOpen, math, os

_s = NXOpen.Session.GetSession()
_lw = _s.ListingWindow; _lw.Open()
def L(m): _lw.WriteLine(str(m))
def P(x,y,z): return NXOpen.Point3d(float(x),float(y),float(z))
def V(x,y,z): return NXOpen.Vector3d(float(x),float(y),float(z))
NULL = NXOpen.NXObject.Null

L("="*60); L("SENTINEL-UGV v2.0 — SIMPLEST NX 2506 TEST"); L("="*60)

pp = "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/v2_simple_test.prt"
try: os.remove(pp)
except: pass
try: os.remove(pp.replace(".prt","_ap242.stp"))
except: pass

part = _s.Parts.NewBaseDisplay(pp, NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple): part = part[0]
mk = _s.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "s")

# Build block
c = [part.Curves.CreateLine(P(0,0,0),P(50,0,0)),part.Curves.CreateLine(P(50,0,0),P(50,50,0)),
     part.Curves.CreateLine(P(50,50,0),P(0,50,0)),part.Curves.CreateLine(P(0,50,0),P(0,0,0))]
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

# Get fresh body reference and edges each time
def get_body():
    return [b for b in part.Bodies if b.IsSolidBody][0]

def get_edges(n=4):
    return list(get_body().GetEdges())[:n]

# 1) Chamfer — simplest: set FirstOffset as string
try:
    cb=part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
    # FirstOffset is str type in NX 2506 — set it as formula string
    cb.FirstOffset = "2.0"
    for e in get_edges(4):
        try: cb.SmartCollector.Add(e)
        except:
            try: cb.AddChainsToCollector([e])
            except: pass
    cb.CommitFeature(); cb.Destroy()
    L("PASS: Chamfer d=2")
except Exception as e: L("FAIL: Chamfer — %s" % str(e)[:120])
_s.UpdateManager.DoUpdate(mk)

# 2) EdgeBlend — fresh edges after Update
try:
    ebb=part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
    for i,e in enumerate(get_edges(4)):
        ebb.AddChainset(e, i)
    for idx in range(ebb.GetNumberOfValidChainsets()):
        ebb.AddVariableRadiusData(idx, 5.0)
    ebb.CommitFeature(); ebb.Destroy()
    L("PASS: EdgeBlend R=5")
except Exception as e: L("FAIL: EdgeBlend — %s" % str(e)[:120])
_s.UpdateManager.DoUpdate(mk)

# 3) Draft — already proven
try:
    db=part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
    db.SymmetricAngle.Value.RightHandSide="3.0"
    db.AddChainsToCollector([get_body()])
    db.CommitFeature(); db.Destroy()
    L("PASS: Draft angle=3")
except Exception as e: L("FAIL: Draft — %s" % str(e)[:120])
_s.UpdateManager.DoUpdate(mk)

# 4) Thread — add cylinder and thread it
circ=part.Curves.CreateArc(P(70.0,25.0,0.0),V(1.0,0.0,0.0),V(0.0,1.0,0.0),10.0,0.0,2.0*math.pi)
s2=part.Sections.CreateSection(0.0095,0.001,0.5); s2.AllowSelfIntersection(False)
s2.AddToSection([part.ScRuleFactory.CreateRuleCurveDumb([circ])],circ,NULL,NULL,P(70.0,25.0,0.0),
    NXOpen.Section.Mode.Create,False)
e2=part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
e2.Section=s2; e2.Direction=part.Directions.CreateDirection(P(70.0,25.0,0.0),V(0.0,0.0,1.0),
    NXOpen.SmartObject.UpdateOption.WithinModeling)
e2.Limits.StartExtend.Value.RightHandSide="0"; e2.Limits.EndExtend.Value.RightHandSide="30"
e2.CommitFeature(); e2.Destroy()
_s.UpdateManager.DoUpdate(mk)

try:
    cyl_bodies = [b for b in part.Bodies if b.IsSolidBody and b.Name != "TEST"]
    if cyl_bodies:
        cyl = cyl_bodies[-1]
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
    else:
        L("WARN: Thread — no separate cylinder body found")
except Exception as e: L("FAIL: Thread — %s" % str(e)[:120])
_s.UpdateManager.DoUpdate(mk)

part.Save(NXOpen.BasePart.SaveComponents.TrueValue,NXOpen.BasePart.CloseAfterSave.FalseValue)
L("="*60); L("TEST COMPLETE"); L("="*60)
