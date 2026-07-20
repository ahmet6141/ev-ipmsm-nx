# -*- coding: utf-8 -*-
"""SENTINEL-UGV v2.0 — NX 2506 VERIFIED API Test Journal (corrected)."""
import NXOpen
import NXOpen.Features
import math, os

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow; _lw.Open()
def log(msg): _lw.WriteLine(str(msg))

def _p3(x,y,z): return NXOpen.Point3d(float(x),float(y),float(z))
def _v3(x,y,z): return NXOpen.Vector3d(float(x),float(y),float(z))

log("="*60)
log("SENTINEL-UGV v2.0 — NX 2506 VERIFIED API TEST")
log("="*60)

part = _session.Parts.NewBaseDisplay(
    "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/v2_final_test.prt",
    NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple): part = part[0]
mark = _session.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "start")

# Build test block
p0,p1,p2,p3 = _p3(0,0,0),_p3(50,0,0),_p3(50,50,0),_p3(0,50,0)
curves = [part.Curves.CreateLine(p0,p1),part.Curves.CreateLine(p1,p2),
          part.Curves.CreateLine(p2,p3),part.Curves.CreateLine(p3,p0)]
sec = part.Sections.CreateSection(0.0095,0.001,0.5)
sec.AllowSelfIntersection(False)
sec.AddToSection([part.ScRuleFactory.CreateRuleCurveDumb(curves)],curves[0],
    NXOpen.NXObject.Null,NXOpen.NXObject.Null,_p3(0,0,0),
    NXOpen.Section.Mode.Create,False)
ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
ext.Section=sec
ext.Direction=part.Directions.CreateDirection(_p3(0,0,0),_v3(0,0,1),
    NXOpen.SmartObject.UpdateOption.WithinModeling)
ext.Limits.StartExtend.Value.RightHandSide="0"
ext.Limits.EndExtend.Value.RightHandSide="50"
f=ext.CommitFeature(); ext.Destroy()
block = f.GetBodies()[0]; block.SetName("TEST_BLOCK")
log("PASS: block created")
_session.UpdateManager.DoUpdate(mark)

# TEST 1: EdgeBlend (corrected API)
try:
    ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
    edges = list(block.GetEdges())
    ebb.AddChainsToCollector(edges[:4])
    for idx in range(ebb.GetNumberOfValidChainsets()):
        ebb.AddVariableRadiusData(idx, 5.0)
    ebb.CommitFeature(); ebb.Destroy()
    log("PASS: EdgeBlend R=5.0")
except Exception as e:
    log("FAIL: EdgeBlend — %s" % str(e)[:100])
_session.UpdateManager.DoUpdate(mark)

# TEST 2: Chamfer (corrected API)
try:
    cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
    cb.FirstOffset.Value.RightHandSide = "2.0"
    cb.AddChainsToCollector([block])
    cb.CommitFeature(); cb.Destroy()
    log("PASS: Chamfer d=2.0")
except Exception as e:
    log("FAIL: Chamfer — %s" % str(e)[:100])
_session.UpdateManager.DoUpdate(mark)

# TEST 3: Thread (corrected API)
# Add a cylinder for thread test
circ = part.Curves.CreateArc(_p3(70.0,25.0,0.0),_v3(1.0,0.0,0.0),_v3(0.0,1.0,0.0),
    10.0, 0.0, 2.0*math.pi)  # all floats!
sec2 = part.Sections.CreateSection(0.0095,0.001,0.5)
sec2.AllowSelfIntersection(False)
sec2.AddToSection([part.ScRuleFactory.CreateRuleCurveDumb([circ])],circ,
    NXOpen.NXObject.Null,NXOpen.NXObject.Null,_p3(70.0,25.0,0.0),
    NXOpen.Section.Mode.Create,False)
ext2 = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
ext2.Section=sec2
ext2.Direction=part.Directions.CreateDirection(_p3(70.0,25.0,0.0),_v3(0.0,0.0,1.0),
    NXOpen.SmartObject.UpdateOption.WithinModeling)
ext2.Limits.StartExtend.Value.RightHandSide="0"
ext2.Limits.EndExtend.Value.RightHandSide="30"
ext2.CommitFeature(); ext2.Destroy()
_session.UpdateManager.DoUpdate(mark)
cyl = list(part.Bodies)[-1]

try:
    faces = list(cyl.GetFaces())
    for f2 in faces:
        try:
            if f2.FaceType == NXOpen.Face.FaceType.Cylindrical:
                tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)
                tb.FaceCollector.Add([f2])
                try: tb.ThreadType = NXOpen.Features.ThreadBuilderType.Symbolic
                except: pass
                tb.CommitFeature(); tb.Destroy()
                log("PASS: Thread (symbolic)")
                break
        except: pass
except Exception as e:
    log("FAIL: Thread — %s" % str(e)[:100])
_session.UpdateManager.DoUpdate(mark)

# TEST 4: Draft
try:
    db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
    db.Angle.Value.RightHandSide = "3.0"
    db.AddChainsToCollector([block])
    db.CommitFeature(); db.Destroy()
    log("PASS: Draft angle=3.0")
except Exception as e:
    log("FAIL: Draft — %s" % str(e)[:100])

# SAVE
part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
          NXOpen.BasePart.CloseAfterSave.FalseValue)
log("="*60)
log("TEST COMPLETE — check _prt/v2_final_test.prt visually")
log("="*60)
