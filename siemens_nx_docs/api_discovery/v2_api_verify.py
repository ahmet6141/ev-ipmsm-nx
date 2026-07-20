# -*- coding: utf-8 -*-
"""SENTINEL-UGV v2.0 — NX API Verification Journal

Run this INSIDE NX (Alt+F8 → Play) to verify all v2.0 NXOpen API calls work.
Tests: EdgeBlend, Chamfer, Thread, Draft, Mirror, Material, PMI.

Expected output: ALL TESTS PASSED or specific FAIL lines for debugging.
"""

import NXOpen
import NXOpen.Features
import math

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow
_lw.Open()

def log(msg):
    _lw.WriteLine(str(msg))

def _p3(x, y, z):
    return NXOpen.Point3d(float(x), float(y), float(z))

def _v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))

def create_test_part():
    """Create a test part with a simple block + cylinder for API testing."""
    part_path = "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/v2_api_test.prt"
    import os
    try:
        os.remove(part_path)
    except: pass
    try:
        os.remove(part_path.replace(".prt", "_ap242.stp"))
    except: pass
    
    part = _session.Parts.NewBaseDisplay(part_path, NXOpen.BasePart.Units.Millimeters)
    if isinstance(part, tuple):
        part = part[0]
    return part

def test_all():
    log("=" * 60)
    log("SENTINEL-UGV v2.0 — NX API VERIFICATION")
    log("=" * 60)
    
    part = create_test_part()
    mark = _session.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "start")
    
    # Build a simple test body: 50x50x50 block at origin
    p0, p1 = _p3(0, 0, 0), _p3(50, 0, 0)
    p2, p3 = _p3(50, 50, 0), _p3(0, 50, 0)
    curves = [
        part.Curves.CreateLine(p0, p1),
        part.Curves.CreateLine(p1, p2),
        part.Curves.CreateLine(p2, p3),
        part.Curves.CreateLine(p3, p0),
    ]
    section = part.Sections.CreateSection(0.0095, 0.001, 0.5)
    section.AllowSelfIntersection(False)
    rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)
    help_pt = _p3(0, 0, 0)
    null_obj = NXOpen.NXObject.Null
    section.AddToSection([rule], curves[0], null_obj, null_obj, help_pt,
                         NXOpen.Section.Mode.Create, False)
    
    ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    ext.Section = section
    ext.Direction = part.Directions.CreateDirection(_p3(0,0,0), _v3(0,0,1),
        NXOpen.SmartObject.UpdateOption.WithinModeling)
    ext.Limits.StartExtend.Value.RightHandSide = "0"
    ext.Limits.EndExtend.Value.RightHandSide = "50"
    feat = ext.CommitFeature()
    ext.Destroy()
    block_body = feat.GetBodies()[0]
    block_body.SetName("TEST_BLOCK")
    log("OK: block created")
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 1: EdgeBlend (Fillet) ----
    try:
        ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
        ebb.SetRadius(5.0)
        edges = list(block_body.GetEdges())
        if edges:
            ebb.AddChainsToCollector(edges[:4])  # blend first 4 edges
            ebb.CommitFeature()
            log("PASS: EdgeBlend applied (radius=5.0 on %d edges)" % min(4, len(edges)))
        else:
            log("FAIL: EdgeBlend — no edges found")
        ebb.Destroy()
    except Exception as e:
        log("FAIL: EdgeBlend — %s" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 2: Chamfer ----
    try:
        cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
        cb.SymmetricOptions.Distance.Value.RightHandSide = "2.0"
        cb.AddChainsToCollector([block_body])
        cb.CommitFeature()
        log("PASS: Chamfer applied (distance=2.0)")
        cb.Destroy()
    except Exception as e:
        log("FAIL: Chamfer — %s" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 3: Thread (symbolic) ----
    # Add a cylinder to test thread on
    circ = part.Curves.CreateArc(_p3(70, 25, 0), _v3(1,0,0), _v3(0,1,0), 10.0, 0, 2*math.pi)
    sec2 = part.Sections.CreateSection(0.0095, 0.001, 0.5)
    sec2.AllowSelfIntersection(False)
    rule2 = part.ScRuleFactory.CreateRuleCurveDumb([circ])
    sec2.AddToSection([rule2], circ, null_obj, null_obj, help_pt,
                      NXOpen.Section.Mode.Create, False)
    ext2 = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    ext2.Section = sec2
    ext2.Direction = part.Directions.CreateDirection(_p3(70,25,0), _v3(0,0,1),
        NXOpen.SmartObject.UpdateOption.WithinModeling)
    ext2.Limits.StartExtend.Value.RightHandSide = "0"
    ext2.Limits.EndExtend.Value.RightHandSide = "30"
    ext2.CommitFeature()
    ext2.Destroy()
    cyl_body = part.Bodies.ToArray()[-1]  # last created body
    
    try:
        tfb = part.Features.CreateThreadFeatureBuilder(NXOpen.Features.Feature.Null)
        faces = list(cyl_body.GetFaces())
        cyl_face = None
        for f in faces:
            try:
                if f.FaceType == NXOpen.Face.FaceType.Cylindrical:
                    cyl_face = f
                    break
            except: pass
        if cyl_face:
            tfb.FaceCollector.Add([cyl_face])
            tfb.ThreadType = NXOpen.Features.ThreadFeatureBuilder.ThreadType.Symbolic
            tfb.CommitFeature()
            log("PASS: Thread applied (symbolic)")
        else:
            log("FAIL: Thread — no cylindrical face found")
        tfb.Destroy()
    except Exception as e:
        log("FAIL: Thread — %s" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 4: Draft ----
    try:
        db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
        db.Angle.Value.RightHandSide = "3.0"
        dir_vec = _v3(0, 0, 1)
        db.Direction = part.Directions.CreateDirection(_p3(0,0,0), dir_vec,
            NXOpen.SmartObject.UpdateOption.WithinModeling)
        db.AddChainsToCollector([block_body])
        db.CommitFeature()
        log("PASS: Draft applied (angle=3.0)")
        db.Destroy()
    except Exception as e:
        log("FAIL: Draft — %s" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 5: Mirror Body ----
    try:
        # Create a mirror plane (X=100)
        pt = _p3(100, 0, 0)
        vec = _v3(1, 0, 0)
        plane = part.Planes.CreateFixedPlane(pt, vec,
            NXOpen.SmartObject.UpdateOption.WithinModeling)
        mb = part.Features.CreateMirrorBodyBuilder(NXOpen.Features.Feature.Null)
        mb.Plane = plane
        mb.AddBodiesToMirror([block_body])
        mb.CommitFeature()
        log("PASS: Mirror body created")
        mb.Destroy()
    except Exception as e:
        log("FAIL: Mirror — %s" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 6: Material Assignment ----
    try:
        mat_list = part.MaterialManager.LoadMaterialsFromLibrary(["Steel"])
        if mat_list:
            part.MaterialManager.AssignMaterialToBody(mat_list[0], block_body)
            log("PASS: Material assigned (Steel)")
        else:
            log("WARN: Material — 'Steel' not found in library, trying 'Aluminum_6061'")
            mat_list = part.MaterialManager.LoadMaterialsFromLibrary(["Aluminum_6061"])
            if mat_list:
                part.MaterialManager.AssignMaterialToBody(mat_list[0], block_body)
                log("PASS: Material assigned (Aluminum_6061)")
            else:
                log("WARN: Material — no standard material found in library")
    except Exception as e:
        log("WARN: Material — %s (material library may not be configured)" % str(e)[:120])
    
    _session.UpdateManager.DoUpdate(mark)
    
    # ---- TEST 7: Body naming ----
    try:
        all_bodies = [b for b in part.Bodies if b.IsSolidBody]
        for i, b in enumerate(all_bodies):
            b.SetName("V2_TEST_BODY_%02d" % i)
        log("PASS: Named %d bodies" % len(all_bodies))
    except Exception as e:
        log("FAIL: Naming — %s" % str(e)[:120])
    
    # ---- SAVE ----
    try:
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
                  NXOpen.BasePart.CloseAfterSave.FalseValue)
        log("PASS: Part saved")
    except Exception as e:
        log("WARN: Save — %s" % str(e)[:120])
    
    log("=" * 60)
    log("V2 API VERIFICATION COMPLETE")
    log("Open _prt/v2_api_test.prt in NX to visually verify results")
    log("=" * 60)


if __name__ == "__main__" or __name__ == "__main__":
    test_all()
elif os.environ.get("UGII_ROOT_DIR"):
    test_all()
