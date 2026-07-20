# -*- coding: utf-8 -*-
"""NX 2506 API DISCOVERY — Find correct EdgeBlend, Chamfer, Thread, Draft API signatures."""
import NXOpen
import NXOpen.Features

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow
_lw.Open()

def log(msg):
    _lw.WriteLine(str(msg))

def discover():
    log("=" * 70)
    log("NX 2506 API DISCOVERY — Finding correct method names")
    log("=" * 70)
    
    # Create a simple part with one block
    part = _session.Parts.NewBaseDisplay(
        "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/api_discovery.prt",
        NXOpen.BasePart.Units.Millimeters)
    if isinstance(part, tuple):
        part = part[0]
    
    p0 = NXOpen.Point3d(0.0, 0.0, 0.0)
    p1 = NXOpen.Point3d(50.0, 0.0, 0.0)
    p2 = NXOpen.Point3d(50.0, 50.0, 0.0)
    p3 = NXOpen.Point3d(0.0, 50.0, 0.0)
    curves = [part.Curves.CreateLine(p0,p1), part.Curves.CreateLine(p1,p2),
              part.Curves.CreateLine(p2,p3), part.Curves.CreateLine(p3,p0)]
    section = part.Sections.CreateSection(0.0095, 0.001, 0.5)
    section.AllowSelfIntersection(False)
    rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)
    null_obj = NXOpen.NXObject.Null
    section.AddToSection([rule], curves[0], null_obj, null_obj, NXOpen.Point3d(0.0,0.0,0.0),
                         NXOpen.Section.Mode.Create, False)
    ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    ext.Section = section
    ext.Direction = part.Directions.CreateDirection(
        NXOpen.Point3d(0.0,0.0,0.0), NXOpen.Vector3d(0.0,0.0,1.0),
        NXOpen.SmartObject.UpdateOption.WithinModeling)
    ext.Limits.StartExtend.Value.RightHandSide = "0"
    ext.Limits.EndExtend.Value.RightHandSide = "50"
    feat = ext.CommitFeature()
    ext.Destroy()
    block = feat.GetBodies()[0]
    block.SetName("API_TEST_BLOCK")
    
    # --- DISCOVER EdgeBlendBuilder API ---
    log("\n--- EdgeBlendBuilder ---")
    ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
    methods = [m for m in dir(ebb) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(methods)))
    
    # Try different radius-setting approaches
    for attr_name in ['Radius', 'BlendRadius', 'DefaultRadius', 'RadiusValue',
                       'RadiusRightHandSide', 'SetRadius', 'SetBlendRadius',
                       'SetDefaultRadius', 'ConstantRadius']:
        if hasattr(ebb, attr_name):
            log("  FOUND: ebb.%s" % attr_name)
            val = getattr(ebb, attr_name)
            log("    type: %s, value: %s" % (type(val).__name__, str(val)[:80]))
    
    try:
        ebb.Destroy()
    except: pass
    
    # --- DISCOVER ChamferBuilder API ---
    log("\n--- ChamferBuilder ---")
    cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
    methods = [m for m in dir(cb) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(methods)))
    
    for attr_name in ['SymmetricOptions', 'ChamferOptions', 'OffsetOptions',
                       'Symmetric', 'Offset', 'SymmetricDistance',
                       'SymmetricOffset']:
        if hasattr(cb, attr_name):
            log("  FOUND: cb.%s" % attr_name)
    
    try:
        cb.Destroy()
    except: pass
    
    # --- DISCOVER ThreadFeatureBuilder API ---
    log("\n--- ThreadFeatureBuilder ---")
    tfb = part.Features.CreateThreadFeatureBuilder(NXOpen.Features.Feature.Null)
    methods = [m for m in dir(tfb) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(methods)))
    
    # Check ThreadType enum
    for attr_name in ['ThreadType', 'Type', 'ThreadStyle']:
        if hasattr(tfb, attr_name):
            log("  FOUND: tfb.%s" % attr_name)
            val = getattr(tfb, attr_name)
            log("    type: %s" % type(val).__name__)
    
    try:
        tfb.Destroy()
    except: pass
    
    # --- DISCOVER DraftBuilder API ---
    log("\n--- DraftBuilder ---")
    db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
    methods = [m for m in dir(db) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(methods)))
    try:
        db.Destroy()
    except: pass
    
    # --- DISCOVER MirrorBodyBuilder API ---
    log("\n--- MirrorBodyBuilder ---")
    mb = part.Features.CreateMirrorBodyBuilder(NXOpen.Features.Feature.Null)
    methods = [m for m in dir(mb) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(methods)))
    try:
        mb.Destroy()
    except: pass
    
    # --- DISCOVER MaterialManager ---
    log("\n--- MaterialManager ---")
    mm = part.MaterialManager
    mm_methods = [m for m in dir(mm) if not m.startswith('_')]
    log("  Available methods/attrs: %s" % ", ".join(sorted(mm_methods)))
    
    log("\n=== DISCOVERY COMPLETE ===")

if __name__ == "__main__" or "UGII_ROOT_DIR" in __import__('os').environ:
    discover()
