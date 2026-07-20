# -*- coding: utf-8 -*-
"""NX 2506 API DISCOVERY Part 4 — Find Chamfer/EdgeBlend/Mirror/Draft API details."""
import NXOpen

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow
_lw.Open()

def log(msg):
    _lw.WriteLine(str(msg))

part = _session.Parts.NewBaseDisplay(
    "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/api_disc4.prt",
    NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple):
    part = part[0]

# Check EdgeBlendBuilder in detail
log("=== EdgeBlendBuilder detail ===")
ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
# Look for radius-related properties
for a in dir(ebb):
    if not a.startswith('_') and 'adius' in a.lower():
        log("  %s" % a)
ebb.Destroy()

# Check ChamferBuilder in detail  
log("\n=== ChamferBuilder detail ===")
cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
for a in dir(cb):
    if not a.startswith('_') and ('offset' in a.lower() or 'method' in a.lower() or 'option' in a.lower()):
        log("  %s" % a)
cb.Destroy()

# Check ThreadBuilder in detail
log("\n=== ThreadBuilder detail ===")
tb = part.Features.CreateThreadBuilder(NXOpen.Features.Feature.Null)
for a in dir(tb):
    if not a.startswith('_'):
        log("  %s" % a)
tb.Destroy()

# Check DraftBuilder in detail
log("\n=== DraftBuilder detail ===")
db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
for a in dir(db):
    if not a.startswith('_') and ('angle' in a.lower() or 'direction' in a.lower()):
        log("  %s" % a)
db.Destroy()

# Check for Mirror body
log("\n=== Mirror body search ===")
fc = part.Features
for m in dir(fc):
    if 'irror' in m.lower() and m.startswith('Create'):
        log("  %s" % m)
# Also check Assemblies
for m in dir(NXOpen.Assemblies):
    if 'irror' in m.lower():
        log("  Assemblies.%s" % m)

log("\n=== DONE ===")
