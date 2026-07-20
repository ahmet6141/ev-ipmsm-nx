# -*- coding: utf-8 -*-
"""NX 2506 API DISCOVERY Part 5 — Draft angle property, EdgeBlend chainset API."""
import NXOpen

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow; _lw.Open()
def log(msg): _lw.WriteLine(str(msg))

part = _session.Parts.NewBaseDisplay(
    "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/api_disc5.prt",
    NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple): part = part[0]

# DraftBuilder Angle discovery
log("=== DraftBuilder ALL properties ===")
db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
for a in dir(db):
    if not a.startswith('_'):
        log("  %s" % a)
db.Destroy()

# ChamferBuilder - check if FirstOffsetExp exists
log("\n=== ChamferBuilder offset/exp ===")
cb = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
log("  FirstOffset type: %s" % type(cb.FirstOffset).__name__)
log("  FirstOffsetExp exists: %s" % hasattr(cb, 'FirstOffsetExp'))
if hasattr(cb, 'FirstOffsetExp'):
    log("  FirstOffsetExp type: %s" % type(cb.FirstOffsetExp).__name__)
cb.Destroy()

# EdgeBlendBuilder - check chainset API
log("\n=== EdgeBlendBuilder collector/chainset ===")
ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
ebb_methods = [m for m in dir(ebb) if not m.startswith('_') and ('chain' in m.lower() or 'collect' in m.lower())]
log("  Chain/Collect methods: %s" % ", ".join(ebb_methods))
ebb.Destroy()

log("\n=== DONE ===")
