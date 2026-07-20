# -*- coding: utf-8 -*-
"""NX 2506 API DISCOVERY Part 3 — Find FeatureCollection Create* methods."""
import NXOpen

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow
_lw.Open()

def log(msg):
    _lw.WriteLine(str(msg))

part = _session.Parts.NewBaseDisplay(
    "C:/Users/ahmet/Desktop/web/askeri/sentinel_ugv/_prt/api_disc3.prt",
    NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple):
    part = part[0]

fc = part.Features
all_fc = [m for m in dir(fc) if not m.startswith('_')]

log("=== FeatureCollection methods (filtered) ===")
for m in sorted(all_fc):
    if m.startswith('Create') and any(k in m.lower() for k in ['blend','chamf','thread','draft','mirror','edge','body','hole']):
        log("  %s" % m)

# Check all Create methods
log("\n=== ALL Create* methods ===")
for m in sorted(all_fc):
    if m.startswith('Create'):
        log("  %s" % m)

log("\n=== DONE ===")
