# -*- coding: utf-8 -*-
"""NX 2506 API DISCOVERY Part 2 — Find correct feature creation methods."""
import NXOpen
import NXOpen.Features

_session = NXOpen.Session.GetSession()
_lw = _session.ListingWindow
_lw.Open()

def log(msg):
    _lw.WriteLine(str(msg))

# Search FeatureCollection for relevant methods
fc = NXOpen.Session.GetSession().Parts.Work
all_feature_methods = [m for m in dir(NXOpen.Features) if not m.startswith('_')]
log("=== NXOpen.Features module contents (filtered) ===")
for m in sorted(all_feature_methods):
    if any(k in m.lower() for k in ['blend', 'chamfer', 'thread', 'draft', 'mirror', 'edge', 'body']):
        log("  %s" % m)

# Now check specifically what Create methods exist
log("\n=== Create* methods in Features ===")
for m in sorted(all_feature_methods):
    if m.startswith('Create') and any(k in m.lower() for k in ['blend', 'chamf', 'thread', 'draft', 'mirror', 'edge', 'body']):
        log("  Features.%s" % m)

# All FeatureCollection Create* methods
if True:
    log("\n=== ALL Create* in Features (first 70) ===")
    create_methods = [m for m in sorted(all_feature_methods) if m.startswith('Create')]
    for m in create_methods[:70]:
        log("  %s" % m)

log("\n=== DONE ===")
