"""Read-only NX diagnostic -- list every solid body's NAME and LAYER, grouped.

Run on the part that is open in NX, or pass a .prt to open first:

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_list_bodies.py
    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_list_bodies.py -args motor_named.prt

This DEFINITIVELY answers "did nx_builder's body naming take?". It reads
``body.Name`` straight from the part:

  * If you see  48 x MAGNET, 434 x COIL, 1 x STATOR_STEEL ...  -> the names ARE set
    (they may simply not show in the model FEATURE tree -- they DO appear in the FEM
    body-selection / Information>Object, which is where you need them).
  * If you see  486 x (blank)  -> SetName did not take on this NX build; tell me and
    I will switch nx_builder to grouping bodies onto named LAYERS instead.

Nothing is modified -- this is a pure read-out.
"""
import os
import sys
import traceback

import NXOpen


def main():
    s = NXOpen.Session.GetSession()
    lw = s.ListingWindow
    lw.Open()

    # optionally open a .prt passed as -args; otherwise use the open work part
    for a in sys.argv[1:]:
        if a.lower().endswith(".prt") and os.path.exists(a):
            try:
                opener = getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open
                opener(a)
                lw.WriteLine("opened %s" % a)
            except Exception as exc:
                lw.WriteLine("WARN could not open %s: %s" % (a, exc))

    wp = s.Parts.Work
    if wp is None or getattr(wp, "Tag", 0) == 0:
        lw.WriteLine("FAIL no work part -- open a .prt or pass one as -args.")
        return

    by_name = {}
    by_layer = {}
    total = 0
    blank = 0
    for b in wp.Bodies:
        try:
            if not b.IsSolidBody:
                continue
        except Exception:
            continue
        total += 1
        try:
            nm = b.Name or ""
        except Exception:
            nm = ""
        if not nm:
            blank += 1
        # collapse the trailing _### index so MAGNET_000.. -> MAGNET
        key = (nm.rsplit("_", 1)[0] if (nm and nm[-1:].isdigit()) else nm) or "(blank)"
        by_name[key] = by_name.get(key, 0) + 1
        try:
            ly = b.Layer
            by_layer[ly] = by_layer.get(ly, 0) + 1
        except Exception:
            pass

    leaf = getattr(wp, "Leaf", "<part>")
    lw.WriteLine("=== %s : %d solid bodies (%d with a blank name) ===" % (leaf, total, blank))
    lw.WriteLine("-- by NAME --")
    for k in sorted(by_name):
        lw.WriteLine("  %4d x %s" % (by_name[k], k))
    if by_layer:
        lw.WriteLine("-- by LAYER --")
        for ly in sorted(by_layer):
            lw.WriteLine("  layer %3d : %d body(ies)" % (ly, by_layer[ly]))
    lw.WriteLine("=== verdict: %s ===" % (
        "names ARE set (use them in the FEM body selection)" if blank < total
        else "names did NOT take -> ask me to switch nx_builder to named LAYERS"))


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        try:
            w = NXOpen.Session.GetSession().ListingWindow
            w.Open()
            w.WriteLine("FATAL nx_list_bodies:\n" + traceback.format_exc())
        except Exception:
            traceback.print_exc()
