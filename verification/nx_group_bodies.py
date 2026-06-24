"""Group the named motor bodies onto per-ROLE LAYERS so a whole role (all 48 magnets,
all 434 coils, the steel, ...) can be selected at once for material assignment / mesh
collectors -- instead of clicking 486 bodies one by one.

Run on the open motor_named part, or pass it explicitly:

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_group_bodies.py
    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_group_bodies.py -args motor_named.prt

Role -> layer:
    STATOR_STEEL = 11   ROTOR_STEEL = 12   MAGNET = 13
    COIL         = 14   SHAFT       = 15   HOUSING = 16

Each used layer is made Selectable (nothing is hidden) and given a named Layer Category,
and the part is saved. Requires the bodies to already carry the nx_builder role names
(MAGNET_000.., STATOR_STEEL, ..) -- run nx_list_bodies.py first if unsure.

AFTER running, to material one role (e.g. magnets):
  Format > Layer Settings -> make ONLY layer 13 Selectable (others Visible) ->
  Edit > Select All -> Assign Materials -> N42SH.  Repeat per layer.
"""
import os
import sys
import traceback

import NXOpen

ROLE_LAYER = {
    "STATOR_STEEL": 11, "ROTOR_STEEL": 12, "MAGNET": 13,
    "COIL": 14, "SHAFT": 15, "HOUSING": 16,
}


def role_of(name):
    """Map a body name (STATOR_STEEL / MAGNET_000 / COIL_123 ..) to its role key."""
    if not name:
        return None
    base = name.rsplit("_", 1)[0] if name[-1:].isdigit() else name
    return base if base in ROLE_LAYER else None


def _set_selectable(lm, layer, lw):
    if lm is None:
        return
    try:
        lm.SetState(layer, NXOpen.Layer.State.Selectable)
    except Exception as exc:
        lw.WriteLine("WARN SetState layer %d: %s" % (layer, exc))


def _move(lm, layer, bodies, lw):
    """Move bodies to a layer; prefer the manager call, fall back to per-body .Layer."""
    if lm is not None:
        try:
            lm.MoveDisplayableObjects(layer, bodies)
            return len(bodies)
        except Exception as exc:
            lw.WriteLine("NOTE MoveDisplayableObjects failed (%s); per-body fallback" % exc)
    moved = 0
    for b in bodies:
        try:
            b.Layer = layer
            moved += 1
        except Exception:
            pass
    return moved


def main():
    s = NXOpen.Session.GetSession()
    lw = s.ListingWindow
    lw.Open()

    for a in sys.argv[1:]:
        if a.lower().endswith(".prt") and os.path.exists(a):
            try:
                (getattr(s.Parts, "OpenDisplay", None) or s.Parts.Open)(a)
                lw.WriteLine("opened %s" % a)
            except Exception as exc:
                lw.WriteLine("WARN could not open %s: %s" % (a, exc))

    wp = s.Parts.Work
    if wp is None or getattr(wp, "Tag", 0) == 0:
        lw.WriteLine("FAIL no work part -- open motor_named.prt or pass it as -args.")
        return

    groups = {}
    unnamed = 0
    for b in wp.Bodies:
        try:
            if not b.IsSolidBody:
                continue
            r = role_of(b.Name or "")
        except Exception:
            r = None
        if r:
            groups.setdefault(r, []).append(b)
        else:
            unnamed += 1
    if not groups:
        lw.WriteLine("FAIL no role-named bodies found -- run on the native motor_named.prt "
                     "(STEP imports lose the names). Check with nx_list_bodies.py.")
        return

    lm = getattr(wp, "Layers", None)
    total = 0
    for role in sorted(groups, key=lambda r: ROLE_LAYER[r]):
        layer = ROLE_LAYER[role]
        bodies = groups[role]
        _set_selectable(lm, layer, lw)                 # keep visible BEFORE moving
        moved = _move(lm, layer, bodies, lw)
        total += moved
        lw.WriteLine("layer %2d  <- %4d x %s" % (layer, moved, role))
        try:
            wp.LayerCategories.CreateCategory(role, "motor_nx %s" % role, str(layer))
        except Exception:
            pass  # category naming is a bonus; the layer numbers are what matters

    try:
        wp.Save(NXOpen.BasePart.SaveComponents.TrueValue, NXOpen.BasePart.CloseAfterSave.FalseValue)
        lw.WriteLine("saved part")
    except Exception as exc:
        lw.WriteLine("WARN save failed: %s" % exc)

    lw.WriteLine("=== grouped %d bodies onto layers 11-16 (%d unnamed left on their layer) ==="
                 % (total, unnamed))
    lw.WriteLine("Next: Format>Layer Settings -> make ONLY the target role's layer Selectable")
    lw.WriteLine("      -> Edit>Select All -> Assign Materials. (11 steel-stator, 12 rotor,")
    lw.WriteLine("      13 magnet, 14 coil, 15 shaft, 16 housing)")


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        try:
            w = NXOpen.Session.GetSession().ListingWindow
            w.Open()
            w.WriteLine("FATAL nx_group_bodies:\n" + traceback.format_exc())
        except Exception:
            traceback.print_exc()
