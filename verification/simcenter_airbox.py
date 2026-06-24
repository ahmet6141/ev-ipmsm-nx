"""Create the 3D AIR domain (an enclosing cylinder) for the Simcenter MAGNET model.

Run in the MASTER / IDEALIZED geometry part (the motor must be the work part), via
run_journal.exe::

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_airbox.py
    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\simcenter_airbox.py -args dia=400 len=210 subtract

It builds a cylinder centred on the motor axis (Z), larger than the stator OD and longer
than the active stack, names the solid ``AIR`` and puts it on layer 20. Magnetics is solved
in the air too, so this region is required.

  * default: just CREATE the air cylinder (it overlaps the motor; the FEM mesher treats the
    motor bodies as carving the air when meshed non-manifold/conformal).
  * ``subtract``: also subtract every motor solid FROM the cylinder (Keep Tools) so the air
    becomes a clean "cylinder minus motor" cavity (heavier with ~486 bodies, but gives an
    unambiguous air region).

The cylinder is built with the SAME extrude-of-circle sequence nx_builder.py uses (already
proven on this NX install). Every NXOpen step is guarded + logged to the Listing Window.

HONEST SCOPE: this prepares the air GEOMETRY only. Meshing, assigning the AIR material,
and the magnetisation / coil-current / A=0 boundary setup remain interactive in Simcenter
MAGNET (its NXOpen surface is too thin/version-specific to script blind); the materials you
already assigned to STATOR_STEEL / ROTOR_STEEL / MAGNET_* / COIL_* carry over.
"""
import math
import os
import sys
import traceback

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
try:
    import simcenter_common as sc
except Exception:
    sc = None

_S = NXOpen.Session.GetSession()
_TOL = 0.001


def _lw():
    w = _S.ListingWindow
    w.Open()
    return w


def _p3(x, y, z):
    return NXOpen.Point3d(float(x), float(y), float(z))


def _v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))


def make_air_cylinder(part, lw, dia_mm, len_mm, z0_mm):
    """Extrude a circle into a solid cylinder (the air domain). Mirrors nx_builder._extrude."""
    r = dia_mm / 2.0
    center = _p3(0.0, 0.0, z0_mm)
    arc = part.Curves.CreateArc(center, _v3(1, 0, 0), _v3(0, 1, 0), float(r), 0.0, 2.0 * math.pi)

    section = part.Sections.CreateSection(0.0095, _TOL, 0.5)
    section.AllowSelfIntersection(False)
    rule = part.ScRuleFactory.CreateRuleCurveDumb([arc])
    null = NXOpen.NXObject.Null
    section.AddToSection([rule], arc, null, null, _p3(0, 0, z0_mm),
                         NXOpen.Section.Mode.Create, False)

    ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
    try:
        ext.Section = section
        ext.Direction = part.Directions.CreateDirection(
            center, _v3(0, 0, 1), NXOpen.SmartObject.UpdateOption.WithinModeling)
        ext.Limits.StartExtend.Value.RightHandSide = "0"
        ext.Limits.EndExtend.Value.RightHandSide = repr(float(len_mm))
        ext.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Create
        feat = ext.CommitFeature()
    finally:
        try:
            ext.Destroy()
        except Exception:
            pass
    bodies = feat.GetBodies()
    body = bodies[0] if bodies else None
    lw.WriteLine("OK   air cylinder: dia %.0f mm, length %.0f mm, z %.0f..%.0f"
                 % (dia_mm, len_mm, z0_mm, z0_mm + len_mm))
    return body


def subtract_motor(part, lw, air_body):
    """Subtract every OTHER solid (the motor) from the air, keeping the tools."""
    tools = [b for b in part.Bodies if b.IsSolidBody and b.Tag != air_body.Tag]
    if not tools:
        lw.WriteLine("NOTE no motor bodies to subtract")
        return
    try:
        bb = part.Features.CreateBooleanBuilderUsingCollector(NXOpen.Features.BooleanFeature.Null)
        bb.Operation = NXOpen.Features.Feature.BooleanType.Subtract
        bb.Target = air_body
        try:
            bb.CopyTargets = False
            bb.CopyTools = True            # Keep Tools -> motor bodies survive
        except Exception:
            pass
        sc_collector = part.ScRuleFactory.CreateRuleBodyDumb(tools)
        coll = part.ScCollectors.CreateCollector()
        coll.ReplaceRules([sc_collector], False)
        bb.ToolBodyCollector = coll
        bb.CommitFeature()
        bb.Destroy()
        lw.WriteLine("OK   subtracted %d motor bodies from AIR (kept tools)" % len(tools))
    except Exception as exc:
        lw.WriteLine("WARN subtract failed (%s) -- keep the overlapping air cylinder and mesh "
                     "non-manifold/conformal instead." % exc)


def main():
    lw = _lw()
    lw.WriteLine("=== motor_nx Simcenter MAGNET air-box journal ===")

    # args: dia=, len=, subtract
    dia = 400.0
    length = None
    do_sub = False
    for a in sys.argv[1:]:
        al = a.lower()
        if al.startswith("dia="):
            try:
                dia = float(a.split("=", 1)[1])
            except Exception:
                pass
        elif al.startswith("len="):
            try:
                length = float(a.split("=", 1)[1])
            except Exception:
                pass
        elif al == "subtract":
            do_sub = True

    # stack length from fea_spec (to size + centre the cylinder axially)
    stack = 134.0
    if sc is not None:
        try:
            spec, _ = sc.load_spec()
            stack = float(sc.spec_get(spec, ["geometry_mm", "stack_length"], 134.0))
            od = float(sc.spec_get(spec, ["geometry_mm", "stator_OD"], 225.0))
            if dia < od * 1.4:
                dia = round(od * 1.8)     # ensure the cylinder clears the stator OD
            lw.WriteLine("spec: stack %.0f mm, stator OD %.0f mm -> air dia %.0f mm" % (stack, od, dia))
        except Exception as exc:
            lw.WriteLine("NOTE could not read fea_spec (%s) -- using defaults" % exc)
    if length is None:
        length = round(stack + 0.5 * stack)   # ~1.5x stack, end-region clearance
    z0 = -(length - stack) / 2.0              # centre the cylinder on the stack

    part = _S.Parts.Work
    if part is None or getattr(part, "Tag", 0) == 0:
        lw.WriteLine("FAIL no work part -- open the master/idealized motor geometry first.")
        return
    lw.WriteLine("work part: %s" % getattr(part, "Leaf", "<part>"))

    try:
        air = make_air_cylinder(part, lw, dia, length, z0)
    except Exception:
        lw.WriteLine("FAIL air cylinder build:\n" + traceback.format_exc())
        return
    if air is None:
        lw.WriteLine("FAIL no air body produced")
        return

    # name + layer so it is selectable as AIR in the FEM
    try:
        air.SetName("AIR")
        lw.WriteLine("OK   named body AIR")
    except Exception as exc:
        lw.WriteLine("WARN SetName AIR failed: %s" % exc)
    try:
        air.Layer = 20
        lw.WriteLine("OK   AIR -> layer 20")
    except Exception as exc:
        lw.WriteLine("WARN set layer failed: %s" % exc)

    if do_sub:
        subtract_motor(part, lw, air)

    _S.UpdateManager.DoUpdate(_S.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "airbox"))
    try:
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue, NXOpen.BasePart.CloseAfterSave.FalseValue)
        lw.WriteLine("OK   saved part")
    except Exception as exc:
        lw.WriteLine("WARN save failed: %s" % exc)

    lw.WriteLine("")
    lw.WriteLine("NEXT (interactive, in the FEM):")
    lw.WriteLine("  1. 3D-mesh the AIR body + all motor bodies (conformal); refine the air gap.")
    lw.WriteLine("  2. assign AIR material to the AIR body (mu_r = 1).")
    lw.WriteLine("  3. set the magnet magnetisation directions (per pole, N/S alternating).")
    lw.WriteLine("  4. coil currents (0 for cogging) + A=0 (flux-tangent) on the AIR outer face.")
    lw.WriteLine("  5. solve; then simcenter_emag.py can drive operating points + scoring.")
    lw.WriteLine("=== air-box journal done ===")


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    try:
        main()
    except Exception:
        try:
            w = _S.ListingWindow
            w.Open()
            w.WriteLine("FATAL simcenter_airbox:\n" + traceback.format_exc())
        except Exception:
            traceback.print_exc()
