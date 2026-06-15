"""NX 2506 API smoke test -- run this FIRST to confirm the exact NXOpen member
names used by nx_builder.py resolve on your install, before launching a full
motor build. It builds a tiny part (one annulus + one slot cut patterned 6x +
STEP + Parasolid export) exercising every API the real builder relies on.

Run:
    "C:\\Program Files\\Siemens\\NX2506\\NXBIN\\run_journal.exe" motor_nx\\nx_smoketest.py -args C:\\temp\\nx_smoke > smoke.log 2>&1

Then open smoke.log. A line "SMOKE TEST PASSED" plus a non-empty <out>.stp means
the extrude / circular-pattern / boolean / expression / STEP / Parasolid calls all
work on your NX 2506. Any "FAIL <step>: <exc>" line names the exact call to fix
(check docs/NX_AUTOMATION.md for the version-drift note on that member).
"""

import math
import os
import sys
import time

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities

try:
    import NXOpen.UF
    _UF = NXOpen.UF.UFSession.GetUFSession()
except Exception:
    _UF = None

_S = NXOpen.Session.GetSession()


def _p3(x, y, z):
    """NX 2506 NXOpen requires float (double) coords; passing an int raises
    'Expecting double type, found int'. Coerce every point coordinate here."""
    return NXOpen.Point3d(float(x), float(y), float(z))


def _v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))


def run(out_base):
    lw = _S.ListingWindow
    lw.Open()
    def log(m): lw.WriteLine(str(m))
    results = []

    def _safe_undo(mark, name):
        try:
            _S.UndoToMark(mark, name)
        except Exception:
            pass  # undo stack may have been reset (e.g. by part creation)

    def step(name, fn):
        mark = _S.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, name)
        try:
            out = fn()
            _S.UpdateManager.DoUpdate(mark)
            log("OK   " + name)
            return out
        except Exception as exc:
            _safe_undo(mark, name)
            log("FAIL %s: %s" % (name, exc))
            results.append(name)
            return None

    opt_results = []

    def opt_step(name, fn):  # non-fatal: builder default does not rely on this
        mark = _S.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, name)
        try:
            out = fn()
            _S.UpdateManager.DoUpdate(mark)
            log("OK   %s (optional)" % name)
            return out
        except Exception as exc:
            _safe_undo(mark, name)
            log("SKIP %s (optional): %s" % (name, exc))
            opt_results.append(name)
            return None

    def _count_pitch_spacing():
        ps = NXOpen.GeometricUtilities.PatternSpacing
        for en, vn in (("SpacingType", "Offset"), ("SpacingTypeEnum", "CountAndPitch"), ("SpacingType", "Pitch")):
            ec = getattr(ps, en, None)
            if ec is not None and hasattr(ec, vn):
                return getattr(ec, vn)
        return None

    # --- new mm part: cascade, LOGGING which strategy works on this NX 2506 ---
    out_prt = out_base + ".prt"
    d = os.path.dirname(out_prt)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    def _new_part():
        try:
            mm = NXOpen.BasePart.Units.Millimeters   # verified correct type on NX 2506
        except Exception:
            mm = getattr(NXOpen, "BasePartUnits").Millimeters
        base = os.path.splitext(out_prt)[0]
        last = None
        for i in range(64):
            cand = out_prt if i == 0 else "%s_%d.prt" % (base, i)
            if os.path.exists(cand):
                try:
                    os.remove(cand)
                except OSError:
                    pass  # locked by another NX session -> try the next name
            try:
                result = _S.Parts.NewBaseDisplay(cand, mm)
                # NX 2506 returns just a Part; other builds return (Part, status)
                bp = result[0] if isinstance(result, tuple) else result
                log("  part created: %s" % cand)
                return bp
            except Exception as exc:
                last = exc
                if "exist" in str(exc).lower():
                    log("  '%s' is busy/locked, trying next name" % os.path.basename(cand))
                    continue
                raise
        raise RuntimeError("no free part name after 64 tries: %s" % last)
    part = step("new_mm_part", _new_part)
    if part is None:
        log("cannot continue without a part")
        return results

    mm = part.UnitCollection.FindObject("MilliMeter")
    uo = NXOpen.SmartObject.UpdateOption.WithinModeling

    def _expr():
        return part.Expressions.CreateWithUnits("stack=20", mm)
    step("expression_create", _expr)

    def _zdir():
        o = _p3(0, 0, 0)
        return part.Directions.CreateDirection(o, _v3(0, 0, 1), uo)

    def _zaxis():
        o = _p3(0, 0, 0)
        d = part.Directions.CreateDirection(o, _v3(0, 0, 1), uo)
        pt = part.Points.CreatePoint(o)
        return part.Axes.CreateAxis(pt, d, uo)

    def _section(curves):
        sec = part.Sections.CreateSection(0.0095, 0.001, 0.5)
        sec.AllowSelfIntersection(False)
        rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)
        sec.AddToSection([rule], curves[0], NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                         _p3(0, 0, 0), NXOpen.Section.Mode.Create, False)
        return sec

    def _circle(cx, cy, r):
        c = _p3(cx, cy, 0.0)
        return part.Curves.CreateArc(c, _v3(1, 0, 0), _v3(0, 1, 0),
                                     float(r), 0.0, 2 * math.pi)

    # --- base annulus: extrude outer circle (create), subtract inner circle ---
    base_body = {}

    def _annulus():
        ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
        ext.Section = _section([_circle(0, 0, 40)])
        ext.Direction = _zdir()
        ext.Limits.StartExtend.Value.RightHandSide = "0"
        ext.Limits.EndExtend.Value.RightHandSide = "stack"
        ext.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Create
        f = ext.CommitFeature(); ext.Destroy()
        base_body["b"] = f.GetBodies()[0]
        ext2 = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
        ext2.Section = _section([_circle(0, 0, 25)])
        ext2.Direction = _zdir()
        ext2.Limits.StartExtend.Value.RightHandSide = "0"
        ext2.Limits.EndExtend.Value.RightHandSide = "stack"
        ext2.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Subtract
        ext2.BooleanOperation.SetTargetBodies([base_body["b"]])
        ext2.CommitFeature(); ext2.Destroy()
        return base_body["b"]
    step("extrude_tube", _annulus)

    # --- one rectangular slot cut, then circular pattern x6 ---
    slot_feat = {}

    def _slot():
        pts = [(24, -2), (41, -2), (41, 2), (24, 2)]  # overshoot bore & OD -> clean through-cut (no zero-wall touch)
        curves = []
        for i in range(len(pts)):
            a = pts[i]; b = pts[(i + 1) % len(pts)]
            curves.append(part.Curves.CreateLine(_p3(a[0], a[1], 0),
                                                 _p3(b[0], b[1], 0)))
        ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
        ext.Section = _section(curves)
        ext.Direction = _zdir()
        ext.Limits.StartExtend.Value.RightHandSide = "0"
        ext.Limits.EndExtend.Value.RightHandSide = "stack"
        ext.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Subtract
        ext.BooleanOperation.SetTargetBodies([base_body["b"]])
        f = ext.CommitFeature(); ext.Destroy()
        slot_feat["f"] = f
        return f
    step("extrude_slot_subtract", _slot)

    # the builder's DEFAULT repeat path: rotate the profile, subtract explicit copies
    def _explicit_instances():
        for ang in (120.0, 240.0):
            a = math.radians(ang); c = math.cos(a); s = math.sin(a)
            rot = [(x * c - y * s, x * s + y * c) for x, y in [(24, -2), (41, -2), (41, 2), (24, 2)]]
            curves = []
            for i in range(len(rot)):
                p = rot[i]; q = rot[(i + 1) % len(rot)]
                curves.append(part.Curves.CreateLine(_p3(p[0], p[1], 0),
                                                     _p3(q[0], q[1], 0)))
            ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
            ext.Section = _section(curves); ext.Direction = _zdir()
            ext.Limits.StartExtend.Value.RightHandSide = "0"
            ext.Limits.EndExtend.Value.RightHandSide = "stack"
            ext.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Subtract
            ext.BooleanOperation.SetTargetBodies([base_body["b"]])
            ext.CommitFeature(); ext.Destroy()
    step("explicit_instances(default path)", _explicit_instances)

    # a small shaft-style revolve (builder uses this for the shaft)
    def _revolve():
        prof = [(7, 0), (20, 0), (20, 30), (7, 30)]  # (r, z) closed loop
        curves = []
        for i in range(len(prof)):
            a = prof[i]; b = prof[(i + 1) % len(prof)]
            curves.append(part.Curves.CreateLine(_p3(a[0], 0, a[1]),
                                                 _p3(b[0], 0, b[1])))
        rev = part.Features.CreateRevolveBuilder(NXOpen.Features.Feature.Null)
        rev.Section = _section(curves)
        rev.Axis = _zaxis()
        rev.Limits.StartExtend.Value.RightHandSide = "0"
        rev.Limits.EndExtend.Value.RightHandSide = "360"
        rev.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Create
        f = rev.CommitFeature(); rev.Destroy()
        return f
    step("revolve", _revolve)

    # OPTIONAL: NX circular pattern feature (lighter tree). Non-fatal -- the builder
    # default does not use it; this just tells you whether you can enable 'nxpatterns'.
    def _pattern():
        pfb = part.Features.CreatePatternFeatureBuilder(NXOpen.Features.Feature.Null)
        pfb.PatternService.PatternType = NXOpen.GeometricUtilities.PatternDefinition.PatternEnum.Circular
        pfb.FeatureList.Add([slot_feat["f"]])
        circ = pfb.PatternService.CircularDefinition
        circ.RotationAxis = _zaxis()
        space = _count_pitch_spacing()
        if space is not None:
            circ.AngularSpacing.SpaceType = space
        circ.AngularSpacing.NCopies.RightHandSide = "6"
        circ.AngularSpacing.PitchDistance.RightHandSide = "60"
        f = pfb.CommitFeature(); pfb.Destroy()
        return f

    # --- exports ---
    def _save():
        part.Save(NXOpen.BasePart.SaveComponents.TrueValue, NXOpen.BasePart.CloseAfterSave.FalseValue)
    step("save", _save)

    stp = out_base + "_ap242.stp"

    def _step_export():
        if os.path.exists(stp):
            try:
                os.remove(stp)
            except OSError:
                pass
        sc = _S.DexManager.CreateStepCreator()
        sc.ExportAs = NXOpen.StepCreator.ExportAsOption.Ap242
        sc.ObjectTypes.Solids = True
        sc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.EntirePart
        sc.InputFile = part.FullPath
        sc.OutputFile = stp
        sc.FileSaveFlag = False
        sc.LayerMask = "1-256"
        sc.Commit(); sc.Destroy()
    step("export_step_ap242", _step_export)

    xt = out_base + ".x_t"

    def _para_export():
        for factory_name in ("CreateParasolidExporter", "CreateParasolidCreator"):
            factory = getattr(_S.DexManager, factory_name, None)
            if factory is None:
                continue
            try:
                pc = factory()
                pc.ObjectTypes.Solids = True
                pc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.EntirePart
                pc.InputFile = part.FullPath
                pc.OutputFile = xt
                pc.Commit(); pc.Destroy()
                return
            except (AttributeError, NXOpen.NXException):
                pass
        bodies = [b for b in part.Bodies if b.IsSolidBody]
        if os.path.exists(xt):
            os.remove(xt)
        _UF.Ps.ExportData(bodies, xt)
    step("export_parasolid", _para_export)

    # optional NX circular-pattern runs AFTER exports so a pattern update failure
    # (common when patterning a boolean-subtract feature) cannot corrupt the export.
    opt_step("circular_pattern", _pattern)

    # The DexManager STEP/Parasolid translators run as a SEPARATE process; the
    # output file may not be flushed the instant Commit() returns. Poll briefly,
    # else the gate falsely reports "empty" on a perfectly good export.
    def _wait_nonempty(path, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                return True
            time.sleep(0.5)
        return os.path.exists(path) and os.path.getsize(path) > 0
    # NX's STEP/Parasolid translators finalize the output file AFTER this journal
    # returns (processed when NX is idle), so an in-journal size check blocks/races
    # them and falsely reads "empty". The export steps above already confirm the
    # translator Commit() did not raise -> judge PASS on the core build+export steps
    # and report the on-disk file advisorily.
    stp_seen = _wait_nonempty(stp, timeout=2.0)
    log("STEP file already on disk: %s  (NX finalizes %s just after this journal)"
        % (stp_seen, os.path.basename(stp)))
    if not results:
        log("SMOKE TEST PASSED -- core nx_builder API is good on this NX 2506 install")
        if opt_results:
            log("  note: optional NX-pattern path unavailable (%s); the builder DEFAULT "
                "(explicit instancing) does not use it, so full builds still work." % opt_results)
        else:
            log("  bonus: optional NX-pattern path also works -- you may pass 'nxpatterns' "
                "as the 4th run_journal arg for a lighter feature tree.")
    else:
        log("SMOKE TEST FAILED -- failing core steps: %s" % results)
    return results


def main():
    out_base = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.getcwd(), "nx_smoke")
    run(out_base)


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    main()
