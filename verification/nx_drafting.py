"""P6 deliverable -- formal NX Drafting output (title block + GD&T + notes).

Run INSIDE NX (headless or interactive) via run_journal.exe on a built motor part:

    "%UGII_ROOT_DIR%\\run_journal.exe" verification\\nx_drafting.py -args motor_v8.prt [A3|A2] [dxf]

What it does, all driven from the LIVE design data (motor_nx.manufacturing /
em_design / drawings -- the same source as the model and the BOM):

  1. enters the Drafting application and adds a drawing sheet (default A3, 1:2),
  2. places associative model views (FRONT lamination plane + a TFR-ISO pictorial),
  3. stamps a TITLE BLOCK note (title / part / material / scale / units / dwg no / rev),
  4. stamps the GD&T + critical-dimension SCHEDULE and the GENERAL NOTES as annotation
     blocks (feature / nominal / tolerance / datum-frame text from manufacturing.TOLERANCES),
  5. optionally (the "dxf" arg) imports the already-dimensioned 2D sheets that
     motor_nx.drawings emits (assembly / stator / rotor) onto their own sheets.

HONEST SCOPE -- like nx_builder.py, this is the hardened-but-unattended automation.
NXOpen Drafting view/sheet/PMI member names drift between NX releases, so every step
is wrapped, logged to the Listing Window, and degrades to a warning (never aborts).
The title block, GD&T schedule and notes are placed as native NX notes positioned on
the sheet -- the values are correct and live; you only fine-tune placement and (if you
want fully-associative PMI) re-attach a few frames to edges interactively. The "dxf"
path lands the fully-dimensioned drawings.py sheets directly with no clean-up.
"""
import os
import sys
import traceback

import NXOpen
import NXOpen.Annotations
import NXOpen.Drawings

_SESSION = NXOpen.Session.GetSession()


def _lw():
    w = _SESSION.ListingWindow
    w.Open()
    return w


# --------------------------------------------------------------------------- #
# live design data (same modules that drive the model + BOM)
# --------------------------------------------------------------------------- #
def _load_design(json_path=None):
    """Import the NX-independent design modules in-session (drop cached copies so
    edits are picked up without restarting NX -- the nx_builder convention)."""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    for m in [m for m in list(sys.modules) if m == "motor_nx" or m.startswith("motor_nx.")]:
        del sys.modules[m]
    from motor_nx.params import MotorParams
    from motor_nx import manufacturing as mfg
    from motor_nx import em_design
    p = MotorParams.load_json(json_path) if json_path else MotorParams()
    return p, mfg, em_design


def _title_rows(p, em_design, sheet, scale):
    g = em_design.derive(p)
    return [
        "EV TRACTION IPMSM",
        "PART : %s" % p.name,
        "SHEET: %s     SCALE %s     UNITS mm" % (sheet, scale),
        "STATOR %dslot / ROTOR %dpole   OD %.0f  L %.0f"
        % (p.stator.slot_count, p.rotor.pole_count, p.stator.outer_diameter, p.stack_length),
        "MAGNET %s   LAM %s" % (p.material.magnet_grade, p.material.electrical_steel),
        "DWG No EVM-%s     REV A     DERIVED FROM params.py" % str(sheet).split()[0],
    ]


# --------------------------------------------------------------------------- #
# drafting helpers (guarded; log each outcome)
# --------------------------------------------------------------------------- #
SHEET_SIZE = {"A4": (210.0, 297.0), "A3": (420.0, 297.0), "A2": (594.0, 420.0),
              "A1": (841.0, 594.0), "A0": (1189.0, 841.0)}


def enter_drafting(work, lw):
    try:
        _SESSION.ApplicationSwitchImmediate("UG_APP_DRAFTING")
        lw.WriteLine("OK   entered Drafting application")
        return True
    except Exception as exc:
        lw.WriteLine("WARN could not switch to Drafting: %s" % exc)
        return False


def add_sheet(work, lw, size="A3", scale=0.5, name="SH1"):
    w, h = SHEET_SIZE.get(size, SHEET_SIZE["A3"])
    try:
        b = work.DrawingSheets.NewDrawingSheetBuilder()
    except Exception:
        try:
            b = work.DrawingSheets.DrawingSheetBuilder(NXOpen.Drawings.DrawingSheet.Null)
        except Exception as exc:
            lw.WriteLine("WARN sheet builder unavailable: %s  (add a sheet by hand, re-run for notes)" % exc)
            return None
    try:
        # size: prefer a custom-size sheet so the metric A-sizes are exact.
        for attr, val in (("Height", h), ("Length", w), ("SheetName", name),
                          ("Scale", scale), ("Name", name)):
            try:
                setattr(b, attr, val)
            except Exception:
                pass
        # projection angle: first-angle (ISO) where the member exists
        try:
            b.ProjectionAngle = NXOpen.Drawings.DrawingSheetBuilder.SheetProjectionAngle.FirstAngleProjection
        except Exception:
            pass
        sheet = b.Commit()
        lw.WriteLine("OK   sheet %s (%.0f x %.0f, 1:%.0f)" % (name, w, h, 1.0 / scale if scale else 1))
        return sheet
    except Exception as exc:
        lw.WriteLine("WARN sheet commit failed: %s" % exc)
        return None
    finally:
        try:
            b.Destroy()
        except Exception:
            pass


def add_base_views(work, lw, scale=0.5):
    """Add a FRONT (lamination plane) and a TFR-ISO pictorial view of the model."""
    placed = 0
    try:
        vb = work.Drawings.CreateBaseViewBuilder(NXOpen.Drawings.BaseView.Null)
    except Exception as exc:
        lw.WriteLine("WARN base-view builder unavailable: %s (add views by hand)" % exc)
        return placed
    try:
        try:
            vb.Style.Scale.ViewScaleMethodValue = (
                NXOpen.Drawings.ViewStyleScaleBuilder.ScaleType.UserDefined)
        except Exception:
            pass
        wants = [("FRONT", 200.0, 200.0), ("TFR-ISO", 200.0, 470.0)]
        for view_name, x, y in wants:
            try:
                try:
                    vb.ModelViewName = view_name
                except Exception:
                    pass
                loc = NXOpen.Point3d(x, y, 0.0)
                view = vb.AddBaseView(loc) if hasattr(vb, "AddBaseView") else None
                if view is None:
                    # legacy: set location then Commit
                    try:
                        vb.Locate.SetValue(loc)
                    except Exception:
                        pass
                    view = vb.Commit()
                placed += 1
                lw.WriteLine("OK   view %-8s at (%.0f, %.0f)" % (view_name, x, y))
            except Exception as exc:
                lw.WriteLine("WARN view %s failed: %s" % (view_name, exc))
    finally:
        try:
            vb.Destroy()
        except Exception:
            pass
    return placed


def place_note(work, lw, lines, x, y, height=3.5, tag=""):
    """Place a multi-line drafting note with its top-left near (x, y) on the sheet."""
    text = lines if isinstance(lines, list) else [str(lines)]
    try:
        nb = work.Annotations.CreateDraftingNoteBuilder(NXOpen.Annotations.DraftingNote.Null)
    except Exception:
        try:
            nb = work.Annotations.CreateNoteBuilder(NXOpen.Annotations.Note.Null)
        except Exception as exc:
            lw.WriteLine("WARN note builder unavailable (%s): %s" % (tag, exc))
            return None
    try:
        # text
        for setter in ("Text", "TextBlock"):
            obj = getattr(nb, setter, None)
            if obj is not None and hasattr(obj, "SetText"):
                try:
                    obj.SetText(text)
                    break
                except Exception:
                    pass
        else:
            try:
                nb.Text.TextBlock.SetText(text)
            except Exception:
                pass
        # lettering size
        try:
            nb.Style.LetteringStyle.GeneralTextSize = height
        except Exception:
            pass
        # origin
        try:
            nb.Origin.SetInferRelativeToGeometry(False)
            nb.Origin.Origin.SetValue(NXOpen.NXObject.Null, NXOpen.View.Null,
                                      NXOpen.Point3d(float(x), float(y), 0.0))
        except Exception:
            try:
                nb.Origin.Origin.SetValue(None, NXOpen.View.Null,
                                          NXOpen.Point3d(float(x), float(y), 0.0))
            except Exception:
                pass
        note = nb.Commit()
        lw.WriteLine("OK   note %-10s (%d line(s)) at (%.0f, %.0f)" % (tag, len(text), x, y))
        return note
    except Exception as exc:
        lw.WriteLine("WARN note %s commit failed: %s" % (tag, exc))
        return None
    finally:
        try:
            nb.Destroy()
        except Exception:
            pass


def gdt_lines(p, mfg):
    """GD&T / critical-dimension schedule as note text from manufacturing.TOLERANCES."""
    rows = mfg.TOLERANCES(p)
    out = ["GD&T / CRITICAL-DIMENSION SCHEDULE", ""]
    for t in rows:
        out.append("- %s" % t.get("feature", ""))
        nom = t.get("nominal", ""); tol = t.get("tolerance", ""); gdt = t.get("gdt", "")
        if nom:
            out.append("    nominal: %s" % nom)
        out.append("    tol: %s" % tol)
        if gdt:
            out.append("    GD&T: %s" % gdt)
    return out


def notes_lines(mfg):
    return ["GENERAL NOTES"] + ["%d. %s" % (i + 1, n) for i, n in enumerate(mfg.general_notes())]


def bom_lines(p, mfg):
    bom = mfg.bill_of_materials(p)
    out = ["BILL OF MATERIALS (modelled)"]
    for it in bom["line_items"]:
        out.append("  %-24s %5.2f kg  x%s" % (it["component"][:24], it["mass_kg"], it.get("qty", 1)))
    out.append("  %-24s %5.2f kg" % ("TOTAL", bom["total_mass_kg"]))
    return out


# --------------------------------------------------------------------------- #
# optional: import the already-dimensioned drawings.py DXF sheets
# --------------------------------------------------------------------------- #
def import_drawings_dxf(work, lw, p):
    """Generate the drawings.py DXF sheets and import them onto NX drawing sheets.
    Reuses the fully-dimensioned assembly/stator/rotor output (title block + dims +
    GD&T notes already laid out) -- no NX dimensioning needed."""
    try:
        from motor_nx import drawings as dwg
    except Exception as exc:
        lw.WriteLine("WARN drawings module import failed: %s" % exc)
        return 0
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "drawings")
    try:
        written = dwg.write_drawings(p, out_dir)
        dxfs = [w for w in written if w.lower().endswith(".dxf")]
        lw.WriteLine("OK   generated %d DXF sheet(s) in %s" % (len(dxfs), out_dir))
    except Exception as exc:
        lw.WriteLine("WARN could not generate DXF sheets: %s" % exc)
        return 0
    imported = 0
    for dxf in sorted(dxfs):
        try:
            imp = _SESSION.DexManager.CreateDxfdwgImporter()
            imp.InputFile = dxf
            imp.OutputFile = work.FullPath
            imp.FileOpenFlag = False
            try:
                imp.ProcessHoldflag = True
            except Exception:
                pass
            imp.Commit()
            imp.Destroy()
            imported += 1
            lw.WriteLine("OK   imported %s" % os.path.basename(dxf))
        except Exception as exc:
            lw.WriteLine("WARN import %s failed: %s" % (os.path.basename(dxf), exc))
    return imported


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def main():
    lw = _lw()
    lw.WriteLine("=== motor_nx NX Drafting journal ===")

    prt = None
    size = "A3"
    do_dxf = False
    json_path = None
    for a in sys.argv[1:]:
        al = a.lower()
        if al in ("a4", "a3", "a2", "a1", "a0"):
            size = al.upper()
        elif al == "dxf":
            do_dxf = True
        elif al.endswith(".json"):
            json_path = a
        elif al.endswith(".prt"):
            prt = a

    # open the model part if one was given and it is not already the work part
    try:
        if prt and os.path.exists(prt):
            _SESSION.Parts.OpenDisplay(prt) if hasattr(_SESSION.Parts, "OpenDisplay") else \
                _SESSION.Parts.Open(prt)
            lw.WriteLine("OK   opened %s" % prt)
    except Exception as exc:
        lw.WriteLine("WARN could not open %s: %s (using current work part)" % (prt, exc))

    work = _SESSION.Parts.Work
    if work is None or work.Tag == 0:
        lw.WriteLine("FAIL no work part -- pass a built .prt as the first -args value.")
        return

    try:
        p, mfg, em_design = _load_design(json_path)
    except Exception:
        lw.WriteLine("FAIL design module import:\n" + traceback.format_exc())
        return

    scale = 0.5 if size in ("A3", "A4") else 0.4
    enter_drafting(work, lw)
    add_sheet(work, lw, size=size, scale=scale, name="SH1")
    add_base_views(work, lw, scale=scale)

    # right-hand title block + schedules down the right margin of the sheet
    w_mm, h_mm = SHEET_SIZE.get(size, SHEET_SIZE["A3"])
    place_note(work, lw, _title_rows(p, em_design, "1 ASSY", "1:%d" % round(1.0 / scale)),
               w_mm - 170, 70, height=3.2, tag="title")
    place_note(work, lw, bom_lines(p, mfg), w_mm - 170, 120, height=2.8, tag="bom")
    place_note(work, lw, gdt_lines(p, mfg), 12, h_mm - 12, height=2.6, tag="gdt")
    place_note(work, lw, notes_lines(mfg), w_mm - 170, h_mm - 12, height=2.6, tag="notes")

    if do_dxf:
        lw.WriteLine("--- importing dimensioned drawings.py DXF sheets ---")
        import_drawings_dxf(work, lw, p)

    # save + try a PDF of the sheet
    try:
        work.Save(NXOpen.BasePart.SaveComponents.TrueValue, NXOpen.BasePart.CloseAfterSave.FalseValue)
        lw.WriteLine("OK   saved part")
    except Exception as exc:
        lw.WriteLine("WARN save failed: %s" % exc)
    try:
        base = os.path.splitext(work.FullPath)[0]
        pb = _SESSION.PrintPDFBuilder if hasattr(_SESSION, "PrintPDFBuilder") else None
        printer = _SESSION.PlotManager.CreatePrintPDFBuilder()
        printer.OutputFile = base + "_drawing.pdf"
        printer.Scale = 1.0
        printer.Append(list(work.DrawingSheets) if hasattr(work, "DrawingSheets") else [])
        printer.Commit()
        printer.Destroy()
        lw.WriteLine("OK   PDF -> %s_drawing.pdf" % base)
    except Exception as exc:
        lw.WriteLine("NOTE PDF export skipped (%s) -- File>Plot in NX to export the sheet." % exc)

    lw.WriteLine("=== drafting journal done ===")


if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    main()
