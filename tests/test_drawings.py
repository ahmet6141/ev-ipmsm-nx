"""NX-independent tests for the 2D manufacturing drawing generator."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_nx import drawings  # noqa: E402
from motor_nx.params import MotorParams  # noqa: E402


def test_five_sheets():
    sheets = drawings.all_sheets(MotorParams())
    assert set(sheets) == {"1_assembly", "2_stator", "3_rotor", "4_shaft", "5_exploded"}
    for d in sheets.values():
        assert d.ents, "sheet should have entities"


def test_dxf_well_formed():
    d = drawings.assembly_sheet(MotorParams())
    dxf = d.to_dxf()
    assert dxf.startswith("0\nSECTION")
    assert "ENTITIES" in dxf and dxf.rstrip().endswith("EOF")
    # the workhorse entity types must all appear
    for tok in ("\nLINE\n", "\nCIRCLE\n", "\nTEXT\n", "\nSOLID\n", "\nLWPOLYLINE\n"):
        assert tok in dxf, tok
    # a LAYER table is declared
    assert "TABLE\n2\nLAYER" in dxf


def test_svg_well_formed():
    sheets = drawings.all_sheets(MotorParams())
    for key, d in sheets.items():
        svg = d.to_svg()
        assert svg.startswith("<svg") and svg.rstrip().endswith("</svg>")
        assert "<text" in svg
        # the radial cross-section sheets carry circles; shaft + exploded are side views
        if key in ("1_assembly", "2_stator", "3_rotor"):
            assert "<circle" in svg, key


def test_dimensions_and_arrowheads_present():
    d = drawings.assembly_sheet(MotorParams())
    kinds = [e[0] for e in d.ents]
    assert kinds.count("solid") >= 8         # arrowheads (>=4 dims x 2)
    assert kinds.count("text") > 10          # dims + title block + BOM
    assert "line" in kinds and "circle" in kinds


def test_bounds_finite():
    for d in drawings.all_sheets(MotorParams()).values():
        x0, y0, x1, y1 = d.bounds()
        assert all(math.isfinite(v) for v in (x0, y0, x1, y1))
        assert x1 > x0 and y1 > y0


def test_arrowhead_is_triangle_at_tip():
    d = drawings.Drawing("t")
    d._arrowhead((10.0, 0.0), (0.0, 0.0))
    solid = next(e for e in d.ents if e[0] == "solid")
    pts = solid[1]
    assert len(pts) == 3
    # one vertex sits at the tip
    assert any(abs(px - 10.0) < 1e-6 and abs(py) < 1e-6 for px, py in pts)


def test_write_drawings(tmp_path=None):
    import tempfile
    out = tempfile.mkdtemp()
    written = drawings.write_drawings(MotorParams(), out, "2026-01-01")
    assert len(written) == 10           # 5 sheets x (dxf + svg)
    assert all(os.path.getsize(p) > 0 for p in written)
    assert sum(p.endswith(".dxf") for p in written) == 5
    assert sum(p.endswith(".svg") for p in written) == 5


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print("PASS ", fn.__name__)
        except Exception:
            failed += 1
            print("FAIL ", fn.__name__)
            traceback.print_exc()
    print("\n%d/%d passed" % (len(fns) - failed, len(fns)))
    sys.exit(1 if failed else 0)
