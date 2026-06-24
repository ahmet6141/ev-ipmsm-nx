"""Tests for the console + HTML renderers (report.py)."""

import re

from nx_inspect.findings import Body, Finding, Report
from nx_inspect.report import render_console, render_html, write_html

ANSI_RE = re.compile(r"\033\[[0-9;]*m")


def _report_with_findings():
    bodies = [
        Body(id=0, name="HOUSING", volume_mm3=1000.0, area_mm2=500.0, mass_kg=0.5,
             centroid=[0.0, 0.0, 0.0], bbox=[-1, -1, -1, 1, 1, 1]),
        Body(id=1, name="(unnamed)", volume_mm3=5.0, area_mm2=10.0, mass_kg=0.0,
             centroid=[2.0, 2.0, 2.0], bbox=[1, 1, 1, 3, 3, 3]),
    ]
    findings = [
        Finding(check="interference", severity="error",
                title="Bodies interpenetrate (HOUSING & SHAFT)",
                detail="Estimated overlap ~412 mm^3.",
                bodies=["HOUSING", "SHAFT"], location=[1.0, 2.0, 3.0],
                metric={"overlap_volume_mm3": 412.0, "sample_points": 37},
                suggestion="Unite the two bodies."),
        Finding(check="tiny_body", severity="warning", title="Sliver body ((unnamed))",
                detail="Volume 5 mm^3 below threshold.", bodies=["(unnamed)"],
                location=[2.0, 2.0, 2.0], metric={"volume_mm3": 5.0},
                suggestion="Confirm it is intended."),
        Finding(check="unnamed_body", severity="info", title="1 unnamed body",
                detail="Unnamed bodies hurt selection.", bodies=["(unnamed)"]),
    ]
    return Report(part="widget", path=r"C:\work\widget.prt", units="mm",
                  is_assembly=False, config={}, bodies=bodies, findings=findings)


# --------------------------------------------------------------------------- #
# console
# --------------------------------------------------------------------------- #
def test_console_clean_sample(sample_report):
    out = render_console(sample_report, color=False)
    assert "suspension_out" in out
    assert "56 bodies" in out
    assert "CLEAN" in out
    assert "INFO (1)" in out
    assert "unnamed" in out.lower()


def test_console_groups_by_severity_in_order():
    out = render_console(_report_with_findings(), color=False)
    i_err = out.index("ERROR (1)")
    i_warn = out.index("WARNING (1)")
    i_info = out.index("INFO (1)")
    assert i_err < i_warn < i_info
    assert "1 ERROR finding(s)" in out


def test_console_shows_finding_details():
    out = render_console(_report_with_findings(), color=False)
    assert "Bodies interpenetrate" in out
    assert "HOUSING, SHAFT" in out
    assert "(1, 2, 3)" in out  # location formatted
    assert "Unite the two bodies." in out
    assert "overlap_volume_mm3=412" in out


def test_console_no_color_has_no_ansi():
    out = render_console(_report_with_findings(), color=False)
    assert ANSI_RE.search(out) is None


def test_console_color_has_ansi():
    out = render_console(_report_with_findings(), color=True)
    assert ANSI_RE.search(out) is not None
    # stripping ANSI gives the same content as the no-color render
    assert ANSI_RE.sub("", out) == render_console(_report_with_findings(), color=False)


def test_console_is_deterministic():
    r = _report_with_findings()
    assert render_console(r, color=False) == render_console(r, color=False)


# --------------------------------------------------------------------------- #
# HTML
# --------------------------------------------------------------------------- #
def test_html_is_self_contained_document():
    doc = render_html(_report_with_findings())
    assert doc.startswith("<!doctype html>")
    assert "</html>" in doc
    assert "<style>" in doc  # inline CSS
    assert "http://" not in doc and "https://" not in doc  # no external assets/links


def test_html_has_badges_and_severities():
    doc = render_html(_report_with_findings())
    assert 'class="badge error"' in doc
    assert 'class="badge warning"' in doc
    assert 'class="badge info"' in doc
    assert 'class="sev error"' in doc
    assert "ERROR" in doc and "WARNING" in doc and "INFO" in doc


def test_html_has_body_rows():
    doc = render_html(_report_with_findings())
    assert "HOUSING" in doc
    assert "Bodies (2)" in doc  # collapsible body table header
    assert "Bounding box" in doc


def test_html_escapes_content():
    bad = Report(part="<x>", path="p&q", units="mm", is_assembly=False, config={},
                 bodies=[], findings=[Finding(check="c", severity="info",
                                              title="a<b>&c", detail="")])
    doc = render_html(bad)
    assert "<x>" not in doc
    assert "&lt;x&gt;" in doc
    assert "a&lt;b&gt;&amp;c" in doc


def test_html_clean_report_says_clean(sample_report):
    doc = render_html(sample_report)
    assert "CLEAN" in doc
    assert "suspension_out" in doc


def test_html_is_deterministic():
    r = _report_with_findings()
    assert render_html(r) == render_html(r)


def test_write_html_creates_file(tmp_path):
    out = tmp_path / "r.html"
    returned = write_html(_report_with_findings(), str(out))
    assert returned == str(out)
    text = out.read_text(encoding="utf-8")
    assert text.startswith("<!doctype html>")
    assert "widget" in text


def test_html_body_missing_numeric_has_sortable_sentinel():
    """Unmeasured (None) numeric cells must carry a numeric data-sort sentinel so
    click-to-sort stays in the numeric branch (not the lexical NaN fallback)."""
    r = Report(part="w", path="/w.prt", units="mm", is_assembly=False, config={},
               bodies=[Body(id=0, name="MEASURED", volume_mm3=10.0,
                            area_mm2=20.0, mass_kg=0.1),
                       Body(id=1, name="UNMEASURED")],  # all numerics None
               findings=[])
    doc = render_html(r)
    # the measured row keeps its real numeric data-sort
    assert 'data-sort="10.0"' in doc
    # the unmeasured row gets a finite numeric sentinel (no empty data-sort, which
    # would make parseFloat NaN and fall back to lexical compare)
    assert 'data-sort=""' not in doc
    assert "1e+300" in doc  # the missing-value sentinel
    # the human-visible cell is still a dash, not the sentinel number
    assert "1e+300" not in doc.replace('data-sort="1e+300"', "")


def test_html_assembly_shows_components():
    from nx_inspect.findings import Component
    r = Report(part="asm", path="/asm.prt", units="mm", is_assembly=True, config={},
               bodies=[], findings=[],
               components=[Component(name="C1", part="c1.prt", origin=[1.0, 2.0, 3.0])])
    doc = render_html(r)
    assert "Components (1)" in doc
    assert "C1" in doc
    assert 'class="badge ok"' in doc  # components badge
