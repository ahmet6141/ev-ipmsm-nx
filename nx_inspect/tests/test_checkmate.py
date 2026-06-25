"""The CLI/report layer must accept + render native Check-Mate findings (check="checkmate").

Uses examples/suspension_checkmate_report.json -- a REAL report from the integrated journal
run, which carries a Check-Mate 'Faces - Spikes/Cuts' error alongside the body-name infos.
This guards that the schema does not reject the native-checker findings and that they
render through the console + HTML like any other finding.
"""
import os

from nx_inspect.findings import load_report_file
from nx_inspect.report import render_console, render_html

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKMATE_SAMPLE = os.path.join(_PKG_ROOT, "examples", "suspension_checkmate_report.json")


def test_checkmate_sample_loads_with_native_finding():
    r = load_report_file(CHECKMATE_SAMPLE)
    cm = [f for f in r.findings if f.check == "checkmate"]
    assert cm, "expected at least one native Check-Mate finding"
    assert any(f.severity == "error" for f in cm)
    # the native finding carries the checker name + the flagged object count
    f = cm[0]
    assert "Check-Mate" in f.title
    assert "status" in f.metric and "object_count" in f.metric
    assert r.n_errors >= 1
    assert r.is_clean is False


def test_checkmate_finding_renders_console_and_html():
    r = load_report_file(CHECKMATE_SAMPLE)
    out = render_console(r, color=False)
    assert "Check-Mate" in out
    assert "ERROR" in out          # the native finding is grouped under ERROR
    doc = render_html(r)
    assert "Check-Mate" in doc
    assert 'class="sev error"' in doc
