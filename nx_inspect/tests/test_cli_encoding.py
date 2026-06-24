"""Regression tests for the two HIGH/MEDIUM review findings:

1. The console renderer must not crash with ``UnicodeEncodeError`` when the
   journal's non-ASCII glyphs (U+2229 INTERSECTION in interference titles,
   U+2014 EM DASH in details + the verdict line) are written to a *real*
   legacy-code-page console.  ``capsys`` cannot catch this because it captures
   into an encoding-less ``StringIO``, so we drive the CLI through a genuine
   ``io.TextIOWrapper`` bound to cp1252 / cp1254, exactly like an interactive
   Windows console.

2. The CI exit-code contract: error-finding counts are clamped to 63 so they can
   never collide with the operational codes (>=64), and the operational codes
   themselves stay distinct.
"""

import io
import json

import pytest

from nx_inspect import cli
from nx_inspect.cli import MAX_ERROR_EXIT, _exit_for_errors, main


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _write_report(tmp_path, name, findings, bodies=None):
    report = {
        "schema": "1", "tool": "nx_inspect", "part": "p", "path": "/p.prt",
        "config": {}, "bodies": bodies or [], "findings": findings,
    }
    p = tmp_path / name
    p.write_text(json.dumps(report), encoding="utf-8")
    return str(p)


def _interference_report(tmp_path):
    """A report whose ONLY finding is an interference (the U+2229 glyph path)."""
    return _write_report(tmp_path, "interf.json", [
        {"check": "interference", "severity": "error",
         "title": "Bodies interpenetrate (A ∩ B)",
         "detail": "Estimated overlap volume ~412 mm^3 — review.",
         "bodies": ["A", "B"], "location": [1.0, 2.0, 3.0],
         "metric": {"overlap_volume_mm3": 412.0, "sample_points": 37},
         "suggestion": "Unite the two bodies."},
    ])


class _RealConsole:
    """A context manager that swaps sys.stdout/stderr for genuine TextIOWrappers
    bound to a legacy single-byte code page (like an interactive Windows shell).
    Unlike capsys' StringIO, these raise UnicodeEncodeError on un-encodable
    glyphs unless the CLI made them tolerant."""

    def __init__(self, monkeypatch, encoding):
        self._mp = monkeypatch
        self._encoding = encoding
        self.out_buf = io.BytesIO()
        self.err_buf = io.BytesIO()

    def __enter__(self):
        # strict errors == a faithful interactive console: encoding failures raise
        self.out = io.TextIOWrapper(self.out_buf, encoding=self._encoding,
                                    errors="strict", newline="")
        self.err = io.TextIOWrapper(self.err_buf, encoding=self._encoding,
                                    errors="strict", newline="")
        self._mp.setattr("sys.stdout", self.out)
        self._mp.setattr("sys.stderr", self.err)
        return self

    def __exit__(self, *exc):
        try:
            self.out.flush()
            self.err.flush()
        except Exception:
            pass
        return False

    def stdout_text(self):
        self.out.flush()
        return self.out_buf.getvalue().decode(self._encoding, "replace")


# --------------------------------------------------------------------------- #
# 1. Unicode console safety (the flagship interference path)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("encoding", ["cp1252", "cp1254"])
def test_interference_renders_on_legacy_codepage(tmp_path, monkeypatch, encoding):
    """Rendering an interference finding to a cp1252/cp1254 console must NOT
    raise, and must exit with the error count (1), not Python's crash code."""
    path = _interference_report(tmp_path)
    # sanity: these code pages genuinely cannot encode U+2229 strictly
    with pytest.raises(UnicodeEncodeError):
        "∩".encode(encoding)

    with _RealConsole(monkeypatch, encoding) as console:
        code = main(["--from-json", path, "--no-color"])
        text = console.stdout_text()

    assert code == 1  # one error finding, exit == error count (NOT a crash)
    assert "Bodies interpenetrate" in text
    # the un-encodable glyph was replaced losslessly, not dropped into a crash
    assert "ERROR" in text


def test_interference_html_survives_console_encoding(tmp_path, monkeypatch):
    """--html artifact is written even though the console path handles a glyph
    that a legacy code page can't encode (HTML is written before the console)."""
    path = _interference_report(tmp_path)
    html_path = tmp_path / "out.html"
    with _RealConsole(monkeypatch, "cp1254"):
        code = main(["--from-json", path, "--html", str(html_path), "--no-color"])
    assert code == 1
    assert html_path.is_file()
    doc = html_path.read_text(encoding="utf-8")
    assert doc.startswith("<!doctype html>")
    assert "∩" in doc  # the real glyph is preserved in the UTF-8 HTML


def test_em_dash_detail_on_legacy_codepage(tmp_path, monkeypatch):
    """A non-clean verdict line ('N ERROR finding(s) — see above', U+2014) and a
    detail string carrying an em dash must render on a legacy console too."""
    path = _write_report(tmp_path, "emdash.json", [
        {"check": "zero_volume", "severity": "error",
         "title": "Body has zero/negative volume (X)",
         "detail": "Measured volume = 0.0 mm^3 — a degenerate body."},
    ])
    # cp1254 *can* encode U+2014, so this exercises the verdict/detail em-dash
    # path without the harder U+2229 case.
    with _RealConsole(monkeypatch, "cp1254") as console:
        code = main(["--from-json", path, "--no-color"])
        text = console.stdout_text()
    assert code == 1
    assert "ERROR finding(s)" in text


def test_emit_fallback_on_unreconfigurable_stream(monkeypatch):
    """_emit must not raise even if a stream cannot be reconfigured and its codec
    cannot encode a glyph (belt-and-suspenders second line of defence)."""
    buf = io.BytesIO()
    # errors='strict' and we will NOT reconfigure it -> _emit must still survive
    stream = io.TextIOWrapper(buf, encoding="cp1252", errors="strict", newline="")
    cli._emit("interference A ∩ B — done", stream=stream)
    stream.flush()
    decoded = buf.getvalue().decode("cp1252", "replace")
    assert "interference A" in decoded
    assert "done" in decoded


# --------------------------------------------------------------------------- #
# 2. Exit-code contract (clamp + operational-code separation)
# --------------------------------------------------------------------------- #
def test_exit_for_errors_clamps():
    assert _exit_for_errors(0) == 0
    assert _exit_for_errors(1) == 1
    assert _exit_for_errors(63) == 63
    assert _exit_for_errors(64) == 63   # would collide with EXIT_USAGE otherwise
    assert _exit_for_errors(65) == 63
    assert _exit_for_errors(70) == 63
    assert _exit_for_errors(1000) == 63


def test_many_errors_clamped_below_operational(tmp_path, capsys):
    """A model with 64 error findings must NOT exit 64 (== EXIT_USAGE)."""
    findings = [{"check": "interference", "severity": "error", "title": "e%d" % i}
                for i in range(64)]
    path = _write_report(tmp_path, "many.json", findings)
    code = main(["--from-json", path, "--no-color"])
    assert code == MAX_ERROR_EXIT == 63
    assert code < cli.EXIT_USAGE  # never collides with an operational code


def test_operational_codes_are_distinct_and_reserved():
    codes = {cli.EXIT_USAGE, cli.EXIT_BAD_REPORT, cli.EXIT_NO_FILE,
             cli.EXIT_NO_NX, cli.EXIT_JOURNAL}
    assert len(codes) == 5  # all distinct
    assert all(c > MAX_ERROR_EXIT for c in codes)  # outside the clamped range


def test_single_error_exits_one(tmp_path, capsys):
    path = _write_report(tmp_path, "one.json", [
        {"check": "zero_volume", "severity": "error", "title": "z"}])
    assert main(["--from-json", path, "--no-color"]) == 1


# --------------------------------------------------------------------------- #
# 3. foreign-JSON rejection (tool field)
# --------------------------------------------------------------------------- #
def test_foreign_tool_rejected(tmp_path, capsys):
    from nx_inspect.cli import EXIT_BAD_REPORT
    report = {"schema": "1", "tool": "some_other_tool", "part": "p", "path": "/p",
              "config": {}, "bodies": [], "findings": []}
    p = tmp_path / "foreign.json"
    p.write_text(json.dumps(report), encoding="utf-8")
    code = main(["--from-json", str(p), "--no-color"])
    assert code == EXIT_BAD_REPORT
    err = capsys.readouterr().err.lower()
    assert "nx_inspect" in err or "tool" in err
