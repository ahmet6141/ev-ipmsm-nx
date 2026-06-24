"""Tests for the CLI (cli.py), exercised entirely offline via --from-json."""

import re

import pytest

from nx_inspect import cli
from nx_inspect.cli import EXIT_BAD_REPORT, EXIT_NO_FILE, main

ANSI_RE = re.compile(r"\033\[[0-9;]*m")


# --------------------------------------------------------------------------- #
# --from-json happy path
# --------------------------------------------------------------------------- #
def test_from_json_sample_exits_zero(sample_path, capsys):
    code = main(["--from-json", sample_path, "--no-color"])
    assert code == 0  # sample has 0 errors
    out = capsys.readouterr().out
    assert "suspension_out" in out
    assert "CLEAN" in out
    assert ANSI_RE.search(out) is None  # --no-color


def test_from_json_writes_html(sample_path, tmp_path, capsys):
    html_path = tmp_path / "r.html"
    code = main(["--from-json", sample_path, "--html", str(html_path), "--no-color"])
    assert code == 0
    assert html_path.is_file()
    doc = html_path.read_text(encoding="utf-8")
    assert doc.startswith("<!doctype html>")
    assert "suspension_out" in doc
    out = capsys.readouterr().out
    assert "HTML report written to" in out


def test_exit_code_equals_error_count(tmp_path, capsys):
    """A report with 2 error findings must exit 2."""
    import json
    report = {
        "schema": "1", "part": "p", "path": "/p.prt", "config": {},
        "bodies": [], "findings": [
            {"check": "interference", "severity": "error", "title": "a"},
            {"check": "zero_volume", "severity": "error", "title": "b"},
            {"check": "tiny_body", "severity": "warning", "title": "c"},
        ],
    }
    p = tmp_path / "two_err.json"
    p.write_text(json.dumps(report), encoding="utf-8")
    code = main(["--from-json", str(p), "--no-color"])
    assert code == 2


def test_quiet_suppresses_console_but_writes_html(sample_path, tmp_path, capsys):
    html_path = tmp_path / "r.html"
    code = main(["--from-json", sample_path, "--html", str(html_path),
                 "--quiet", "--no-color"])
    assert code == 0
    assert html_path.is_file()
    out = capsys.readouterr().out
    assert "suspension_out" not in out  # console summary suppressed
    assert "HTML report written to" in out  # but the write notice still prints


# --------------------------------------------------------------------------- #
# error handling
# --------------------------------------------------------------------------- #
def test_missing_json_file(tmp_path, capsys):
    code = main(["--from-json", str(tmp_path / "nope.json")])
    assert code == EXIT_NO_FILE
    err = capsys.readouterr().err
    assert "not found" in err


def test_malformed_report_file(tmp_path, capsys):
    p = tmp_path / "bad.json"
    p.write_text("{ broken", encoding="utf-8")
    code = main(["--from-json", str(p)])
    assert code == EXIT_BAD_REPORT
    err = capsys.readouterr().err
    assert "error" in err.lower()


def test_schema_invalid_report(tmp_path, capsys):
    import json
    p = tmp_path / "wrong.json"
    p.write_text(json.dumps({"schema": "1", "part": "x"}), encoding="utf-8")  # missing keys
    code = main(["--from-json", str(p)])
    assert code == EXIT_BAD_REPORT


# --------------------------------------------------------------------------- #
# argparse behavior
# --------------------------------------------------------------------------- #
def test_requires_part_or_from_json(capsys):
    with pytest.raises(SystemExit) as exc:
        main([])
    assert exc.value.code != 0
    err = capsys.readouterr().err
    assert "required" in err


def test_part_and_from_json_mutually_exclusive(sample_path, capsys):
    with pytest.raises(SystemExit) as exc:
        main(["some.prt", "--from-json", sample_path])
    assert exc.value.code != 0
    err = capsys.readouterr().err
    assert "not both" in err


def test_bad_check_name_rejected(sample_path, capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--from-json", sample_path, "--checks", "interference,bogus"])
    assert exc.value.code != 0
    err = capsys.readouterr().err
    assert "bogus" in err


def test_missing_part_file(tmp_path, capsys):
    code = main([str(tmp_path / "nope.prt")])
    assert code == EXIT_NO_FILE
    err = capsys.readouterr().err
    assert "not found" in err


def test_parser_version(capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--version"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "nx-inspect" in out


def test_build_parser_lists_checks():
    parser = cli._build_parser()
    help_text = parser.format_help()
    assert "interference" in help_text
    assert "duplicate_name" in help_text
