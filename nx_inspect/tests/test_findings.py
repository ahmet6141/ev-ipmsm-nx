"""Tests for the report model + loader (findings.py)."""

import copy

import pytest

from nx_inspect.findings import (
    Body,
    Finding,
    Report,
    SchemaError,
    SEVERITY_ORDER,
    count_by_severity,
    group_by_check,
    load_report,
    load_report_file,
    severity_rank,
    sorted_findings,
)


# --------------------------------------------------------------------------- #
# loading the real sample
# --------------------------------------------------------------------------- #
def test_load_sample(sample_report):
    r = sample_report
    assert isinstance(r, Report)
    assert r.part == "suspension_out"
    assert r.schema == "1"
    assert r.is_assembly is False
    assert r.n_bodies == 56
    assert len(r.bodies) == 56
    assert isinstance(r.bodies[0], Body)
    assert r.bodies[0].name == "KNUCKLE_R_000"


def test_sample_counts_and_clean(sample_report):
    counts = sample_report.counts()
    assert counts == {"error": 0, "warning": 0, "info": 1}
    assert sample_report.n_errors == 0
    assert sample_report.n_warnings == 0
    assert sample_report.n_info == 1
    assert sample_report.is_clean is True


def test_sample_finding_fields(sample_report):
    f = sample_report.findings[0]
    assert isinstance(f, Finding)
    assert f.check == "unnamed_body"
    assert f.severity == "info"
    assert f.bodies == ["(unnamed)", "(unnamed)", "(unnamed)"]
    assert f.location is None
    assert f.suggestion  # non-empty


def test_load_report_file_equals_load_report(sample_path, sample_data):
    a = load_report_file(sample_path)
    b = load_report(sample_data)
    assert a == b


# --------------------------------------------------------------------------- #
# severity ordering / helpers
# --------------------------------------------------------------------------- #
def test_severity_order_error_first():
    assert SEVERITY_ORDER["error"] < SEVERITY_ORDER["warning"] < SEVERITY_ORDER["info"]
    assert severity_rank("error") == 0
    assert severity_rank("nonsense") > severity_rank("info")


def _mk(check, sev, title="t"):
    return Finding(check=check, severity=sev, title=title)


def test_sorted_findings_is_severity_then_check_then_title():
    fs = [
        _mk("unnamed_body", "info", "z"),
        _mk("interference", "error", "b"),
        _mk("interference", "error", "a"),
        _mk("tiny_body", "warning", "m"),
    ]
    out = sorted_findings(fs)
    assert [f.severity for f in out] == ["error", "error", "warning", "info"]
    assert [f.title for f in out[:2]] == ["a", "b"]  # tie broken by title


def test_sorted_findings_is_stable_and_deterministic():
    fs = [_mk("a", "info", "x"), _mk("a", "info", "x"), _mk("b", "info", "y")]
    assert sorted_findings(fs) == sorted_findings(list(fs))


def test_count_by_severity():
    fs = [_mk("a", "error"), _mk("b", "error"), _mk("c", "warning")]
    assert count_by_severity(fs) == {"error": 2, "warning": 1, "info": 0}


def test_group_by_check():
    fs = [_mk("interference", "error"), _mk("interference", "error"), _mk("tiny_body", "warning")]
    groups = group_by_check(fs)
    assert set(groups) == {"interference", "tiny_body"}
    assert len(groups["interference"]) == 2


def test_report_grouping_helpers(sample_report):
    assert sample_report.sorted_findings() == sorted_findings(sample_report.findings)
    by_check = sample_report.findings_by_check()
    assert set(by_check) == {"unnamed_body"}


# --------------------------------------------------------------------------- #
# schema validation: malformed reports must raise
# --------------------------------------------------------------------------- #
def test_top_level_must_be_object():
    with pytest.raises(SchemaError):
        load_report([1, 2, 3])


def test_missing_required_key_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    del bad["bodies"]
    with pytest.raises(SchemaError):
        load_report(bad)


def test_wrong_schema_version_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["schema"] = "2"
    with pytest.raises(SchemaError):
        load_report(bad)


def test_bad_severity_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["findings"][0]["severity"] = "critical"
    with pytest.raises(SchemaError):
        load_report(bad)


def test_bad_body_field_type_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["bodies"][0]["volume_mm3"] = "not a number"
    with pytest.raises(SchemaError):
        load_report(bad)


def test_bad_centroid_length_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["bodies"][0]["centroid"] = [1.0, 2.0]  # must be length 3
    with pytest.raises(SchemaError):
        load_report(bad)


def test_summary_disagreement_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["summary"]["info"] = 99  # disagrees with the single info finding
    with pytest.raises(SchemaError):
        load_report(bad)


def test_summary_n_bodies_disagreement_raises(sample_data):
    bad = copy.deepcopy(sample_data)
    bad["summary"]["n_bodies"] = 1
    with pytest.raises(SchemaError):
        load_report(bad)


def test_missing_file_raises_schema_error(tmp_path):
    with pytest.raises(SchemaError):
        load_report_file(str(tmp_path / "nope.json"))


def test_malformed_json_raises_schema_error(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{ not valid json", encoding="utf-8")
    with pytest.raises(SchemaError):
        load_report_file(str(p))


def test_optional_fields_default(sample_data):
    """A minimal valid report (no summary/components) still loads."""
    minimal = {
        "schema": "1",
        "part": "x",
        "path": "/x.prt",
        "config": {},
        "bodies": [],
        "findings": [],
    }
    r = load_report(minimal)
    assert r.units == "mm"
    assert r.is_assembly is False
    assert r.components == []
    assert r.n_bodies == 0
