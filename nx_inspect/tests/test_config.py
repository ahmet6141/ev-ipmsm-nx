"""Tests for config defaults / overrides (config.py)."""

import pytest

from nx_inspect.config import (
    ALL_CHECKS,
    Config,
    normalize_checks,
)


def test_defaults_match_journal():
    c = Config()
    assert c.grid == 10
    assert c.tol_mm3 == 50.0
    assert c.tiny_mm3 == 30.0
    assert c.dup_mm == 1.0
    assert c.checks == ALL_CHECKS


def test_with_overrides_ignores_none():
    c = Config().with_overrides(grid=None, tiny_mm3=12.0)
    assert c.grid == 10        # untouched
    assert c.tiny_mm3 == 12.0  # applied


def test_with_overrides_is_immutable():
    base = Config()
    derived = base.with_overrides(grid=99)
    assert base.grid == 10     # original unchanged (frozen dataclass)
    assert derived.grid == 99


def test_normalize_checks_string():
    assert normalize_checks("tiny_body,interference") == ["interference", "tiny_body"]


def test_normalize_checks_dedups_and_orders():
    out = normalize_checks("tiny_body,tiny_body,interference")
    assert out == ["interference", "tiny_body"]


def test_normalize_checks_none_is_all():
    assert normalize_checks(None) == ALL_CHECKS
    assert normalize_checks("") == ALL_CHECKS


def test_normalize_checks_rejects_unknown():
    with pytest.raises(ValueError) as exc:
        normalize_checks("interference,nope")
    assert "nope" in str(exc.value)


def test_journal_args_basic():
    args = Config().journal_args()
    assert "grid=10" in args
    assert "tol=50" in args
    assert "tiny=30" in args
    assert "dup=1" in args
    # all checks selected -> no explicit checks= token
    assert not any(a.startswith("checks=") for a in args)


def test_journal_args_subset_emits_checks():
    c = Config().with_overrides(checks=["interference"])
    args = c.journal_args()
    assert "checks=interference" in args
