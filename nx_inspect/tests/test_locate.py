"""Tests for run_journal.exe discovery (locate.py).

These never touch a real NX install: they fabricate a fake NXBIN tree in a
tmp dir and steer discovery via monkeypatched environment variables.
"""

import os

import pytest

from nx_inspect import locate
from nx_inspect.locate import (
    RunJournalNotFound,
    candidate_paths,
    find_run_journal,
)


def _make_fake_nx(tmp_path):
    """Create <base>/NXBIN/run_journal.exe and return (base, exe_path)."""
    nxbin = tmp_path / "NXBIN"
    nxbin.mkdir()
    exe = nxbin / "run_journal.exe"
    exe.write_text("stub", encoding="utf-8")
    return str(tmp_path), str(exe)


@pytest.fixture(autouse=True)
def _clear_nx_env(monkeypatch):
    """Start every test with a clean NX environment."""
    monkeypatch.delenv("UGII_BASE_DIR", raising=False)
    monkeypatch.delenv("UGII_ROOT_DIR", raising=False)
    # Neutralize registry + common-dir discovery so tests are hermetic.
    monkeypatch.setattr(locate, "_registry_bases", lambda: [])
    monkeypatch.setattr(locate, "_common_install_bases", lambda: [])


def test_ugii_base_dir_resolves(tmp_path, monkeypatch):
    base, exe = _make_fake_nx(tmp_path)
    monkeypatch.setenv("UGII_BASE_DIR", base)
    assert os.path.samefile(find_run_journal(), exe)


def test_ugii_root_dir_pointing_at_nxbin_resolves(tmp_path, monkeypatch):
    base, exe = _make_fake_nx(tmp_path)
    # Some installs set UGII_ROOT_DIR to the NXBIN dir itself.
    monkeypatch.setenv("UGII_ROOT_DIR", os.path.join(base, "NXBIN"))
    assert os.path.samefile(find_run_journal(), exe)


def test_override_exe_path_wins(tmp_path, monkeypatch):
    base, exe = _make_fake_nx(tmp_path)
    # No env set; explicit override should still resolve.
    assert os.path.samefile(find_run_journal(override=exe), exe)


def test_override_base_dir_resolves(tmp_path):
    base, exe = _make_fake_nx(tmp_path)
    assert os.path.samefile(find_run_journal(override=base), exe)


def test_not_found_lists_searched_paths(tmp_path, monkeypatch):
    monkeypatch.setenv("UGII_BASE_DIR", str(tmp_path / "does_not_exist"))
    with pytest.raises(RunJournalNotFound) as exc:
        find_run_journal()
    msg = str(exc.value)
    assert "run_journal.exe" in msg
    assert "Looked in" in msg
    assert "does_not_exist" in msg


def test_candidate_paths_are_unique_and_ordered(tmp_path, monkeypatch):
    base, _ = _make_fake_nx(tmp_path)
    monkeypatch.setenv("UGII_BASE_DIR", base)
    monkeypatch.setenv("UGII_ROOT_DIR", base)  # duplicate source
    cands = candidate_paths()
    # No duplicates after normalization.
    norm = [os.path.normcase(os.path.normpath(c)) for c in cands]
    assert len(norm) == len(set(norm))
    # The env-derived candidate is present.
    assert any(c.endswith(os.path.join("NXBIN", "run_journal.exe")) for c in cands)


def test_override_listed_first(tmp_path, monkeypatch):
    base, exe = _make_fake_nx(tmp_path)
    monkeypatch.setenv("UGII_BASE_DIR", base)
    cands = candidate_paths(override=exe)
    assert os.path.normpath(cands[0]) == os.path.normpath(exe)
