"""Shared pytest fixtures: paths to the real sample report and a loaded Report."""

import json
import os
import sys

import pytest

# Make the package importable when running the tests from a source checkout
# without installing (so `python -m pytest nx_inspect/tests` just works).
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from nx_inspect.findings import load_report_file  # noqa: E402

SAMPLE_PATH = os.path.join(_PKG_ROOT, "examples", "suspension_report.json")


@pytest.fixture
def sample_path():
    """Absolute path to the committed real sample report."""
    assert os.path.isfile(SAMPLE_PATH), "sample report missing: %s" % SAMPLE_PATH
    return SAMPLE_PATH


@pytest.fixture
def sample_data(sample_path):
    """The sample report parsed as a raw dict (for mutation in schema tests)."""
    with open(sample_path, "r", encoding="utf-8") as fh:
        return json.load(fh)


@pytest.fixture
def sample_report(sample_path):
    """The sample report loaded and validated into a Report object."""
    return load_report_file(sample_path)
