"""Locate ``run_journal.exe`` generically across machines.

``run_journal`` is the headless NX journal runner.  It lives at
``<NX base>/NXBIN/run_journal.exe``.  We never hardcode an install path; instead
we search, in priority order:

1. An explicit override (``--run-journal`` / ``find_run_journal(override=...)``).
2. The ``UGII_BASE_DIR`` and ``UGII_ROOT_DIR`` environment variables (NX sets
   these when an NX shell is active).  ``run_journal`` sits in ``NXBIN`` under
   the base; ``UGII_ROOT_DIR`` itself usually *is* ``NXBIN`` or the base.
3. The Windows registry: ``HKLM\\SOFTWARE\\Siemens\\NX\\<version>`` keys, reading
   ``INSTALLDIR`` / ``UGII_BASE_DIR``.
4. Common install roots: ``C:\\Program Files\\Siemens\\*``, ``E:\\program``, etc.

If nothing is found, :func:`find_run_journal` raises :class:`RunJournalNotFound`
whose message lists every place that was searched, so the failure is actionable.

This module is import-safe on any OS (the registry step is guarded), so the
test-suite can exercise the env-var path on non-Windows CI too.
"""

import os
import sys
from typing import List, Optional

EXE_NAME = "run_journal.exe"


class RunJournalNotFound(FileNotFoundError):
    """Raised when ``run_journal.exe`` cannot be located anywhere we looked."""


# --------------------------------------------------------------------------- #
# candidate generation
# --------------------------------------------------------------------------- #
def _exe_candidates_for_base(base: str) -> List[str]:
    """Possible run_journal locations given an NX *base* (or NXBIN) directory."""
    base = base.strip().strip('"')
    if not base:
        return []
    return [
        os.path.join(base, "NXBIN", EXE_NAME),  # base dir -> NXBIN/run_journal.exe
        os.path.join(base, EXE_NAME),           # base already *is* NXBIN
    ]


def _env_candidates() -> List[str]:
    out: List[str] = []
    for var in ("UGII_BASE_DIR", "UGII_ROOT_DIR"):
        val = os.environ.get(var)
        if val:
            out.extend(_exe_candidates_for_base(val))
    return out


def _registry_bases() -> List[str]:
    """NX install bases discovered in the Windows registry (empty off Windows)."""
    if not sys.platform.startswith("win"):
        return []
    try:
        import winreg  # noqa: WPS433 (local import: Windows-only, optional)
    except Exception:  # pragma: no cover - non-Windows
        return []

    bases: List[str] = []
    roots = [
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Siemens\NX"),
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\Siemens\NX"),
        (winreg.HKEY_CURRENT_USER, r"SOFTWARE\Siemens\NX"),
    ]
    value_names = ("INSTALLDIR", "UGII_BASE_DIR", "InstallDir", "Path")
    for hive, subkey in roots:
        try:
            with winreg.OpenKey(hive, subkey) as key:
                _read_version_subkeys(winreg, key, value_names, bases)
        except OSError:
            continue
    return bases


def _read_version_subkeys(winreg, key, value_names, bases: List[str]) -> None:
    """Enumerate ``...\\NX\\<version>`` subkeys and harvest install-dir values."""
    i = 0
    while True:
        try:
            ver = winreg.EnumKey(key, i)
        except OSError:
            break
        i += 1
        try:
            with winreg.OpenKey(key, ver) as vkey:
                for name in value_names:
                    try:
                        val, _ = winreg.QueryValueEx(vkey, name)
                    except OSError:
                        continue
                    if val and isinstance(val, str):
                        bases.append(val)
        except OSError:
            continue


def _common_install_bases() -> List[str]:
    """Best-effort list of conventional NX install roots to scan."""
    bases: List[str] = []
    program_dirs = []
    for env in ("ProgramFiles", "ProgramFiles(x86)", "ProgramW6432"):
        p = os.environ.get(env)
        if p:
            program_dirs.append(p)
    program_dirs += [r"C:\Program Files", r"C:\Program Files (x86)"]

    for pd in program_dirs:
        siemens = os.path.join(pd, "Siemens")
        # Scan Siemens/NX*  and  Siemens/<anything>/NXBIN
        for child in _safe_listdir(siemens):
            bases.append(os.path.join(siemens, child))

    # Drive-letter conventions seen in the wild (incl. this machine's E:\program).
    bases += [
        r"E:\program",
        r"E:\Program Files\Siemens\NX",
        r"D:\Siemens\NX",
        r"C:\Siemens\NX",
    ]
    return bases


def _safe_listdir(path: str) -> List[str]:
    try:
        return sorted(os.listdir(path))
    except OSError:
        return []


# --------------------------------------------------------------------------- #
# public API
# --------------------------------------------------------------------------- #
def candidate_paths(override: Optional[str] = None) -> List[str]:
    """Ordered, de-duplicated list of every path we would check for run_journal."""
    cands: List[str] = []
    if override:
        ov = os.path.abspath(os.path.expanduser(override.strip().strip('"')))
        # An override may point at the exe directly or at a base/NXBIN dir.
        if ov.lower().endswith(".exe"):
            cands.append(ov)
        else:
            cands.extend(_exe_candidates_for_base(ov))

    cands.extend(_env_candidates())
    for base in _registry_bases():
        cands.extend(_exe_candidates_for_base(base))
    for base in _common_install_bases():
        cands.extend(_exe_candidates_for_base(base))

    seen = set()
    ordered = []
    for c in cands:
        norm = os.path.normcase(os.path.normpath(c))
        if norm not in seen:
            seen.add(norm)
            ordered.append(c)
    return ordered


def find_run_journal(override: Optional[str] = None) -> str:
    """Return an existing ``run_journal.exe`` path, or raise :class:`RunJournalNotFound`.

    Parameters
    ----------
    override:
        An explicit path to either ``run_journal.exe`` or an NX base/NXBIN
        directory.  Tried first.
    """
    checked = candidate_paths(override)
    for path in checked:
        if os.path.isfile(path):
            return path

    where = "\n  ".join(checked) if checked else "(no candidate locations)"
    hint = (
        "Could not find %s.\n"
        "Set UGII_BASE_DIR or UGII_ROOT_DIR to your NX install, or pass "
        "--run-journal <path>.\nLooked in:\n  %s" % (EXE_NAME, where)
    )
    raise RunJournalNotFound(hint)
