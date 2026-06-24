"""Defaults and thresholds for nx_inspect.

These mirror the ``DEFAULTS`` / ``ALL_CHECKS`` in the NX-side journal
(``inspect_journal.py``) so that what the CLI sends and what the journal
assumes never drift apart.  Everything here is overridable from the CLI.

Note on naming: the journal accepts the *short* argument keys ``tol`` and
``tiny`` on its command line, but writes the *long* keys ``tol_mm3`` and
``tiny_mm3`` into the report ``config`` block.  Both names appear below so the
two halves agree.
"""

from dataclasses import dataclass, field, replace
from typing import List

# ---- schema --------------------------------------------------------------- #
#: The report schema version this package understands.
SCHEMA = "1"

# ---- check catalog -------------------------------------------------------- #
#: Every check the journal can run, in the journal's own order.
ALL_CHECKS: List[str] = [
    "interference",
    "zero_volume",
    "tiny_body",
    "duplicate_body",
    "unnamed_body",
    "duplicate_name",
]

#: Human-readable, one-line description of each check (used by --help / README
#: generation and as a single source of truth for the catalog).
CHECK_DESCRIPTIONS = {
    "interference": "Solid bodies that interpenetrate (overlap in space).",
    "zero_volume": "A solid body with zero or negative volume (degenerate).",
    "tiny_body": "A sliver body below the tiny-volume threshold.",
    "duplicate_body": "Two bodies with the same volume and coincident centroid.",
    "unnamed_body": "A body with no display name.",
    "duplicate_name": "The same display name on multiple bodies.",
}

#: Default severity each check reports at (informational mirror of the journal).
CHECK_SEVERITY = {
    "interference": "error",
    "zero_volume": "error",
    "tiny_body": "warning",
    "duplicate_body": "warning",
    "unnamed_body": "info",
    "duplicate_name": "info",
}


# ---- thresholds ----------------------------------------------------------- #
@dataclass(frozen=True)
class Config:
    """Inspection thresholds, matching the journal DEFAULTS.

    Attributes
    ----------
    grid:
        Per-axis sample-grid resolution for the interference check (the journal
        samples ``grid**3`` points across each bbox-overlap region).
    tol_mm3:
        Minimum estimated overlap volume (mm^3) for an interference to be flagged.
    tiny_mm3:
        Bodies with ``0 < volume < tiny_mm3`` are flagged as slivers.
    dup_mm:
        Centroid coincidence tolerance (mm) for the duplicate-body check.
    checks:
        Which checks to run.  Empty/None means "all".
    """

    grid: int = 10
    tol_mm3: float = 50.0
    tiny_mm3: float = 30.0
    dup_mm: float = 1.0
    checks: List[str] = field(default_factory=lambda: list(ALL_CHECKS))

    # -- construction helpers ---------------------------------------------- #
    @classmethod
    def default(cls) -> "Config":
        """A fresh Config with the journal's default thresholds."""
        return cls()

    def with_overrides(self, **kwargs) -> "Config":
        """Return a copy with the given (non-None) fields replaced.

        ``None`` values are ignored, so callers can pass argparse results
        straight through without first stripping unset options.
        """
        clean = {k: v for k, v in kwargs.items() if v is not None}
        if "checks" in clean:
            clean["checks"] = normalize_checks(clean["checks"])
        return replace(self, **clean)

    # -- journal interop ---------------------------------------------------- #
    def journal_args(self) -> List[str]:
        """The ``key=value`` tokens this config maps to on the journal command line.

        The journal reads ``grid=``, ``tol=``, ``tiny=``, ``dup=`` and
        ``checks=`` (note the short ``tol``/``tiny`` keys).
        """
        args = [
            "grid=%d" % self.grid,
            "tol=%g" % self.tol_mm3,
            "tiny=%g" % self.tiny_mm3,
            "dup=%g" % self.dup_mm,
        ]
        if self.checks and set(self.checks) != set(ALL_CHECKS):
            args.append("checks=" + ",".join(self.checks))
        return args


def normalize_checks(checks) -> List[str]:
    """Coerce a comma string or iterable of names into a clean check list.

    Accepts ``"a,b"``, ``["a", "b"]``, or ``None`` (-> all checks).  Unknown
    names raise :class:`ValueError` listing the offenders so a typo on the CLI
    fails loudly instead of silently doing nothing.
    """
    if checks is None:
        return list(ALL_CHECKS)
    if isinstance(checks, str):
        names = [c.strip() for c in checks.split(",") if c.strip()]
    else:
        names = [str(c).strip() for c in checks if str(c).strip()]
    if not names:
        return list(ALL_CHECKS)
    unknown = [n for n in names if n not in ALL_CHECKS]
    if unknown:
        raise ValueError(
            "unknown check(s): %s (valid: %s)"
            % (", ".join(unknown), ", ".join(ALL_CHECKS))
        )
    # de-duplicate while preserving the canonical journal order
    return [c for c in ALL_CHECKS if c in set(names)]
