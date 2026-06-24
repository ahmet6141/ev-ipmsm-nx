"""The report model + loader for nx_inspect.

This module is the single authority on the **v1 report schema** emitted by the
NX-side journal (see the journal docstring).  It provides:

* dataclasses :class:`Body`, :class:`Finding`, :class:`Component`, :class:`Report`;
* :func:`load_report` / :func:`load_report_file`, which parse *and validate* a
  report against the schema (raising :class:`SchemaError` on any violation);
* a fixed :data:`SEVERITY_ORDER` (error > warning > info) plus helpers for
  counting, sorting and grouping findings.

Validation is intentionally strict-but-tolerant: every field the renderer needs
is checked for presence and type, but unknown extra keys are ignored so a future
journal that adds fields does not break an older CLI.
"""

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

# --------------------------------------------------------------------------- #
# severity
# --------------------------------------------------------------------------- #
#: Canonical severities, most-severe first.
SEVERITIES = ("error", "warning", "info")

#: Sort weight per severity (lower sorts first == more severe first).
SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}


def severity_rank(severity: str) -> int:
    """Sort key for a severity string; unknown severities sort last."""
    return SEVERITY_ORDER.get(severity, len(SEVERITY_ORDER))


# --------------------------------------------------------------------------- #
# errors
# --------------------------------------------------------------------------- #
class SchemaError(ValueError):
    """Raised when a report JSON does not conform to the v1 schema."""


# --------------------------------------------------------------------------- #
# small validation helpers
# --------------------------------------------------------------------------- #
def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise SchemaError(msg)


def _get(d: dict, key: str, where: str):
    _require(isinstance(d, dict), "%s: expected an object" % where)
    _require(key in d, "%s: missing required key '%s'" % (where, key))
    return d[key]


def _as_str(v, where: str) -> str:
    _require(isinstance(v, str), "%s: expected a string, got %s" % (where, type(v).__name__))
    return v


def _as_int(v, where: str) -> int:
    # bool is a subclass of int; reject it explicitly to catch sloppy data.
    _require(isinstance(v, int) and not isinstance(v, bool),
             "%s: expected an integer, got %s" % (where, type(v).__name__))
    return v


def _as_bool(v, where: str) -> bool:
    _require(isinstance(v, bool), "%s: expected a boolean, got %s" % (where, type(v).__name__))
    return v


def _as_opt_number(v, where: str) -> Optional[float]:
    if v is None:
        return None
    _require(isinstance(v, (int, float)) and not isinstance(v, bool),
             "%s: expected a number or null, got %s" % (where, type(v).__name__))
    return float(v)


def _as_opt_vec(v, where: str, length: Optional[int] = None) -> Optional[List[float]]:
    if v is None:
        return None
    _require(isinstance(v, (list, tuple)), "%s: expected a list or null" % where)
    if length is not None:
        _require(len(v) == length, "%s: expected %d numbers, got %d" % (where, length, len(v)))
    out = []
    for i, x in enumerate(v):
        _require(isinstance(x, (int, float)) and not isinstance(x, bool),
                 "%s[%d]: expected a number" % (where, i))
        out.append(float(x))
    return out


# --------------------------------------------------------------------------- #
# dataclasses
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Body:
    """One solid body's measured geometry (all lengths in mm)."""

    id: int
    name: str
    volume_mm3: Optional[float] = None
    area_mm2: Optional[float] = None
    mass_kg: Optional[float] = None
    centroid: Optional[List[float]] = None  # [x, y, z]
    bbox: Optional[List[float]] = None       # [x0, y0, z0, x1, y1, z1]

    @classmethod
    def from_dict(cls, d: dict, where: str) -> "Body":
        return cls(
            id=_as_int(_get(d, "id", where), where + ".id"),
            name=_as_str(_get(d, "name", where), where + ".name"),
            volume_mm3=_as_opt_number(d.get("volume_mm3"), where + ".volume_mm3"),
            area_mm2=_as_opt_number(d.get("area_mm2"), where + ".area_mm2"),
            mass_kg=_as_opt_number(d.get("mass_kg"), where + ".mass_kg"),
            centroid=_as_opt_vec(d.get("centroid"), where + ".centroid", 3),
            bbox=_as_opt_vec(d.get("bbox"), where + ".bbox", 6),
        )


@dataclass(frozen=True)
class Component:
    """One leaf component of an assembly (assemblies only)."""

    name: str
    part: str = ""
    origin: Optional[List[float]] = None  # [x, y, z]

    @classmethod
    def from_dict(cls, d: dict, where: str) -> "Component":
        return cls(
            name=_as_str(_get(d, "name", where), where + ".name"),
            part=_as_str(d.get("part", ""), where + ".part"),
            origin=_as_opt_vec(d.get("origin"), where + ".origin", 3),
        )


@dataclass(frozen=True)
class Finding:
    """A single model-quality finding."""

    check: str
    severity: str
    title: str
    detail: str = ""
    bodies: List[str] = field(default_factory=list)
    location: Optional[List[float]] = None  # [x, y, z]
    metric: Dict = field(default_factory=dict)
    suggestion: str = ""

    @classmethod
    def from_dict(cls, d: dict, where: str) -> "Finding":
        sev = _as_str(_get(d, "severity", where), where + ".severity")
        _require(sev in SEVERITIES,
                 "%s.severity: '%s' is not one of %s" % (where, sev, ", ".join(SEVERITIES)))
        raw_bodies = d.get("bodies", [])
        _require(isinstance(raw_bodies, (list, tuple)), "%s.bodies: expected a list" % where)
        bodies = [_as_str(b, "%s.bodies[%d]" % (where, i)) for i, b in enumerate(raw_bodies)]
        metric = d.get("metric", {})
        _require(isinstance(metric, dict), "%s.metric: expected an object" % where)
        return cls(
            check=_as_str(_get(d, "check", where), where + ".check"),
            severity=sev,
            title=_as_str(_get(d, "title", where), where + ".title"),
            detail=_as_str(d.get("detail", ""), where + ".detail"),
            bodies=bodies,
            location=_as_opt_vec(d.get("location"), where + ".location", 3),
            metric=metric,
            suggestion=_as_str(d.get("suggestion", ""), where + ".suggestion"),
        )


@dataclass(frozen=True)
class Report:
    """A fully-parsed, validated inspection report (schema v1)."""

    part: str
    path: str
    units: str
    is_assembly: bool
    config: Dict
    bodies: List[Body]
    findings: List[Finding]
    components: List[Component] = field(default_factory=list)
    schema: str = "1"
    tool: str = "nx_inspect"

    # -- derived summary ---------------------------------------------------- #
    def counts(self) -> Dict[str, int]:
        """Finding counts by severity, e.g. ``{'error': 0, 'warning': 2, 'info': 1}``."""
        out = {s: 0 for s in SEVERITIES}
        for f in self.findings:
            out[f.severity] = out.get(f.severity, 0) + 1
        return out

    @property
    def n_errors(self) -> int:
        return self.counts()["error"]

    @property
    def n_warnings(self) -> int:
        return self.counts()["warning"]

    @property
    def n_info(self) -> int:
        return self.counts()["info"]

    @property
    def n_bodies(self) -> int:
        return len(self.bodies)

    @property
    def is_clean(self) -> bool:
        """True when there are no error-severity findings (CI pass)."""
        return self.n_errors == 0

    def sorted_findings(self) -> List[Finding]:
        """Findings most-severe first, then by check, then by title (stable, deterministic)."""
        return sorted_findings(self.findings)

    def findings_by_check(self) -> "Dict[str, List[Finding]]":
        """Findings grouped by their ``check``, in canonical severity/check order."""
        return group_by_check(self.findings)


# --------------------------------------------------------------------------- #
# free-function helpers (usable without a Report instance)
# --------------------------------------------------------------------------- #
def count_by_severity(findings: Sequence[Finding]) -> Dict[str, int]:
    out = {s: 0 for s in SEVERITIES}
    for f in findings:
        out[f.severity] = out.get(f.severity, 0) + 1
    return out


def sorted_findings(findings: Sequence[Finding]) -> List[Finding]:
    """Deterministic sort: severity (error first), then check, then title."""
    return sorted(findings, key=lambda f: (severity_rank(f.severity), f.check, f.title))


def group_by_check(findings: Sequence[Finding]) -> Dict[str, List[Finding]]:
    """Group findings by check name; groups ordered by their most-severe member."""
    groups: Dict[str, List[Finding]] = {}
    for f in sorted_findings(findings):
        groups.setdefault(f.check, []).append(f)
    return groups


# --------------------------------------------------------------------------- #
# loader / validator
# --------------------------------------------------------------------------- #
def load_report(data: dict) -> Report:
    """Validate a parsed JSON object against schema v1 and return a :class:`Report`.

    Raises :class:`SchemaError` on any structural or type violation.
    """
    _require(isinstance(data, dict), "report: top level must be a JSON object")

    tool = _as_str(data.get("tool", "nx_inspect"), "report.tool")
    _require(tool == "nx_inspect",
             "report.tool: expected 'nx_inspect', got %r (is this an nx_inspect report?)" % tool)
    schema = _as_str(_get(data, "schema", "report"), "report.schema")
    _require(schema == "1", "report.schema: unsupported version '%s' (this tool reads v1)" % schema)

    part = _as_str(_get(data, "part", "report"), "report.part")
    path = _as_str(_get(data, "path", "report"), "report.path")
    units = _as_str(data.get("units", "mm"), "report.units")
    is_assembly = _as_bool(data.get("is_assembly", False), "report.is_assembly")

    config = _get(data, "config", "report")
    _require(isinstance(config, dict), "report.config: expected an object")

    summary = data.get("summary")
    if summary is not None:
        _require(isinstance(summary, dict), "report.summary: expected an object")

    raw_bodies = _get(data, "bodies", "report")
    _require(isinstance(raw_bodies, list), "report.bodies: expected a list")
    bodies = [Body.from_dict(b, "bodies[%d]" % i) for i, b in enumerate(raw_bodies)]

    raw_findings = _get(data, "findings", "report")
    _require(isinstance(raw_findings, list), "report.findings: expected a list")
    findings = [Finding.from_dict(f, "findings[%d]" % i) for i, f in enumerate(raw_findings)]

    raw_components = data.get("components", []) or []
    _require(isinstance(raw_components, list), "report.components: expected a list")
    components = [Component.from_dict(c, "components[%d]" % i) for i, c in enumerate(raw_components)]

    # Cross-check the journal's own summary if present (a corrupt/edited report
    # whose summary disagrees with its findings is suspicious).
    if summary is not None:
        actual = count_by_severity(findings)
        for sev, key in (("error", "errors"), ("warning", "warnings"), ("info", "info")):
            if key in summary:
                claimed = summary[key]
                _require(
                    isinstance(claimed, int) and not isinstance(claimed, bool),
                    "report.summary.%s: expected an integer" % key,
                )
                _require(
                    claimed == actual[sev],
                    "report.summary.%s (%s) disagrees with the %d %s finding(s) present"
                    % (key, claimed, actual[sev], sev),
                )
        if "n_bodies" in summary:
            _require(summary["n_bodies"] == len(bodies),
                     "report.summary.n_bodies (%s) disagrees with %d bodies present"
                     % (summary["n_bodies"], len(bodies)))

    return Report(
        part=part,
        path=path,
        units=units,
        is_assembly=is_assembly,
        config=config,
        bodies=bodies,
        findings=findings,
        components=components,
        schema=schema,
        tool=tool,
    )


def load_report_file(path: str) -> Report:
    """Read a report JSON file from ``path`` and validate it.

    Raises :class:`SchemaError` (including for malformed JSON) so callers have a
    single exception type to catch.
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        raise SchemaError("report file not found: %s" % path)
    except json.JSONDecodeError as exc:
        raise SchemaError("report file is not valid JSON (%s): %s" % (path, exc))
    return load_report(data)
