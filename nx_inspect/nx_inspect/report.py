"""Render a loaded :class:`~nx_inspect.findings.Report` to console text or HTML.

Two renderers, both pure-Python and deterministic (no clocks, no dict-order
surprises — findings are sorted via :func:`~nx_inspect.findings.sorted_findings`):

* :func:`render_console` — a severity-grouped, column-aligned text summary with
  optional ANSI color (``--no-color`` yields plain ASCII with zero escapes).
* :func:`render_html` — a single self-contained HTML document (inline CSS, no
  external assets, no JS dependencies) with a header + severity badges, a
  client-sortable findings table colored by severity, and a collapsible body
  table.

Neither renderer touches the filesystem; :func:`write_html` is the only writer.
"""

import html
from typing import List, Optional

from .findings import (
    Body,
    Finding,
    Report,
    SEVERITIES,
    sorted_findings,
)

# --------------------------------------------------------------------------- #
# ANSI color
# --------------------------------------------------------------------------- #
_ANSI = {
    "reset": "\033[0m",
    "bold": "\033[1m",
    "dim": "\033[2m",
    "error": "\033[31m",    # red
    "warning": "\033[33m",  # yellow
    "info": "\033[36m",     # cyan
    "ok": "\033[32m",       # green
}

_SEV_LABEL = {"error": "ERROR", "warning": "WARNING", "info": "INFO"}
_SEV_SYMBOL = {"error": "X", "warning": "!", "info": "i"}


class _Painter:
    """Wraps text in ANSI codes, or returns it untouched when color is off."""

    def __init__(self, enabled: bool):
        self.enabled = enabled

    def __call__(self, text: str, *styles: str) -> str:
        if not self.enabled or not styles:
            return text
        prefix = "".join(_ANSI.get(s, "") for s in styles)
        return "%s%s%s" % (prefix, text, _ANSI["reset"])


def _fmt_num(v: Optional[float], nd: int = 2) -> str:
    """Format a number for display; ``None`` -> ``'-'``; integers stay clean."""
    if v is None:
        return "-"
    if abs(v - round(v)) < 10 ** (-nd) / 2 and abs(v) < 1e15:
        return "%d" % round(v)
    return ("%.*f" % (nd, v)).rstrip("0").rstrip(".")


def _fmt_vec(vec: Optional[List[float]], nd: int = 2) -> str:
    if vec is None:
        return "-"
    return "(" + ", ".join(_fmt_num(v, nd) for v in vec) + ")"


def _fmt_bbox(bbox: Optional[List[float]]) -> str:
    if bbox is None:
        return "-"
    lo = ", ".join(_fmt_num(v) for v in bbox[:3])
    hi = ", ".join(_fmt_num(v) for v in bbox[3:])
    return "[%s] -> [%s]" % (lo, hi)


# --------------------------------------------------------------------------- #
# console renderer
# --------------------------------------------------------------------------- #
def render_console(report: Report, color: bool = True) -> str:
    """Return a deterministic, severity-grouped console summary string."""
    paint = _Painter(color)
    counts = report.counts()
    lines: List[str] = []

    # -- header ------------------------------------------------------------- #
    kind = "assembly" if report.is_assembly else "part"
    lines.append(paint("nx_inspect", "bold") + "  " + paint(report.part, "bold")
                 + paint("  (%s)" % kind, "dim"))
    lines.append(paint(report.path, "dim"))
    lines.append("")

    # -- summary counts ----------------------------------------------------- #
    body_word = "body" if report.n_bodies == 1 else "bodies"
    summary_bits = [
        "%d %s" % (report.n_bodies, body_word),
        paint("%d error%s" % (counts["error"], "" if counts["error"] == 1 else "s"),
              "error" if counts["error"] else "dim"),
        paint("%d warning%s" % (counts["warning"], "" if counts["warning"] == 1 else "s"),
              "warning" if counts["warning"] else "dim"),
        paint("%d info" % counts["info"], "info" if counts["info"] else "dim"),
    ]
    lines.append("  ".join(summary_bits))
    if report.is_assembly and report.components:
        lines.append(paint("%d leaf component(s)" % len(report.components), "dim"))
    lines.append("")

    # -- findings, grouped by severity -------------------------------------- #
    ordered = sorted_findings(report.findings)
    if not ordered:
        lines.append(paint("No findings. Model is clean.", "ok", "bold"))
        return "\n".join(lines)

    for sev in SEVERITIES:
        group = [f for f in ordered if f.severity == sev]
        if not group:
            continue
        header = "%s  %s (%d)" % (_SEV_SYMBOL[sev], _SEV_LABEL[sev], len(group))
        lines.append(paint(header, sev, "bold"))
        for f in group:
            lines.extend(_console_finding(f, paint))
        lines.append("")

    # trailing verdict
    verdict = ("CLEAN (no errors)" if report.is_clean
               else "%d ERROR finding(s) — see above" % counts["error"])
    lines.append(paint(verdict, "ok" if report.is_clean else "error", "bold"))
    return "\n".join(lines).rstrip("\n")


def _console_finding(f: Finding, paint: _Painter) -> List[str]:
    out = ["    " + paint(f.title, "bold")]
    if f.detail:
        out.append("      " + f.detail)
    if f.bodies:
        shown = ", ".join(f.bodies[:8])
        if len(f.bodies) > 8:
            shown += ", ... (+%d)" % (len(f.bodies) - 8)
        out.append("      " + paint("bodies:   ", "dim") + shown)
    if f.location is not None:
        out.append("      " + paint("location: ", "dim") + _fmt_vec(f.location))
    if f.metric:
        metric = ", ".join("%s=%s" % (k, _fmt_metric_value(v)) for k, v in sorted(f.metric.items()))
        out.append("      " + paint("metric:   ", "dim") + metric)
    if f.suggestion:
        out.append("      " + paint("fix:      ", "dim") + f.suggestion)
    return out


def _fmt_metric_value(v) -> str:
    if isinstance(v, float):
        return _fmt_num(v)
    return str(v)


# --------------------------------------------------------------------------- #
# HTML renderer
# --------------------------------------------------------------------------- #
_HTML_CSS = """
:root{--err:#c0392b;--warn:#b9770e;--info:#1f6f8b;--ok:#1e8449;--ink:#1c2733;
--muted:#6b7785;--line:#dfe4ea;--bg:#f7f9fb;--card:#ffffff;}
*{box-sizing:border-box;}
body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.5 -apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;}
.wrap{max-width:1100px;margin:0 auto;padding:28px 20px 60px;}
header h1{margin:0 0 2px;font-size:22px;}
header .path{color:var(--muted);font-size:13px;word-break:break-all;}
header .kind{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;}
.badges{display:flex;flex-wrap:wrap;gap:10px;margin:18px 0 6px;}
.badge{border-radius:8px;padding:8px 14px;background:var(--card);
border:1px solid var(--line);min-width:84px;text-align:center;}
.badge .n{display:block;font-size:22px;font-weight:700;line-height:1.1;}
.badge .l{font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);}
.badge.error .n{color:var(--err);} .badge.warning .n{color:var(--warn);}
.badge.info .n{color:var(--info);} .badge.ok .n{color:var(--ok);}
.verdict{margin:14px 0 26px;font-weight:600;}
.verdict.clean{color:var(--ok);} .verdict.bad{color:var(--err);}
h2{font-size:16px;margin:30px 0 10px;border-bottom:1px solid var(--line);padding-bottom:6px;}
table{width:100%;border-collapse:collapse;background:var(--card);
border:1px solid var(--line);border-radius:8px;overflow:hidden;}
th,td{text-align:left;padding:8px 10px;border-bottom:1px solid var(--line);
vertical-align:top;font-size:13px;}
th{background:#eef2f6;font-weight:600;cursor:pointer;user-select:none;white-space:nowrap;}
th:hover{background:#e3e9ef;}
tr:last-child td{border-bottom:none;}
td.num{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap;}
.sev{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;
font-weight:700;text-transform:uppercase;letter-spacing:.04em;color:#fff;}
.sev.error{background:var(--err);} .sev.warning{background:var(--warn);}
.sev.info{background:var(--info);}
tr.error{background:rgba(192,57,43,.045);} tr.warning{background:rgba(185,119,14,.05);}
.findtitle{font-weight:600;} .detail{color:var(--muted);font-size:12px;margin-top:2px;}
.suggestion{font-size:12px;margin-top:4px;}
.bodies{font-size:12px;color:var(--muted);margin-top:3px;}
code{background:#eef2f6;border-radius:4px;padding:1px 5px;font-size:12px;}
details{margin-top:6px;} summary{cursor:pointer;font-size:16px;font-weight:600;
margin:30px 0 10px;}
.empty{color:var(--ok);font-weight:600;padding:14px 0;}
footer{margin-top:40px;color:var(--muted);font-size:12px;}
""".strip()

# Minimal, dependency-free click-to-sort for any <table data-sortable>.
_HTML_JS = """
document.querySelectorAll('table[data-sortable] th').forEach(function(th){
  th.addEventListener('click',function(){
    var table=th.closest('table'),tb=table.tBodies[0],
        idx=Array.prototype.indexOf.call(th.parentNode.children,th),
        rows=Array.prototype.slice.call(tb.rows),
        asc=!(th.dataset.asc==='1');
    th.parentNode.querySelectorAll('th').forEach(function(o){o.dataset.asc='';});
    th.dataset.asc=asc?'1':'0';
    rows.sort(function(a,b){
      var x=a.cells[idx],y=b.cells[idx],
          xv=x.dataset.sort!=null?x.dataset.sort:x.textContent,
          yv=y.dataset.sort!=null?y.dataset.sort:y.textContent,
          nx=parseFloat(xv),ny=parseFloat(yv),
          c=(!isNaN(nx)&&!isNaN(ny))?nx-ny:String(xv).localeCompare(String(yv));
      return asc?c:-c;
    });
    rows.forEach(function(r){tb.appendChild(r);});
  });
});
""".strip()


def _esc(text) -> str:
    return html.escape(str(text), quote=True)


def _badge(n: int, label: str, kind: str) -> str:
    klass = kind if n else "ok" if kind == "error" else kind
    return ('<div class="badge %s"><span class="n">%d</span>'
            '<span class="l">%s</span></div>' % (klass, n, _esc(label)))


def render_html(report: Report, title: Optional[str] = None) -> str:
    """Return a complete, self-contained HTML document for ``report`` (deterministic)."""
    counts = report.counts()
    doc_title = title or ("nx_inspect — %s" % report.part)
    kind = "Assembly" if report.is_assembly else "Part"

    parts: List[str] = []
    parts.append("<!doctype html>")
    parts.append('<html lang="en"><head><meta charset="utf-8">')
    parts.append('<meta name="viewport" content="width=device-width,initial-scale=1">')
    parts.append("<title>%s</title>" % _esc(doc_title))
    parts.append("<style>%s</style></head><body><div class=\"wrap\">" % _HTML_CSS)

    # header
    parts.append("<header>")
    parts.append('<div class="kind">%s &middot; nx_inspect report (schema v%s)</div>'
                 % (_esc(kind), _esc(report.schema)))
    parts.append("<h1>%s</h1>" % _esc(report.part))
    parts.append('<div class="path">%s</div>' % _esc(report.path))
    parts.append("</header>")

    # badges
    parts.append('<div class="badges">')
    parts.append(_badge(report.n_bodies, "bodies", "ok"))
    parts.append(_badge(counts["error"], "errors", "error"))
    parts.append(_badge(counts["warning"], "warnings", "warning"))
    parts.append(_badge(counts["info"], "info", "info"))
    if report.is_assembly:
        parts.append(_badge(len(report.components), "components", "ok"))
    parts.append("</div>")

    if report.is_clean:
        parts.append('<div class="verdict clean">CLEAN &mdash; no error findings.</div>')
    else:
        parts.append('<div class="verdict bad">%d error finding(s) &mdash; review below.</div>'
                     % counts["error"])

    # findings table
    parts.append("<h2>Findings</h2>")
    parts.append(_html_findings_table(report))

    # body table (collapsible)
    parts.append(_html_body_section(report.bodies))

    # components (assemblies)
    if report.is_assembly and report.components:
        parts.append(_html_components_section(report))

    parts.append('<footer>Generated by nx_inspect. Read-only inspection; the part '
                 'was not modified. Units: %s.</footer>' % _esc(report.units))
    parts.append("</div>")
    parts.append("<script>%s</script>" % _HTML_JS)
    parts.append("</body></html>")
    return "\n".join(parts) + "\n"


def _html_findings_table(report: Report) -> str:
    ordered = sorted_findings(report.findings)
    if not ordered:
        return '<p class="empty">No findings. Model is clean.</p>'

    rows = []
    head = ("<thead><tr><th>Severity</th><th>Check</th><th>Finding</th>"
            "<th>Bodies</th><th>Location</th><th>Metric</th></tr></thead>")
    from .findings import severity_rank
    for f in ordered:
        bodies = ", ".join(_esc(b) for b in f.bodies) if f.bodies else "&mdash;"
        loc = _esc(_fmt_vec(f.location)) if f.location is not None else "&mdash;"
        metric = (", ".join("%s=%s" % (_esc(k), _esc(_fmt_metric_value(v)))
                            for k, v in sorted(f.metric.items()))
                  if f.metric else "&mdash;")
        finding_cell = '<div class="findtitle">%s</div>' % _esc(f.title)
        if f.detail:
            finding_cell += '<div class="detail">%s</div>' % _esc(f.detail)
        if f.suggestion:
            finding_cell += '<div class="suggestion"><b>Fix:</b> %s</div>' % _esc(f.suggestion)
        rows.append(
            '<tr class="%s">'
            '<td data-sort="%d"><span class="sev %s">%s</span></td>'
            '<td><code>%s</code></td>'
            '<td>%s</td>'
            '<td class="bodies">%s</td>'
            '<td>%s</td>'
            '<td>%s</td></tr>'
            % (f.severity, severity_rank(f.severity), f.severity, _esc(_SEV_LABEL[f.severity]),
               _esc(f.check), finding_cell, bodies, loc, metric)
        )
    return ('<table data-sortable>%s<tbody>%s</tbody></table>'
            % (head, "".join(rows)))


#: Sort sentinel for an unmeasured (None) numeric cell.  It is larger than any
#: realistic mm^3 / mm^2 / kg value, so missing values sort *last* ascending and
#: first descending instead of falling into the lexical (NaN) branch.
_MISSING_SORT = 1e300


def _num_sort(v: Optional[float]) -> str:
    """data-sort attribute value for an optional numeric cell (sentinel if None)."""
    return repr(_MISSING_SORT) if v is None else repr(float(v))


def _html_body_section(bodies: List[Body]) -> str:
    if not bodies:
        return ("<summary>Bodies (0)</summary>"
                '<p class="empty">No solid bodies in this part.</p>')
    head = ("<thead><tr><th>ID</th><th>Name</th><th>Volume (mm&sup3;)</th>"
            "<th>Area (mm&sup2;)</th><th>Mass (kg)</th><th>Centroid</th>"
            "<th>Bounding box</th></tr></thead>")
    rows = []
    for b in bodies:
        rows.append(
            "<tr>"
            '<td class="num" data-sort="%d">%d</td>'
            "<td>%s</td>"
            '<td class="num" data-sort="%s">%s</td>'
            '<td class="num" data-sort="%s">%s</td>'
            '<td class="num" data-sort="%s">%s</td>'
            "<td>%s</td><td>%s</td></tr>"
            % (b.id, b.id, _esc(b.name),
               _num_sort(b.volume_mm3), _esc(_fmt_num(b.volume_mm3)),
               _num_sort(b.area_mm2), _esc(_fmt_num(b.area_mm2)),
               _num_sort(b.mass_kg), _esc(_fmt_num(b.mass_kg, 4)),
               _esc(_fmt_vec(b.centroid)), _esc(_fmt_bbox(b.bbox)))
        )
    table = ('<table data-sortable>%s<tbody>%s</tbody></table>'
             % (head, "".join(rows)))
    return ("<details><summary>Bodies (%d)</summary>%s</details>"
            % (len(bodies), table))


def _html_components_section(report: Report) -> str:
    head = ("<thead><tr><th>Component</th><th>Source part</th>"
            "<th>Origin</th></tr></thead>")
    rows = []
    for c in report.components:
        rows.append("<tr><td>%s</td><td><code>%s</code></td><td>%s</td></tr>"
                    % (_esc(c.name), _esc(c.part or "&mdash;"), _esc(_fmt_vec(c.origin))))
    table = ('<table data-sortable>%s<tbody>%s</tbody></table>'
             % (head, "".join(rows)))
    return ("<details><summary>Components (%d)</summary>%s</details>"
            % (len(report.components), table))


def write_html(report: Report, path: str, title: Optional[str] = None) -> str:
    """Render ``report`` to HTML and write it to ``path`` (UTF-8). Returns the path."""
    doc = render_html(report, title=title)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(doc)
    return path
