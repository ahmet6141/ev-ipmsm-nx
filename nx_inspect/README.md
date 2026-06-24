# nx_inspect

**A professional, open-source model-quality inspector for Siemens NX.**

`nx_inspect` finds the model problems that quietly break downstream FEA, CAM,
per-part export, and assembly work: bodies that interpenetrate, degenerate
zero-volume solids, stray slivers left by booleans, accidental double-builds,
and bodies that are unnamed or ambiguously named. It runs read-only against the
**true NX B-rep geometry** (not a mesh approximation) and produces a clean
console summary, a shareable self-contained HTML report, and a machine-readable
JSON report you can hand to CI or an AI agent.

It is **generic**: it makes no assumptions about any particular part or project
and works on any NX solid-body part or assembly.

```
$ nx-inspect knuckle.prt --html knuckle_report.html
nx_inspect  knuckle  (part)
C:\work\knuckle.prt

  3 bodies   1 error   0 warnings   2 info

X  ERROR (1)
    Bodies interpenetrate (HOUSING ∩ SHAFT)
      Estimated overlap volume ~412 mm^3 (37 interior sample points).
      bodies:   HOUSING, SHAFT
      location: (12.4, -3.1, 0.0)
      metric:   overlap_volume_mm3=412, sample_points=37
      fix:      Unite the two into one body, separate them so they only touch ...

i  INFO (2)
    ...

1 ERROR finding(s) — see above
```

---

## Why

CAD model defects are cheap to fix in CAD and expensive everywhere else. A
sliver fragment crashes a mesher; a hidden interference invalidates a contact
analysis; an unnamed body can't be selected by role in a CAM setup or exported
per-part. `nx_inspect` catches these in seconds, and its exit code is the number
of *error* findings, so you can gate a pipeline on a clean model.

## Architecture

```
                         (runs INSIDE NX)              (pure Python, no NX)
  part.prt  ──►  run_journal.exe inspect_journal.py  ──►  report.json  ──►  nx-inspect CLI
                 │ reads true B-rep geometry          │  schema v1        │  console + HTML
                 │ interference / volume / naming      │                   │  exit = #errors
                 └─────────────────────────────────────┘                   └────────────────►
```

* The **NX-side journal** (`nx_inspect/journal/inspect_journal.py`) is the only
  NX-aware code. It runs inside NX via `run_journal`, measures real geometry,
  and writes a single JSON report. It never modifies the part.
* The **pure-Python package** (everything else) locates `run_journal`, drives
  the journal, then loads, validates, and renders the JSON. It is standard
  library only (Python 3.8+), so it installs anywhere — including machines
  without NX, for offline/CI rendering of existing reports.

## Features

- Read-only inspection of the real NX solid geometry (no mesh approximation).
- Six checks across three severities (see catalog below).
- Console report: severity-grouped, aligned, optional ANSI color (`--no-color`).
- Self-contained HTML report: inline CSS/JS, no external assets, severity
  badges, a click-sortable findings table, and a collapsible body table with
  volume / area / mass / centroid / bounding box.
- Machine-readable JSON (schema v1) for CI and AI agents.
- CI-friendly exit code = number of **error** findings (`0` = clean).
- Offline mode (`--from-json`) renders an existing report with no NX involved.
- Generic `run_journal.exe` discovery (env / registry / common dirs) — no
  hardcoded install path.
- Pure Python, standard library only, MIT licensed.

## Install

```bash
# isolated CLI install (recommended)
pipx install nx-inspect

# or from a checkout, editable
pip install -e .

# with the test extras
pip install -e ".[dev]"
```

No third-party dependencies are pulled in — the package is standard library
only by design.

## Quick start

```bash
# Inspect a part and write an HTML report (live; needs NX + run_journal)
nx-inspect path/to/part.prt --html report.html

# Tune thresholds and limit the checks
nx-inspect part.prt --grid 12 --tiny 50 --checks interference,tiny_body

# Render an EXISTING report with no NX (offline / CI / sharing)
nx-inspect --from-json examples/suspension_report.json --html report.html --no-color

# Same via the module form
python -m nx_inspect --from-json examples/suspension_report.json
```

Exit code is the number of error findings, so in CI:

```bash
nx-inspect part.prt || echo "model has $? error(s)"
```

The exit code is **clamped to 63** for findings, and operational failures use
distinct codes **at or above 64**, so a CI gate can always tell "the tool broke"
apart from "the model has N errors":

| Exit code | Meaning                                                      |
|-----------|--------------------------------------------------------------|
| `0`       | clean — no error findings                                    |
| `1`–`63`  | that many error findings (a model with ≥63 errors reports 63)|
| `64`      | bad arguments (usage error)                                  |
| `65`      | report JSON failed schema validation                         |
| `66`      | input part / report file missing                             |
| `69`      | `run_journal.exe` not found                                  |
| `70`      | journal ran but produced no/invalid report                   |

## Check catalog

| Check            | Severity | What it flags                                                        |
|------------------|----------|----------------------------------------------------------------------|
| `interference`   | error    | Two solid bodies that interpenetrate (overlap in space). Touching faces / press fits score ~0 and are **not** flagged. |
| `zero_volume`    | error    | A "solid" body with zero or negative measured volume (degenerate).   |
| `tiny_body`      | warning  | A sliver body below the `--tiny` mm³ threshold — often an unintended boolean fragment. |
| `duplicate_body` | warning  | Two bodies with the same volume and a coincident centroid — a likely double-build. |
| `unnamed_body`   | info     | A body with no display name (hurts FEA/CAM selection and per-part export). |
| `duplicate_name` | info     | The same display name on multiple bodies (ambiguous selection).      |

Thresholds (CLI flag → report key, with defaults):

| Flag       | Report key | Default | Meaning                                                  |
|------------|------------|---------|----------------------------------------------------------|
| `--grid`   | `grid`     | `10`    | Interference sample grid per axis (`grid³` points/region)|
| `--tol`    | `tol_mm3`  | `50`    | Min interference overlap volume (mm³) to flag            |
| `--tiny`   | `tiny_mm3` | `30`    | Sliver volume threshold (mm³)                            |
| `--dup`    | `dup_mm`   | `1.0`   | Duplicate-body centroid coincidence tolerance (mm)       |
| `--checks` | `checks`   | all     | Comma-separated subset of checks to run                  |

## How `run_journal` is located

`run_journal.exe` lives at `<NX base>/NXBIN/run_journal.exe`. The CLI never
hardcodes an install path; it searches, in order:

1. `--run-journal PATH` (explicit override; may point at the exe or a base dir).
2. Environment variables `UGII_BASE_DIR` and `UGII_ROOT_DIR`.
3. The Windows registry: `HKLM\SOFTWARE\Siemens\NX\<version>` (`INSTALLDIR` /
   `UGII_BASE_DIR`), including the `WOW6432Node` mirror.
4. Common install roots (`C:\Program Files\Siemens\*`, `E:\program`, …).

If none resolve, the error message lists every path that was checked so you can
fix your environment or pass `--run-journal`.

## JSON report schema (v1)

The journal emits exactly this structure; the CLI validates it before rendering:

```json
{
  "tool": "nx_inspect", "schema": "1", "part": "...", "path": "...",
  "units": "mm", "is_assembly": false,
  "config": { "grid": 10, "tol_mm3": 50.0, "tiny_mm3": 30.0, "dup_mm": 1.0, "checks": ["..."] },
  "summary": { "n_bodies": 56, "errors": 0, "warnings": 0, "info": 1, "checks_run": ["..."] },
  "components": [ { "name": "...", "part": "...", "origin": [x, y, z] } ],
  "bodies": [
    { "id": 0, "name": "...", "volume_mm3": 0.0, "area_mm2": 0.0, "mass_kg": 0.0,
      "centroid": [x, y, z], "bbox": [x0, y0, z0, x1, y1, z1] }
  ],
  "findings": [
    { "check": "...", "severity": "error|warning|info", "title": "...", "detail": "...",
      "bodies": ["..."], "location": [x, y, z], "metric": { }, "suggestion": "..." }
  ]
}
```

A real sample produced by the journal (a 56-body suspension assembly export) is
in [`examples/suspension_report.json`](examples/suspension_report.json), with the
rendered HTML in [`examples/suspension_report.html`](examples/suspension_report.html).

## Using it with Claude / AI agents

The JSON report is designed to be agent-friendly. A coding or CAD agent can:

1. Run `nx-inspect part.prt --out report.json` (or read an existing report).
2. Read `report.json` — `summary` gives the headline counts; `findings` is a
   ranked list where each entry has a `check`, `severity`, the offending
   `bodies`, a `location`, a numeric `metric`, and a concrete `suggestion`.
3. Act: the `suggestion` text and `metric` are written to be directly
   actionable (e.g. "unite the two bodies", "remove the duplicate build step").

Because the exit code equals the error count, an agent loop can keep editing the
generating script and re-inspecting until `nx-inspect` exits `0`. Point the
agent at the JSON, not the console text — the JSON is stable and typed.

## Limitations

- **Single-part bodies** are inspected fully (geometry-measured checks run on
  every solid body in the work part).
- **Assemblies** are handled by listing their leaf **components**
  (`is_assembly: true`, populated `components`); the body-level checks apply to
  the bodies present in the inspected part. For full cross-component
  interference, inspect a part that contains the bodies (e.g. an exported
  single-part representation).
- The interference check is a **grid sampling** estimate (true NX point
  containment at `grid³` points per overlap region): it is conservative and
  tunable via `--grid`/`--tol`, not an exact Boolean intersection volume.
- The journal requires a licensed NX with `run_journal`; the pure-Python side
  (including `--from-json`) needs neither.
- In live mode the journal also writes a `<part>_inspect.json` sidecar next to
  the `.prt` (in addition to `--out`); it is git-ignored. Use `--out` to control
  the primary report location.

## Contributing

Contributions are welcome. Please:

- Keep the pure-Python package **standard library only** (no third-party
  runtime deps) and **NX-free** — all NX-aware code lives in the journal.
- Do not change the v1 report schema or the journal without a schema bump; the
  loader and renderer treat schema `"1"` as a contract.
- Add or update tests under `tests/` (pytest, CPython-only, no NX required) and
  run them before opening a PR:

  ```bash
  python -m pytest -q
  ```

- Keep renderer output **deterministic** (findings are sorted; no timestamps).

## License

MIT — see [LICENSE](LICENSE). Copyright (c) 2026 nx_inspect contributors.
