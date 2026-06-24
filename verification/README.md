# verification/ — external-tool drivers (P1 EM-FEA, P6 drawings)

These scripts execute the project's verification phases in the licensed tools the
`motor_nx` package hands off to. They are the runnable counterparts of the plan in
[`docs/PROJECT_PLAN.md`](../docs/PROJECT_PLAN.md): `motor_nx` produces the design +
hand-off package; these drivers push it into Motor-CAD / NX and read results back.

| Script | Phase | Tool | Run with |
|---|---|---|---|
| `femm_emag.py` | **P1** EM-FEA (free) | FEMM 4.2 (pyFEMM) | plain `python` |
| `motorcad_emag.py` | **P1** EM-FEA (+ P2/P3 seeds) | Ansys Motor-CAD (PyMotorCAD) | plain `python` |
| `simcenter_emag.py` | **P1** EM-FEA | Simcenter 3D Magnetics (NXOpen journal) | `run_journal.exe` |
| `simcenter_thermal.py` | **P2** thermal (continuous rating) | Simcenter 3D Thermal (NXOpen journal) | `run_journal.exe` |
| `simcenter_structural.py` | **P3** rotor stress + rotordynamics | Simcenter Nastran SOL 101/103 (NXOpen journal) | `run_journal.exe` |
| `simcenter_run_all.py` | **P1→P3** combined gate | Simcenter 3D (NXOpen journal) | `run_journal.exe` |
| `simcenter_common.py` | shared layer for the Simcenter journals | — (pure Python, no NX) | plain `python` (self-check) |
| `nx_drafting.py` | **P6** formal 2D drawings | Siemens NX (NXOpen journal) | `run_journal.exe` |
| `femm_geom.py` | geometry layer for `femm_emag` | — (pure Python, no FEMM) | plain `python` |

The numeric answers come from FEMM / Motor-CAD / NX. The drivers only map the frozen
design onto tool inputs, run the matrix headless, and report **go / no-go against
`fea_spec.acceptance_targets`**. They do not compute physics themselves.

`femm_emag.py` and `motorcad_emag.py` are interchangeable EM-FEA front ends scored on
the **same** acceptance gate — run the free FEMM one when no Ansys license is available;
the plan's **G1 gate** wants the two within ~3–5 % of each other on average torque.

## The hand-off chain

```
params.py ──► python -m motor_nx.cli fea -o fea/
                         │
                         ├─ fea/fea_spec.json     (geometry, materials, winding, excitation,
                         │                          operating points, analysis matrix, targets)
                         ├─ fea/winding.csv        (slot → phase → sign)
                         ├─ fea/femm_labels.csv    (block-label recipe for the FEMM cross-check)
                         └─ fea/cross_section.dxf  (2D lamination plane, per-material layers)
                         │
        ┌────────────────┴───────────────────┐
        ▼                                     ▼
verification/motorcad_emag.py        verification/femm_emag.py  (free; via femm_geom.py)
   → fea/motorcad_results.json          → fea/femm_results.json
```

## P1 — Motor-CAD EM-FEA (`motorcad_emag.py`)

```bash
pip install ansys-motorcad-core          # one time; needs a licensed Motor-CAD install
python -m motor_nx.cli fea -o fea/        # (re)generate the hand-off package
python verification/motorcad_emag.py --spec fea/fea_spec.json --out fea/motorcad_results.json
```

Stages (subset via `--stages`): `cogging, back_emf, torque_angle, ripple, demag,
thermal, mechanical`. The `torque_angle` stage sweeps current advance (β) at peak
current to locate **MTPA**; `ripple` re-solves at that β; `demag` re-solves at peak
current and the hot (150 °C) magnet temperature. `thermal`/`mechanical` are P2/P3 seeds.

Output `fea/motorcad_results.json` carries every stage's numbers plus an `acceptance`
block scored against `fea_spec.acceptance_targets` (peak torque, ripple ≤ 5 %, cogging
≤ 1 % of rated, no demag, rotor SF ≥ 1.5, peak efficiency ≥ 95 %).

**Before the first run — confirm variable names.** Motor-CAD scripting names differ by
rotor template and version. The slot / airgap / stack names are stable; the **V-magnet
pocket** names (bridge / web / V-angle / magnet bar width) are flagged in the script. It
sets the best-known name, logs whether each set/get succeeded, and prints the **target
value** so you can finish any missed field by hand (right-click a field in Motor-CAD →
*Copy variable name* to get the exact string, then edit `GEOM_VARS` / the V-pocket block).
Also verify the auto winding pattern equals `fea/winding.csv` (the script reports kw vs
the spec's 0.96).

## P6 — NX formal drawing (`nx_drafting.py`)

```bat
"%UGII_ROOT_DIR%\run_journal.exe" verification\nx_drafting.py -args motor_v8.prt A3
"%UGII_ROOT_DIR%\run_journal.exe" verification\nx_drafting.py -args motor_v8.prt A3 dxf
```

Enters Drafting, adds a sheet (A3 default) + associative FRONT and TFR-ISO model views,
and stamps the **title block, GD&T / critical-dimension schedule, general notes and BOM**
as native NX notes — all from the live design data (`motor_nx.manufacturing`,
`em_design`). The optional `dxf` arg additionally imports the already-dimensioned
`motor_nx.drawings` sheets (assembly / stator / rotor) onto their own sheets, so the
fully-dimensioned 2D output lands in NX with no clean-up.

Like `nx_builder.py`, every NXOpen step is wrapped and logged to the Listing Window; a
drifted member name degrades to a warning instead of aborting. Fine view placement and
fully-associative PMI re-attachment are quick interactive touch-ups.

## P1 — FEMM EM-FEA (`femm_emag.py`, free / no license)

The license-free EM-FEA front end. It builds a **full 54-slot / 6-pole** magnetostatic
model in FEMM directly from the parametric geometry (no DXF import / manual labelling)
and runs the same stage matrix, scored on the same gate as the Motor-CAD driver.

```bash
pip install pyfemm                        # COM client (already in .venv)
winget install DavidMeeker.FEMM           # one-time FEMM 4.2 install (or femm.info/wiki/Download)
python -m motor_nx.cli fea -o fea/        # (re)generate the hand-off package
python verification/femm_emag.py --out fea/femm_results.json
python verification/femm_emag.py --stages cogging,back_emf,torque_angle,ripple,demag --coarse
```

Architecture (mirrors the project's *pure-math blueprint → thin emitter* split):

* **`femm_geom.py`** — pure Python, **no FEMM**: turns `motor_nx.blueprint` polygons +
  `em_design` radii + `fea.femm_label_recipe` into a planar straight-line graph
  (nodes/segments/arcs/block-labels). Runnable standalone (`python verification/femm_geom.py`
  prints a sanity report) and unit-tested in `tests/test_femm_geom.py`. The stator bore is
  broken into tooth-tip arcs with a **gap at each slot mouth**, so the slot opening stays
  continuous with the air gap (accurate cogging) while every region is still bounded.
* **`femm_emag.py`** — the thin emitter: replays that graph via `mi_*`, defines the
  nonlinear lamination BH / NdFeB recoil / coil materials, the A/B/C series circuits and the
  A = 0 OD boundary, then rotates the **rotor group** and re-solves each step. Torque is the
  weighted Maxwell stress tensor over the clean inner air-gap ring; flux linkage comes from
  `mo_getcircuitproperties`.

Stages (`--stages`): `cogging` (1 slot pitch, no current), `back_emf` (open-circuit flux
linkage → fundamental EMF + THD), `torque_angle` (β sweep at peak current → MTPA), `ripple`
(one electrical period at MTPA), `demag` (peak −d current at the hot magnet temp, materials
rebuilt with derated Br/Hc → min magnet B vs knee). Output `fea/femm_results.json` carries
every stage plus the `acceptance` block.

> **Parallel paths.** The FEMM circuit folds the winding's parallel paths into the turns so
> its flux linkage is the **terminal** value (turns/slot = `conductors_per_slot / parallel_paths`,
> circuit current = full phase current). Verify the resulting back-EMF against the analytical
> `ke` from `em_design` on the first run.
```

## Simcenter 3D — NXOpen journals (P1 EM + P2 thermal + P3 structural)

Platform-native verification on the **Siemens NX 2506 / Simcenter 3D** install (the same
platform the CAD/CAM journals use). These run as **NXOpen journals** under `run_journal.exe`,
read the same `fea/fea_spec.json` hand-off, and score the same `acceptance_targets`.

```bat
:: each discipline takes its own .sim; spec/out are optional (defaults shown)
"%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_emag.py       -args motor_emag.sim   cogging,back_emf,torque_angle,ripple,demag
"%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_thermal.py    -args motor_thermal.sim losses=fea\simcenter_emag_results.json
"%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_structural.py -args motor_struct.sim
:: or all three in the EM->Thermal->Structural discipline order, one combined report:
"%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_run_all.py    -args emag=motor_emag.sim thermal=motor_thermal.sim structural=motor_struct.sim out=fea\simcenter_all_results.json
```

| Journal | Solver / solution | Reads gate |
|---|---|---|
| `simcenter_emag.py` | Magnetics, 2D Transient | peak torque, ripple ≤ 5 %, cogging ≤ 1 %, no demag @ peak+150 °C (the peak-efficiency gate is a follow-on from the loss field + torque-speed sweep, not auto-scored here) |
| `simcenter_thermal.py` | Thermal, steady state | continuous torque ≥ floor, magnet ≤ 150 °C, winding ≤ 180 °C |
| `simcenter_structural.py` | Nastran SOL 101 + SOL 103 | rotor von Mises SF ≥ 1.5 @ 1.2× max speed; 1st bending critical speed > 18000 rpm |

**Architecture** mirrors the project's *pure layer → thin emitter* split:

* **`simcenter_common.py`** — pure Python, **no NXOpen**: the spec loader, the guarded/logged
  `Journal` wrapper (writes to the NX Listing Window in-session, stdout otherwise; a drifted
  member name becomes a warning carrying the `TARGET` value, never an abort — same hardening as
  `nx_builder.py` / `motorcad_emag.py`), the `run_journal` argument convention, the three
  acceptance scorers, and math helpers. Runnable standalone (`python verification/simcenter_common.py`
  prints a self-check) so the gate logic is testable with no license.
* the three discipline journals + the `simcenter_run_all.py` coordinator — the thin emitters.

**Honest scope (read before the first run).** In Simcenter 3D the **FEM + mesh + idealization +
coil/collector setup is interactive geometry work** that does *not* survive journal replay (the
same caveat the plan documents for NX CAM). So each journal follows the **pre-build + drive +
score** pattern: you build the FEM+Sim **once** by hand (the journal prints a complete *setup
card* — every geometry/BC/material/limit value from `fea_spec` — when no usable `.sim` is open),
then re-run the journal on the saved `.sim` to drive the **scriptable, high-value** layer
(material property *values*, coil current / advance angle / speed / step schedule, loss loads,
convection BC, rotational-velocity load, solve, result read-back) and score go/no-go. Magnetics
has the thinnest NXOpen surface of the three, so the EM journal pairs with the
`femm_emag.py` / `motorcad_emag.py` drivers as the cross-check (the **G1 gate** wants them within
a few %); thermal and Nastran journal far more completely. The numbers are Simcenter's, not ours.
