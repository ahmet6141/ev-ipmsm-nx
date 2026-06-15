# verification/ — external-tool drivers (P1 EM-FEA, P6 drawings)

These scripts execute the project's verification phases in the licensed tools the
`motor_nx` package hands off to. They are the runnable counterparts of the plan in
[`docs/PROJECT_PLAN.md`](../docs/PROJECT_PLAN.md): `motor_nx` produces the design +
hand-off package; these drivers push it into Motor-CAD / NX and read results back.

| Script | Phase | Tool | Run with |
|---|---|---|---|
| `motorcad_emag.py` | **P1** EM-FEA (+ P2/P3 seeds) | Ansys Motor-CAD (PyMotorCAD) | plain `python` |
| `nx_drafting.py` | **P6** formal 2D drawings | Siemens NX (NXOpen journal) | `run_journal.exe` |

The numeric answers come from Motor-CAD / NX. The drivers only map the frozen design
onto tool inputs, run the matrix headless, and report **go / no-go against
`fea_spec.acceptance_targets`**. They do not compute physics themselves.

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
verification/motorcad_emag.py        (FEMM cross-check — femm_labels.csv)
   → fea/motorcad_results.json
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

## Free cross-check (FEMM)

`fea/femm_labels.csv` is a `(x, y, material, circuit, turns, magdir)` recipe: import
`fea/cross_section.dxf` into FEMM, drop a block label at each row, set a 1-pole
anti-periodic boundary with A = 0 on the OD, and solve. The plan's G1 gate requires
Motor-CAD average torque within ~3–5 % of the FEMM cross-check.
```
