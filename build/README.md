# build/ — generated CAD output (regenerable, not version-controlled)

Everything here is **produced by a tool and can be regenerated** — it is ignored by
git (only this `README.md` is tracked).

| Path | Produced by | What it is |
|---|---|---|
| `build/<name>.prt`, `<name>_ap242.stp`, `<name>.x_t`, `<name>.log`, `manifest.json` | `python batch_build.py configs/<cfg>.json` | one folder of NX part + STEP/Parasolid export + build log per valid variant |
| `build/legacy/` | earlier NX sessions | historical `.prt/.stp/.x_t/.log` from previous iterations (motor_v*, nx_smoke*, motor_demo*, …); kept for reference, safe to delete |

Regenerate the active build at any time:

```bash
python batch_build.py configs/default.json          # single motor
python batch_build.py configs/sweep_example.json     # parameter sweep
python batch_build.py configs/sweep_example.json --dry-run   # no NX needed
```

Nothing here is an input — delete the whole folder and the next build re-creates it.
