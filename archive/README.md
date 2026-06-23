# archive/ — experimental / scratch files (kept on disk, not version-controlled)

Throwaway material from earlier sessions, parked here to keep the project root
clean. Ignored by git (only this `README.md` is tracked). **Nothing here is part of
the supported pipeline** — it is safe to delete.

| File | What it was |
|---|---|
| `_perf.py` | scratch performance-probe journal (superseded by `motor_nx/em_design.py` `estimate_performance`) |
| `_fea.py` | scratch FEA-probe journal (superseded by `motor_nx/fea.py`) |
| `_mc_probe.py` | scratch Motor-CAD API probe (superseded by `verification/motorcad_emag.py`) |
| `fem2.fem` | experimental Simcenter / NX FEM model (regenerate from `verification/simcenter_*.py`) |
| `sim1.sim` | experimental NX `.sim` simulation file |

The supported entry points live in [`motor_nx/`](../motor_nx/) (library + CLI),
[`verification/`](../verification/) (external-tool drivers) and
[`batch_build.py`](../batch_build.py).
