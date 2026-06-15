# Deep audit -- findings & resolution

A 6-dimension multi-agent audit (correctness, EM/physics, geometry, cross-artifact
consistency, code quality, missing analyses/production) raised **47 findings, all
47 adversarially confirmed**. Resolved across commits C1-C6 (2026-06-15).

- **Fixed: 41** | Deferred/acknowledged: 6
- Test suite after fixes: 43/43 (blueprint 8, manufacturing 7, drawings 7, analysis 8, em_design 9, fea 4).

## HIGH (4)

| id | file | finding | resolution |
|---|---|---|---|
| base-speed-voltage-limit | em_design.py | Inverter voltage ceiling Vdc/sqrt(3) is physically unachievable, corrupting base speed | Fixed |
| loss-breakdown-absent | em_design.py | Loss breakdown reduced to DC copper only -- no iron, magnet-eddy, or AC (skin/proximity) l | Fixed |
| thermal-continuous-rating-absent | em_design.py | No thermal model -> the 'continuous' rating is unbounded by temperature (purely current-de | Fixed |
| validate-missing-center-post-rib | em_design.py | No check that the two V-pocket arms keep a positive d-axis center rib -> pockets can overl | Fixed |

## MEDIUM (17)

| id | file | finding | resolution |
|---|---|---|---|
| cost-model-absent | manufacturing.py | No cost model -- BOM has masses but no material cost, especially the NdFeB/heavy-rare-eart | Fixed |
| dead-pitch-factor | em_design.py | Pitch factor kp is structurally pinned to 1.0 -- the coil-pitch machinery is dead code | Fixed |
| demag-margin-not-computed | fea.py | Demagnetisation margin is a target/plan, never a calculation -- all temp-coeff data is pre | Fixed |
| design-md-steel-grade-mismatch | docs/DESIGN.md | DESIGN.md names lamination grade 'M250-35A' while code and every other artifact say 'M250- | Fixed |
| dup-rotate-crosssection | preview.py | Build-step -> XY cross-section projection is copy-pasted across 3 modules (4 copies of _ro | Fixed |
| efficiency-map-absent | em_design.py | No efficiency map / torque-speed envelope -- only one operating point and a Cu-only effici | Deferred (FEA-level torque-speed envelope; first-order loss model now in place as prerequisite) |
| fea-zero-coverage | fea.py | Entire fea.py module is untested (winding map, FEA spec, DXF export) | Fixed |
| hcb-hardcoded-inconsistent | fea.py | Magnet Hcb is hardcoded (915 kA/m) and inconsistent with Br/mu_recoil; does not track magn | Fixed |
| magnet-seg-gap-mismatch | docs/MANUFACTURING.md | Axial magnet segment gap: code default 0.2 mm vs MANUFACTURING.md process spec 0.05-0.1 mm | Fixed |
| perf-zero-coverage | em_design.py | The first-order performance model (estimate_performance) has zero automated test coverage | Fixed |
| rotor-structural-absent | em_design.py | No rotor centrifugal/structural check -- bridge & center-post stress at speed never evalua | Fixed |
| slot-fill-assumption-vs-docs | em_design.py | Analytical/FEA slot-fill 0.62 disagrees with the design and manufacturing docs (0.65-0.75  | Fixed |
| tolerance-stackup-not-computed | manufacturing.py | Eccentricity tolerance stack-up is hand-asserted prose, not computed from the tolerance ta | Fixed |
| validate-magnet-outside-rounded-pocket | em_design.py | Magnet solid can poke outside the rounded pocket corners (interference with un-cut rotor s | Fixed |
| validate-missing-conductor-tangential-fit | em_design.py | validate() only checks conductor RADIAL fit, never the tangential bar width -> silent empt | Fixed |
| validate-shaft-bore-ge-diameter | em_design.py | Hollow-shaft bore >= shaft diameter is not validated and produces a self-intersecting revo | Fixed |
| validate-zero-conductors-paths | em_design.py | validate() misses conductors_per_slot==0 / parallel_paths==0; estimate_performance() then  | Fixed |

## LOW (26)

| id | file | finding | resolution |
|---|---|---|---|
| cogging-nvh-absent | em_design.py | No cogging/torque-ripple/NVH indicator -- not even the analytic LCM(slots,poles) cogging-o | Fixed |
| dc-bus-default-mismatch | em_design.py | Default DC-bus 350 V contradicts the documented 400-800 V architecture | Fixed |
| dead-import-preview | preview.py | Unused `blueprint as _bp` import in preview.py | Fixed |
| design-md-magnet-grade-mismatch | docs/DESIGN.md | DESIGN.md lists magnet grades N42UH/N45SH/N48SH; the actual default is N42SH (every other  | Fixed |
| design-md-slot-body-width-rounding | docs/DESIGN.md | DESIGN.md slot-body width '~3.9 mm' vs derived 3.88 (and TOLERANCES/MANUFACTURING use 3.88 | Fixed |
| efficiency-cu-only-no-iron-ac | em_design.py | Efficiency uses DC copper loss only; AC copper, iron and magnet losses omitted (labeled bu | Fixed |
| estimate-perf-zero-flux-basespeed | em_design.py | base_speed divides by back-EMF with no zero guard (ZeroDivisionError when Bg1=0) | Fixed |
| expr-unit-magnets-per-pole | params.py | Dimensionless count rotor_magnets_per_pole is exported to NX with unit 'mm' | Fixed |
| inconsistent-circle-tessellation | preview.py | Three different circle representations across renderers/builder (96-gon, 48-gon, exact arc | Acknowledged by design (preview tessellates for fill; fea/drawings emit true circles) |
| magnet-pocket-datum-label | manufacturing.py | Magnet-pocket tolerances use datum B = 'rotor bore', but the documented datum strategy mak | Fixed |
| magnet-tempco-linear-no-knee | fea.py | Magnet temperature handling is a single linear tempco with no knee/irreversible-loss model | Deferred to FEA (demag knee); analysis.demag_margin derates Br/Hcj linearly as a first-order bound |
| manufacturing-bymaterial-deadcode | manufacturing.py | by_material aggregation is computed but never returned or used | Fixed |
| no-error-handling-renderers | preview.py | Cross-section renderers assume well-formed steps and crash on missing keys; no malformed-b | Acknowledged (blueprint is the sole producer of well-formed steps) |
| obfuscated-anchor-expr | blueprint.py | Obfuscated `r_anchor ** 2 * 0.0 + 1.0` floor instead of a plain literal | Fixed |
| params-from_dict-dead-isdataclass | params.py | is_dataclass(f.type) branch is dead code under `from __future__ import annotations` | Fixed |
| pitch-factor-hardcoded-full | em_design.py | Pitch factor is hardcoded to 1.0 (full-pitch) so chorded hairpin windings are not modeled | Fixed |
| results-violate-doc-bands | em_design.py | Code's own default outputs fall outside the documented target bands (peak torque, base spe | Fixed |
| robust-fillet-acknowledge | blueprint.py | Fillet clamping is robust against self-intersection but silently drops oversized fillets w | By design (fillet clamps to avoid self-intersection; oversized radii left sharp -- safe) |
| rotor-drawing-shaft-bore-label | drawings.py | Rotor sheet dimensions the rotor-shaft fit (45 mm) but labels it 'SHAFT BORE', the term us | Fixed |
| slot-fill-vs-geometry | em_design.py | Assumed slot_fill 0.62 disagrees with both the actual bar geometry (0.595) and the doc cla | Fixed |
| slot-mouth-margin-vs-air-gap | blueprint.py | Hard-coded 0.5 mm slot-mouth over-cut into the bore is not validated against air_gap / ope | Acknowledged minor (0.5 mm mouth over-cut is into air-gap air; harmless) |
| v-inner-center-silent-clamp | blueprint.py | Degenerate magnet vertex anchor silently clamps to x=1.0 via an obfuscated constant | Fixed |
| v-inner-end-center-sqrt-fallback | blueprint.py | Magnet inner-anchor uses an obfuscated sqrt fallback that silently yields garbage geometry | Fixed |
| validate-inverted-housing-tube | em_design.py | Zero/negative jacket_thickness yields an inverted housing tube that validate() only catche | Fixed |
| validate-missing-pocket-clearance-sign | em_design.py | Negative pocket_clearance / end_barrier makes the pocket smaller than the magnet -> magnet | Fixed |
| validate-msg-wrong-param | em_design.py | Validation error message cites a non-existent parameter 'magnet_tilt_deg' | Fixed |
