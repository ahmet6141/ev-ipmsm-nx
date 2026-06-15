# Uçtan uca yürütme planı — FEA → iterasyon → imalat

Bu plan **cowork** (çok-ajanlı workflow) ile, mevcut `motor_nx` artefakt zincirine
dayanarak üretildi: Ansys Motor-CAD (FEA hub) + Siemens NX 2506 CAM (talaşlı parçalar).
Ajanlar Motor-CAD/NX'i çalıştırmaz; bu plan + betik iskeletleri + araştırmadır — çözüm/
toolpath/test senin tarafında koşar. Kabul kapıları `fea/fea_spec.json → acceptance_targets`.

## Genel strateji

A stage-gated V-cycle that reuses the motor_nx artifact chain end-to-end. Discipline order is load-bearing: EM FEA runs FIRST in Ansys Motor-CAD (it sets the loss field everything else consumes), THEN Thermal (consumes EM losses, sets the thermally-limited continuous rating that analysis.py already flags at J_cont~9.8), THEN Structural rotor stress (can veto thin 1.0 mm bridges), all wrapped in a coupled PyMotorCAD iterate-to-acceptance loop because hot magnets derate Br (-0.12%/C) and copper resistivity rises, changing EM. FEMM/pyFEMM (driven by fea/femm_labels.csv) is the free EM cross-check; analysis.py first-order numbers (base ~3480 rpm, peak ~473 Nm vs the 412 Nm acceptance floor) are the sanity gate. Every discipline gate reads directly from fea/fea_spec.json acceptance_targets. Nothing changes geometry except params.py: cowork edits params/JSON overrides and regenerates fea_spec.json/cross_section.dxf/winding.csv/femm_labels.csv/BOM/tolerances/drawings/per-part STEP via the cli; the user runs Motor-CAD and NX interactively. Only AFTER all gates pass simultaneously (Design Freeze) is tooling money spent: laminations -> stamping/laser (NOT CAM), magnets -> N42SH supplier (4 axial segments, shipped unmagnetized), hairpins -> wire-forming supplier, and ONLY the hollow shaft (turning) + Al housing (milling) go through NX 2506 CAM driven by the nx_builder.py `parts` STEP exports. Build -> in-place magnetize -> balance -> dyno correlates measured performance back against the same fea_spec targets for release.

## Faz haritası

| Faz | Başlık | Araç | Kabul kapısı |
|---|---|---|---|
| P0 | Concept/sizing freeze + pre-FEA triage | motor_nx CLI (NX-independent Python) | 3 kriter |
| P1 | EM FEA verification in Motor-CAD (sets the loss field) + FEMM cross-check | Ansys Motor-CAD E-Magnetic (PyMotorCAD); FEMM/pyFEMM as free cross-check | 6 kriter |
| P2 | Thermal verification (continuous rating) in Motor-CAD | Ansys Motor-CAD Thermal (steady-state, EMag-linked); analysis.py lumped seed | 3 kriter |
| P3 | Structural rotor stress (+ rotordynamics) verification | Motor-CAD Mechanical (2D rotor stress) for SF; NX/Ansys Mechanical for rotordynamics | 3 kriter |
| P4 | Coupled iterate-to-acceptance loop | motor_nx CLI (geometry) + PyMotorCAD sweep/Sensitivity (physics) | 2 kriter |
| P5 | NVH check | Motor-CAD (radial force harmonics) + NX/Ansys modal (housing/stator) | 1 kriter |
| P6 | DFM + tolerance freeze = DESIGN FREEZE gate | motor_nx CLI (bom/tolerances/drawings) + DFM review | 3 kriter |
| P7 | Tooling + supply long-leads (non-CAM split) + per-part STEP export | motor_nx nx_builder.py (parts STEP export) + suppliers; NX is only the STEP source here | 3 kriter |
| P8a | Prototype build + winding/insulation tests | Shop/assembly + EOL-class electrical test gear (no Motor-CAD/NX) | 2 kriter |
| P8b | NX CAM machining: SHAFT (turning) + HOUSING (milling) | Siemens NX 2506 CAM (turning + mill_planar/mill_contour); NXOpen CAM journal for pre-build | 3 kriter |
| P9 | In-place magnetize + dynamic balance | Magnetizing pulse fixture + two-plane balance machine (no Motor-CAD/NX) | 2 kriter |
| P10 | Final assembly + dyno characterization + EOL test = RELEASE | Dyno + EOL test stand (no Motor-CAD/NX; correlate to fea_spec) | 3 kriter |

---

## Fazlar (detay)

### P0 — Concept/sizing freeze + pre-FEA triage

- **Hedef:** Confirm the default variant is buildable and meets first-order targets with margin, and identify the three suspect points FEA must check first (thermally-limited continuous rating, worst-case demag, rotor bridge SF).
- **Araç:** motor_nx CLI (NX-independent Python)
- **Otomasyon (cowork/script vs elle):** FULLY cowork/script-automatable; no Motor-CAD/NX. Cowork runs the CLI and parses outputs. User only reviews the go/no-go.
- **Girdiler:** motor_nx/params.py default MotorParams (54/6, OD225/bore161/gap0.70/stack134, single-V N42SH 4-seg, 8 hairpin bars/slot, hollow 45 mm shaft); configs/default.json
- **Görevler:**
  1. Run `python -m motor_nx.cli validate` (must exit 0 = no geometrically impossible combo) and `python -m motor_nx.cli report` (em_design validate + first-order performance: base ~3480 rpm SVPWM Vdc/sqrt6, peak ~473 Nm / 172 kW).
  2. Run `python -m motor_nx.cli analysis` and record the three flagged risks: thermal_rating (J_cont~9.8 A/mm2, thermally_limited=True), demag_margin (worst-case conservative flag), rotor_stress (centrifugal SF at 1.2x max speed).
  3. Run `python -m motor_nx.cli bom` and `python -m motor_nx.cli tolerances` to baseline mass (~40.6 kg, magnet ~43% of cost) and the eccentricity stack-up (RSS vs 0.07 mm budget).
  4. Freeze requirements: peak 473 Nm/172 kW, base ~3480 rpm, max 18000 rpm, DC bus 400 V; these become the Motor-CAD operating-point inputs in P1.
- **Çıktılar:** report.txt / analysis.txt / bom.csv / tolerances.csv (cowork-captured); A one-page triage note listing the 3 suspect points as the FEA starting priorities
- **KABUL KAPISI:**
  - validate exits 0 (buildable)
  - first-order peak torque > 412 Nm acceptance floor WITH margin (analysis shows ~473)
  - analysis.py runs clean and the 3 risk flags are documented (these are EXPECTED to be flagged, not blockers)

### P1 — EM FEA verification in Motor-CAD (sets the loss field) + FEMM cross-check

- **Hedef:** Verify the EM acceptance gate: peak torque, back-EMF/Vdc headroom, MTPA, torque ripple, cogging, demag, and the loss field (iron + magnet-eddy + AC copper) that Thermal will consume.
- **Araç:** Ansys Motor-CAD E-Magnetic (PyMotorCAD); FEMM/pyFEMM as free cross-check
- **Otomasyon (cowork/script vs elle):** Cowork PRE-BUILDS: (a) the PyMotorCAD build script that maps fea_spec.json geometry_mm/materials/winding into set_variable/set_array_variable calls, builds the single-V BPM template, sets MagneticWindingType=Hairpin (8 cond/slot, 2 parallel paths, coil pitch 9, q=3) and VERIFIES the auto pattern against winding.csv slot_phase_map + kw~0.9598; (b) the pyFEMM cross-check script that imports cross_section.dxf and applies the femm_labels.csv block-label recipe (interior point + material + circuit/turns/sign + magnetization angle); (c) a geom(params)->metrics loop skeleton reading acceptance_targets. USER runs both solves interactively (Motor-CAD GUI/headless, confirms exact variable-name spellings for NX2506-era build, watches mesh/convergence), and runs FEMM.
- **Girdiler:** fea/fea_spec.json (geometry_mm, materials.lamination 13-pt BH + note to swap for vendor M250-27 datasheet, materials.magnet N42SH Br1.28/Hcj1592 + tempco -0.12/-0.55, symmetry_and_bc 1-pole 60deg anti-periodic + A=0 on OD, excitation beta sweep 0..45 deg, mesh: 3 airgap layers + refine bridge/post/tooth-tip/magnet-corner, analyses[], acceptance_targets); fea/cross_section.dxf (material-layered 2D plane; pre-orient to origin/+X d-axis if used via Adaptive Templates 2024R2+); fea/winding.csv + fea_spec.winding.slot_phase_map (exact 54-slot A/B/C +/- belt); fea/femm_labels.csv (FEMM block-label recipe); analysis.py loss_breakdown (AC F_R proximity seed) as a magnitude sanity check
- **Görevler:**
  1. Build the parametric single-V IPM model in Motor-CAD from fea_spec.geometry_mm (Slot_Number=54, Pole_Number=6, Stator_Lam_Dia=225, Stator_Bore=161, Airgap=0.70, stack 134 x 0.96, Shaft_Dia=45, Tooth_Width=5.6, back-iron 13.0; tune V-angle/magnet width to land rotor_OD 159.6, outer_bridge 1.0, center-post halfwidth 1.0). Use cross_section.dxf via Adaptive Templates ONLY if the template V-pocket cannot match.
  2. Define materials: custom NO-steel from the 13-pt BH (swap to vendor datasheet + Bertotti/Steinmetz coeffs before the FINAL qualifying run), N42SH with the temp coeffs and axial_segments=4, copper with tempco.
  3. Set the hairpin winding and VERIFY Motor-CAD's auto wave pattern equals winding.csv/slot_phase_map and kw~0.9598; force with set_winding_coil if it differs (auto belt/parallel-path transposition may not match the FEMM/Maxwell map).
  4. Run the EM matrix from fea_spec.analyses: cogging (no current, fine steps over 1 slot pitch), back-EMF/THD vs Vdc at base speed, Ld/Lq + saliency, torque-vs-beta MTPA sweep at rated 126 A_rms and peak 273 A_rms, torque ripple at MTPA, losses (iron + magnet-eddy with 4-seg + AC copper skin/proximity in the 8-bar stack -- NOT DC resistance), efficiency map.
  5. Demag: set magnet 150 C, apply worst-case d-axis-opposing peak current at the demag-worst beta, read min magnet B vs the hot 2nd-quadrant knee / percent demagnetized.
  6. Cross-check the winning point in FEMM (1-pole anti-periodic, A=0) via femm_labels.csv; require Motor-CAD torque within a few % of FEMM.
  7. EXPORT the EM loss field (copper/iron/magnet per region) for the P2 thermal hand-off.
- **Çıktılar:** Solved .mot with EM results; torque-angle/MTPA locus, ripple %, cogging %, back-EMF THD, Ld/Lq, efficiency map; EM loss field (copper+iron+magnet) for P2; FEMM cross-check report (torque delta vs Motor-CAD); demag result (min magnet B vs knee @ peak+150C)
- **KABUL KAPISI:**
  - peak_torque_Nm_min >= 412 (fea_spec acceptance_targets)
  - torque_ripple_pct_max <= 5.0
  - cogging_pct_of_rated_max <= 1.0
  - no_demag at peak current AND 150 C magnet (min B above the hot knee)
  - peak_efficiency_pct_min >= 95.0
  - Motor-CAD vs FEMM average torque agree within ~3-5%

### P2 — Thermal verification (continuous rating) in Motor-CAD

- **Hedef:** Find the thermally-limited continuous torque from the P1 loss field through the water-jacket housing model and confirm magnet hotspot stays under 150 C.
- **Araç:** Ansys Motor-CAD Thermal (steady-state, EMag-linked); analysis.py lumped seed
- **Otomasyon (cowork/script vs elle):** Cowork PRE-BUILDS the PyMotorCAD thermal script (show_thermal_context, water-jacket vars HousingWJFluid/flow rate/inlet temp, EMag->Thermal loss coupling, do_steady_state_analysis, read T_[Winding_Max] + magnet hotspot node, and the current-reduction loop to hit the limit). USER runs the solve, supplies the real coolant flow/inlet temp and jacket h-correlation for the 12-channel Al jacket, watches convergence.
- **Girdiler:** P1 EM loss field (copper + iron + magnet-eddy); fea_spec.materials.conductor (Class H 180 C limit) and magnet max_service 150 C; analysis.thermal_rating seed (J_cont~9.8 A/mm2 thermally limited; coolant ~65 C; effective slot k_eff) as the starting current; CoolingParams from params.py (12 axial channels, 6 mm jacket) for the jacket model
- **Görevler:**
  1. Switch to Thermal context, select the water-jacket housing cooling model, set coolant fluid / volume flow / inlet temperature and the h-correlation inputs.
  2. Feed the P1 EM losses by EMag->Thermal coupling (or imposed losses); solve steady state.
  3. Read winding max temp vs 180 C Class H and the magnet hotspot node vs 150 C.
  4. Iterate current DOWN until the binding hotspot (winding 180 C or magnet 150 C) is reached; that current/torque is the continuous rating. Re-run P1 EM at that current if Br/copper derating shifts torque materially (coupling).
- **Çıktılar:** Continuous torque-speed rating point; Winding + magnet steady-state hotspot temps; Coupled (hot) Br/resistivity feedback note for P4
- **KABUL KAPISI:**
  - continuous_torque_Nm_min >= 190 (fea_spec acceptance_targets)
  - magnet_temp_C_max_continuous <= 150.0 at the continuous point
  - winding hotspot <= 180 C (Class H)

### P3 — Structural rotor stress (+ rotordynamics) verification

- **Hedef:** Confirm the 1.0 mm outer bridges and 1.0 mm-halfwidth center post survive centrifugal load at 1.2x max speed, and the shaft-rotor first bending critical speed clears max speed.
- **Araç:** Motor-CAD Mechanical (2D rotor stress) for SF; NX/Ansys Mechanical for rotordynamics
- **Otomasyon (cowork/script vs elle):** Cowork PRE-BUILDS the PyMotorCAD mechanical script (set overspeed = 1.2 x 18000 = 21600 rpm, do_mechanical_calculation, read MaxStress_RotorLam + YieldStress_RotorLam, compute SF in bridges/post). USER runs it AND runs the separate rotordynamics check in NX/Ansys with the hollow 45 mm shaft + rotor mass (Motor-CAD's 2D solver does NOT do shaft bending).
- **Girdiler:** The P1/P2 converged rotor geometry (outer_bridge 1.0, center_post_halfwidth 1.0, magnet mass per pole from BOM); fea_spec.analyses rotor_stress + rotordynamics; analysis.rotor_stress first-order seed (bridge sigma / SF at 1.2x); Shaft STEP (Shaft per-part) + rotor lamination mass for the NX/Ansys rotordynamics model
- **Görevler:**
  1. Run the Motor-CAD 2D rotor-stress solve at 21600 rpm; read peak von Mises in the outer bridges and center post; SF = rotor-steel yield / max stress.
  2. Check magnet retention by the bridges at 18000 rpm.
  3. In NX/Ansys Mechanical: build the hollow-shaft + rotor-mass beam/3D model, solve the first bending critical speed, confirm margin above 18000 rpm.
  4. If SF < 1.5 OR critical-speed margin is thin, flag the bridge/post (or shaft) for the P4 loop -- do NOT pass.
- **Çıktılar:** Rotor von Mises SF in bridges + center post at 1.2x max speed; Magnet retention verdict at max speed; First bending critical speed + margin
- **KABUL KAPISI:**
  - rotor_vonMises_safety_factor_min_at_1.2x_maxspeed >= 1.5 (fea_spec acceptance_targets)
  - magnets retained at 18000 rpm
  - first bending critical speed > 18000 rpm with margin

### P4 — Coupled iterate-to-acceptance loop

- **Hedef:** Resolve any failed gate by editing params.py, regenerating artifacts, and re-solving until ALL acceptance_targets pass simultaneously.
- **Araç:** motor_nx CLI (geometry) + PyMotorCAD sweep/Sensitivity (physics)
- **Otomasyon (cowork/script vs elle):** Cowork AUTOMATES the geometry side: edit params.py / JSON override, run `cli validate`->`cli fea`->`cli analysis`->`cli bom/tolerances/drawings`, and batch_build.py to sweep variants + emit a manifest; cowork wraps P1-P3 as geom(params)->metrics comparing to acceptance_targets. USER runs each Motor-CAD re-solve (and the NX rotordynamics re-check when shaft/rotor mass changes) and judges convergence.
- **Girdiler:** fea_spec.acceptance_targets (the loop exit criteria, all 8); Failing metric(s) from P1/P2/P3; params.py knobs: rotor.outer_bridge, center_post_halfwidth, magnet_width/thickness, v_angle_deg, winding turns/parallel paths; configs/sweep_example.json
- **Görevler:**
  1. Map each failure to the right knob and loop-back: ripple>5% or cogging>1% -> shape pole-arc/V-angle/notch or add skew, re-run P1. peak torque<412 -> more magnet/current or thinner bridge, re-run P1 (then RE-CHECK P3). continuous<190 or magnet>150C -> lower current / improve jacket / more axial magnet segments, re-run P2. demag fail -> thicker magnet or reduce worst-case d-axis current, re-run P1 demag. rotor SF<1.5 -> thicken bridge/post or higher-strength steel, re-run P3 (then RE-CHECK P1 leakage/torque). eff<95% -> reduce AC copper loss (bar subdivision) / iron loss, re-run P1.
  2. Beware the three-way bridge trap: thinner bridge raises torque + cuts leakage BUT raises ripple (bridge saturation) AND cuts rotor SF -- never optimize torque without re-checking ripple (P1) and SF (P3) in the SAME iteration.
  3. Couple thermal<->EM: apply hot Br derate (-0.12%/C) and copper resistivity rise from P2 temps back into P1 until temperatures and torque converge.
  4. Each accepted iteration: regenerate fea_spec.json + cross_section.dxf + winding.csv + femm_labels.csv from the new params so downstream artifacts stay in lockstep; mc.save_to_file() winners (wrap solves in try/except so a crash doesn't lose the converged geometry).
- **Çıktılar:** The winning parameter set in params.py / JSON; Regenerated fea hand-off package matching the winner; Iteration log mapping each change to the gate it fixed
- **KABUL KAPISI:**
  - ALL fea_spec acceptance_targets pass SIMULTANEOUSLY on one geometry: peak>=412, continuous>=190, ripple<=5%, cogging<=1%, no demag@peak+150C, magnet<=150C, rotor SF>=1.5@1.2x, peak eff>=95%
  - Motor-CAD winner cross-checked against FEMM and within a few % of analysis.py first-order peak (~473 Nm) -- no unexplained divergence

### P5 — NVH check

- **Hedef:** Confirm no EM radial-force excitation order coincides with a stator/housing structural mode across 0-18000 rpm.
- **Araç:** Motor-CAD (radial force harmonics) + NX/Ansys modal (housing/stator)
- **Otomasyon (cowork/script vs elle):** Cowork PRE-BUILDS the PyMotorCAD force-harmonic extraction script and the NX modal-setup notes. USER runs the EM force extraction and the structural modal solve and reads the Campbell/coincidence map.
- **Girdiler:** P4 winning EM model (air-gap radial force harmonics); Stator + housing geometry (STEP from nx_builder, Al housing + 12-channel jacket); analysis.cogging_index (LCM cogging cycles/rev, GCD note) as the order seed
- **Görevler:**
  1. Extract air-gap radial force spatial/temporal harmonics from the winning EM solve.
  2. Build a stator/housing modal model (NX/Ansys), get natural frequencies.
  3. Map excitation orders vs natural frequencies across the speed range (Campbell); check for in-band coincidence.
  4. If a resonance lands in-band: add skew (~1 slot pitch), pole-arc/notch shaping, or revisit slot/pole -- loop back to P1/P3 (geometry change re-opens those gates).
- **Çıktılar:** EM force-harmonic spectrum; Stator/housing modal frequencies + Campbell coincidence map; NVH verdict (pass or geometry change feeding P1/P3)
- **KABUL KAPISI:**
  - No dominant EM excitation order coincides with a structural natural frequency within the 0-18000 rpm operating band (with margin)

### P6 — DFM + tolerance freeze = DESIGN FREEZE gate

- **Hedef:** Lock the manufacturing package; confirm GD&T is within process capability and the eccentricity stack-up protects the 0.70 mm air gap. After this, NO design change without re-running P1-P5.
- **Araç:** motor_nx CLI (bom/tolerances/drawings) + DFM review
- **Otomasyon (cowork/script vs elle):** Cowork FULLY generates the package: `cli bom --csv`, `cli tolerances --csv`, `cli drawings -o drawings/` (2D DXF+SVG assembly/stator/rotor). Cowork also runs eccentricity_stackup and checks RSS<=budget. USER + manufacturing engineers do the DFM/DFMEA/PFMEA sign-off review.
- **Girdiler:** P4/P5 frozen params.py; manufacturing.TOLERANCES (12-feature GD&T table, datum A = bearing axis); manufacturing.eccentricity_stackup (RSS of bore/rotor-OD/coaxiality vs 0.07 mm budget); drawings.py output (dimensioned DXF+SVG); BOM (~40.6 kg, magnet ~43% cost)
- **Görevler:**
  1. Generate BOM, tolerance table, and 2D drawings from the frozen params.
  2. Review every GD&T callout against proven stamping/turning/milling capability -- do NOT spec tighter than process can hold (e.g. die +/-0.02 mm, journals k5, Ra<=0.4 ground).
  3. Confirm the air gap is held by the eccentricity STACK-UP (RSS <= 0.07 mm, ~10% of 0.70 mm) and the bearing-axis datum strategy -- the gap is NOT directly toleranced.
  4. Complete DFMEA/PFMEA + control-plan maturity; sign off Design Freeze.
- **Çıktılar:** Frozen bom.csv, tolerances.csv, drawings/ (DXF+SVG); Eccentricity stack-up pass (RSS <= 0.07 mm); Signed DFMEA/PFMEA + control plan
- **KABUL KAPISI:**
  - No open EM/thermal/structural/NVH gate from P1-P5
  - Eccentricity RSS <= 0.07 mm budget (manufacturing.eccentricity_stackup rss_pass=True)
  - DFM signed off; drawings + BOM + tolerances frozen (change control engaged)

### P7 — Tooling + supply long-leads (non-CAM split) + per-part STEP export

- **Hedef:** Release the four NON-CAM supply chains and export the two CAM-bound solids, in parallel, to compress schedule.
- **Araç:** motor_nx nx_builder.py (parts STEP export) + suppliers; NX is only the STEP source here
- **Otomasyon (cowork/script vs elle):** Cowork generates the NXOpen build+export invocation and the supplier data packs (DXF, magnet spec, hairpin geometry). USER runs nx_builder under run_journal.exe and places supplier orders.
- **Girdiler:** fea/cross_section.dxf (lamination outline for stamping die / laser); fea_spec.materials.magnet (N42SH, 4 axial segments, Br/Hcj/tempco) for the magnet PO; WindingParams (8 bars/slot, bar geometry) for the hairpin wire-former; nx_builder.py with the `parts` arg -> per-component STEP via _component_of()
- **Görevler:**
  1. Run `"%UGII_ROOT_DIR%\run_journal.exe" motor_nx/nx_builder.py -args <blueprint.json> <out>.prt both parts` -> writes per-part STEP: <base>_Shaft.stp and <base>_Housing.stp (the ONLY CAM inputs), plus _Stator_Lamination/_Rotor_Lamination/_Magnets/_Winding for supplier reference.
  2. LAMINATIONS (NOT CAM): release cross_section.dxf to a progressive stamping die (6-8 wk lead) AND order prototype laser-cut sheets (~5 days) in parallel to de-risk while the die is cut. Stress-relief anneal only BEFORE bonding.
  3. MAGNETS (supplier): order N42SH in 4 axial segments, UNMAGNETIZED (consider GBD to cut heavy-rare-earth).
  4. HAIRPINS (wire-form supplier): release the 8-bar/slot geometry for flatten/cut/strip/bend/insert/twist/weld setup.
  5. SHAFT + HOUSING -> queue the per-part STEP for NX CAM in P8b.
- **Çıktılar:** Stamping die in work + prototype laser laminations; N42SH magnet PO (4-seg, unmagnetized); Hairpin wire-form setup; <base>_Shaft.stp and <base>_Housing.stp ready for CAM
- **KABUL KAPISI:**
  - All four non-CAM supply chains released against the FROZEN package
  - Shaft + Housing STEP exported and dimensionally verified against drawings
  - Prototype laminations in hand for early stator-stack build

### P8a — Prototype build + winding/insulation tests

- **Hedef:** Build the wound stator and rotor pack per the assembly sequence and screen the winding insulation BEFORE further value-add.
- **Araç:** Shop/assembly + EOL-class electrical test gear (no Motor-CAD/NX)
- **Otomasyon (cowork/script vs elle):** Cowork provides the step-by-step assembly checklist + test acceptance limits from manufacturing.general_notes and TOLERANCES. USER/shop executes physically.
- **Girdiler:** Prototype laminations, hairpins, magnets, shaft, housing; manufacturing.general_notes (backlack bond, anneal-before-bond, finish-machine bore/OD AFTER stacking, slot liner Nomex-Kapton-Nomex); TOLERANCES (bore 161 H7, OD 225 shrink fit, journal k5, Ra targets)
- **Görevler:**
  1. Press/bond stator stack (backlack); finish-machine bore 161 and OD 225 AFTER stacking.
  2. Insert slot liner; form/insert/twist the 8 hairpins per slot; laser/TIG weld crowns with 100% weld inspection.
  3. VPI impregnate (Class H).
  4. On the WOUND STATOR before any further assembly: IR, hi-pot (phase-ground + phase-phase), turn-turn surge, phase-resistance balance.
- **Çıktılar:** Wound, impregnated, weld-inspected stator; Rotor lamination pack ready for magnet bonding
- **KABUL KAPISI:**
  - IR / hi-pot / surge / phase-resistance-balance all pass on the wound stator (GATE 8a) before further value-add
  - 100% hairpin weld inspection pass

### P8b — NX CAM machining: SHAFT (turning) + HOUSING (milling)

- **Hedef:** Machine the only two CAM parts to the GD&T in manufacturing.TOLERANCES, holding bearing-bore concentricity to the stator-bore axis (air-gap eccentricity contributor).
- **Araç:** Siemens NX 2506 CAM (turning + mill_planar/mill_contour); NXOpen CAM journal for pre-build
- **Otomasyon (cowork/script vs elle):** Cowork ships cam_prebuild.py (same run_journal harness as nx_builder): given the imported part, it creates the CAM setup + MCS + WORKPIECE + PROGRAM/METHOD groups + TOOLs + near-complete OPERATION shells with feeds/speeds/stepover/DOC/finish-stock PRESET, then STOPS. USER attaches geometry collectors (faces/boundaries/blank -- NOT journal-replayable), runs GENERATE -> VERIFY (3D IPW) -> gouge/collision check -> Postprocess with the machine-specific post.
- **Girdiler:** <base>_Shaft.stp and <base>_Housing.stp (from P7); manufacturing.TOLERANCES: journals k5 (+0.013/+0.002) Ra<=0.4; bearing-bore H6/J6/K6; rotor press-seat; bore/bearing-bore concentricity to stator-bore datum; Ã˜14 hollow bore; eccentricity budget
- **Görevler:**
  1. SHAFT: import solid, Manufacturing cam_general + turning template; MCS_SPINDLE on +Z; TURNING_WORKPIECE PART=shaft, BLANK=pre-bored tube or bar+gun-drill (account for Ã˜14 hollow bore in IPW). Ops: FACE -> ROUGH_TURN_OD -> CENTERLINE_DRILL (Ã˜14) -> GROOVE_OD (shoulder relief) -> FINISH_TURN_OD leaving ~0.1-0.2 mm radial GRIND stock on k5 journals (final k5 + Ra<=0.4 is GRINDING, not turning). Generate->Verify(IPW)->gouge check->post.
  2. HOUSING: import solid, Manufacturing cam_general + mill_planar/mill_contour; set MCS on the stator-bore/register axis so BOTH bearing bores come out concentric to the stator-bore datum; WORKPIECE PART=housing, BLANK=as-cast body (From Part/IPW, NOT a prismatic billet). Ops: FACE_MILLING (bearing faces, perpendicularity) + bore-finish the DE/NDE bearing seats (mill_contour finish / circular-helical bore or bore cycle) to H6/J6/K6. Generate->Verify(IPW)->gouge check->post.
  3. Use FindObject defensively for version-localized group names (MCS vs MCS_SPINDLE, WORKPIECE vs TURNING_WORKPIECE), mirroring the version-drift hardening already in nx_builder.py.
  4. Hand off journals to a cylindrical grinder for the final k5 journal size/finish (often non-NX).
- **Çıktılar:** Verified G-code for shaft turning + housing milling; Machined hollow shaft (with grind stock) and bored housing; Cylindrical-grind step queued for the k5 journals
- **KABUL KAPISI:**
  - IPW VERIFY clean + gouge/collision check pass on both parts
  - Housing bearing bores concentric to the stator-bore datum within the GD&T (protects the eccentricity budget)
  - Shaft features to print with correct grind stock on k5 journals; journals ground to k5 + Ra<=0.4

### P9 — In-place magnetize + dynamic balance

- **Hedef:** Assemble the rotor with UNMAGNETIZED magnets, magnetize in place, verify flux vs FEA, and balance.
- **Araç:** Magnetizing pulse fixture + two-plane balance machine (no Motor-CAD/NX)
- **Otomasyon (cowork/script vs elle):** Cowork provides the magnetize/balance procedure + back-EMF acceptance band from the P1 FEA prediction. USER/shop executes.
- **Girdiler:** Rotor pack + 4-axial-segment unmagnetized N42SH + machined+ground shaft; P1 back-EMF / surface-flux FEA prediction; TOLERANCES rotor balance (ISO 21940-11 G2.5, design target G1.0)
- **Görevler:**
  1. Bond the 4 unmagnetized axial magnet segments into the V pockets with high-temp epoxy; cure. NEVER glue pre-magnetized blocks (cracking/FOD/safety).
  2. Press/shrink the rotor onto the hollow 45 mm shaft (machine OD after pressing if required).
  3. Magnetize IN-PLACE on the assembled rotor with the pulse fixture.
  4. Verify back-EMF / surface-flux map vs the P1 FEA prediction.
  5. Two-plane dynamic balance to ISO 21940-11 G2.5 (target G1.0 for NVH).
- **Çıktılar:** Magnetized, balanced rotor assembly; Back-EMF / flux-map correlation record vs FEA
- **KABUL KAPISI:**
  - Measured back-EMF / surface-flux within tolerance of the P1 FEA prediction
  - Balance meets G2.5 (target G1.0) at 18000 rpm

### P10 — Final assembly + dyno characterization + EOL test = RELEASE

- **Hedef:** Assemble the motor, characterize on the dyno against FEA acceptance_targets, and stand up per-unit EOL screening for production release.
- **Araç:** Dyno + EOL test stand (no Motor-CAD/NX; correlate to fea_spec)
- **Otomasyon (cowork/script vs elle):** Cowork builds the dyno test matrix + correlation template mapping measured vs fea_spec acceptance_targets, and the EOL per-unit test spec. USER runs dyno + EOL.
- **Girdiler:** Machined+ground shaft, bored housing, wound stator, magnetized/balanced rotor; fea_spec.acceptance_targets (the correlation basis) + P1 efficiency map / MTPA; manufacturing.general_notes EOL list (back-EMF symmetry, cogging/no-load screen, surge/hi-pot)
- **Görevler:**
  1. Shrink the wound stator into the housing (primary heat path); fit insulated/hybrid-ceramic bearings (block PWM common-mode EDM currents); set the 0.70 mm gap; route resolver + PT100/NTC.
  2. Dyno: torque-speed-power-efficiency map, MTPA verification, thermal endurance/duty cycle, NVH acoustic -- correlate against P1/P2 FEA.
  3. Stand up per-unit EOL: back-EMF, hi-pot, surge (turn-turn shorts), performance, NVH; serialize/label. (EOL screens every unit; dyno validates the design once -- both required.)
  4. Sign the RELEASE / PPAP / start-of-production gate.
- **Çıktılar:** Dyno characterization report correlated to fea_spec acceptance_targets; Per-unit EOL test spec + first-article EOL pass; PPAP / SOP release package
- **KABUL KAPISI:**
  - Measured performance within tolerance of the FEA acceptance_targets (peak torque, continuous rating, efficiency, ripple/cogging, magnet temp)
  - First-article EOL (back-EMF/hi-pot/surge/NVH) pass
  - PPAP signed -> production release

---

## Go/No-Go kapıları

- G0 (after P0): validate exits 0 AND first-order peak torque > 412 Nm floor with margin; the 3 suspect points (thermal-limited J_cont, worst-case demag, rotor SF) documented as FEA priorities.
- G1 (EM, after P1): peak torque >= 412 Nm, ripple <= 5%, cogging <= 1% of rated, no demag @ peak+150C, peak eff >= 95%, Motor-CAD within ~3-5% of FEMM cross-check.
- G2 (Thermal, after P2): continuous torque >= 190 Nm with magnet hotspot <= 150 C and winding <= 180 C.
- G3 (Structural, after P3): rotor von Mises SF >= 1.5 @ 1.2x max speed (21600 rpm), magnets retained at 18000 rpm, first bending critical speed > 18000 rpm with margin.
- G4 (Iterate exit, after P4): ALL 8 acceptance_targets pass SIMULTANEOUSLY on one geometry, cross-checked vs FEMM and analysis.py first-order.
- G5 (NVH, after P5): no EM excitation order coincides with a structural mode in 0-18000 rpm band.
- G6 (DESIGN FREEZE, after P6): no open analysis gate, eccentricity RSS <= 0.07 mm, DFM/DFMEA signed; package (drawings/BOM/tolerances) frozen under change control -- BEFORE any tooling money.
- G8a (after P8a): wound-stator IR/hi-pot/surge/resistance-balance pass before further value-add.
- G8b (after P8b): IPW verify + gouge check clean; housing bearing bores concentric to stator-bore datum; shaft features to print with k5 grind stock.
- G9 (after P9): in-place-magnetized back-EMF/flux within tolerance of FEA; balance G2.5 (target G1.0).
- G10 (RELEASE, after P10): dyno measured performance within tolerance of fea_spec acceptance_targets AND first-article EOL pass -> PPAP/SOP.

## Riskler & azaltma

| Risk | Azaltma |
|---|---|
| Discipline order violated (running thermal or structural before EM, or once-through instead of coupled). The continuous rating is thermally limited (analysis.py flags J_cont~9.8) and hot magnets derate Br -0.12%/C, changing EM torque. | Enforce EM->Thermal->Structural order; run the P4 coupled loop (hot Br/resistivity fed back into EM) until temperatures and torque converge; never accept a single forward pass. |
| The bridge three-way conflict trap: thinning the 1.0 mm outer bridge/center post to raise torque cuts leakage BUT raises ripple via bridge saturation AND cuts rotor SF below 1.5. | In every P4 iteration that touches the bridge, re-check ripple (P1) and rotor SF (P3) in the SAME pass; do not lock a torque improvement until both still pass. |
| Hairpin AC copper loss (skin/proximity in the 8 stacked bars, fe ~900 Hz at 18000 rpm) modeled as DC resistance -> optimistic efficiency map and continuous rating. | Mesh the actual per-slot bar stack and enable AC/strand loss in Motor-CAD EMag (analysis._ac_resistance_factor gives the magnitude seed); model the 4 axial magnet segments so eddy loss is not overstated. |
| Demag passed at room temp / rated current but fails at the true worst case (peak 273 A_rms opposing AND 150 C, where Hcj has dropped -0.55%/C). | Run the dedicated demag calc ONLY at peak+150C at the demag-worst beta (analysis.demag_margin already flags this as conservative-worst); require min magnet B above the hot 2nd-quadrant knee. |
| Placeholder M250-27 13-point BH + single 2.3 W/kg loss number used for the final qualifying run -> unreliable losses/efficiency (fea_spec note explicitly says REPLACE). | Swap in the vendor lamination datasheet BH + Bertotti/Steinmetz coefficients before the final acceptance run; treat earlier loss numbers as triage only. |
| Motor-CAD auto hairpin wave pattern differs from fea_spec.winding.slot_phase_map (belt order / parallel-path transposition), shifting back-EMF phase and torque. | Always VERIFY the auto pattern against winding.csv + kw~0.9598 before solving; force with set_winding_coil if it differs; cross-check in FEMM via femm_labels.csv. |
| Housing MCS mis-set in NX CAM so bearing bores are not concentric to the stator-bore datum -> silent violation of the air-gap eccentricity budget. | Set the housing MCS on the stator-bore/register axis; verify both bored seats against the manufacturing.TOLERANCES concentricity callout and the eccentricity_stackup budget; CAM journal pre-builds groups but geometry/MCS confirmation stays interactive. |
| Promising k5 journals + Ra<=0.4 straight from a turn op; NX CAM geometry selection (faces/boundaries/blank) does not survive journal replay. | Finish-turn leaving 0.1-0.2 mm grind stock and route k5/Ra to a cylindrical grinder; keep geometry attachment + IPW verify INTERACTIVE (cam_prebuild only seeds groups/tools/params). |
| Tooling lead time (progressive stamping die 6-8 wk+) drives schedule; gluing pre-magnetized magnets causes cracking/FOD/safety; CAM built for non-CAM parts. | Run prototype laser laminations in parallel with the die (but don't fail the design on laser-edge core loss); magnetize IN-PLACE on the assembled rotor; restrict NX CAM to ONLY shaft+housing (laminations=stamping/laser, magnets=supplier, hairpins=wire-form). |
| PyMotorCAD / NXOpen CAM variable and group names drift by release; headless solves hang on modal dialogs. | User confirms each variable name in the installed version (right-click field -> Copy variable name); use FindObject with fallbacks for CAM groups; set MessageDisplayState=2 and wrap solves in try/except with save_to_file between iterations. |

## Program teslimatları

- PyMotorCAD EM build+solve script (maps fea_spec.json geometry/materials/winding -> set_variable/set_array_variable; runs the fea_spec.analyses matrix; verifies winding vs slot_phase_map) + pyFEMM cross-check script driven by femm_labels.csv
- PyMotorCAD thermal + mechanical scripts (water-jacket steady-state continuous rating; 21600 rpm rotor-stress SF) and the geom(params)->metrics iterate-to-acceptance loop reading fea_spec.acceptance_targets
- Updated params.py / JSON override for the WINNING geometry plus the regenerated fea hand-off package (fea_spec.json, cross_section.dxf, winding.csv, femm_labels.csv) in lockstep
- Frozen manufacturing package: bom.csv, tolerances.csv (12-feature GD&T, datum A bearing axis), drawings/ (DXF+SVG assembly/stator/rotor), eccentricity stack-up pass, DFMEA/PFMEA + control plan
- Per-part STEP exports from nx_builder.py `parts` mode: <base>_Shaft.stp + <base>_Housing.stp (CAM inputs) and _Stator_Lamination/_Rotor_Lamination/_Magnets/_Winding (supplier reference)
- cam_prebuild.py NXOpen journal (seeds CAM setup/MCS/WORKPIECE/PROGRAM/METHOD/TOOLs/operation shells with feeds/speeds/stock) + interactive checklist for geometry attach/verify/post; verified G-code for shaft turning + housing milling
- Supply-chain release packs: lamination DXF for stamping die + prototype laser; N42SH 4-axial-segment unmagnetized magnet PO; hairpin wire-form geometry
- Test + correlation deliverables: wound-stator electrical test results (IR/hi-pot/surge), in-place magnetize + balance (G2.5/G1.0) records, dyno characterization correlated to fea_spec acceptance_targets, per-unit EOL spec, PPAP/SOP release package

---

## Ek A — Ansys Motor-CAD otomasyonu (PyMotorCAD)

**Scriptlenebilirlik:** Yes, fully scriptable. Primary API: PyMotorCAD = pip package `ansys-motorcad-core` (import ansys.motorcad.core as pymotorcad), a JSON-RPC/HTTP interface; bundled inside Motor-CAD's internal Scripting tab since v2023R1, also usable externally (Python 3.9-3.14, Windows). Entry point: mc = pymotorcad.MotorCAD() launches/connects an instance; mc = pymotorcad.MotorCAD(open_new_instance=False) attaches to a running GUI for debugging. Legacy ActiveX/COM is supported via the MotorCADCompatibility class for old scripts. Core verbs: load_from_file / save_to_file (.mot); show_magnetic_context / show_thermal_context / set_motorlab_context to switch module; set_variable(name,value) / get_variable(name) and set_array_variable / get_array_variable for the thousands of named inputs; do_magnetic_calculation(), do_steady_state_analysis()/do_transient_analysis(), do_mechanical_calculation(), build_model_lab()/calculate_magnetic_lab()/calculate_operating_point_lab() for the four solvers; result graphs via get_magnetic_graph / get_fea_graph / get_magnetic_graph_harmonics; set_component_material / set_fluid for materials and coolant; set_winding_coil for explicit winding; Adaptive Templates geometry objects get_region / set_region / get_region_dxf / reset_adaptive_geometry / load_adaptive_script for custom DXF/parametric geometry. Set set_variable("MessageDisplayState",2) to suppress pop-ups for headless runs.

**Kurulum sırası:**
1. INSTALL/CONNECT: pip install ansys-motorcad-core; in Python: import ansys.motorcad.core as pymotorcad; mc = pymotorcad.MotorCAD(); mc.set_variable('MessageDisplayState',2) to run headless without pop-ups. (Or write the script inside Motor-CAD's Scripting tab where mc is pre-bound.)
2. PICK TEMPLATE + TOPOLOGY: mc.set_variable('Motor_Type', <BPM index>); load the single-layer V interior-PM rotor template (e9-class IPM, the V-magnet bank). mc.show_magnetic_context() and mc.display_screen('Scripting').
3. GEOMETRY from fea_spec.geometry_mm via set_variable: Slot_Number=54, Pole_Number=6 (poles), Stator_Lam_Dia=225, Stator_Bore=161, Airgap=0.7, Stator_Lam_Length / stack=134 with stacking factor 0.96, Shaft_Dia=45, Tooth_Width=5.6, back-iron 13.0. For the single-V rotor set the magnet bank: magnet thickness, V web/separation, pole arc, outer bridge=1.0 and center-post half-width=1.0 (use set_array_variable for layered arrays like WebThickness_Array, PoleArc_Array). Tune V-angle + magnet width to land rotor_OD effective at 159.6 (bore 161 minus 2x airgap 0.7).
4. OPTIONAL DXF IMPORT (only if the template V-pocket can't match cross_section.dxf): requires Motor-CAD 2024R2+. Pre-process the DXF so the rotation axis is at origin (0,0) and each sector's lower edge sits on the +X axis (the spec's d-axis is already +X). Geometry->Editor->Geometry tab, tick Import to see DXF regions; set Geometry Templates Type = Adaptive (set_variable('GeometryTemplateType',1)); in the adaptive script use src=mc.get_region_dxf('<dxf region>'), tmpl=mc.get_region('<template region e.g. RotorPocket/Magnet>'), tmpl.replace(src), mc.set_region(tmpl); mc.load_adaptive_script(). Keep the lamination outline from the template and only swap the magnet/pocket pockets.
5. MATERIALS: create a custom NO-steel lamination material with the 13-point BH curve from fea_spec.materials.lamination (B/H arrays), density 7650, stacking 0.96, and enter Bertotti/Steinmetz iron-loss coefficients (curve-fit the 2.3 W/kg @1.5T/50Hz datapoint and replace with the real vendor datasheet per the spec note). Assign via mc.set_component_material('Stator Lam (Back Iron)', <grade>) and the rotor lam. Magnet: N42SH with Br=1.28 T@20C, Hcj=1592 kA/m, recoil mu=1.05, Br tempco -0.12%/C, Hcj tempco -0.55%/C, max service 150 C, resistivity 1.4 uOhm-m, axial segments=4 (for eddy loss). Conductor: copper sigma 59.6 MS/m @20C, tempco 0.00393/C, class-H enamel.
6. WINDING (hairpin): set MagneticWindingType=Hairpin; conductors/turns per slot = 8, parallel paths = 2, layers = 8 (one phase belt per slot, full pitch), coil/throw pitch = 9 slots, q=3, series turns/phase=36. Let Motor-CAD auto-generate the balanced hairpin wave pattern, then VERIFY it against fea_spec.winding.slot_phase_map (the exact 54-slot A/B/C +/- belt). If the auto pattern differs, force it with the Winding pattern editor / mc.set_winding_coil(coil, phase, go_layer, return_layer, ...) to reproduce the slot_phase_map. Check computed kw approx 0.9598.
7. EXCITATION + OPERATING POINTS: set DCBusVoltage=400; CurrentDefinition (peak/RMS); rated 126 A_rms / peak 273 A_rms (convert to the peak the variable expects); Shaft_Speed_[RPM]=3480 base; PhaseAdvance swept over 0..45 deg from q-axis for MTPA; TorquePointsPerCycle and TorqueNumberCycles for ripple resolution; magnet operating temperature for the demag/hot runs.
8. RUN EMAG: enable CoggingTorqueCalculation, BackEMFCalculation, TorqueCalculation (transient torque vs angle), loss calcs; mc.do_magnetic_calculation(); read mc.get_variable('ShaftTorque'), PeakLineLineVoltage, mc.get_magnetic_graph('TorqueVW') for ripple, get_magnetic_graph_harmonics for back-EMF THD and cogging. Sweep PhaseAdvance in a Python loop to find MTPA; sweep id/iq for the Ld/Lq saturation maps.
9. DEMAGNETIZATION: switch to the demag calc, set magnet temperature to 150 C, apply the worst-case d-axis-opposing peak current (273 A_rms peak, beta at the demag-worst angle), re-solve and read the per-element/min magnet B vs the 2nd-quadrant knee and the demag-proportion result; require zero irreversible demag at peak current AND 150 C.
10. THERMAL continuous rating: mc.show_thermal_context(); select the water-jacket housing cooling model; set HousingWJFluid / WJ_Fluid_Volume_Flow_Rate / WJ_Fluid_Inlet_Temperature (and the h-correlation inputs); feed the EMag losses (copper+iron+magnet) either by EMag->Thermal coupling or as imposed losses; mc.do_steady_state_analysis(); read T_[Winding_Max] and the magnet hotspot node (get_node_temperature) vs the 150 C magnet / 180 C class-H limits. Iterate current down to find the thermally-limited continuous torque (target >=190 Nm).
11. LAB envelope + efficiency map: mc.set_motorlab_context(); set ModelType_MotorLAB, BuildSatModel_MotorLAB, ModelBuildSpeed/MaxModelCurrent; mc.build_model_lab() to build the saturation+loss model; set SpeedMax_MotorLAB=18000, Imax_MotorLAB (peak), DC voltage, then mc.calculate_magnetic_lab() for the torque-speed + efficiency map (verify peak eff >=95%), and mc.calculate_operating_point_lab() with LabThermalCoupling on for the thermally-constrained continuous envelope. Run the drive-cycle/duty-cycle for the duty rating.
12. MECHANICAL rotor stress: switch to the Mechanical/stress context; set ShaftSpeed to 1.2 x 18000 = 21600 rpm (overspeed); mc.do_mechanical_calculation(); read MaxStress_RotorLam and YieldStress_RotorLam and compute SF = yield/max in the outer bridges (1.0 mm) and center post (1.0 mm half-width); require SF>=1.5. (Rotordynamics/1st-bending critical speed is NOT in Motor-CAD's 2D rotor-stress solver -- do that shaft-bending check in NX/Ansys Mechanical with the shaft+rotor mass.)
13. ITERATE-TO-ACCEPTANCE LOOP: wrap steps 3-13 in a Python function geom(params)->metrics, compare each metric to fea_spec.acceptance_targets (peak torque>=412, cont torque>=190, ripple<=5%, cogging<=1% of rated, no demag@150C/peak, magnet<=150C, rotor SF>=1.5@1.2x, peak eff>=95%), adjust V-angle/magnet width/bridge/web/turns, re-solve. Save winners with mc.save_to_file(). Optionally use Motor-CAD's optimization or drive it from a PyMotorCAD parameter sweep.
14. HANDOFF TO MANUFACTURING/NX: take the accepted geometry numbers back into the existing motor_nx params/blueprint, regenerate the artifacts (drawings.py 2D, nx_builder.py 3D STEP per component). Laminations -> stamping/laser tooling (NOT CAM), magnets -> N42SH supplier order (4 axial segments), hairpins -> wire-forming supplier; only Shaft (turning) and Housing (milling) go through NX CAM.

**Tuzaklar:** Manufacturing reality (from the project constraint): only Shaft (turning) and Housing (milling) use NX CAM. Laminations are stamped/laser-cut from tooling, magnets are bought from a supplier, hairpins are wire-formed. Motor-CAD outputs the validated geometry/DXF for those supply chains, NOT a CAM toolpath. | Planning agents cannot run Motor-CAD or NX -- these are deliverable scripts/sequences the USER runs. The exact variable-name strings here are from PyMotorCAD docs/cheat-sheet but some differ slightly by Motor-CAD release; the user should confirm each name in their installed version (right-click a field -> 'Copy variable name', or the Scripting tab variable lookup). | DXF import into Adaptive Templates requires Motor-CAD 2024R2 or later; before that DXF was import-only for visualization. The DXF must have its rotation axis at origin (0,0) and each sector's lower edge aligned on the +X axis -- cross_section.dxf likely needs re-orienting/trimming to a single pole sector first. Prefer the parametric template and use DXF only if the V-pocket can't be matched. | Hairpin AC copper loss (skin/proximity in the 8 stacked bars) is the dominant high-speed loss and is layer-position dependent -- you must model the actual bar stack per slot and enable AC-loss/strand calculation, not just DC resistance, or the efficiency map and continuous rating will be optimistic. | Motor-CAD's Mechanical module is a 2D rotor-stress (plane) solver -- it covers centrifugal von Mises in bridges/center-post (acceptance SF>=1.5 @ 1.2x speed) but does NOT do the shaft-rotor 1st-bending critical-speed (rotordynamics) check in fea_spec analysis 'rotordynamics'. Do that separately in NX/Ansys Mechanical with the hollow 45 mm shaft + rotor mass. | The lamination iron-loss model needs Bertotti/Steinmetz coefficients, not a single 2.3 W/kg @1.5T/50Hz number; curve-fit or, per the spec note, replace the placeholder BH+loss with the real vendor M250-27-class datasheet before the final acceptance run. | Demag must be checked at the worst-case combination (peak current 273 A_rms AND 150 C magnet) -- a room-temperature or rated-current demag pass can falsely pass; the Hcj at 150 C is much lower (-0.55%/C from 1592 kA/m). | Always verify Motor-CAD's auto-generated hairpin pattern against the fea_spec 54-slot slot_phase_map (and kw ~0.9598) before solving -- the auto wave-winding belt order/parallel-path transposition may not match the FEMM/Maxwell map, which would shift back-EMF phase and torque. | set MessageDisplayState=2 for headless/batch loops or modal dialog boxes will hang the script; wrap solves in try/except and save_to_file between iterations so a crash doesn't lose the converged geometry. | Keep Motor-CAD as the design hub but cross-check the final winning point against the existing FEMM/Maxwell 1-pole anti-periodic model and analysis.py first-order numbers (base speed ~3480 rpm, peak ~473 Nm) -- the spec's first-order peak 473 Nm vs acceptance 412 Nm leaves margin, so treat Motor-CAD torque within a few % of FEMM as the validation gate.

## Ek B — Siemens NX 2506 CAM (şaft tornalama + gövde freze)

**Scriptlenebilirlik:** Partially scriptable via NXOpen (Python journal run headless with `%UGII_ROOT_DIR%\run_journal.exe`, same harness nx_builder.py already uses). RELIABLE to pre-build in a journal: (1) create/select the CAM setup and root via workPart.CAMSetup (CAMSetup is obtained from Part.CAMSetup; setup is created from a manufacturing template the same way nx_builder creates a part); (2) create GEOMETRY groups â€” MCS/OrientGeometry and WORKPIECE via CAMSetup.CAMGroupCollection (FindObject('MCS_SPINDLE'|'MCS'|'WORKPIECE'|'TURNING_WORKPIECE') for template-seeded groups, or CreateGeometryGroup to add new ones); (3) create PROGRAM and METHOD groups likewise; (4) create TOOLs in the CAMGroupCollection (turning OD tool, grooving tool, drill, face mill, boring bar) with their numeric params (diameter, nose radius, angles, feeds) set non-interactively; (5) create OPERATIONS shells via CAMSetup.CAMOperationCollection.Create(programGroup, methodGroup, tool, orientGeom/workpiece, opTypeName, opSubtypeName, paste) â€” e.g. ROUGH_TURN/FINISH_TURN/GROOVE/CENTERLINE_DRILL, FACE_MILLING/CAVITY/ZLEVEL; (6) set numeric cut params via the operation Builders (FeedsBuilder/FeedCutBuilder, SpindleRpmBuilder, stock/stepover/depth-of-cut), and the BLANK definition where it is a simple cylinder/offset; (7) batch-set the same params across many ops; (8) drive GenerateToolPath(CAMObject[]) and Postprocess(CAMObject[], machineType/posterName, outFile, OutputUnits.Metric) and CreateGougeCheckBuilder()/Validate()+Commit programmatically. Entry points: NXOpen.CAM namespace â€” CAMSetup, CAMGroupCollection, CAMOperationCollection, OrientGeometry, NCGroup, FeatureGeometry, GougeCheckBuilder; CAMSetup.GenerateToolPath / .Postprocess / .CreateGougeCheckBuilder; CAMSetup.View enum {ProgramOrder, MachineMethod, Geometry, MachineTool}; OutputUnits {Inch, Metric, PostDefined}.

**Kurulum sırası:**
1. Generate the two machined-part solids with the existing builder: run `"%UGII_ROOT_DIR%\run_journal.exe" motor_nx/nx_builder.py -args <out>.prt parts` â€” this writes per-component STEP files including motor_nx_out_Shaft.stp and motor_nx_out_Housing.stp (see _component_of/export_parts in nx_builder.py). These are the CAM inputs; no other motor_nx part is machined.
2. SHAFT: open/import the Shaft solid into a new .prt as the master model component; switch to Manufacturing (cam_general configuration, `turning` setup template). The shaft is a revolve about +Z (the builder's motor axis), so NX extracts the turn profile cleanly.
3. SHAFT: set MCS_SPINDLE on the Z rotation axis, program zero at a face; specify ZM-XM as the turning work plane. Open TURNING_WORKPIECE: set PART = shaft solid; set BLANK = cylindrical bar (Ã˜>=45 finished + stock) or a pre-bored tube (Ã˜14 hollow). Confirm the extracted turn boundary follows the journals, the two bearing seats (Ã˜40, 22 mm), the rotor press-seat (Ã˜45), and shoulders.
4. SHAFT: create tools â€” OD turning tool (rough+finish), grooving tool for shoulder relief grooves, centerline drill for the Ã˜14 oil bore. Create operations in order: FACE -> ROUGH_TURN_OD -> CENTERLINE_DRILL (Ã˜14) -> GROOVE_OD (relief at journal shoulders) -> FINISH_TURN_OD. On the k5 journals/bearing seats leave ~0.1-0.2 mm radial stock (FINISH_TURN stock) because the +0.013/+0.002 k5 fit and Ra<=0.4 um are achieved by GRINDING, not turning â€” flag grinding as a downstream (often non-NX) op.
5. SHAFT: GENERATE all toolpaths; VERIFY with 3D material-removal (IPW) playback; run Gouge/Collision check; Postprocess with the lathe-specific post to G-code. Manually confirm finish stock and tool clearance into the hollow bore.
6. HOUSING: import the Housing solid; Manufacturing (cam_general, `mill_planar` and/or mill_contour template). Set MCS so Z is the stator-bore/bearing-bore axis and origin on the housing register face â€” this makes both bearing bores concentric to the stator-bore datum (the critical-to-function GD&T in manufacturing.py TOLERANCES).
7. HOUSING: WORKPIECE PART = housing solid, BLANK = the as-cast body (use From Part / IPW or a small offset envelope, since it is die-cast near-net). Create tools: face mill, boring bar or end mill for the DE/NDE bearing bores.
8. HOUSING: create operations â€” FACE_MILLING for the two bearing faces (perpendicularity to axis), then bore-finish the DE/NDE bearing seats (mill_contour finish / helical or circular bore, or a bore hole-cycle) to the H6/J6/K6 fits. GENERATE -> VERIFY (IPW) -> Gouge/Collision check -> Postprocess to G-code with the mill post.
9. AUTOMATION HAND-OFF: ship an NXOpen Python journal (cam_prebuild.py, same run_journal harness as nx_builder.py) that, given the imported part, pre-builds setup + MCS + WORKPIECE + PROGRAM/METHOD + TOOLs + empty/near-complete OPERATION shells with all numeric params (feeds, speeds, stepover, DOC, finish stock) preset, then STOPS for the operator to attach geometry collectors and run generate/verify. Keep an interactive checklist for the geometry-selection + verify + post steps the journal cannot reliably do.

**Tuzaklar:** A CAM journal CANNOT reliably record/replay interactive GEOMETRY SELECTION (faces, edges, boundaries, cut regions, blank-from-part picks). Recorded selections are tied to transient tag/screen picks and break on re-import; eng-tips and NX Journaling both document face/boundary selection not surviving journal replay. So attaching PART/BLANK faces, turn-boundaries, and milling cut regions/floors must stay INTERACTIVE (or use robust selection-intent rules authored by hand, e.g. ScCollector + face rules â€” not recorded). | VERIFY (3D material-removal / IPW playback) and visual acceptance are interactive/visual â€” a journal can launch generate+gouge-check and read pass/fail, but a human must watch the simulation to accept it. | k5 journals (+0.013/+0.002) and Ra<=0.4 um are NOT achieved by turning. Finish-turn must leave grind stock (~0.1-0.2 mm); final size/finish is a GRINDING operation, frequently on a dedicated cylindrical grinder outside NX CAM. Do not promise the k5/Ra target straight from a turn op. | Laminations, magnets, and hairpins are explicitly NOT NX CAM (stamping/laser, supplier, wire-forming). Only Shaft (turn) and Housing (mill) belong in CAM â€” do not build CAM setups for the other components. | Housing is die-cast (near-net): the BLANK should be the as-cast body (From Part/IPW), not a big prismatic billet, or roughing toolpaths will be wrong and huge. Bearing bores are a FINISH milling/boring operation on cast stock, not full pocketing. | MCS placement is load-bearing: the housing MCS must be on the stator-bore axis/register so both bearing bores come out concentric to the stator bore â€” that concentricity is the air-gap eccentricity contributor in manufacturing.py (budget ~10% of the 0.70 mm gap). A mis-set MCS silently violates the acceptance GD&T. | NXOpen CAM template/group NAMES are version- and template-localized (e.g. MCS vs MCS_SPINDLE, WORKPIECE vs TURNING_WORKPIECE, NONE method). Use FindObject defensively with fallbacks (mirror the version-drift hardening pattern already in nx_builder.py: try multiple member/name spellings) and confirm exact names against NX 2506 once interactively. | CreateOperation/Create needs valid parent groups (program, method, tool, geometry) to already exist; a journal must create groups in dependency order before operations or Create throws. | Postprocess requires a real posterName/machine post installed in the NX post library; the journal can call Postprocess but the correct machine-specific post must be configured first (otherwise default/dummy G-code). | The hollow Ã˜14 shaft means the turning BLANK and centerline-drill strategy must account for the bore (pre-bored tube vs solid bar + gun-drill); IPW for a tube blank differs from a solid blank.

## Kaynaklar

- http://www2.me.rochester.edu/courses/ME204/nx_help/en_US/tdocExt/content/d/turning_turn_geom_mcs_spind.xml
- https://blog.janus-engineering.com/en_en/details/article/siemens-nx-release-2506/
- https://blog.ozeninc.com/resources/ansys-motor-cad-demagnetization-of-an-ipm-motor
- https://blog.ozeninc.com/resources/ansys-motor-cad-lab-module-efficiency-maps
- https://blog.ozeninc.com/resources/motorcad-overview-multi-physics-em-thermal-nvh
- https://chargedevs.com/whitepapers/detecting-latent-defects-in-electric-motors-with-partial-discharge/
- https://community.sw.siemens.com/s/article/Understanding-the-different-methods-to-initialize-NX-CAM
- https://developer.ansys.com/blog/pymotorcad-cheat-sheet
- https://docs.plm.automation.siemens.com/data_services/resources/nx/12/nx_api/custom/en_US/nxopen_python_ref/NXOpen.CAM.GougeCheckBuilder.html
- https://docs.sw.siemens.com/documentation/external/PL20191127135844554/en-US/nx_api_sc/nx/1926/nx_api_sc/en_US/nxopen_net/a02792.html
- https://github.com/ansys/pymotorcad
- https://help.altair.com/fluxmotor/topics/MF_Design_WindingHairpin.htm
- https://motorcad.docs.pyansys.com/
- https://motorcad.docs.pyansys.com/version/stable/examples/adaptive_library/DXFImport.html
- https://motorcad.docs.pyansys.com/version/stable/examples/basics/emag_basics.html
- https://motorcad.docs.pyansys.com/version/stable/examples/basics/lab_basics.html
- https://motorcad.docs.pyansys.com/version/stable/examples/basics/thermal_basics.html
- https://motorcad.docs.pyansys.com/version/stable/examples/internal_scripting/mechanical_force.html
- https://motorcad.docs.pyansys.com/version/stable/examples/internal_scripting/mechanical_stress.html
- https://motorcad.docs.pyansys.com/version/stable/methods/index.html
- https://motorcad.docs.pyansys.com/version/stable/user_guide/adaptive_templates.html
- https://motorcad.docs.pyansys.com/version/stable/user_guide/internal_scripting.html
- https://motorneo.com/progressive-stamping/
- https://nxjournaling.com/content/select-program-group-and-iterate-operations
- https://nxopencsdocumentation.thescriptingengineer.com/NX2022_1/NXOpen.CAM.html
- https://plmtechtalk.com/2023/03/03/how-to-create-and-simulate-a-basic-face-milling-operation-siemens-nx-2206-cam-application/
- https://pubs.aip.org/aip/adv/article/8/4/047504/1032271/Study-on-optimal-design-of-210kW-traction-IPMSM
- https://quality-one.com/apqp/
- https://quality-one.com/dfm-dfa/
- https://simutechgroup.com/resources/blog/motorcad-overview-multi-physics-em-thermal-nvh
- https://www.ansys.com/applications/electric-motors
- https://www.ansys.com/blog/hairpin-winding-for-electric-machine-design
- https://www.ansys.com/blog/new-adaptive-templates-ansys-motor-cad-make-motor-design-faster-easier-more-scalable
- https://www.ansys.com/blog/thermal-management-solutions-electric-traction-motors
- https://www.ansys.com/products/electronics/ansys-motor-cad
- https://www.appliedcax.com/resources/nx-cam/nx-cam-tutorial-creating-associative-ipw-blank-between-setup-files/
- https://www.ceas.uc.edu/research/centers-labs/siemens-simulation-technology-center/courses---projects/nx-cam/manufacturing-processes-course/manufacturing-processes-example/instructions-nx-cam.html
- https://www.emobility-engineering.com/electric-motor-testing-methods-reliability-performance/
- https://www.emobility-engineering.com/motor-development-manufacturing/
- https://www.emobility-engineering.com/motor-laminations/
- https://www.eng-tips.com/threads/find-mcs-used-for-cam-operation-in-nx-open.370364/
- https://www.eng-tips.com/threads/journal-on-cam-face-milling-not-able-to-select-the-face-geometry-boundries.376646/
- https://www.gtisoft.com/blog-post/how-to-analyze-noise-vibration-and-harshness-in-electric-powertrains-using-simulation/
- https://www.marposs.com/eng/application/end-of-line-testing-of-electric-motors
- https://www.marposs.com/eng/blog/electrical-and-functional-testing-systems-for-electric-motors-and-components
- https://www.mdpi.com/2032-6653/17/6/299
- https://www.mdpi.com/2075-1702/10/8/715
- https://www.nature.com/articles/s41598-025-93285-x
- https://www.qad.com/blog/2019/04/the-5-phases-of-apqp
- https://www.researchgate.net/publication/365183607_Hairpin_Windings_for_Electric_Vehicle_Motors_Modeling_and_Investigation_of_AC_Loss-Mitigating_Approaches
- https://www.sciencedirect.com/science/article/abs/pii/S0304885323002482
- https://www.sciencedirect.com/science/article/pii/S2772671125000920
- https://www.simscale.com/blog/electric-motor-simulation-and-design/
- https://www.swooshtech.com/2021/04/02/how-to-set-up-ipw-in-nx-cam/
- https://www.swooshtech.com/2021/05/07/setting-up-mill-turn-parts-in-nx/
