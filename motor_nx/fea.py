"""FEA hand-off: turn a MotorParams design into the inputs a finite-element
study needs -- WITHOUT running FEA here.

Produces three artefacts (see `write_package`):
  * fea_spec.json    -- materials (BH / Br / temp-coeffs), symmetry & boundary
                        conditions, winding slot->phase map, excitation, mesh,
                        the analysis matrix and the acceptance targets.
  * cross_section.dxf-- the 2D lamination plane (steel / slots / magnets / copper
                        on named layers) for import into any 2D EM solver.
  * winding.csv      -- slot, phase, sign  (one row per stator slot).

The numbers here are first-order, vendor-datasheet-replaceable defaults. The EM,
thermal and structural test plan that consumes them lives in docs/FEA_PREP.md.
"""
from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Tuple

from . import blueprint as _bp
from . import em_design
from .params import MotorParams

# 60-degree phase-belt order for a balanced 3-phase integer-slot winding,
# one (phase, sign) per belt of q slots: A+, C-, B+, A-, C+, B-.
_BELTS: List[Tuple[str, int]] = [("A", +1), ("C", -1), ("B", +1),
                                 ("A", -1), ("C", +1), ("B", -1)]


def winding_layout(p: MotorParams) -> List[Tuple[str, int]]:
    """Return [(phase, sign)] indexed by stator slot. This integer-slot (q) winding
    puts each slot wholly in one 60-deg phase belt, so ALL conductors_per_slot
    hairpin bars in a slot carry the same (phase, sign): one entry per slot fully
    describes the excitation (the bars stack radially; chording via
    winding.coil_span_slots would shift belts -- modelled full-pitch here)."""
    g = em_design.derive(p)
    q = max(1, int(round(g.slots_per_pole_per_phase)))
    return [_BELTS[(i // q) % 6] for i in range(p.stator.slot_count)]


def fea_spec(p: MotorParams, a: "em_design.EMAssumptions" = None) -> Dict[str, Any]:
    """Complete, machine-readable FEA setup for `p`."""
    a = a or em_design.EMAssumptions()
    g = em_design.derive(p)
    e = em_design.estimate_performance(p, a)
    s, r, w, m = p.stator, p.rotor, p.winding, p.material
    layout = winding_layout(p)

    return {
        "name": p.name,
        "machine": "interior-PM synchronous (IPMSM), single-V rotor",
        "geometry_mm": {
            "stator_OD": s.outer_diameter, "stator_bore": s.bore_diameter,
            "rotor_OD": round(2 * g.rotor_outer_radius, 2), "shaft_OD": p.shaft.diameter,
            "air_gap": r.air_gap, "stack_length": p.stack_length,
            "slots": s.slot_count, "poles": r.pole_count,
            "back_iron_mm": s.back_iron_thickness, "tooth_width_mm": s.tooth_width,
            "outer_bridge_mm": r.outer_bridge, "center_post_halfwidth_mm": r.center_post_halfwidth,
        },
        "symmetry_and_bc": {
            "fea_sector": "1 pole",
            "sector_angle_deg": round(360.0 / r.pole_count, 4),
            "radial_cut_faces": "ANTI-periodic (odd) master/slave  -- field repeats with sign flip each pole",
            "alt_sector": "1 pole-pair (%.1f deg) with EVEN (periodic) master/slave" % (720.0 / r.pole_count),
            "outer_boundary": "magnetic vector potential A = 0 on the stator OD",
            "axisymmetry_note": "2D planar EM is standard for radial-flux IPMSM; end effects via the analytical end-leakage / 3D check",
        },
        "materials": {
            "lamination": {
                "grade": m.electrical_steel, "thickness_mm": s.lamination_thickness,
                "stacking_factor": m.stacking_factor, "density_kg_m3": 7650,
                # representative NON-ORIENTED silicon-steel single-valued BH curve
                "BH_H_A_per_m": [0, 50, 100, 150, 200, 300, 500, 1000, 2000, 5000, 10000, 30000, 80000],
                "BH_B_T":       [0, 0.55, 0.95, 1.18, 1.33, 1.49, 1.60, 1.71, 1.80, 1.90, 1.96, 2.05, 2.20],
                "core_loss_W_per_kg_at_1.5T_50Hz": 2.3,
                "note": "REPLACE with the vendor datasheet BH + Bertotti/Steinmetz loss coefficients for the final run",
            },
            "magnet": {
                "grade": m.magnet_grade, "Br_T_at_20C": m.magnet_br_t,
                # Hcb from the linear recoil line Br = mu0*mu_recoil*Hcb (kept
                # self-consistent with Br/mu_recoil instead of a hardcoded value).
                "Hcb_kA_per_m": round(m.magnet_br_t / (4e-7 * math.pi * m.magnet_mu_recoil) / 1000.0, 1),
                "Hcj_kA_per_m": m.magnet_hcj_ka_m, "mu_recoil": m.magnet_mu_recoil,
                "Br_tempco_pct_per_C": -0.12, "Hcj_tempco_pct_per_C": -0.55,
                "max_service_C": m.magnet_max_service_c,
                "density_kg_m3": 7500, "resistivity_uOhm_m": 1.4,
                "axial_segments": m.magnet_segments_axial,
                "note": "demag check uses the B-H 2nd-quadrant knee at the hot operating temp",
            },
            "conductor": {
                "material": "copper", "conductivity_S_per_m_20C": 5.96e7,
                "temp_coeff_per_C": 0.00393, "insulation_class": m.copper_insulation_class,
                "slot_fill_factor": a.slot_fill,
            },
            "shaft": {"material": "alloy steel (e.g. 42CrMo4), yield ~750 MPa"},
            "housing": {"material": m.housing_material},
        },
        "winding": {
            "type": "integer-slot distributed hairpin (%d bars/slot, all bars per slot in one phase belt), full pitch" % w.conductors_per_slot,
            "phases": w.phases, "q_slots_per_pole_per_phase": round(g.slots_per_pole_per_phase, 3),
            "conductors_per_slot": w.conductors_per_slot, "parallel_paths": w.parallel_paths,
            "series_turns_per_phase": round(e.series_turns_per_phase),
            "coil_pitch_slots": g.coil_pitch_slots, "winding_factor_kw": round(g.winding_factor, 4),
            "d_axis": "rotor magnet-V symmetry axis on +X for the reference pole",
            "slot_phase_map": [{"slot": i, "phase": ph, "sign": ("+" if sg > 0 else "-")}
                               for i, (ph, sg) in enumerate(layout)],
        },
        "excitation": {
            "rated_current_A_rms": round(e.i_phase_cont_a),
            "peak_current_A_rms": round(e.i_phase_peak_a),
            "current_advance_angle_deg_from_q_axis": [0, 5, 10, 15, 20, 25, 30, 35, 40, 45],
            "note": "sweep beta to locate MTPA; IPM saliency contributes reluctance torque",
        },
        "operating_points": {
            "dc_bus_V": a.dc_bus_v, "base_speed_rpm": round(e.base_speed_rpm),
            "max_speed_rpm": round(a.max_speed_rpm),
            "corner_torque_Nm": round(e.torque_peak_nm),
            "continuous_torque_Nm": round(e.torque_cont_nm),
        },
        "mesh": {
            "airgap_radial_layers": 3, "airgap_element_mm": round(r.air_gap / 3.0, 3),
            "refine": ["outer_bridge", "center_post", "tooth_tips", "magnet_corners"],
            "global_element_mm": 2.0,
        },
        "analyses": _ANALYSES,
        "acceptance_targets": {
            "peak_torque_Nm_min": round(0.9 * e.torque_peak_nm),
            "continuous_torque_Nm_min": round(0.9 * e.torque_cont_nm),
            "torque_ripple_pct_max": 5.0, "cogging_pct_of_rated_max": 1.0,
            "no_demag_at": "peak current AND %.0f C magnet temp" % m.magnet_max_service_c,
            "magnet_temp_C_max_continuous": m.magnet_max_service_c,
            "rotor_vonMises_safety_factor_min_at_1.2x_maxspeed": 1.5,
            "peak_efficiency_pct_min": 95.0,
        },
        "tools": {
            "em_2d": ["Ansys Maxwell 2D", "Ansys/Motor-CAD E-Magnetic", "FEMM / pyFEMM (free)"],
            "thermal": ["Motor-CAD Thermal", "Ansys Mechanical/Fluent", "lumped-parameter network"],
            "structural": ["Ansys Mechanical", "any rotor-stress FEA"],
        },
    }


_ANALYSES = [
    {"id": "cogging", "domain": "EM", "what": "cogging torque vs rotor angle (no current, fine steps over 1 slot pitch)"},
    {"id": "back_emf", "domain": "EM", "what": "open-circuit phase flux-linkage & back-EMF at base speed; magnitude vs Vdc + harmonic (THD)"},
    {"id": "flux_ld_lq", "domain": "EM", "what": "flux linkage maps -> Ld, Lq vs (id, iq); saliency ratio"},
    {"id": "torque_angle", "domain": "EM", "what": "average torque vs current advance angle at rated & peak current -> MTPA locus"},
    {"id": "torque_ripple", "domain": "EM", "what": "instantaneous torque vs rotor angle at MTPA -> ripple %"},
    {"id": "demag", "domain": "EM", "what": "worst-case demag: peak q-axis-opposing current at max magnet temp; check min magnet B vs knee"},
    {"id": "losses", "domain": "EM", "what": "iron loss (stator/rotor), magnet eddy loss (with axial segmentation), AC copper loss (skin/proximity in hairpins)"},
    {"id": "efficiency_map", "domain": "EM", "what": "efficiency over the torque-speed envelope (field weakening above base speed)"},
    {"id": "jacket_thermal", "domain": "Thermal", "what": "water-jacket cooled steady-state temps from EM losses; winding & MAGNET hotspot vs limits; continuous rating"},
    {"id": "rotor_stress", "domain": "Structural", "what": "centrifugal von Mises in the outer bridges & center post at 1.2x max speed; magnet retention"},
    {"id": "rotordynamics", "domain": "Structural", "what": "shaft-rotor 1st bending critical speed > max operating speed with margin"},
]


# --------------------------------------------------------------------------- #
# 2D cross-section DXF (R12 ASCII; LWPOLYLINE + CIRCLE on per-material layers)
# --------------------------------------------------------------------------- #
_LAYER = {
    "stator_steel": "STATOR_STEEL", "stator_slot_cut": "SLOT_AIR",
    "rotor_steel": "ROTOR_STEEL", "rotor_hole_cut": "ROTOR_AIR",
    "magnet_pocket_cut": "POCKET_AIR", "magnet": "MAGNET",
    "conductor": "COPPER", "shaft": "SHAFT", "housing": "HOUSING",
    "cooling_channel_cut": "COOLANT",
}


def _dxf_polyline(layer: str, pts) -> List[str]:
    out = ["0", "LWPOLYLINE", "8", layer, "90", str(len(pts)), "70", "1"]
    for x, y in pts:
        out += ["10", "%.4f" % x, "20", "%.4f" % y]
    return out


def _dxf_circle(layer: str, cx: float, cy: float, rad: float) -> List[str]:
    return ["0", "CIRCLE", "8", layer, "10", "%.4f" % cx, "20", "%.4f" % cy, "40", "%.4f" % rad]


def to_dxf(blueprint: Dict[str, Any]) -> str:
    """Project the build steps onto the XY lamination plane as a DXF, one layer
    per material. Uses the shared blueprint.iter_cross_section projection."""
    body = []
    for role, shape in _bp.iter_cross_section(blueprint):
        layer = _LAYER.get(role, "MISC")
        if shape[0] == "circle":
            body += _dxf_circle(layer, shape[1], shape[2], shape[3])
        else:  # polygon
            body += _dxf_polyline(layer, shape[1])
    return "\n".join(["0", "SECTION", "2", "ENTITIES"] + body + ["0", "ENDSEC", "0", "EOF"]) + "\n"


# --------------------------------------------------------------------------- #
# FEMM block-label recipe (import cross_section.dxf, then place these labels)
# --------------------------------------------------------------------------- #
def _rot(x: float, y: float, deg: float) -> Tuple[float, float]:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x * c - y * s, x * s + y * c)


def femm_label_recipe(p: MotorParams) -> List[Dict[str, Any]]:
    """One row per FEMM block label: interior point (x, y) + material, plus
    circuit/turns for copper and the magnetisation direction for magnets. After
    importing cross_section.dxf into FEMM, drop a block label at each (x, y) with
    the listed property. Winding is modeled as ONE copper region per slot with
    +/-conductors_per_slot turns (standard FEA equivalent of the discrete bars)."""
    g = em_design.derive(p)
    s, r = p.stator, p.rotor
    rows: List[Dict[str, Any]] = []

    def add(x, y, material, circuit="", turns="", magdir="", note=""):
        rows.append({"x": round(x, 3), "y": round(y, 3), "material": material,
                     "circuit": circuit, "turns": turns, "magdir": magdir, "note": note})

    add((g.slot_body_outer_radius + g.stator_outer_radius) / 2.0, 0.0, "Steel",
        note="stator yoke (lamination BH curve)")
    ext = em_design._magnet_pocket_extent(p, g)
    r_min = ext[1] if ext else g.shaft_radius + 5.0
    add((g.shaft_radius + r_min) / 2.0, 0.0, "Steel", note="rotor core (lamination BH curve)")
    add(g.rotor_outer_radius + r.air_gap / 2.0, 0.0, "Air", note="air gap")
    add(g.shaft_radius / 2.0, 0.0, "Air", note="shaft bore (non-magnetic in EM, or assign shaft steel)")

    # one copper region per slot, turns = +/- conductors_per_slot (sign = belt sign)
    layout = winding_layout(p)
    r_slot = (g.slot_body_inner_radius + g.slot_body_outer_radius) / 2.0
    slot_pitch = 360.0 / s.slot_count
    for i, (ph, sg) in enumerate(layout):
        x, y = _rot(r_slot, 0.0, i * slot_pitch)
        add(x, y, "Copper", circuit=ph, turns=sg * p.winding.conductors_per_slot,
            note="slot %d (%d bars)" % (i, p.winding.conductors_per_slot))

    # magnets: 2 V-arms x poles; magdir = magnetisation angle, N/S alternating per pole
    arms = _bp.magnet_polygons(p, g)
    th = _bp.v_tilt_deg(r.v_angle_deg)
    base_dir = [-th, th]            # +Y arm, -Y arm thickness-axis angle
    pole_pitch = 360.0 / r.pole_count
    for k in range(r.pole_count):
        flip = 180.0 if (k % 2) else 0.0
        for arm_idx, poly in enumerate(arms):
            cx = sum(pt[0] for pt in poly) / len(poly)
            cy = sum(pt[1] for pt in poly) / len(poly)
            x, y = _rot(cx, cy, k * pole_pitch)
            magdir = (base_dir[arm_idx] + k * pole_pitch + flip) % 360.0
            add(x, y, "NdFeB", magdir=round(magdir, 1),
                note="pole %d %s arm (%s)" % (k, "+Y" if arm_idx == 0 else "-Y",
                                              "N" if flip == 0 else "S"))
    return rows


# --------------------------------------------------------------------------- #
# package writer
# --------------------------------------------------------------------------- #
def write_package(p: MotorParams, out_dir: str = "fea") -> List[str]:
    import os
    os.makedirs(out_dir, exist_ok=True)
    spec = fea_spec(p)
    blueprint = _bp.generate(p)
    written = []

    spec_path = os.path.join(out_dir, "fea_spec.json")
    with open(spec_path, "w", encoding="utf-8") as fh:
        json.dump(spec, fh, indent=2)
    written.append(spec_path)

    dxf_path = os.path.join(out_dir, "cross_section.dxf")
    with open(dxf_path, "w", encoding="utf-8") as fh:
        fh.write(to_dxf(blueprint))
    written.append(dxf_path)

    csv_path = os.path.join(out_dir, "winding.csv")
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write("slot,phase,sign\n")
        for i, (ph, sg) in enumerate(winding_layout(p)):
            fh.write("%d,%s,%s\n" % (i, ph, "+" if sg > 0 else "-"))
    written.append(csv_path)

    lbl_path = os.path.join(out_dir, "femm_labels.csv")
    with open(lbl_path, "w", encoding="utf-8") as fh:
        fh.write("x_mm,y_mm,material,circuit,turns,magdir_deg,note\n")
        for row in femm_label_recipe(p):
            fh.write("%s,%s,%s,%s,%s,%s,%s\n" % (
                row["x"], row["y"], row["material"], row["circuit"],
                row["turns"], row["magdir"], row["note"]))
    written.append(lbl_path)
    return written
