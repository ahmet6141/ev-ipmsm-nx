"""Tests for subframe_nx (NX-independent layers): params round-trip + hardpoint
table, engineering load paths / mass / bolt sizing + validation, and the geometry
blueprint's structural integrity + the ICD §7.4.2 mating ties (pads on the chassis
pads, pickup bosses on the suspension inboard hardpoints, tower at the damper top)."""

import json

import pytest

from subframe_nx import blueprint as bp
from subframe_nx import engineering as eng
from subframe_nx.params import SubframeParams, _LEFT_HARDPOINTS_LOCAL

from motor_nx.blueprint import profile_to_world


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def test_params_json_roundtrip():
    p = SubframeParams()
    p2 = SubframeParams.from_json(p.to_json())
    assert p2.to_dict() == p.to_dict()


def test_partial_dict_keeps_defaults():
    p = SubframeParams.from_dict({"tower": {"post_diameter_mm": 80.0}})
    assert p.tower.post_diameter_mm == 80.0
    assert p.tower.seat_diameter_mm == SubframeParams().tower.seat_diameter_mm  # untouched
    assert p.axle == "rear"


def test_overridden_dotted():
    p = SubframeParams().overridden(**{"axle": "front", "pad.bolt_count": 6})
    assert p.axle == "front"
    assert p.pad.bolt_count == 6


def test_expressions_are_numeric_and_unit_tagged():
    for name, val, unit in SubframeParams().expressions():
        assert isinstance(val, float)
        assert unit in ("", "mm", "deg")


def test_hardpoints_match_icd_rear_left_vehicle_coords():
    """The local hardpoint table must reproduce the ICD §7.2 rear-left VEHICLE
    coordinates once the rear axle station (-1437.5,0,0) is added back."""
    p = SubframeParams()  # rear
    hp = p.hardpoints_local("l")
    axle_x = -1437.5
    expect_vehicle = {
        "lower_pickup_fore": (-1310.0, 344.0, 277.0),
        "lower_pickup_aft": (-1550.0, 344.0, 277.0),
        "upper_pickup_fore": (-1324.0, 424.0, 386.0),
        "upper_pickup_aft": (-1564.0, 424.0, 386.0),
        "toe_pickup": (-1606.0, 412.0, 343.0),
        "damper_top": (-1437.0, 427.0, 692.0),
    }
    for nm, (vx, vy, vz) in expect_vehicle.items():
        lx, ly, lz = hp[nm]
        assert (lx + axle_x, ly, lz) == pytest.approx((vx, vy, vz), abs=0.6)


def test_right_side_is_rz180_of_left():
    """The right corner is Rz(180) of the same canonical suspension corner, so its
    subframe-local boss negates BOTH local X and Y of the left (the +-track/2 offset is
    already folded into the per-side values), Z unchanged. (The OLD convention only
    mirrored Y and left X alone -- the four-corner-mismatch bug.)"""
    p = SubframeParams()
    l = p.hardpoints_local("l")
    r = p.hardpoints_local("r")
    for nm in _LEFT_HARDPOINTS_LOCAL:
        assert r[nm][0] == pytest.approx(-l[nm][0])
        assert r[nm][1] == pytest.approx(-l[nm][1])
        assert r[nm][2] == pytest.approx(l[nm][2])


def test_front_variant_is_not_x_mirrored():
    """The suspension is ONE canonical corner reused front and rear (the front axle is
    NOT X-mirrored -- only the placement origin's X changes). So the subframe boss table
    is IDENTICAL front and rear; the assembler places each at its axle station. (The OLD
    convention X-mirrored the front, which -- combined with the suspension not mirroring --
    is exactly what threw three of the four corners off.)"""
    rear = SubframeParams()  # rear
    front = SubframeParams().overridden(axle="front")
    for nm in _LEFT_HARDPOINTS_LOCAL:
        assert front.hardpoints_local("l")[nm] == pytest.approx(rear.hardpoints_local("l")[nm])


# --------------------------------------------------------------------------- #
# engineering
# --------------------------------------------------------------------------- #
def test_default_design_is_buildable():
    assert eng.validate(SubframeParams()) == []


def test_front_variant_is_buildable():
    assert eng.validate(SubframeParams().overridden(axle="front")) == []


def test_load_path_has_sound_margins():
    """The cradle side rail + the pad / pickup bolts must clear a >= 1.5 static SF."""
    g = eng.derive(SubframeParams())
    assert g.side_rail_safety_factor >= 1.5
    assert g.pad_bolt_safety_factor >= 1.5
    assert g.pickup_bolt_safety_factor >= 1.5


def test_tower_reaches_damper_top():
    g = eng.derive(SubframeParams())
    assert g.tower_reach_z_mm == pytest.approx(g.damper_top_z_mm)
    assert g.damper_top_z_mm == pytest.approx(692.0)


def test_higher_corner_load_lowers_safety_factor():
    light = eng.derive(SubframeParams().overridden(corner_vertical_load_n=8000.0))
    heavy = eng.derive(SubframeParams().overridden(corner_vertical_load_n=20000.0))
    assert heavy.side_rail_safety_factor < light.side_rail_safety_factor
    assert heavy.pickup_bolt_safety_factor < light.pickup_bolt_safety_factor


def test_mass_breakdown_positive_and_sums():
    g = eng.derive(SubframeParams())
    assert g.total_mass_kg > 0
    parts = g.cradle_mass_kg + g.pad_post_mass_kg + g.tower_mass_kg + g.boss_mass_kg
    assert g.total_mass_kg == pytest.approx(parts, rel=1e-3)


def test_validate_catches_unknown_axle():
    assert any("axle" in i for i in eng.validate(SubframeParams().overridden(axle="middle")))


def test_validate_catches_pad_off_chassis_y():
    """ICD §7.4.2: the pads must align to the chassis rail centre-line y=±585."""
    p = SubframeParams().overridden(**{"pad.pad_y_mm": 500.0})
    assert any("pad_y_mm" in i and "585" in i for i in eng.validate(p))


def test_validate_catches_thin_beam_wall():
    p = SubframeParams().overridden(**{"cradle.beam_wall_mm": 40.0})  # 2*40 > 60
    assert any("beam_wall" in i for i in eng.validate(p))


def test_validate_catches_tower_below_upper_pickups():
    """The damper top must rise above the upper pickups (a real shock tower)."""
    p = SubframeParams().overridden(**{"cradle.base_plane_z_mm": 700.0})  # above damper top
    issues = eng.validate(p)
    assert any("base_plane" in i for i in issues)


# --------------------------------------------------------------------------- #
# blueprint -- structural integrity
# --------------------------------------------------------------------------- #
def test_blueprint_schema_and_json():
    blue = bp.generate(SubframeParams())
    assert blue["schema"] == "subframe_nx.blueprint/1"
    assert blue["units"] == "mm" and blue["axis"] == "X"
    json.loads(bp.to_json(blue))  # serialisable
    assert len(blue["build_steps"]) > 20


def test_every_boolean_targets_an_existing_create():
    """A subtract/unite must target a body that an earlier create step made."""
    blue = bp.generate(SubframeParams())
    created = set()
    for s in blue["build_steps"]:
        if s["boolean"] == "create":
            created.add(s["id"])
        elif s["boolean"] in ("subtract", "unite"):
            assert s["target"] in created, "%s targets missing body %s" % (s["id"], s["target"])


def test_both_sides_present():
    ids = [s["id"] for s in bp.generate(SubframeParams())["build_steps"]]
    assert any(i.endswith("_l") for i in ids) and any(i.endswith("_r") for i in ids)


def test_all_required_components_present():
    roles = {s["role"] for s in bp.generate(SubframeParams())["build_steps"]}
    assert "cradle_rail" in roles            # perimeter cradle
    assert "pad_post" in roles               # chassis pad interface
    assert "pickup_boss" in roles            # suspension pickups
    assert "shock_tower" in roles            # damper-top support
    assert "eaxle_mount" in roles            # e-axle / diff mounts


def test_eaxle_mounts_can_be_disabled():
    blue = bp.generate(SubframeParams().overridden(**{"eaxle.enabled": False}))
    assert not any(s["role"] == "eaxle_mount" for s in blue["build_steps"])


# --------------------------------------------------------------------------- #
# ICD §7.4.2 mating ties -- read straight off the built geometry (world coords)
# --------------------------------------------------------------------------- #
def _create(blue, sid):
    for s in blue["build_steps"]:
        if s["id"] == sid and s["boolean"] == "create":
            return s
    raise AssertionError("no create body %s" % sid)


def _step(blue, sid):
    """Find a build step by id regardless of its boolean op (united pad flanges /
    tower seats are not create steps but still carry their world placement)."""
    for s in blue["build_steps"]:
        if s["id"] == sid:
            return s
    raise AssertionError("no build step %s" % sid)


def _cyl_centre(step):
    """Local centre of an axis-placed cylinder = origin3 + axis * length/2."""
    o, a, L = step["origin3"], step["axis"], step["length"]
    return tuple(o[i] + a[i] * L / 2.0 for i in range(3))


def test_pickup_bosses_land_on_the_hardpoints():
    """Every pickup boss centre must sit on its suspension inboard hardpoint
    (ICD §7.4.2: the suspension arm inboard end bolts here -- no floating arm)."""
    p = SubframeParams()
    blue = bp.generate(p)
    for side in ("l", "r"):
        hp = p.hardpoints_local(side)
        for nm in ("lower_pickup_fore", "lower_pickup_aft", "upper_pickup_fore",
                   "upper_pickup_aft", "toe_pickup"):
            c = _cyl_centre(_create(blue, "pickup_boss_%s_%s" % (nm, side)))
            assert c == pytest.approx(hp[nm], abs=1e-6), "boss %s_%s off hardpoint" % (nm, side)


def test_tower_seat_sits_at_the_damper_top():
    """The shock-tower seat lower face must land at the damper/strut-top hardpoint so
    the spring/damper top is supported (no floating spring, ICD §7.2)."""
    p = SubframeParams()
    blue = bp.generate(p)
    for side in ("l", "r"):
        seat = _step(blue, "tower_seat_%s" % side)
        assert tuple(seat["origin3"]) == pytest.approx(p.hardpoints_local(side)["damper_top"], abs=1e-6)


def test_pad_flanges_land_on_chassis_pad_stations():
    """ICD §7.4.2: the four chassis-pad flanges must sit at the chassis rail centre-line
    y=±585 on the rail-top mating plane (the subframe's pad_z)."""
    p = SubframeParams()
    blue = bp.generate(p)
    for fa in ("fore", "aft"):
        for side in ("l", "r"):
            flange = _step(blue, "pad_flange_%s_%s" % (fa, side))
            expect = p.pad_centre_local(fa, side)
            assert tuple(flange["origin3"]) == pytest.approx(expect, abs=1e-6)
            assert abs(flange["origin3"][1]) == pytest.approx(585.0)
            assert flange["origin3"][2] == pytest.approx(390.0)


def test_cradle_perimeter_brackets_the_pad_y_via_world_bbox():
    """The perimeter cradle (side rails + crossbeams) must span the pad lateral width:
    its world bounding box in Y must reach out to ±pad_y. Computed from the prism
    sections via the NX-free profile_to_world twin (a real world-bbox check)."""
    p = SubframeParams()
    blue = bp.generate(p)
    ys = []
    for s in blue["build_steps"]:
        if s["role"] in ("cradle_rail", "cradle_cross") and s["boolean"] == "create":
            near = profile_to_world([tuple(q) for q in s["profile"]], tuple(s["origin3"]),
                                    tuple(s["axis"]), tuple(s["u_dir"]))
            a, L = s["axis"], s["length"]
            far = [(x + a[0] * L, y + a[1] * L, z + a[2] * L) for (x, y, z) in near]
            ys += [q[1] for q in near + far]
    assert max(ys) >= p.pad.pad_y_mm - 1.0
    assert min(ys) <= -(p.pad.pad_y_mm - 1.0)


def test_chassis_pad_alignment_against_chassis_package():
    """Cross-check: the subframe pad Y/Z must equal what chassis_nx actually builds for
    the subframe mount pad (rail centre-line y, rail-top z) at the same axle station."""
    from chassis_nx.blueprint import subframe_pad_centre
    from chassis_nx.params import ChassisParams
    cp = ChassisParams()
    for axle in ("rear", "front"):
        p = SubframeParams().overridden(axle=axle)
        veh = subframe_pad_centre(cp, axle, "l")
        assert p.pad.pad_y_mm == pytest.approx(abs(veh[1]))
        assert p.pad.pad_z_mm == pytest.approx(veh[2])


def test_pickup_bosses_match_suspension_inboard_hardpoints():
    """ICD §7.4.2 coincidence: mapping the suspension corner's inboard pickups (hub-
    datumed) into the subframe local frame must land on the subframe bosses (the
    headline 'no floating arm' tie)."""
    from suspension_nx.engineering import hardpoints as s_hardpoints
    from suspension_nx.params import SuspensionParams
    s_hp = s_hardpoints(SuspensionParams())
    p = SubframeParams()  # rear-left corner
    sub = p.hardpoints_local("l")
    # hub-frame -> subframe-local: rear-left HUB_CENTRE=(-1437.5,790,335), axle station
    # =(-1437.5,0,0) -> offset (0,790,335).
    offset = (0.0, 790.0, 335.0)
    for nm in ("lower_pickup_fore", "lower_pickup_aft", "upper_pickup_fore",
               "upper_pickup_aft", "toe_pickup"):
        sx, sy, sz = s_hp[nm]
        mapped = (sx + offset[0], sy + offset[1], sz + offset[2])
        d = sum((mapped[i] - sub[nm][i]) ** 2 for i in range(3)) ** 0.5
        assert d < 5.0, "boss %s is %.1f mm from the suspension hardpoint" % (nm, d)
