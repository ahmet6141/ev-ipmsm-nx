"""Tests for the arbitrary-orientation 'prism' primitive and axis-placed
cylinder/tube support added to the shared geometry layer (motor_nx.blueprint).

These exercise the NX-FREE math twins (prism_frame / profile_to_world) and the
BuildStep schema fields the NX builder consumes -- so beams/plates in true vehicle
coordinates have a regression anchor without needing a live NX session.
"""

import math

from motor_nx.blueprint import BuildStep, prism_frame, profile_to_world


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def test_prism_frame_orthonormal_right_handed():
    for axis, u_dir in [((1, 0, 0), (0, 0, 1)), ((0, 1, 0), (1, 0, 0)),
                        ((0, 0, 1), (1, 0, 0)), ((1, 1, 0), (0, 0, 1)),
                        ((0, 0, 1), (0, 0, 1))]:  # u_dir parallel to axis -> fallback
        u, v, w = prism_frame(axis, u_dir)
        # orthonormal
        for a in (u, v, w):
            assert abs(_dot(a, a) - 1.0) < 1e-9
        assert abs(_dot(u, v)) < 1e-9
        assert abs(_dot(u, w)) < 1e-9
        assert abs(_dot(v, w)) < 1e-9
        # right-handed: u x v == w
        cross = (u[1] * v[2] - u[2] * v[1],
                 u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0])
        assert all(abs(cross[i] - w[i]) < 1e-9 for i in range(3))
        # w is the (normalised) axis
        n = math.sqrt(sum(c * c for c in axis))
        assert all(abs(w[i] - axis[i] / n) < 1e-9 for i in range(3))


def test_profile_to_world_beam_along_x():
    """A rail extruded along +X: the section (u,v) lays out in Y-Z; the +X length
    is applied by the extrude, so the section's world X is fixed at the origin X."""
    rect = [(-60, -10), (60, -10), (60, 10), (-60, 10)]  # 120 (u) x 20 (v)
    pts = profile_to_world(rect, (1437.5, 790.0, 335.0), axis=(1, 0, 0), u_dir=(0, 1, 0))
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    assert max(xs) - min(xs) < 1e-6        # section is perpendicular to +X
    assert abs(max(ys) - min(ys) - 120.0) < 1e-6   # u spans 120 in Y
    assert abs(max(zs) - min(zs) - 20.0) < 1e-6    # v spans 20 in Z
    assert all(abs(x - 1437.5) < 1e-6 for x in xs)


def test_buildstep_prism_roundtrips_origin_and_axis():
    step = BuildStep(
        id="rail_l", role="frame_rail", kind="prism", boolean="create",
        body_name="Frame_Rail_L", profile=[(-50, -10), (50, -10), (50, 10), (-50, 10)],
        origin3=(-1500.0, 790.0, 300.0), axis=(1.0, 0.0, 0.0), u_dir=(0.0, 1.0, 0.0),
        length=3000.0)
    d = step.as_dict()
    assert d["kind"] == "prism"
    assert d["origin3"] == (-1500.0, 790.0, 300.0)
    assert d["axis"] == (1.0, 0.0, 0.0)
    assert d["u_dir"] == (0.0, 1.0, 0.0)
    assert d["profile"] == [[-50, -10], [50, -10], [50, 10], [-50, 10]]


def test_buildstep_defaults_keep_legacy_plus_z():
    """A step that sets neither origin3 nor a non-Z axis must serialise as the
    legacy +Z primitive (origin3 is None, axis is +Z) so the proven path is used."""
    step = BuildStep(id="s", role="r", kind="cylinder", outer_radius=10.0, length=5.0)
    d = step.as_dict()
    assert d["origin3"] is None
    assert d["axis"] == (0.0, 0.0, 1.0)
