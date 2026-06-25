"""Siemens NX builder -- consumes a blueprint and replays its build steps with
the NXOpen Python API. RUN INSIDE NX (headless) via run_journal.exe:

    "%UGII_ROOT_DIR%\\run_journal.exe" nx_builder.py -args <blueprint.json> <out.prt> [step|parasolid|both|none] [parts]

Export mode DEFAULTS to "step" (AP242, reliable). Parasolid (.x_t) is opt-in
("parasolid"/"both") and non-fatal -- it can fault + poison the session on large
parts, so STEP carries the full assembly by default.

This is the ONLY module that imports NXOpen, so it never runs under the plain
CPython test interpreter -- it is exercised inside a real NX session.

The NXOpen call sequences below are the *hardened* versions verified by the
research workflow against the NX 1900..2406 API (current pattern/boolean/export
member names, batch part-creation, mandatory Update.DoUpdate). See
docs/NX_AUTOMATION.md for the per-call rationale and known version-drift points.

Design of the builder
    * Every build step becomes one (or two) NX features; a small id->Body
      registry lets later "subtract"/"unite" steps target an earlier body.
    * Construction curves are dumb curves consumed by Sections; the STEP/Parasolid
      export selects SOLID BODIES only, so the export stays clean.
    * The single global "stack_length" expression drives the axial length of the
      active-stack features, so editing it in NX + Update rescales the stack -- a
      taste of in-NX associativity on top of the regenerate-from-params workflow.
    * Build-step kinds: tube / cylinder / extrude / revolve (all on +Z) and "hole"
      (a cylindrical cut on an ARBITRARY axis -- radial coolant/terminal/lifting
      ports and hollow-shaft oil cross-holes). The "hole" branch reuses the proven
      extrude+boolean path with a caller-supplied direction; CONFIRM it on the next
      in-NX smoke run (the manufacturing-assembly features were added after the
      last verified NX 1900..2406 pass -- see docs/NX_AUTOMATION.md).
"""

import json
import math
import os
import sys
import traceback

import NXOpen
import NXOpen.Features
import NXOpen.GeometricUtilities

try:
    import NXOpen.UF
    _UF = NXOpen.UF.UFSession.GetUFSession()
except Exception:  # pragma: no cover - UF nearly always present
    _UF = None

_SESSION = NXOpen.Session.GetSession()
_TOL = 0.001  # modeling distance tolerance (mm)


def _p3(x, y, z):
    """NX 2506 NXOpen requires float (double) coords; passing an int raises
    'Expecting double type, found int'. Coerce every point coordinate here."""
    return NXOpen.Point3d(float(x), float(y), float(z))


def _v3(x, y, z):
    return NXOpen.Vector3d(float(x), float(y), float(z))


# --------------------------------------------------------------------------- #
# session / part bootstrap  (hardened: capture part from Commit, mm units)
# --------------------------------------------------------------------------- #
def _log_window():
    lw = _SESSION.ListingWindow
    lw.Open()
    return lw


def _unpack_part(result):
    """Parts.NewBaseDisplay returns just a Part on NX 2506, but a
    (Part, PartLoadStatus) tuple on some other builds. Handle both."""
    if isinstance(result, tuple):
        part = result[0]
        for extra in result[1:]:
            try:
                extra.Dispose()
            except Exception:
                pass
        return part
    return result


def _mm_unit_values():
    """The 'Millimeters' part-units value in the spellings that carry the right
    type for NX 2506's Parts.NewBaseDisplay (verified on real hardware:
    NXOpen.BasePart.Units / the flattened BasePartUnits -- NOT Part.Units)."""
    vals = []
    for getter in (
        lambda: NXOpen.BasePart.Units.Millimeters,
        lambda: getattr(NXOpen, "BasePartUnits").Millimeters,
    ):
        try:
            vals.append(getter())
        except Exception:
            pass
    return vals


def new_mm_part(full_path, make_displayed=True):
    """Create a NEW millimetre modelling part via Parts.NewBaseDisplay with the
    verified NX 2506 units enum (NXOpen.BasePart.Units.Millimeters).

    NewBaseDisplay refuses to create over an EXISTING/LOADED part name ("File
    already exists") -- which also happens when a prior NX session or an open NX
    GUI still locks the .prt. We delete the stale file when we can, and otherwise
    fall through to a fresh, unused name (path_1.prt, path_2.prt, ...)."""
    d = os.path.dirname(full_path)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    vals = _mm_unit_values()
    if not vals:
        raise RuntimeError("NXOpen.BasePart.Units.Millimeters is not available")
    mm = vals[0]
    base = os.path.splitext(full_path)[0]
    last = None
    for i in range(64):
        candidate = full_path if i == 0 else "%s_%d.prt" % (base, i)
        if os.path.exists(candidate):
            try:
                os.remove(candidate)
            except OSError:
                pass  # locked by another NX session -> the next name will be free
        try:
            return _unpack_part(_SESSION.Parts.NewBaseDisplay(candidate, mm))
        except Exception as exc:
            last = exc
            if "exist" in str(exc).lower():
                continue
            raise
    raise RuntimeError("could not find a free part name (64 tries): %s" % last)


# --------------------------------------------------------------------------- #
# the builder
# --------------------------------------------------------------------------- #
class MotorBuilder:
    def __init__(self, work_part, stack_length, use_nx_patterns=False):
        self.part = work_part
        self.stack_length = float(stack_length)
        # Default OFF: repeated features are expanded into explicit instances using
        # ONLY confirmed-stable API (extrude + boolean). NX PatternFeatureBuilder
        # member names drift between releases; flip this on (after the smoke test
        # confirms it) for a lighter feature tree.
        self.use_nx_patterns = use_nx_patterns
        self.bodies = {}        # step id -> NXOpen.Body
        self._role_counts = {}  # role -> running count, for unique body names
        self._named = []        # every body name actually applied (for a build-log summary)
        self.errors = []
        self.lw = _log_window()
        self._z_axis = None
        self._setup_expressions_unit()

    def log(self, msg):
        self.lw.WriteLine(str(msg))

    # -- expressions ------------------------------------------------------- #
    def _setup_expressions_unit(self):
        self._mm = self.part.UnitCollection.FindObject("MilliMeter")
        try:
            self._deg = self.part.UnitCollection.FindObject("Degrees")
            self._num = self.part.UnitCollection.FindObject("Number")
        except Exception:
            self._deg = self._num = None

    def set_expr(self, name, formula, unit=None):
        """Create-or-edit a named expression (idempotent). Values use RightHandSide
        in the expression's own units, never the part base units."""
        unit = unit or self._mm
        try:
            e = self.part.Expressions.FindObject(name)
            self.part.Expressions.EditWithUnits(e, unit, str(formula))
            return e
        except NXOpen.NXException:
            return self.part.Expressions.CreateWithUnits("%s=%s" % (name, formula), unit)

    def push_expressions(self, expressions):
        """Record the full parameter table in the part for traceability + the one
        load-bearing 'stack_length' expression used to drive axial length."""
        for item in expressions:
            unit = self._mm
            if item["unit"] == "deg" and self._deg is not None:
                unit = self._deg
            elif item["unit"] == "" and self._num is not None:
                unit = self._num
            try:
                self.set_expr(item["name"], item["value"], unit)
            except Exception as exc:  # keep going; expressions are advisory
                self.errors.append("expr %s: %s" % (item["name"], exc))

    # -- low level geometry ------------------------------------------------ #
    def _z_direction(self):
        origin = _p3(0.0, 0.0, 0.0)
        vec = _v3(0.0, 0.0, 1.0)
        return self.part.Directions.CreateDirection(
            origin, vec, NXOpen.SmartObject.UpdateOption.WithinModeling)

    def _z_axis_obj(self):
        if self._z_axis is None:
            uo = NXOpen.SmartObject.UpdateOption.WithinModeling
            p = _p3(0.0, 0.0, 0.0)
            d = self.part.Directions.CreateDirection(p, _v3(0.0, 0.0, 1.0), uo)
            pt = self.part.Points.CreatePoint(p)
            self._z_axis = self.part.Axes.CreateAxis(pt, d, uo)
        return self._z_axis

    def _lines_from_polygon(self, points, plane_z=0.0, in_xz=False, start_angle_deg=0.0):
        """Dumb-curve closed loop. in_xz: place points as (r, 0, z) -- or, when
        start_angle_deg != 0, in the half-plane at that angle (r cos a, r sin a, z)
        -- for a revolve profile; otherwise (x, y, plane_z) for an extrude on a Z plane."""
        ca = math.cos(math.radians(start_angle_deg))
        sa = math.sin(math.radians(start_angle_deg))
        curves = []
        n = len(points)
        for i in range(n):
            a = points[i]
            b = points[(i + 1) % n]
            if in_xz:
                p0 = _p3(a[0] * ca, a[0] * sa, a[1])
                p1 = _p3(b[0] * ca, b[0] * sa, b[1])
            else:
                p0 = _p3(a[0], a[1], plane_z)
                p1 = _p3(b[0], b[1], plane_z)
            curves.append(self.part.Curves.CreateLine(p0, p1))
        return curves

    def _circle_curve(self, cx, cy, radius, plane_z=0.0):
        c = _p3(cx, cy, plane_z)
        xdir = _v3(1.0, 0.0, 0.0)
        ydir = _v3(0.0, 1.0, 0.0)
        return self.part.Curves.CreateArc(c, xdir, ydir, float(radius), 0.0, 2.0 * math.pi)

    @staticmethod
    def _perp_basis(axis):
        """Two orthonormal vectors (u, v) spanning the plane perpendicular to
        `axis`, plus the normalised axis -- used to draw a circle on an arbitrary
        plane (radial / oil-hole 'hole' steps)."""
        ax, ay, az = axis
        n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        ax, ay, az = ax / n, ay / n, az / n
        helper = (0.0, 0.0, 1.0) if abs(az) < 0.9 else (1.0, 0.0, 0.0)
        ux = ay * helper[2] - az * helper[1]
        uy = az * helper[0] - ax * helper[2]
        uz = ax * helper[1] - ay * helper[0]
        un = math.sqrt(ux * ux + uy * uy + uz * uz) or 1.0
        ux, uy, uz = ux / un, uy / un, uz / un
        vx = ay * uz - az * uy
        vy = az * ux - ax * uz
        vz = ax * uy - ay * ux
        return (ux, uy, uz), (vx, vy, vz), (ax, ay, az)

    def _circle_curve_on_axis(self, base, axis, radius):
        """A full circle of `radius` centred at `base`, in the plane normal to `axis`."""
        u, v, _ = self._perp_basis(axis)
        return self.part.Curves.CreateArc(
            _p3(*base), _v3(*u), _v3(*v), float(radius), 0.0, 2.0 * math.pi)

    @staticmethod
    def _prism_frame(axis, u_dir):
        """Right-handed (u, v, w) frame for a prism: w = unit(axis) (extrude
        direction), u = unit component of u_dir perpendicular to w, v = w x u.
        Falls back to a stable perpendicular when u_dir is parallel to axis.
        Mirrors motor_nx.blueprint.prism_frame so the in-NX geometry matches the
        NX-free world bounding boxes used by the tests."""
        ax, ay, az = axis
        n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        wx, wy, wz = ax / n, ay / n, az / n
        ux, uy, uz = u_dir
        d = ux * wx + uy * wy + uz * wz
        ux, uy, uz = ux - d * wx, uy - d * wy, uz - d * wz
        if math.sqrt(ux * ux + uy * uy + uz * uz) < 1e-9:
            helper = (0.0, 0.0, 1.0) if abs(wz) < 0.9 else (1.0, 0.0, 0.0)
            ux = wy * helper[2] - wz * helper[1]
            uy = wz * helper[0] - wx * helper[2]
            uz = wx * helper[1] - wy * helper[0]
        un = math.sqrt(ux * ux + uy * uy + uz * uz) or 1.0
        ux, uy, uz = ux / un, uy / un, uz / un
        vx = wy * uz - wz * uy
        vy = wz * ux - wx * uz
        vz = wx * uy - wy * ux
        return (ux, uy, uz), (vx, vy, vz), (wx, wy, wz)

    def _lines_from_profile_3d(self, profile_uv, origin3, axis, u_dir):
        """Closed loop of dumb lines for a 2D (u, v) profile placed at `origin3`,
        oriented by (axis, u_dir) -- the arbitrary-orientation analogue of
        :meth:`_lines_from_polygon`. Used by the 'prism' build step."""
        (uxx, uxy, uxz), (vxx, vxy, vxz), _ = self._prism_frame(axis, u_dir)
        ox, oy, oz = origin3
        pts = [(ox + pu * uxx + pv * vxx,
                oy + pu * uxy + pv * vxy,
                oz + pu * uxz + pv * vxz) for (pu, pv) in profile_uv]
        curves = []
        n = len(pts)
        for i in range(n):
            curves.append(self.part.Curves.CreateLine(_p3(*pts[i]), _p3(*pts[(i + 1) % n])))
        return curves

    def _loft_twist(self, step, op, target):
        """Loft (through-curves) the 2D `profile` between origin3 (rotated start_twist_deg)
        and origin3 + axis*length (rotated start_twist_deg + twist_deg) -> a TRUE twisted
        solid (helical gear lead). The section rotates linearly along the axis. Honours an
        inline boolean (create/unite/subtract) when ThroughCurvesBuilder exposes one (it
        shares the Feature builder's BooleanOperation). Returns (feature, boolean_applied)."""
        profile = step["profile"]
        base = self._axis_base(step)
        axis = step.get("axis", [0.0, 0.0, 1.0])
        u_dir = step.get("u_dir", [1.0, 0.0, 0.0])
        length = float(step.get("length", 0.0))
        twist = float(step.get("twist_deg", 0.0))
        start = float(step.get("start_twist_deg", 0.0))
        _, _, w = self._prism_frame(axis, u_dir)          # w = unit axis
        top = (base[0] + w[0] * length, base[1] + w[1] * length, base[2] + w[2] * length)
        bot_curves = self._lines_from_profile_3d(self._rotate2d(profile, start), base, axis, u_dir)
        top_curves = self._lines_from_profile_3d(self._rotate2d(profile, start + twist), top, axis, u_dir)
        tcb = self.part.Features.CreateThroughCurvesBuilder(NXOpen.Features.Feature.Null)
        tcb.SectionsList.Append(self._section(bot_curves))
        tcb.SectionsList.Append(self._section(top_curves))
        applied = False
        if op in ("subtract", "unite") and target is not None:
            try:
                tcb.BooleanOperation.Type = self._bool_type(op)
                tcb.BooleanOperation.SetTargetBodies([target])
                applied = True
            except Exception:
                applied = False
        feat = tcb.CommitFeature()
        tcb.Destroy()
        return feat, applied

    def _extrude_on_axis(self, curves, base, axis, length, op, target_body):
        """Extrude a closed curve loop along an ARBITRARY axis (not just +Z).
        Mirrors :meth:`_extrude` but with a caller-supplied direction -- the radial
        'hole' primitive (ports, oil cross-holes) needs a non-Z extrude."""
        ext = self.part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
        ext.Section = self._section(curves)
        ext.Direction = self.part.Directions.CreateDirection(
            _p3(*base), _v3(*axis), NXOpen.SmartObject.UpdateOption.WithinModeling)
        ext.Limits.StartExtend.Value.RightHandSide = "0"
        ext.Limits.EndExtend.Value.RightHandSide = repr(float(length))
        ext.BooleanOperation.Type = self._bool_type(op)
        if target_body is not None and op in ("subtract", "unite"):
            ext.BooleanOperation.SetTargetBodies([target_body])
        feat = ext.CommitFeature()
        ext.Destroy()
        return feat

    def _section(self, curves):
        section = self.part.Sections.CreateSection(0.0095, _TOL, 0.5)
        section.AllowSelfIntersection(False)
        rule = self.part.ScRuleFactory.CreateRuleCurveDumb(curves)  # not the deprecated *BaseCurveDumb
        help_pt = _p3(0.0, 0.0, 0.0)
        null_obj = NXOpen.NXObject.Null
        section.AddToSection([rule], curves[0], null_obj, null_obj, help_pt,
                             NXOpen.Section.Mode.Create, False)
        return section

    def _bool_type(self, op):
        B = NXOpen.GeometricUtilities.BooleanOperation.BooleanType
        return {"create": B.Create, "unite": B.Unite, "subtract": B.Subtract}[op]

    def _extrude(self, curves, z0, length, op, target_body, use_stack_expr):
        """Extrude a closed curve loop along +Z. Returns the created Feature."""
        ext = self.part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
        ext.Section = self._section(curves)
        ext.Direction = self._z_direction()
        ext.Limits.StartExtend.Value.RightHandSide = "0"
        if use_stack_expr:
            ext.Limits.EndExtend.Value.RightHandSide = "stack_length"
        else:
            ext.Limits.EndExtend.Value.RightHandSide = repr(float(length))
        ext.BooleanOperation.Type = self._bool_type(op)
        if target_body is not None and op in ("subtract", "unite"):
            ext.BooleanOperation.SetTargetBodies([target_body])
        feat = ext.CommitFeature()
        ext.Destroy()
        return feat

    def _revolve(self, curves, op, target_body, angle_deg=360.0):
        rev = self.part.Features.CreateRevolveBuilder(NXOpen.Features.Feature.Null)
        rev.Section = self._section(curves)
        rev.Axis = self._z_axis_obj()
        rev.Limits.StartExtend.Value.RightHandSide = "0"
        rev.Limits.EndExtend.Value.RightHandSide = repr(float(angle_deg))
        rev.BooleanOperation.Type = self._bool_type(op)
        if target_body is not None and op in ("subtract", "unite"):
            rev.BooleanOperation.SetTargetBodies([target_body])
        feat = rev.CommitFeature()
        rev.Destroy()
        return feat

    @staticmethod
    def _rotate2d(points, deg):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        return [[x * c - y * s, x * s + y * c] for x, y in points]

    def _rotate_pt_z(self, p, deg):
        """Rotate a 3D point about the Z axis by `deg` (Z unchanged)."""
        x, y = self._rotate2d([[p[0], p[1]]], deg)[0]
        return (x, y, p[2])

    def _rotate_vec_z(self, v, deg):
        """Rotate a 3D vector about the Z axis by `deg` (Z component unchanged)."""
        x, y = self._rotate2d([[v[0], v[1]]], deg)[0]
        return (x, y, v[2])

    @staticmethod
    def _is_axis_placed(step):
        """True when a tube/cylinder asks for ARBITRARY-axis placement: an explicit
        origin3, or an `axis` that is not the default +Z. (Keeps the proven +Z path
        for every legacy step that sets neither.)"""
        if step.get("origin3") is not None:
            return True
        ax = step.get("axis", [0.0, 0.0, 1.0])
        return abs(ax[0]) > 1e-9 or abs(ax[1]) > 1e-9 or abs(float(ax[2]) - 1.0) > 1e-9

    @staticmethod
    def _axis_base(step):
        """Base point for an axis-placed primitive: origin3 if given, else the legacy
        (cx, cy, z0)."""
        o3 = step.get("origin3")
        if o3 is not None:
            return (float(o3[0]), float(o3[1]), float(o3[2]))
        return (float(step.get("cx", 0.0)), float(step.get("cy", 0.0)), float(step.get("z0", 0.0)))

    @staticmethod
    def _count_pitch_spacing():
        """The 'count + pitch' spacing-type enum value, resilient to NX version
        drift. NX 2506 uses PatternSpacing.SpacingType.Offset; older builds used
        SpacingTypeEnum.CountAndPitch. Return the first that resolves."""
        ps = NXOpen.GeometricUtilities.PatternSpacing
        for enum_name, val_name in (("SpacingType", "Offset"),
                                    ("SpacingTypeEnum", "CountAndPitch"),
                                    ("SpacingType", "Pitch")):
            enum_cls = getattr(ps, enum_name, None)
            if enum_cls is not None and hasattr(enum_cls, val_name):
                return getattr(enum_cls, val_name)
        return None

    def _circular_pattern(self, feature, count, angle_deg):
        """OPTIONAL lighter-tree path (self.use_nx_patterns): one feature + an NX
        circular pattern about Z. Member names per the NX 2506 verification."""
        pfb = self.part.Features.CreatePatternFeatureBuilder(NXOpen.Features.Feature.Null)
        pfb.PatternService.PatternType = (
            NXOpen.GeometricUtilities.PatternDefinition.PatternEnum.Circular)
        pfb.FeatureList.Add([feature])
        circ = pfb.PatternService.CircularDefinition
        circ.RotationAxis = self._z_axis_obj()
        space = self._count_pitch_spacing()
        if space is not None:
            circ.AngularSpacing.SpaceType = space
        circ.AngularSpacing.NCopies.RightHandSide = str(int(count))
        circ.AngularSpacing.PitchDistance.RightHandSide = repr(float(angle_deg))
        feat = pfb.CommitFeature()
        pfb.Destroy()
        return feat

    @staticmethod
    def _feature_body(feature):
        bodies = feature.GetBodies()
        return bodies[0] if bodies else None

    def _register(self, step_id, i, body):
        """Register a created body. The first instance keeps the step id (so later
        booleans can target it); extra pattern instances get a derived id."""
        self.bodies[step_id if i == 0 else "%s#%d" % (step_id, i)] = body
        self._name_body(step_id, i, body)

    def _name_body(self, step_id, i, body):
        """Give each solid body a material-role DISPLAY NAME so the FEA pre/post can
        select bodies by role and assign materials / build mesh collectors without
        hand-identifying every body. Names: STATOR_STEEL, ROTOR_STEEL, MAGNET_nnn,
        COIL_nnn, SHAFT, HOUSING. Single-body roles drop the index. NXOpen Body.SetName
        is guarded -- a failure is advisory (the body still builds)."""
        if body is None:
            return
        role = _ROLE_NAME.get(_component_of(step_id))
        if role is None:
            return
        singular = role in ("STATOR_STEEL", "ROTOR_STEEL", "SHAFT", "HOUSING")
        if singular:
            name = role
        else:
            # a running per-role counter -> globally-unique MAGNET_000.., COIL_000..
            # (several build-steps map to the same role, so the per-step index would clash)
            n = self._role_counts.get(role, 0)
            self._role_counts[role] = n + 1
            name = "%s_%03d" % (role, n)
        try:
            body.SetName(name)
            self._named.append(name)
        except Exception as exc:  # advisory -- naming must never abort the build
            self.errors.append("name %s#%d: %s" % (step_id, i, exc))

    # -- per-kind step handlers ------------------------------------------- #
    def build_step(self, step):
        kind = step["kind"]
        op = step["boolean"]
        z0 = float(step.get("z0", 0.0))
        length = float(step.get("length", 0.0))
        target = self.bodies.get(step["target"]) if step.get("target") else None
        # bind the axial length to the NX 'stack_length' expression ONLY for the
        # steps the blueprint flags as active-stack (laminations/slots/conductors);
        # discrete bodies (magnets/end-windings/housing) stay literal. (Inferring
        # this from z0/length wrongly bound single-segment magnets to stack_length.)
        use_stack = bool(step.get("drive_with_stack", False))
        if op == "create" and step.get("target"):
            self.errors.append("WARN %s: create op should not target a body" % step["id"])
        # A subtract/unite whose target body never got created (its create step
        # failed) would otherwise raise the cryptic "Invalid boolean type". Skip it
        # with a clear, root-cause-pointing message so one failed create does not
        # cascade into dozens of misleading errors.
        if op in ("subtract", "unite") and step.get("target") and target is None:
            raise RuntimeError(
                "SKIPPED: target body '%s' does not exist -- its create step failed "
                "earlier (see the first FAIL above)" % step["target"])

        if kind == "tube":
            if self._is_axis_placed(step):
                # tube coaxial with an ARBITRARY axis through origin3 (lateral bar,
                # inclined sleeve): outer cylinder-on-axis (create) - inner (subtract).
                base = self._axis_base(step)
                axis = step.get("axis", [0.0, 0.0, 1.0])
                outer = self._circle_curve_on_axis(base, axis, step["outer_radius"])
                feat = self._extrude_on_axis([outer], base, axis, length, "create", None)
                body = self._feature_body(feat)
                inner = self._circle_curve_on_axis(base, axis, step["inner_radius"])
                self._extrude_on_axis([inner], base, axis, length, "subtract", body)
                self.bodies[step["id"]] = body
                self._name_body(step["id"], 0, body)
            else:
                outer = self._circle_curve(0.0, 0.0, step["outer_radius"], z0)
                feat = self._extrude([outer], z0, length, "create", None, use_stack)
                body = self._feature_body(feat)
                inner = self._circle_curve(0.0, 0.0, step["inner_radius"], z0)
                self._extrude([inner], z0, length, "subtract", body, use_stack)
                self.bodies[step["id"]] = body
                self._name_body(step["id"], 0, body)

        elif kind == "prism":
            # extrude a 2D (u,v) profile along an ARBITRARY axis at an arbitrary world
            # origin -- beams/plates in true vehicle coordinates. A circular pattern
            # about Z rotates BOTH the origin and the (axis, u_dir) in XY (like 'hole').
            count = int(step.get("pattern_count", 1))
            angle = step.get("pattern_angle_deg", 0.0)
            base = self._axis_base(step)
            axis = step.get("axis", [0.0, 0.0, 1.0])
            u_dir = step.get("u_dir", [1.0, 0.0, 0.0])
            for i in range(max(1, count)):
                o3 = self._rotate_pt_z(base, i * angle)
                ax = self._rotate_vec_z(axis, i * angle)
                ud = self._rotate_vec_z(u_dir, i * angle)
                curves = self._lines_from_profile_3d(step["profile"], o3, ax, ud)
                feat = self._extrude_on_axis(curves, o3, ax, length, op, target)
                if op == "create":
                    self._register(step["id"], i, self._feature_body(feat))

        elif kind == "loft_twist":
            # TRUE twisted/helical solid: loft the profile between two ends, the top
            # rotated by twist_deg about the axis (helical gear lead). Optional inline boolean.
            feat, applied = self._loft_twist(step, op, target)
            if op == "create":
                self.bodies[step["id"]] = self._feature_body(feat)
                self._name_body(step["id"], 0, self.bodies[step["id"]])
            elif not applied:
                # inline boolean unavailable -> the loft body stands alone; flag it so the
                # caller can switch to a separate boolean step (rather than silently leaking).
                self.errors.append(
                    "WARN %s: loft_twist could not apply inline %s; body left standalone"
                    % (step["id"], op))

        elif kind == "cylinder":
            count = int(step.get("pattern_count", 1))
            angle = step.get("pattern_angle_deg", 0.0)
            cx0, cy0 = step.get("cx", 0.0), step.get("cy", 0.0)
            if self._is_axis_placed(step):
                # cylinder coaxial with an ARBITRARY axis through origin3 (inclined
                # damper, lateral stub). Circular pattern about Z like 'hole'.
                base = self._axis_base(step)
                axis = step.get("axis", [0.0, 0.0, 1.0])
                for i in range(max(1, count)):
                    o3 = self._rotate_pt_z(base, i * angle)
                    ax = self._rotate_vec_z(axis, i * angle)
                    circ = self._circle_curve_on_axis(o3, ax, step["outer_radius"])
                    feat = self._extrude_on_axis([circ], o3, ax, length, op, target)
                    if op == "create":
                        self._register(step["id"], i, self._feature_body(feat))
            elif count > 1 and self.use_nx_patterns:
                circ = self._circle_curve(cx0, cy0, step["outer_radius"], z0)
                feat = self._extrude([circ], z0, length, op, target, use_stack)
                if op == "create":
                    self.bodies[step["id"]] = self._feature_body(feat)
                    self._name_body(step["id"], 0, self.bodies[step["id"]])
                self._circular_pattern(feat, count, angle)
            else:  # explicit instances (default, robust)
                for i in range(max(1, count)):
                    cx, cy = self._rotate2d([[cx0, cy0]], i * angle)[0]
                    circ = self._circle_curve(cx, cy, step["outer_radius"], z0)
                    feat = self._extrude([circ], z0, length, op, target, use_stack)
                    if op == "create":
                        self._register(step["id"], i, self._feature_body(feat))

        elif kind == "extrude":
            count = int(step.get("pattern_count", 1))
            angle = step.get("pattern_angle_deg", 0.0)
            if count > 1 and self.use_nx_patterns:
                curves = self._lines_from_polygon(step["profile"], plane_z=z0)
                feat = self._extrude(curves, z0, length, op, target, use_stack)
                if op == "create":
                    self.bodies[step["id"]] = self._feature_body(feat)
                    self._name_body(step["id"], 0, self.bodies[step["id"]])
                self._circular_pattern(feat, count, angle)
            else:  # explicit instances (default, robust)
                for i in range(max(1, count)):
                    prof = self._rotate2d(step["profile"], i * angle)
                    curves = self._lines_from_polygon(prof, plane_z=z0)
                    feat = self._extrude(curves, z0, length, op, target, use_stack)
                    if op == "create":
                        self._register(step["id"], i, self._feature_body(feat))

        elif kind == "revolve":
            curves = self._lines_from_polygon(
                step["profile"], in_xz=True,
                start_angle_deg=float(step.get("start_angle_deg", 0.0)))
            feat = self._revolve(curves, op, target,
                                 angle_deg=float(step.get("angle_deg", 360.0)))
            if op == "create":
                self.bodies[step["id"]] = self._feature_body(feat)
                self._name_body(step["id"], 0, self.bodies[step["id"]])

        elif kind == "hole":
            # cylindrical hole on an ARBITRARY axis (radial coolant/terminal/lifting
            # ports, hollow-shaft oil cross-holes). A circular pattern rotates BOTH
            # the base point and the axis about Z. Holes are cuts -> never registered.
            count = int(step.get("pattern_count", 1))
            angle = step.get("pattern_angle_deg", 0.0)
            bx, by = step.get("cx", 0.0), step.get("cy", 0.0)
            bz = z0
            ax, ay, az = step.get("axis", [0.0, 0.0, 1.0])
            radius = step["outer_radius"]
            for i in range(max(1, count)):
                cx, cy = self._rotate2d([[bx, by]], i * angle)[0]
                rax, ray = self._rotate2d([[ax, ay]], i * angle)[0]
                base = (cx, cy, bz)
                axis = (rax, ray, az)
                circ = self._circle_curve_on_axis(base, axis, radius)
                self._extrude_on_axis([circ], base, axis, length, op, target)

        else:
            raise ValueError("unknown build-step kind: %s" % kind)

    def build(self, blueprint):
        self.push_expressions(blueprint.get("expressions", []))
        for step in blueprint["build_steps"]:
            mark = _SESSION.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, step["id"])
            try:
                self.build_step(step)
                _SESSION.UpdateManager.DoUpdate(mark)
                self.log("OK   %-22s (%s %s)" % (step["id"], step["kind"], step["boolean"]))
            except Exception as exc:
                try:
                    _SESSION.UndoToMark(mark, step["id"])
                except Exception:
                    pass  # undo stack may be unavailable; keep going
                msg = "FAIL %-22s : %s" % (step["id"], exc)
                self.log(msg)
                self.errors.append(msg)
        _SESSION.UpdateManager.DoUpdate(
            _SESSION.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "final"))
        n_solids = sum(1 for b in self.part.Bodies if b.IsSolidBody)
        self.log("built %d solid bodies, %d step error(s)" % (n_solids, len(self.errors)))
        # Diagnose a corrupted modeling session: if even the basic CREATE steps fault
        # with the generic Parasolid "please report fault" and NOTHING built, the NX
        # session itself is poisoned (commonly carried over from a prior Parasolid /
        # modeler fault in the SAME session). A code bug would fail specific steps,
        # not every create -- so the fix is to restart NX, not to edit the journal.
        if n_solids == 0 and any("please report fault" in e.lower() for e in self.errors):
            self.log("HINT: 0 bodies built and basic CREATE steps hit 'please report fault'. "
                     "This is a CORRUPTED NX modeling session (often left by a prior modeler/Parasolid "
                     "fault). Fully CLOSE and RESTART NX (not just replay the journal), then run again. "
                     "The geometry itself is unchanged and built cleanly in earlier sessions.")
        # explicit naming confirmation (so the role names are visible in the build log)
        if self._named:
            by_role = {}
            for nm in self._named:
                key = nm.rsplit("_", 1)[0] if nm[-1:].isdigit() else nm
                by_role[key] = by_role.get(key, 0) + 1
            self.log("named %d bodies: %s" % (
                len(self._named),
                ", ".join("%dx %s" % (by_role[r], r) for r in sorted(by_role))))
        else:
            self.log("WARN no bodies were named -- Body.SetName returned/raised; "
                     "tell me and I will switch to named LAYERS instead.")

    def export_parts(self, base, flavor="ap242"):
        """Export each manufacturable COMPONENT as its own STEP file (piece-by-piece
        production hand-off): Stator_Lamination, Rotor_Lamination, Magnets, Winding,
        Shaft, Housing. Bodies are grouped from the build-step registry by role."""
        groups = {}
        for sid, body in self.bodies.items():
            comp = _component_of(sid)
            if comp and body is not None:
                groups.setdefault(comp, []).append(body)
        for comp, bodies in sorted(groups.items()):
            path = "%s_%s.stp" % (base, comp)
            try:
                export_step(self.part, path, flavor, bodies)
                self.log("part export: %-18s %d body(ies) -> %s" % (comp, len(bodies), path))
            except Exception:
                self.log("part export FAILED %s:\n%s" % (comp, traceback.format_exc()))


# --------------------------------------------------------------------------- #
# export  (hardened: solid-body selection, Parasolid fallback fixed)
# --------------------------------------------------------------------------- #
def _save(part):
    part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
              NXOpen.BasePart.CloseAfterSave.FalseValue)


def export_step(part, out_path, flavor="ap242", bodies=None):
    """Export STEP. bodies=None -> all solid bodies; else just the given bodies
    (used for per-part / piece-by-piece production hand-off files)."""
    _save(part)
    if os.path.exists(out_path):
        try:
            os.remove(out_path)  # avoid translator "file exists" on re-runs
        except OSError:
            pass
    sc = _SESSION.DexManager.CreateStepCreator()
    sc.ExportAs = {
        "ap203": NXOpen.StepCreator.ExportAsOption.Ap203,
        "ap214": NXOpen.StepCreator.ExportAsOption.Ap214,
        "ap242": NXOpen.StepCreator.ExportAsOption.Ap242,
    }[flavor]
    sc.ObjectTypes.Solids = True
    sc.ObjectTypes.Surfaces = True
    if bodies is None:
        bodies = [b for b in part.Bodies if b.IsSolidBody]
    sc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.SelectedObjects
    sc.ExportSelectionBlock.SelectionComp.Add(bodies)
    sc.InputFile = part.FullPath
    sc.OutputFile = out_path
    sc.FileSaveFlag = False
    sc.LayerMask = "1-256"
    sc.Commit()
    sc.Destroy()


# material-role display names for FEA body identification (set via Body.SetName so
# the bodies are selectable by role in NX / the FEM material + mesh-collector setup)
_ROLE_NAME = {
    "Stator_Lamination": "STATOR_STEEL",
    "Rotor_Lamination": "ROTOR_STEEL",
    "Magnets": "MAGNET",
    "Winding": "COIL",
    "Shaft": "SHAFT",
    "Housing": "HOUSING",
    "EndShield": "ENDSHIELD",
}


# component grouping for per-part export (by build-step id prefix in the registry)
def _component_of(step_id):
    base = step_id.split("#")[0]
    if base.startswith("stator_steel"):
        return "Stator_Lamination"
    if base.startswith("rotor_steel"):
        return "Rotor_Lamination"
    if base.startswith("magnet"):          # magnet solids (pockets are subtracts, not registered)
        return "Magnets"
    if base.startswith("conductor") or base.startswith("endwinding") or base.startswith("hp_"):
        return "Winding"
    if base == "shaft":
        return "Shaft"
    if base.startswith("endshield"):       # end-shield bodies (bolts are subtracts, not registered)
        return "EndShield"
    if base == "housing":
        return "Housing"
    return None


def export_parasolid(part, out_path, bodies=None):
    """Export Parasolid (.x_t), selecting ONLY the solid bodies.

    IMPORTANT: select the bodies explicitly (like export_step) -- do NOT export the
    EntirePart. Each extrude/revolve leaves its section's *dumb construction curves*
    in the part; EntirePart drags those into the Parasolid translator and trips it
    ("Modeler error: please report fault"). The fault scales with curve count, so it
    surfaced once the manufacturing-assembly features added ~40 more sectioned cuts.
    STEP already selects bodies only -- which is why STEP succeeds where the Parasolid
    UF path faults on real NX 2506. STEP (AP242) is therefore the reliable, default
    export; Parasolid is opt-in and may be unavailable on some installs (then skipped)."""
    _save(part)
    if bodies is None:
        bodies = [b for b in part.Bodies if b.IsSolidBody]
    if os.path.exists(out_path):
        try:
            os.remove(out_path)  # translator / UF ExportData errors if the file exists
        except OSError:
            pass
    # NX 2506 uses DexManager.CreateParasolidExporter(); older builds exposed
    # CreateParasolidCreator(). Prefer the DexManager exporter (bodies-only, then
    # EntirePart). The UF Ps.ExportData fallback can throw "Modeler error: please
    # report fault" on a large part AND POISON the modeling session for the next run
    # -- so it is used ONLY when no DexManager Parasolid factory exists at all.
    last = None
    had_factory = False
    for factory_name in ("CreateParasolidExporter", "CreateParasolidCreator"):
        factory = getattr(_SESSION.DexManager, factory_name, None)
        if factory is None:
            continue
        had_factory = True
        for use_bodies in (True, False):   # bodies-only first (no construction curves), then EntirePart
            try:
                pc = factory()
                pc.ObjectTypes.Solids = True
                if use_bodies:
                    pc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.SelectedObjects
                    pc.ExportSelectionBlock.SelectionComp.Add(bodies)
                else:
                    pc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.EntirePart
                pc.InputFile = part.FullPath
                pc.OutputFile = out_path
                pc.Commit()
                pc.Destroy()
                return
            except (AttributeError, NXOpen.NXException) as exc:
                last = exc
    if not had_factory and _UF is not None:
        # older NX without a DexManager Parasolid exporter: UF takes object TAGS
        _UF.Ps.ExportData([b.Tag for b in bodies], out_path)
        return
    raise RuntimeError(
        "Parasolid (.x_t) export unavailable on this NX -- use the STEP (.stp) file "
        "(full assembly, AP242) or export Parasolid from the NX GUI (File > Export > "
        "Parasolid). Last DexManager error: %s" % last)


# --------------------------------------------------------------------------- #
# entry point  (run_journal -args  <blueprint.json>  <out.prt>  [export])
# --------------------------------------------------------------------------- #
def _default_blueprint(end_winding_style=None):
    """Generate the default EV IPMSM blueprint in-process (no NXOpen needed for
    the blueprint layer), so `run_journal nx_builder.py` works with no JSON."""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    # NX keeps ONE Python interpreter per session, so a prior Play-Journal run
    # caches the motor_nx submodules; drop them so edits to params/em_design/
    # blueprint are picked up on the next run without restarting NX.
    for _m in [m for m in list(sys.modules) if m == "motor_nx" or m.startswith("motor_nx.")]:
        del sys.modules[_m]
    from motor_nx import blueprint as _bp
    from motor_nx.params import MotorParams
    p = MotorParams()
    if end_winding_style:
        p.winding.end_winding_style = end_winding_style
    return _bp.generate(p)


def main():
    # Flexible args (order-free): a .json is a blueprint, a .prt is the output,
    # step|parasolid|both|none picks the whole-part export, "parts" ALSO writes each
    # component as its own STEP (piece-by-piece), nxpatterns enables NX pattern
    # features. With NO blueprint given, the default EV IPMSM is built.
    # DEFAULT EXPORT = "step" (AP242): it reliably selects bodies only. Parasolid is
    # OPT-IN ("parasolid"/"both") because the .x_t path can fault on this large part
    # and poison the NX session -- STEP carries the full assembly regardless.
    blueprint = None
    out_prt = None
    export_mode = "step"
    use_nx_patterns = False
    for a in sys.argv[1:]:
        al = a.lower()
        if al in ("step", "parasolid", "both", "none"):
            export_mode = al
        elif al in ("nxpatterns", "patterns"):
            use_nx_patterns = True
        elif al.endswith(".json"):
            with open(a, "r") as fh:
                blueprint = json.load(fh)
        elif al.endswith(".prt"):
            out_prt = a
        else:
            out_prt = a + ".prt"
    hairpin = any("hairpin" in a.lower() for a in sys.argv[1:])
    per_part = any(a.lower() == "parts" for a in sys.argv[1:])  # also export each component as its own STEP
    default_design = blueprint is None
    if blueprint is None:
        blueprint = _default_blueprint("hairpin" if hairpin else None)
    if out_prt is None:
        out_prt = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_out.prt")

    part = new_mm_part(out_prt, make_displayed=True)
    builder = MotorBuilder(part, blueprint.get("stack_length", 134.0), use_nx_patterns=use_nx_patterns)
    builder.log("=== motor_nx build: %s ===" % blueprint.get("name", "motor"))
    if default_design:
        try:
            from motor_nx import em_design as _emd
            from motor_nx.params import MotorParams as _MP
            builder.log(_emd.report(_MP()))
            builder.log("")
            builder.log(_emd.performance_report(_MP()))
            builder.log("")
        except Exception as _rexc:
            builder.log("design report skipped: %s" % _rexc)
        try:
            from motor_nx import fea as _fea
            from motor_nx.params import MotorParams as _MP2
            _feadir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fea")
            _w = _fea.write_package(_MP2(), _feadir)
            builder.log("FEA hand-off package: %d files -> %s" % (len(_w), _feadir))
        except Exception as _fexc:
            builder.log("FEA package skipped: %s" % _fexc)
    for issue in blueprint.get("validation", []):
        builder.log("VALIDATION: %s" % issue)

    try:
        builder.build(blueprint)
    except Exception:
        builder.log("BUILD ABORTED:\n" + traceback.format_exc())

    base = os.path.splitext(out_prt)[0]
    # STEP is the reliable, primary export (AP242, body-selected) -- keep it isolated.
    try:
        if export_mode in ("step", "both"):
            export_step(part, base + "_ap242.stp", "ap242")
            builder.log("exported %s_ap242.stp" % base)
        if export_mode == "none":
            _save(part)
    except Exception:
        builder.log("STEP EXPORT FAILED:\n" + traceback.format_exc())
    # Parasolid is OPT-IN and NON-FATAL: a .x_t fault must not abort the run or hide
    # the (already-saved) .prt/.stp. Warn clearly and recommend a restart if it faults.
    if export_mode in ("parasolid", "both"):
        try:
            export_parasolid(part, base + ".x_t")
            builder.log("exported %s.x_t" % base)
        except Exception as _pexc:
            builder.log("Parasolid (.x_t) export SKIPPED: %s" % _pexc)
            builder.log("  -> use the STEP (.stp) file; if NX threw a modeler fault, RESTART NX "
                        "before the next build (the fault can poison the session).")
    try:
        if per_part:
            builder.log("--- per-part (piece-by-piece) STEP export ---")
            builder.export_parts(base)
    except Exception:
        builder.log("PER-PART EXPORT FAILED:\n" + traceback.format_exc())

    builder.log("=== done (%d errors) ===" % len(builder.errors))


# run_journal executes the module top-to-bottom; guard so an accidental import
# (outside NX) does not auto-run, but a journal run does.
if __name__ == "__main__" or os.environ.get("UGII_ROOT_DIR"):
    main()
