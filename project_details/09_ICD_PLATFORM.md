# Interface Control Document — EV Platform (vehicle frame, datums, mating points)

This ICD pins the **single coordinate convention** and the **inter-subsystem mating
interfaces** so that every package (`chassis_nx`, `suspension_nx`, `driveline_nx`,
`inverter_nx`, `motor_nx`) builds geometry that assembles consistently in
`vehicle_nx`. It is the authority when a subsystem datum and the assembly placement
disagree.

## 1. Master vehicle frame — ISO 8855

- **+X** = forward, **+Y** = left, **+Z** = up.
- **Origin** = vehicle centre: mid-wheelbase, on the **ground plane (z = 0)**.
- Master dimensions live in `vehicle_nx.params.LayoutParams` (the single source of truth):
  - wheelbase `L` = 2875 mm → **front axle x = +L/2 = +1437.5**, **rear axle x = −L/2 = −1437.5**
  - track `T` (front/rear) = 1580 mm → **hub-centre y = ±T/2 = ±790**
  - loaded tyre radius `r` = 335 mm → **hub-centre height z = r = 335** (axle centre-line)

## 2. THE shared datum — the wheel-hub centre

Every corner has ONE mating point all subsystems agree on:

```
    HUB_CENTRE(axle, side) = ( ±L/2 ,  ±T/2 ,  r )   in vehicle coords
```

The **driveline wheel-hub flange face**, the **suspension upright hub bore**, and the
**wheel** all coincide at this point. Build each subsystem so its hub feature lands
there after the assembly transform below.

## 3. Per-subsystem local frame, datum, and assembly placement

| Subsystem    | Local frame (build coords)                                   | Local datum (origin)                         | Assembly transform (local → vehicle)                                  |
|--------------|--------------------------------------------------------------|----------------------------------------------|-----------------------------------------------------------------------|
| **chassis**  | = vehicle frame (X fwd, Y left, Z up)                        | vehicle centre on ground (z=0)               | **identity**, origin (0,0,0)                                          |
| **driveline**| local **+Z = wheel rotation axis**; +Z = left, −Z = right    | differential centre                          | **Rx(−90)** (+Z→+Y), origin `[axle_x, 0, r]`                          |
| **motor**    | local **+Z = rotation axis** (unchanged)                     | stack centre                                 | **Rx(−90)**, origin `[axle_x − off_x, 0, r + off_z]`                  |
| **suspension**| local **+X fwd, +Y outboard, +Z up**                        | **wheel-hub centre** (upright bore on origin)| left: **identity**; right: **Rz(180)**; origin `HUB_CENTRE(axle,side)`|
| **inverter** | local box axes                                               | mounting face centre                         | identity, mounted above the motor                                     |

### Consequences the redesign MUST honour
- **chassis** builds in TRUE vehicle coordinates:
  - two longitudinal rails along **+X**, centre-line `y = ±(frame_inner_width/2 + rail_width/2)`, spanning the overall length centred on x=0;
  - crossmembers along **+Y bridging the rails** (they must physically connect rail-to-rail), distributed along the wheelbase in X;
  - battery tray a sealed box **between the rails**, low (floor), centred on x=0;
  - subframe mount pads at the **front/rear axle x-stations** on the rail; body mounts on the rail tops; crush cans extend along **+X** beyond the wheelbase.
  - Use the new `prism` primitive (extrude a section along an arbitrary axis) for every beam — rails `axis=+X`, crossmembers `axis=+Y`.
- **suspension** local origin is the **hub centre** (not the vehicle centreline):
  - upright/knuckle hub bore centred on the local origin;
  - inboard control-arm pickups at local **−Y** (toward chassis centreline) at `y ≈ −arm_length`;
  - links modelled along their TRUE 3D axes with `prism`/axis-placed cylinders (A-arms, toe link), not flat +Z plates;
  - spring & damper at their real inclination (axis-placed cylinders/tube), seats on the lower arm and the body.
  - This re-datuming is REQUIRED so the assembly placing the origin at `HUB_CENTRE` puts the hub at the wheel (no double-count of T/2).
- **driveline** total built track length must equal the vehicle track:
  - `2 × (diff_half + cv_inboard + halfshaft + cv_outboard + hub_bearing + hub_flange) ≈ T`
  - so each wheel-hub flange face lands at local `|z| = T/2 = 790`, i.e. vehicle `y = ±790`.
  - keep the existing high-fidelity detail (CV joints, keyways, bolt circles, ABS ring, lug PCD); add the consistency tie to `T`.

## 4. Dimensional-consistency rules (assembly validation must check these)

1. `driveline` built track length ≈ `LayoutParams.track_*` (±2 %).
2. `suspension` hub-centre local Y ≈ `T/2`; corner envelope clears the rail.
3. `chassis` overall length ≥ wheelbase + both overhangs; `frame_inner_width` ≥ battery tray width + clearance; rail spacing brackets `±T/2` mount pads.
4. No two non-chassis components share an origin (existing check — keep).
5. Motor + inverter clear the ground (`r + off_z − envelope/2 > 0`).

## 5. Build vocabulary (shared, hardened engine — `motor_nx.nx_builder`)

`tube | cylinder | extrude | revolve | hole | prism`. **Do NOT edit `motor_nx`** in a
subsystem redesign — it is the one verified NXOpen module. If the engine lacks a
primitive you need, STOP and report it; do not add NXOpen calls in a subsystem.

- `prism`: profile is local `(u, v)`; `origin3` = world placement; `axis` = extrude
  direction; `u_dir` = local +u in world (v = axis × u). Use for all oriented beams/plates.
- `cylinder`/`tube`: set `origin3`/`axis` for a body coaxial with an arbitrary axis;
  omit both for the legacy +Z column at `(cx, cy, z0)`.
- Pure-math twins for NX-free tests/bounding boxes: `blueprint.prism_frame`,
  `blueprint.profile_to_world`.

## 6. Quality bar

Match `motor_nx` package quality: NX-independent + unit-tested geometry, an
`engineering.derive()/validate()` pass with real formulae and standards, NX-safe
named expressions, material-role body names for FEA/CAM, and per-part STEP export
grouping. Every redesigned package keeps `python -m pytest` green and adds tests for
the new geometry (world bounding boxes, connectivity, mating-point coordinates).
