"""suspension_nx -- parametric corner suspension that carries the EV wheel hub
modelled in :mod:`driveline_nx`.

One passenger-EV corner is modelled in a local frame (X = vehicle longitudinal,
Y = lateral with outboard = +Y toward the wheel, Z = vertical/up). The knuckle /
upright carries the Gen-3 wheel-hub bearing; control arms (lower / upper / toe
link) run from inboard chassis pickups to the knuckle; a coil spring + damper and
an anti-roll bar complete the corner. The default layout is a rear multi-link;
"double_wishbone" and "macpherson" are also supported.

Layered exactly like :mod:`driveline_nx` / :mod:`motor_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable)
    engineering  -- wheel rate / ride frequency / roll stiffness / damping + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)
    cli          -- NX-independent command line (report / validate / blueprint / rates)
"""
