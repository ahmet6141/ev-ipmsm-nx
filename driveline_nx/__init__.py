"""driveline_nx -- parametric driveline (differential + half-shafts + wheel hubs)
that mates to the EV traction motor modelled in :mod:`motor_nx`.

The motor's keyed output stub feeds a single-speed final drive + differential; the
differential drives two half-shafts (with inboard tripod + outboard Rzeppa CV
joints) out to Gen-3 wheel-hub bearing units carrying the wheel-mounting bolt
circle (the "wheel connection elements").

Layered exactly like :mod:`motor_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable)
    engineering  -- ratios / torque capacity / shaft stress / bearing life + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)
    cli          -- NX-independent command line (report / validate / blueprint / ...)
"""
