"""chassis_nx -- parametric "skateboard" EV chassis / frame that houses the
battery and mounts the front/rear suspension subframes and the e-axle (the
traction motor of :mod:`motor_nx` + the driveline of :mod:`driveline_nx`).

The platform is a flat skateboard structure: two longitudinal frame rails tied
by a set of crossmembers carry a sealed battery tray between them; subframe
mounting bosses pick up the front and rear suspension/e-axle subframes; body
mount holes locate the cabin/body-in-white above; front and rear crush cans form
the crash structure. Modelled in the vehicle frame (X = longitudinal / front +X,
Y = lateral, Z = vertical / up).

Layered exactly like :mod:`motor_nx` / :mod:`driveline_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable)
    engineering  -- section properties / mass / torsional stiffness + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)
    cli          -- NX-independent command line (report / validate / blueprint / mass)
"""
