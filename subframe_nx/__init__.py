"""subframe_nx -- the parametric suspension / e-axle SUBFRAME (cradle) that closes
the chassis <-> suspension joint (ICD §7.2).

The suspension corner is datumed on the hub centre; in the vehicle its inboard
control-arm pickups land ~160-240 mm inboard of the chassis rails with nothing to
attach to, and the spring/damper top reaches z≈690 with no body mount. This cradle
bolts UP to the chassis subframe mount pads (y=±585, z≈400 at the axle x-station) and
reaches inboard/down to present:
    * a perimeter CRADLE (hollow box beams) -- the structural loop,
    * SUSPENSION PICKUP BOSSES at every inboard hardpoint (no floating arm),
    * a SHOCK / body TOWER reaching the damper/strut top (no floating spring),
    * E-AXLE / DIFF MOUNTS to carry the gearbox + differential.

Front and rear variants (a param). Built in the VEHICLE FRAME (like the chassis),
datumed at the axle x-station on the ground, so the assembler places it at IDENTITY
per axle (origin = the axle x-station).

Layered exactly like :mod:`chassis_nx` / :mod:`driveline_nx` / :mod:`motor_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable) + hardpoint table
    engineering  -- cradle load paths / stiffness / mass / bolt sizing + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)
    cli          -- NX-independent command line (report / validate / blueprint / mass)
"""
