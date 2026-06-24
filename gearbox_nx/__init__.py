"""gearbox_nx -- the reduction-gearbox housing that CONNECTS the traction motor
(:mod:`motor_nx`) to the differential (:mod:`driveline_nx`) so their solids stop
interpenetrating (ICD §7.1).

In the assembled vehicle the motor (OD 225, r~=112) and the differential (carrier
OD 150, ring-gear pitch Ø208 -> r~=104) overlap because the motor sat only ~125 mm
from the diff axis. Real EV e-axles bridge that gap with a parallel-axis 2-stage
reduction in a cast housing: the motor and diff axes stay PARALLEL (both along the
wheel axis) but are separated far enough to clear (centre distance >= motor_OD/2 +
ring_gear_pitch/2 + clearance).

This package models that connector:

    * a cast REDUCTION HOUSING (representative shell) spanning from the differential
      up/over to the motor mounting face, enclosing the gear train;
    * a 2-STAGE parallel reduction as representative gear BLANKS at pitch diameter --
      motor pinion -> idler/layshaft gear+pinion -> output gear coaxial with the diff
      (the gearbox OUTPUT couples to the driveline's existing diff input flange). The
      two centre distances SUM to the motor<->diff offset so the train reaches; total
      ratio ~9-10:1;
    * a MOTOR-MOUNTING FACE: a bolted flange matching the motor's DE housing flange
      (diameter + bolt circle + pilot read from motor_nx) so it bolts straight on;
    * a DIFF-MOUNTING interface to the differential carrier (carrier OD 150).

Layered exactly like :mod:`motor_nx` / :mod:`driveline_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable, NX expressions)
    engineering  -- reduction ratio, centre distances, pitch-line velocity, housing
                    wall / oil-sump sizing, mass + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)

Local frame: +Z = the gear axes (same as motor/driveline), datum = the DIFFERENTIAL
axis at the local origin. The assembler places it with Rx(-90) (local +Z -> vehicle
+Y) at the diff axis station (ICD §3/§7.1).
"""
