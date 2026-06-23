"""vehicle_nx -- top-level VEHICLE ASSEMBLY of all the subsystem parts.

The motor (motor_nx), inverter (inverter_nx), driveline (driveline_nx), suspension
corners (suspension_nx) and chassis (chassis_nx) are each built as their own NX
.prt in their own local frame. This package computes WHERE each one goes in the
vehicle (origin + orientation in vehicle coordinates) and the NX journal
(`nx_assembler.py`) creates the top assembly and adds every part as a positioned
component.

Vehicle coordinate frame (ISO 8855 road-vehicle convention):
    +X = forward (toward the front axle)
    +Y = left
    +Z = up;  ground plane at z = 0, wheel/hub centre at z = tyre_radius
Origin at the chassis centre, mid-wheelbase.

Layers:
    params       -- vehicle layout + per-part placement inputs (JSON-serialisable)
    assembly     -- NX-independent placement math: the assembly PLAN (part + 4x4
                    transform per component) + validate() + report()
    nx_assembler -- the Siemens NX journal: build each subsystem part, then
                    Assemblies.AddComponent each with its computed transform
    cli          -- NX-independent: plan / report / validate
"""
