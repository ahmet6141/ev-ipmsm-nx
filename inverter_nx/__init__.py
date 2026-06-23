"""inverter_nx -- parametric traction INVERTER / motor drive ("sürücü") that drives
the EV IPMSM modelled in :mod:`motor_nx`.

A SiC three-phase voltage-source inverter sits on the HV DC bus, switches the six
power devices (SVPWM) to synthesise the motor phase currents, and is field-oriented
controlled (FOC + MTPA + field weakening) over the motor's full speed range. The
package sizes the power stage, DC link, regen path, control + functional safety, the
liquid cold plate and the HV enclosure, then emits an editable NX solid model.

Layered exactly like :mod:`motor_nx` / :mod:`driveline_nx`:
    params       -- parametric inputs (dataclasses, JSON-serialisable)
    engineering  -- device sizing / field-weakening / DC-link ripple / thermal + validate()
    blueprint    -- NX-independent geometry as ordered build steps (+ JSON)
    nx_builder   -- the Siemens NX journal (reuses motor_nx's hardened NXOpen engine)
    cli          -- NX-independent command line (report / validate / blueprint / thermal)
"""
