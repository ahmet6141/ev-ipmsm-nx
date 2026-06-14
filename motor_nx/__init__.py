"""motor_nx -- parametric IPMSM (interior permanent-magnet synchronous motor)
generator for EV traction, built for Siemens NX automation.

Architecture (mirrors the proven `tarik_xluuv_generator` split):

    params.py     -> parametric inputs (sweep-friendly dataclasses, JSON I/O)
    em_design.py  -> derived geometry + winding factors + design validation
    blueprint.py  -> PURE-MATH geometry -> ordered list of CAD "build steps" + JSON
    nx_builder.py -> NXOpen Python builder that consumes a blueprint inside NX
    cli.py        -> emit a blueprint JSON without NX (for inspection / batch input)

`params`, `em_design` and `blueprint` have NO dependency on NX -- they run with a
plain CPython interpreter and are fully unit-testable. Only `nx_builder` imports
`NXOpen`, so it is executed via run_journal.exe inside a Siemens NX session.
"""

from . import params, em_design, blueprint  # noqa: F401

__all__ = ["params", "em_design", "blueprint"]
__version__ = "0.1.0"
