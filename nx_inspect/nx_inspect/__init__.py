"""nx_inspect — a professional, open-source NX model-quality inspector.

This package is the pure-Python tooling that drives and renders the output of
the NX-side journal (``nx_inspect/journal/inspect_journal.py``).  The journal
runs *inside* Siemens NX (via ``run_journal``) and reads the true B-rep
geometry, emitting a single JSON report (schema v1).  Everything in *this*
package is NX-free, pure Python, standard-library only, so it installs and runs
anywhere — CI, a laptop without NX, or a developer's machine.

Workflow
--------
1. :mod:`nx_inspect.locate` finds ``run_journal.exe`` generically (env vars,
   Windows registry, common install dirs).
2. :mod:`nx_inspect.cli` invokes the journal on a ``.prt``, which writes a JSON
   report to disk.
3. :mod:`nx_inspect.findings` loads and validates that report against schema v1.
4. :mod:`nx_inspect.report` renders it to a console summary and/or a
   self-contained HTML file.

The CLI can also render an *existing* report with ``--from-json`` (no NX
required), which is what the test-suite and CI use.

The tool is generic: it makes no assumptions about any particular part — it
works on any NX solid-body part or assembly.
"""

__version__ = "1.0.0"
__all__ = ["__version__"]
