"""The ``nx-inspect`` command-line interface.

Two modes:

* **Live** — given a ``.prt``, locate ``run_journal.exe``, run the NX journal on
  the part (writing a JSON report), then load and render it.
* **Offline** — ``--from-json EXISTING.json`` renders a previously-produced
  report with no NX involved (for CI, testing, or sharing).

In both modes the loaded report is rendered to a console summary and,
optionally, a self-contained HTML file.

Exit code = the number of ERROR-severity findings (0 == clean, 1..63 == that
many errors), which makes the tool drop-in for CI gates.  Operational failures
(NX not found, part missing, malformed report) exit with a distinct code >=64,
chosen to sit *outside* the clamped finding range so CI can always tell "the
tool broke" apart from "the model has N errors" (see EXIT_* below).
"""

import argparse
import os
import subprocess
import sys
import tempfile

from . import __version__
from .config import ALL_CHECKS, CHECK_DESCRIPTIONS, Config, normalize_checks
from .findings import SchemaError, load_report_file
from .locate import RunJournalNotFound, find_run_journal
from .report import render_console, write_html

# Exit-code contract:
#   0       clean (no error findings)
#   1..63   that many ERROR findings (the count is clamped to 63 so it can never
#           collide with an operational code below; a model with >=63 errors
#           reports 63 and is, in any case, decisively "not clean")
#   >=64    operational failure (the tool itself could not complete) — these sit
#           outside the clamped finding range so CI/agents can branch on them
#           unambiguously.
# The 64..70 values follow the BSD sysexits.h convention.
MAX_ERROR_EXIT = 63    # finding counts are clamped here; >=64 is reserved
EXIT_USAGE = 64        # bad arguments
EXIT_BAD_REPORT = 65   # report JSON failed schema validation
EXIT_NO_FILE = 66      # input part / json missing
EXIT_NO_NX = 69        # run_journal not found
EXIT_JOURNAL = 70      # journal ran but produced no/invalid report


def _exit_for_errors(n_errors: int) -> int:
    """Map an error-finding count to a clamped exit code (0, or 1..MAX_ERROR_EXIT)."""
    if n_errors <= 0:
        return 0
    return min(n_errors, MAX_ERROR_EXIT)


# Path to the bundled NX journal (shipped inside the package).
_JOURNAL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "journal", "inspect_journal.py")


def _build_parser() -> argparse.ArgumentParser:
    catalog = "\n".join("  %-15s %s" % (c, CHECK_DESCRIPTIONS[c]) for c in ALL_CHECKS)
    p = argparse.ArgumentParser(
        prog="nx-inspect",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Inspect a Siemens NX part for model-quality issues "
                    "(interference, slivers, duplicates, naming).",
        epilog="checks:\n%s\n\n"
               "exit codes (CI-friendly):\n"
               "  0        clean (no ERROR findings)\n"
               "  1-63     that many ERROR findings (count clamped at 63)\n"
               "  64       bad arguments\n"
               "  65       report JSON failed schema validation\n"
               "  66       input part / report file missing\n"
               "  69       run_journal.exe not found\n"
               "  70       journal ran but produced no/invalid report\n"
               "Codes >=64 mean the tool could not complete; they never collide\n"
               "with a finding count, so CI can branch on them unambiguously.\n\n"
               "Examples:\n"
               "  nx-inspect part.prt --html report.html\n"
               "  nx-inspect --from-json report.json --no-color\n" % catalog,
    )
    p.add_argument("part", nargs="?", help="path to the .prt to inspect (omit with --from-json)")
    p.add_argument("--from-json", metavar="REPORT.json", dest="from_json",
                   help="render an existing report JSON without running NX (offline/CI)")
    p.add_argument("--out", metavar="REPORT.json",
                   help="where the journal writes its JSON report "
                        "(default: a temp file, deleted afterwards)")
    p.add_argument("--html", metavar="REPORT.html",
                   help="also write a self-contained HTML report to this path")
    p.add_argument("--grid", type=int, help="interference sample grid per axis (default %d)"
                   % Config().grid)
    p.add_argument("--tol", type=float, dest="tol_mm3",
                   help="min interference overlap volume in mm^3 (default %g)" % Config().tol_mm3)
    p.add_argument("--tiny", type=float, dest="tiny_mm3",
                   help="sliver/tiny-body volume threshold in mm^3 (default %g)" % Config().tiny_mm3)
    p.add_argument("--dup", type=float, dest="dup_mm",
                   help="duplicate-body centroid tolerance in mm (default %g)" % Config().dup_mm)
    p.add_argument("--checks", help="comma-separated subset of checks to run (default: all)")
    p.add_argument("--run-journal", dest="run_journal", metavar="PATH",
                   help="explicit path to run_journal.exe (overrides auto-locate)")
    p.add_argument("--no-color", action="store_true", help="disable ANSI color in console output")
    p.add_argument("--quiet", action="store_true", help="suppress the console report (still writes files)")
    p.add_argument("--version", action="version", version="nx-inspect %s" % __version__)
    return p


def _make_streams_tolerant() -> None:
    """Make stdout/stderr survive non-ASCII glyphs on a legacy code-page console.

    The NX journal puts U+2229 (INTERSECTION) in every interference title and
    U+2014 (EM DASH) in several detail strings; the console renderer emits an em
    dash too.  On an interactive Windows console the stream encoding is the OEM/
    ANSI code page (e.g. cp1252/cp1254/cp437), none of which can encode U+2229,
    so a bare ``print`` would raise ``UnicodeEncodeError`` and crash the tool's
    flagship path.  ``TextIOWrapper.reconfigure`` (3.7+) lets us switch to a
    lossless error handler without replacing the stream object (so capture and
    redirection still work).  Streams that lack ``reconfigure`` (e.g. a plain
    StringIO under test) are left untouched; ``_emit`` has a second-line defence.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors="backslashreplace")
        except (ValueError, OSError):  # pragma: no cover - exotic stream
            pass


def _emit(msg: str, stream=None) -> None:
    stream = stream or sys.stdout
    try:
        print(msg, file=stream)
    except UnicodeEncodeError:
        # Belt-and-suspenders: the stream could not be reconfigured (e.g. its
        # encoding cannot represent a glyph and the error handler is strict).
        # Re-encode lossily through the stream's own codec so we still print
        # *something* instead of dying with a traceback.
        enc = getattr(stream, "encoding", None) or "ascii"
        safe = msg.encode(enc, "backslashreplace").decode(enc, "replace")
        print(safe, file=stream)


def _should_color(no_color: bool) -> bool:
    if no_color or os.environ.get("NO_COLOR"):
        return False
    return sys.stdout.isatty()


def _render(report, args) -> None:
    # Write the HTML artifact FIRST: a console-encode failure must not be able to
    # destroy the file the user asked for.  (With _make_streams_tolerant() the
    # console no longer crashes, but the ordering keeps the artifact safe even if
    # _emit's fallback is ever reached.)
    html_written = None
    if args.html:
        write_html(report, args.html)
        html_written = os.path.abspath(args.html)
    if not args.quiet:
        _emit(render_console(report, color=_should_color(args.no_color)))
    if html_written:
        _emit("\nHTML report written to %s" % html_written)


def _run_journal(part, args, cfg: Config) -> str:
    """Invoke run_journal on ``part``, returning the path to the JSON report it wrote."""
    exe = find_run_journal(args.run_journal)  # may raise RunJournalNotFound

    tmp_out = None
    if args.out:
        out_path = os.path.abspath(args.out)
    else:
        fd, out_path = tempfile.mkstemp(prefix="nx_inspect_", suffix=".json")
        os.close(fd)
        tmp_out = out_path

    cmd = [exe, _JOURNAL, "-args", os.path.abspath(part), "out=" + out_path] + cfg.journal_args()
    _emit("Running NX journal:\n  %s" % " ".join(cmd))
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
    except OSError as exc:
        raise RunJournalNotFound("failed to launch run_journal (%s): %s" % (exe, exc))

    if proc.stdout:
        sys.stdout.write(proc.stdout)
    if proc.stderr:
        sys.stderr.write(proc.stderr)

    if not os.path.isfile(out_path) or os.path.getsize(out_path) == 0:
        if tmp_out and os.path.isfile(tmp_out):
            os.unlink(tmp_out)
        raise _JournalError(
            "the NX journal did not produce a report at %s (exit %s). "
            "Is this a valid .prt and is NX licensed?" % (out_path, proc.returncode))
    return out_path


class _JournalError(RuntimeError):
    """The journal launched but produced no usable report."""


def main(argv=None) -> int:
    # Before any output: make the console tolerant of the non-ASCII glyphs the
    # journal emits (U+2229, U+2014), so the flagship interference path cannot
    # crash with UnicodeEncodeError on a legacy Windows code page.
    _make_streams_tolerant()

    parser = _build_parser()
    args = parser.parse_args(argv)

    # ---- validate argument combination -----------------------------------#
    if not args.from_json and not args.part:
        parser.error("a PART path or --from-json is required")
    if args.from_json and args.part:
        parser.error("give either a PART or --from-json, not both")

    # Validate --checks early so a typo fails before we touch NX.
    cfg = Config().with_overrides(
        grid=args.grid, tol_mm3=args.tol_mm3, tiny_mm3=args.tiny_mm3, dup_mm=args.dup_mm,
    )
    if args.checks:
        try:
            cfg = cfg.with_overrides(checks=normalize_checks(args.checks))
        except ValueError as exc:
            parser.error(str(exc))

    # ---- offline mode -----------------------------------------------------#
    if args.from_json:
        if not os.path.isfile(args.from_json):
            _emit("error: report file not found: %s" % args.from_json, sys.stderr)
            return EXIT_NO_FILE
        try:
            report = load_report_file(args.from_json)
        except SchemaError as exc:
            _emit("error: %s" % exc, sys.stderr)
            return EXIT_BAD_REPORT
        _render(report, args)
        return _exit_for_errors(report.n_errors)

    # ---- live mode --------------------------------------------------------#
    if not os.path.isfile(args.part):
        _emit("error: part file not found: %s" % args.part, sys.stderr)
        return EXIT_NO_FILE
    if not args.part.lower().endswith(".prt"):
        _emit("warning: %s does not look like a .prt file" % args.part, sys.stderr)

    tmp_to_clean = None
    try:
        out_path = _run_journal(args.part, args, cfg)
        if not args.out:
            tmp_to_clean = out_path
    except RunJournalNotFound as exc:
        _emit("error: %s" % exc, sys.stderr)
        return EXIT_NO_NX
    except _JournalError as exc:
        _emit("error: %s" % exc, sys.stderr)
        return EXIT_JOURNAL

    try:
        report = load_report_file(out_path)
    except SchemaError as exc:
        _emit("error: the journal's report failed validation: %s" % exc, sys.stderr)
        return EXIT_BAD_REPORT
    finally:
        if tmp_to_clean and os.path.isfile(tmp_to_clean):
            try:
                os.unlink(tmp_to_clean)
            except OSError:
                pass

    _render(report, args)
    return _exit_for_errors(report.n_errors)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
