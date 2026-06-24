# Examples

A real, end-to-end sample so you can see exactly what `nx_inspect` produces
without needing NX installed.

## Files

- **`suspension_report.json`** — an actual report emitted by the NX-side journal
  (`inspect_journal.py`) running on a 56-body rear-suspension assembly export
  (`suspension_out.prt`). Schema v1. It contains full per-body mass properties
  and a single `info` finding (3 unnamed bodies), so it is a *clean* model
  (0 errors).
- **`suspension_report.html`** — the same report rendered by this package's
  `report.py`. Open it in any browser; it is fully self-contained (inline CSS
  and JS, no external assets). Click a table header to sort.

## Reproduce the HTML from the JSON

Offline, no NX required:

```bash
nx-inspect --from-json examples/suspension_report.json --html examples/suspension_report.html --no-color
```

This is exactly how the committed `suspension_report.html` was generated, and it
is the command the acceptance check / CI uses. Because the sample has zero error
findings, the CLI prints `CLEAN` and exits `0`.

## Use it as a test fixture

The test-suite loads `suspension_report.json` directly (see
`tests/conftest.py`), so it doubles as the canonical fixture for the loader and
renderers.
