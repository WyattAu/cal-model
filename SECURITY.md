# Security Policy — cal-model

## Supported versions

| Version | Supported |
|---------|-----------|
| 0.1.x   | ✅        |

## Reporting a vulnerability

Report privately via [GitHub security advisories] for this repository, or
email **wyatt_au@protonmail.com**. Do **not** open a public issue for
security reports.

You will receive an acknowledgement within **72 hours**. Coordinated
disclosure: we ask for up to 90 days before public disclosure while a
patch ships.

[GitHub security advisories]: https://github.com/WyattAu/cal-model/security/advisories/new

## Threat model

**Two trust boundaries, one safety invariant.**

1. **Untrusted input: the A2L description.** A2L files originate from
   suppliers, version control, ticket systems, and other semi-trusted
   sources. The trust boundary is the contents of every `&str` handed to
   `CalibrationProject::from_a2l`.
2. **Untrusted input: the DBC database**, for the same reasons, handed to
   `attach_dbc_text`.
3. **Semi-trusted: the ECU.** A calibration session drives real memory. The
   invariant this crate defends is *never report a calibration it did not
   perform*: every write is limit-checked, converted, bracket-checked, and
   read back before it is reported as done.

Callers control where text comes from; `cal-model` guarantees the safety of
the parse and the write regardless.

| ID | Threat | Mitigation |
|----|--------|------------|
| T1 | Malformed A2L or DBC text drives the calibration layer out of bounds or into a panic | Totality by construction: bounds are checked at every access; `unwrap`/`expect`/`panic`/`indexing_slicing` denied at the lib target; exhaustive `CalError`; cargo-fuzz target `cal_parse` with a committed corpus, 30 s CI fuzz smoke |
| T2 | An undeclared `COMPU_METHOD`/`RECORD_LAYOUT` silently defaults, and the ECU is written with values the operator never saw | Strict reference resolution: an unresolvable declaration is `CalError::Unsupported`, never a default. A `TABLE` conversion with no registered `COMPU_TAB_REF` points is likewise refused |
| T3 | A physical value the deposit cannot represent is written and latched as a different number | `to_raw` inverts against the datatype's raw bounds and reports `OutOfBounds`; bisection inversion is bracket-checked at both endpoints |
| T4 | An out-of-limit value reaches ECU memory | `write_characteristic` limit-checks and inverts *before* any bus traffic; a rejected write costs zero frames (asserted in `tests/session.rs`) |
| T5 | A write is reported that the ECU did not latch (write-protected page, lost frame, wrong page) | Every session write is read back and compared; a mismatch is `CalError::Transport`. A failed `SET_CAL_PAGE` leaves the session's page unchanged, so a later write cannot silently land on the wrong page |
| T6 | A multi-element deposit is partially written, destroying the other curve points | Writing one element reads the whole deposit, replaces that element, and writes it back; `restore` writes captured raws verbatim, so it is bit-exact and cannot drift through re-inversion |
| T7 | An IEEE-754 deposit is decoded as an integer bit pattern, producing an enormous or nonsensical value | IEEE datatypes are refused as `CalError::Unsupported` rather than misread |
| T8 | A snapshot restores a parameter the project no longer declares, into a memory region whose layout has changed | `restore` re-resolves every entry against the current project and fails on an unknown module or characteristic |
| T9 | A DBC signal layout that cannot be bit-coded is bound anyway, and later encodes out of bounds | `attach_dbc` validates the matched layout (width `1..=64`, start bit ≤ 511) and refuses the whole attachment on a bad one, leaving no database attached |
| T10 | The optimiser returns a point or value from a diverged search, which then gets flashed | The search is deterministic and bounded; a non-finite objective or an exhausted sweep budget is `CalError::Convergence` carrying the iteration count, never a number |
| T11 | A curve fit produces a curve that lies about the ECU's own interpolation | `calibrate_curve` returns exactly a piecewise-linear function, which is what an ECU's `TAB_INTP` computes; non-monotone nodes are refused rather than fitted |
| T12 | Dependency-supply compromise | Three estate dependencies (two L1, one L0) and `thiserror`, all pinned by `Cargo.lock`; `cargo-deny` advisories/bans/licenses gate in CI; `cargo-vet` supply-chain directory committed |
