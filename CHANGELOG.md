# Changelog

All notable changes to this project will be documented in this file.
Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning:
[SemVer](https://semver.org/).

## [0.1.0] — 2026-10-06

Initial release — L2 application layer over `a2l-parse` (L1), `xcp-core`
(L1), and `dbc-parse` (L0).

### Added

- **Project**: `CalibrationProject::from_a2l` / `from_a2l_file` /
  `from_parsed`, with strict reference resolution — an undeclared
  `COMPU_METHOD` or `RECORD_LAYOUT` is `CalError::Unsupported`, never a
  guessed default. `to_physical` / `to_raw` / `check_limits`,
  `attach_dbc` / `attach_dbc_text` with layout validation,
  `signal_for_characteristic` / `signal_for_measurement` and their
  `require_` forms, `register_table`, `declare_elements`,
  `set_deposit_byte_order`, `locate`, and deposit coding (`read_raw` /
  `write_raw` / `physical_values`).
- **Conversions**: `CompuMethod` over all four A2L conversion types —
  LINEAR and IDENTITY closed-form, RAT_FUNC evaluated as written and
  inverted by bisection (ascending or descending), TABLE with `TAB_INTP`
  interpolation and `TAB_NOINTP` step lookup, each invertible and
  bracket-checked against the deposit's raw bounds.
- **Model**: `Characteristic` (kind, address, deposit + `FNC_VALUES`
  position, datatype, element count, conversion, limits, `MAX_DIFF`),
  `Measurement`, `Module`, `CharKind`, with signed bit-pattern handling and
  datatype-derived value bounds.
- **Session**: the `XcpTransport` trait (read, write, optional `set_page`,
  optional `as_any`), `CalibrationSession` with verified read-modify-write,
  whole-curve reads, single-element writes that preserve the rest of the
  deposit, `SET_CAL_PAGE`, `SET_MTA`+`UPLOAD`/`DOWNLOAD` frame builders,
  and typed errors when a slave advertises no CAL/PAG resource.
- **Snapshots**: `Snapshot` / `SnapshotEntry` / `SnapshotValue`
  (`PartialEq`, `Display`), `snapshot` / `snapshot_module` /
  `capture_baseline`, bit-exact `restore`, `diff` / `diff_snapshots`, and
  `render_deltas` for the report.
- **Bindings**: `SignalBinding` for both DBC byte orders, with extract,
  decode, encode, and payload sizing.
- **Calibration arithmetic**: `calibrate_curve` (monotone piecewise-linear
  fit, both directions, `NonMonotone` on a turn) and
  `optimal_point` / `optimize_working_point` (bounded deterministic
  coordinate descent with multi-start, `Convergence` carrying the sweep
  count).
- **Mock**: `mock::MockTransport` — sparse per-page ECU memory, recorded
  traffic, injectable read/write failures, and `from_transport` for
  recovering it from a `&dyn XcpTransport`.
- **Fixtures**: an embedded two-ECU demo project (15 characteristics
  covering every kind and conversion type, 5 measurements, vendor `IF_DATA`
  / `AXIS_DESCR` / `COMPU_TAB` blocks), its DBC, seed memory, and a factory
  calibration.
- **Errors**: `CalError`, typed and exhaustive, 13 variants, `source()`
  through to the wrapped substrate errors.
- **Gates**: 157 tests across seven suites, a 5-property proptest battery
  (500 / 200 / 300 / 100 / 100 cases), a cargo-fuzz target `cal_parse` with
  a committed corpus, tier-a shared CI pinned to engineering-standards
  `main`, and 90 %+ line coverage.

[0.1.0]: https://github.com/WyattAu/cal-model/releases/tag/v0.1.0
