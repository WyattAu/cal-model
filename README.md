# cal-model

A2L calibration data model and session layer — characteristic binding,
COMPU_METHOD conversions, limit validation, curve calibration.

**Layer:** L2 — domain · **Estate deps:** `a2l-parse` L1, `xcp-core` L1,
`dbc-parse` L0 · **Runtime deps:** `thiserror` only.

## What this layer is

`cal-model` is the application half of the estate's automotive calibration
stack. The three L0/L1 substrates it sits on each do one thing:

| Substrate | Layer | What it gives this layer |
|---|---|---|
| [`a2l-parse`] | L1 | the A2L declaration: addresses, datatypes, limits, COMPU_METHODs |
| [`dbc-parse`] | L0 | the CAN layout: start bit, width, byte order, scaling |
| [`xcp-core`] | L1 | the XCP protocol: resource masks, command framing |

None of them knows the others exist, so a host that wants to *calibrate* has
to do the joining. That is this crate:

```text
  A2L text ──a2l_parse──► PROJECT/MODULE/CHARACTERISTIC/MEASUREMENT
                                 │  resolve every reference
                                 ▼
                         CalibrationProject ──── dbc_parse ───► SignalBinding
                                 │
       COMPU_METHOD ◄─────────────┤ apply / invert     LOWER_LIMIT / UPPER_LIMIT
                                 ▼
                         CalibrationSession ──── XcpTransport ───► ECU
                                 │ read / write / SET_CAL_PAGE
                                 ▼
                         Snapshot ──► diff ──► CalibrationDelta report
```

## What's in it

| Module | Contents |
|---|---|
| `project` | [`CalibrationProject`] — `from_a2l` / `from_a2l_file`, reference resolution (an undeclared `COMPU_METHOD` or `RECORD_LAYOUT` is a typed error, never a guessed default), `to_physical` / `to_raw` / `check_limits`, `attach_dbc`, `register_table`, `declare_elements`, deposit coding (`read_raw` / `write_raw`) |
| `conversion` | [`CompuMethod`] — LINEAR, RAT_FUNC (evaluated as written, inverted by bisection), TABLE (`TAB_INTP` interpolation and `TAB_NOINTP` step lookup) and IDENTITY, with `apply` total and `invert` bracket-checked |
| `model` | [`Characteristic`], [`Measurement`], [`Module`], [`CharKind`] — the resolved, addressable view of an A2L declaration, including signed-bit-pattern handling and datatype-derived bounds |
| `session` | [`XcpTransport`] (two methods, plus an optional `set_page`), [`CalibrationSession`] — read, read-whole-curve, verified write, `SET_CAL_PAGE`, [`Snapshot`] / restore / [`diff_snapshots`] / [`render_deltas`] |
| `signal` | [`SignalBinding`] — a characteristic or measurement bound to a DBC signal, with both byte orders' bit coding |
| `calibrate` | [`calibrate_curve`] — a monotone piecewise-linear fit through measured nodes; [`optimal_point`] / [`optimize_working_point`] — bounded, deterministic coordinate descent |
| `mock` | [`mock::MockTransport`] — an in-memory ECU (pages, recorded traffic, injectable failures), so a session can be exercised with no hardware |
| `error` | [`CalError`] — typed and exhaustive, 13 variants, `source()` through to the wrapped substrate errors |
| `sample` | the embedded two-ECU demo project: 15 characteristics (every kind, every conversion type), 5 measurements, a DBC, seed memory, and a factory calibration |

## Design decisions worth knowing

**Raw values are bit patterns.** A signed `SBYTE` characteristic holding
−1 °C reads `0xFF`. Sign extension happens in
`Characteristic::raw_value`, on the way into the conversion, and in
`raw_bits`, on the way out. This is what makes read-modify-write bit-exact:
nothing is re-derived from a float.

**Inversion is bracket-checked.** `to_raw` is not "undo the arithmetic". A
value is only writable if the raw that produces it exists inside the
deposit's datatype range, so inversion is checked against those bounds and
reports `OutOfBounds` otherwise. That is the check that stops a tool from
writing a value the ECU would latch as something else.

**Writes are verified.** `write_characteristic` limit-checks, inverts,
writes, then reads back and fails if the ECU did not latch it. A tool that
reports a write it cannot confirm is worse than one that reports an error.

**Element counts and tabulations are supplied, not guessed.** `a2l_parse`
does not retain the ASAP2 `NUMBER` / `NO_AXIS_PTS` keywords, and it skips the
`COMPU_TAB` blocks, so neither a curve's element count nor a `TABLE`
conversion's points arrive with the parse. Both are explicit
(`declare_elements`, `register_table`) rather than inferred — a wrong element
count reads the wrong number of bytes out of ECU memory, and a wrong tabulation
converts raw values the engineer never saw. `sample::complete` applies the
demo project's own values to any project you parsed yourself.

**A degenerate limit range imposes no constraint.** Generators emit
`0.0 0.0` for ASCII identifiers; reading that as "the value must be exactly
zero" would be wrong, so `lower >= upper` means "unconstrained" — the same
convention `dbc-parse` applies to an empty `[min|max]`.

**IEEE-754 deposits are refused.** Every raw/physical contract in this crate
is stated over integer bit patterns; a `FLOAT32_IEEE` deposit carries a
value of a different kind, so it is a typed error rather than a
misinterpretation.

## Example

```rust
use cal_model::{CalibrationProject, CalibrationSession, CalError, ResourceMode, SAMPLE_A2L};

# fn main() -> Result<(), CalError> {
let mut project = CalibrationProject::from_a2l(SAMPLE_A2L)?;
project.register_table("boost_curve_tab", vec![(0.0, 0.0), (6000.0, 150.0])?;

// Raw ↔ physical, both directions.
let physical = project.to_physical("engine", "idle_target_rpm", 3200)?;
assert!((physical - 800.0).abs() < 1e-9);
assert_eq!(project.to_raw("engine", "idle_target_rpm", 800.0)?, 3200);

// Limits are enforced before anything reaches the bus.
assert!(project.to_raw("engine", "idle_target_rpm", 4000.0).is_err());

// And a verified write over a bench ECU.
let bench = cal_model::mock::MockTransport::seeded(&cal_model::seed_memory());
let mut session =
    CalibrationSession::connect(&project, Box::new(bench), ResourceMode::CONNECT_NORMAL)?;
session.write_characteristic("engine", "idle_target_rpm", 950.0)?;
# Ok(())
# }
```

The `calibrate` example runs the whole cycle end to end — inventory with
limits, read-modify-write, a refused out-of-limit write, page switching,
the diff report, and a curve fit:

```sh
cargo run --example calibrate                                    # embedded demo
cargo run --example calibrate -- data/powertrain.a2l data/powertrain.dbc
```

## Tests

`cargo test` runs 157 tests across seven suites plus two doctests:

| Suite | Covers |
|---|---|
| `project` | the embedded description, lookups, typed misses, strict reference resolution, byte order, deposit coding, IEEE refusal |
| `conversion` | round-trips for LINEAR and RAT_FUNC (closed-form and bisection), table monotonicity and inversion, identity passthrough, limit checking, non-invertible and unresolvable conversions |
| `session` | read-modify-write against the mock, read-back verification, page isolation, snapshot/restore bit-exactness, diff reports, hand-computed byte-order extraction, transport error propagation |
| `calibrate` | monotone fits recovering synthetic data, non-monotone rejection, bounded 2-D optimisation, convergence reporting |
| `errors` | `Display` for every `CalError` variant, each variant reached from the public API, the `Error` / `source()` contract |
| `properties` | 500-case linear round-trip, 200-case snapshot/restore idempotence, 300-case byte-order extraction against an independently written naive bit-walk, plus curve and error-rendering properties |

`cargo fuzz run cal_parse` covers the calibration layer over arbitrary input
(committed corpus; CI runs a 30-second smoke). Coverage is gated at 90 % by
the shared tier-a CI workflow.

## Layer

Declared in `Cargo.toml` as `[package.metadata.layers] tier = "L2"` and
registered in `WyattAu/engineering-standards` (`scripts/estate-tiers.json`
and the L2 section of `docs/layers.md`). Every estate-internal edge points
down the stack, so the ceiling rule in `docs/layers.md` §Rules rule 1 holds
with the declared layer exactly.

## License

MIT OR Apache-2.0 — see LICENSE-MIT and LICENSE-APACHE.

[`a2l-parse`]: https://docs.rs/a2l-parse
[`dbc-parse`]: https://docs.rs/dbc-parse
[`xcp-core`]: https://docs.rs/xcp-core
[`CalibrationProject`]: https://docs.rs/cal-model/latest/cal_model/struct.CalibrationProject.html
[`CompuMethod`]: https://docs.rs/cal-model/latest/cal_model/enum.CompuMethod.html
[`Characteristic`]: https://docs.rs/cal-model/latest/cal_model/struct.Characteristic.html
[`Measurement`]: https://docs.rs/cal-model/latest/cal_model/struct.Measurement.html
[`Module`]: https://docs.rs/cal-model/latest/cal_model/struct.Module.html
[`CharKind`]: https://docs.rs/cal-model/latest/cal_model/enum.CharKind.html
[`XcpTransport`]: https://docs.rs/cal-model/latest/cal_model/trait.XcpTransport.html
[`CalibrationSession`]: https://docs.rs/cal-model/latest/cal_model/struct.CalibrationSession.html
[`Snapshot`]: https://docs.rs/cal-model/latest/cal_model/struct.Snapshot.html
[`SignalBinding`]: https://docs.rs/cal-model/latest/cal_model/struct.SignalBinding.html
[`calibrate_curve`]: https://docs.rs/cal-model/latest/cal_model/fn.calibrate_curve.html
[`optimal_point`]: https://docs.rs/cal-model/latest/cal_model/fn.optimal_point.html
[`optimize_working_point`]: https://docs.rs/cal-model/latest/cal_model/fn.optimize_working_point.html
[`diff_snapshots`]: https://docs.rs/cal-model/latest/cal_model/fn.diff_snapshots.html
[`render_deltas`]: https://docs.rs/cal-model/latest/cal_model/fn.render_deltas.html
[`mock::MockTransport`]: https://docs.rs/cal-model/latest/cal_model/mock/struct.MockTransport.html
[`CalError`]: https://docs.rs/cal-model/latest/cal_model/enum.CalError.html
