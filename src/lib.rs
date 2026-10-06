//! A2L calibration data model and session layer — characteristic binding,
//! COMPU_METHOD conversions, limit validation, curve calibration.
//!
//! `cal-model` is the estate's **L2 application layer** for the automotive
//! calibration stack. It sits on three L0/L1 substrates and turns their
//! declarations into an actionable calibration session:
//!
//! | Substrate | Layer | What it contributes |
//! |---|---|---|
//! | [`a2l_parse`] | L1 | the A2L description: addresses, datatypes, limits, COMPU_METHODs |
//! | [`dbc_parse`] | L0 | the CAN layout: start bit, width, byte order, scaling |
//! | [`xcp_core`] | L1 | the XCP protocol: resource masks, command framing |
//!
//! # The layer's job
//!
//! ```text
//!   A2L text ──a2l_parse──► PROJECT/MODULE/CHARACTERISTIC/MEASUREMENT
//!                                  │  resolve every reference
//!                                  ▼
//!                          CalibrationProject ──── dbc_parse ───► SignalBinding
//!                                  │
//!        COMPU_METHOD ◄─────────────┤ apply / invert      LOWER_LIMIT / UPPER_LIMIT
//!                                  ▼
//!                          CalibrationSession ──── XcpTransport ───► ECU
//!                                  │ read / write / SET_CAL_PAGE
//!                                  ▼
//!                          Snapshot ──► diff ──► CalibrationDelta report
//! ```
//!
//! Five things happen here that no substrate does:
//!
//! 1. **Reference resolution.** A characteristic whose `COMPU_METHOD` or
//!    `RECORD_LAYOUT` is undeclared, or whose `COMPU_TAB_REF` has no
//!    registered points, is a typed [`CalError::Unsupported`] — never a
//!    silently defaulted conversion. A wrong conversion writes the wrong
//!    number into an ECU.
//! 2. **Inversion.** [`to_physical`](CalibrationProject::to_physical) is
//!    evaluation; [`to_raw`](CalibrationProject::to_raw) is the part that
//!    matters, and it bracket-checks against the deposit's datatype range so
//!    the tool cannot ask for a value the ECU cannot hold.
//! 3. **Transport framing.** [`XcpTransport`] is deliberately two methods
//!    wide, which makes a session testable against [`mock::MockTransport`]
//!    while [`CalibrationSession::upload_frames`] and friends hand a real
//!    bridge the exact `xcp_core` frames to send.
//! 4. **Verified writes.** A session write reads the value back and fails if
//!    the ECU did not latch it.
//! 5. **Calibration arithmetic.** [`calibrate_curve`] fits a monotone
//!    piecewise-linear curve through measured nodes;
//!    [`optimize_working_point`] finds a bounded working point by
//!    deterministic coordinate descent.
//!
//! # Raw values are bit patterns
//!
//! Every raw value in this crate is the **memory bit pattern**, so a signed
//! `SBYTE` characteristic holding −1 °C reads `0xFF`. Sign extension happens
//! in [`Characteristic::raw_value`], on the way into the conversion, and in
//! [`Characteristic::raw_bits`], on the way out. This is what makes a
//! read-modify-write cycle bit-exact: nothing is re-derived from a float.
//!
//! # Example
//!
//! ```
//! use cal_model::{
//!     CalibrationProject, CalibrationSession, CalError, ResourceMode, XcpTransport, SAMPLE_A2L,
//! };
//! # fn main() -> Result<(), CalError> {
//!
//! // 1. Load the description and resolve every reference it makes.
//! let mut project = CalibrationProject::from_a2l(SAMPLE_A2L)?;
//! project.register_table("boost_curve_tab", vec![(0.0, 0.0), (6000.0, 150.0)])?;
//! assert_eq!(project.module("engine")?.characteristics.len(), 9);
//!
//! // 2. Look a parameter up and convert both ways.
//! let idle = project.characteristic("engine", "idle_target_rpm")?;
//! assert_eq!(idle.address, 0x720108);
//! // 3200 counts · 0.25 rpm/count = 800 rpm.
//! let physical = project.to_physical("engine", "idle_target_rpm", 3200)?;
//! assert!((physical - 800.0).abs() < 1e-9, "{physical}");
//! assert_eq!(project.to_raw("engine", "idle_target_rpm", 800.0)?, 3200);
//!
//! // 3. Limits are enforced on the way in.
//! project.check_limits("engine", "idle_target_rpm", 3200)?;
//! assert!(project.check_limits("engine", "idle_target_rpm", 0).is_err());
//!
//! // 4. Calibrate against a bench ECU: write, read back, diff.
//! let bench = cal_model::mock::MockTransport::seeded(&cal_model::seed_memory());
//! let mut session = CalibrationSession::connect(
//!     &project, Box::new(bench), ResourceMode::CONNECT_NORMAL,
//! )?;
//! // The seed memory holds 800 rpm; the write is limit-checked (500…1200),
//! // inverted through `rpm_lin`, and read back before it is reported.
//! session.write_characteristic("engine", "idle_target_rpm", 900.0)?;
//! let live = session.read_characteristic("engine", "idle_target_rpm")?;
//! assert!((live - 900.0).abs() < 1e-9, "{live}");
//!
//! // 5. Snapshot, adjust, and report the diff.
//! let before = session.snapshot(&["idle_target_rpm", "eng_torque_max"])?;
//! session.write_characteristic("engine", "idle_target_rpm", 950.0)?;
//! let after = session.snapshot(&["idle_target_rpm", "eng_torque_max"])?;
//! let deltas = cal_model::diff_snapshots(Some(&before), &after, &project);
//! assert_eq!(deltas[0].before, 900.0);
//! assert_eq!(deltas[0].after, 950.0);
//! assert!((deltas[0].delta - 50.0).abs() < 1e-9);
//! assert!(deltas[0].within_limits);
//! # Ok(())
//! # }
//! ```

// A2L/ASAM grammar keywords (CHARACTERISTIC, COMPU_METHOD, ECU_ADDRESS …)
// are proper nouns of the formats this crate reads; backticking every
// occurrence would bury the prose. Allowed with this justification per the
// fleet pedantic policy.
#![allow(clippy::doc_markdown)]

mod calibrate;
mod conversion;
mod error;
pub mod mock;
mod model;
mod project;
pub mod sample;
mod session;
mod signal;

pub use calibrate::{calibrate_curve, optimal_point, optimize_working_point, CalParameter, Curve};
pub use conversion::CompuMethod;
pub use dbc_parse::ByteOrder;
pub use error::CalError;
pub use model::{CharKind, Characteristic, Measurement, Module};
pub use project::CalibrationProject;
pub use sample::{
    complete, element_counts, sample_project, seed_memory, tables, SAMPLE_A2L, SAMPLE_DBC,
};
pub use session::{
    diff_snapshots, render_deltas, CalibrationDelta, CalibrationSession, Snapshot, SnapshotEntry,
    SnapshotValue, XcpTransport,
};
pub use signal::SignalBinding;
pub use xcp_core::ResourceMode;

/// Re-export of the A2L substrate, so a consumer needs one dependency.
pub use a2l_parse as a2l;
/// Re-export of the DBC substrate, so a consumer needs one dependency.
pub use dbc_parse as dbc;
/// Re-export of the XCP substrate, so a consumer needs one dependency.
pub use xcp_core as xcp;
