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
//! Four things happen here that no substrate does:
//!
//! 1. **Reference resolution.** A characteristic whose `COMPU_METHOD` or
//!    `RECORD_LAYOUT` is undeclared, or whose `COMPU_TAB_REF` has no
//!    registered points, is a typed [`CalError::Unsupported`] — never a
//!    silently defaulted conversion. A wrong conversion writes the wrong
//!    number into an ECU.
//! 2. **Inversion.** `to_physical` is evaluation; `to_raw` is the part that
//!    matters, and it bracket-checks against the deposit's datatype range so
//!    the tool cannot ask for a value the ECU cannot hold.
//! 3. **Transport framing.** [`XcpTransport`] is deliberately two methods
//!    wide, which makes a session testable against a bench mock while
//!    [`CalibrationSession::upload_frames`] and friends hand a real bridge
//!    the exact `xcp_core` frames to send.
//! 4. **Calibration arithmetic.** [`calibrate_curve`] fits a monotone
//!    piecewise-linear curve through measured nodes; [`optimize_working_point`]
//!    finds a bounded working point by deterministic coordinate descent.
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
//! use cal_model::{CalibrationProject, ResourceMode, XcpTransport, SAMPLE_A2L, CalError};
//! # fn main() -> Result<(), CalError> {
//!
//! // 1. Load the description and resolve it.
//! let mut project = CalibrationProject::from_a2l(SAMPLE_A2L)?;
//! project.register_table("boost_curve_tab", vec![(0.0, 0.0), (6000.0, 150.0)])?;
//!
//! // 2. Look a parameter up, convert both ways.
//! let module = project.module("engine")?;
//! assert_eq!(module.characteristics.len(), 9);
//! let idle = project.characteristic("engine", "idle_target_rpm")?;
//! assert_eq!(idle.address, 0x720104);
//! // 3200 counts · 0.25 rpm/count = 800 rpm.
//! assert!((project.to_physical("engine", "idle_target_rpm", 3200)? - 800.0).abs() < 1e-9);
//! assert_eq!(project.to_raw("engine", "idle_target_rpm", 800.0)?, 3200);
//!
//! // 3. Limits are enforced on the way in.
//! assert!(project.check_limits("engine", "idle_target_rpm", 3200).is_ok());
//! assert!(project.check_limits("engine", "idle_target_rpm", 0).is_err());
//!
//! // 4. Calibrate against a bench ECU.
//! struct Bench;
//! impl XcpTransport for Bench {
//!     fn read(&mut self, _addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
//!         Ok(vec![0; usize::from(len)])
//!     }
//!     fn write(&mut self, _addr: u32, _data: &[u8]) -> Result<(), CalError> { Ok(()) }
//! }
//! let mut session = cal_model::CalibrationSession::connect(
//!     &project, Box::new(Bench), ResourceMode::CONNECT_NORMAL,
//! )?;
//! session.write_characteristic("engine", "idle_target_rpm", 900.0)?;
//! assert_eq!(session.read_characteristic("engine", "idle_target_rpm")?, 0.0);
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
mod model;
mod project;
pub mod sample;
mod session;
mod signal;

pub use calibrate::{
    calibrate_curve, optimal_point, optimize_working_point, CalParameter, Curve,
};
pub use conversion::CompuMethod;
pub use dbc_parse::ByteOrder;
pub use error::CalError;
pub use model::{CharKind, Characteristic, Measurement, Module};
pub use project::CalibrationProject;
pub use sample::{
    element_counts, seed_memory, tables, SAMPLE_A2L, SAMPLE_DBC,
};
pub use session::{
    diff_snapshots, render_deltas, CalibrationDelta, CalibrationSession, Snapshot, SnapshotEntry,
    SnapshotValue, XcpTransport,
};
pub use xcp_core::ResourceMode;

/// Re-export of the A2L substrate, so a consumer needs one dependency.
pub use a2l_parse as a2l;
/// Re-export of the DBC substrate, so a consumer needs one dependency.
pub use dbc_parse as dbc;
/// Re-export of the XCP substrate, so a consumer needs one dependency.
pub use xcp_core as xcp;
