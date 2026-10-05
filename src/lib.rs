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
//!    registered points, is a typed [`CalError::UnknownComputationTable`] —
//!    never a silently defaulted conversion. A wrong conversion writes the
//!    wrong number into an ECU.
//! 2. **Inversion.** `to_physical` is evaluation; [`CalibrationProject::to_raw`]
//!    is the part that matters, and it bracket-checks against the deposit's
//!    datatype range so the tool cannot ask for a value the ECU cannot hold.
//! 3. **Transport framing.** [`XcpTransport`] is deliberately two methods
//!    wide, which makes a session testable against a bench mock while
//!    [`CalibrationSession`] hands a real bridge the exact memory access.
//! 4. **Calibration arithmetic.** [`calibrate_curve`] fits a monotone
//!    piecewise-linear curve through measured nodes; [`optimize_working_point`]
//!    finds a bounded working point by deterministic coordinate descent.
//!
//! # What this layer re-reads from the A2L text
//!
//! `a2l-parse` deliberately keeps only the subset the stack consumes: it
//! records a `COMPU_METHOD`'s `COMPU_TAB_REF` *name* and skips the
//! `COMPU_TAB`/`COMPU_VTAB` block itself. A `TABLE` conversion is
//! unimplementable without the tabulated points, so [`tables`] re-scans the
//! description text for exactly those blocks — a small, total, independently
//! tested scanner. Everything else comes from the substrate.
//!
//! # Raw values are bit patterns
//!
//! Every raw value in this crate is the **memory bit pattern**, so a signed
//! `SBYTE` characteristic holding −1 °C reads `0xFF`. Sign extension happens
//! on the way into the conversion, and the reverse on the way out. This is
//! what makes a read-modify-write cycle bit-exact: nothing is re-derived from
//! a float.
//!
//! # Example
//!
//! ```
//! use cal_model::{CalibrationProject, CalError, MockTransport, ResourceMode, XcpTransport, SAMPLE_A2L};
//! # fn main() -> Result<(), CalError> {
//!
//! // 1. Load the description and resolve it.
//! let project = CalibrationProject::from_a2l(SAMPLE_A2L)?;
//!
//! // 2. Look a parameter up and convert both ways.
//! let module = project.module("engine")?;
//! assert_eq!(module.characteristics.len(), 9);
//! let idle = project.characteristic("engine", "idle_target_rpm")?;
//! assert_eq!(idle.address, 0x720104);
//! // 3200 counts * 0.25 rpm/count = 800 rpm.
//! assert!((project.to_physical("engine", "idle_target_rpm", 3200.0)? - 800.0).abs() < 1e-9);
//! assert_eq!(project.to_raw("engine", "idle_target_rpm", 800.0)?, 3200);
//!
//! // 3. Limits are enforced on the way in.
//! assert!(project.check_limits("engine", "idle_target_rpm", 3200).is_ok());
//! assert!(project.check_limits("engine", "idle_target_rpm", 0).is_err());
//!
//! // 4. Read-modify-write against a bench mock.
//! let mut session = cal_model::CalibrationSession::connect(
//!     &project, Box::new(cal_model::seeded_transport(&project)?),
//!     ResourceMode::CONNECT_NORMAL,
//! )?;
//! let before = session.read_characteristic("engine", "idle_target_rpm")?;
//! assert!((before - 800.0).abs() < 1e-9);
//! session.write_characteristic("engine", "idle_target_rpm", 900.0)?;
//! assert!((session.read_characteristic("engine", "idle_target_rpm")? - 900.0).abs() < 1e-9);
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
mod mock;
mod model;
mod project;
mod sample;
mod session;
mod signal;
mod tables;

pub use calibrate::{
    calibrate_curve, optimal_point, optimize_working_point, optimize_working_point_with_budget,
    CalParameter, Curve,
};
pub use conversion::CompuMethod;
pub use error::CalError;
pub use mock::{MemoryPage, MockTransport};
pub use model::{CharKind, Characteristic, Measurement, Module};
pub use project::CalibrationProject;
pub use sample::{seeded_transport, seed_values, SAMPLE_A2L, SAMPLE_DBC};
pub use session::{
    render_deltas, CalibrationDelta, CalibrationSession, Snapshot, SnapshotEntry, XcpTransport,
};
pub use signal::SignalBinding;
pub use tables::{CompuTabKind, CompuTable};

pub use dbc_parse::ByteOrder;
pub use xcp_core::ResourceMode;

/// Re-export of the A2L substrate, so a consumer needs one dependency.
pub use a2l_parse as a2l;
/// Re-export of the DBC substrate, so a consumer needs one dependency.
pub use dbc_parse as dbc;
/// Re-export of the XCP substrate, so a consumer needs one dependency.
pub use xcp_core as xcp;
