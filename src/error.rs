//! Typed, exhaustive error taxonomy for the calibration application layer.
//!
//! The enum is deliberately **exhaustive** (no `#[non_exhaustive]`):
//! callers match every variant, and a new variant is a semver-minor event
//! by design — the same policy as `a2l_parse::A2lError`, `xcp_core::XcpError`,
//! and `dbc_parse::DbcError`, all of which are re-exported here as the
//! `A2l`, `Xcp`, and `Dbc` variants.
//!
//! # What "no panics" means here
//!
//! Every failure mode of a calibration session resolves to one of these
//! variants: a malformed A2L file ([`CalError::A2l`]), a slave ERR response
//! ([`CalError::Xcp`]), a DBC layout that cannot be coded
//! ([`CalError::Dbc`]), a name that does not resolve, a physical value
//! outside the A2L limits, a value the deposit cannot represent, an
//! unusable conversion, a non-converging optimiser, or a transport that
//! reported a failure. There is no variant-free panic path, and no
//! `unwrap` or index expression in the library target.

/// The single crate-level error for every calibration operation.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CalError {
    /// The A2L project description could not be parsed, or a declaration it
    /// carries does not resolve (an undeclared `COMPU_METHOD` or
    /// `RECORD_LAYOUT` is reported as [`CalError::Unsupported`]).
    #[error("A2L description error: {0}")]
    A2l(#[from] a2l_parse::A2lError),

    /// An XCP protocol failure: a slave ERR response, or a transport that
    /// advertised no CAL/PAG resource.
    #[error("XCP error: {0}")]
    Xcp(#[from] xcp_core::XcpError),

    /// A DBC failure: a malformed database, or a signal layout that cannot
    /// be bit-coded into a payload buffer.
    #[error("DBC error: {0}")]
    Dbc(#[from] dbc_parse::DbcError),

    /// No `/begin MODULE` with this name exists in the project.
    #[error("unknown module `{0}`")]
    UnknownModule(String),

    /// The module exists but has no CHARACTERISTIC (or MEASUREMENT) with
    /// this name.
    #[error("unknown characteristic `{0}`")]
    UnknownCharacteristic(String),

    /// A physical value lies outside the A2L `LOWER_LIMIT`/`UPPER_LIMIT` of
    /// the characteristic. `value` is the offending **physical** value.
    #[error("`{name}` = {value} violates its calibration limits [{lower}, {upper}]")]
    LimitViolation {
        /// The CHARACTERISTIC name.
        name: String,
        /// The offending physical value.
        value: f64,
        /// `LOWER_LIMIT` from the A2L description.
        lower: f64,
        /// `UPPER_LIMIT` from the A2L description.
        upper: f64,
    },

    /// A value is inside the A2L limits but outside what the ECU deposit
    /// can represent — the raw value the conversion would need lies beyond
    /// the datatype's range, or the conversion is not invertible there.
    /// `value` is the offending physical value; `lower`/`upper` are the
    /// bounds of the deposit, in physical units.
    #[error("`{name}` = {value} is outside the representable range [{lower}, {upper}]")]
    OutOfBounds {
        /// The CHARACTERISTIC name.
        name: String,
        /// The offending physical value.
        value: f64,
        /// Smallest value the destination can hold, in physical units.
        lower: f64,
        /// Largest value the destination can hold, in physical units.
        upper: f64,
    },

    /// No CAN signal in the attached DBC binds to this characteristic. The
    /// payload is `module.characteristic`.
    #[error("no CAN signal binding for `{0}`")]
    NoSignalBinding(String),

    /// The [`XcpTransport`](crate::XcpTransport) reported a failure. The
    /// payload is the transport's own message.
    #[error("transport failure: {0}")]
    Transport(String),

    /// The optimiser did not converge. Carries the number of iterations it
    /// spent before giving up.
    #[error("optimiser did not converge after {iterations} iterations")]
    Convergence {
        /// Sweeps performed before giving up.
        iterations: u32,
    },

    /// A declaration cannot be honoured: an undeclared `COMPU_METHOD` /
    /// `RECORD_LAYOUT`, a TABLE conversion whose `COMPU_TAB_REF` has no
    /// registered points, a non-invertible conversion, a signal layout
    /// outside the DBC range, or a deposit too large for one XCP UPLOAD.
    #[error("{subject}: {reason}")]
    Unsupported {
        /// What the failure is about (a characteristic, a signal, a
        /// conversion, …).
        subject: String,
        /// Why it cannot be honoured.
        reason: String,
    },

    /// A calibration curve's ordinates change direction: the fit requires a
    /// monotone piecewise-linear function.
    #[error("non-monotone curve at `{name}`: {value} follows {previous}")]
    NonMonotone {
        /// The node name whose ordinate breaks monotonicity.
        name: String,
        /// The offending ordinate.
        value: f64,
        /// The ordinate it must not cross (the preceding node's value).
        previous: f64,
    },

    /// Reading an A2L file from disk failed. `path` is the file, `detail`
    /// the platform's message.
    #[error("cannot read `{path}`: {detail}")]
    Io {
        /// The path that could not be read.
        path: String,
        /// The platform error message.
        detail: String,
    },
}
