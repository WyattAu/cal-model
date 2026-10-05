//! Typed, exhaustive error taxonomy for the calibration application layer.
//!
//! [`CalError`] is the single error type of the crate. It is deliberately
//! **exhaustive** (no `#[non_exhaustive]`): callers match every variant, and
//! a new variant is a semver-minor event by design — the same policy as
//! `a2l_parse::A2lError`, `dbc_parse::DbcError`, and `xcp_core::XcpError`.
//!
//! # Failure classes
//!
//! 1. **Description errors** — the A2L or DBC input is malformed
//!   ([`CalError::A2l`], [`CalError::Dbc`]), the file could not be read
//!   ([`CalError::Io`]), or a referenced module, characteristic, measurement,
//!   COMPU_METHOD, or COMPU_TAB is absent ([`CalError::UnknownModule`] and
//!   its siblings).
//! 2. **Domain errors** — a value is outside its declared limits
//!   ([`CalError::LimitViolation`]), outside what its COMPU_METHOD and
//!   deposit datatype can represent ([`CalError::OutOfBounds`]), not finite
//!   ([`CalError::NonFiniteValue`]), or the declared conversion cannot be
//!   evaluated at all ([`CalError::UnsupportedConversion`]).
//! 3. **Session errors** — the transport failed ([`CalError::Transport`]),
//!   the slave answered a protocol-level error ([`CalError::Xcp`]), or a
//!   CHARACTERISTIC has no unambiguous CAN signal binding
//!   ([`CalError::NoSignalBinding`], [`CalError::AmbiguousSignal`]).
//! 4. **Solver errors** — the calibration problem is infeasible
//!   ([`CalError::InfeasibleCurve`], [`CalError::NoParameters`]) or the
//!   optimizer hit its iteration budget without converging
//!   ([`CalError::Convergence`]).
//!
//! Every variant carries the identifying detail an operator needs — the
//! characteristic name, the offending value, the limits it violated — so a
//! failure can be reported without a second lookup.

use a2l_parse::A2lError;
use dbc_parse::DbcError;
use xcp_core::XcpError;

/// The single crate-level error for project loading, conversion, limit
/// validation, session access, and calibration solving.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CalError {
    /// The A2L description could not be parsed.
    #[error("A2L description error: {0}")]
    A2l(#[from] A2lError),

    /// The DBC database could not be parsed.
    #[error("DBC database error: {0}")]
    Dbc(#[from] DbcError),

    /// The XCP slave or its framing layer reported a protocol error.
    #[error("XCP protocol error: {0}")]
    Xcp(#[from] XcpError),

    /// The description file could not be read from the filesystem.
    #[error("cannot read {path}: {message}")]
    Io {
        /// Path that could not be read.
        path: String,
        /// The underlying I/O error message.
        message: String,
    },

    /// No MODULE with this name exists in the project.
    #[error("unknown module `{0}`")]
    UnknownModule(String),

    /// No CHARACTERISTIC with this name exists in the module.
    #[error("unknown characteristic `{0}`")]
    UnknownCharacteristic(String),

    /// No MEASUREMENT with this name exists in the module.
    #[error("unknown measurement `{0}`")]
    UnknownMeasurement(String),

    /// A CHARACTERISTIC or MEASUREMENT references a COMPU_METHOD the
    /// module does not declare.
    #[error("unknown COMPU_METHOD `{0}`")]
    UnknownCompuMethod(String),

    /// A `TABLE` COMPU_METHOD references a COMPU_TAB/COMPU_VTAB that is
    /// absent or carries no usable numeric points.
    #[error("unknown or unusable COMPU_TAB `{0}`")]
    UnknownComputationTable(String),

    /// The value violates the declared CHARACTERISTIC limits.
    #[error("characteristic `{name}` value {value} outside declared limits [{lower}, {upper}]")]
    LimitViolation {
        /// Characteristic name.
        name: String,
        /// The offending physical value.
        value: f64,
        /// Declared lower limit.
        lower: f64,
        /// Declared upper limit.
        upper: f64,
    },

    /// The value cannot be represented by the raw deposit: it lies outside
    /// the physical span the COMPU_METHOD and deposit datatype can hold.
    #[error("`{name}` value {value} cannot be represented as a raw deposit in [{lower}, {upper}]")]
    OutOfBounds {
        /// Characteristic or measurement name.
        name: String,
        /// The offending physical value.
        value: f64,
        /// Lowest representable physical value.
        lower: f64,
        /// Highest representable physical value.
        upper: f64,
    },

    /// The value is not finite (NaN or infinity) where a finite value is
    /// required.
    #[error("`{name}` value {value} is not finite")]
    NonFiniteValue {
        /// Characteristic, measurement, parameter, or `"objective"` name.
        name: String,
        /// The offending value.
        value: f64,
    },

    /// The COMPU_METHOD cannot be evaluated in the requested direction: a
    /// `TABLE` method with no usable points, or a `RAT_FUNC` whose
    /// denominator vanishes or has no non-negative real inverse.
    #[error("COMPU_METHOD `{name}` cannot be evaluated: {detail}")]
    UnsupportedConversion {
        /// COMPU_METHOD or characteristic name.
        name: String,
        /// Why the conversion is not evaluable.
        detail: &'static str,
    },

    /// No CAN signal is bound to the CHARACTERISTIC.
    #[error("no CAN signal binding for characteristic `{0}`")]
    NoSignalBinding(String),

    /// A CHARACTERISTIC name matches signals in more than one DBC message,
    /// so the binding would be ambiguous.
    #[error(
        "signal `{name}` is declared in more than one message (CAN id 0x{can_id:X} and 0x{other_can_id:X})"
    )]
    AmbiguousSignal {
        /// Signal (and characteristic) name.
        name: String,
        /// CAN id of the message being bound.
        can_id: u32,
        /// CAN id of the message already bound.
        other_can_id: u32,
    },

    /// The transport bridge failed, or does not support the requested
    /// operation (e.g. page switching without the CAL/PAG resource).
    #[error("transport error: {0}")]
    Transport(String),

    /// The optimizer exhausted its iteration budget without converging.
    #[error("optimizer did not converge within {iterations} iterations")]
    Convergence {
        /// Rounds of coordinate descent performed before giving up.
        iterations: u32,
    },

    /// The monotone curve problem has no feasible solution — the parameter
    /// bounds contradict the required ordering.
    #[error("curve is infeasible at `{name}`: bounds contradict monotone ordering")]
    InfeasibleCurve {
        /// The parameter whose bounds are unsatisfiable.
        name: String,
    },

    /// A solver was given an empty parameter set.
    #[error("no calibration parameters supplied")]
    NoParameters,
}

impl CalError {
    /// The characteristic, measurement, or parameter the error refers to,
    /// when the variant names one.
    #[must_use]
    pub fn subject(&self) -> Option<&str> {
        Some(match self {
            Self::LimitViolation { name, .. }
            | Self::OutOfBounds { name, .. }
            | Self::NonFiniteValue { name, .. }
            | Self::NoSignalBinding(name)
            | Self::UnknownCharacteristic(name)
            | Self::UnknownMeasurement(name)
            | Self::UnknownCompuMethod(name)
            | Self::UnknownComputationTable(name)
            | Self::UnsupportedConversion { name, .. }
            | Self::InfeasibleCurve { name }
            | Self::AmbiguousSignal { name, .. } => name.as_str(),
            Self::A2l(_)
            | Self::Dbc(_)
            | Self::Xcp(_)
            | Self::Io { .. }
            | Self::UnknownModule(_)
            | Self::Transport(_)
            | Self::Convergence { .. }
            | Self::NoParameters => return None,
        })
    }
}