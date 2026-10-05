//! COMPU_METHOD evaluation — the raw ↔ physical conversion of one quantity.
//!
//! `a2l_parse` (L1) hands back the *declaration* of a conversion: its type,
//! its coefficients or its table reference. This module turns that
//! declaration into arithmetic a calibration session can run —
//! [`CompuMethod::to_physical`] and [`CompuMethod::to_raw`] evaluate
//! `LINEAR`, `RAT_FUNC`, `TABLE`, and `IDENTITY` in both directions.
//!
//! # Inversion
//!
//! Going physical → raw is the interesting direction, and each conversion
//! type inverts in its own way:
//!
//! * `IDENTITY` — the value is its own raw.
//! * `LINEAR` — one division by the slope.
//! * `RAT_FUNC` — the inverse of `(a·x² + b·x + c) / (d·x² + e·x + f)` is
//!   the root of `(a − p·d)·x² + (b − p·e)·x + (c − p·f) = 0`, solved as a
//!   quadratic (degenerating to a linear solve when the leading term
//!   vanishes). A rational function can be two-to-one over the reals, so
//!   both roots are tested and the one that maps back to `physical` most
//!   closely wins; ties go to the smaller root, which keeps the result
//!   deterministic.
//! * `TABLE` — inversion within the bracketing segment (interpolating) or
//!   to the tabulated raw that produces the value (stepping).
//!
//! Every inversion is total: a conversion that cannot produce a finite,
//! non-negative raw resolves to [`CalError::UnsupportedConversion`] or
//! [`CalError::OutOfBounds`] rather than a panic or a silent `0`.

use a2l_parse::{Coeffs, ConversionType, LinearFn};

use crate::error::CalError;
use crate::table::CompuTable;

/// Relative tolerance below which a leading quadratic coefficient is
/// treated as zero, and an absolute tolerance for "the answer is zero".
const EPSILON: f64 = 1e-12;

/// A resolved COMPU_METHOD: a conversion declaration plus, for tabular
/// methods, the tabulated points its `COMPU_TAB_REF` names.
#[derive(Debug, Clone, PartialEq)]
pub struct CompuMethod {
    name: String,
    kind: ConversionType,
    coeffs: Coeffs,
    unit: String,
    table: Option<CompuTable>,
}

impl CompuMethod {
    /// Resolve a parsed COMPU_METHOD, attaching the table its
    /// `COMPU_TAB_REF` names (when the caller could find one).
    #[must_use]
    pub fn new(declaration: &a2l_parse::CompuMethod, table: Option<CompuTable>) -> Self {
        Self {
            name: declaration.name.clone(),
            kind: declaration.conversion_type,
            coeffs: declaration.coeffs,
            unit: declaration.unit.clone(),
            table,
        }
    }

    /// The COMPU_METHOD name (what a CHARACTERISTIC references).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The declared conversion type.
    #[must_use]
    pub const fn kind(&self) -> ConversionType {
        self.kind
    }

    /// The physical unit string (may be empty).
    #[must_use]
    pub fn unit(&self) -> &str {
        &self.unit
    }

    /// The tabulated points, for `TABLE` methods.
    #[must_use]
    pub const fn table(&self) -> Option<&CompuTable> {
        self.table.as_ref()
    }

    /// The declared coefficient form.
    #[must_use]
    pub const fn coefficients(&self) -> &Coeffs {
        &self.coeffs
    }

    /// Reduce the method to a linear raw → physical function when the
    /// conversion is linear-or-reducibly-rational — delegated to the L1
    /// substrate's own reduction, so the two agree exactly.
    #[must_use]
    pub fn linear(&self) -> Option<LinearFn> {
        let declaration = a2l_parse::CompuMethod {
            name: self.name.clone(),
            conversion_type: self.kind,
            coeffs: self.coeffs,
            tab_ref: None,
            unit: self.unit.clone(),
        };
        declaration.to_linear()
    }

    /// The physical value of a raw deposit.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a non-finite raw,
    /// [`CalError::UnsupportedConversion`] when the declaration cannot be
    /// evaluated (a `TABLE` method with no usable table, or a `RAT_FUNC`
    /// whose denominator vanishes at this raw value),
    /// [`CalError::UnknownComputationTable`] when the referenced table is
    /// missing or empty.
    pub fn to_physical(&self, raw: f64) -> Result<f64, CalError> {
        if !raw.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: raw,
            });
        }
        match self.kind {
            ConversionType::Identity => Ok(raw),
            ConversionType::Linear => match self.coeffs {
                Coeffs::Linear([a, b]) => Ok(a * raw + b),
                _ => Err(self.unsupported("no COEFFS_LINEAR pair")),
            },
            ConversionType::RatFunc => match self.coeffs {
                Coeffs::RatFunc([a, b, c, d, e, f]) => {
                    let numerator = (a * raw + b) * raw + c;
                    let denominator = (d * raw + e) * raw + f;
                    if denominator == 0.0 {
                        return Err(self.unsupported("the denominator vanishes at this raw value"));
                    }
                    Ok(numerator / denominator)
                }
                _ => Err(self.unsupported("no COEFFS sextuple")),
            },
            ConversionType::Table => match &self.table {
                Some(table) => table.to_physical(raw),
                None => Err(CalError::UnknownComputationTable(self.name.clone())),
            },
        }
    }

    /// The raw deposit for a physical value.
    ///
    /// The result is an unsigned integer (an ECU deposit is integral);
    /// fractions round half away from zero. The *representable* span is not
    /// consulted here — the deposit datatype belongs to the RECORD_LAYOUT,
    /// so the width check belongs to
    /// [`CalibrationProject::to_raw`](crate::CalibrationProject::to_raw),
    /// which reports the physical span that actually failed.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a non-finite value,
    /// [`CalError::UnsupportedConversion`] when the conversion has no
    /// finite non-negative inverse (a constant or single-valued method, a
    /// `RAT_FUNC` with a vanishing denominator, or a quadratic with no
    /// real root at this physical value),
    /// [`CalError::OutOfBounds`] when the inverse is negative,
    /// [`CalError::UnknownComputationTable`] when the referenced table is
    /// missing or empty.
    pub fn to_raw(&self, physical: f64) -> Result<u64, CalError> {
        if !physical.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: physical,
            });
        }
        match self.kind {
            ConversionType::Identity => self.integral(physical),
            ConversionType::Linear => match self.coeffs {
                Coeffs::Linear([a, b]) => {
                    if a == 0.0 {
                        return Err(self.unsupported("a constant conversion has no inverse"));
                    }
                    self.integral((physical - b) / a)
                }
                _ => Err(self.unsupported("no COEFFS_LINEAR pair")),
            },
            ConversionType::RatFunc => match self.coeffs {
                Coeffs::RatFunc([a, b, c, d, e, f]) => self.invert_rational([a, b, c, d, e, f], physical),
                _ => Err(self.unsupported("no COEFFS sextuple")),
            },
            ConversionType::Table => match &self.table {
                Some(table) => table.to_raw(physical),
                None => Err(CalError::UnknownComputationTable(self.name.clone())),
            },
        }
    }

    /// Invert `f(x) = (a·x² + b·x + c) / (d·x² + e·x + f)` at `physical`.
    fn invert_rational(&self, c: [f64; 6], physical: f64) -> Result<u64, CalError> {
        let [a, b, c0, d, e, f] = c;
        // (a − p·d)·x² + (b − p·e)·x + (c − p·f) = 0.
        let lead = a - physical * d;
        let mid = b - physical * e;
        let constant = c0 - physical * f;
        // Scale for the "is this coefficient zero" test, so a huge physical
        // value does not make a genuinely-zero term look significant.
        let scale = lead.abs() + mid.abs() + constant.abs() + 1.0;
        let zero = EPSILON * scale;

        let candidates: Vec<f64> = if lead.abs() <= zero {
            if mid.abs() <= zero {
                if constant.abs() <= zero {
                    // The identity holds everywhere: any raw works, and 0 is
                    // the canonical choice.
                    vec![0.0]
                } else {
                    return Err(self.unsupported("the equation has no solution"));
                }
            } else {
                vec![-constant / mid]
            }
        } else {
            let discriminant = mid * mid - 4.0 * lead * constant;
            if discriminant < 0.0 {
                return Err(self.unsupported("the equation has no real root"));
            }
            let root = discriminant.sqrt();
            vec![(-mid + root) / (2.0 * lead), (-mid - root) / (2.0 * lead)]
        };

        // Keep the non-negative finite candidates and pick the one whose
        // forward value is closest to the requested physical value; ties go
        // to the smaller root, which keeps inversion deterministic.
        let mut best: Option<(f64, f64)> = None;
        for candidate in candidates {
            if !candidate.is_finite() || candidate < 0.0 {
                continue;
            }
            let forward = ((a * candidate + b) * candidate + c0) / ((d * candidate + e) * candidate + f);
            if !forward.is_finite() {
                continue;
            }
            let error = (forward - physical).abs();
            match best {
                Some((best_error, best_root))
                    if best_error < error || (best_error == error && candidate >= best_root) => {}
                _ => best = Some((error, candidate)),
            }
        }
        let (_, raw) = best.ok_or_else(|| {
            self.unsupported("no non-negative finite root at this physical value")
        })?;
        self.integral(raw)
    }

    /// Round a real-valued inverse to an integral deposit, rejecting
    /// anything negative.
    fn integral(&self, raw: f64) -> Result<u64, CalError> {
        if !raw.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: raw,
            });
        }
        let rounded = raw.round();
        if rounded < 0.0 || rounded > 18_446_744_073_709_551_616.0 {
            return Err(CalError::OutOfBounds {
                name: self.name.clone(),
                value: raw,
                lower: 0.0,
                upper: 18_446_744_073_709_551_616.0,
            });
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Ok(rounded as u64)
    }

    /// The unsupported-conversion error for this method.
    fn unsupported(&self, detail: &'static str) -> CalError {
        CalError::UnsupportedConversion {
            name: self.name.clone(),
            detail,
        }
    }
}