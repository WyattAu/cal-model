//! The COMPU_METHOD conversion model: raw ↔ physical, in both directions.
//!
//! An A2L `COMPU_METHOD` is a pure function of the raw value stored in ECU
//! memory. `a2l_parse` retains the *declaration* (conversion type,
//! coefficients, `COMPU_TAB_REF`); this module turns that declaration into
//! an evaluator ([`CompuMethod::apply`]) and — the part every calibration
//! tool actually needs — an inverter ([`CompuMethod::invert`]).
//!
//! # Inversion is where the interesting failures live
//!
//! `to_raw` is not simply "undo the arithmetic". A physical value is only
//! writable if the raw value that produces it exists inside the deposit's
//! datatype range, so [`CompuMethod::invert`] takes the raw bounds
//! (`value_min`, `value_max`) and reports [`CalError::OutOfBounds`] when the
//! target is not bracketed by them. That is the check that stops a
//! calibration engineer from writing a value the ECU would latch as
//! something else entirely.
//!
//! # TABLE conversions
//!
//! `a2l_parse` retains the `COMPU_TAB_REF` *name* but not the `COMPU_TAB`
//! points (the ASAP2 format stores them in a separate block, which the L1
//! parser skips as an unknown sub-block). [`CompuMethod::Table`] therefore
//! carries an empty point list until the points are registered on the
//! project ([`CalibrationProject::register_table`](crate::CalibrationProject::register_table))
//! or set directly. [`CompuMethod::apply`] stays **total** on an empty
//! table (it falls back to the identity function); it is
//! [`CalibrationProject`](crate::CalibrationProject) — the layer that knows
//! the project — that refuses to resolve a table with no points, so an
//! unresolvable conversion is a typed error rather than a silently wrong
//! number.

use crate::CalError;
use std::collections::BTreeMap;

/// Bisection steps used to invert a rational conversion. 128 halvings of a
/// 32-bit raw range land far below the 1e-9 tolerance the rest of the stack
/// works to.
const BISECTION_STEPS: usize = 128;

/// A raw ↔ physical conversion, driven by an A2L `COMPU_METHOD`.
#[derive(Debug, Clone, PartialEq)]
pub enum CompuMethod {
    /// `IDENTITY` (also spelled `IDENTICAL`): `f(x) = x`.
    Identity,
    /// `LINEAR` — `f(x) = slope · x + intercept`.
    Linear {
        /// Multiplicative term (`COEFFS_LINEAR` `a`).
        slope: f64,
        /// Additive term (`COEFFS_LINEAR` `b`).
        intercept: f64,
    },
    /// `RAT_FUNC` — `f(x) = (a·x² + b·x + c) / (d·x² + e·x + f)`, the six
    /// `COEFFS` in the spec's own notation. Reducible-linear forms (the
    /// common case, `d = e = a = 0`) are evaluated as written, which is
    /// both faster and free of any reduction's rounding.
    RatFunc {
        /// `COEFFS a b c d e f`.
        coeffs: [f64; 6],
    },
    /// `TABLE` — a conversion through tabulated points. `points` are
    /// `(raw, physical)` pairs ascending by raw; `interpolate` is `true` for
    /// `TAB_INTP` (piecewise-linear, the ASAP2 default) and `false` for
    /// `TAB_NOINTP`/`TAB_VERB` (step lookup, nearest point wins).
    Table {
        /// The `COMPU_TAB_REF` name, when the declaration carried one.
        tab_ref: Option<String>,
        /// `(raw, physical)` points, ascending by raw.
        points: Vec<(f64, f64)>,
        /// `TAB_INTP` interpolation (`true`) or step lookup (`false`).
        interpolate: bool,
    },
}

impl CompuMethod {
    /// `IDENTITY` — the passthrough conversion.
    #[must_use]
    pub const fn identity() -> Self {
        Self::Identity
    }

    /// `LINEAR` — `f(x) = slope · x + intercept`.
    #[must_use]
    pub const fn linear(slope: f64, intercept: f64) -> Self {
        Self::Linear { slope, intercept }
    }

    /// `RAT_FUNC` — `f(x) = (a·x² + b·x + c) / (d·x² + e·x + f)`.
    #[must_use]
    pub const fn rat_func(coeffs: [f64; 6]) -> Self {
        Self::RatFunc { coeffs }
    }

    /// `TAB_INTP` — an interpolated table conversion.
    #[must_use]
    pub fn tab_intp(points: Vec<(f64, f64)>) -> Self {
        Self::Table {
            tab_ref: None,
            points,
            interpolate: true,
        }
    }

    /// `TAB_NOINTP`/`TAB_VERB` — a stepped table conversion.
    #[must_use]
    pub fn tab_no_intp(points: Vec<(f64, f64)>) -> Self {
        Self::Table {
            tab_ref: None,
            points,
            interpolate: false,
        }
    }

    /// Build the conversion carried by an [`a2l_parse::CompuMethod`].
    ///
    /// A `TABLE` declaration arrives with its `COMPU_TAB_REF` name and an
    /// empty point list; the points are registered on the project.
    #[must_use]
    pub fn from_a2l(method: &a2l_parse::CompuMethod) -> Self {
        match method.conversion_type {
            a2l_parse::ConversionType::Identity => Self::Identity,
            a2l_parse::ConversionType::Linear => match method.coeffs {
                a2l_parse::Coeffs::Linear([slope, intercept]) => Self::Linear { slope, intercept },
                // A LINEAR method without COEFFS_LINEAR: the format requires
                // the keyword, but a sloppy generator omits it. a2l_parse
                // models that as Coeffs::None; identity is the only reading
                // that cannot invent a wrong scale.
                _ => Self::Identity,
            },
            a2l_parse::ConversionType::RatFunc => match method.coeffs {
                a2l_parse::Coeffs::RatFunc(coeffs) => Self::RatFunc { coeffs },
                _ => Self::Identity,
            },
            a2l_parse::ConversionType::Table => Self::Table {
                tab_ref: method.tab_ref.clone(),
                points: Vec::new(),
                interpolate: true,
            },
        }
    }

    /// The A2L conversion-type keyword this method came from
    /// (`LINEAR`, `RAT_FUNC`, `TABLE`, `IDENTITY`).
    #[must_use]
    pub const fn keyword(&self) -> &'static str {
        match self {
            Self::Identity => "IDENTITY",
            Self::Linear { .. } => "LINEAR",
            Self::RatFunc { .. } => "RAT_FUNC",
            Self::Table { .. } => "TABLE",
        }
    }

    /// The tabulated points, empty for every non-`TABLE` method.
    #[must_use]
    pub fn points(&self) -> &[(f64, f64)] {
        match self {
            Self::Table { points, .. } => points,
            _ => &[],
        }
    }

    /// The `COMPU_TAB_REF` name, when the declaration carried one.
    #[must_use]
    pub fn tab_ref(&self) -> Option<&str> {
        match self {
            Self::Table { tab_ref, .. } => tab_ref.as_deref(),
            _ => None,
        }
    }

    /// `true` for the passthrough conversion.
    #[must_use]
    pub const fn is_identity(&self) -> bool {
        matches!(self, Self::Identity)
    }

    /// Replace the point list of a `TABLE` method (a no-op otherwise).
    pub fn set_points(&mut self, points: Vec<(f64, f64)>) {
        if let Self::Table { points: slot, .. } = self {
            *slot = points;
        }
    }

    /// Fill in the points of a `TABLE` method from the project's table
    /// registry when it carries none of its own.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when a `TABLE` conversion cannot be
    /// resolved: no points of its own and no registered `COMPU_TAB_REF`.
    pub fn resolved(&self, tables: &BTreeMap<String, Vec<(f64, f64)>>) -> Result<Self, CalError> {
        let Self::Table {
            tab_ref,
            points,
            interpolate,
        } = self
        else {
            return Ok(self.clone());
        };
        if !points.is_empty() {
            return Ok(self.clone());
        }
        let Some(name) = tab_ref else {
            return Err(CalError::Unsupported {
                subject: "TABLE conversion".to_string(),
                reason: "no points and no COMPU_TAB_REF to resolve them from".to_string(),
            });
        };
        let Some(found) = tables.get(name.as_str()) else {
            return Err(CalError::Unsupported {
                subject: format!("COMPU_TAB_REF `{name}`"),
                reason: "no table registered under that name".to_string(),
            });
        };
        if found.is_empty() {
            return Err(CalError::Unsupported {
                subject: format!("COMPU_TAB_REF `{name}`"),
                reason: "the registered table has no points".to_string(),
            });
        }
        Ok(Self::Table {
            tab_ref: tab_ref.clone(),
            points: found.clone(),
            interpolate: *interpolate,
        })
    }

    /// Evaluate the conversion: raw value in, physical value out.
    ///
    /// Total — no input panics and every variant is a plain evaluation. A
    /// `TABLE` with no points evaluates as the identity; use
    /// [`CompuMethod::resolved`] first when completeness matters.
    #[must_use]
    pub fn apply(&self, raw: f64) -> f64 {
        match self {
            Self::Identity => raw,
            Self::Linear { slope, intercept } => slope * raw + intercept,
            Self::RatFunc { coeffs } => {
                let [a, b, c, d, e, f] = *coeffs;
                // f64 division by zero is infinity/NaN, never a panic: a
                // degenerate denominator yields a non-finite physical value,
                // which every caller rejects through its limit check.
                let x = raw;
                (a * x * x + b * x + c) / (d * x * x + e * x + f)
            }
            Self::Table {
                points,
                interpolate,
                ..
            } => table_apply(points, *interpolate, raw),
        }
    }

    /// Invert the conversion: physical value in, raw value out, bracketed
    /// by the deposit's raw bounds `value_min`/`value_max` (in physical
    /// units, after sign extension).
    ///
    /// * `IDENTITY` and `LINEAR` are closed-form (a zero slope is not
    ///   invertible and is reported as [`CalError::Unsupported`]).
    /// * `RAT_FUNC` is inverted by bisection over `[value_min, value_max]`,
    ///   which needs no derivative and is exact to well under one raw count
    ///   across the whole 64-bit range.
    /// * `TABLE` inverts segment-wise, linearly for `TAB_INTP` and by
    ///   nearest point for a stepped table.
    ///
    /// `name` is the characteristic the inversion is for; it appears in the
    /// error so a session log says which parameter failed.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when the conversion is not invertible (zero
    /// slope, or a `TABLE` with no points), [`CalError::OutOfBounds`] when
    /// the physical value is not produced anywhere inside
    /// `[value_min, value_max]`.
    pub fn invert(
        &self,
        name: &str,
        physical: f64,
        value_min: f64,
        value_max: f64,
    ) -> Result<f64, CalError> {
        match self {
            Self::Identity => in_range(name, physical, value_min, value_max),
            Self::Linear { slope, intercept } => {
                if *slope == 0.0 || !slope.is_finite() {
                    return Err(CalError::Unsupported {
                        subject: format!("`{name}`"),
                        reason: format!("linear conversion with slope {slope} is not invertible"),
                    });
                }
                in_range(name, (physical - intercept) / slope, value_min, value_max)
            }
            Self::RatFunc { .. } => bisect(self, name, physical, value_min, value_max),
            Self::Table {
                points,
                interpolate,
                ..
            } => table_invert(points, *interpolate, name, physical, value_min, value_max),
        }
    }
}

/// Range-check a computed raw value against the deposit bounds.
fn in_range(name: &str, raw: f64, value_min: f64, value_max: f64) -> Result<f64, CalError> {
    if !raw.is_finite() || raw < value_min || raw > value_max {
        return Err(CalError::OutOfBounds {
            name: name.to_string(),
            value: raw,
            lower: value_min,
            upper: value_max,
        });
    }
    Ok(raw)
}

/// Bisection inversion of a conversion that is monotone on the bracket.
fn bisect(
    method: &CompuMethod,
    name: &str,
    physical: f64,
    value_min: f64,
    value_max: f64,
) -> Result<f64, CalError> {
    let mut lo = value_min;
    let mut hi = value_max;
    let y_lo = method.apply(lo);
    let y_hi = method.apply(hi);
    if !y_lo.is_finite() || !y_hi.is_finite() {
        return Err(CalError::Unsupported {
            subject: format!("`{name}`"),
            reason: "rational conversion is not finite at the deposit bounds".to_string(),
        });
    }
    // Bisection needs the target *between* the endpoint values, whichever
    // way the conversion runs. A descending conversion is searched by
    // comparing against the reversed endpoint, not by swapping the bracket:
    // the raw axis stays ascending throughout.
    let descending = y_lo > y_hi;
    let low_physical = if descending { y_hi } else { y_lo };
    let high_physical = if descending { y_lo } else { y_hi };
    if physical < low_physical || physical > high_physical {
        return Err(CalError::OutOfBounds {
            name: name.to_string(),
            value: physical,
            lower: low_physical,
            upper: high_physical,
        });
    }
    for _ in 0..BISECTION_STEPS {
        let mid = lo.midpoint(hi);
        if hi - lo <= 1e-9 * mid.abs().max(1.0) {
            break;
        }
        let below = if descending {
            method.apply(mid) > physical
        } else {
            method.apply(mid) < physical
        };
        if below {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(lo.midpoint(hi))
}

/// Piecewise evaluation of a tabulated conversion; identity when empty.
fn table_apply(points: &[(f64, f64)], interpolate: bool, raw: f64) -> f64 {
    let Some(&(first_x, first_y)) = points.first() else {
        return raw;
    };
    if raw <= first_x {
        return first_y;
    }
    let Some(&(last_x, last_y)) = points.last() else {
        return raw;
    };
    if raw >= last_x {
        return last_y;
    }
    // Points are ascending by raw (validated on registration), so the
    // bracketing segment is found by a single linear scan. The upper end is
    // half-open so a raw value sitting exactly on a breakpoint takes the
    // *later* point — which is what a step lookup means, and what makes
    // interpolation continuous across the breakpoint.
    for pair in points.windows(2) {
        let [first, second] = pair else { continue };
        let (x0, y0) = *first;
        let (x1, y1) = *second;
        if raw >= x0 && raw < x1 {
            if !interpolate {
                // `TAB_NOINTP` holds the segment's *lower* tabulated value
                // across the whole segment, switching to the next point at
                // its own breakpoint — the step the name describes.
                return y0;
            }
            let span = x1 - x0;
            if span == 0.0 {
                return y0;
            }
            return y0 + (y1 - y0) * (raw - x0) / span;
        }
    }
    first_y
}

/// Segment-wise inversion of a tabulated conversion.
fn table_invert(
    points: &[(f64, f64)],
    interpolate: bool,
    name: &str,
    physical: f64,
    value_min: f64,
    value_max: f64,
) -> Result<f64, CalError> {
    if points.is_empty() {
        return Err(CalError::Unsupported {
            subject: format!("`{name}`"),
            reason: "TABLE conversion has no points to invert".to_string(),
        });
    }
    let lowest = points
        .iter()
        .map(|&(_, phys)| phys)
        .fold(f64::INFINITY, f64::min);
    let highest = points
        .iter()
        .map(|&(_, phys)| phys)
        .fold(f64::NEG_INFINITY, f64::max);
    if !physical.is_finite() || physical < lowest || physical > highest {
        return Err(CalError::OutOfBounds {
            name: name.to_string(),
            value: physical,
            lower: lowest,
            upper: highest,
        });
    }
    if !interpolate {
        // The inverse of a step function is the step's own lower breakpoint:
        // the largest tabulated point whose value is still at or below the
        // target (the mirror of `table_apply`'s `y0`). `points` is non-empty
        // here — the early return above guarantees it.
        let Some(&first) = points.first() else {
            return Err(CalError::Unsupported {
                subject: format!("`{name}`"),
                reason: "TABLE conversion has no points to invert".to_string(),
            });
        };
        let mut best = first;
        for &point in points {
            if point.1 <= physical {
                best = point;
            }
        }
        if physical < best.1 {
            // Below the first step: clamp to the domain's first point.
            best = first;
        }
        return in_range(name, best.0, value_min, value_max);
    }
    // Ascending tables invert segment-wise; a descending or non-monotone
    // table still resolves, because every bracketing pair is examined and
    // the one whose round trip is most accurate wins.
    let mut best_raw: Option<f64> = None;
    let mut best_error = f64::INFINITY;
    for pair in points.windows(2) {
        let [first, second] = pair else { continue };
        let (x0, y0) = *first;
        let (x1, y1) = *second;
        let (low_y, high_y) = if y0 <= y1 { (y0, y1) } else { (y1, y0) };
        if physical < low_y || physical > high_y {
            continue;
        }
        let span = y1 - y0;
        let raw = if span == 0.0 {
            x0
        } else {
            x0 + (physical - y0) * (x1 - x0) / span
        };
        let error = (table_apply(points, interpolate, raw) - physical).abs();
        if error < best_error {
            best_error = error;
            best_raw = Some(raw);
        }
    }
    let Some(raw) = best_raw else {
        return Err(CalError::OutOfBounds {
            name: name.to_string(),
            value: physical,
            lower: lowest,
            upper: highest,
        });
    };
    in_range(name, raw, value_min, value_max)
}
