//! The resolved conversion for one quantity: the A2L `COMPU_METHOD` plus,
//! for a `TABLE` method, the points it refers to.

use a2l_parse::{Coeffs, CompuMethod as A2lCompuMethod, ConversionType, LinearFn};

use crate::error::CalError;
use crate::tables::{CompuTabKind, CompuTable, TableSet};

/// A `COMPU_METHOD` resolved against the tables of its project.
///
/// This is a thin wrapper over [`a2l_parse::CompuMethod`] that adds the one
/// thing the substrate cannot supply: the `COMPU_TAB` points a `TABLE`
/// method needs in order to be evaluated.
#[derive(Debug, Clone, PartialEq)]
pub struct CompuMethod {
    /// The substrate's declaration, kept verbatim.
    pub method: A2lCompuMethod,
    /// The tabulated points, for a `TABLE` method. `None` otherwise, or when
    /// the referenced `COMPU_TAB` was not recovered from the description.
    table: Option<CompuTable>,
}

impl CompuMethod {
    /// Resolve `method`, attaching `tables`' points when it is a `TABLE`.
    #[must_use]
    pub fn new(method: A2lCompuMethod, tables: &TableSet) -> Self {
        let table = method
            .tab_ref
            .as_ref()
            .and_then(|name| tables.get(name).cloned());
        Self { method, table }
    }

    /// Attach (or replace) the tabulated points for a `TABLE` method.
    #[must_use]
    pub fn with_table(mut self, table: CompuTable) -> Self {
        self.table = Some(table);
        self
    }

    /// The conversion type keyword.
    #[must_use]
    pub const fn conversion_type(&self) -> ConversionType {
        self.method.conversion_type
    }

    /// The physical unit, possibly empty.
    #[must_use]
    pub fn unit(&self) -> &str {
        &self.method.unit
    }

    /// The tabulated points, for a `TABLE` method.
    #[must_use]
    pub fn table(&self) -> Option<&CompuTable> {
        self.table.as_ref()
    }

    /// Reduce to a linear raw → physical function, when the conversion is
    /// linear or reducibly rational.
    ///
    /// Delegates to the substrate, which owns the reduction rules. Note this
    /// applies to **raw counts**, not to physical units.
    #[must_use]
    pub fn to_linear(&self) -> Option<LinearFn> {
        self.method.to_linear()
    }

    /// `true` when this conversion has a single-valued inverse — i.e. it is
    /// monotone over its operating range. [`CompuMethod::to_raw`] needs this;
    /// a table that folds back on itself does not have a usable inverse.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        match self.method.conversion_type {
            ConversionType::Identity => true,
            ConversionType::Linear => {
                matches!(self.method.coeffs, Coeffs::Linear([slope, _]) if slope != 0.0)
            }
            ConversionType::RatFunc => self.rational_numerator_is_affine(),
            ConversionType::Table => self.table.as_ref().is_some_and(|table| {
                table.kind == CompuTabKind::Intp
                    && table.points.windows(2).all(|w| w[1].1 >= w[0].1)
            }),
        }
    }

    /// Convert a raw count to its physical value.
    ///
    /// Total: non-finite input yields a non-finite result rather than a
    /// panic, and the NaN propagates to the limit check, which is where it
    /// becomes a typed [`CalError::NonFiniteValue`].
    #[must_use]
    pub fn to_physical(&self, raw: f64) -> f64 {
        match self.method.conversion_type {
            ConversionType::Identity => raw,
            ConversionType::Linear => match self.method.coeffs {
                Coeffs::Linear([a, b]) => a * raw + b,
                _ => raw,
            },
            ConversionType::RatFunc => self.rational(raw),
            ConversionType::Table => self
                .table
                .as_ref()
                .map_or(raw, |table| table.evaluate(raw)),
        }
    }

    /// Convert a physical value back to an **unsigned** raw count.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a non-finite physical value, and
    /// [`CalError::UnsupportedConversion`] when the conversion is not
    /// single-valued invertible: a `TABLE` with no recovered points, a
    /// non-affine `RAT_FUNC`, a zero slope, or a discriminant below zero.
    pub fn to_raw(&self, physical: f64) -> Result<u64, CalError> {
        Ok(self.to_raw_i64(physical)?.unsigned_abs())
    }

    /// Convert a physical value back to a **signed** raw count.
    ///
    /// This is the form a signed deposit needs: an `SBYTE` holding −1 °C is
    /// the count −1, and [`Characteristic::signed_value`](crate::Characteristic::signed_value)
    /// re-encodes it into the memory bit pattern.
    ///
    /// # Errors
    ///
    /// As [`CompuMethod::to_raw`].
    pub fn to_raw_i64(&self, physical: f64) -> Result<i64, CalError> {
        if !physical.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.method.name.clone(),
                value: physical,
            });
        }
        let raw = match self.method.conversion_type {
            ConversionType::Identity => physical,
            ConversionType::Linear => match self.method.coeffs {
                Coeffs::Linear([slope, intercept]) if slope != 0.0 => (physical - intercept) / slope,
                _ => {
                    return Err(CalError::UnsupportedConversion {
                        name: self.method.name.clone(),
                        detail: "a zero-slope LINEAR conversion has no inverse",
                    })
                }
            },
            ConversionType::RatFunc => self.invert_rational(physical)?,
            ConversionType::Table => {
                let Some(table) = self.table.as_ref() else {
                    return Err(CalError::UnknownComputationTable(
                        self.method.tab_ref.clone().unwrap_or_default(),
                    ));
                };
                match table.kind {
                    CompuTabKind::Verb => table.invert_verb(physical),
                    CompuTabKind::Intp => table.invert_intp(physical),
                }
                .ok_or(CalError::UnsupportedConversion {
                    name: self.method.name.clone(),
                    detail: "the tabulated points do not invert to a single input",
                })?
            }
        };
        if !raw.is_finite() {
            return Err(CalError::UnsupportedConversion {
                name: self.method.name.clone(),
                detail: "the inverse is not finite over the reals",
            });
        }
        Ok(round_half_away(raw))
    }

    /// `f(x) = (a·x² + b·x + c) / (d·x² + e·x + f)` from
    /// `COEFFS a b c d e f`.
    fn rational(&self, raw: f64) -> f64 {
        match self.method.coeffs {
            Coeffs::RatFunc([a, b, c, d, e, f]) => {
                let numerator = (a * raw + b) * raw + c;
                let denominator = (d * raw + e) * raw + f;
                if denominator == 0.0 {
                    f64::NAN
                } else {
                    numerator / denominator
                }
            }
            _ => raw,
        }
    }

    /// Invert `(a·x² + b·x + c) / (d·x² + e·x + f) = y` for `x`.
    ///
    /// Rearranges to `A·x² + B·x + C = 0` with `A = a - y·d`,
    /// `B = b - y·e`, `C = c - y·f`, solved by the quadratic formula.
    ///
    /// The degenerate linear case (`A == 0`) is handled directly, which is
    /// what makes the reducible `(b·x + c) / f` family invert **exactly** —
    /// the non-zero-intercept case a scale-only shortcut silently gets wrong.
    fn invert_rational(&self, physical: f64) -> Result<f64, CalError> {
        let Coeffs::RatFunc([a, b, c, d, e, f]) = self.method.coeffs else {
            return Err(CalError::UnsupportedConversion {
                name: self.method.name.clone(),
                detail: "a RAT_FUNC method must declare COEFFS a b c d e f",
            });
        };
        let quad_a = a - physical * d;
        let quad_b = b - physical * e;
        let quad_c = c - physical * f;
        if quad_a == 0.0 {
            if quad_b == 0.0 {
                return Err(CalError::UnsupportedConversion {
                    name: self.method.name.clone(),
                    detail: "the rational function is constant, so it has no inverse",
                });
            }
            return Ok(-quad_c / quad_b);
        }
        let discriminant = quad_b * quad_b - 4.0 * quad_a * quad_c;
        if discriminant < 0.0 {
            return Err(CalError::UnsupportedConversion {
                name: self.method.name.clone(),
                detail: "the physical value lies outside the rational function's range",
            });
        }
        let root = discriminant.sqrt();
        // Numerically stable quadratic formula: pick the sign that makes the
        // numerator large, so cancellation does not eat the smaller root.
        let q = -0.5 * (quad_b + if quad_b >= 0.0 { root } else { -root });
        let primary = q / quad_a;
        let secondary = if q != 0.0 { quad_c / q } else { -quad_b / quad_a };
        // Take the smaller real root: for the monotone rational forms real
        // ECUs use, that is the branch inside the operating range.
        match (primary.is_finite(), secondary.is_finite()) {
            (true, true) => Ok(primary.min(secondary)),
            (true, false) => Ok(primary),
            (false, true) => Ok(secondary),
            (false, false) => Err(CalError::UnsupportedConversion {
                name: self.method.name.clone(),
                detail: "the quadratic inverse has no finite root",
            }),
        }
    }

    /// `true` when the rational numerator is at most linear (`a == 0`), the
    /// reducible form the substrate also recognises.
    fn rational_numerator_is_affine(&self) -> bool {
        matches!(self.method.coeffs, Coeffs::RatFunc([0.0, ..]))
    }
}

/// Round half away from zero, matching `dbc-parse`'s `encode_raw` rule.
///
/// `f64::round` rounds half *to even*, which would make a physical value of
/// exactly 0.5 quantise differently here than it does in the DBC encoder.
#[must_use]
pub fn round_half_away(value: f64) -> i64 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    if value >= 0.0 {
        let floor = value.floor();
        let rounded = if value - floor >= 0.5 { floor + 1.0 } else { floor };
        rounded as i64
    } else {
        let ceiling = value.ceil();
        let rounded = if ceiling - value >= 0.5 { ceiling - 1.0 } else { ceiling };
        rounded as i64
    }
}
