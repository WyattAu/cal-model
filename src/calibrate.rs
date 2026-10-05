//! Calibration arithmetic: monotone curve fitting and bounded working-point
//! search.
//!
//! These are the two things a calibration engineer reaches for that are not
//! protocol: turning measured bench points into a curve the ECU can hold,
//! and finding the operating point that maximises a measured objective
//! inside declared bounds.
//!
//! # Determinism
//!
//! Both routines are **deterministic**: no random restarts, no
//! wall-clock-dependent caps, no parallelism. A calibration result an
//! engineer cannot reproduce is a result they cannot sign off, so the
//! iteration budget is a fixed constant and the same inputs always produce
//! the same curve.
//!
//! # The abscissa of a fitted curve
//!
//! A calibration curve's knots are indexed by the **order of the
//! parameters**, not by a measured input: the calibration engineer supplies
//! them in ascending operating-point order, which is the order they are
//! written into the deposit. [`Curve::xs`] is therefore `0..n`. A caller
//! that has real abscissae and wants them reported can zip them in.

use crate::error::CalError;

/// One calibration parameter: a value inside a declared box.
#[derive(Debug, Clone, PartialEq)]
pub struct CalParameter {
    /// Parameter name — the characteristic it came from.
    pub name: String,
    /// The measured or proposed value.
    pub value: f64,
    /// Lower bound, on the same scale as `value`.
    pub lower: f64,
    /// Upper bound, on the same scale as `value`.
    pub upper: f64,
}

impl CalParameter {
    /// A parameter with bounds.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a non-finite value or bound, and
    /// [`CalError::InfeasibleCurve`] when the bounds are the wrong way round
    /// (`lower > upper`).
    pub fn new(
        name: impl Into<String>,
        value: f64,
        lower: f64,
        upper: f64,
    ) -> Result<Self, CalError> {
        let parameter = Self {
            name: name.into(),
            value,
            lower,
            upper,
        };
        if parameter.lower > parameter.upper {
            return Err(CalError::InfeasibleCurve {
                name: parameter.name.clone(),
            });
        }
        Ok(parameter)
    }

    /// Check the value and bounds are usable.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a non-finite value or bound,
    /// [`CalError::InfeasibleCurve`] for inverted bounds, and
    /// [`CalError::OutOfBounds`] when `value` lies outside them.
    pub fn validate(&self) -> Result<(), CalError> {
        if !self.value.is_finite() || !self.lower.is_finite() || !self.upper.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: self.value,
            });
        }
        if self.lower > self.upper {
            return Err(CalError::InfeasibleCurve {
                name: self.name.clone(),
            });
        }
        if self.value < self.lower || self.value > self.upper {
            return Err(CalError::OutOfBounds {
                name: self.name.clone(),
                value: self.value,
                lower: self.lower,
                upper: self.upper,
            });
        }
        Ok(())
    }

    /// Clamp `candidate` into this parameter's bounds; a NaN maps to the
    /// parameter's own value, so a bad proposal cannot poison a sweep.
    #[must_use]
    pub fn clamp(&self, candidate: f64) -> f64 {
        if candidate.is_nan() {
            return self.value;
        }
        candidate.clamp(self.lower, self.upper)
    }

    /// The width of the parameter's box.
    #[must_use]
    pub fn span(&self) -> f64 {
        self.upper - self.lower
    }
}

/// A fitted monotone piecewise-linear curve: the knots and the value at each.
///
/// This is the shape a `CURVE` or `MAP` characteristic holds, so a [`Curve`]
/// can be written into a deposit once converted.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Curve {
    /// The knots, strictly ascending.
    pub xs: Vec<f64>,
    /// The value at each knot, non-decreasing.
    pub ys: Vec<f64>,
}

impl Curve {
    /// A curve through `points`, with the knots taken in the order given.
    ///
    /// # Errors
    ///
    /// As [`calibrate_curve`], applied to `points` in order.
    pub fn through(points: &[(f64, f64)]) -> Result<Self, CalError> {
        let parameters: Vec<CalParameter> = points
            .iter()
            .enumerate()
            .map(|(index, (x, y))| CalParameter {
                name: format!("knot{index}"),
                value: *y,
                lower: *x,
                upper: *x,
            })
            .collect();
        let mut curve = calibrate_curve(&parameters)?;
        curve.xs = points.iter().map(|(x, _)| *x).collect();
        Ok(curve)
    }

    /// Evaluate the curve at `x` by linear interpolation, clamping outside
    /// the knot range to the endpoint values.
    ///
    /// A curve with no knots evaluates to `0.0` and a single-knot curve is
    /// constant; neither can fail.
    #[must_use]
    pub fn evaluate(&self, x: f64) -> f64 {
        if self.xs.len() != self.ys.len() {
            return 0.0;
        }
        if self.xs.is_empty() {
            return 0.0;
        }
        if !x.is_finite() {
            return f64::NAN;
        }
        let Some(first_x) = self.xs.first().copied() else {
            return 0.0;
        };
        let first_y = self.ys.first().copied().unwrap_or(0.0);
        if self.xs.len() == 1 || x <= first_x {
            return first_y;
        }
        let Some(last_x) = self.xs.last().copied() else {
            return 0.0;
        };
        let last_y = self.ys.last().copied().unwrap_or(0.0);
        if x >= last_x {
            return last_y;
        }
        // `partition_point` gives the first knot strictly above `x`, so the
        // bracketing pair is (idx-1, idx).
        let upper_index = self.xs.partition_point(|knot| *knot <= x).min(self.xs.len() - 1);
        let Some(lower_index) = upper_index.checked_sub(1) else {
            return first_y;
        };
        let (Some(lower_x), Some(upper_x)) = (self.xs.get(lower_index), self.xs.get(upper_index))
        else {
            return first_y;
        };
        let (Some(lower_y), Some(upper_y)) = (self.ys.get(lower_index), self.ys.get(upper_index))
        else {
            return first_y;
        };
        if *upper_x == *lower_x {
            return *upper_y;
        }
        let weight = (x - lower_x) / (upper_x - lower_x);
        lower_y + weight * (upper_y - lower_y)
    }

    /// The largest absolute difference between the curve and each supplied
    /// point, for a fit-quality assertion.
    #[must_use]
    pub fn max_error_against(&self, xs: &[f64], ys: &[f64]) -> f64 {
        xs.iter()
            .zip(ys.iter())
            .map(|(x, y)| (self.evaluate(*x) - y).abs())
            .fold(0.0, f64::max)
    }

    /// `true` when the curve is non-decreasing across its knots.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        self.ys.windows(2).all(|w| w[1] >= w[0])
    }

    /// The knots as `(x, y)` pairs.
    #[must_use]
    pub fn points(&self) -> Vec<(f64, f64)> {
        self.xs
            .iter()
            .copied()
            .zip(self.ys.iter().copied())
            .collect()
    }
}

/// Fit a monotone piecewise-linear curve through the measured parameters.
///
/// The parameters are the curve's knots **in ascending operating-point
/// order** — the order they are deposited — so the fit is the piecewise
/// linear function through `(i, p[i].value)`.
///
/// Monotonicity is *checked*, not repaired. A controller handed a curve that
/// rises and then falls is ambiguous, so contradictory bench data is a
/// reportable defect: the engineer has two measurements that disagree and
/// needs to know, not a silently smoothed curve that hides the disagreement.
/// Each knot's declared bounds are validated too, because a knot outside its
/// own limits is equally contradictory.
///
/// # Errors
///
/// [`CalError::NoParameters`] for an empty set, [`CalError::NonFiniteValue`],
/// [`CalError::InfeasibleCurve`] when the values are **not** non-decreasing
/// (the contradictory case), and [`CalError::OutOfBounds`] when a value lies
/// outside its own declared bounds.
pub fn calibrate_curve(parameters: &[CalParameter]) -> Result<Curve, CalError> {
    if parameters.is_empty() {
        return Err(CalError::NoParameters);
    }
    for parameter in parameters {
        parameter.validate()?;
    }
    // Non-decreasing is the contract; the first pair that breaks it names
    // the knot, so the engineer knows which measurement to redo.
    for pair in parameters.windows(2) {
        let (Some(first), Some(second)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        if second.value < first.value {
            return Err(CalError::InfeasibleCurve {
                name: second.name.clone(),
            });
        }
    }
    Ok(Curve {
        xs: (0..parameters.len()).map(|index| index as f64).collect(),
        ys: parameters.iter().map(|parameter| parameter.value).collect(),
    })
}

/// Default iteration budget for [`optimize_working_point`].
pub const DEFAULT_MAX_ITERATIONS: u32 = 200;
/// Step multiplier after a productive sweep.
const STEP_GROW: f64 = 1.6;
/// Step multiplier after a sweep that found nothing.
const STEP_SHRINK: f64 = 0.5;
/// Step scale below which the search is declared settled.
const STEP_FLOOR: f64 = 1e-12;

/// Find the parameter setting that maximises `objective` inside the declared
/// bounds, by deterministic coordinate descent.
///
/// Each round sweeps every axis in order and slides it to the best of four
/// candidate steps — outward, inward, and two intermediate scales — keeping
/// the move when the objective improves. A sweep that improves nothing
/// halves the step and the round repeats, so the search refines rather than
/// restarts. It stops when the step scale falls below [`STEP_FLOOR`].
///
/// The result **never** leaves the box: every candidate is clamped to its
/// parameter's bounds before it is scored, so the returned point is inside
/// the bounds even when the objective is unbounded outside them.
///
/// # Errors
///
/// [`CalError::NoParameters`] for an empty set, [`CalError::NonFiniteValue`],
/// [`CalError::InfeasibleCurve`] for inverted bounds, [`CalError::OutOfBounds`]
/// for a value outside its bounds, and [`CalError::Convergence`] when the
/// iteration budget runs out before the step scale settles.
pub fn optimize_working_point(
    parameters: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<f64, CalError> {
    Ok(descend(parameters, objective, DEFAULT_MAX_ITERATIONS)?.1)
}

/// [`optimize_working_point`], returning both the optimal setting and the
/// objective value it attained.
///
/// The two are consistent by construction — one search, both results — so a
/// caller can never report a value from a different point.
///
/// # Errors
///
/// Exactly as [`optimize_working_point`].
pub fn optimal_point(
    parameters: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<(Vec<f64>, f64), CalError> {
    descend(parameters, objective, DEFAULT_MAX_ITERATIONS)
}

/// [`optimize_working_point`] with an explicit iteration budget.
///
/// Exposed so a caller can bound the work — a GUI sweep, or a test that
/// needs the non-convergence path.
///
/// # Errors
///
/// As [`optimize_working_point`], with `max_iterations` rounds attempted.
pub fn optimize_working_point_with_budget(
    parameters: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
    max_iterations: u32,
) -> Result<f64, CalError> {
    Ok(descend(parameters, objective, max_iterations)?.1)
}

/// The descent itself: returns `(best point, best objective value)`.
fn descend(
    parameters: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
    max_iterations: u32,
) -> Result<(Vec<f64>, f64), CalError> {
    if parameters.is_empty() {
        return Err(CalError::NoParameters);
    }
    for parameter in parameters {
        parameter.validate()?;
    }
    if max_iterations == 0 {
        return Err(CalError::Convergence { iterations: 0 });
    }

    let mut point: Vec<f64> = parameters.iter().map(|p| p.value).collect();
    let mut best = objective(&point);
    // A non-finite seed objective is reported as non-finite input rather
    // than silently starting from a sweep that can never improve.
    if !best.is_finite() {
        return Err(CalError::NonFiniteValue {
            name: "objective".to_owned(),
            value: best,
        });
    }
    let mut step = 1.0f64;

    for iteration in 1..=max_iterations {
        let mut improved = false;
        for (axis, parameter) in parameters.iter().enumerate() {
            let span = parameter.span();
            if !span.is_finite() || span == 0.0 {
                continue;
            }
            let current = point.get(axis).copied().unwrap_or(parameter.value);
            for fraction in [STEP_GROW, 1.0, 1.0 / STEP_GROW, STEP_SHRINK] {
                for sign in [1.0f64, -1.0] {
                    let mut candidate = point.clone();
                    if let Some(slot) = candidate.get_mut(axis) {
                        *slot = parameter.clamp(current + sign * fraction * step * span);
                    }
                    let Some(slotted) = candidate.get(axis).copied() else {
                        continue;
                    };
                    if slotted == current && fraction != 1.0 {
                        // Already clamped to this end of the axis; trying
                        // the mirror is the only thing left that can help.
                        continue;
                    }
                    let score = objective(&candidate);
                    if score.is_finite() && score > best {
                        best = score;
                        if let Some(slot) = point.get_mut(axis) {
                            *slot = slotted;
                        }
                        improved = true;
                        break;
                    }
                }
                if improved {
                    break;
                }
            }
        }
        if improved {
            step = (step * STEP_GROW).min(1.0);
        } else {
            step *= STEP_SHRINK;
            if step < STEP_FLOOR {
                return Ok((point, best));
            }
        }
        if iteration == max_iterations {
            return Err(CalError::Convergence {
                iterations: iteration,
            });
        }
    }
    Ok((point, best))
}
